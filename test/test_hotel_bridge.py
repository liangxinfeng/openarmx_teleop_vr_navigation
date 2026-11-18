"""Exercise the production hotel bridge without ROS imports or robot services."""
import ast
import asyncio
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from uuid import uuid4

from aiohttp import web

from openarmx_teleop_vr_navigation_306.hotel_service import CameraEvidence, HotelMissionService
from openarmx_teleop_vr_navigation_306.task_workflow import TaskWorkflow, navigation_terminal_outcome
from openarmx_teleop_vr_navigation_306.simple_task_catalog import load_catalog


ROOT = Path(__file__).parents[1]
source = ROOT / 'openarmx_teleop_vr_navigation_306/hotel_bridge.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
scope = dict(asyncio=asyncio, ThreadPoolExecutor=ThreadPoolExecutor, partial=partial,
             math=math, Path=Path, threading=threading, web=web,
             Trigger=NS(Request=lambda: NS()), CameraEvidence=CameraEvidence,
             HotelMissionService=HotelMissionService, navigation_terminal_outcome=navigation_terminal_outcome,
             get_package_share_directory=lambda _: str(ROOT))
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef)], type_ignores=[]), str(source), 'exec'), scope)


class Harness(scope['HotelBridgeMixin']):
    def __init__(self, tmp_path):
        config = json.loads((ROOT / 'config/hotel_missions.json').read_text(encoding='utf-8'))
        config['record_root'] = str(tmp_path / 'records')
        self.config_path = tmp_path / 'hotel.json'
        self._task_catalog_path = ROOT / 'config/task_actions.txt'
        self.config_path.write_text(json.dumps(config, ensure_ascii=False), encoding='utf-8')
        self.parameters = {}
        self._rgbd_camera = None
        self._nav_lock = threading.Lock()
        self._drive_lock = threading.Lock()
        self._drive_sample = None
        self._pose = {'x': 0, 'y': 0}
        self._task_height_target_m = None
        self._exit_in_progress = False
        self._waypoints = ['前台','桌子1','桌子2','桌子3','桌子4','垃圾桶','充电桩']
        self._task_workflow = TaskWorkflow(load_catalog(self._task_catalog_path))
        self._cancel_navigation_client = object()
        self.calls = []
        self.status = {'active_arms': [], 'backend': {'hardware_ready': True, 'head_follow_enabled': False}}
        self.clock_ns = 100_000_000_000
        self._hotel_initialize()
        self._hotel.capture_photo = lambda *_: b'jpeg'
        self._hotel_register_routes(web.Application())

    def declare_parameter(self, name, default):
        self.parameters[name] = str(self.config_path) if name == 'hotel_mission_config_path' else default

    def get_parameter(self, name): return NS(value=self.parameters[name])
    def _teleop_status_snapshot(self): return self.status, 0.01, True
    def _navigation_is_active(self): return False
    def get_clock(self): return NS(now=lambda: NS(nanoseconds=self.clock_ns))
    def get_logger(self): return NS(error=lambda message: self.calls.append(('error', message)))

    async def _operator_payload(self, request):
        if request.get('client_id') != 'headset': raise RuntimeError('请由当前接管的头显操作')
        return request

    async def _call_service(self, client, request):
        self.calls.append(('cancel', None))
        return True, 'ok'

    async def _start_current_task_navigation(self):
        self._task_workflow.navigation_started()
        self.calls.append(('navigate', self._task_workflow.current()['location']))

    def _release_task_height_for_manual_control(self): self._task_workflow.height_adjusted()

    async def start(self, complete_pickup=True):
        response = await self._api_hotel_start({'client_id':'headset', 'confirmed':True,
            'start_at_dock_confirmed':True, 'mission_id':'hotel_public_garbage_patrol_v1',
            'catalog_token':self._hotel.catalog()['token']})
        if complete_pickup and response.status == 200:
            await self.arrive()
            assert (await self._api_hotel_action(self.payload('complete'))).status == 200
        return response

    def payload(self, action, **fields):
        return {'client_id':'headset', 'action':action, 'token':self._hotel_cache['token'],
                'operation_id':uuid4().hex, **fields}

    async def arrive(self, state='succeeded'):
        current = self._task_workflow.current()
        self._task_workflow.navigation_arrived(current['location'], operator=state == 'operator_arrived')
        self._task_workflow.height_adjusted()
        with self._hotel_leg_lock:
            self._hotel_leg['terminal'] = True
            leg = dict(self._hotel_leg)
        await self._hotel_process_navigation(leg, {'active_waypoint':current['location'], 'state':state}, 'succeeded')

    async def close(self):
        await self._hotel_shutdown()
        self._hotel_pool.shutdown(wait=True)


def test_bridge_full_patrol_and_duplicate_request(tmp_path):
    async def run():
        bridge = Harness(tmp_path)
        try:
            assert (await bridge.start()).status == 200
            await bridge.arrive()
            payload = bridge.payload('complete')
            assert (await bridge._api_hotel_action(payload)).status == 200
            before = list(bridge.calls)
            assert json.loads((await bridge._api_hotel_action(payload)).text)['duplicate']
            assert bridge.calls == before
            for _ in range(3):
                await bridge.arrive()
                assert (await bridge._api_hotel_action(bridge.payload('complete'))).status == 200
            await bridge.arrive('docked_by_charge_status')
            assert bridge._hotel_cache['mission']['result'] == 'completed'
            assert bridge._hotel_cache['mission']['charging_confirmed'] is True
            assert [v for k,v in bridge.calls if k == 'navigate'] == ['前台','桌子1','桌子2','桌子3','桌子4','充电桩']
        finally: await bridge.close()
    asyncio.run(run())


def test_stale_skip_cannot_cancel_and_arm_follow_blocks_movement(tmp_path):
    async def run():
        bridge = Harness(tmp_path)
        try:
            await bridge.start()
            await bridge.arrive()
            stale = bridge.payload('skip_area', confirmed=True)
            await bridge._api_hotel_action(bridge.payload('basket_full'))
            before = list(bridge.calls)
            assert (await bridge._api_hotel_action(stale)).status == 409
            assert bridge.calls == before
            bridge.status['active_arms'] = ['right']
            assert (await bridge._api_hotel_action(bridge.payload('dispose', arms_secured=True))).status == 409
            assert bridge._hotel_cache['state'] == 'WAITING_FOR_CLEANING'
            bridge.status['active_arms'] = []
            assert (await bridge._api_hotel_action(bridge.payload('dispose', arms_secured=True))).status == 200
            assert bridge.calls[-1] == ('navigate', '垃圾桶')
            await bridge.arrive()
            await bridge._api_hotel_action(bridge.payload('complete'))
            assert bridge.calls[-1] == ('navigate', '桌子1')
        finally: await bridge.close()
    asyncio.run(run())


def test_old_or_wrong_navigation_result_is_rejected(tmp_path):
    async def run():
        bridge = Harness(tmp_path)
        try:
            await bridge.start()
            assert not bridge._hotel_status_matches({'state':'succeeded', 'active_waypoint':'桌子1', 'stamp':99})
            assert not bridge._hotel_status_matches({'state':'succeeded', 'active_waypoint':'桌子2', 'stamp':101})
            assert not bridge._hotel_status_matches({'state':'succeeded', 'active_waypoint':'桌子1'})
            assert bridge._hotel_status_matches({'state':'succeeded', 'active_waypoint':'桌子1', 'stamp':101})
        finally: await bridge.close()
    asyncio.run(run())


def test_manual_pose_confirmation_and_navigation_resume(tmp_path):
    async def run():
        bridge = Harness(tmp_path)
        try:
            await bridge.start()
            bridge._task_workflow.navigation_failed('manual base takeover')
            with bridge._hotel_leg_lock:
                bridge._hotel_leg['terminal'] = True
                leg = dict(bridge._hotel_leg)
            await bridge._hotel_process_navigation(leg, {'message':'manual base takeover'}, 'failed')
            assert bridge._hotel_cache['state'] == 'PAUSED'
            bridge._task_workflow.navigation_resumed('桌子1')
            await bridge._hotel_resume_existing_leg('桌子1')
            assert bridge._hotel_cache['state'] == 'NAVIGATING_TO_AREA'
            assert bridge._hotel_leg['terminal'] is False
            await bridge.arrive()
            bridge._task_workflow._state = 'failed'
            before = list(bridge.calls)
            response = await bridge._api_hotel_action(bridge.payload('manual_ready', confirmed=True))
            assert response.status == 200
            assert bridge._task_workflow.snapshot()['state'] == 'awaiting_action'
            assert bridge._hotel_cache['mission']['areas'][0]['pose_confirmation'] == 'operator'
            assert bridge.calls == before
            await bridge._api_hotel_action(bridge.payload('basket_full'))
            await bridge._api_hotel_action(bridge.payload('dispose', arms_secured=True))
            await bridge.arrive()
            bridge._task_workflow._state = 'failed'
            assert (await bridge._api_hotel_action(bridge.payload('manual_ready', confirmed=True))).status == 200
            assert bridge._hotel_cache['state'] == 'DUMPING'
            assert bridge._task_workflow.snapshot()['state'] == 'awaiting_action'
        finally: await bridge.close()
    asyncio.run(run())


def test_mark_item_http_action_records_item_and_rejects_stale_replay(tmp_path):
    async def run():
        bridge = Harness(tmp_path)
        try:
            await bridge.start()
            await bridge.arrive()
            payload = bridge.payload('mark_item', label='water_bottle')
            assert (await bridge._api_hotel_action(payload)).status == 200
            assert json.loads((await bridge._api_hotel_action(payload)).text)['duplicate']
            assert bridge._hotel_cache['mission']['areas'][0]['cleaned_items'] == [{'label':'water_bottle','count':1}]
            assert (await bridge._api_hotel_action(bridge.payload('complete'))).status == 200
            await bridge.arrive()
            assert (await bridge._api_hotel_action({**payload, 'operation_id':'stale-mark'})).status == 409
            assert not bridge._hotel_cache['mission']['areas'][1].get('cleaned_items')
        finally:
            await bridge.close()
    asyncio.run(run())


def test_non_dock_legs_reject_task_name_outside_shared_catalog(tmp_path):
    async def run():
        bridge = Harness(tmp_path)
        try:
            await bridge.start(complete_pickup=False)
            await bridge.arrive()
            bridge._task_workflow._catalog.pop(('清理垃圾', '桌子1'))
            response = await bridge._api_hotel_action(bridge.payload('complete'))
            assert response.status == 409
            assert bridge._hotel_cache['state'] == 'PAUSED'
            assert bridge._hotel_cache['mission']['failures']
            assert [x for x in bridge.calls if x[0] == 'navigate'] == [('navigate','前台')]
        finally:
            await bridge.close()
    asyncio.run(run())

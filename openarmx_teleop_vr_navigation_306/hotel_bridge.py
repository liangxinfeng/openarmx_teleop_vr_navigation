"""Hotel mission routes for the 283 navigation bridge (no independent motion loop)."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import math
from pathlib import Path
import threading

from aiohttp import web
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

from .hotel_service import CameraEvidence, HotelMissionService
from .task_workflow import navigation_terminal_outcome


class HotelBridgeMixin:
    def _hotel_manual_action_ready(self):
        """Treat a validated arrival as stopped for manual-only controls.

        The navigation status topic can retain a previous running value after
        the goal has already reached its point.  Hotel states below are only
        entered from a matching arrival callback or an explicit operator
        arrival confirmation, so they are a narrow fallback for reset/pose
        gates. Actual navigating states remain blocked.
        """
        return self._hotel_active() and self._hotel_cache.get('state') in {
            'PICKING_BASKET', 'WAITING_FOR_CLEANING',
            'WAITING_AREA_CONFIRM', 'DUMPING',
        }
    def _hotel_initialize(self):
        share = Path(get_package_share_directory('openarmx_teleop_vr_navigation_306'))
        source = Path('/home/ubuntu/ros2_ws/src/openarmx_teleop_vr_navigation_306/config/hotel_missions.json')
        self.declare_parameter('hotel_mission_config_path', str(source if source.is_file() else share / 'config/hotel_missions.json'))
        self.declare_parameter('hotel_robot_id', '283')
        self._hotel = HotelMissionService(
            self.get_parameter('hotel_mission_config_path').value,
            self.get_parameter('hotel_robot_id').value,
            CameraEvidence(self._rgbd_camera, self._hotel_telemetry),
            task_catalog_path=getattr(self, '_task_catalog_path', None),
            task_catalog_provider=getattr(self, '_task_catalog_snapshot', None),
        )
        self._hotel_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='hotel-mission')
        self._hotel_cache = {'active': False, 'state': 'IDLE', 'token': ''}
        self._hotel_leg = None
        self._hotel_leg_lock = threading.Lock()
        self._hotel_loop = None
        self._hotel_action_lock = None
        self._hotel_observing = False
        self._hotel_jobs = set()

    def _hotel_telemetry(self):
        status, age, fresh = self._teleop_status_snapshot()
        with self._nav_lock:
            pose = dict(self._pose) if self._pose else None
        status = status if isinstance(status, dict) else {}
        backend = status.get('backend') or {}
        return {'pose': pose, 'status_fresh': fresh, 'status_age': age,
                'measured_neck_deg': backend.get('measured_neck_deg'),
                'head_follow_enabled': backend.get('head_follow_enabled'),
                'active_arms': status.get('active_arms')}

    def _hotel_active(self):
        return bool(getattr(self, '_hotel_cache', {}).get('active'))

    async def _hotel_call(self, method, *args, **kwargs):
        loop = asyncio.get_running_loop()
        try:
            return await loop.run_in_executor(self._hotel_pool, partial(method, *args, **kwargs))
        finally:
            self._hotel_cache = await loop.run_in_executor(self._hotel_pool, self._hotel.snapshot)

    def _hotel_register_routes(self, app):
        self._hotel_loop = asyncio.get_running_loop()
        self._hotel_action_lock = asyncio.Lock()
        app.router.add_get('/api/hotel/catalog', self._api_hotel_catalog)
        app.router.add_post('/api/hotel/start', self._api_hotel_start)
        app.router.add_post('/api/hotel/action', self._api_hotel_action)

    async def _api_hotel_catalog(self, request):
        try:
            catalog = await self._hotel_call(self._hotel.catalog)
            return web.json_response({'success': True, **catalog})
        except (OSError, ValueError, TypeError) as error:
            return web.json_response({'success': False, 'message': str(error)}, status=422)

    def _hotel_require_motion_ready(self):
        status, _age, fresh = self._teleop_status_snapshot()
        status = status if isinstance(status, dict) else {}
        if self._exit_in_progress or not fresh or not (status.get('backend') or {}).get('hardware_ready'):
            raise RuntimeError('请先完成VR使能并确认机器人反馈正常')
        if status.get('active_arms'):
            raise RuntimeError('请先松开双手腰键，收好手臂与篮子，再确认移动')
        if (status.get('backend') or {}).get('head_follow_enabled'):
            raise RuntimeError('请先按 B 锁定头部，再确认任务移动')
        with self._drive_lock:
            if self._drive_sample and self._drive_sample.get('deadman'):
                raise RuntimeError('请先退出底盘接管再确认任务移动')

    async def _api_hotel_start(self, request):
        try:
            payload = await self._operator_payload(request)
            if payload.get('confirmed') is not True:
                raise ValueError('请在头显内确认开始酒店巡查')
            if self._hotel_action_lock.locked():
                raise RuntimeError('上一项酒店任务操作仍在处理')
            async with self._hotel_action_lock:
                self._hotel_require_motion_ready()
                if self._hotel_active() or self._task_workflow.snapshot()['state'] not in {'idle', 'completed', 'cancelled', 'failed'} or self._navigation_is_active():
                    raise RuntimeError('请先结束当前任务和导航')
                if self._task_height_target_m is not None:
                    raise RuntimeError('请等待姿态调整完成')
                with self._nav_lock:
                    waypoints = list(self._waypoints)
                await self._hotel_call(self._hotel.start, payload.get('mission_id'), waypoints, payload.get('catalog_token'),
                    start_at_dock_confirmed=payload.get('start_at_dock_confirmed') is True,
                    defer_record=True)
                await self._hotel_start_leg()
                return web.json_response({'success': True, 'message': '酒店巡查已开始，正在自动记录',
                                          'hotel_mission': self._hotel_cache})
        except (OSError, ValueError, TypeError, RuntimeError) as error:
            return web.json_response({'success': False, 'message': str(error), 'hotel_mission': self._hotel_cache}, status=409)

    async def _hotel_start_leg(self):
        leg = await self._hotel_call(self._hotel.pop_leg)
        if leg is None:
            return
        config = self._hotel_cache['mission']['metadata']['config']
        if leg['leg_type'] == 'pickup':
            point = config['pickup']
            task_name = point.get('task', '取垃圾篮')
        elif leg['leg_type'] == 'area':
            point = config['areas'][leg.get('sequence', self._hotel_cache['area_index'] + 1)-1]
            task_name = point.get('task') or '清理' + point.get('name', point['id'])
        elif leg['leg_type'] == 'disposal':
            point = config['trash_bin']
            task_name = point.get('task', '丢垃圾')
        else:
            point = config.get('dock_pose', {})
            # Posture comes from the txt chain's final row (充电桩|充电)；the
            # label stays 返回充电桩 because charging is recorded separately.
            task_name = '返回充电桩'
        with self._hotel_leg_lock:
            self._hotel_leg = {**leg, 'terminal': False, 'suppress_status': False,
                               'sent_stamp': self.get_clock().now().nanoseconds / 1e9}
        try:
            self._task_workflow.cancel('酒店任务切换执行点位')
            # Resolve task capabilities and posture from the shared TXT chain.
            self._task_workflow.submit(config.get('name', '酒店公区巡查'), [{
                'location': leg['waypoint'], 'task': task_name,
                'body_lowering_cm': point.get('body_lowering_cm', 0),
                'head_pitch_deg': point.get('head_pitch_deg', 30),
                'reverse_squat': point.get('reverse_squat', False),
                'automatic': False,
            }], allow_unlisted=leg['leg_type'] == 'return_to_dock')
            self._task_workflow.confirm(list(self._waypoints))
            self._showroom_single_task_active = False
            if not self._hotel_cache.get('record_committed'):
                await self._hotel_call(self._hotel.commit_start_record)
            await self._start_current_task_navigation()
            announcement = str(
                (config.get('voice') or {}).get('leg_announcement', '')
            ).strip()
            if announcement:
                # Queued after this leg's departure prompt, so it plays once
                # that prompt finishes. Runs for every navigation leg,
                # including the first one and the return leg.
                self._speak_task(announcement)
        except (RuntimeError, ValueError) as error:
            with self._hotel_leg_lock:
                self._hotel_leg['terminal'] = True
            discarded = await self._hotel_call(self._hotel.discard_uncommitted_start)
            if discarded:
                self._task_workflow.cancel('酒店任务串未能接受，未创建运营记录')
                with self._hotel_leg_lock:
                    self._hotel_leg = None
            elif self._hotel_cache.get('state') != 'PAUSED':
                await self._hotel_call(self._hotel.apply, 'navigation_failed',
                                       {'detail': str(error)}, internal=True)
            raise

    def _hotel_status_matches(self, status):
        if not self._hotel_active():
            return True
        with self._hotel_leg_lock:
            leg = self._hotel_leg
            if leg is None:
                return False
            if leg.get('suppress_status'):
                return False
            if navigation_terminal_outcome(status.get('state')) is None:
                return True
            try:
                stamp = float(status['stamp'])
            except (KeyError, TypeError, ValueError):
                return False
            return (math.isfinite(stamp) and stamp >= leg['sent_stamp']
                    and str(status.get('active_waypoint', '')).strip() == leg['waypoint'])

    def _hotel_schedule(self, coroutine_factory):
        loop = getattr(self, '_hotel_loop', None)
        if loop is None or loop.is_closed():
            return
        def enqueue():
            job = loop.create_task(coroutine_factory())
            self._hotel_jobs.add(job)
            job.add_done_callback(self._hotel_job_done)
        loop.call_soon_threadsafe(enqueue)

    def _hotel_job_done(self, job):
        self._hotel_jobs.discard(job)
        if not job.cancelled() and job.exception() is not None:
            self.get_logger().error('酒店任务后台处理失败：' + str(job.exception()))

    def _hotel_navigation_status(self, status):
        if not self._hotel_active():
            return
        waypoint = str(status.get('active_waypoint', '')).strip()
        state = str(status.get('state', '')).lower()
        outcome = navigation_terminal_outcome(state)
        if outcome is None:
            return
        with self._hotel_leg_lock:
            leg = self._hotel_leg
            if not leg or not waypoint or waypoint != leg['waypoint']:
                return
            operator = state == 'operator_arrived' and self._hotel_cache.get('state') == 'PAUSED'
            if leg['terminal'] and not operator:
                return
            leg['terminal'] = True
            leg = dict(leg)
        self._hotel_schedule(lambda: self._hotel_process_navigation(leg, status, outcome))

    async def _hotel_process_navigation(self, leg, status, outcome):
        async with self._hotel_action_lock:
            if not self._hotel_active() or self._hotel_leg != leg:
                return
            if self._hotel_cache['state'] not in {'NAVIGATING_TO_PICKUP', 'NAVIGATING_TO_AREA', 'NAVIGATING_TO_BIN', 'RETURNING_TO_DOCK', 'PAUSED'}:
                return
            if outcome == 'failed':
                if self._hotel_cache['state'] == 'PAUSED':
                    return
                await self._hotel_call(self._hotel.apply, 'navigation_failed',
                    {'detail': status.get('message') or status.get('detail') or '导航中断'}, internal=True)
            elif leg['leg_type'] == 'return_to_dock':
                state = str(status.get('state', '')).lower()
                charging = status.get('charging_confirmed')
                if not isinstance(charging, bool):
                    charging = True if state == 'docked_by_charge_status' else None
                await self._hotel_call(self._hotel.apply, 'dock_arrived',
                    {'charging_confirmed': charging, 'operator': state == 'operator_arrived'}, internal=True)
                self._task_workflow.cancel('酒店巡查结束；充电状态已单独记录')
            else:
                await self._hotel_call(self._hotel.apply, 'arrived', {
                    'waypoint': leg['waypoint'], 'operator': str(status.get('state')) == 'operator_arrived',
                }, internal=True)

    async def _api_hotel_action(self, request, forced_action=None):
        try:
            payload = await self._operator_payload(request)
            action = forced_action or payload.get('action')
            if action not in {'complete','retry','basket_full','dispose','dump_failed','retry_dump','pickup_failed','retry_pickup','photo','exception','skip_area','manual_ready','mark_item'}:
                raise ValueError('无效的酒店任务操作')
            if self._hotel_action_lock.locked():
                raise RuntimeError('正在拍照或处理上一操作，请稍候')
            async with self._hotel_action_lock:
                duplicate = await self._hotel_call(self._hotel.validate_request, action, payload)
                if duplicate:
                    return web.json_response({'success': True, 'duplicate': True,
                        'hotel_mission': self._hotel_cache, 'task_dispatch': self._task_workflow.snapshot()})
                if action in {'complete','retry','dispose','skip_area'}:
                    self._hotel_require_motion_ready()
                if action == 'complete' and self._task_workflow.snapshot()['state'] != 'awaiting_action':
                    raise RuntimeError('任务姿态尚未就绪；异常情况请从酒店菜单记录并跳过')
                if action == 'manual_ready':
                    if payload.get('confirmed') is not True or self._hotel_cache['state'] not in {'PICKING_BASKET', 'WAITING_FOR_CLEANING', 'WAITING_AREA_CONFIRM', 'DUMPING'}:
                        raise RuntimeError('请到达前台、桌面或垃圾桶后确认当前姿态可作业')
                    if self._task_workflow.snapshot()['state'] not in {'adjusting_height', 'failed'}:
                        raise RuntimeError('当前无需人工确认作业姿态')
                    status, _age, fresh = self._teleop_status_snapshot()
                    if not fresh or not ((status or {}).get('backend') or {}).get('hardware_ready'):
                        raise RuntimeError('请先完成VR使能并确认反馈正常')
                    # Stop the preset before the workflow is released. This does
                    # not start motion or assert that the preset was reached.
                    self._release_task_height_for_manual_control()
                    if self._task_workflow.snapshot()['state'] != 'awaiting_action':
                        self._task_workflow.operator_action_ready()
                if action == 'skip_area':
                    if payload.get('confirmed') is not True:
                        raise ValueError('跳过桌面需要确认')
                    state = self._hotel_cache['state']
                    if state not in {'NAVIGATING_TO_AREA','WAITING_FOR_CLEANING','WAITING_AREA_CONFIRM','PAUSED'} or not self._hotel_cache['current_area_id']:
                        raise RuntimeError('当前环节不是可跳过的桌面')
                    if state == 'PAUSED' and (self._hotel_cache.get('last_failure') or {}).get('stage') != 'NAVIGATING_TO_AREA':
                        raise RuntimeError('垃圾桶或回桩导航不可当作桌面跳过')
                    with self._hotel_leg_lock:
                        if self._hotel_leg:
                            self._hotel_leg['suppress_status'] = True
                    try:
                        if self._navigation_is_active() or getattr(self, '_navigation_paused_for_drive', False):
                            ok, message = await self._call_service(self._cancel_navigation_client, Trigger.Request())
                            if not ok:
                                raise RuntimeError(message or '无法停止当前导航')
                    finally:
                        with self._hotel_leg_lock:
                            if self._hotel_leg:
                                self._hotel_leg['suppress_status'] = False
                    self._release_task_height_for_manual_control()
                await self._hotel_call(self._hotel.apply, action, payload)
                await self._hotel_start_leg()
                if not self._hotel_active():
                    self._task_workflow.cancel('酒店任务串已按编辑后的任务链完成')
                return web.json_response({'success': True, 'message': '酒店任务操作已记录',
                    'hotel_mission': self._hotel_cache, 'task_dispatch': self._task_workflow.snapshot()})
        except (OSError, ValueError, TypeError, RuntimeError) as error:
            return web.json_response({'success': False, 'message': str(error), 'hotel_mission': self._hotel_cache}, status=409)

    async def _hotel_abort(self, reason):
        if self._hotel_active():
            async with self._hotel_action_lock:
                await self._hotel_call(self._hotel.apply, 'abort', {'reason': reason}, internal=True)
            with self._hotel_leg_lock:
                self._hotel_leg = None

    async def _hotel_resume_existing_leg(self, waypoint):
        if not self._hotel_active():
            return
        async with self._hotel_action_lock:
            if self._hotel_cache['state'] == 'PAUSED':
                await self._hotel_call(self._hotel.apply, 'navigation_resumed',
                                       {'waypoint': waypoint}, internal=True)
            with self._hotel_leg_lock:
                if self._hotel_leg and self._hotel_leg['waypoint'] == waypoint:
                    self._hotel_leg['terminal'] = False
            with self._nav_lock:
                latest = dict(getattr(self, '_navigation_status', {}))
            # A short resumed leg can finish before the resume service returns.
            if self._hotel_status_matches(latest) and navigation_terminal_outcome(latest.get('state')) == 'succeeded':
                self._hotel_navigation_status(latest)

    async def _hotel_shutdown(self):
        for job in tuple(self._hotel_jobs):
            job.cancel()
        if self._hotel_jobs:
            await asyncio.gather(*self._hotel_jobs, return_exceptions=True)
        await self._hotel_call(self._hotel.close)
        self._hotel_loop = None

    async def _hotel_observe(self):
        if self._hotel_observing or not self._hotel_active() or self._hotel_action_lock.locked():
            return
        self._hotel_observing = True
        try:
            async with self._hotel_action_lock:
                status, _age, fresh = self._teleop_status_snapshot()
                if fresh and isinstance(status, dict):
                    await self._hotel_call(self._hotel.observe_head, status)
        finally:
            self._hotel_observing = False

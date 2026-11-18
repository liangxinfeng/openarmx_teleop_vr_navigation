import asyncio
import json
from pathlib import Path

import pytest

from openarmx_teleop_vr_navigation_306.hotel_service import HotelMissionService
from openarmx_teleop_vr_navigation_306.simple_task_catalog import load_catalog
from test_hotel_bridge import Harness

ROOT = Path(__file__).parents[1]
POINTS = ['前台', '桌子1', '桌子2', '桌子3', '桌子4', '垃圾桶', '充电桩']


def create_service(tmp_path):
    config = json.loads((ROOT / 'config/hotel_missions.json').read_text(encoding='utf-8'))
    config['record_root'] = str(tmp_path / 'records')
    path = tmp_path / 'hotel_missions.json'
    path.write_text(json.dumps(config, ensure_ascii=False), encoding='utf-8')
    txt = tmp_path / 'task_actions.txt'
    txt.write_bytes((ROOT / 'config/task_actions.txt').read_bytes())
    return HotelMissionService(path, '283', lambda *_: b'jpeg', task_catalog_path=txt), txt


def test_catalog_uses_txt_order_height_head_and_reverse(tmp_path):
    service, txt = create_service(tmp_path)
    text = txt.read_text(encoding='utf-8')
    text = text.replace('桌子1 | 清理垃圾 | 10 | 否 | 40', '桌子1 | 清理垃圾 | 12 | 否 | 35 | 反方向下蹲')
    text = text.replace('桌子2 | 清理垃圾 | 10 | 否 | 40', '桌子2 | 清理垃圾 | 11 | 否 | 32')
    a, b = '桌子1 | 清理垃圾 | 12 | 否 | 35 | 反方向下蹲', '桌子2 | 清理垃圾 | 11 | 否 | 32'
    text = text.replace(a + '\n' + b, b + '\n' + a)
    txt.write_text(text, encoding='utf-8')
    config = service.catalog()['missions'][0]
    assert [x['waypoint'] for x in config['areas']] == ['桌子2', '桌子1', '桌子3', '桌子4']
    assert config['areas'][1]['body_lowering_cm'] == 12
    assert config['areas'][1]['head_pitch_deg'] == 35
    assert config['areas'][1]['reverse_squat'] is True
    assert config['pickup']['body_lowering_cm'] == 20
    assert config['trash_bin']['body_lowering_cm'] == 15
    assert config['trash_bin']['head_pitch_deg'] == 45
    assert config['dock_pose']['head_pitch_deg'] == 0


def test_start_rejects_missing_pickup_point_unconfirmed_dock_and_changed_txt(tmp_path):
    service, txt = create_service(tmp_path)
    catalog = service.catalog()
    mid = catalog['missions'][0]['id']
    with pytest.raises(RuntimeError, match='充电桩'):
        service.start(mid, POINTS, catalog['token'])
    with pytest.raises(ValueError, match='前台'):
        service.start(mid, POINTS[1:], catalog['token'], start_at_dock_confirmed=True)
    txt.write_text(txt.read_text(encoding='utf-8') + '\n# edited\n', encoding='utf-8')
    with pytest.raises(RuntimeError, match='配置已改变'):
        service.start(mid, POINTS, catalog['token'], start_at_dock_confirmed=True)
    assert service.controller is None
    assert not (tmp_path / 'records').exists()


def test_pickup_failure_retry_dedup_and_navigation_gate(tmp_path):
    async def run():
        bridge = Harness(tmp_path)
        try:
            assert (await bridge.start(complete_pickup=False)).status == 200
            assert bridge.calls == [('navigate', '前台')]
            assert bridge._hotel_cache['mission']['areas'] == []
            await bridge.arrive()
            assert bridge._hotel_cache['state'] == 'PICKING_BASKET'
            assert bridge._task_workflow.current()['body_lowering_cm'] == 20
            assert (await bridge._api_hotel_action(bridge.payload('pickup_failed'))).status == 200
            assert bridge._hotel_cache['state'] == 'PICKUP_FAILED'
            assert (await bridge._api_hotel_action(bridge.payload('complete'))).status == 409
            assert (await bridge._api_hotel_action(bridge.payload('retry_pickup'))).status == 200
            bridge.status['active_arms'] = ['left']
            assert (await bridge._api_hotel_action(bridge.payload('complete'))).status == 409
            bridge.status['active_arms'] = []
            payload = bridge.payload('complete')
            task_id = bridge._hotel_cache['task_id']
            assert (await bridge._api_hotel_action(payload)).status == 200
            assert bridge._hotel_cache['task_id'] == task_id
            assert bridge._hotel_cache['mission']['pickup']['result'] == 'completed'
            assert bridge.calls == [('navigate', '前台'), ('navigate', '桌子1')]
            assert json.loads((await bridge._api_hotel_action(payload)).text)['duplicate']
            assert len(bridge.calls) == 2
        finally:
            await bridge.close()
    asyncio.run(run())


def test_pickup_navigation_failure_retries_front_desk_and_manual_arrival(tmp_path):
    service, txt = create_service(tmp_path)
    try:
        catalog = service.catalog()
        service.start(catalog['missions'][0]['id'], POINTS, catalog['token'], start_at_dock_confirmed=True)
        task_id = service.snapshot()['task_id']
        service.pop_leg()
        service.apply('navigation_failed', {'detail': 'blocked'}, internal=True)
        service.apply('retry', {}, internal=True)
        assert service.pop_leg()['waypoint'] == '前台'
        service.apply('navigation_failed', {'detail': 'manual takeover'}, internal=True)
        service.apply('arrived', {'waypoint': '前台', 'operator': True}, internal=True)
        assert service.snapshot()['state'] == 'PICKING_BASKET'
        assert service.snapshot()['task_id'] == task_id
        assert not service.snapshot()['mission']['areas']
    finally:
        service.close()


def test_static_chain_shape_and_pairs_match_task_library(tmp_path):
    catalog = load_catalog(ROOT / 'config/task_actions.txt')
    chain = next(x for x in catalog['task_templates'] if x['name'] == '酒店垃圾巡查')
    # 静态路线＝前台取篮、四桌各一次、回桩；垃圾桶只允许作为篮满动态分支。
    assert [t['location'] for t in chain['tasks']] == ['前台', '桌子1', '桌子2', '桌子3', '桌子4', '充电桩']
    assert not any(t['location'] == '垃圾桶' for t in chain['tasks'])
    listed = {(location, entry['task']): entry
              for location, entries in catalog['task_locations'].items() for entry in entries}
    for location, task, cm, pitch in [('前台', '取垃圾篮', 20, 40), ('桌子1', '清理垃圾', 10, 40),
                                      ('桌子2', '清理垃圾', 10, 40), ('桌子3', '清理垃圾', 20, 40),
                                      ('桌子4', '清理垃圾', 20, 40), ('垃圾桶', '丢垃圾', 15, 45),
                                      ('充电桩', '充电', 0, 0)]:
        entry = listed.get((location, task))
        assert entry, f'任务库缺少 {location} / {task}'
        assert entry['body_lowering_cm'] == cm and entry['head_pitch_deg'] == pitch


def test_patrol_legs_use_txt_chain_names_posture_and_marks(tmp_path):
    async def run():
        bridge = Harness(tmp_path)
        try:
            assert (await bridge.start(complete_pickup=False)).status == 200
            leg = bridge._task_workflow.current()
            assert (leg['location'], leg['task']) == ('前台', '取垃圾篮')
            assert (leg['body_lowering_cm'], leg['head_pitch_deg']) == (20, 40)
            assert bridge._task_workflow.snapshot()['source_text'] == '酒店公区垃圾巡查'
            await bridge.arrive()
            assert (await bridge._api_hotel_action(bridge.payload('complete'))).status == 200
            task_id = bridge._hotel_cache['task_id']
            for index, cm in enumerate([10, 10, 20, 20], 1):
                leg = bridge._task_workflow.current()
                assert (leg['location'], leg['task']) == (f'桌子{index}', '清理垃圾')
                assert (leg['body_lowering_cm'], leg['head_pitch_deg']) == (cm, 40)
                await bridge.arrive()
                if index == 1:
                    assert (await bridge._api_hotel_action(
                        bridge.payload('mark_item', label='water_bottle'))).status == 200
                    assert (await bridge._api_hotel_action(
                        bridge.payload('mark_item', label='tissue'))).status == 200
                    assert (await bridge._api_hotel_action(
                        bridge.payload('mark_item', label='bad_label'))).status == 409
                    await bridge._api_hotel_action(bridge.payload('basket_full'))
                    assert (await bridge._api_hotel_action(
                        bridge.payload('dispose', arms_secured=True))).status == 200
                    leg = bridge._task_workflow.current()
                    assert (leg['location'], leg['task']) == ('垃圾桶', '丢垃圾')
                    assert (leg['body_lowering_cm'], leg['head_pitch_deg']) == (15, 45)
                    await bridge.arrive()
                    assert (await bridge._api_hotel_action(bridge.payload('complete'))).status == 200
                    assert bridge._hotel_cache['task_id'] == task_id
                    assert bridge._task_workflow.current()['location'] == '桌子1'
                    await bridge.arrive()
                assert (await bridge._api_hotel_action(bridge.payload('complete'))).status == 200
            leg = bridge._task_workflow.current()
            assert (leg['location'], leg['task']) == ('充电桩', '返回充电桩')
            assert (leg['body_lowering_cm'], leg['head_pitch_deg']) == (0, 0)
            await bridge.arrive('docked_by_charge_status')
            assert bridge._hotel_cache['task_id'] == task_id
            assert bridge._hotel_cache['mission']['result'] == 'completed'
            resumed = next(x for x in bridge._hotel_cache['mission']['areas']
                           if x['visit_id'].endswith('visit_002'))
            assert resumed['cleaned_items'] == [{'label': 'tissue', 'count': 1},
                                                {'label': 'water_bottle', 'count': 1}]
            events = (Path(bridge._hotel_cache['record_directory']) / 'events.jsonl').read_text(encoding='utf-8')
            assert events.count('cleaned_item_marked') == 2
        finally:
            await bridge.close()
    asyncio.run(run())

import json
from pathlib import Path
from uuid import uuid4

import pytest

from openarmx_teleop_vr_navigation_306.hotel_service import CameraEvidence, HotelMissionService
from openarmx_teleop_vr_navigation_306.recognition_provider import validate_recognition_result


@pytest.fixture
def service(tmp_path):
    config = json.loads((Path(__file__).parents[1] / 'config/hotel_missions.json').read_text(encoding='utf-8'))
    config['record_root'] = str(tmp_path / 'records')
    path = tmp_path / 'hotel.json'
    path.write_text(json.dumps(config, ensure_ascii=False), encoding='utf-8')
    value = HotelMissionService(path, '283', lambda stage, ctx: {'data': b'jpeg', 'metadata': {'frame_id': 3}},
        task_catalog_path=Path(__file__).parents[1] / 'config/task_actions.txt')
    value.start(config['missions'][0]['id'], ['前台','桌子1','桌子2','桌子3','桌子4','垃圾桶','充电桩'],
                value.catalog()['token'], start_at_dock_confirmed=True)
    assert value.pop_leg()['waypoint'] == '前台'
    value.apply('arrived', {'waypoint': '前台'}, internal=True)
    command(value, 'complete')
    yield value
    value.close()


def command(service, action, **extra):
    return service.apply(action, {'token': service.snapshot()['token'], 'operation_id': uuid4().hex, **extra})


def arrive(service):
    leg = service.pop_leg()
    service.apply('arrived', {'waypoint': leg['waypoint']}, internal=True)
    return leg


def test_mid_area_detour_keeps_task_and_returns_to_unfinished_table(service):
    task_id = service.snapshot()['task_id']
    assert arrive(service)['waypoint'] == '桌子1'
    command(service, 'basket_full')
    with pytest.raises(RuntimeError):
        command(service, 'dispose', arms_secured=False)
    command(service, 'dispose', arms_secured=True)
    assert arrive(service)['waypoint'] == '垃圾桶'
    command(service, 'complete')
    assert service.snapshot()['task_id'] == task_id
    assert arrive(service)['waypoint'] == '桌子1'
    areas = service.snapshot()['mission']['areas']
    assert areas[0]['status'] == 'suspended_for_disposal'
    assert areas[1]['resumes_visit_id'] == areas[0]['visit_id']


def test_finished_table_detour_moves_to_next_and_dump_retry_is_stationary(service):
    arrive(service)
    command(service, 'basket_full')
    command(service, 'complete')
    assert service.snapshot()['state'] == 'PAUSED_FOR_DISPOSAL'
    command(service, 'dispose', arms_secured=True)
    arrive(service)
    command(service, 'dump_failed', note='篮子未对准')
    command(service, 'retry_dump')
    assert service.pop_leg() is None
    assert service.snapshot()['state'] == 'DUMPING'
    command(service, 'complete')
    assert service.pop_leg()['waypoint'] == '桌子2'


def test_duplicate_completion_cannot_advance_or_cancel_next_leg(service):
    arrive(service)
    payload = {'token': service.snapshot()['token'], 'operation_id': 'complete-1'}
    service.apply('complete', payload)
    assert service.validate_request('complete', payload) is True
    assert service.apply('complete', payload)['duplicate'] is True
    assert service.pop_leg()['waypoint'] == '桌子2'
    assert service.pop_leg() is None
    with pytest.raises(RuntimeError):
        service.apply('complete', {**payload, 'result': 'failed'})
    with pytest.raises(RuntimeError):
        service.validate_request('skip_area', {**payload, 'operation_id': 'stale-skip'})


def test_manual_head_stability_photos_are_bounded_and_keep_confirmation_token(service):
    arrive(service)
    token = service.snapshot()['token']
    status = {'backend': {'head_follow_enabled': True, 'measured_neck_deg': [0, -30, 0]}}
    service.observe_head(status, now=10)
    service.observe_head(status, now=10.5)
    assert len(service.snapshot()['mission']['artifacts']['images']) == 1
    service.observe_head(status, now=11)
    for tick in range(12, 80):
        service.observe_head(status, now=tick)
    photos = service.snapshot()['mission']['artifacts']['images']
    assert sum(p['stage'] == 'supplemental' for p in photos) == 6
    assert service.snapshot()['token'] == token


def test_photo_failure_remains_incomplete_after_successful_patrol(service):
    service.controller.capture_photo = lambda *_: None
    for _ in range(4):
        arrive(service)
        command(service, 'complete')
    assert service.pop_leg()['waypoint'] == '充电桩'
    service.apply('dock_arrived', {'charging_confirmed': True}, internal=True)
    assert service.snapshot()['mission']['data_quality'] == 'incomplete'
    assert service.snapshot()['mission']['review_required'] is True


def test_navigation_skip_and_retry_records_are_not_synthetic_success(service):
    service.pop_leg()
    service.apply('navigation_failed', {'detail': '被遮挡'}, internal=True)
    command(service, 'skip_area', reason='看不到桌面')
    assert service.snapshot()['mission']['areas'][0]['result'] == 'skipped'
    assert service.snapshot()['mission']['areas'][0]['after_photo'] is None
    assert service.snapshot()['mission']['artifacts']['images'] == []
    assert service.pop_leg()['waypoint'] == '桌子2'
    assert service.snapshot()['mission']['data_quality'] == 'incomplete'


def test_fresh_camera_requires_frame_after_request():
    class Camera:
        def status(self): return {'frame_version': 17}
        def snapshot(self, after, timeout):
            assert after == 17
            return b'jpeg', 18, {'online': True, 'frame_age': 0.01}
    assert CameraEvidence(Camera())('before', {})['metadata']['frame_id'] == 18
    camera = Camera()
    camera.snapshot = lambda *_: (b'jpeg', 17, {'online': True})
    with pytest.raises(RuntimeError): CameraEvidence(camera)('after', {})
    camera.snapshot = lambda *_: (b'jpeg', 18, {'online': False, 'frame_age': 2})
    with pytest.raises(RuntimeError): CameraEvidence(camera)('after', {})


@pytest.mark.parametrize('payload', [{'objects': [{'label':'water_cup','count':True}]},
    {'objects':[{'label':'water_bottle','confidence':float('nan')}]}, {'objects':[{'label':'unsupported'}]}])
def test_malformed_recognition_is_rejected(payload):
    with pytest.raises(ValueError): validate_recognition_result(payload)


def test_recognition_artifacts_link_back_to_photo_and_markdown(service):
    arrive(service)
    assert service.worker.wait_idle(3)
    summary = service.controller.recorder.summary()
    detection = summary['artifacts']['detections'][0]
    assert detection['image_path'] == summary['artifacts']['images'][0]['path']
    assert detection['status'] == 'not_configured'
    assert summary['areas'][0]['recognition']['before']
    assert (service.controller.recorder.mission_dir / 'report.md').is_file()


def test_operator_arrival_after_dock_navigation_failure(service):
    for _ in range(4):
        arrive(service)
        command(service, 'complete')
    service.pop_leg()
    service.apply('navigation_failed', {'detail':'dock blocked'}, internal=True)
    service.apply('dock_arrived', {'operator':True, 'charging_confirmed':None}, internal=True)
    assert service.snapshot()['state'] == 'FINISHED'
    assert service.snapshot()['mission']['result'] == 'completed'
    assert service.snapshot()['mission']['charging_confirmed'] is None
    assert service.snapshot()['mission']['review_required'] is True

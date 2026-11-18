import json

import pytest

from openarmx_teleop_vr_navigation_306.hotel_mission import (
    HotelMissionController,
    MissionError,
    SafetyCheckpointRequired,
)
from openarmx_teleop_vr_navigation_306.mission_recorder import MissionRecorder


CONFIG = {
    "id": "hotel_public_garbage_patrol_v1",
    "start_waypoint": "充电桩",
    "return_waypoint": "充电桩",
    "areas": [
        {"id": "round_table_1", "name": "圆桌1", "waypoint": "桌子1"},
        {"id": "round_table_2", "name": "圆桌2", "waypoint": "桌子2"},
        {"id": "round_table_3", "name": "圆桌3", "waypoint": "桌子3"},
        {"id": "round_table_4", "name": "圆桌4", "waypoint": "桌子4"},
    ],
    "trash_bin": {"waypoint": "垃圾桶"},
    "basket": {"carrier_hand": "left", "grasp_hand": "right"},
}


def _controller(tmp_path, navigate_log):
    recorder = MissionRecorder(tmp_path, task_id="mission_001", robot_id="328")

    def capture(stage, context):
        return {
            "data": f"{stage}:{context['area_id']}".encode(),
            "metadata": {"head_pose_ready": stage != "supplemental" or True},
        }

    return HotelMissionController(
        CONFIG,
        recorder,
        navigate=lambda waypoint, context: navigate_log.append((waypoint, dict(context))),
        capture_photo=capture,
    )


def _arrive_and_complete(controller, result="completed", arms_secured=False):
    controller.on_navigation_arrived()
    controller.confirm_area(
        result=result,
        cleaned_items=[{"label": "water_bottle", "count": 1}],
        arms_secured=arms_secured,
    )


def test_normal_four_area_route_returns_to_dock(tmp_path):
    navigation = []
    controller = _controller(tmp_path, navigation)
    controller.start()
    assert navigation[0][0] == "桌子1"

    for _ in range(4):
        _arrive_and_complete(controller)
        if controller.state == "RETURNING_TO_DOCK":
            break

    assert controller.state == "RETURNING_TO_DOCK"
    controller.on_dock_arrived(charging_confirmed=True)
    snapshot = controller.snapshot()
    assert snapshot["state"] == "FINISHED"
    assert snapshot["mission"]["result"] == "completed"
    assert snapshot["task_id"] == "mission_001"
    assert [item[0] for item in navigation] == ["桌子1", "桌子2", "桌子3", "桌子4", "充电桩"]


def test_basket_full_after_completed_area_inserts_bin_and_resumes_next_area(tmp_path):
    navigation = []
    controller = _controller(tmp_path, navigation)
    controller.start()
    controller.on_navigation_arrived()
    controller.set_basket_state("full")
    controller.confirm_area(arms_secured=True)

    assert controller.state == "NAVIGATING_TO_BIN"
    assert navigation[-1][0] == "垃圾桶"
    controller.on_navigation_arrived()
    controller.confirm_disposal(success=True)

    assert controller.state == "NAVIGATING_TO_AREA"
    assert navigation[-1][0] == "桌子2"
    assert controller.task_id == "mission_001"
    assert controller.snapshot()["mission"]["detours"][0]["resume_area"] == "round_table_2"


def test_basket_full_mid_area_requires_safe_checkpoint_and_resumes_same_area(tmp_path):
    navigation = []
    controller = _controller(tmp_path, navigation)
    controller.start()
    controller.on_navigation_arrived()
    controller.set_basket_state("full")
    with pytest.raises(SafetyCheckpointRequired):
        controller.request_disposal(arms_secured=False)
    controller.request_disposal(arms_secured=True)
    controller.on_navigation_arrived()
    controller.confirm_disposal(success=True)

    assert navigation[-1][0] == "桌子1"
    assert controller.snapshot()["mission"]["areas"][-1]["resumes_visit_id"] == "round_table_1_visit_001"


def test_head_alignment_supplemental_photo_does_not_advance_mission(tmp_path):
    navigation = []
    controller = _controller(tmp_path, navigation)
    controller.start()
    controller.on_navigation_arrived()
    path = controller.capture_supplemental_photo("head_b_manual_alignment")
    assert path and "supplemental_" in path
    assert controller.state == "WAITING_FOR_CLEANING"
    assert len(controller.snapshot()["mission"]["artifacts"]["images"]) == 2


def test_partial_area_is_not_reported_as_completed(tmp_path):
    navigation = []
    controller = _controller(tmp_path, navigation)
    controller.start()
    _arrive_and_complete(controller, result="partial")
    assert controller.snapshot()["mission"]["review_required"] is True
    for _ in range(3):
        _arrive_and_complete(controller)
        if controller.state == "RETURNING_TO_DOCK":
            break
    controller.on_dock_arrived(charging_confirmed=True)
    assert controller.snapshot()["mission"]["result"] == "failed"


def test_disposal_failure_is_retryable_and_recorded(tmp_path):
    navigation = []
    controller = _controller(tmp_path, navigation)
    controller.start()
    controller.on_navigation_arrived()
    controller.set_basket_state("full")
    controller.confirm_area(arms_secured=True)
    controller.on_navigation_arrived()
    controller.confirm_disposal(success=False, note="篮子没有对准垃圾桶")
    assert controller.state == "PAUSED_FOR_DISPOSAL"
    assert controller.snapshot()["mission"]["failures"][0]["code"] == "DUMP_FAILED"
    events = (controller.recorder.mission_dir / "events.jsonl").read_text(encoding="utf-8")
    assert "disposal_failed" in events


def test_recoverable_navigation_failure_can_retry_without_changing_task_id(tmp_path):
    navigation = []
    controller = _controller(tmp_path, navigation)
    controller.start()
    task_id = controller.task_id
    controller.on_navigation_failed("NAVIGATION_TIMEOUT", "桌子1未到达")
    assert controller.state == "PAUSED"
    controller.retry_navigation()
    assert controller.state == "NAVIGATING_TO_AREA"
    controller.on_navigation_arrived()
    assert controller.task_id == task_id
    assert controller.snapshot()["mission"]["review_required"] is True


def test_marked_cleaned_items_validate_merge_and_survive_disposal_resume(tmp_path):
    navigation = []
    controller = _controller(tmp_path, navigation)
    controller.start()
    with pytest.raises(MissionError):
        controller.mark_cleaned_item("water_bottle")  # 仅到达桌面后可标记
    controller.on_navigation_arrived()
    with pytest.raises(ValueError):
        controller.mark_cleaned_item("not_allowed")
    controller.mark_cleaned_item("water_bottle")
    controller.mark_cleaned_item("water_bottle", operation_id="op-1")
    controller.mark_cleaned_item("water_bottle", operation_id="op-1")  # 幂等
    controller.mark_cleaned_item("tissue")
    record = next(x for x in controller.snapshot()["mission"]["areas"]
                  if x["visit_id"] == "round_table_1_visit_001")
    assert record["cleaned_items"] == [{"label": "tissue", "count": 1},
                                       {"label": "water_bottle", "count": 2}]
    with pytest.raises(ValueError):
        controller.confirm_area(cleaned_items=[{"label": "not_allowed", "count": 1}])
    assert controller.state == "WAITING_FOR_CLEANING"  # 校验失败不得推进任务
    controller.set_basket_state("full")
    controller.request_disposal(arms_secured=True)  # 本桌尚未确认完成，倒完原地恢复
    controller.on_navigation_arrived()
    controller.confirm_disposal(success=True)
    assert navigation[-2][0] == "垃圾桶"
    assert navigation[-1][0] == "桌子1"  # 恢复当前桌
    controller.on_navigation_arrived()
    controller.confirm_area(cleaned_items=[{"label": "other", "count": 1}])
    finished = next(x for x in controller.snapshot()["mission"]["areas"]
                    if x["visit_id"] == "round_table_1_visit_002")
    assert finished["cleaned_items"] == [{"label": "other", "count": 1},
                                         {"label": "tissue", "count": 1},
                                         {"label": "water_bottle", "count": 2}]
    assert controller.state == "NAVIGATING_TO_AREA"  # 上一桌确认后已自动推进到下一桌
    controller.on_navigation_arrived()
    controller.mark_cleaned_item("paper")
    controller.confirm_area()
    tables = controller.snapshot()["mission"]["areas"]
    assert next(x for x in tables if x["visit_id"] == "round_table_2_visit_001")["cleaned_items"] \
        == [{"label": "paper", "count": 1}]
    assert "cleaned_item_marked" in (controller.recorder.mission_dir / "events.jsonl").read_text(encoding="utf-8")


def test_invalid_item_payload_can_be_corrected_without_consuming_operation(tmp_path):
    controller = _controller(tmp_path, [])
    controller.start()
    controller.on_navigation_arrived()
    with pytest.raises(ValueError):
        controller.mark_cleaned_item('invalid', operation_id='mark-corrected')
    controller.mark_cleaned_item('water_bottle', operation_id='mark-corrected')
    controller.mark_cleaned_item('water_bottle', operation_id='mark-corrected')
    for count in [True, 1.7, '2']:
        with pytest.raises(ValueError, match='整数'):
            controller.confirm_area(cleaned_items=[{'label':'water_bottle', 'count':count}],
                                    operation_id='complete-corrected')
        assert controller.state == 'WAITING_FOR_CLEANING'
    controller.confirm_area(cleaned_items=[{'label':'water_bottle', 'count':2}],
                            operation_id='complete-corrected')
    assert controller.snapshot()['mission']['areas'][0]['cleaned_items'] == [{'label':'water_bottle','count':2}]

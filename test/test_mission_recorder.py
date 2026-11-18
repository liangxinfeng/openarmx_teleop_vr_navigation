import json

from openarmx_teleop_vr_navigation_306.mission_recorder import MissionRecorder


def test_recorder_writes_idempotent_events_and_artifacts(tmp_path):
    recorder = MissionRecorder(
        tmp_path,
        task_id="cleanup_test_001",
        robot_id="328",
    )
    mission_dir = recorder.start()
    first = recorder.append_event("operator_action", event_id="a-1", payload={"key": "A"})
    duplicate = recorder.append_event("operator_action", event_id="a-1", payload={"key": "A"})
    assert first["event_id"] == "a-1"
    assert duplicate["duplicate"] is True

    recorder.upsert_area("round_table_1", "round_table_1_visit_001", status="ready")
    image_path = recorder.save_photo_bytes(
        b"jpeg-bytes",
        area_id="round_table_1",
        visit_id="round_table_1_visit_001",
        stage="before",
        metadata={"frame_id": 7},
    )
    assert (mission_dir / image_path).read_bytes() == b"jpeg-bytes"

    summary = recorder.finish(result="completed")
    assert summary["result"] == "completed"
    assert summary["ended_at"]
    events = [
        json.loads(line)
        for line in (mission_dir / "events.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert [event["event_type"] for event in events] == [
        "mission_started",
        "operator_action",
        "photo_captured",
        "mission_finished",
    ]


def test_recorder_can_reopen_and_keep_event_idempotency(tmp_path):
    recorder = MissionRecorder(tmp_path, task_id="recover_001", robot_id="306")
    mission_dir = recorder.start()
    recorder.append_event("checkpoint", event_id="checkpoint-1")

    reopened = MissionRecorder.open_existing(mission_dir)
    duplicate = reopened.append_event("checkpoint", event_id="checkpoint-1")
    assert duplicate["duplicate"] is True

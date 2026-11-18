from openarmx_teleop_vr_navigation_306.mission_recorder import MissionRecorder
from openarmx_teleop_vr_navigation_306.recognition_worker import RecognitionWorker


def test_recognition_worker_is_async_and_persists_result(tmp_path):
    recorder = MissionRecorder(tmp_path, task_id="recognition_001", robot_id="328")
    recorder.start()
    image = tmp_path / "before.jpg"
    image.write_bytes(b"image")

    def recognizer(path, metadata):
        assert path == image
        assert metadata["source"] == "test"
        return {"status": "completed", "objects": [{"label": "water_bottle"}]}

    worker = RecognitionWorker(recorder, recognizer)
    request_id = worker.submit(
        image,
        area_id="round_table_1",
        visit_id="round_table_1_visit_001",
        stage="before",
        metadata={"source": "test"},
    )
    assert worker.wait_idle(2.0) is True
    worker.close()

    events = (recorder.mission_dir / "events.jsonl").read_text(encoding="utf-8")
    assert "recognition_queued" in events
    assert "recognition_completed" in events
    assert request_id in events
    assert list((recorder.mission_dir / "detections").rglob("*.json"))

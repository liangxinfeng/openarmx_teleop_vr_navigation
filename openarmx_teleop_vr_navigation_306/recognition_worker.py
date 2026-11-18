"""Non-blocking image recognition queue for hotel mission evidence."""

from __future__ import annotations

from dataclasses import dataclass
import queue
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping
from uuid import uuid4

from .mission_recorder import MissionRecorder, safe_path_token
from .recognition_provider import validate_recognition_result


Recognizer = Callable[[Path, Mapping[str, Any]], Mapping[str, Any]]


@dataclass(frozen=True)
class RecognitionRequest:
    request_id: str
    image_path: Path
    area_id: str
    visit_id: str
    stage: str
    metadata: dict[str, Any]


class RecognitionWorker:
    """Run optional recognition work away from VR/navigation callbacks.

    The worker accepts an injected recognizer so the first implementation can
    use a local model, a subprocess, or a remote service without changing the
    mission recorder contract.  No recognition result is used to gate motion.
    """

    def __init__(
        self,
        recorder: MissionRecorder,
        recognizer: Recognizer | None = None,
    ) -> None:
        self.recorder = recorder
        self.recognizer = recognizer or self._not_configured
        self._queue: queue.Queue[RecognitionRequest | None] = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name=f"recognition-{recorder.task_id}",
            daemon=True,
        )
        self._thread.start()

    @staticmethod
    def _not_configured(_image_path: Path, _metadata: Mapping[str, Any]) -> Mapping[str, Any]:
        return {
            "status": "not_configured",
            "objects": [],
            "cleaning_result": "pending_review",
        }

    def submit(
        self,
        image_path: str | Path,
        *,
        area_id: str,
        visit_id: str,
        stage: str,
        metadata: Mapping[str, Any] | None = None,
    ) -> str:
        request_id = str(uuid4())
        request = RecognitionRequest(
            request_id=request_id,
            image_path=Path(image_path),
            area_id=str(area_id),
            visit_id=str(visit_id),
            stage=str(stage),
            metadata=dict(metadata or {}),
        )
        if self._stop.is_set():
            raise RuntimeError('recognition worker is closed')
        self.recorder.append_event(
            "recognition_queued",
            area_id=request.area_id,
            visit_id=request.visit_id,
            phase=request.stage,
            payload={
                "request_id": request.request_id,
                "image_path": str(request.image_path),
            },
        )
        try:
            self._queue.put_nowait(request)
        except queue.Full:
            self.recorder.append_event('recognition_deferred', area_id=request.area_id,
                visit_id=request.visit_id, phase=request.stage,
                payload={'request_id': request_id, 'image_path': str(request.image_path),
                         'reason': 'queue_full', 'retry_offline': True})
        return request_id

    def _run(self) -> None:
        while True:
            try:
                request = self._queue.get(timeout=0.1)
            except queue.Empty:
                if self._stop.is_set():
                    return
                continue
            try:
                if request is None:
                    return
                if self._stop.is_set():
                    self.recorder.append_event('recognition_deferred', area_id=request.area_id,
                        visit_id=request.visit_id, phase=request.stage,
                        payload={'request_id': request.request_id, 'image_path': str(request.image_path),
                                 'reason': 'worker_shutdown', 'retry_offline': True})
                    continue
                try:
                    result = validate_recognition_result(dict(self.recognizer(request.image_path, request.metadata)))
                    result.setdefault("status", "completed")
                    result["request_id"] = request.request_id
                    result["area_id"] = request.area_id
                    result["visit_id"] = request.visit_id
                    result["stage"] = request.stage
                    result['image_path'] = request.image_path.relative_to(self.recorder.mission_dir).as_posix() if request.image_path.is_relative_to(self.recorder.mission_dir) else str(request.image_path)
                    relative = (
                        Path("detections")
                        / safe_path_token(request.area_id, "area")
                        / safe_path_token(request.visit_id, "visit")
                        / f"{safe_path_token(request.stage, 'stage')}_{request.request_id[:12]}.json"
                    ).as_posix()
                    artifact_path = self.recorder.save_json_artifact(relative, result)
                    self.recorder.record_detection(artifact_path, result)
                    self.recorder.append_event(
                        "recognition_completed",
                        area_id=request.area_id,
                        visit_id=request.visit_id,
                        phase=request.stage,
                        payload={
                            "request_id": request.request_id,
                            "artifact_path": artifact_path,
                            "status": result.get("status"),
                        },
                    )
                except Exception as error:  # noqa: BLE001 - worker must survive one bad image
                    self.recorder.append_event(
                        "recognition_failed",
                        area_id=request.area_id,
                        visit_id=request.visit_id,
                        phase=request.stage,
                        payload={
                            "request_id": request.request_id,
                            "error": str(error),
                        },
                    )
            finally:
                self._queue.task_done()

    def wait_idle(self, timeout: float = 5.0) -> bool:
        end = time.monotonic() + max(0.0, float(timeout))
        while self._queue.unfinished_tasks and time.monotonic() < end:
            time.sleep(0.01)
        return self._queue.unfinished_tasks == 0

    def close(self, timeout: float = 2.0) -> None:
        if self._stop.is_set():
            return
        self.wait_idle(timeout=timeout)
        self._stop.set()
        self._thread.join(timeout=max(0.0, float(timeout)))

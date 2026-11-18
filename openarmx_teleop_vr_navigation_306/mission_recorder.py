"""Atomic, single-file mission records and non-overwriting photo artifacts."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime
import json
import logging
import os
from pathlib import Path
import re
import tempfile
import threading
import time
from typing import Any, Mapping
from uuid import uuid4


MAX_EVENTS = 2048
MAX_OPERATIONS = 512
MAX_PHOTOS = 512
MAX_RECORD_BYTES = 8 * 1024 * 1024


def local_iso_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def safe_path_token(value: Any, fallback: str = "unknown") -> str:
    token = re.sub(r"[^\w.\-]+", "_", str(value or "").strip(), flags=re.UNICODE)
    return token.strip("._") or fallback


ITEM_LABELS = {"water_bottle": "水瓶", "water_cup": "水杯", "tissue": "纸巾",
               "paper": "纸张", "other": "其他"}
LOGGER = logging.getLogger(__name__)


class MissionRecorder:
    """One mission.json is authoritative; photo bytes are stored separately."""

    def __init__(
        self,
        root: str | Path,
        *,
        task_id: str,
        robot_id: str,
        task_type: str = "hotel_public_garbage_patrol",
        metadata: Mapping[str, Any] | None = None,
        started_at: str | None = None,
        defer_persistence: bool = False,
    ) -> None:
        self.root = Path(root).expanduser()
        self.task_id = str(task_id).strip()
        self.robot_id = str(robot_id).strip()
        if not self.task_id or not self.robot_id:
            raise ValueError("task_id and robot_id must not be empty")
        self.task_type = str(task_type).strip() or "hotel_public_garbage_patrol"
        self.started_at = started_at or local_iso_now()
        self.mission_dir = self.root / self.started_at[:10].replace("-", "") / safe_path_token(self.task_id, "mission")
        self.summary_path = self.mission_dir / "mission.json"
        self.events_path = self.mission_dir / "events.jsonl"  # legacy path; never created
        self._lock = threading.RLock()
        self._started = False
        self._committed = False
        self._defer_persistence = bool(defer_persistence)
        self._revision = 0
        self._summary: dict[str, Any] = {
            "schema_version": "2.0",
            "task_id": self.task_id,
            "task_type": self.task_type,
            "robot_id": self.robot_id,
            "started_at": self.started_at,
            "ended_at": None,
            "status": "running",
            "execution_status": "running",
            "result": "pending",
            "data_quality": "complete",
            "review_required": False,
            "current_state": "IDLE",
            "current_area_id": None,
            "current_visit_id": None,
            "steps": [],
            "events": [],
            "processed_operations": [],
            "failures": [],
            "detours": [],
        }
        if metadata:
            self._summary["metadata"] = deepcopy(dict(metadata))

    @classmethod
    def open_existing(cls, mission_dir: str | Path) -> "MissionRecorder":
        """Open v2 records for idempotent updates; legacy records remain read-only."""
        mission_path = Path(mission_dir).expanduser().resolve()
        summary_path = mission_path / "mission.json"
        if not summary_path.is_file():
            raise FileNotFoundError(summary_path)
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if summary.get("schema_version") != "2.0":
            raise ValueError("legacy mission records are read-only and are never migrated in place")
        recorder = cls.__new__(cls)
        recorder.root = mission_path.parent.parent
        recorder.task_id = str(summary["task_id"])
        recorder.robot_id = str(summary["robot_id"])
        recorder.task_type = str(summary.get("task_type", "hotel_public_garbage_patrol"))
        recorder.started_at = str(summary.get("started_at") or local_iso_now())
        recorder.mission_dir = mission_path
        recorder.summary_path = summary_path
        recorder.events_path = mission_path / "events.jsonl"
        recorder._lock = threading.RLock()
        recorder._started = True
        recorder._committed = True
        recorder._defer_persistence = False
        recorder._revision = 0
        recorder._summary = summary
        return recorder

    @property
    def started(self) -> bool:
        return self._started

    @property
    def revision(self) -> int:
        return self._revision

    @property
    def committed(self) -> bool:
        return self._committed

    def start(self) -> Path:
        with self._lock:
            if self._started:
                return self.mission_dir
            self._started = True
            self.append_event("mission_started", source="system")
            if not self._defer_persistence:
                self.commit()
            return self.mission_dir

    def commit(self) -> Path:
        """Persist a prepared mission after the hotel task chain is accepted."""
        with self._lock:
            self._require_started()
            if self._committed:
                return self.mission_dir
            self.mission_dir.mkdir(parents=True, exist_ok=True)
            if self.summary_path.exists():
                raise FileExistsError(f"mission record already exists: {self.summary_path}")
            (self.mission_dir / "photos" / "detections").mkdir(parents=True, exist_ok=True)
            self._write_summary_locked(force=True)
            self._committed = True
            self._defer_persistence = False
            return self.mission_dir

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("mission recorder has not been started")

    def _write_summary_locked(self, *, force: bool = False) -> None:
        if not self._committed and not force:
            self._revision += 1
            return
        self.mission_dir.mkdir(parents=True, exist_ok=True)
        encoded = (json.dumps(self._summary, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
        if len(encoded) > MAX_RECORD_BYTES:
            raise ValueError(f"mission.json exceeds the {MAX_RECORD_BYTES}-byte record limit")
        fd, temp_name = tempfile.mkstemp(prefix=".mission-", suffix=".tmp", dir=self.mission_dir)
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, self.summary_path)
            try:
                dir_fd = os.open(self.mission_dir, os.O_RDONLY)
                try:
                    os.fsync(dir_fd)
                finally:
                    os.close(dir_fd)
            except OSError:
                pass
            try:
                self._write_markdown_locked()
            except OSError:
                LOGGER.exception("failed to refresh report.md; mission.json remains authoritative")
            self._revision += 1
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)

    @staticmethod
    def _markdown_cell(value: Any) -> str:
        return str(value if value is not None else "").replace("|", "\\|").replace("\r", " ").replace("\n", " ")

    @staticmethod
    def _report_time(value: Any, *, pending: str) -> str:
        if not value:
            return pending
        return str(value).replace("T", " ", 1)

    @staticmethod
    def _report_task_status(summary: Mapping[str, Any]) -> str:
        if not summary.get("ended_at"):
            return "进行中"
        result = str(summary.get("result", "")).lower()
        return {"completed": "已完成", "failed": "失败", "aborted": "中止"}.get(result, "已结束")

    @staticmethod
    def _report_area_status(step: Mapping[str, Any]) -> str:
        result = str(step.get("result", "")).lower()
        if result in {"completed", "no_target"}:
            return "已完成"
        if result in {"failed", "partial", "skipped"}:
            return "失败（已跳过）" if result == "skipped" else "失败"
        status = str(step.get("status", "")).lower()
        if status == "suspended_for_disposal":
            return "进行中（等待倒垃圾后恢复）"
        if status in {"failed", "navigation_failed"}:
            return "失败"
        if status in {"pending", "navigating", "in_progress", "ready_for_cleaning", "awaiting_confirmation"}:
            return "待处理"
        return "待确认"

    @staticmethod
    def _report_failure_stage(value: Any) -> str:
        stage = str(value or "").strip()
        labels = {
            "basket_pickup": "前台取篮",
            "NAVIGATING_TO_PICKUP": "前往前台",
            "area": "清理区域导航",
            "navigation": "前往清理区域",
            "NAVIGATING_TO_AREA": "前往清理区域",
            "CAPTURING_BEFORE": "清理前拍照",
            "CAPTURING_AFTER": "清理后拍照",
            "cleaning": "遥操作业/夹取",
            "WAITING_FOR_CLEANING": "遥操作业/夹取",
            "disposal": "垃圾桶倾倒",
            "dumping": "垃圾桶倾倒",
            "NAVIGATING_TO_BIN": "前往垃圾桶",
            "DUMPING": "垃圾桶作业",
            "return_to_dock": "返回充电桩",
            "RETURNING_TO_DOCK": "返回充电桩",
            "charging": "充电确认",
        }
        return labels.get(stage, stage or "未注明")

    def _write_markdown_locked(self) -> None:
        summary = self._summary

        def cell(value: Any) -> str:
            return self._markdown_cell(value)

        def image_for(photos: list[Mapping[str, Any]], stage: str, label: str, *, first: bool) -> str:
            entries = [photo for photo in photos if photo.get("stage") == stage]
            captured = [photo for photo in entries if photo.get("status") == "captured" and photo.get("path")]
            photo = (captured[0] if first else captured[-1]) if captured else None
            if photo:
                return f"![{label}]({cell(photo['path'])})"
            failed = next((photo for photo in reversed(entries) if photo.get("status") == "failed"), None)
            if failed:
                detail = failed.get("error") or failed.get("reason")
                return "拍摄失败" + (f"：{cell(detail)}" if detail else "")
            return "未拍摄"

        def item_text(step: Mapping[str, Any]) -> str:
            items = step.get("cleaned_items") or []
            if str(step.get("result", "")).lower() == "no_target":
                return "无目标物"
            if not items:
                return "未记录"
            return "、".join(
                f"{cell(ITEM_LABELS.get(str(item.get('label')), item.get('label', '未分类')))} × "
                f"{cell(item.get('count', 1))}"
                for item in items
            )

        failures = summary.get("failures", [])
        area_steps: dict[str, list[dict[str, Any]]] = {}
        for step in summary.get("steps", []):
            if step.get("kind") == "cleaning" and step.get("area_id"):
                area_steps.setdefault(str(step["area_id"]), []).append(step)

        lines = [
            "# 酒店公区巡查记录", "",
            "| 项目 | 记录 |", "| --- | --- |",
            f"| 任务 ID | {cell(summary.get('task_id'))} |",
            f"| 机器人编号 | {cell(summary.get('robot_id'))} |",
            f"| 开始时间 | {cell(self._report_time(summary.get('started_at'), pending='未记录'))} |",
            f"| 结束时间 | {cell(self._report_time(summary.get('ended_at'), pending='未结束'))} |",
            f"| 任务状态 | {cell(self._report_task_status(summary))} |",
        ]
        mission_failures = [failure for failure in failures if not failure.get("area_id")]
        mission_failure_text = "；".join(
            f"{cell(self._report_failure_stage(failure.get('stage') or failure.get('phase')))}"
            + (f"：{cell(failure.get('detail'))}" if failure.get("detail") else "")
            for failure in mission_failures
        ) or "无"
        lines.extend([f"| 任务级失败环节（子任务） | {mission_failure_text} |", "", "## 清理区域", ""])

        for area_id, visits in area_steps.items():
            latest_result = next((step for step in reversed(visits) if step.get("result") is not None), visits[-1])
            display_name = latest_result.get("display_name") or latest_result.get("location") or area_id
            photos = [photo for step in visits for photo in step.get("photos", [])]
            visit_ids = {str(step.get("visit_id", "")) for step in visits}
            related_failures = [failure for failure in failures
                                if str(failure.get("area_id", "")) == area_id
                                and (not failure.get("visit_id") or str(failure.get("visit_id")) in visit_ids)]
            stage_values = []
            for failure in related_failures:
                stage = self._report_failure_stage(failure.get("stage") or failure.get("phase"))
                detail = failure.get("detail")
                label = f"{stage}（{cell(detail)}）" if detail else stage
                if label not in stage_values:
                    stage_values.append(label)
            for step in visits:
                stage = step.get("failure_stage")
                if stage:
                    label = self._report_failure_stage(stage)
                    if all(value.split("（", 1)[0] != label for value in stage_values):
                        stage_values.append(label)
            failure_text = "；".join(stage_values) or "无"
            lines.extend([
                f"### {cell(display_name)}", "",
                f"- 清理前照片：{image_for(photos, 'before', '清理前照片', first=True)}",
                f"- 清理物品：{item_text(latest_result)}",
                f"- 完成情况：{cell(self._report_area_status(latest_result))}",
                f"- 清理后照片：{image_for(photos, 'after', '清理后照片', first=False)}",
                f"- 失败环节（子任务）：{failure_text}", "",
            ])
        if not area_steps:
            lines.extend(["尚未到达清理区域。", ""])

        target = self.mission_dir / "report.md"
        encoded = ("\n".join(lines) + "\n").encode("utf-8")
        fd, temp_name = tempfile.mkstemp(prefix=".report-", suffix=".tmp", dir=self.mission_dir)
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, target)
            try:
                dir_fd = os.open(self.mission_dir, os.O_RDONLY)
                try:
                    os.fsync(dir_fd)
                finally:
                    os.close(dir_fd)
            except OSError:
                pass
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)

    def _find_step_locked(self, *, area_id: str | None = None, visit_id: str | None = None):
        return next((step for step in self._summary.setdefault("steps", [])
                     if step.get("area_id") == area_id and step.get("visit_id") == visit_id), None)

    def _compat_summary_locked(self) -> dict[str, Any]:
        result = deepcopy(self._summary)
        area_steps = [deepcopy(step) for step in self._summary.get("steps", [])
                      if step.get("kind") == "cleaning"]
        images = []
        detections = []
        for step in area_steps:
            recognition_by_stage: dict[str, list] = {}
            for detection in step.get("recognition", []):
                detections.append(deepcopy(detection))
                recognition_by_stage.setdefault(str(detection.get("stage", "unknown")), []).append(deepcopy(detection))
            step["recognition"] = recognition_by_stage
            for photo in step.get("photos", []):
                if photo.get("status") == "captured" and photo.get("path"):
                    images.append(deepcopy(photo))
            for photo in step.get("photos", []):
                if photo.get("stage") == "before":
                    step["before_photo"] = photo.get("path")
                elif photo.get("stage") == "after":
                    step["after_photo"] = photo.get("path")
        result["areas"] = area_steps
        result["artifacts"] = {"images": images, "detections": detections}
        return result

    def upsert_step(self, step_id: str, kind: str, location: str, **fields: Any) -> dict[str, Any]:
        with self._lock:
            self._require_started()
            steps = self._summary.setdefault("steps", [])
            step = next((item for item in steps if item.get("step_id") == str(step_id)), None)
            if step is None:
                step = {"step_id": str(step_id), "kind": str(kind),
                        "location": str(location), "status": "pending"}
                steps.append(step)
            step.update(deepcopy(fields))
            step.pop("before_photo", None)
            step.pop("after_photo", None)
            step.setdefault("kind", str(kind))
            step.setdefault("location", str(location))
            self._write_summary_locked()
            return deepcopy(step)

    def summary(self) -> dict[str, Any]:
        with self._lock:
            return self._compat_summary_locked()

    def update_summary(self, **fields: Any) -> dict[str, Any]:
        with self._lock:
            self._require_started()
            if "areas" in fields or "artifacts" in fields:
                raise ValueError("areas and artifacts are derived from steps and photos")
            self._summary.update(deepcopy(fields))
            self._write_summary_locked()
            return self._compat_summary_locked()

    def append_event(
        self,
        event_type: str,
        *,
        source: str = "system",
        event_id: str | None = None,
        area_id: str | None = None,
        visit_id: str | None = None,
        phase: str | None = None,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            self._require_started()
            resolved_id = str(event_id or uuid4())
            for existing in self._summary["events"]:
                if existing.get("event_id") == resolved_id:
                    return {**deepcopy(existing), "duplicate": True}
            if len(self._summary["events"]) >= MAX_EVENTS:
                raise ValueError("mission event limit reached")
            event = {"event_id": resolved_id, "task_id": self.task_id,
                     "event_type": str(event_type), "source": str(source),
                     "occurred_at": local_iso_now(), "monotonic_ns": time.monotonic_ns()}
            if area_id is not None:
                event["area_id"] = str(area_id)
            if visit_id is not None:
                event["visit_id"] = str(visit_id)
            if phase is not None:
                event["phase"] = str(phase)
            if payload:
                event["payload"] = deepcopy(dict(payload))
            self._summary["events"].append(event)
            self._write_summary_locked()
            return deepcopy(event)

    def operation(self, operation_id: str, signature: str | None = None) -> dict[str, Any] | None:
        with self._lock:
            item = next((x for x in self._summary.get("processed_operations", [])
                         if x.get("operation_id") == str(operation_id)), None)
            if item and signature is not None and item.get("signature") != signature:
                raise ValueError("operation_id was already used with different parameters")
            return deepcopy(item) if item else None

    def begin_operation(self, operation_id: str, signature: str) -> dict[str, Any]:
        with self._lock:
            prior = self.operation(operation_id, signature)
            if prior:
                return prior
            operations = self._summary.setdefault("processed_operations", [])
            if len(operations) >= MAX_OPERATIONS:
                raise ValueError("mission operation limit reached")
            item = {"operation_id": str(operation_id), "signature": str(signature),
                    "status": "pending", "started_at": local_iso_now()}
            operations.append(item)
            self._write_summary_locked()
            return deepcopy(item)

    def complete_operation(self, operation_id: str, outcome: str = "applied") -> None:
        with self._lock:
            item = next((x for x in self._summary["processed_operations"]
                         if x.get("operation_id") == str(operation_id)), None)
            if item is None:
                return
            item.update(status="completed", outcome=str(outcome), completed_at=local_iso_now())
            self._write_summary_locked()

    def mark_operation_uncertain(self, operation_id: str, detail: str) -> None:
        with self._lock:
            item = next((x for x in self._summary["processed_operations"]
                         if x.get("operation_id") == str(operation_id)), None)
            if item is None:
                return
            item.update(status="uncertain", detail=str(detail), updated_at=local_iso_now())
            self._summary["review_required"] = True
            self._write_summary_locked()

    def release_operation(self, operation_id: str) -> None:
        with self._lock:
            before = len(self._summary["processed_operations"])
            self._summary["processed_operations"] = [x for x in self._summary["processed_operations"]
                if x.get("operation_id") != str(operation_id)]
            if len(self._summary["processed_operations"]) != before:
                self._write_summary_locked()

    def has_event_id(self, event_id: str) -> bool:
        with self._lock:
            return any(x.get("event_id") == str(event_id) for x in self._summary.get("events", []))

    def upsert_area(self, area_id: str, visit_id: str, **fields: Any) -> dict[str, Any]:
        with self._lock:
            self._require_started()
            step = self._find_step_locked(area_id=str(area_id), visit_id=str(visit_id))
            if step is None:
                step = {"step_id": str(visit_id), "kind": "cleaning", "area_id": str(area_id),
                        "visit_id": str(visit_id), "status": "pending", "cleaned_items": [],
                        "photos": [], "recognition": []}
                self._summary["steps"].append(step)
            fields = deepcopy(fields)
            fields.pop("before_photo", None)
            fields.pop("after_photo", None)
            step.update(fields)
            step.setdefault("kind", "cleaning")
            step.setdefault("photos", [])
            step.setdefault("recognition", [])
            self._summary["current_area_id"] = str(area_id)
            self._summary["current_visit_id"] = str(visit_id)
            self._write_summary_locked()
            return deepcopy(step)

    def add_detour(self, detour: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._require_started()
            item = deepcopy(dict(detour))
            self._summary.setdefault("detours", []).append(item)
            self._write_summary_locked()
            return item

    def add_failure(self, failure: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._require_started()
            item = deepcopy(dict(failure))
            self._summary.setdefault("failures", []).append(item)
            self._summary["review_required"] = True
            self._write_summary_locked()
            return item

    def _write_binary_artifact(self, relative_path: str, data: bytes) -> str:
        relative = Path(relative_path)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("artifact path must stay inside the mission directory")
        target = self.mission_dir / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        reserved = False
        try:
            with target.open("xb"):
                reserved = True
        except FileExistsError:
            raise
        fd, temp_name = tempfile.mkstemp(prefix="." + target.name + "-", suffix=".tmp", dir=target.parent)
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(bytes(data))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, target)
            return relative.as_posix()
        except Exception:
            if reserved:
                try:
                    target.unlink()
                except OSError:
                    pass
            raise
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)

    def _area_display_name_locked(self, area_id: str) -> str:
        for step in self._summary.get("steps", []):
            if step.get("area_id") == area_id:
                return str(step.get("display_name") or area_id)
        return area_id

    def _next_photo_path_locked(self, area_id: str, visit_id: str, stage: str, extension: str) -> str:
        area = safe_path_token(self._area_display_name_locked(area_id), "area")
        match = re.search(r"(\d+)$", str(visit_id))
        visit = match.group(1) if match else safe_path_token(visit_id, "001")
        prefix = f"photos/{area}_{visit}"
        if stage == "supplemental":
            index = 1
            while True:
                candidate = f"{prefix}_supplemental_{index:03d}{extension}"
                if not (self.mission_dir / candidate).exists():
                    return candidate
                index += 1
        base = f"{prefix}_{safe_path_token(stage, 'photo')}"
        candidate = base + extension
        index = 1
        while (self.mission_dir / candidate).exists():
            candidate = f"{base}_{index:03d}{extension}"
            index += 1
        return candidate

    def save_photo_bytes(
        self,
        data: bytes,
        *,
        area_id: str,
        visit_id: str,
        stage: str,
        reason: str | None = None,
        metadata: Mapping[str, Any] | None = None,
        extension: str = ".jpg",
    ) -> str:
        with self._lock:
            self._require_started()
            if sum(len(step.get("photos", [])) for step in self._summary["steps"]) >= MAX_PHOTOS:
                raise ValueError("mission photo limit reached")
            relative = self._next_photo_path_locked(str(area_id), str(visit_id), str(stage), extension)
            self._write_binary_artifact(relative, data)
            photo = {"path": relative, "area_id": str(area_id), "visit_id": str(visit_id),
                     "stage": str(stage), "status": "captured", "captured_at": local_iso_now()}
            if reason:
                photo["reason"] = str(reason)
            if metadata:
                photo.update(deepcopy(dict(metadata)))
            step = self._find_step_locked(area_id=str(area_id), visit_id=str(visit_id))
            if step is None:
                step = self.upsert_area(str(area_id), str(visit_id), status="in_progress")
                step = self._find_step_locked(area_id=str(area_id), visit_id=str(visit_id))
            step.setdefault("photos", []).append(photo)
            self._write_summary_locked()
            self.append_event("photo_captured", area_id=area_id, visit_id=visit_id,
                              phase=stage, payload=photo)
            return relative

    def record_photo_failure(self, area_id: str, visit_id: str, stage: str, error: str,
                             *, reason: str | None = None) -> None:
        with self._lock:
            step = self._find_step_locked(area_id=str(area_id), visit_id=str(visit_id))
            if step is None:
                step = self.upsert_area(str(area_id), str(visit_id), status="in_progress")
                step = self._find_step_locked(area_id=str(area_id), visit_id=str(visit_id))
            photo = {"stage": str(stage), "status": "failed", "error": str(error),
                     "captured_at": local_iso_now()}
            if reason:
                photo["reason"] = str(reason)
            step.setdefault("photos", []).append(photo)
            self._summary["data_quality"] = "incomplete"
            self._summary["review_required"] = True
            self._write_summary_locked()

    def save_detection_image(self, area_id: str, visit_id: str, stage: str, data: bytes) -> str:
        with self._lock:
            area = safe_path_token(self._area_display_name_locked(str(area_id)), "area")
            match = re.search(r"(\d+)$", str(visit_id))
            visit = match.group(1) if match else safe_path_token(visit_id, "001")
            base = f"photos/detections/{area}_{visit}_{safe_path_token(stage, 'stage')}"
            candidate = base + ".jpg"
            index = 1
            while (self.mission_dir / candidate).exists():
                candidate = f"{base}_{index:03d}.jpg"
                index += 1
            return self._write_binary_artifact(candidate, data)

    def record_detection(self, _legacy_path: str | None, result: Mapping[str, Any]) -> None:
        self.record_recognition_status(result)

    def record_recognition_status(self, result: Mapping[str, Any]) -> None:
        """Persist a recognition state without creating a sidecar result file."""
        with self._lock:
            self._require_started()
            value = deepcopy(dict(result))
            area_id = str(value.get("area_id", ""))
            visit_id = str(value.get("visit_id", ""))
            step = self._find_step_locked(area_id=area_id, visit_id=visit_id)
            if step is None:
                step = self.upsert_area(area_id, visit_id, status="in_progress")
                step = self._find_step_locked(area_id=area_id, visit_id=visit_id)
            recognitions = step.setdefault("recognition", [])
            request_id = str(value.get("request_id", ""))
            prior = next((item for item in recognitions
                          if request_id and item.get("request_id") == request_id), None)
            if prior is None:
                recognitions.append(value)
            else:
                prior.update(value)
            for photo in step.get("photos", []):
                if photo.get("path") == value.get("image_path"):
                    photo.setdefault("recognition", [])
                    linked = next((item for item in photo["recognition"]
                                   if request_id and item.get("request_id") == request_id), None)
                    if linked is None:
                        photo["recognition"].append(deepcopy(value))
                    else:
                        linked.update(deepcopy(value))
            if value.get("status") not in {"completed", "no_objects"} or value.get("view_quality") in {"unknown", "blocked", "poor"}:
                self._summary["review_required"] = True
            self._write_summary_locked()

    def mark_step_photo(self, area_id: str, visit_id: str, photo: Mapping[str, Any]) -> None:
        with self._lock:
            step = self._find_step_locked(area_id=str(area_id), visit_id=str(visit_id))
            if step is None:
                raise KeyError(f"unknown cleaning visit: {visit_id}")
            for existing in step.get("photos", []):
                if existing.get("path") == photo.get("path"):
                    existing.update(deepcopy(dict(photo)))
                    self._write_summary_locked()
                    return
            raise KeyError(f"unknown photo path: {photo.get('path')}")

    def finish(
        self,
        *,
        result: str,
        execution_status: str = "finished",
        data_quality: str = "complete",
        review_required: bool | None = None,
        failure_reason: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            self._require_started()
            if self._summary.get("ended_at") is not None:
                return self._compat_summary_locked()
            self._summary["ended_at"] = local_iso_now()
            self._summary.update({
                "status": str(execution_status),
                "execution_status": str(execution_status),
                "result": str(result),
                "data_quality": "incomplete" if self._summary.get("data_quality") == "incomplete" else str(data_quality),
                "current_state": "FINISHED" if execution_status == "finished" else str(execution_status).upper(),
            })
            if review_required is not None:
                self._summary["review_required"] = bool(review_required or self._summary.get("review_required"))
            if failure_reason:
                self._summary.setdefault("failures", []).append(deepcopy(dict(failure_reason)))
                self._summary["review_required"] = True
            self._write_summary_locked()
            self.append_event("mission_finished", payload={"result": result,
                              "execution_status": execution_status,
                              "data_quality": self._summary["data_quality"]})
            return self._compat_summary_locked()

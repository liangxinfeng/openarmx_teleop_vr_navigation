"""ROS-independent, serialized hotel mission service and camera evidence adapter."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time
from uuid import uuid4

from .hotel_mission import HotelMissionController, MissionError
from .mission_recorder import MissionRecorder
from .recognition_worker import RecognitionWorker
from .hotel_config import hotel_catalog


TERMINAL = {"FINISHED", "FAILED", "ABORTED"}


class CameraEvidence:
    """Request a new raw camera JPEG, rejecting cached or stale frames."""

    def __init__(self, camera, telemetry=lambda: {}, timeout=1.0):
        self.camera = camera
        self.telemetry = telemetry
        self.timeout = timeout

    def __call__(self, stage, context):
        if self.camera is None:
            raise RuntimeError("head RGB camera is unavailable")
        before = self.camera.status()
        after_id = int(before.get("frame_version", before.get("frame_id", -1)))
        jpeg, frame_id, status = self.camera.snapshot(after_id, self.timeout)
        age = status.get("frame_age", status.get("age_seconds"))
        if (not jpeg or frame_id <= after_id or not status.get("online")
                or (age is not None and float(age) >= 1.0)):
            raise RuntimeError("no fresh RGB frame arrived before the capture deadline")
        return {"data": bytes(jpeg), "metadata": {
            "camera": "rgbd_head_color", "frame_id": frame_id,
            "camera_status": deepcopy(status),
            "robot_status": deepcopy(self.telemetry()),
        }}


class HotelMissionService:
    """Own one mission, navigation effects, recognition jobs and request dedup.

    All public calls are serialized. The bridge runs them in a dedicated worker,
    never on the ROS executor or the WebXR network/event loop.
    """

    def __init__(self, config_path, robot_id, capture_photo=None, recognizer=None,
                 task_catalog_path=None, task_catalog_provider=None):
        self.config_path = Path(config_path)
        self.task_catalog_path = task_catalog_path
        # Optional callable returning the bridge's accepted catalog snapshot
        # ({'catalog', 'bytes', 'path', ...}). When present the hotel recorder
        # never re-reads task_actions.txt on its own, so the task menu and the
        # recording flow always agree on the same accepted revision.
        self.task_catalog_provider = task_catalog_provider
        self.robot_id = str(robot_id)
        self.capture_photo = capture_photo
        self.recognizer = recognizer
        self.controller = None
        self.worker = None
        self._lock = threading.RLock()
        self._legs = deque()
        self._revision = 0
        self._operations = {}
        self._last_head = None
        self._head_stable_at = 0.0
        self._head_capture_at = 0.0
        self._supplemental_count = {}

    @property
    def active(self):
        with self._lock:
            return self.controller is not None and self.controller.state not in TERMINAL

    def catalog(self):
        if self.task_catalog_provider is not None:
            snapshot = self.task_catalog_provider() or {}
            catalog = snapshot.get('catalog')
            if catalog is not None:
                return hotel_catalog(
                    self.config_path,
                    snapshot.get('path') or self.task_catalog_path,
                    task_catalog=catalog,
                    catalog_bytes=snapshot.get('bytes'),
                )
        return hotel_catalog(self.config_path, self.task_catalog_path)

    def chain_names(self):
        payload = json.loads(self.config_path.read_text(encoding='utf-8-sig'))
        return [x['source_chain'] for x in payload.get('missions', []) if x.get('source_chain')]

    def snapshot(self):
        with self._lock:
            if self.controller is None:
                return {"active": False, "state": "IDLE", "token": ""}
            result = self.controller.snapshot()
            result.update({
                "active": self.active,
                "token": f"{self.controller.task_id}:{self._revision}",
                "record_directory": str(self.controller.recorder.mission_dir),
                "record_committed": self.controller.recorder.committed,
            })
            return result

    def start(self, mission_id, waypoints, catalog_token, *, start_at_dock_confirmed=False,
              defer_record=False):
        with self._lock:
            if self.active:
                raise MissionError("当前酒店任务尚未结束")
            catalog = self.catalog()
            if catalog_token != catalog["token"]:
                raise MissionError("任务配置已改变，请重新打开酒店任务菜单")
            config = next((x for x in catalog["missions"] if x["id"] == mission_id), None)
            if config is None:
                raise ValueError("未知酒店任务模板")
            if (config.get('require_start_confirmation')
                    and not config.get('generic_route')
                    and start_at_dock_confirmed is not True):
                raise MissionError('请确认机器人位于充电桩起点，再开始前台取篮任务')
            required = {x["waypoint"] for x in config["areas"]}
            if not config.get('generic_route'):
                required.update([config["start_waypoint"], config["return_waypoint"],
                                 config["trash_bin"]["waypoint"]])
                if config.get('pickup'):
                    required.add(config['pickup']['waypoint'])
            missing = sorted(required - set(waypoints))
            if missing:
                raise ValueError("地图缺少航点：" + "、".join(missing))
            from .recognition_provider import configured_recognizer
            recognizer = self.recognizer or configured_recognizer(catalog["recognition"])
            if self.worker is not None:
                self.worker.close()
            task_id = datetime.now().strftime("hotel_%Y%m%d_%H%M%S_") + uuid4().hex[:8]
            recorder = MissionRecorder(catalog["record_root"], task_id=task_id,
                                       robot_id=self.robot_id, task_type=config.get("task_type", "hotel_public_garbage_patrol"),
                                       metadata={"config": config, "config_token": catalog["token"],
                                           'start_verification': {'waypoint': config['start_waypoint'],
                                               'source': 'operator_confirmed' if start_at_dock_confirmed else 'not_verified'}},
                                       defer_persistence=defer_record)
            self.controller = HotelMissionController(config, recorder, navigate=self._navigate,
                                                     capture_photo=self.capture_photo)
            self._legs.clear()
            self._operations.clear()
            self._supplemental_count.clear()
            self._revision += 1
            self._last_head = None
            try:
                self.controller.start()
            except Exception:
                self.controller = None
                self._legs.clear()
                raise
            self.worker = RecognitionWorker(recorder, recognizer)
            return self.snapshot()

    def commit_start_record(self):
        with self._lock:
            if self.controller is None:
                raise MissionError("当前没有待提交的酒店任务记录")
            return self.controller.recorder.commit()

    def discard_uncommitted_start(self):
        """Discard an unaccepted hotel launch without leaving an empty record."""
        with self._lock:
            if self.controller is None or self.controller.recorder.committed:
                return False
            if self.worker is not None:
                self.worker.close()
            self.worker = None
            self.controller = None
            self._legs.clear()
            self._operations.clear()
            self._last_head = None
            self._supplemental_count.clear()
            return True

    def _navigate(self, waypoint, context):
        self._legs.append({"waypoint": waypoint, **dict(context)})

    def pop_leg(self):
        with self._lock:
            return self._legs.popleft() if self._legs else None

    def _queue_new_photos(self, old_count):
        recorder = self.controller.recorder
        for photo in recorder.summary()["artifacts"]["images"][old_count:]:
            self.worker.submit(recorder.mission_dir / photo["path"],
                               area_id=photo["area_id"], visit_id=photo["visit_id"],
                               stage=photo["stage"], metadata=photo)

    @staticmethod
    def _request_signature(action, payload):
        fields = {k: v for k, v in payload.items() if k not in {'token', 'client_id'}}
        return json.dumps([action, fields], sort_keys=True, ensure_ascii=False)

    def validate_request(self, action, payload):
        """Validate before the bridge calls any navigation service."""
        with self._lock:
            operation = str(payload.get('operation_id') or '')
            if not operation or len(operation) > 128:
                raise MissionError('操作缺少有效编号，请刷新头显页面')
            signature = self._request_signature(action, payload)
            persisted = self.controller.recorder.operation(operation, signature)
            if persisted:
                if persisted.get('status') == 'pending':
                    self.controller.recorder.mark_operation_uncertain(
                        operation, 'request interrupted after acceptance; inspect current mission state before retrying')
                    raise MissionError('该操作曾开始但未确认完成，请检查当前任务状态后再操作')
                return True
            if operation in self._operations:
                if self._operations[operation] != signature:
                    raise MissionError('同一个操作编号不可用于不同命令或参数')
                return True
            if payload.get('token') != self.snapshot()['token']:
                raise MissionError('任务状态已改变，请重新确认')
            if not self.active:
                raise MissionError('酒店任务已经结束')
            return False

    def apply(self, action, payload=None, *, internal=False):
        with self._lock:
            payload = dict(payload or {})
            controller = self.controller
            if controller is None:
                raise MissionError("当前没有酒店任务")
            operation = str(payload.get("operation_id", ""))
            if not internal and self.validate_request(action, payload):
                return {**self.snapshot(), "duplicate": True}
            if not self.active:
                raise MissionError("酒店任务已经结束")
            old_count = len(controller.recorder.summary()["artifacts"]["images"])
            tracked_operation = not internal and bool(operation)
            signature = self._request_signature(action, payload) if tracked_operation else None
            if tracked_operation:
                controller.recorder.begin_operation(operation, signature)
            operation_revision = controller.recorder.revision
            try:
                self._apply_action(action, payload, controller, operation, internal)
                self._queue_new_photos(old_count)
                if action != "photo":
                    self._revision += 1
                if tracked_operation:
                    self._operations[operation] = signature
                    controller.recorder.complete_operation(operation)
                return self.snapshot()
            except Exception as error:
                if tracked_operation:
                    if controller.recorder.revision == operation_revision:
                        controller.recorder.release_operation(operation)
                    else:
                        controller.recorder.mark_operation_uncertain(operation, str(error))
                raise

    def _apply_action(self, action, payload, controller, operation, internal):
            if action in {'arrived', 'dock_arrived'} and controller.state == 'PAUSED' and payload.get('operator') is True:
                failed_stage = (controller.snapshot().get('last_failure') or {}).get('stage')
                if failed_stage not in {'NAVIGATING_TO_PICKUP', 'NAVIGATING_TO_AREA', 'NAVIGATING_TO_BIN', 'RETURNING_TO_DOCK'}:
                    raise MissionError('当前失败不支持人工到达确认')
                controller._set_state(failed_stage, payload={'operator_arrived': True})
                controller.recorder.append_event('operator_arrival_confirmed', source='operator',
                    area_id=controller.current_area_id, visit_id=controller.current_visit_id,
                    phase='navigation', payload={'waypoint': payload.get('waypoint')})
            if action == "arrived":
                controller.on_navigation_arrived(payload.get("waypoint"))
            elif action == "navigation_failed":
                controller.on_navigation_failed(payload.get("code", "NAVIGATION_FAILED"), payload.get("detail", "导航失败"))
            elif action == "retry":
                controller.retry_navigation()
            elif action == 'navigation_resumed':
                controller.navigation_resumed_externally(payload.get('waypoint'))
            elif action == "complete":
                if controller.state == 'PICKING_BASKET':
                    controller.confirm_pickup(operation_id=operation)
                elif controller.state == "DUMPING":
                    controller.confirm_disposal(success=True, operation_id=operation)
                else:
                    result = payload.get("result", "completed")
                    if result not in {"completed", "no_target", "partial", "failed", "skipped"}:
                        raise ValueError("无效的桌面完成情况")
                    controller.confirm_area(result=result, note=payload.get("note"),
                                            cleaned_items=payload.get("cleaned_items"),
                                            operation_id=operation)
            elif action == "mark_item":
                if controller.state not in {"WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
                    raise MissionError("只能在桌面作业阶段标记清理物品")
                controller.mark_cleaned_item(payload.get("label"), operation_id=operation)
            elif action == "basket_full":
                if controller.state not in {"WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
                    raise MissionError("只能在桌面作业阶段报告篮子已满")
                controller.set_basket_state("full")
            elif action == "dispose":
                controller.request_disposal(arms_secured=payload.get("arms_secured") is True,
                                            operation_id=operation)
            elif action == "dump_failed":
                controller.confirm_disposal(success=False, note=payload.get("note"), operation_id=operation)
            elif action == "retry_dump":
                controller.retry_disposal()
            elif action == 'pickup_failed':
                controller.confirm_pickup(success=False, operation_id=operation, note=payload.get('note'))
            elif action == 'retry_pickup':
                controller.retry_pickup()
            elif action == "skip_area":
                controller.skip_area(reason=payload.get("reason", "operator_skipped"))
            elif action == "photo":
                controller.capture_supplemental_photo(payload.get("reason", "operator_supplemental"))
            elif action == 'manual_ready':
                if controller.state not in {'PICKING_BASKET', 'WAITING_FOR_CLEANING', 'WAITING_AREA_CONFIRM', 'DUMPING'}:
                    raise MissionError('manual pose confirmation requires an arrived action leg')
                controller.recorder.append_event('operator_pose_confirmed', source='operator',
                    event_id=operation, area_id=controller.current_area_id, visit_id=controller.current_visit_id,
                    phase='basket_pickup' if controller.state == 'PICKING_BASKET' else ('disposal' if controller.state == 'DUMPING' else 'cleaning'),
                    payload={'preset_reached': False})
                if controller.current_area_id and controller.state != 'DUMPING':
                    controller.recorder.upsert_area(controller.current_area_id, controller.current_visit_id,
                                                   pose_confirmation='operator')
            elif action == "exception":
                if controller.state not in {'WAITING_FOR_CLEANING', 'WAITING_AREA_CONFIRM'}:
                    raise MissionError('请在当前桌面作业阶段记录操作异常')
                code = str(payload.get("code", "")).strip()
                if code not in {"GRASP_FAILED", "OBJECT_DROPPED", "SCENE_CHANGED", "VIEW_BLOCKED", "OTHER"}:
                    raise ValueError("请选择有效异常原因")
                controller.record_exception(code, phase="cleaning", detail=payload.get("detail", ""),
                                            operation_id=operation)
            elif action == "dock_arrived":
                controller.on_dock_arrived(payload.get("charging_confirmed"))
            elif action == "abort":
                controller.abort(payload.get("reason", "operator_abort"))
                self._legs.clear()
            else:
                raise ValueError("未知酒店任务操作")

    def observe_head(self, status, now=None):
        """Capture only after manual head following settles; bounded per visit."""
        with self._lock:
            if not self.active or self.controller.state not in {"WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
                self._last_head = None
                return
            backend = status.get("backend") or {}
            if not backend.get("head_follow_enabled"):
                self._last_head = None
                return
            try:
                head = tuple(float(x) for x in backend["measured_neck_deg"])
                if not head or not all(math.isfinite(x) for x in head):
                    return
            except (KeyError, TypeError, ValueError):
                return
            now = time.monotonic() if now is None else now
            if self._last_head is None or len(head) != len(self._last_head) or max(abs(a-b) for a,b in zip(head,self._last_head)) > 2.0:
                self._last_head = head
                self._head_stable_at = now
                return
            visit = self.controller.current_visit_id
            if now - self._head_stable_at < 0.8 or now - self._head_capture_at < 5.0 or self._supplemental_count.get(visit, 0) >= 6:
                return
            self._head_capture_at = now
            self._supplemental_count[visit] = self._supplemental_count.get(visit, 0) + 1
            self.apply("photo", {"reason": "manual_head_stable"}, internal=True)

    def close(self):
        with self._lock:
            if self.active:
                self.apply("abort", {"reason": "bridge_shutdown"}, internal=True)
            if self.worker:
                self.worker.close()

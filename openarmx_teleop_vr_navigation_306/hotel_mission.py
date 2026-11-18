"""Hotel public-area cleanup mission orchestration.

This module owns mission semantics while the existing TaskWorkflow remains the
executor for one navigation/action leg.  It is intentionally ROS-free so the
dynamic detour and recording rules can be verified before deployment.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Mapping

from .mission_recorder import MissionRecorder, safe_path_token
from .simple_task_catalog import task_settings


MISSION_STATES = {
    "IDLE",
    "DISPATCHED",
    "NAVIGATING_TO_PICKUP",
    "PICKING_BASKET",
    "PICKUP_FAILED",
    "NAVIGATING_TO_AREA",
    "CAPTURING_BEFORE",
    "WAITING_FOR_CLEANING",
    "CAPTURING_AFTER",
    "WAITING_AREA_CONFIRM",
    "PAUSED_FOR_DISPOSAL",
    "PAUSED",
    "NAVIGATING_TO_BIN",
    "DUMPING",
    "RETURNING_TO_DOCK",
    "FINISHED",
    "FAILED",
    "ABORTED",
}


class MissionError(RuntimeError):
    """Invalid mission operation for the current state."""


class SafetyCheckpointRequired(MissionError):
    """A detour was requested before the arms/basket were secured."""


# Matches the recognition categories; a template may override per area via
# ``allowed_items``.  Missing allow-lists stay permissive for legacy configs.
DEFAULT_ALLOWED_ITEMS = ("water_bottle", "water_cup", "tissue", "paper", "other")


def validate_mission_config(config: Mapping[str, Any]) -> dict[str, Any]:
    """Validate and copy a hotel mission template."""

    result = deepcopy(dict(config))
    mission_id = str(result.get("id", "")).strip()
    generic_route = bool(result.get('generic_route'))
    areas = result.get("areas") or result.get('route')
    trash_bin = result.get("trash_bin") or {}
    if not mission_id:
        raise ValueError("mission config requires id")
    if not isinstance(areas, list) or not areas:
        raise ValueError("mission config requires at least one area")
    if (not generic_route and
            (not isinstance(trash_bin, Mapping) or not str(trash_bin.get("waypoint", "")).strip())):
        raise ValueError("mission config requires trash_bin.waypoint")
    area_ids: set[str] = set()
    for index, area in enumerate(areas, 1):
        if not isinstance(area, Mapping):
            raise ValueError(f"area {index} must be an object")
        area_id = str(area.get("id", "")).strip()
        waypoint = str(area.get("waypoint", "")).strip()
        if not area_id or not waypoint:
            raise ValueError(f"area {index} requires id and waypoint")
        if area_id in area_ids:
            raise ValueError(f"duplicate area id: {area_id}")
        area_ids.add(area_id)
        area.update(task_settings(area))
    start = str(result.get("start_waypoint", "")).strip()
    return_waypoint = str(result.get("return_waypoint", start)).strip()
    if generic_route:
        start = start or str(areas[0].get('waypoint', '')).strip()
        return_waypoint = return_waypoint or str(areas[-1].get('waypoint', start)).strip()
    if not start or not return_waypoint:
        raise ValueError("mission config requires start_waypoint and return_waypoint")
    result["start_waypoint"] = start
    result["return_waypoint"] = return_waypoint
    result["trash_bin"] = dict(trash_bin)
    if result['trash_bin'].get('waypoint'):
        result['trash_bin'].update(task_settings(trash_bin))
    result['dock_pose'] = task_settings(result.get('dock_pose') or {})
    result["basket"] = dict(result.get("basket") or {})
    if result.get('pickup'):
        pickup = dict(result['pickup'])
        if not str(pickup.get('waypoint', '')).strip():
            raise ValueError('pickup requires waypoint')
        pickup.update(task_settings(pickup))
        result['pickup'] = pickup
    return result


def load_mission_config(path: str | Path, mission_id: str) -> dict[str, Any]:
    import json

    source = Path(path).expanduser()
    payload = json.loads(source.read_text(encoding="utf-8"))
    missions = payload.get("missions", [])
    selected = next(
        (item for item in missions if str(item.get("id", "")) == str(mission_id)),
        None,
    )
    if selected is None:
        raise ValueError(f"mission template not found: {mission_id}")
    return validate_mission_config(selected)


CaptureCallback = Callable[[str, Mapping[str, Any]], Any]
NavigateCallback = Callable[[str, Mapping[str, Any]], Any]


class HotelMissionController:
    """Stateful controller for one public-area cleanup mission."""

    def __init__(
        self,
        config: Mapping[str, Any],
        recorder: MissionRecorder,
        *,
        navigate: NavigateCallback | None = None,
        capture_photo: CaptureCallback | None = None,
    ) -> None:
        self.config = validate_mission_config(config)
        self.recorder = recorder
        self.navigate = navigate or (lambda _waypoint, _context: None)
        self.capture_photo = capture_photo
        self.state = "IDLE"
        self.area_index = -1
        self.current_area_id: str | None = None
        self.current_visit_id: str | None = None
        self.current_area_finished = False
        self.current_area_result: str | None = None
        self.basket_state = "empty"
        self._visit_counts: dict[str, int] = {}
        self._pending_resume_index: int | None = None
        self._pending_resume_of: str | None = None
        self._pending_disposal_kind: str | None = None
        self._handled_operations: set[str] = set()
        self._marked_items: dict[str, int] = {}
        self._has_area_failure = False
        self._unresolved_failure = False
        self._last_failure: dict[str, Any] | None = None
        self._current_disposal_step_id: str | None = None

    @property
    def task_id(self) -> str:
        return self.recorder.task_id

    def start(self) -> dict[str, Any]:
        if self.state != "IDLE":
            raise MissionError("mission has already started")
        self.recorder.start()
        self.recorder.update_summary(
            mission_config_id=self.config["id"],
            start_waypoint=self.config["start_waypoint"],
            return_waypoint=self.config["return_waypoint"],
            trash_bin_waypoint=self.config.get("trash_bin", {}).get("waypoint"),
            scheduled_final_disposal=bool(self.config.get('scheduled_final_disposal')),
            generic_route=bool(self.config.get('generic_route')),
            final_bin_completed=False,
            basket=deepcopy(self.config.get("basket", {})),
        )
        self._set_state("DISPATCHED", payload={"mission_config_id": self.config["id"]})
        if self.config.get('pickup'):
            pickup = self.config['pickup']
            self.recorder.update_summary(pickup={'waypoint': pickup['waypoint'], 'result': 'pending'})
            self.recorder.upsert_step('basket_pickup', 'pickup', pickup['waypoint'],
                                      status='navigating', result='pending')
            self._set_state('NAVIGATING_TO_PICKUP', payload={'waypoint': pickup['waypoint']})
            self.recorder.append_event('navigation_requested', phase='basket_pickup',
                payload={'waypoint': pickup['waypoint'], 'leg_type': 'pickup'})
            self.navigate(pickup['waypoint'], {'leg_type': 'pickup'})
        else:
            self._begin_area(0)
        return self.snapshot()

    def confirm_pickup(self, *, success=True, operation_id=None, note=None):
        if self.state != 'PICKING_BASKET':
            raise MissionError('basket pickup is not awaiting confirmation')
        if self._operation_seen(operation_id):
            return self.snapshot()
        self.recorder.append_event('basket_pickup_confirmed' if success else 'basket_pickup_failed',
            source='operator', event_id=operation_id, phase='basket_pickup',
            payload={'success': success, 'note': note, 'carrier_hand': 'left'})
        if not success:
            failure = {'code': 'BASKET_PICKUP_FAILED', 'stage': 'basket_pickup',
                       'detail': note or 'operator could not secure the basket', 'recoverable': True}
            self._last_failure = failure
            self._unresolved_failure = True
            self.recorder.add_failure(failure)
            self.recorder.update_summary(pickup={'waypoint': self.config['pickup']['waypoint'],
                'result': 'failed', 'result_source': 'operator_confirmed'}, review_required=True)
            self.recorder.upsert_step('basket_pickup', 'pickup', self.config['pickup']['waypoint'],
                                      status='failed', result='failed', failure_stage='basket_pickup')
            self._set_state('PICKUP_FAILED')
        else:
            self._unresolved_failure = False
            self.set_basket_state('empty', reason='operator_pickup_confirmed')
            self.recorder.update_summary(pickup={'waypoint': self.config['pickup']['waypoint'],
                'result': 'completed', 'result_source': 'operator_confirmed',
                'completed_at': datetime.now().astimezone().isoformat(timespec='milliseconds')})
            self.recorder.upsert_step('basket_pickup', 'pickup', self.config['pickup']['waypoint'],
                                      status='completed', result='completed',
                                      completed_at=datetime.now().astimezone().isoformat(timespec='milliseconds'))
            self._begin_area(0)
        return self.snapshot()

    def retry_pickup(self):
        if self.state != 'PICKUP_FAILED':
            raise MissionError('basket pickup has not failed')
        self.recorder.append_event('basket_pickup_retry_requested', source='operator', phase='basket_pickup')
        self._set_state('PICKING_BASKET')
        return self.snapshot()

    def _set_state(self, state: str, *, payload: Mapping[str, Any] | None = None) -> None:
        if state not in MISSION_STATES:
            raise ValueError(f"unknown mission state: {state}")
        self.state = state
        self.recorder.update_summary(
            current_state=state,
            current_area_id=self.current_area_id,
            current_visit_id=self.current_visit_id,
        )
        self.recorder.append_event(
            "mission_state_changed",
            area_id=self.current_area_id,
            visit_id=self.current_visit_id,
            payload={"state": state, **dict(payload or {})},
        )

    def _operation_seen(self, operation_id: str | None) -> bool:
        if not operation_id:
            return False
        if operation_id in self._handled_operations:
            return True
        self._handled_operations.add(operation_id)
        return False

    def _begin_area(self, index: int, *, resume_of: str | None = None) -> None:
        if index < 0 or index >= len(self.config["areas"]):
            self._begin_return()
            return
        area = self.config["areas"][index]
        area_id = str(area["id"])
        self.area_index = index
        self.current_area_id = area_id
        count = self._visit_counts.get(area_id, 0) + 1
        self._visit_counts[area_id] = count
        self.current_visit_id = f"{safe_path_token(area_id)}_visit_{count:03d}"
        self.current_area_finished = False
        self.current_area_result = None
        if resume_of is None:
            # Item marks survive a disposal detour back to the same table.
            self._marked_items = {}
        self._pending_resume_index = None
        self._pending_resume_of = resume_of
        self.recorder.upsert_area(
            area_id,
            self.current_visit_id,
            display_name=str(area.get("name", area_id)),
            location=str(area.get("name", area_id)),
            waypoint=str(area["waypoint"]),
            sequence=index + 1,
            status="navigating",
            resumes_visit_id=resume_of,
        )
        self._set_state(
            "NAVIGATING_TO_AREA",
            payload={"waypoint": area["waypoint"], "sequence": index + 1},
        )
        self.recorder.append_event(
            "navigation_requested",
            area_id=area_id,
            visit_id=self.current_visit_id,
            phase="area",
            payload={"waypoint": area["waypoint"], "leg_type": "area"},
        )
        self.navigate(
            str(area["waypoint"]),
            {
                "leg_type": "area",
                "area_id": area_id,
                "visit_id": self.current_visit_id,
                "sequence": index + 1,
            },
        )

    def _begin_return(self) -> None:
        self.current_area_id = None
        self.current_visit_id = None
        self.recorder.upsert_step('return_to_dock', 'return', self.config['return_waypoint'],
                                  status='navigating', result='pending')
        self._set_state(
            "RETURNING_TO_DOCK",
            payload={"waypoint": self.config["return_waypoint"]},
        )
        self.recorder.append_event(
            "navigation_requested",
            phase="return_to_dock",
            payload={
                "waypoint": self.config["return_waypoint"],
                "leg_type": "return_to_dock",
            },
        )
        self.navigate(
            str(self.config["return_waypoint"]),
            {"leg_type": "return_to_dock", "waypoint": self.config["return_waypoint"]},
        )

    def on_navigation_arrived(self, waypoint: str | None = None) -> dict[str, Any]:
        if self.state == 'NAVIGATING_TO_PICKUP':
            expected = self.config['pickup']['waypoint']
            if waypoint and str(waypoint) != expected:
                raise MissionError(f'unexpected pickup waypoint: {waypoint}')
            self._unresolved_failure = False
            self.recorder.append_event('navigation_arrived', phase='basket_pickup',
                                       payload={'waypoint': expected})
            self.recorder.upsert_step('basket_pickup', 'pickup', expected,
                                      status='awaiting_confirmation',
                                      arrived_at=datetime.now().astimezone().isoformat(timespec='milliseconds'))
            self._set_state('PICKING_BASKET')
            return self.snapshot()
        if self.state == "NAVIGATING_TO_AREA":
            expected = self.config["areas"][self.area_index]["waypoint"]
            if waypoint and str(waypoint) != str(expected):
                raise MissionError(f"unexpected waypoint: {waypoint}")
            self.recorder.append_event(
                "navigation_arrived",
                area_id=self.current_area_id,
                visit_id=self.current_visit_id,
                phase="area",
                payload={"waypoint": expected},
            )
            self._unresolved_failure = False
            capture_evidence = self._area_capture_evidence()
            self._set_state("CAPTURING_BEFORE" if capture_evidence else "WAITING_FOR_CLEANING")
            before_path = self._capture("before", reason="area_arrival") if capture_evidence else None
            self._set_state("WAITING_FOR_CLEANING")
            self.recorder.upsert_area(
                self.current_area_id,
                self.current_visit_id,
                status="ready_for_cleaning",
                before_photo=before_path,
                arrived_at=datetime.now().astimezone().isoformat(timespec="milliseconds"),
            )
            return self.snapshot()
        if self.state == "NAVIGATING_TO_BIN":
            expected = self.config["trash_bin"]["waypoint"]
            if waypoint and str(waypoint) != str(expected):
                raise MissionError(f"unexpected waypoint: {waypoint}")
            self.recorder.append_event(
                "navigation_arrived",
                phase="disposal",
                payload={"waypoint": self.config["trash_bin"]["waypoint"]},
            )
            self._unresolved_failure = False
            self._set_state("DUMPING")
            if self._current_disposal_step_id:
                self.recorder.upsert_step(self._current_disposal_step_id, 'disposal', expected,
                                          status='awaiting_confirmation',
                                          arrived_at=datetime.now().astimezone().isoformat(timespec='milliseconds'))
            return self.snapshot()
        if self.state == "RETURNING_TO_DOCK":
            if waypoint and str(waypoint) != str(self.config["return_waypoint"]):
                raise MissionError(f"unexpected waypoint: {waypoint}")
            self.recorder.append_event(
                "navigation_arrived",
                phase="return_to_dock",
                payload={"waypoint": self.config["return_waypoint"]},
            )
            self._unresolved_failure = False
            return self.snapshot()
        raise MissionError(f"navigation arrival is not expected in state {self.state}")

    def on_navigation_failed(self, code: str, detail: str, *, recoverable: bool = True) -> dict[str, Any]:
        failure = {
            "code": str(code),
            "detail": str(detail),
            "stage": self.state,
            "area_id": self.current_area_id,
            "visit_id": self.current_visit_id,
            "recoverable": bool(recoverable),
        }
        self._last_failure = failure
        self._unresolved_failure = True
        self.recorder.add_failure(failure)
        if self.state == 'NAVIGATING_TO_PICKUP':
            self.recorder.upsert_step('basket_pickup', 'pickup', self.config['pickup']['waypoint'],
                                      status='navigation_failed', failure_stage='basket_pickup', failure_code=str(code))
        elif self.state == 'NAVIGATING_TO_BIN' and self._current_disposal_step_id:
            self.recorder.upsert_step(self._current_disposal_step_id, 'disposal', self.config['trash_bin']['waypoint'],
                                      status='navigation_failed', failure_stage='disposal', failure_code=str(code))
        elif self.state == 'RETURNING_TO_DOCK':
            self.recorder.upsert_step('return_to_dock', 'return', self.config['return_waypoint'],
                                      status='navigation_failed', failure_stage='return_to_dock', failure_code=str(code))
        elif self.state == 'NAVIGATING_TO_AREA' and self.current_area_id:
            self.recorder.upsert_area(self.current_area_id, self.current_visit_id,
                                      status='navigation_failed', failure_stage='navigation')
        self.recorder.append_event(
            "navigation_failed",
            area_id=self.current_area_id,
            visit_id=self.current_visit_id,
            phase="navigation",
            payload=failure,
        )
        if recoverable:
            self._set_state("PAUSED")
        else:
            self._finish_failed(failure)
        return self.snapshot()

    def retry_navigation(self) -> dict[str, Any]:
        """Retry the currently paused navigation leg without changing task_id."""

        if self.state != "PAUSED":
            raise MissionError("there is no paused navigation leg to retry")
        failed_stage = str((self._last_failure or {}).get("stage", ""))
        if failed_stage == 'NAVIGATING_TO_PICKUP':
            waypoint = self.config['pickup']['waypoint']
            leg_type = 'pickup'
            self._set_state('NAVIGATING_TO_PICKUP', payload={'waypoint': waypoint, 'retry': True})
            context = {'leg_type': leg_type, 'retry': True}
        elif failed_stage == "NAVIGATING_TO_BIN":
            waypoint = self.config["trash_bin"]["waypoint"]
            leg_type = "disposal"
            self._set_state("NAVIGATING_TO_BIN", payload={"waypoint": waypoint, "retry": True})
            context = {"leg_type": leg_type, "retry": True}
        elif failed_stage == "RETURNING_TO_DOCK":
            waypoint = self.config["return_waypoint"]
            leg_type = "return_to_dock"
            self._set_state("RETURNING_TO_DOCK", payload={"waypoint": waypoint, "retry": True})
            context = {"leg_type": leg_type, "retry": True}
        elif self.current_area_id is not None and self.area_index >= 0:
            waypoint = self.config["areas"][self.area_index]["waypoint"]
            leg_type = "area"
            self._set_state("NAVIGATING_TO_AREA", payload={"waypoint": waypoint, "retry": True})
            context = {
                "leg_type": leg_type,
                "area_id": self.current_area_id,
                "visit_id": self.current_visit_id,
                "retry": True,
            }
        else:
            waypoint = self.config["return_waypoint"]
            self._set_state("RETURNING_TO_DOCK", payload={"waypoint": waypoint, "retry": True})
            context = {"leg_type": "return_to_dock", "retry": True}
        self.recorder.append_event(
            "navigation_retry_requested",
            area_id=self.current_area_id,
            visit_id=self.current_visit_id,
            phase="navigation",
            payload={"waypoint": waypoint, "leg_type": leg_type},
        )
        self.navigate(str(waypoint), context)
        return self.snapshot()

    def navigation_resumed_externally(self, waypoint: str) -> dict[str, Any]:
        """Reconcile an explicit existing base-release/resume service call."""
        if self.state != 'PAUSED':
            return self.snapshot()
        stage = (self._last_failure or {}).get('stage')
        if stage == 'NAVIGATING_TO_PICKUP':
            expected = self.config['pickup']['waypoint']
        elif stage == 'NAVIGATING_TO_AREA':
            expected = self.config['areas'][self.area_index]['waypoint']
        elif stage == 'NAVIGATING_TO_BIN':
            expected = self.config['trash_bin']['waypoint']
        elif stage == 'RETURNING_TO_DOCK':
            expected = self.config['return_waypoint']
        else:
            raise MissionError('paused stage is not a navigation leg')
        if str(waypoint) != str(expected):
            raise MissionError('resumed navigation does not match the active mission leg')
        self._set_state(stage, payload={'resume_source': 'explicit_base_release'})
        self.recorder.append_event('navigation_resumed', area_id=self.current_area_id,
            visit_id=self.current_visit_id, phase='navigation', payload={'waypoint': expected})
        return self.snapshot()

    def _capture(self, stage: str, *, reason: str) -> str | None:
        if not self.current_area_id or not self.current_visit_id:
            return None
        context = {
            "task_id": self.task_id,
            "area_id": self.current_area_id,
            "visit_id": self.current_visit_id,
            "stage": stage,
            "reason": reason,
            "state": self.state,
        }
        if self.capture_photo is None:
            self.recorder.record_photo_failure(
                self.current_area_id, self.current_visit_id, stage,
                "no_capture_callback", reason=reason,
            )
            self.recorder.append_event(
                "photo_capture_unavailable",
                area_id=self.current_area_id,
                visit_id=self.current_visit_id,
                phase=stage,
                payload={"reason": "no_capture_callback", "capture_reason": reason},
            )
            self.recorder.update_summary(data_quality="incomplete", review_required=True)
            return None
        try:
            captured = self.capture_photo(stage, context)
            if captured is None:
                raise RuntimeError("capture callback returned no image")
            metadata: Mapping[str, Any] = {}
            image_bytes: bytes
            if isinstance(captured, bytes):
                image_bytes = captured
            elif isinstance(captured, Mapping):
                image_bytes = bytes(captured.get("data", b""))
                metadata = dict(captured.get("metadata") or {})
                if not image_bytes:
                    raise RuntimeError("capture callback returned empty image data")
            else:
                raise TypeError("capture callback must return bytes or {data, metadata}")
            return self.recorder.save_photo_bytes(
                image_bytes,
                area_id=self.current_area_id,
                visit_id=self.current_visit_id,
                stage=stage,
                reason=reason,
                metadata=metadata,
            )
        except Exception as error:  # noqa: BLE001 - photo failure must not block VR
            self.recorder.record_photo_failure(
                self.current_area_id, self.current_visit_id, stage,
                str(error), reason=reason,
            )
            self.recorder.append_event(
                "photo_capture_failed",
                area_id=self.current_area_id,
                visit_id=self.current_visit_id,
                phase=stage,
                payload={"reason": reason, "error": str(error)},
            )
            self.recorder.update_summary(data_quality="incomplete", review_required=True)
            return None

    def capture_supplemental_photo(self, reason: str = "head_alignment") -> str | None:
        """Capture an optional extra image while B has manual head control."""

        if self.state not in {"WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
            raise MissionError("supplemental photo requires an active area")
        return self._capture("supplemental", reason=reason)

    def record_manipulation_event(
        self,
        event_type: str,
        *,
        payload: Mapping[str, Any] | None = None,
        operation_id: str | None = None,
    ) -> dict[str, Any]:
        if self.state not in {"WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
            raise MissionError("manipulation events require an active area")
        if self._operation_seen(operation_id):
            return self.snapshot()
        self.recorder.append_event(
            event_type,
            source="teleop",
            event_id=operation_id,
            area_id=self.current_area_id,
            visit_id=self.current_visit_id,
            phase="cleaning",
            payload=payload,
        )
        return self.snapshot()

    def set_basket_state(self, state: str, *, reason: str = "operator") -> dict[str, Any]:
        normalized = str(state).strip().lower()
        if normalized not in {"empty", "normal", "full", "unknown"}:
            raise ValueError("basket state must be empty, normal, full or unknown")
        self.basket_state = normalized
        self.recorder.update_summary(basket_state=normalized)
        self.recorder.append_event(
            "basket_state_changed",
            area_id=self.current_area_id,
            visit_id=self.current_visit_id,
            phase="cleaning",
            payload={"state": normalized, "reason": reason},
        )
        return self.snapshot()

    def _current_area_config(self) -> Mapping[str, Any]:
        if not 0 <= self.area_index < len(self.config["areas"]):
            raise MissionError("there is no active area")
        return self.config["areas"][self.area_index]

    def _area_allowed_items(self) -> set[str]:
        allowed = self._current_area_config().get("allowed_items")
        return set(allowed) if allowed else set(DEFAULT_ALLOWED_ITEMS)

    def _validated_items(self, items) -> list[dict[str, Any]]:
        """Accept only explicit operator items inside the area's allow-list."""
        if items is None:
            return []
        if not isinstance(items, list) or len(items) > 8:
            raise ValueError("清理物品需为不超过8项的列表")
        allowed = self._area_allowed_items()
        normalized: list[dict[str, Any]] = []
        for item in items:
            if not isinstance(item, Mapping) or str(item.get("label", "")).strip() not in allowed:
                raise ValueError("清理物品标签不在本桌允许列表内")
            count = item.get("count", 1)
            if isinstance(count, bool) or not isinstance(count, int):
                raise ValueError("清理物品数量需为整数")
            if not 1 <= count <= 99:
                raise ValueError("清理物品数量需在1–99内")
            normalized.append({"label": str(item["label"]).strip(), "count": count})
        return normalized

    def _merged_items(self, explicit: list[dict[str, Any]] | None) -> list[dict[str, Any]]:
        merged = dict(self._marked_items)
        for item in explicit or []:
            merged[item["label"]] = item["count"]
        return [{"label": label, "count": count} for label, count in sorted(merged.items())]

    def mark_cleaned_item(self, label: str, *, operation_id: str | None = None) -> dict[str, Any]:
        """Record one operator-confirmed cleaned item for the current visit."""
        if self.state not in {"WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
            raise MissionError("只能在桌面作业阶段标记清理物品")
        label = str(label).strip()
        if label not in self._area_allowed_items():
            raise ValueError("清理物品标签不在本桌允许列表内")
        if self._operation_seen(operation_id):
            return self.snapshot()
        self._marked_items[label] = min(99, self._marked_items.get(label, 0) + 1)
        self.recorder.upsert_area(self.current_area_id, self.current_visit_id,
                                  cleaned_items=self._merged_items([]))
        self.recorder.append_event("cleaned_item_marked", source="operator", event_id=operation_id,
            area_id=self.current_area_id, visit_id=self.current_visit_id, phase="cleaning",
            payload={"label": label, "count": self._marked_items[label]})
        return self.snapshot()

    def confirm_area(
        self,
        *,
        result: str = "completed",
        cleaned_items: list[Mapping[str, Any]] | None = None,
        arms_secured: bool = False,
        operation_id: str | None = None,
        note: str | None = None,
        capture_after: bool = True,
    ) -> dict[str, Any]:
        if self.state not in {"WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
            raise MissionError(f"area confirmation is not allowed in state {self.state}")
        # Validate operator items before any state change or photo capture.
        cleaned_items = self._merged_items(self._validated_items(cleaned_items))
        if self._operation_seen(operation_id):
            return self.snapshot()
        self._set_state("CAPTURING_AFTER")
        after_path = (self._capture("after", reason="area_confirm")
                      if capture_after and self._area_capture_evidence() else None)
        self._set_state("WAITING_AREA_CONFIRM")
        normalized_result = str(result).strip() or "completed"
        self.current_area_finished = True
        self.current_area_result = normalized_result
        if normalized_result not in {"completed", "no_target"}:
            self._has_area_failure = True
            self.recorder.update_summary(review_required=True)
            previous = self._last_failure or {}
            if previous.get('visit_id') != self.current_visit_id:
                previous = {}
            code = previous.get('code') if previous.get('visit_id') == self.current_visit_id else None
            self.recorder.add_failure({
                'code': code or ('AREA_SKIPPED' if normalized_result == 'skipped' else 'AREA_INCOMPLETE'),
                'stage': 'cleaning', 'area_id': self.current_area_id,
                'visit_id': self.current_visit_id, 'result': normalized_result,
                'detail': note or previous.get('detail') or 'operator confirmed the area was not completed',
                'recoverable': False,
            })
        self.recorder.upsert_area(
            self.current_area_id,
            self.current_visit_id,
            status="finished",
            result=normalized_result,
            after_photo=after_path,
            cleaned_items=deepcopy(cleaned_items or []),
            ended_at=datetime.now().astimezone().isoformat(timespec="milliseconds"),
            note=note,
            failure_stage='cleaning' if normalized_result not in {'completed', 'no_target'} else None,
        )
        self.recorder.append_event(
            "area_confirmed",
            source="operator",
            event_id=operation_id,
            area_id=self.current_area_id,
            visit_id=self.current_visit_id,
            phase="cleaning",
            payload={
                "result": normalized_result,
                "cleaned_items": deepcopy(cleaned_items or []),
                "after_photo": after_path,
                "note": note,
            },
        )
        if self.basket_state == "full":
            self._pending_resume_index = self.area_index + 1
            self._pending_resume_of = None
            self._pending_disposal_kind = 'basket_full'
            self._set_state(
                "PAUSED_FOR_DISPOSAL",
                payload={"reason": "basket_full", "resume_index": self._pending_resume_index},
            )
            if arms_secured:
                self.request_disposal(arms_secured=True)
        else:
            self._advance_after_area()
        return self.snapshot()

    def request_disposal(self, *, arms_secured: bool, operation_id: str | None = None) -> dict[str, Any]:
        scheduled_final = self._pending_disposal_kind == 'scheduled_final'
        if self.basket_state != "full" and not scheduled_final:
            raise MissionError("disposal is only required when the basket is full")
        if not arms_secured:
            raise SafetyCheckpointRequired(
                "secure the arms and basket before navigating to the bin"
            )
        if not self.config.get('trash_bin', {}).get('waypoint'):
            raise MissionError('当前任务串没有配置垃圾桶航点，不能执行倒垃圾')
        if self.state not in {"PAUSED_FOR_DISPOSAL", "WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
            raise MissionError(f"disposal is not allowed in state {self.state}")
        if self._operation_seen(operation_id):
            return self.snapshot()
        if not self.current_area_finished:
            self.recorder.upsert_area(self.current_area_id, self.current_visit_id,
                                      status="suspended_for_disposal")
        if self._current_disposal_step_id is None:
            next_number = 1 + sum(1 for step in self.recorder.summary().get('steps', [])
                                  if step.get('kind') == 'disposal')
            self._current_disposal_step_id = f'disposal_{next_number:03d}'
        self.recorder.upsert_step(self._current_disposal_step_id, 'disposal',
                                  self.config['trash_bin']['waypoint'],
                                  status='navigating', result='pending',
                                  resume_visit_id=self.current_visit_id,
                                  resume_index=self._pending_resume_index,
                                  reason=self._pending_disposal_kind)
        self._set_state(
            "NAVIGATING_TO_BIN",
            payload={"waypoint": self.config["trash_bin"]["waypoint"]},
        )
        self.recorder.append_event(
            "disposal_requested",
            source="operator",
            event_id=operation_id,
            area_id=self.current_area_id,
            visit_id=self.current_visit_id,
            phase="disposal",
            payload={
                "waypoint": self.config["trash_bin"]["waypoint"],
                "resume_index": self._pending_resume_index,
                "disposal_kind": self._pending_disposal_kind,
            },
        )
        self.navigate(
            str(self.config["trash_bin"]["waypoint"]),
            {
                "leg_type": "disposal",
                "area_id": self.current_area_id,
                "visit_id": self.current_visit_id,
                "resume_index": self._pending_resume_index,
                "disposal_kind": self._pending_disposal_kind,
            },
        )
        return self.snapshot()

    def confirm_disposal(
        self,
        *,
        success: bool,
        operation_id: str | None = None,
        note: str | None = None,
        disposal_result: str | None = None,
    ) -> dict[str, Any]:
        if self.state != "DUMPING":
            raise MissionError(f"disposal confirmation is not allowed in state {self.state}")
        # Keep the old optional value accepted for one release, but expose
        # only success or failure for the bin step.
        if success and disposal_result not in {None, 'dumped', 'empty'}:
            raise ValueError('invalid disposal result')
        if self._operation_seen(operation_id):
            return self.snapshot()
        if not success:
            failure = {
                "code": "DUMP_FAILED",
                "stage": "dumping",
                "area_id": self.current_area_id,
                "visit_id": self.current_visit_id,
                "detail": note or "operator could not complete the disposal",
                "recoverable": True,
                "result": "failed",
            }
            self._last_failure = failure
            self._unresolved_failure = True
            self.recorder.add_failure(failure)
            self.recorder.append_event(
                "disposal_failed",
                source="operator",
                event_id=operation_id,
                phase="disposal",
                payload=failure,
            )
            self.recorder.update_summary(last_disposal={
                "type": self._pending_disposal_kind or "basket_full",
                "result": "failed",
                "note": note,
            })
            if self._current_disposal_step_id:
                self.recorder.upsert_step(self._current_disposal_step_id, 'disposal',
                                          self.config['trash_bin']['waypoint'],
                                          status='failed', result='failed',
                                          failure_stage='disposal', note=note)
            self._set_state("PAUSED_FOR_DISPOSAL")
            return self.snapshot()
        resume_index = self._pending_resume_index
        if resume_index is None:
            resume_index = self.area_index if not self.current_area_finished else self.area_index + 1
        resume_of = None if self.current_area_finished else self.current_visit_id
        disposal_kind = self._pending_disposal_kind or 'basket_full'
        detour = {
            "type": disposal_kind,
            "trigger_area": self.current_area_id,
            "trigger_visit_id": self.current_visit_id,
            "resume_area": (
                self.config["areas"][resume_index]["id"]
                if resume_index < len(self.config["areas"])
                else None
            ),
            "result": "success",
            "note": note,
        }
        self.recorder.add_detour(detour)
        if self._current_disposal_step_id:
            self.recorder.upsert_step(self._current_disposal_step_id, 'disposal',
                                      self.config['trash_bin']['waypoint'],
                                      status='completed', result='completed', outcome='success',
                                      resume_area=detour['resume_area'])
        self.recorder.append_event(
            'disposal_completed',
            source="operator",
            event_id=operation_id,
            phase="disposal",
            payload=detour,
        )
        self.basket_state = "empty"
        self._unresolved_failure = False
        final_bin_completed = (self.config.get('scheduled_final_disposal')
            and resume_index >= len(self.config['areas']) and self.current_area_finished)
        self.recorder.update_summary(basket_state="empty", last_disposal=detour,
                                     final_bin_completed=bool(final_bin_completed))
        self._pending_resume_index = None
        self._pending_resume_of = None
        self._pending_disposal_kind = None
        self._current_disposal_step_id = None
        self._begin_area(resume_index, resume_of=resume_of)
        return self.snapshot()

    def retry_disposal(self) -> dict[str, Any]:
        if self.state != "PAUSED_FOR_DISPOSAL" or (self._last_failure or {}).get("code") != "DUMP_FAILED":
            raise MissionError("there is no failed disposal to retry in place")
        self._set_state("DUMPING", payload={"retry": True})
        return self.snapshot()

    def skip_area(self, *, reason: str) -> dict[str, Any]:
        if not self.current_area_id or self.state not in {"NAVIGATING_TO_AREA", "PAUSED", "WAITING_FOR_CLEANING", "WAITING_AREA_CONFIRM"}:
            raise MissionError("the current leg is not a skippable area")
        if self.state == "PAUSED" and (self._last_failure or {}).get("stage") != "NAVIGATING_TO_AREA":
            raise MissionError("disposal or dock navigation cannot be skipped as an area")
        reached = self.state not in {'NAVIGATING_TO_AREA', 'PAUSED'}
        if not reached:
            self.recorder.update_summary(data_quality="incomplete")
            self.recorder.append_event('photo_capture_skipped', area_id=self.current_area_id,
                visit_id=self.current_visit_id, phase='after', payload={'reason':'area_not_reached'})
            self._set_state("WAITING_AREA_CONFIRM")
        self._unresolved_failure = False
        return self.confirm_area(result="skipped", note=reason, capture_after=reached)

    def on_dock_arrived(self, charging_confirmed: bool | None = None) -> dict[str, Any]:
        if self.state != "RETURNING_TO_DOCK":
            raise MissionError(f"dock arrival is not expected in state {self.state}")
        self._unresolved_failure = False
        self.recorder.update_summary(charging_confirmed=charging_confirmed)
        self.recorder.upsert_step('return_to_dock', 'return', self.config['return_waypoint'],
                                  status='completed' if charging_confirmed is not False else 'failed',
                                  result='completed' if charging_confirmed is not False else 'failed',
                                  charging_confirmed=charging_confirmed,
                                  completed_at=datetime.now().astimezone().isoformat(timespec='milliseconds'))
        self.recorder.append_event(
            "dock_arrived",
            phase="return_to_dock",
            payload={"charging_confirmed": charging_confirmed},
        )
        if charging_confirmed is False:
            self._finish_failed({
                "code": "CHARGING_NOT_CONFIRMED",
                "stage": "charging",
                "detail": "robot returned near the dock but charging was not confirmed",
                "recoverable": False,
            })
        else:
            result = "failed" if self._has_area_failure or self._unresolved_failure else "completed"
            review_required = bool(
                charging_confirmed is None
                or result != "completed"
                or self.recorder.summary().get("failures")
                or self.recorder.summary().get('review_required')
            )
            self.recorder.update_summary(
                charging_confirmed=charging_confirmed,
                review_required=review_required,
            )
            self.recorder.finish(
                result=result,
                execution_status="finished",
                data_quality=("complete" if charging_confirmed is not None
                              and self.recorder.summary().get("data_quality") == "complete"
                              else "incomplete"),
                review_required=review_required,
            )
            self.state = "FINISHED"
        return self.snapshot()

    def record_exception(
        self,
        code: str,
        *,
        phase: str,
        detail: str = "",
        recoverable: bool = True,
        action: str = "continue",
        operation_id: str | None = None,
    ) -> dict[str, Any]:
        if self._operation_seen(operation_id):
            return self.snapshot()
        failure = {
            "code": str(code),
            "phase": str(phase),
            "detail": str(detail),
            "area_id": self.current_area_id,
            "visit_id": self.current_visit_id,
            "recoverable": bool(recoverable),
        }
        self._last_failure = failure
        if not recoverable:
            self._has_area_failure = True
        self._unresolved_failure = action in {"retry", "pause"} or not recoverable
        self.recorder.add_failure(failure)
        self.recorder.append_event(
            "exception_recorded",
            source="operator",
            event_id=operation_id,
            area_id=self.current_area_id,
            visit_id=self.current_visit_id,
            phase=phase,
            payload=failure,
        )
        if action in {"abort", "fail"} or not recoverable:
            self._finish_failed(failure)
        else:
            self.recorder.update_summary(review_required=True)
        return self.snapshot()

    def abort(self, reason: str = "operator_abort") -> dict[str, Any]:
        self.recorder.append_event(
            "mission_aborted",
            source="operator",
            phase=self.state,
            payload={"reason": reason},
        )
        self.recorder.finish(
            result="aborted",
            execution_status="aborted",
            data_quality="incomplete",
            review_required=True,
        )
        self.state = "ABORTED"
        return self.snapshot()

    def _finish_failed(self, failure: Mapping[str, Any]) -> None:
        existing = self.recorder.summary().get("failures", [])
        if not any(
            item.get("code") == failure.get("code")
            and item.get("detail") == failure.get("detail")
            for item in existing
        ):
            self.recorder.add_failure(failure)
        self.recorder.finish(
            result="failed",
            execution_status="finished",
            data_quality="complete",
            review_required=True,
        )
        self.state = "FAILED"
        self.recorder.update_summary(current_state="FAILED")

    def _advance_after_area(self) -> None:
        next_index = self.area_index + 1
        if next_index < len(self.config["areas"]):
            self._begin_area(next_index)
        elif self.config.get('generic_route'):
            self._finish_generic_route()
        elif self.config.get('scheduled_final_disposal'):
            self._pending_resume_index = len(self.config['areas'])
            self._pending_resume_of = None
            self._pending_disposal_kind = 'scheduled_final'
            self._set_state('PAUSED_FOR_DISPOSAL', payload={
                'reason': 'scheduled_final_disposal',
                'waypoint': self.config['trash_bin']['waypoint'],
                'resume_index': self._pending_resume_index,
            })
            self.recorder.append_event('scheduled_final_disposal_ready',
                area_id=self.current_area_id, visit_id=self.current_visit_id, phase='disposal',
                payload={'waypoint': self.config['trash_bin']['waypoint']})
        else:
            self._begin_return()

    def _area_capture_evidence(self) -> bool:
        """Return whether this point belongs to the photo evidence classes."""
        area = self._current_area_config()
        if self.config.get('generic_route'):
            return bool(area.get('capture_evidence'))
        return True

    def _finish_generic_route(self) -> None:
        summary = self.recorder.summary()
        result = 'failed' if self._has_area_failure or self._unresolved_failure else 'completed'
        review_required = bool(result != 'completed' or summary.get('failures')
                               or summary.get('review_required'))
        self.recorder.finish(
            result=result,
            execution_status='finished',
            data_quality=('complete' if summary.get('data_quality') == 'complete' else 'incomplete'),
            review_required=review_required,
        )
        self.state = 'FINISHED' if result == 'completed' else 'FAILED'
        self.recorder.update_summary(current_state=self.state)

    def snapshot(self) -> dict[str, Any]:
        summary = self.recorder.summary()
        return {
            "task_id": self.task_id,
            "state": self.state,
            "area_index": self.area_index,
            "current_area_id": self.current_area_id,
            "current_visit_id": self.current_visit_id,
            "current_area_finished": self.current_area_finished,
            "current_area_result": self.current_area_result,
            "basket_state": self.basket_state,
            "pending_resume_index": self._pending_resume_index,
            "pending_disposal_kind": self._pending_disposal_kind,
            "last_failure": deepcopy(self._last_failure),
            "mission": summary,
        }

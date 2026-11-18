"""Record map, waypoints and travelled path for a hotel public-area mission.

Pure stdlib and ROS-free: the bridge feeds one sample per tick and the sync
worker stores the resulting JSON next to ``mission.json``.  The track is
uploaded inside ``mission.track`` (plan A: no new server endpoint).
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import threading
import time
from datetime import datetime
from typing import Any, Mapping

SCHEMA_VERSION = 1
MIN_MOVE_M = 0.05
MAX_GAP_M = 2.0
NAVIGATING_STATES = {
    "NAVIGATING_TO_AREA",
    "NAVIGATING_TO_PICKUP",
    "NAVIGATING_TO_BIN",
    "RETURNING_TO_DOCK",
}


def _text(value: Any) -> str:
    return "" if value is None else str(value).strip()


def _now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def _distance(a: Mapping[str, Any], b: Mapping[str, Any]) -> float:
    try:
        return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))
    except (KeyError, TypeError, ValueError):
        return 0.0


def _target_waypoint(mission: Mapping[str, Any], state: str) -> str:
    config = ((mission.get("metadata") or {}).get("config")) or {}
    if state == "NAVIGATING_TO_AREA":
        areas = config.get("areas") or []
        try:
            index = int(mission.get("area_index", -1))
        except (TypeError, ValueError):
            index = -1
        if 0 <= index < len(areas):
            return _text(areas[index].get("waypoint"))
    if state == "NAVIGATING_TO_BIN":
        return _text((config.get("trash_bin") or {}).get("waypoint"))
    if state == "NAVIGATING_TO_PICKUP":
        return _text((config.get("pickup") or {}).get("waypoint"))
    if state == "RETURNING_TO_DOCK":
        return _text(config.get("return_waypoint")) or _text(mission.get("return_waypoint"))
    return ""


class TrackRecorder:
    """Accumulate one mission's map snapshot, waypoints and per-leg path."""

    def __init__(
        self,
        *,
        sample_seconds: float = 2.0,
        max_points: int = 6000,
        clock=time.monotonic,
    ) -> None:
        self.sample_seconds = max(0.5, float(sample_seconds))
        self.max_points = max(200, int(max_points))
        self._clock = clock
        self._lock = threading.RLock()
        self._dirty = False
        self._reset_locked()

    # -- lifecycle ---------------------------------------------------------
    def _reset_locked(self) -> None:
        self.task_id = ""
        self.robot_id = ""
        self.started_at = ""
        self._start_mono = 0.0
        self._last_sample = 0.0
        self._last_pose: dict[str, Any] | None = None
        self._map: dict[str, Any] | None = None
        self._map_version: int | None = None
        self._waypoints: dict[str, dict[str, Any]] = {}
        self._legs: list[dict[str, Any]] = []
        self._current: dict[str, Any] | None = None
        self._points = 0
        self._truncated = False
        self._complete = False
        self._ended_at = ""

    def reset(self) -> None:
        with self._lock:
            self._reset_locked()
            self._dirty = True

    @property
    def dirty(self) -> bool:
        with self._lock:
            return self._dirty

    def clear_dirty(self) -> None:
        with self._lock:
            self._dirty = False

    # -- ingestion ---------------------------------------------------------
    def update(self, snapshot: Mapping[str, Any], context: Mapping[str, Any]) -> None:
        """Feed one sample. ``dirty`` tells the caller to persist the payload."""

        mission = snapshot.get("mission") or {}
        task_id = _text(mission.get("task_id"))
        if not task_id:
            return
        with self._lock:
            if task_id != self.task_id:
                self._reset_locked()
                self.task_id = task_id
                self.robot_id = _text(mission.get("robot_id")) or "283"
                self.started_at = _text(mission.get("started_at")) or _now_iso()
                self._start_mono = self._clock()
                self._dirty = True
            if self._complete:
                return

            self._capture_map(context)
            self._capture_waypoints(mission, context)
            now = self._clock()
            if now - self._last_sample < self.sample_seconds:
                return
            self._last_sample = now
            self._sample_pose(mission, context, now)
            if _text(mission.get("ended_at")):
                self._complete = True
                self._ended_at = _text(mission.get("ended_at"))
                self._close_leg(now)
                self._dirty = True

    def _capture_map(self, context: Mapping[str, Any]) -> None:
        png = context.get("map_png")
        meta = context.get("map_meta") or {}
        version = context.get("map_version")
        if not png or not meta:
            return
        if self._map is not None and version == self._map_version:
            return
        digest = hashlib.sha256(bytes(png)).hexdigest()[:12]
        self._map = {
            "map_id": f"283-{digest}",
            "map_version": version,
            "width": int(meta.get("width", 0)),
            "height": int(meta.get("height", 0)),
            "resolution": float(meta.get("resolution", 0.0)),
            "origin": {
                "x": float(meta.get("origin_x", 0.0)),
                "y": float(meta.get("origin_y", 0.0)),
                "yaw": float(meta.get("origin_yaw", 0.0)),
            },
            "frame_id": _text(meta.get("frame_id")) or "map",
            "image": "data:image/png;base64," + base64.b64encode(bytes(png)).decode("ascii"),
            "image_bytes": len(png),
        }
        self._map_version = version
        self._dirty = True

    def _capture_waypoints(self, mission: Mapping[str, Any], context: Mapping[str, Any]) -> None:
        config = ((mission.get("metadata") or {}).get("config")) or {}
        wanted: list[str] = []
        for area in config.get("areas") or []:
            name = _text(area.get("waypoint"))
            if name:
                wanted.append(name)
        for key in ("trash_bin", "pickup"):
            section = config.get(key) or {}
            if _text(section.get("waypoint")):
                wanted.append(_text(section.get("waypoint")))
        if _text(config.get("return_waypoint")):
            wanted.append(_text(config.get("return_waypoint")))
        poses = {_text(item.get("name")): item for item in context.get("waypoint_poses") or []}
        changed = False
        for order, name in enumerate(dict.fromkeys(wanted), 1):
            pose = poses.get(name)
            if pose is None:
                continue
            entry = self._waypoints.get(name)
            if entry is None:
                entry = {
                    "name": name,
                    "x": float(pose.get("x", 0.0)),
                    "y": float(pose.get("y", 0.0)),
                    "yaw": float(pose.get("yaw", 0.0)),
                    "frame_id": _text(pose.get("frame_id")) or "map",
                    "order": order,
                    "area_name": name,
                }
                self._waypoints[name] = entry
                changed = True
        if changed:
            self._dirty = True

    def _sample_pose(self, mission: Mapping[str, Any], context: Mapping[str, Any], now: float) -> None:
        if not context.get("pose_fresh"):
            return
        pose = context.get("pose")
        if not isinstance(pose, Mapping):
            return
        point = {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "yaw": float(pose.get("yaw", 0.0)),
        }
        state = _text(mission.get("current_state"))
        target = _target_waypoint(mission, state)
        if state in NAVIGATING_STATES and target:
            self._ensure_leg(state, target)
        else:
            self._close_leg(now)
        leg = self._current
        if leg is None:
            return
        if self._points >= self.max_points:
            if not self._truncated:
                self._truncated = True
                self._dirty = True
            return
        last = self._last_pose
        if last is not None:
            movement = _distance(last, point)
            if movement < MIN_MOVE_M:
                return
            if movement > MAX_GAP_M:
                leg.setdefault("gaps", []).append(len(leg["poses"]))
        leg["poses"].append([
            round(now - self._start_mono, 1),
            round(point["x"], 3),
            round(point["y"], 3),
            round(point["yaw"], 3),
        ])
        self._points += 1
        self._last_pose = point
        self._dirty = True

    def _ensure_leg(self, state: str, target: str) -> None:
        key = f"{state}:{target}"
        current = self._current
        if current is not None and current["key"] == key:
            return
        now = self._clock()
        self._close_leg(now)
        previous = self._legs[-1]["to"] if self._legs else ""
        self._current = {
            "key": key,
            "leg_id": str(len(self._legs) + 1),
            "from": previous,
            "to": target,
            "started_at": _now_iso(),
            "ended_at": "",
            "result": "running",
            "poses": [],
            "gaps": [],
        }
        self._dirty = True

    def _close_leg(self, now: float) -> None:
        current = self._current
        if current is None:
            return
        current["ended_at"] = _now_iso()
        current["result"] = "completed" if current["poses"] else "no_data"
        current.pop("key", None)
        self._legs.append(current)
        self._current = None
        self._dirty = True

    # -- output ------------------------------------------------------------
    def payload(self) -> dict[str, Any] | None:
        with self._lock:
            if not self.task_id:
                return None
            legs = list(self._legs)
            current = self._current
            if current is not None:
                snapshot = dict(current)
                snapshot.pop("key", None)
                snapshot["ended_at"] = ""
                snapshot["result"] = "running"
                legs = legs + [snapshot]
            return {
                "schema_version": SCHEMA_VERSION,
                "task_id": self.task_id,
                "robot_id": self.robot_id,
                "started_at": self.started_at,
                "ended_at": self._ended_at,
                "sampling_seconds": self.sample_seconds,
                "map": self._map,
                "waypoints": [self._waypoints[name] for name in sorted(self._waypoints, key=lambda n: self._waypoints[n]["order"])],
                "legs": legs,
                "point_count": self._points,
                "truncated": self._truncated,
                "complete": self._complete,
            }

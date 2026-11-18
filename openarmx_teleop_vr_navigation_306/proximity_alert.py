"""Pure front-lidar proximity detector for the VR safety announcement.

Kept free of rclpy so the trigger/clear hysteresis can be unit tested on a
workstation. The node wraps this class and publishes the configured text.
"""

from __future__ import annotations

import math


class ProximityDetector:
    """Hysteresis detector: alert after N close scans, re-arm after M clear scans."""

    def __init__(
        self,
        *,
        trigger_distance_m: float = 0.80,
        clear_distance_m: float = 0.95,
        forward_angle_deg: float = 180.0,
        half_angle_deg: float = 40.0,
        minimum_points: int = 3,
        trigger_consecutive_scans: int = 3,
        clear_consecutive_scans: int = 3,
        repeat_while_near_sec: float = 15.0,
        startup_grace_sec: float = 4.0,
        started_at: float = 0.0,
    ) -> None:
        self.trigger_distance_m = max(0.10, float(trigger_distance_m))
        self.clear_distance_m = max(
            self.trigger_distance_m + 0.05, float(clear_distance_m)
        )
        self.forward_angle = math.radians(float(forward_angle_deg))
        self.half_angle = math.radians(max(5.0, min(90.0, float(half_angle_deg))))
        self.minimum_points = max(1, int(minimum_points))
        self.trigger_scans = max(1, int(trigger_consecutive_scans))
        self.clear_scans = max(1, int(clear_consecutive_scans))
        self.repeat_period = max(0.0, float(repeat_while_near_sec))
        self.startup_grace = max(0.0, float(startup_grace_sec))
        self._started_at = float(started_at)
        self._near_samples = 0
        self._clear_samples = 0
        self._near_active = False
        self._last_alert_at = -1e9

    @staticmethod
    def _normalized_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def count_within(
        self,
        ranges,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        limit: float,
    ) -> int:
        count = 0
        angle = float(angle_min)
        for raw in ranges:
            distance = float(raw)
            if (
                math.isfinite(distance)
                and range_min <= distance <= range_max
                and distance <= limit
                and abs(
                    self._normalized_angle(angle - self.forward_angle)
                ) <= self.half_angle
            ):
                count += 1
                if count >= self.minimum_points:
                    return count
            angle += float(angle_increment)
        return count

    def update(
        self,
        ranges,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        now: float,
    ) -> bool:
        """Return True when the caller should publish one alert now."""
        if now - self._started_at < self.startup_grace:
            return False

        if self._near_active:
            still_near = self.count_within(
                ranges, angle_min, angle_increment, range_min, range_max,
                self.clear_distance_m,
            ) >= self.minimum_points
            if still_near:
                self._clear_samples = 0
                if (
                    self.repeat_period > 0.0
                    and now - self._last_alert_at >= self.repeat_period
                ):
                    return self._alert(now)
            else:
                self._clear_samples += 1
                if self._clear_samples >= self.clear_scans:
                    self.reset()
            return False

        is_near = self.count_within(
            ranges, angle_min, angle_increment, range_min, range_max,
            self.trigger_distance_m,
        ) >= self.minimum_points
        if not is_near:
            self._near_samples = 0
            return False
        self._near_samples += 1
        if self._near_samples >= self.trigger_scans:
            self._near_active = True
            self._clear_samples = 0
            return self._alert(now)
        return False

    def _alert(self, now: float) -> bool:
        self._last_alert_at = now
        return True

    def reset(self) -> None:
        self._near_samples = 0
        self._clear_samples = 0
        self._near_active = False

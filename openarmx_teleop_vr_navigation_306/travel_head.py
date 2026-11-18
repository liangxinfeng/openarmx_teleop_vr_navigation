"""Travel head pose state machine.

Pure logic, no ROS imports: the bridge feeds it navigation status and the
teleop backend snapshot, and it answers with an optional head-preset
publish payload. Unit-tested without a ROS runtime.

Behaviour (2026-10-31 field spec):
* while the base navigates to a waypoint (any leg: single-point, multi-task
  chain, dynamic trash detour) hold the travel pitch so the operator sees
  ahead;
* a new leg re-arms the pose even when the operator took the head with B
  during the previous leg;
* operator head takeover (head follow) suppresses the pose for the rest of
  the leg without touching navigation;
* the V4 controller rejects task head targets while disabled/resetting/
  estopped and ignores the rejected request id forever, so rejections
  rotate the id at a bounded retry rate and engage automatically once the
  operator enables hardware;
* on single-point arrival hold the task-library pitch for a short deadline
  until the encoder reports the neck on target (multi-task arrivals are
  owned by the bridge's ``_set_task_height`` instead).
"""
import time


_MOVING_EXCLUDED_STATES = {'', 'idle', 'waiting', 'paused', 'map_ready'}


def _request_token():
    return str(time.monotonic_ns())


class TravelHeadController:
    def __init__(self, pitch_deg, enabled=True,
                 retry_interval_seconds=1.0, arrival_timeout_seconds=8.0):
        self.pitch_deg = max(0.0, min(45.0, float(pitch_deg)))
        self.enabled = bool(enabled)
        self.retry_interval = float(retry_interval_seconds)
        self.arrival_timeout = float(arrival_timeout_seconds)
        self._active = False
        self._suppressed = False
        self._request_id = ''
        self._leg_waypoint = ''
        self._retry_at = 0.0
        self._arrival_pitch = None
        self._arrival_request_id = ''
        self._arrival_deadline = 0.0

    @property
    def arrival(self):
        """Current single-point arrival target, if one is being held."""
        if self._arrival_pitch is None:
            return None
        return {'head_pitch_deg': self._arrival_pitch,
                'request_id': self._arrival_request_id}

    def leg_update(self, state, waypoint, outcome, now):
        """Feed one navigation status; return a stop request id on leg end."""
        if not self.enabled:
            return ''
        moving = (outcome is None and str(state) not in _MOVING_EXCLUDED_STATES)
        waypoint = str(waypoint or '')
        if moving:
            if not self._active or self._leg_waypoint != waypoint:
                self._active = True
                self._suppressed = False
                self._request_id = 'travel-' + _request_token()
                self._leg_waypoint = waypoint
                self._arrival_pitch = None
                self._arrival_deadline = 0.0
            return ''
        if not self._active:
            return ''
        stop_id = self._request_id
        self._active = False
        self._suppressed = False
        self._leg_waypoint = ''
        return stop_id

    def start_arrival(self, pitch_deg, now):
        self._arrival_pitch = max(0.0, min(45.0, float(pitch_deg)))
        self._arrival_request_id = 'arrival-' + _request_token()
        self._arrival_deadline = now + self.arrival_timeout

    def tick(self, now, backend, arrival_reached=False):
        """Answer one 0.1 s tick with a publish payload dict, or None."""
        if not self.enabled:
            return None
        backend = backend if isinstance(backend, dict) else {}
        if backend.get('dry_run'):
            return None
        if self._active:
            if self._suppressed:
                return None
            return self._decide(now, backend, self._request_id, self.pitch_deg,
                                travel=True)
        if self._arrival_pitch is None:
            return None
        if now >= self._arrival_deadline:
            self._arrival_pitch = None
            return None
        if arrival_reached:
            self._arrival_pitch = None
            return None
        return self._decide(now, backend, self._arrival_request_id,
                            self._arrival_pitch, travel=False)

    def _decide(self, now, backend, request_id, pitch, travel):
        head_following = bool(backend.get('head_follow_enabled', False))
        cancelled_id = str(backend.get('task_head_cancelled_id', '') or '')
        cancel_reason = str(backend.get('task_head_cancel_reason', '') or '')
        if head_following or cancel_reason == 'head_tracking':
            if travel:
                self._suppressed = True
            return None
        if cancel_reason == 'estop' and cancelled_id == request_id:
            return None
        if cancelled_id == request_id:
            # Rejected (hardware_disabled / reset / enabling): the controller
            # ignores the rejected id forever, so rotate it at a bounded
            # rate; the pose engages as soon as the blocker clears.
            if now - self._retry_at < self.retry_interval:
                return None
            self._retry_at = now
            rotated = request_id + ':' + _request_token()
            if travel:
                self._request_id = rotated
            else:
                self._arrival_request_id = rotated
            request_id = rotated
        return {'enabled': True, 'request_id': request_id,
                'head_pitch_deg': float(pitch)}

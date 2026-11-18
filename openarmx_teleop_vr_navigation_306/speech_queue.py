"""Priority queues for the VR speech player.

Safety announcements (front-lidar proximity) jump ahead of ordinary task
prompts, while a single playback worker keeps the two streams from ever
overlapping. Pure Python so it can be unit tested without rclpy.
"""

from __future__ import annotations

import queue


class SpeechQueues:
    def __init__(self, maxsize: int = 24) -> None:
        self._safety: queue.Queue = queue.Queue(maxsize=maxsize)
        self._normal: queue.Queue = queue.Queue(maxsize=maxsize)

    @staticmethod
    def _is_safety(priority) -> bool:
        return str(priority or '').strip().lower() == 'safety'

    def put(self, item, priority: str = 'normal') -> bool:
        target = self._safety if self._is_safety(priority) else self._normal
        try:
            target.put_nowait(item)
            return True
        except queue.Full:
            return False

    def get(self, timeout: float = 0.2):
        try:
            return self._safety.get_nowait()
        except queue.Empty:
            pass
        try:
            return self._normal.get(timeout=timeout)
        except queue.Empty:
            return None

    def clear(self) -> None:
        for target in (self._safety, self._normal):
            while True:
                try:
                    target.get_nowait()
                except queue.Empty:
                    break

    def empty(self) -> bool:
        return self._safety.empty() and self._normal.empty()

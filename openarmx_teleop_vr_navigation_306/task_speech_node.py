"""Sequential local speech output for task workflow events."""

from __future__ import annotations

import asyncio
import json
import queue
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from .speech_queue import SpeechQueues


class TaskSpeechNode(Node):
    def __init__(self):
        super().__init__('openarmx_navigation_task_speech_306')
        self.declare_parameter(
            'input_topic', '/openarmx_teleop_vr_navigation_306/task_speech'
        )
        self.declare_parameter('volume', 1.0)
        self._speech = SpeechQueues(maxsize=24)
        self._engine = None
        self._stop = threading.Event()
        self._enabled = False
        self._generation = 0
        self._engine_ready = False
        self._played = 0
        self._output_volume = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._health = self.create_publisher(String,
            '/openarmx_teleop_vr_navigation_306/speech_health', qos)
        self.create_subscription(String, '/openarmx_teleop_vr_navigation_306/voice_state',
                                 self._on_mode, qos)
        self.create_subscription(
            String, str(self.get_parameter('input_topic').value), self._on_text, 10
        )
        self._worker = threading.Thread(
            target=self._speech_worker, name='task-speech', daemon=True
        )
        self._worker.start()

    def _emit_health(self, state, error=''):
        self._health.publish(String(data=json.dumps({'state':state, 'enabled':self._enabled,
            'engine_ready':self._engine_ready, 'played':self._played, 'last_error':error,
            'output_volume':self._output_volume})))

    def _on_mode(self, message):
        try:
            enabled = json.loads(message.data).get('enabled') is True
        except (TypeError, ValueError, AttributeError):
            enabled = False
        if enabled == self._enabled:
            return
        self._enabled = enabled
        self._generation += 1
        if self._engine is not None:
            self._engine.request_stop()
        self._speech.clear()
        self._emit_health('loading' if enabled else 'disabled')

    def _on_text(self, message):
        if not self._enabled:
            return
        priority = 'normal'
        try:
            payload = json.loads(message.data)
            text = str(payload.get('text', '')).strip()
            priority = str(payload.get('priority', 'normal')).strip().lower()
        except (TypeError, ValueError, AttributeError):
            text = str(message.data).strip()
        if not text:
            return
        if not self._speech.put((self._generation, text, priority), priority):
            self.get_logger().warning('任务语音队列已满，丢弃提示')
            return
        if priority == 'safety' and self._engine is not None:
            # Cut the phrase currently playing; the worker picks the safety
            # item first once playback returns.
            self._engine.request_stop()

    def _speech_worker(self):
        # Do not import/load synthesis models until the independent checkbox
        # is enabled. One worker owns synthesis/playback; no competing routes.
        engine = None
        failed_generation = None
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._emit_health('disabled')
        try:
            while not self._stop.is_set():
                if not self._enabled or failed_generation == self._generation:
                    self._stop.wait(.2)
                    continue
                if engine is None:
                    try:
                        from control_center_service_manager.speaker_volume import ensure_saved_volume
                        self._output_volume = ensure_saved_volume()['volume']
                        from autolife_s2_voice.run.tts.tts_factory import create_tts_engine
                        engine = create_tts_engine(engine='matcha', playback='auto',
                            volume=float(self.get_parameter('volume').value))
                        self._engine = engine
                        self._engine_ready = True
                        self.get_logger().info(f'语音引擎就绪：{engine.engine_name} / {engine.playback_mode}')
                        self._emit_health('ready' if self._enabled else 'disabled')
                    except Exception as error:
                        failed_generation = self._generation
                        self.get_logger().error(f'语音初始化失败：{error}')
                        self._emit_health('failed', str(error))
                        continue
                item = self._speech.get(timeout=0.2)
                if item is None:
                    continue
                generation, text, priority = item
                if not self._enabled or generation != self._generation:
                    continue
                try:
                    from control_center_service_manager.speaker_volume import ensure_saved_volume
                    self._output_volume = ensure_saved_volume()['volume']
                    if self._output_volume == 0:
                        self._emit_health('muted')
                        continue
                    engine.clear_stop()
                    result = loop.run_until_complete(engine.service.asynthesize(text))
                    if not getattr(result, 'success', False) or result.audio is None:
                        raise RuntimeError('语音合成未生成音频，请检查模型与运行环境')
                    if not self._enabled or generation != self._generation:
                        continue
                    if engine.stop_requested and priority != 'safety':
                        # A safety phrase arrived while this one was being
                        # synthesized: drop it and play the safety phrase next.
                        continue
                    self._emit_health('playing')
                    loop.run_until_complete(engine.play_audio(result.audio, result.sample_rate))
                    self._played += 1
                    self._emit_health('ready' if self._enabled else 'disabled')
                except Exception as error:
                    self.get_logger().error(f'任务语音播报失败：{error}')
                    self._emit_health('failed', str(error))
        finally:
            loop.close()

    def destroy_node(self):
        self._stop.set()
        if self._engine is not None:
            self._engine.request_stop()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskSpeechNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

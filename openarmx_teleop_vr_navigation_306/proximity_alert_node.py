#!/usr/bin/env python3
"""Speak a polite warning while an obstacle stays close in front of the robot.

Runs for the whole VR session: unlike the control-center node it does not
require a NavigateToPose goal, so manual thumbstick driving is covered too.
The text is published on the VR task-speech topic so it uses the same player
as the other VR prompts.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .proximity_alert import ProximityDetector


class ProximityAlertNode(Node):
    def __init__(self) -> None:
        super().__init__('openarmx_vr_proximity_alert_306')
        self.declare_parameter('enabled', True)
        self.declare_parameter(
            'front_scan_topic', '/topic_gv_front_lidar_0_283'
        )
        self.declare_parameter(
            'tts_topic', '/openarmx_teleop_vr_navigation_306/task_speech'
        )
        self.declare_parameter(
            'alert_text', '前方有人或障碍物，请注意避让'
        )
        self.declare_parameter('trigger_distance_m', 0.80)
        self.declare_parameter('clear_distance_m', 0.95)
        self.declare_parameter('forward_angle_deg', 180.0)
        self.declare_parameter('half_angle_deg', 40.0)
        self.declare_parameter('minimum_points', 3)
        self.declare_parameter('trigger_consecutive_scans', 3)
        self.declare_parameter('clear_consecutive_scans', 3)
        self.declare_parameter('repeat_while_near_sec', 15.0)
        self.declare_parameter('startup_grace_sec', 4.0)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._alert_text = str(self.get_parameter('alert_text').value).strip()
        self._detector = ProximityDetector(
            trigger_distance_m=self.get_parameter('trigger_distance_m').value,
            clear_distance_m=self.get_parameter('clear_distance_m').value,
            forward_angle_deg=self.get_parameter('forward_angle_deg').value,
            half_angle_deg=self.get_parameter('half_angle_deg').value,
            minimum_points=self.get_parameter('minimum_points').value,
            trigger_consecutive_scans=self.get_parameter(
                'trigger_consecutive_scans').value,
            clear_consecutive_scans=self.get_parameter(
                'clear_consecutive_scans').value,
            repeat_while_near_sec=self.get_parameter(
                'repeat_while_near_sec').value,
            startup_grace_sec=self.get_parameter('startup_grace_sec').value,
            started_at=time.monotonic(),
        )

        laser_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            String, str(self.get_parameter('tts_topic').value), 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('front_scan_topic').value),
            self._handle_scan,
            laser_qos,
        )
        self.get_logger().info(
            '前方避让播报%s：%s，距离 %.2fm/解除 %.2fm，扇区 ±%.1f°，连续 %d 帧，重播 %.1fs'
            % (
                '已启用' if self._enabled else '已禁用',
                self.get_parameter('front_scan_topic').value,
                self._detector.trigger_distance_m,
                self._detector.clear_distance_m,
                self._detector.half_angle * 180.0 / 3.141592653589793,
                self._detector.trigger_scans,
                self._detector.repeat_period,
            )
        )

    def _handle_scan(self, message: LaserScan) -> None:
        if not self._enabled or not self._alert_text:
            return
        fire = self._detector.update(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            time.monotonic(),
        )
        if not fire:
            return
        self._publisher.publish(String(data=json.dumps({
            'text': self._alert_text,
            'priority': 'safety',
        }, ensure_ascii=False)))
        self.get_logger().info(f'已播报前方避让提醒：{self._alert_text}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProximityAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

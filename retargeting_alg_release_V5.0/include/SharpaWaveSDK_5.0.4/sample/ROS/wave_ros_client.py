#!/usr/bin/env python3
"""
WaveRosClient — Client node for controlling the SharpaWave dexterous hand via ROS2

Responsibilities:
  1. Publish joint control commands to ROS2 topic (per hand)
  2. Subscribe to tactile sensor data (raw / force6d / deform), print summary
  3. Subscribe to joint state feedback, print current joint angles
  4. Provide preset poses

Published Topics:
  - wave/{hand}/joint_commands       (sensor_msgs/JointState)     Joint position commands

Subscribed Topics:
  - wave/{hand}/tactile/{finger}/raw       (sensor_msgs/Image)          Tactile raw image
  - wave/{hand}/tactile/{finger}/force6d   (geometry_msgs/WrenchStamped) 6-axis force/torque
  - wave/{hand}/tactile/{finger}/deform    (sensor_msgs/Image)          Deformation image
  - wave/{hand}/joint_states               (sensor_msgs/JointState)     Joint angle feedback

  Where {hand} is: left, right
        {finger} is: thumb, index, middle, ring, pinky

Usage:
  python3 wave_ros_client.py --demo                         # Both hands demo simultaneously
  python3 wave_ros_client.py --hand right --demo            # Right hand demo (index finger)
  python3 wave_ros_client.py --hand left --demo             # Left hand demo (middle finger)
  python3 wave_ros_client.py --hand left --pose open        # Send open-hand pose (left hand)
  python3 wave_ros_client.py --hand right --joints "0.1,0.2,..."  # Send 22 joint angles
  python3 wave_ros_client.py                                # Listen to both hands
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import WrenchStamped
from cv_bridge import CvBridge

import numpy as np

import sys
import argparse
import threading
import math
import time

# ================================================================
#  Constants
# ================================================================

JOINT_NAMES = [
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_DIP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC_FE", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
]

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
HAND_SIDES = ["left", "right"]

# Demo joint config per hand — different fingers so you can tell them apart
DEMO_JOINTS = {
    'right': (8,  'index_DIP'),    # right hand moves index fingertip
    'left':  (12, 'middle_DIP'),   # left hand moves middle fingertip
}


# ================================================================
#  Preset poses (22 joint angles, unit: radians)
# ================================================================
PRESET_POSES = {
    'open': [0.0] * 22,
}


class WaveRosClient(Node):
    """Client node for controlling the dexterous hand via ROS2 topics."""

    def __init__(self, hand='both', verbose=True):
        super().__init__('wave_ros_client')

        self.hand = hand
        self.verbose = verbose
        self.bridge = CvBridge()

        # ---------- Joint command publishers ----------
        self.pub_joint_cmd_per_hand = {}
        if hand == 'both':
            for h in HAND_SIDES:
                self.pub_joint_cmd_per_hand[h] = self.create_publisher(
                    JointState, f'wave/{h}/joint_commands', 10)
        else:
            self.pub_joint_cmd_per_hand[hand] = self.create_publisher(
                JointState, f'wave/{hand}/joint_commands', 10)

        # ---------- Joint state subscriptions (both hands) ----------
        self.latest_joint_states = {}   # "left" -> msg, "right" -> msg
        self._joint_state_lock = threading.Lock()
        self._joint_state_count = 0
        self._state_latency_sum = 0.0
        self._state_latency_count = 0
        self.sub_joint_states = {}

        for h in HAND_SIDES:
            self.sub_joint_states[h] = self.create_subscription(
                JointState, f'wave/{h}/joint_states',
                lambda msg, hh=h: self._on_joint_states(msg, hh), 10)

        # ---------- Tactile data subscriptions (both hands, all fingers) ----------
        self.sub_raw = {}
        self.sub_f6 = {}
        self.sub_deform = {}
        self._raw_counts = {}
        self._f6_counts = {}
        self._deform_counts = {}
        self._force_data = {}

        for h in HAND_SIDES:
            for finger in FINGER_NAMES:
                key = f'{h}/{finger}'
                self._raw_counts[key] = 0
                self._f6_counts[key] = 0
                self._deform_counts[key] = 0
                self.sub_raw[key] = self.create_subscription(
                    Image, f'wave/{h}/tactile/{finger}/raw',
                    lambda msg, k=key: self._on_tactile_raw(msg, k), 2)
                self.sub_f6[key] = self.create_subscription(
                    WrenchStamped, f'wave/{h}/tactile/{finger}/force6d',
                    lambda msg, k=key: self._on_tactile_force(msg, k), 2)
                self.sub_deform[key] = self.create_subscription(
                    Image, f'wave/{h}/tactile/{finger}/deform',
                    lambda msg, k=key: self._on_tactile_deform(msg, k), 2)

        self.get_logger().info(
            f'WaveRosClient initialized  |  control: {hand}  |  '
            f'listening: both hands  |  tactile: {FINGER_NAMES}')

    # ================================================================
    #  Connection check
    # ================================================================
    def check_server_connection(self) -> bool:
        """Check if the Server is listening. Returns True if any subscribers exist."""
        found = False
        for h, pub in self.pub_joint_cmd_per_hand.items():
            sub_count = pub.get_subscription_count()
            if sub_count > 0:
                self.get_logger().info(
                    f'Server connected  |  {sub_count} subscriber(s) on '
                    f'wave/{h}/joint_commands')
                found = True
            else:
                self.get_logger().warn(
                    f'No subscribers on wave/{h}/joint_commands')
        if not found:
            self.get_logger().warn(
                'Is the Server running? (python3 wave_ros_server.py)')
        return found

    # ================================================================
    #  Send joint commands
    # ================================================================
    def send_joint_command(self, positions, names=None, hand=None):
        """Send joint position commands (radians) to the specified hand."""
        if names is None:
            names = JOINT_NAMES[:len(positions)]
        if hand is None:
            hand = self.hand

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(p) for p in positions]
        msg.velocity = []
        msg.effort = []

        if hand == 'both':
            for h, pub in self.pub_joint_cmd_per_hand.items():
                pub.publish(msg)
            self.get_logger().info(
                f'[Send] both hands  |  joints: {len(positions)}  |  '
                f'first 5: {[round(p, 3) for p in positions[:5]]}')
        else:
            pub = self.pub_joint_cmd_per_hand.get(hand)
            if pub:
                pub.publish(msg)
            self.get_logger().info(
                f'[Send] {hand} hand  |  joints: {len(positions)}  |  '
                f'first 5: {[round(p, 3) for p in positions[:5]]}')

    def send_preset_pose(self, pose_name, steps=1500, step_delay=0.001):
        """Send a preset pose with smooth interpolation: open"""
        if pose_name not in PRESET_POSES:
            self.get_logger().error(
                f'Unknown pose: {pose_name}  |  '
                f'available: {", ".join(PRESET_POSES.keys())}')
            return

        target = PRESET_POSES[pose_name]
        hands = list(self.pub_joint_cmd_per_hand.keys())

        starts = {}
        for h in hands:
            with self._joint_state_lock:
                state = self.latest_joint_states.get(h)
                s = list(state.position) if state is not None else [0.0] * 22
            if len(s) < 22:
                s = s + [0.0] * (22 - len(s))
            starts[h] = s

        duration = steps * step_delay
        self.get_logger().info(
            f'[Pose] {", ".join(hands)} hand(s)  |  {pose_name}  ({duration:.1f}s, {steps} steps)')

        for step in range(1, steps + 1):
            t = step / steps
            for h in hands:
                t_smooth = t * t * t * (t * (t * 6 - 15) + 10)
                positions = [s + (g - s) * t_smooth for s, g in zip(starts[h], target)]
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = JOINT_NAMES[:len(positions)]
                msg.position = [float(p) for p in positions]
                dt = step_delay
                velocities = [(g - s) * 30 * t * t * (t - 1) * (t - 1) / dt for s, g in zip(starts[h], target)]
                msg.velocity = [float(v) for v in velocities]
                msg.effort = []
                self.pub_joint_cmd_per_hand[h].publish(msg)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(step_delay)

        self.get_logger().info(f'[Pose] {pose_name} complete')

    def send_single_joint(self, joint_index, angle_rad, hand=None):
        """Modify a single joint angle while keeping others at current position or zero."""
        if hand is None:
            hand = self.hand if self.hand != 'both' else 'right'
        with self._joint_state_lock:
            state = self.latest_joint_states.get(hand)
            if state is not None:
                positions = list(state.position)
            else:
                positions = [0.0] * 22

        if 0 <= joint_index < len(positions):
            positions[joint_index] = angle_rad
            self.get_logger().info(
                f'[Single joint] {hand} hand  |  '
                f'{JOINT_NAMES[joint_index]} -> {angle_rad:.4f} rad '
                f'({math.degrees(angle_rad):.1f} deg)')
            self.send_joint_command(positions, hand=hand)
        else:
            self.get_logger().error(
                f'Joint index out of range: {joint_index} (valid: 0-21)')

    # ================================================================
    #  Joint state callback
    # ================================================================
    def _on_joint_states(self, msg: JointState, hand: str):
        """Receive and print joint state feedback."""
        with self._joint_state_lock:
            self.latest_joint_states[hand] = msg

        # Measure Server -> Client latency from message stamp
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        latency_ms = (now - msg_time).nanoseconds / 1e6
        self._state_latency_sum += latency_ms
        self._state_latency_count += 1

        self._joint_state_count += 1
        if self.verbose and self._joint_state_count % 10 == 1:
            avg_ms = self._state_latency_sum / max(self._state_latency_count, 1)
            angles_deg = [round(math.degrees(a), 1) for a in msg.position[:5]]
            self.get_logger().info(
                f'[Joint State] {hand} hand  |  SN: {msg.header.frame_id}  |  '
                f'latency: {latency_ms:.1f} ms (avg: {avg_ms:.1f} ms)  |  '
                f'first 5 (deg): {angles_deg}')

    # ================================================================
    #  Tactile data callbacks (print summary)
    # ================================================================
    def _on_tactile_raw(self, msg: Image, key: str):
        """Receive tactile raw image, periodically print summary."""
        self._raw_counts[key] += 1
        if self.verbose and self._raw_counts[key] % 50 == 1:
            img = self.bridge.imgmsg_to_cv2(msg)
            self.get_logger().info(
                f'[Tactile RAW] {key}  |  '
                f'count: {self._raw_counts[key]}  |  '
                f'size: {msg.width}x{msg.height}  |  '
                f'pixel min={int(img.min())} max={int(img.max())} '
                f'mean={img.mean():.1f}')

    def _on_tactile_force(self, msg: WrenchStamped, key: str):
        """Receive 6-axis force data, periodically print."""
        f = msg.wrench.force
        t = msg.wrench.torque
        self._force_data[key] = {
            'force': (f.x, f.y, f.z),
            'torque': (t.x, t.y, t.z),
        }
        self._f6_counts[key] += 1
        if self.verbose and self._f6_counts[key] % 50 == 1:
            self.get_logger().info(
                f'[Tactile F6D] {key}  |  '
                f'count: {self._f6_counts[key]}  |  '
                f'F=({f.x:.3f}, {f.y:.3f}, {f.z:.3f})  '
                f'T=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')

    def _on_tactile_deform(self, msg: Image, key: str):
        """Receive deformation image, periodically print summary."""
        self._deform_counts[key] += 1
        if self.verbose and self._deform_counts[key] % 50 == 1:
            img = self.bridge.imgmsg_to_cv2(msg)
            self.get_logger().info(
                f'[Tactile DEFORM] {key}  |  '
                f'count: {self._deform_counts[key]}  |  '
                f'size: {msg.width}x{msg.height}  |  '
                f'min={img.min():.3f} max={img.max():.3f} '
                f'mean={img.mean():.3f}')

    # ================================================================
    #  Query interface
    # ================================================================
    def get_joint_states(self, hand=None):
        """Get the most recent joint state feedback. hand=None uses self.hand."""
        if hand is None:
            hand = self.hand if self.hand != 'both' else 'right'
        with self._joint_state_lock:
            return self.latest_joint_states.get(hand)

    def get_force_data(self, key=None):
        """Get the most recent force data. key='{hand}/{finger}' or None for all."""
        if key is not None:
            return self._force_data.get(key)
        return dict(self._force_data)


def _build_joint_msg(node, positions):
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = JOINT_NAMES[:len(positions)]
    msg.position = [float(p) for p in positions]
    msg.velocity = []
    msg.effort = []
    return msg


def main():
    parser = argparse.ArgumentParser(
        description='WaveRosClient — ROS2 dexterous hand control client')
    parser.add_argument('--hand', type=str, default='both',
                        choices=['left', 'right', 'both'],
                        help='Target hand for control commands (default: both)')
    parser.add_argument('--pose', type=str, default='',
                        help='Send a preset pose then exit (open)')
    parser.add_argument('--joints', type=str, default='',
                        help='Send joint angles (comma-separated 22 radian values) then exit')
    parser.add_argument('--demo', action='store_true', default=False,
                        help='Run demo: right=index DIP, left=middle DIP (both if --hand both)')
    parser.add_argument('--quiet', action='store_true', default=False,
                        help='Quiet mode, reduce log output')
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = WaveRosClient(hand=args.hand, verbose=not args.quiet)

    # Wait briefly for ROS2 discovery before checking connection
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.1)

    # Mode 1: Send preset pose then exit
    if args.pose:
        node.check_server_connection()
        node.send_preset_pose(args.pose)
        node.destroy_node()
        rclpy.shutdown()
        return

    # Mode 2: Send custom joint angles then exit
    if args.joints:
        node.check_server_connection()
        vals = [float(v) for v in args.joints.split(',')]
        node.send_joint_command(vals)
        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info('Command sent, exiting')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Mode 3: Demo mode
    if args.demo:
        if not node.check_server_connection():
            node.get_logger().error(
                'Demo aborted: Server not detected. '
                'Start the Server first (python3 wave_ros_server.py)')
            node.destroy_node()
            rclpy.shutdown()
            return

        demo_hands = list(node.pub_joint_cmd_per_hand.keys())
        demo_config = {h: DEMO_JOINTS[h] for h in demo_hands}

        target_rad = math.radians(60)
        steps = 30
        step_delay = 0.05

        desc = ', '.join(f'{h}: {name}' for h, (_, name) in demo_config.items())
        node.get_logger().info(
            f'[Demo] {desc}  |  0 -> 60 deg  '
            f'({steps} steps, {step_delay}s/step)')

        # Phase 1: 0 -> 60 deg
        for i in range(steps + 1):
            angle = target_rad * i / steps
            for h, (joint_idx, joint_name) in demo_config.items():
                positions = [0.0] * 22
                positions[joint_idx] = angle
                node.pub_joint_cmd_per_hand[h].publish(
                    _build_joint_msg(node, positions))
            rclpy.spin_once(node, timeout_sec=0)
            if i % 10 == 0:
                node.get_logger().info(f'  angle = {math.degrees(angle):.1f} deg')
            time.sleep(step_delay)

        # Hold 2 seconds
        node.get_logger().info('[Demo] Holding position for 2 seconds...')
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)

        for h, (joint_idx, joint_name) in demo_config.items():
            state = node.get_joint_states(hand=h)
            if state and len(state.position) > joint_idx:
                actual = math.degrees(state.position[joint_idx])
                node.get_logger().info(
                    f'[Demo] {h} hand {joint_name}  |  '
                    f'target: 60.0 deg  actual: {actual:.1f} deg')
            else:
                node.get_logger().warn(
                    f'[Demo] No joint state feedback from {h} hand  |  '
                    f'Is the device connected?')

        # Phase 2: 60 -> 0 deg
        node.get_logger().info(f'[Demo] 60 -> 0 deg')
        for i in range(steps + 1):
            angle = target_rad * (1.0 - i / steps)
            for h, (joint_idx, joint_name) in demo_config.items():
                positions = [0.0] * 22
                positions[joint_idx] = angle
                node.pub_joint_cmd_per_hand[h].publish(
                    _build_joint_msg(node, positions))
            rclpy.spin_once(node, timeout_sec=0)
            if i % 10 == 0:
                node.get_logger().info(f'  angle = {math.degrees(angle):.1f} deg')
            time.sleep(step_delay)

        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info('[Demo] Complete')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Default: Continuous listening mode (receive tactile data and joint states)
    node.get_logger().info(
        'WaveRosClient continuous listening mode  |  Press Ctrl+C to exit')
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

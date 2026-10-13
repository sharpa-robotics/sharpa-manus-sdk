#!/usr/bin/env python3
"""
WaveRosServer — SharpaWave SDK <-> ROS2 bridge service node

Responsibilities:
  1. Connect to SharpaWave dexterous hand devices (via SharpaWaveSDK)
  2. Automatically enable devices on startup (set_enable_state(True))
  3. Publish tactile sensor data (raw / force6d / deform) to ROS2 topics
  4. Subscribe to joint control command topics and forward to devices
  5. Periodically publish joint state feedback

Published Topics (left/right separated):
  - wave/{hand}/tactile/{finger}/raw      (sensor_msgs/Image)          Tactile raw image
  - wave/{hand}/tactile/{finger}/force6d  (geometry_msgs/WrenchStamped) 6-axis force/torque
  - wave/{hand}/tactile/{finger}/deform   (sensor_msgs/Image)          Deformation image (mono8, same as sample/python/sharpa_tactile_callback.py)
  - wave/{hand}/joint_states              (sensor_msgs/JointState)     Joint angle feedback

  Where {hand} is: left, right
        {finger} is: thumb, index, middle, ring, pinky

Subscribed Topics:
  - wave/{hand}/joint_commands          (sensor_msgs/JointState)     Joint position commands

Usage:
  python3 wave_ros_server.py
  python3 wave_ros_server.py --feedback_hz 10
  python3 wave_ros_server.py --channels 0-9
  python3 wave_ros_server.py --watchdog_interval 5
"""

import sys
import os
import ctypes
import time
import argparse
import threading

sys.path.insert(0, '/opt/sharpa-wave-sdk/python')

# Preload system libfmt to prevent conflict between SDK's libfmt.so.10 and
# ROS2 Humble's libfmt.so.8 — both loaded via spdlog, causing SIGSEGV in
# Node.__init__ when symbols from incompatible versions collide.
try:
    ctypes.CDLL("libfmt.so.8", mode=ctypes.RTLD_GLOBAL)
except OSError:
    pass

from sharpa import SharpaWaveManager, HandSide, DeviceType, ControlSource

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import WrenchStamped
from cv_bridge import CvBridge

import numpy as np

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

CHANNEL_TO_FINGER = {
    0: "thumb",  1: "index",  2: "middle",  3: "ring",  4: "pinky",
    5: "thumb",  6: "index",  7: "middle",  8: "ring",  9: "pinky",
}

CHANNEL_TO_HAND = {
    0: "right", 1: "right", 2: "right", 3: "right", 4: "right",
    5: "left",  6: "left",  7: "left",  8: "left",  9: "left",
}


def channel_to_topic_prefix(ch: int) -> str:
    """Convert channel number to topic prefix: wave/{hand}/tactile/{finger}"""
    hand = CHANNEL_TO_HAND.get(ch, "unknown")
    finger = CHANNEL_TO_FINGER.get(ch, f"ch_{ch}")
    return f'wave/{hand}/tactile/{finger}'


def device_has_tactile(device_info) -> bool:
    checker = getattr(device_info, "has_fingertip_tactile", None)
    if callable(checker):
        return bool(checker())
    return bool(checker)


class WaveRosServer(Node):
    """SharpaWave SDK to ROS2 bridge service node."""

    def __init__(self, channels=range(10), feedback_hz=10.0):
        super().__init__('wave_ros_server')

        self.channels = list(channels)
        self.bridge = CvBridge()

        # ---------- Tactile data publishers (per hand per finger) ----------
        qos = 2
        self.pub_raw = {}
        self.pub_f6 = {}
        self.pub_deform = {}

        for ch in self.channels:
            prefix = channel_to_topic_prefix(ch)
            self.pub_raw[ch] = self.create_publisher(
                Image, f'{prefix}/raw', qos)
            self.pub_f6[ch] = self.create_publisher(
                WrenchStamped, f'{prefix}/force6d', qos)
            self.pub_deform[ch] = self.create_publisher(
                Image, f'{prefix}/deform', qos)

        # ---------- Joint state publishers (per hand) ----------
        self.pub_joint_states = {}
        for hand in ('left', 'right'):
            self.pub_joint_states[hand] = self.create_publisher(
                JointState, f'wave/{hand}/joint_states', 10)

        # ---------- Joint command subscribers (per hand) ----------
        self._wave_by_hand = {}   # "left" -> wave, "right" -> wave
        self.sub_joint_cmd = {}
        for hand in ('left', 'right'):
            self.sub_joint_cmd[hand] = self.create_subscription(
                JointState, f'wave/{hand}/joint_commands',
                lambda msg, h=hand: self._on_joint_command(msg, h), 10)

        # ---------- Device management ----------
        self.manager = None
        self.waves = []
        self.device_sns = []
        self._device_hands = {}   # sn -> "left" / "right"
        self._device_lock = threading.Lock()

        # ---------- Timestamp baseline ----------
        self._tactile_t0 = time.time_ns() * 1e-9
        self._ros_t0 = self.get_clock().now()

        # ---------- Joint state feedback timer ----------
        if feedback_hz > 0:
            period = 1.0 / feedback_hz
            self.feedback_timer = self.create_timer(
                period, self._publish_joint_states)

        # ---------- Statistics ----------
        self._tactile_frame_count = 0
        self._joint_cmd_count = 0
        self._cmd_latency_sum = 0.0
        self._cmd_latency_count = 0

        # ---------- Auto-reconnect watchdog ----------
        self._reconnect_interval = 3.0
        self._max_fail_before_reconnect = 3
        self._consecutive_missing = {}   # sn -> consecutive "not in discovery list" count
        self._reconnecting = False

        # ---------- Rate-limit for "no hand" warnings ----------
        self._no_hand_warn_count = {}    # hand -> count
        self._no_hand_warn_interval = 50

        self.get_logger().info(
            f'WaveRosServer initialized  |  '
            f'tactile channels: {self.channels}  |  feedback: {feedback_hz} Hz')

    # ================================================================
    #  Device connection
    # ================================================================
    def connect_devices(self):
        """Block and wait for all HAND-type SharpaWave devices to connect."""
        self.get_logger().info('Waiting for SharpaWave devices...')
        self.manager = SharpaWaveManager.get_instance()
        time.sleep(1)

        device_infos = self.manager.get_all_devices()
        while not device_infos or not any(
            info.device_type == DeviceType.HAND for info in device_infos
        ):
            self.get_logger().info('Waiting for device connection...')
            time.sleep(1)
            device_infos = self.manager.get_all_devices()

        device_infos = [
            info for info in device_infos
            if info.device_type == DeviceType.HAND
        ]
        self.get_logger().info(
            f'Found {len(device_infos)} HAND device(s): '
            f'{[info.sn for info in device_infos]}')

        for i, info in enumerate(device_infos):
            self.get_logger().info(f'Connecting device {i}: {info.sn}')
            wave = self.manager.connect(info.sn)
            connected_info = wave.get_device_info()
            if not device_has_tactile(connected_info):
                raise RuntimeError(
                    f'Device {info.sn} firmware does not support tactile feature')

            # Detect hand side from SDK
            hand = "right"
            if hasattr(info, 'hand_side'):
                hand = "left" if info.hand_side == HandSide.LEFT else "right"
            elif hasattr(wave, 'get_hand_side'):
                hs = wave.get_hand_side()
                hand = "left" if hs == HandSide.LEFT else "right"
            self._device_hands[info.sn] = hand

            err = wave.set_control_source(ControlSource.SDK)
            if err.code == 0:
                self.get_logger().info(f'Device {info.sn} ({hand}) control_source -> SDK')
            else:
                self.get_logger().warn(
                    f'Device {info.sn} set_control_source failed: {err.message}')

            err = wave.set_enable_state(True)
            if err.code == 0:
                self.get_logger().info(f'Device {info.sn} ({hand}) enable_state -> True')
            else:
                self.get_logger().warn(
                    f'Device {info.sn} set_enable_state failed: {err.message}')
            self.waves.append(wave)
            self.device_sns.append(info.sn)
            self._wave_by_hand[hand] = wave
            self.get_logger().info(f'Device {i} ({info.sn}) connected as {hand} hand')

        for i, wave in enumerate(self.waves):
            wave.set_tactile_callback(
                lambda data, sn=self.device_sns[i]:
                    self._on_tactile_data(data, sn))

        for i, wave in enumerate(self.waves):
            if wave.start():
                self.get_logger().info(
                    f'Device {i} ({self.device_sns[i]}) started successfully')
            else:
                self.get_logger().error(
                    f'Device {i} ({self.device_sns[i]}) failed to start')

        hands_connected = list(self._wave_by_hand.keys())
        self.get_logger().info(
            f'All devices ready  |  hands: {hands_connected}  |  '
            f'Publishing: wave/{{hand}}/tactile/{{finger}}/*, wave/{{hand}}/joint_states  |  '
            f'Subscribing: wave/{{hand}}/joint_commands')

    def disconnect_devices(self):
        """Disconnect all devices."""
        for wave in self.waves:
            try:
                wave.set_enable_state(False)
                wave.stop()
                wave.destroy()
            except Exception as e:
                self.get_logger().warn(f'Error stopping device: {e}')
        if self.manager:
            self.manager.disconnect_all()
        self.waves.clear()
        self.device_sns.clear()
        self._wave_by_hand.clear()
        self._device_hands.clear()
        self._consecutive_missing.clear()
        self.get_logger().info('All devices disconnected')

    # ================================================================
    #  Auto-reconnect watchdog
    # ================================================================
    def start_watchdog(self):
        """Start a periodic timer to check device health."""
        self._watchdog_timer = self.create_timer(
            self._reconnect_interval, self._watchdog_check)
        self.get_logger().info(
            f'Watchdog started  |  check every {self._reconnect_interval}s  |  '
            f'reconnect after {self._max_fail_before_reconnect} consecutive failures')

    def _watchdog_check(self):
        """Periodically check device health via manager discovery list."""
        if self._reconnecting or not self.device_sns:
            return

        try:
            discovered = self.manager.get_all_devices() if self.manager else []
        except Exception:
            discovered = []
        discovered_sns = set(
            info.sn for info in (discovered or [])
            if info.device_type == DeviceType.HAND
        )

        dead_sns = []
        for sn in list(self.device_sns):
            if sn in discovered_sns:
                self._consecutive_missing[sn] = 0
            else:
                cnt = self._consecutive_missing.get(sn, 0) + 1
                self._consecutive_missing[sn] = cnt
                if cnt == 1:
                    self.get_logger().warn(
                        f'[Watchdog] Device {sn} not in discovery list (1/{self._max_fail_before_reconnect})')
                if cnt >= self._max_fail_before_reconnect:
                    dead_sns.append(sn)

        if not dead_sns:
            return

        for sn in dead_sns:
            hand = self._device_hands.get(sn, '?')
            self.get_logger().warn(
                f'[Watchdog] Device {sn} ({hand}) offline for '
                f'{self._consecutive_missing[sn]} checks, triggering reconnect...')

        self._reconnecting = True
        threading.Thread(target=self._do_reconnect, daemon=True).start()

    def _do_reconnect(self):
        """Background thread: disconnect all, wait for devices, reconnect."""
        try:
            self.get_logger().info('[Watchdog] Disconnecting stale devices...')
            with self._device_lock:
                for wave in self.waves:
                    try:
                        wave.set_enable_state(False)
                        wave.stop()
                        wave.destroy()
                    except Exception:
                        pass
                if self.manager:
                    self.manager.disconnect_all()
                self.waves.clear()
                self.device_sns.clear()
                self._wave_by_hand.clear()
                self._device_hands.clear()
                self._consecutive_missing.clear()
                self._no_hand_warn_count.clear()

            self.get_logger().info('[Watchdog] Waiting for devices to come back online...')
            if self.manager is None:
                self.manager = SharpaWaveManager.get_instance()

            wait_count = 0
            while rclpy.ok():
                time.sleep(2)
                device_infos = self.manager.get_all_devices()
                hands_found = [
                    info for info in (device_infos or [])
                    if info.device_type == DeviceType.HAND
                ]
                if hands_found:
                    self.get_logger().info(
                        f'[Watchdog] Found {len(hands_found)} device(s), reconnecting...')
                    time.sleep(1)
                    break
                wait_count += 1
                if wait_count % 5 == 1:
                    self.get_logger().info('[Watchdog] Still waiting for devices...')

            self._reconnect_devices()
        except Exception as e:
            self.get_logger().error(f'[Watchdog] Reconnect failed: {e}')
        finally:
            self._reconnecting = False

    def _reconnect_devices(self):
        """Re-run device connection logic (called from watchdog thread)."""
        device_infos = self.manager.get_all_devices()
        device_infos = [
            info for info in (device_infos or [])
            if info.device_type == DeviceType.HAND
        ]
        if not device_infos:
            self.get_logger().warn('[Watchdog] No HAND devices found after wait')
            return

        with self._device_lock:
            for i, info in enumerate(device_infos):
                self.get_logger().info(f'[Watchdog] Connecting device {i}: {info.sn}')
                wave = self.manager.connect(info.sn)
                connected_info = wave.get_device_info()
                if not device_has_tactile(connected_info):
                    self.get_logger().error(
                        f'Device {info.sn} firmware does not support tactile feature, shutting down.')
                    if rclpy.ok():
                        rclpy.shutdown()
                    return

                hand = "right"
                if hasattr(info, 'hand_side'):
                    hand = "left" if info.hand_side == HandSide.LEFT else "right"
                elif hasattr(wave, 'get_hand_side'):
                    hs = wave.get_hand_side()
                    hand = "left" if hs == HandSide.LEFT else "right"
                self._device_hands[info.sn] = hand

                err = wave.set_control_source(ControlSource.SDK)
                if err.code == 0:
                    self.get_logger().info(
                        f'[Watchdog] {info.sn} ({hand}) control_source -> SDK')

                err = wave.set_enable_state(True)
                if err.code == 0:
                    self.get_logger().info(
                        f'[Watchdog] {info.sn} ({hand}) enable_state -> True')

                self.waves.append(wave)
                self.device_sns.append(info.sn)
                self._wave_by_hand[hand] = wave

            for i, wave in enumerate(self.waves):
                wave.set_tactile_callback(
                    lambda data, sn=self.device_sns[i]:
                        self._on_tactile_data(data, sn))

            for i, wave in enumerate(self.waves):
                if wave.start():
                    self.get_logger().info(
                        f'[Watchdog] Device {self.device_sns[i]} started')
                else:
                    self.get_logger().error(
                        f'[Watchdog] Device {self.device_sns[i]} failed to start')

        hands_connected = list(self._wave_by_hand.keys())
        self.get_logger().info(
            f'[Watchdog] Reconnect complete  |  hands: {hands_connected}')

    # ================================================================
    #  Tactile data callback -> publish to ROS2
    # ================================================================
    def _on_tactile_data(self, frame, sn):
        """SDK tactile callback: parse like sharpa_tactile_callback.callback, publish to ROS2."""
        try:
            ch = frame["channel"]
            ts = frame["ts"]
            content = frame.get("content")
            if content is None:
                return

            elapsed = ts - self._tactile_t0
            stamp = (self._ros_t0 + rclpy.time.Duration(seconds=elapsed)).to_msg()
            hand = CHANNEL_TO_HAND.get(ch, "unknown")
            finger = CHANNEL_TO_FINGER.get(ch, f"ch_{ch}")
            header = Header()
            header.stamp = stamp
            header.frame_id = f'wave/{hand}/tactile/{finger}'

            # Process RAW image if available (sharpa_tactile_callback.py)
            img_reshaped = None
            raw_data = content.get("RAW")
            if raw_data is not None:
                img = raw_data
                img = img.squeeze()
                height = frame["shape"]["RAW"][1]
                width = frame["shape"]["RAW"][2]
                img_reshaped = img.reshape(height, width).astype(np.uint8)

            f, deform_reshaped, contact_points = None, None, None
            f6_data = content.get("F6")
            deform_data = content.get("DEFORM")
            contact_point_data = content.get("CONTACT_POINT")
            if f6_data is not None and deform_data is not None:
                f = f6_data
                deform = deform_data
                height = frame["shape"]["DEFORM"][1]
                width = frame["shape"]["DEFORM"][2]
                deform_reshaped = deform.reshape(height, width).astype(np.uint8)

            if contact_point_data is not None:
                try:
                    if isinstance(contact_point_data, list):
                        contact_points = np.array(contact_point_data, dtype=np.float32)
                    else:
                        contact_points = np.asarray(contact_point_data, dtype=np.float32)

                    contact_points = contact_points.squeeze()
                    if contact_points.size > 0:
                        contact_points = contact_points.reshape(
                            -1,
                            contact_points.shape[-1] if contact_points.ndim > 1 else 3)
                except Exception as e:
                    self.get_logger().warn(
                        f"Error processing CONTACT_POINT data: {e}")
                    contact_points = None

            if img_reshaped is not None and ch in self.pub_raw:
                msg = self.bridge.cv2_to_imgmsg(
                    img_reshaped, encoding='mono8', header=header)
                self.pub_raw[ch].publish(msg)

            if f is not None and deform_reshaped is not None:
                if ch in self.pub_f6:
                    f_arr = np.asarray(f, dtype=float).flatten()
                    if f_arr.size >= 6:
                        msg = WrenchStamped(header=header)
                        msg.wrench.force.x = float(f_arr[0])
                        msg.wrench.force.y = float(f_arr[1])
                        msg.wrench.force.z = float(f_arr[2])
                        msg.wrench.torque.x = float(f_arr[3])
                        msg.wrench.torque.y = float(f_arr[4])
                        msg.wrench.torque.z = float(f_arr[5])
                        self.pub_f6[ch].publish(msg)
                if ch in self.pub_deform:
                    msg = self.bridge.cv2_to_imgmsg(
                        deform_reshaped, encoding='mono8', header=header)
                    self.pub_deform[ch].publish(msg)

            _ = contact_points  # parsed for parity with sharpa_tactile_callback (no ROS topic yet)

            self._tactile_frame_count += 1
            if self._tactile_frame_count % 500 == 0:
                self.get_logger().info(
                    f'[Tactile] Published {self._tactile_frame_count} frames  '
                    f'(latest: {hand}/{finger}, sn={sn})')
        except Exception as e:
            self.get_logger().warn(f"Tactile callback processing error: {e}")

    # ================================================================
    #  Joint command subscription -> forward to device
    # ================================================================
    def _on_joint_command(self, msg: JointState, hand: str):
        """Receive joint position commands and forward to the specified hand."""
        wave = self._wave_by_hand.get(hand)
        if wave is None:
            cnt = self._no_hand_warn_count.get(hand, 0) + 1
            self._no_hand_warn_count[hand] = cnt
            if cnt <= 1 or cnt % self._no_hand_warn_interval == 0:
                self.get_logger().warn(
                    f'No {hand} hand connected, ignoring joint commands '
                    f'(suppressed {cnt} msgs)')
            return

        # Measure Client -> Server latency from message stamp
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        latency_ms = (now - msg_time).nanoseconds / 1e6
        self._cmd_latency_sum += latency_ms
        self._cmd_latency_count += 1

        positions = [float(p) for p in msg.position]

        with self._device_lock:
            try:
                wave.set_joint_position(positions)
            except Exception as e:
                self.get_logger().error(f'Failed to send joint command to {hand}: {e}')

        self._joint_cmd_count += 1
        if self._joint_cmd_count <= 3 or self._joint_cmd_count % 50 == 0:
            avg_ms = self._cmd_latency_sum / max(self._cmd_latency_count, 1)
            self.get_logger().info(
                f'[Joint] {hand} hand  |  cmd #{self._joint_cmd_count}  |  '
                f'latency: {latency_ms:.1f} ms (avg: {avg_ms:.1f} ms)  |  '
                f'positions (first 9): {[round(p, 3) for p in positions[:9]]}')

    # ================================================================
    #  Periodic joint state feedback publishing
    # ================================================================
    def _publish_joint_states(self):
        """Periodically read device joint angles and publish per hand."""
        if not self.waves:
            return

        with self._device_lock:
            for idx, wave in enumerate(self.waves):
                try:
                    state = wave.get_states()
                    angles = list(state.angles) if hasattr(state, 'angles') else []
                except Exception as e:
                    self.get_logger().debug(f'Failed to read device {idx} state: {e}')
                    continue

                sn = self.device_sns[idx] if idx < len(self.device_sns) else ''
                hand = self._device_hands.get(sn, 'right')

                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = sn
                msg.name = JOINT_NAMES[:len(angles)]
                msg.position = [float(a) for a in angles]
                msg.velocity = []
                msg.effort = []

                if hand in self.pub_joint_states:
                    self.pub_joint_states[hand].publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description='WaveRosServer — SharpaWave SDK <-> ROS2 bridge')
    parser.add_argument('--feedback_hz', type=float, default=10.0,
                        help='Joint state feedback frequency in Hz (default: 10, set 0 to disable)')
    parser.add_argument('--channels', type=str, default='0-9',
                        help='Tactile channel range (default: 0-9, covers both hands)')
    parser.add_argument('--watchdog_interval', type=float, default=3.0,
                        help='Device health check interval in seconds (default: 3.0)')
    args = parser.parse_args()

    if '-' in args.channels:
        start, end = args.channels.split('-')
        channels = range(int(start), int(end) + 1)
    else:
        channels = [int(c) for c in args.channels.split(',')]

    rclpy.init(args=sys.argv)
    node = WaveRosServer(
        channels=channels,
        feedback_hz=args.feedback_hz,
    )
    node._reconnect_interval = args.watchdog_interval

    try:
        node.connect_devices()
        node.start_watchdog()
        node.get_logger().info('WaveRosServer running... Press Ctrl+C to exit')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupt received, shutting down...')
    finally:
        node.disconnect_devices()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

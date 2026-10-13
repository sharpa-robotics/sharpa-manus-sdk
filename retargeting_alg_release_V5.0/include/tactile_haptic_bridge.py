"""
WAVE tactile (F6) -> ZMQ HapticFeedback publisher for Manus Client haptics.

Data path:
  SharpaWaveSDK set_tactile_callback (per channel, ~30Hz)
    -> per-finger force magnitude (no filtering)
    -> immediate ZMQ publish on each tactile frame

Channel layout (WAVE tactile channel order):
  ch 0-4: right pinky, ring, middle, index, thumb
  ch 5-9: left pinky, ring, middle, index, thumb
  Mapped to Manus order: thumb, index, middle, ring, pinky
"""

from __future__ import annotations

import os
import socket
import sys
import time
import threading
from typing import List, Optional, Tuple

import numpy as np
import zmq

# Proto lives next to this module's sibling proto_hand/
_INCLUDE_DIR = os.path.dirname(os.path.abspath(__file__))
if _INCLUDE_DIR not in sys.path:
    sys.path.insert(0, _INCLUDE_DIR)
_PROTO_DIR = os.path.join(_INCLUDE_DIR, "proto_hand")
if _PROTO_DIR not in sys.path:
    sys.path.insert(0, _PROTO_DIR)

import sharpa_hand_pb2  # noqa: E402

# Thumb, Index, Middle, Ring, Pinky — matches Manus CoreSdk_VibrateFingers order
FINGER_NAMES: List[str] = ["thumb", "index", "middle", "ring", "pinky"]

# WAVE tactile channel -> Manus finger index (thumb=0 .. pinky=4)
# Channel order per hand: pinky(0), ring(1), middle(2), index(3), thumb(4)
CHANNEL_TO_FINGER_IDX = {
    0: 4, 1: 3, 2: 2, 3: 1, 4: 0,
    5: 4, 6: 3, 7: 2, 8: 1, 9: 0,
}
CHANNEL_TO_HAND = {
    0: "right", 1: "right", 2: "right", 3: "right", 4: "right",
    5: "left", 6: "left", 7: "left", 8: "left", 9: "left",
}

_DEFAULT_HAPTIC_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "haptic_feedback.yaml"
)


def default_haptic_config_path() -> str:
    """Canonical haptic_feedback.yaml next to this module."""
    env_path = os.environ.get("HAPTIC_FEEDBACK_CONFIG", "").strip()
    return env_path or _DEFAULT_HAPTIC_CONFIG_PATH


def load_bridge_settings_from_yaml(config_path: Optional[str] = None) -> dict:
    """
    Load retargeting bridge + shared deadzone from haptic_feedback.yaml.
    Thresholds/PWM are consumed by SharpaManusLinuxClient (C++).
    """
    path = config_path or default_haptic_config_path()
    settings = {
        "deadzone_n": 0.8,
        "publish_hz": 30.0,
        "debug_interval_sec": 1.0,
        "finger_stale_timeout_sec": 0.08,
        "zero_burst_count": 5,
    }
    if not path or not os.path.isfile(path):
        return settings

    section = ""
    try:
        with open(path, "r", encoding="utf-8") as f:
            for raw_line in f:
                line = raw_line.split("#", 1)[0].strip()
                if not line:
                    continue
                if line.endswith(":") and ":" not in line[:-1]:
                    section = line[:-1].strip()
                    continue
                if ":" not in line:
                    continue
                key, value = (p.strip() for p in line.split(":", 1))
                if not value:
                    continue
                if section == "thresholds" and key == "deadzone_n":
                    settings["deadzone_n"] = float(value)
                elif section == "bridge":
                    if key == "publish_hz":
                        settings["publish_hz"] = float(value)
                    elif key == "debug_interval_sec":
                        settings["debug_interval_sec"] = float(value)
                    elif key == "finger_stale_timeout_sec":
                        settings["finger_stale_timeout_sec"] = float(value)
                    elif key == "zero_burst_count":
                        settings["zero_burst_count"] = int(float(value))
    except Exception as exc:
        print(f"[HapticBridge] WARN: failed to read bridge config {path}: {exc}")
    return settings


from sharpa_sdk_bootstrap import default_sharpa_python_paths, setup_sharpa_sdk


def _try_import_sharpa():
    """Import sharpa module; returns (module_or_None, ok, error_str)."""
    setup_sharpa_sdk()
    for path in default_sharpa_python_paths():
        if path and os.path.isdir(path):
            break
    else:
        return None, False, "no SharpaWave SDK python path found"
    try:
        import sharpa  # type: ignore

        return sharpa, True, ""
    except ImportError as exc:
        return None, False, str(exc)


def _wait_for_hand_devices(manager, wait_sec: float = 15.0):
    """
    SDK 5.x returns list[DeviceInfo] from get_all_devices(), not dict.
    Wait for HAND devices to appear (discovery is async after manager init).
    """
    DeviceType = None
    try:
        from sharpa import DeviceType as _DeviceType  # type: ignore
        DeviceType = _DeviceType
    except Exception:
        pass

    deadline = time.time() + max(wait_sec, 1.0)
    time.sleep(1.0)
    while time.time() < deadline:
        device_infos = manager.get_all_devices()
        if DeviceType is not None:
            hand_infos = [
                info for info in device_infos
                if getattr(info, "device_type", None) == DeviceType.HAND
            ]
        else:
            hand_infos = list(device_infos)
        if hand_infos:
            return hand_infos
        time.sleep(1.0)
    return []


def _is_udp_port_in_use(host: str, port: int) -> bool:
    """Return True if something is already bound on host:port (UDP)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        bind_host = host or "0.0.0.0"
        sock.bind((bind_host, int(port)))
        return False
    except OSError:
        return True
    finally:
        sock.close()


def _detect_tactile_port_conflict(
    tactile_ip: str = "0.0.0.0",
    tactile_port: int = 50001,
) -> Tuple[bool, str, int]:
    """
    Check whether the WAVE tactile UDP port is already taken locally.
    Returns (in_use, bind_host, port).
    """
    candidates = []
    if tactile_ip:
        candidates.append(tactile_ip)
    candidates.extend(["0.0.0.0", "127.0.0.1"])
    seen = set()
    for host in candidates:
        if host in seen:
            continue
        seen.add(host)
        if _is_udp_port_in_use(host, tactile_port):
            return True, host, tactile_port
    return False, tactile_ip or "0.0.0.0", tactile_port


def f6_to_force_magnitude(f6) -> float:
    """Resultant force from F6 [Fx,Fy,Fz,Tx,Ty,Tz] — use linear force part only."""
    arr = np.asarray(f6, dtype=np.float64).flatten()
    if arr.size < 3:
        return 0.0
    return float(np.linalg.norm(arr[:3]))


def _make_haptic_msg(left: List[float], right: List[float], frame_id: int = 0):
    """Build proto.HapticFeedback with per-finger force magnitudes (Newtons)."""
    t = time.time()
    msg = sharpa_hand_pb2.HapticFeedback()
    msg.header.frame_id = str(frame_id)
    sec = int(t)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = int((t - sec) * 1e9)
    msg.left_force_mag.extend(left)
    msg.right_force_mag.extend(right)
    return msg


def _start_wave_tactile(wave, sn: str, allow_alternate_port: bool = False) -> bool:
    """
    Start tactile UDP receiver on the device-configured target port (usually 50001).

    When allow_alternate_port is False (default), do not bind a spare local port:
    WAVE still sends tactile to the configured port (e.g. 50001). If that port is
    already used by host software, an alternate local bind cannot receive packets.
    """
    tactile_ready = False
    device_port = 50001
    device_ip = "0.0.0.0"
    try:
        info = wave.get_device_info()
        cfg = getattr(info, "tactile_data_config", None)
        if cfg is not None:
            device_port = int(getattr(cfg, "target_port", device_port) or device_port)
            device_ip = str(getattr(cfg, "target_ip", device_ip) or device_ip)
    except Exception:
        pass

    if hasattr(wave, "start"):
        if wave.start():
            tactile_ready = True
            print(f"[HapticBridge] wave.start() OK sn={sn} (tactile {device_ip}:{device_port})")
        else:
            print(
                f"[HapticBridge] WARN: wave.start() failed sn={sn} "
                f"(tactile port {device_ip}:{device_port} may be occupied by host software)"
            )

    if (
        not tactile_ready
        and allow_alternate_port
        and hasattr(wave, "retry_tactile_alternate_port")
    ):
        if wave.retry_tactile_alternate_port():
            tactile_ready = True
            try:
                info = wave.get_device_info()
                local_port = getattr(getattr(info, "tactile_data_config", None), "target_port", "?")
                print(
                    f"[HapticBridge] Tactile bound on alternate local port sn={sn} "
                    f"local_port={local_port} (device still sends to {device_ip}:{device_port})"
                )
            except Exception:
                print(f"[HapticBridge] Tactile started on alternate local port sn={sn}")

    if (
        not tactile_ready
        and allow_alternate_port
        and hasattr(wave, "get_next_available_tactile_port")
        and hasattr(wave, "bind_tactile_port")
    ):
        try:
            port = wave.get_next_available_tactile_port([device_port])
            if port > 0 and wave.bind_tactile_port(port):
                print(f"[HapticBridge] bind_tactile_port({port}) sn={sn}")
                if hasattr(wave, "start") and wave.start():
                    tactile_ready = True
        except Exception as exc:
            print(f"[HapticBridge] WARN: bind_tactile_port failed sn={sn}: {exc}")

    if not tactile_ready:
        print(f"[HapticBridge] ERROR: tactile not started sn={sn}")
        if hasattr(wave, "tactile_summary"):
            try:
                print(f"[HapticBridge] {wave.tactile_summary()}")
            except Exception:
                pass
        return False

    if hasattr(wave, "is_tactile_ready"):
        try:
            ready = wave.is_tactile_ready()
            print(f"[HapticBridge] is_tactile_ready={ready} sn={sn}")
            return bool(ready)
        except Exception:
            pass
    return True


def _publish_zero_forces(pub, count: int = 15, hz: float = 30.0):
    """
    Send zero-force frames so Manus Client clears PWM state and stops motors.
    Without this, the client keeps vibrating from the last received magnitudes.
    """
    dt = 1.0 / max(hz, 1.0)
    zeros = [0.0] * 5
    for i in range(count):
        try:
            pub.send(_make_haptic_msg(zeros, zeros, frame_id=-(i + 1)).SerializeToString())
        except Exception:
            break
        time.sleep(dt)


class TactileHapticBridge:
    """
    Collects WAVE tactile F6 (SDK or upstream ZMQ), filters, publishes HapticFeedback.

    tactile_source:
      - "auto": use SDK when tactile UDP port is free; otherwise upstream if configured
      - "sdk": always try SharpaWave SDK tactile (fails if host software holds 50001)
      - "upstream": only subscribe proto.HapticFeedback from host software (no SDK tactile)
    """

    def __init__(
        self,
        pub_address: str = "tcp://*:2045",
        publish_hz: float = 30.0,
        debug: bool = False,
        debug_interval_sec: float = 1.0,
        tactile_source: str = "auto",
        upstream_sub_address: Optional[str] = None,
        tactile_port: int = 50001,
        tactile_ip: str = "0.0.0.0",
        haptic_config_path: Optional[str] = None,
    ):
        yaml_settings = load_bridge_settings_from_yaml(haptic_config_path)
        self.haptic_config_path = haptic_config_path or default_haptic_config_path()
        self.deadzone_n = float(yaml_settings.get("deadzone_n", 0.8))
        self.publish_hz = float(yaml_settings.get("publish_hz", publish_hz))
        self.debug_interval_sec = float(yaml_settings.get("debug_interval_sec", debug_interval_sec))
        self.finger_stale_timeout_sec = float(yaml_settings.get("finger_stale_timeout_sec", 0.08))
        self.zero_burst_count = int(yaml_settings.get("zero_burst_count", 5))
        self.pub_address = pub_address
        self.debug = debug
        self.tactile_source = (tactile_source or "auto").lower()
        self.upstream_sub_address = upstream_sub_address
        self.tactile_port = int(tactile_port)
        self.tactile_ip = tactile_ip

        self._zmq_ctx: Optional[zmq.Context] = None
        self._pub: Optional[zmq.Socket] = None
        self._upstream_sub: Optional[zmq.Socket] = None
        self._lock = threading.Lock()
        # Latest raw magnitudes from SDK callbacks (may arrive faster than publish_hz)
        self._left_raw = np.zeros(5, dtype=np.float64)
        self._right_raw = np.zeros(5, dtype=np.float64)
        self._left_finger_updated_at = np.zeros(5, dtype=np.float64)
        self._right_finger_updated_at = np.zeros(5, dtype=np.float64)
        self._left_hand_was_active = False
        self._right_hand_was_active = False
        self._frame_id = 0
        self._publish_count = 0
        self._callback_count = 0
        self._upstream_count = 0
        self._running = False
        self._publish_thread: Optional[threading.Thread] = None
        self._upstream_thread: Optional[threading.Thread] = None
        self._waves = []  # keep references to wave device handles
        self._last_debug_time = 0.0
        self._active_source = "none"

    def _resolve_source_mode(self) -> str:
        """Pick sdk vs upstream based on tactile_source and local port availability."""
        if self.tactile_source == "upstream":
            if not self.upstream_sub_address:
                print(
                    "[HapticBridge] ERROR: -haptics_source upstream requires "
                    "-haptic_force_sub (host software ZMQ address)"
                )
                return "none"
            return "upstream"

        port_busy, bind_host, port = _detect_tactile_port_conflict(
            self.tactile_ip, self.tactile_port
        )
        if self.tactile_source == "sdk":
            if port_busy:
                print(
                    f"[HapticBridge] WARN: tactile UDP {bind_host}:{port} is in use "
                    f"(likely host software). SDK cannot share this port with host software."
                )
            return "sdk"

        # auto
        if port_busy:
            if self.upstream_sub_address:
                print(
                    f"[HapticBridge] tactile UDP {bind_host}:{port} in use — "
                    f"using upstream ZMQ {self.upstream_sub_address}"
                )
                return "upstream"
            print(
                f"[HapticBridge] WARN: tactile UDP {bind_host}:{port} is already used "
                f"by host software; skipping SDK tactile (cb will stay 0).\n"
                f"[HapticBridge] Keep host software on {port}. For Manus haptics, either:\n"
                f"  1) Host software publishes proto.HapticFeedback to ZMQ, then run with\n"
                f"     -haptics_source upstream -haptic_force_sub tcp://127.0.0.1:<port>\n"
                f"  2) Or close host software and use -haptics_source sdk (SDK owns {port})"
            )
            return "none"
        return "sdk"

    def _start_upstream_subscriber(self) -> bool:
        if not self.upstream_sub_address:
            return False
        self._upstream_sub = self._zmq_ctx.socket(zmq.SUB)
        self._upstream_sub.setsockopt(zmq.RCVHWM, 5)
        self._upstream_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self._upstream_sub.connect(self.upstream_sub_address)
        self._upstream_thread = threading.Thread(
            target=self._upstream_loop, name="HapticUpstreamSubscriber", daemon=True
        )
        self._upstream_thread.start()
        print(f"[HapticBridge] Upstream SUB connected: {self.upstream_sub_address}")
        return True

    def _start_sdk_tactile(self) -> None:
        sharpa_mod, ok, err = _try_import_sharpa()
        if not ok:
            print(f"[HapticBridge] ERROR: cannot import sharpa: {err}")
            return

        SharpaWaveManager = sharpa_mod.SharpaWaveManager
        manager = SharpaWaveManager.get_instance()
        hand_infos = _wait_for_hand_devices(manager, wait_sec=15.0)
        if not hand_infos:
            print("[HapticBridge] WARN: no HAND devices found after 15s (WAVE powered & on network?)")
            return

        sns = [getattr(info, "sn", "?") for info in hand_infos]
        print(f"[HapticBridge] Found {len(hand_infos)} HAND device(s): {sns}")

        for info in hand_infos:
            sn = getattr(info, "sn", None)
            if not sn:
                continue
            try:
                cfg = getattr(info, "tactile_data_config", None)
                if cfg is not None:
                    self.tactile_port = int(getattr(cfg, "target_port", self.tactile_port) or self.tactile_port)
                    self.tactile_ip = str(getattr(cfg, "target_ip", self.tactile_ip) or self.tactile_ip)

                wave = manager.connect(sn)
                if hasattr(wave, "set_tactile_enable_state"):
                    err_obj = wave.set_tactile_enable_state(True)
                    if getattr(err_obj, "code", 0) != 0:
                        print(
                            f"[HapticBridge] WARN: set_tactile_enable_state sn={sn}: "
                            f"{getattr(err_obj, 'message', err_obj)}"
                        )
                wave.set_tactile_callback(
                    lambda data, serial=sn: self._on_tactile(data, serial)
                )
                tactile_ok = _start_wave_tactile(wave, sn, allow_alternate_port=False)
                self._waves.append(wave)
                hand_side = getattr(info, "hand_side", "?")
                status = "ready" if tactile_ok else f"NOT receiving ({self.tactile_ip}:{self.tactile_port})"
                print(f"[HapticBridge] Tactile callback registered: sn={sn} hand={hand_side} ({status})")
            except Exception as exc:
                print(f"[HapticBridge] WARN: failed tactile setup sn={sn}: {exc}")

    def start(self) -> bool:
        """Bind ZMQ PUB; ingest tactile from SDK or upstream host software."""
        self._zmq_ctx = zmq.Context.instance()
        self._pub = self._zmq_ctx.socket(zmq.PUB)
        self._pub.setsockopt(zmq.SNDHWM, 5)
        self._pub.bind(self.pub_address)
        print(f"[HapticBridge] ZMQ PUB bound: {self.pub_address} @ {self.publish_hz:.1f} Hz")
        print(
            f"[HapticBridge] deadzone={self.deadzone_n:.2f}N "
            f"stale_timeout={self.finger_stale_timeout_sec:.3f}s "
            f"zero_burst={self.zero_burst_count}"
        )

        mode = self._resolve_source_mode()
        self._active_source = mode
        if mode == "sdk":
            self._start_sdk_tactile()
        elif mode == "upstream":
            if not self._start_upstream_subscriber():
                return False
        elif mode == "none":
            print("[HapticBridge] No tactile input source — publishing zeros until stopped")

        self._running = True
        self._publish_thread = threading.Thread(
            target=self._publish_loop, name="HapticFeedbackPublisher", daemon=True
        )
        self._publish_thread.start()
        if mode == "upstream":
            print("[HapticBridge] Started (host HapticFeedback ZMQ -> Manus ZMQ)")
        elif mode == "sdk":
            print("[HapticBridge] Started (WAVE F6 -> HapticFeedback ZMQ)")
        else:
            print("[HapticBridge] Started (no tactile input)")
        return True

    def stop(self):
        """Stop publisher thread and release ZMQ."""
        self._running = False
        if self._publish_thread and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=2.0)
        self._publish_thread = None
        if self._upstream_thread and self._upstream_thread.is_alive():
            self._upstream_thread.join(timeout=2.0)
        self._upstream_thread = None
        if self._upstream_sub is not None:
            try:
                self._upstream_sub.close()
            except Exception:
                pass
            self._upstream_sub = None
        for wave in self._waves:
            try:
                if hasattr(wave, "stop"):
                    wave.stop()
            except Exception:
                pass
        self._waves.clear()
        if self._pub is not None:
            try:
                _publish_zero_forces(self._pub, count=15, hz=self.publish_hz)
            except Exception:
                pass
            try:
                self._pub.close()
            except Exception:
                pass
            self._pub = None
        print(
            f"[HapticBridge] Stopped. published={self._publish_count} "
            f"tactile_callbacks={self._callback_count} upstream={self._upstream_count}"
        )

    def _upstream_loop(self):
        """Receive HapticFeedback from host software and copy into raw force buffers."""
        while self._running and self._upstream_sub is not None:
            try:
                data = self._upstream_sub.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.005)
                continue
            except Exception as exc:
                if self.debug:
                    print(f"[HapticBridge] upstream recv error: {exc}")
                time.sleep(0.05)
                continue

            try:
                msg = sharpa_hand_pb2.HapticFeedback()
                msg.ParseFromString(data)
                left = list(msg.left_force_mag[:5])
                right = list(msg.right_force_mag[:5])
                while len(left) < 5:
                    left.append(0.0)
                while len(right) < 5:
                    right.append(0.0)
                now = time.time()
                with self._lock:
                    self._upstream_count += 1
                    for i in range(5):
                        self._left_raw[i] = left[i]
                        self._right_raw[i] = right[i]
                        if left[i] > 0.0:
                            self._left_finger_updated_at[i] = now
                        if right[i] > 0.0:
                            self._right_finger_updated_at[i] = now
                    snap_left, snap_right = self._prepare_snapshot_locked(now)
                self._publish_snapshot(snap_left, snap_right, now)
            except Exception as exc:
                if self.debug:
                    print(f"[HapticBridge] upstream parse error: {exc}")

    def _clamp_deadzone(self, force_n: float) -> float:
        """Publish 0 when below deadzone (matches Manus Off, avoids noise vibration)."""
        return 0.0 if force_n < self.deadzone_n else float(force_n)

    def _hand_is_active(self, forces: np.ndarray) -> bool:
        return bool(np.any(forces > 0.0))

    def _expire_stale_fingers_locked(self, now: float) -> None:
        """Zero fingers with no fresh callback (prevents sticky force from other channels)."""
        timeout = max(self.finger_stale_timeout_sec, 0.01)
        for i in range(5):
            if self._left_raw[i] > 0.0 and (now - self._left_finger_updated_at[i]) > timeout:
                self._left_raw[i] = 0.0
            if self._right_raw[i] > 0.0 and (now - self._right_finger_updated_at[i]) > timeout:
                self._right_raw[i] = 0.0

    def _prepare_snapshot_locked(self, now: float) -> Tuple[np.ndarray, np.ndarray]:
        self._expire_stale_fingers_locked(now)
        left = np.array([self._clamp_deadzone(v) for v in self._left_raw], dtype=np.float64)
        right = np.array([self._clamp_deadzone(v) for v in self._right_raw], dtype=np.float64)
        return left, right

    def _publish_snapshot(self, left: np.ndarray, right: np.ndarray, t0: Optional[float] = None) -> int:
        """Publish once; burst extra zero frames when a hand transitions to all-off."""
        fid = self._send_haptic_msg(left, right, t0)
        burst = max(self.zero_burst_count, 1)

        left_active = self._hand_is_active(left)
        right_active = self._hand_is_active(right)
        if self._left_hand_was_active and not left_active:
            for _ in range(burst - 1):
                self._send_haptic_msg(left, right, t0)
        if self._right_hand_was_active and not right_active:
            for _ in range(burst - 1):
                self._send_haptic_msg(left, right, t0)
        self._left_hand_was_active = left_active
        self._right_hand_was_active = right_active
        return fid

    def _send_haptic_msg(
        self,
        left: np.ndarray,
        right: np.ndarray,
        t0: Optional[float] = None,
    ) -> int:
        """Publish raw per-finger force magnitudes (Newtons), no filtering."""
        if self._pub is None:
            return 0
        if t0 is None:
            t0 = time.time()
        with self._lock:
            self._frame_id += 1
            fid = self._frame_id
        msg = sharpa_hand_pb2.HapticFeedback()
        msg.header.frame_id = str(fid)
        sec = int(t0)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((t0 - sec) * 1e9)
        msg.left_force_mag.extend(left.tolist())
        msg.right_force_mag.extend(right.tolist())
        try:
            self._pub.send(msg.SerializeToString(), flags=zmq.NOBLOCK)
            self._publish_count += 1
        except zmq.Again:
            pass
        return fid

    def _on_tactile(self, frame, sn: str):
        """SharpaWaveSDK tactile callback — store per-finger |F| from F6."""
        try:
            content = frame.get("content") if isinstance(frame, dict) else None
            if content is None:
                return
            f6 = content.get("F6")
            if f6 is None:
                return
            ch = int(frame["channel"])
            if ch not in CHANNEL_TO_FINGER_IDX:
                return
            f_mag = f6_to_force_magnitude(f6)
            finger_idx = CHANNEL_TO_FINGER_IDX[ch]
            hand = CHANNEL_TO_HAND.get(ch, "right")
            now = time.time()
            with self._lock:
                self._callback_count += 1
                if hand == "left":
                    self._left_raw[finger_idx] = f_mag
                    self._left_finger_updated_at[finger_idx] = now
                else:
                    self._right_raw[finger_idx] = f_mag
                    self._right_finger_updated_at[finger_idx] = now
                snap_left, snap_right = self._prepare_snapshot_locked(now)
            self._publish_snapshot(snap_left, snap_right, now)
        except Exception as exc:
            if self.debug:
                print(f"[HapticBridge] tactile callback error sn={sn}: {exc}")

    def _publish_loop(self):
        """Debug print timer; SDK/upstream paths publish immediately on each tactile frame."""
        dt = 1.0 / max(self.publish_hz, 1.0)
        while self._running:
            t0 = time.time()
            with self._lock:
                left, right = self._prepare_snapshot_locked(t0)
                fid = self._frame_id

            # Periodic publish: expire stale fingers + keep Manus synced when forces decay.
            if self._active_source in ("sdk", "upstream", "none"):
                self._publish_snapshot(left, right, t0)

            if self.debug and (t0 - self._last_debug_time) >= self.debug_interval_sec:
                self._last_debug_time = t0
                self._print_debug(left, right, fid)

            elapsed = time.time() - t0
            time.sleep(max(0.0, dt - elapsed))

    def _print_debug(self, left: np.ndarray, right: np.ndarray, fid: int):
        """Periodic debug line for tuning force thresholds on Manus side."""
        def fmt(arr):
            return " ".join(f"{FINGER_NAMES[i]}={arr[i]:.2f}N" for i in range(5))

        src = self._active_source
        extra = f" upstream={self._upstream_count}" if src == "upstream" else ""
        print(
            f"[HapticBridge] pub#{self._publish_count} cb={self._callback_count}{extra} "
            f"frame={fid} src={src} | L[{fmt(left)}] R[{fmt(right)}]"
        )


def run_standalone_test(pub_address: str = "tcp://*:2045", duration_sec: float = 10.0):
    """
    Standalone test: publish synthetic ramp forces without WAVE hardware.
    Manus Client can verify ZMQ SUB + tier PWM with: -haptic_sub tcp://127.0.0.1:2045
    """
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(pub_address)
    print(f"[HapticBridge TEST] Publishing synthetic HapticFeedback to {pub_address}")
    time.sleep(0.5)
    t_start = time.time()
    frame = 0
    try:
        while time.time() - t_start < duration_sec:
            t = time.time() - t_start
            # Ramp 0..5 N on index finger for left/right
            ramp = 2.5 * (1.0 + np.sin(2 * np.pi * 0.5 * t))
            left = [0.0, float(ramp), 0.0, 0.0, 0.0]
            right = [0.0, float(ramp) * 0.5, 0.0, 0.0, 0.0]
            pub.send(_make_haptic_msg(left, right, frame).SerializeToString())
            if frame % 30 == 0:
                print(f"[HapticBridge TEST] frame={frame} L_index={ramp:.2f}N")
            frame += 1
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        print("\n[HapticBridge TEST] Interrupted — sending zero forces to stop motors...")
    finally:
        print("[HapticBridge TEST] Sending zero forces...")
        _publish_zero_forces(pub, count=15, hz=30.0)
        pub.close()
        print("[HapticBridge TEST] Done")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="WAVE tactile -> Manus haptic ZMQ bridge")
    parser.add_argument("--pub", default="tcp://*:2045", help="ZMQ PUB bind address")
    parser.add_argument("--hz", type=float, default=30.0, help="Publish rate Hz")
    parser.add_argument("--debug", action="store_true", help="Print force magnitudes periodically")
    parser.add_argument(
        "--test",
        action="store_true",
        help="Synthetic publisher (no WAVE SDK) for Manus Client debugging",
    )
    parser.add_argument("--test-duration", type=float, default=15.0)
    parser.add_argument(
        "--source",
        choices=["auto", "sdk", "upstream"],
        default="auto",
        help="Tactile input: auto (detect port conflict), sdk, or upstream ZMQ",
    )
    parser.add_argument(
        "--force-sub",
        default=None,
        help="ZMQ SUB for proto.HapticFeedback from host software (upstream mode)",
    )
    args = parser.parse_args()

    if args.test:
        run_standalone_test(args.pub, args.test_duration)
    else:
        bridge = TactileHapticBridge(
            pub_address=args.pub,
            publish_hz=args.hz,
            debug=args.debug,
            tactile_source=args.source,
            upstream_sub_address=args.force_sub,
        )
        if not bridge.start():
            sys.exit(1)
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            bridge.stop()

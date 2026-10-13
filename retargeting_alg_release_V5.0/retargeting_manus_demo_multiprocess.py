"""
retarget manus meta/meta pro to Sharpa Wave dexterous hand (URDF) using multiprocess optimization
use zmq for mocap and optional joint publisher; use -wave (UDP glove protocol) or -sdk (SharpaWave SDK) for hardware
"""

import zmq
import numpy as np
import time
import signal
import sys
import os
import multiprocessing as mp
from rich.live import Live
from rich.table import Table
from rich.console import Console
import threading
import queue
import traceback
import logging

# Add include directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/include")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/include/proto_hand")
from hand_retargeting_optimizer import  MultiprocessOptimizationManager, init_hand_model
from proto_hand.receiver_demo import MocapKeypointsReceiver
from proto_hand.sender_demo import HandActionSender
import sharpa_hand_pb2

# Configure logging system
def setup_logging():
    """Configure logging system, support multiprocess, output to file and console"""
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(os.path.join(log_dir, 'retargeting.log'), encoding='utf-8'),
            logging.StreamHandler()  # Also output to console
        ]
    )

setup_logging()
logger = logging.getLogger(__name__)


def install_casadi_inf_warning_filter():
    """
    Mute benign CasADi/IPOPT Inf/NaN trial-point warnings on stderr.

    WAVE cost landscape can evaluate to Inf at intermediate IPOPT steps even when
    the solve succeeds. Install a FD-level filter before spawn so worker processes
    inherit the same sink.
    """
    if getattr(install_casadi_inf_warning_filter, '_installed', False):
        return

    real_fd = os.dup(2)
    real_err = os.fdopen(real_fd, 'w', buffering=1, closefd=True)
    read_fd, write_fd = os.pipe()
    os.dup2(write_fd, 2)
    os.close(write_fd)
    sys.stderr = os.fdopen(2, 'w', buffering=1, closefd=False)

    skip_substrings = (
        'nlp_f failed: Inf detected',
        'nlp_grad failed: NaN detected',
        'nlp_grad failed: Inf detected',
        'Failed to calculate multipliers',
    )

    def _pump():
        with os.fdopen(read_fd, 'r', buffering=1, closefd=True) as src:
            for line in src:
                if any(s in line for s in skip_substrings):
                    continue
                real_err.write(line)
                real_err.flush()

    threading.Thread(target=_pump, name='CasadiStderrFilter', daemon=True).start()
    install_casadi_inf_warning_filter._installed = True


# Add visualization related imports
from visualizer import DualHandVisualizer
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Add WaveController related imports (UDP / "glove" protocol)
try:
    from heartbeat_wave import send_heartbeat
    from pub_glove_wave import MockGlove, HandType, DEFAULT_WAVE_PORT_RIGHT, DEFAULT_WAVE_PORT_LEFT
    WAVE_MODE_AVAILABLE = True
except ImportError as e:
    print(f"Unable to import Wave mode required modules, but this demo can be used without wave mode: {e}")
    WAVE_MODE_AVAILABLE = False

# SharpaWave native SDK (bundled include/SharpaWaveSDK_5.0.4, fallback /opt/sharpa-wave-sdk)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "include"))
from sharpa_sdk_bootstrap import setup_sharpa_sdk

setup_sharpa_sdk()

try:
    from sharpa import (
        SharpaWaveManager,
        ControlMode,
        ControlSource,
        HandSide,
        DeviceType,
    )
    SDK_MODE_AVAILABLE = True
except ImportError as e:
    SDK_MODE_AVAILABLE = False
    SharpaWaveManager = None  # type: ignore
    print(f"Unable to import SharpaWave SDK Python module (optional): {e}")

# WAVE tactile F6 -> ZMQ HapticFeedback for Manus Client (optional, -haptics)
try:
    from tactile_haptic_bridge import TactileHapticBridge
    HAPTICS_BRIDGE_AVAILABLE = True
except ImportError as e:
    print(f"Haptics bridge not available (use -haptics only if sharpa SDK installed): {e}")
    HAPTICS_BRIDGE_AVAILABLE = False


def zmq_mocap_subscriber_process(mocap_queue, address="tcp://localhost:6667"):
    """ZMQ mocap subscriber process running at 200Hz with minimal control flow."""
    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.connect(address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print(f"[ZMQ Mocap Subscriber Process] Connected to {address}")

    target_interval = 1.0 / 1000.0  # 200Hz

    while True:
        try:
            payload = socket.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            payload = None

        if payload:
            t_zmq_recv = time.time()
            msg = sharpa_hand_pb2.MocapKeypoints()
            msg.ParseFromString(payload)

            message_added = False
            attempts = 0
            max_attempts = 10 

            while not message_added and attempts < max_attempts:
                try:
                    # (protobuf, host receive time) for end-to-end latency
                    mocap_queue.put_nowait((msg, t_zmq_recv))
                    message_added = True
                except queue.Full:
                    attempts += 1
                    try:
                        mocap_queue.get_nowait()  # Remove oldest message
                    except queue.Empty:
                        time.sleep(0.001)
                        continue
        
        time.sleep(target_interval)

# ==============================================================================
# Joint Smoother Class (Integrated)
# ==============================================================================
class JointSmoother:
    def __init__(self, send_callback, hz=120.0, w=25.0, z=0.8):
        """
        Generic Joint Smoother (2nd Order Low-Pass Filter + High Freq Sender Thread)
        
        Args:
            send_callback: Function to execute actual hardware sending.
                           Signature: func(angles: list) -> None
            hz (float): Sending frequency (e.g., 120Hz)
            w (float): Natural frequency (Response speed)
            z (float): Damping ratio (Smoothness)
        """
        self.send_callback = send_callback
        self.hz = hz
        
        # 2nd Order Dynamics Parameters
        self.k_p = w * w
        self.k_d = 2 * z * w
        
        # State variables
        self.current_angles = np.zeros(22, dtype=np.float32)
        self.current_velocity = np.zeros(22, dtype=np.float32)
        self.target_angles = None
        self.smooth_target = None   # Input side smoothing variable

        # Thread control
        self.lock = threading.Lock()
        self.data_event = threading.Event()
        self.is_running = False
        self.worker_thread = None
        
        # Timeout control
        self.last_update_time = 0
        self.data_timeout = 0.5

        # Latency: recv_ts stamped on update(); measured on first UDP send after that target
        self._pending_recv_ts = None
        self._on_send_latency = None

    def set_latency_callback(self, cb):
        """cb(latency_s) called once per new target, on first hardware send."""
        self._on_send_latency = cb

    def start(self):
        """Start background smoothing thread"""
        if self.is_running:
            return
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.worker_thread.start()
        logger.info(f"JointSmoother Started (Freq={self.hz}Hz)")

    def stop(self):
        """Stop thread"""
        self.is_running = False
        self.data_event.set() # Wake up thread to exit
        if self.worker_thread:
            self.worker_thread.join(timeout=0.2)

    def update(self, new_target, recv_ts=None):
        """[Called by Main Thread] Update target angles"""
        if not self.is_running:
            return False
            
        with self.lock:
            # Update target
            self.target_angles = np.array(new_target, dtype=np.float32)
            self.last_update_time = time.time()
            if recv_ts is not None:
                self._pending_recv_ts = recv_ts
            
            # Initialization alignment: if first time receiving, snap to target
            if np.all(self.current_angles == 0):
                self.current_angles = self.target_angles.copy()
                self.current_velocity[:] = 0
        
            # If just started or recovered from timeout, reset smooth target
            if self.smooth_target is None:
                self.smooth_target = self.target_angles.copy()

        # Wake up worker thread if waiting
        if not self.data_event.is_set():
            self.data_event.set()
        return True

    def _control_loop(self):
        """[Internal Thread] Physics simulation and callback execution"""
        dt = 1.0 / self.hz
        
        while self.is_running:
            # Wait for data
            if not self.data_event.wait(timeout=0.1):
                continue
            
            # Timeout logic: stop sending and zero velocity if no new data
            if time.time() - self.last_update_time > self.data_timeout:
                self.data_event.clear()
                self.current_velocity[:] = 0 
                with self.lock:
                    self.smooth_target = None
                    self._pending_recv_ts = None
                continue

            loop_start = time.time()
            
            # 2nd Order Low Pass Filter
            if self.target_angles is not None:
                with self.lock:
                    if self.smooth_target is None:
                        self.smooth_target = self.target_angles.copy()
                    
                    # Input smoothing (smooth the error/step)
                    alpha = 0.6
                    self.smooth_target = (1 - alpha) * self.smooth_target + alpha * self.target_angles

                    # Calculate deviation
                    error = self.smooth_target - self.current_angles

                    # Calculate acceleration (a = kp * error - kd * velocity)
                    accel = self.k_p * error - self.k_d * self.current_velocity

                    # Update velocity (v = v + a * dt)
                    self.current_velocity += accel * dt

                    # Update position (x = x + v * dt)
                    self.current_angles += self.current_velocity * dt
                    
                    # Prepare data to send
                    angles_to_send = self.current_angles.tolist()
                    pending_recv_ts = self._pending_recv_ts
                    self._pending_recv_ts = None
                
                # Execute Callback: Send to hardware
                try:
                    self.send_callback(angles_to_send)
                    if pending_recv_ts is not None and self._on_send_latency is not None:
                        self._on_send_latency(time.time() - pending_recv_ts)
                except Exception as e:
                    logger.error(f"Smoother callback error: {e}")

            # Strict frequency control
            elapsed = time.time() - loop_start
            time.sleep(max(0, dt - elapsed))

# ==============================================================================

class WaveController:
    """Wave mode controller, for sending optimized joint angles"""
    
    def __init__(self, target_ip='192.168.1.100', hand_type=None, wave_port=None):
        self.target_ip = target_ip
        self.hand_type = hand_type
        self.mock_glove = None
        self.heartbeat_thread = None
        self.is_running = False
        
        if self.hand_type is None:
            raise ValueError("hand_type parameter is required")
        if self.hand_type not in [HandType.LEFT, HandType.RIGHT]:
            raise ValueError(f"Invalid hand_type: {self.hand_type}, must be HandType.LEFT or HandType.RIGHT")
        
        if wave_port is not None:
            self.wave_port = wave_port
        else:
            self.wave_port = DEFAULT_WAVE_PORT_LEFT if self.hand_type == HandType.LEFT else DEFAULT_WAVE_PORT_RIGHT

    def start(self):
        if not WAVE_MODE_AVAILABLE:
            raise ImportError("Wave mode required modules not installed")
            
        heartbeat_name = f'Glove-{"L" if self.hand_type == HandType.LEFT else "R"}-retargeting'
        self.heartbeat_thread = threading.Thread(
            target=send_heartbeat, 
            args=(heartbeat_name, self.target_ip)
        )
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        
        self.mock_glove = MockGlove()
        if not self.mock_glove.create_glove(self.target_ip, self.wave_port, self.hand_type):
            raise RuntimeError("Failed to create UDP connection")
        
        self.is_running = True
        hand_type_name = "LEFT" if self.hand_type == HandType.LEFT else "RIGHT"
        print(f"Wave controller started, hand_type: {hand_type_name}, target IP: {self.target_ip}, port: {self.wave_port}")
        
    def send_joint_angles(self, joint_angles):
        if not self.is_running or self.mock_glove is None:
            return False
        
        try:
            if len(joint_angles) != 22:
                # logger.warning(f"Incorrect number of input angles: {len(joint_angles)}, expected 22")
                # raise ValueError("Incorrect number of input angles, cannot send")
                return False
            
            wave_angles = joint_angles  # Input is already list or numpy array
            self.mock_glove.send_WAVE_packet(wave_angles, self.hand_type)

            return True
        except Exception as e:
            print(f"Failed to send joint angles: {e}")
            return False
    
    def stop(self):
        self.is_running = False
        if self.mock_glove is not None:
            self.mock_glove = None
        print("Wave controller stopped")

class SharpaSdkDualBackend:
    """Control dexterous hands via SharpaWave native SDK (see sharpa_wave_example.py)."""

    def __init__(self, speed_coeff=0.3, current_coeff=0.6):
        self.speed_coeff = speed_coeff
        self.current_coeff = current_coeff
        self.wave_by_hand = {}
        self.manager = None
        self._started = False

    @staticmethod
    def _init_wave_hand(wave, speed_coeff, current_coeff):
        err = wave.set_control_mode(ControlMode.POSITION)
        if err.code != 0:
            logger.error(f"SDK set_control_mode failed: {err.message}")
            return False
        err = wave.set_speed_coeff(speed_coeff)
        if err.code != 0:
            logger.error(f"SDK set_speed_coeff failed: {err.message}")
            return False
        err = wave.set_current_coeff(current_coeff)
        if err.code != 0:
            logger.error(f"SDK set_current_coeff failed: {err.message}")
            return False
        err = wave.set_control_source(ControlSource.SDK)
        if err.code != 0:
            logger.error(f"SDK set_control_source failed: {err.message}")
            return False
        return True

    def connect_and_start(self):
        if not SDK_MODE_AVAILABLE:
            raise ImportError("SharpaWave SDK Python module not available")
        self.manager = SharpaWaveManager.get_instance()
        time.sleep(1.0)

        while True:
            device_infos = self.manager.get_all_devices()
            if device_infos and any(
                info.device_type == DeviceType.HAND for info in device_infos
            ):
                break
            logger.info("Waiting for SharpaWave HAND devices...")
            time.sleep(1.0)

        device_infos = [info for info in device_infos if info.device_type == DeviceType.HAND]
        logger.info(f"Sharpa SDK: found HAND device(s): {[info.sn for info in device_infos]}")

        self.wave_by_hand = {}
        for info in device_infos:
            wave = self.manager.connect(info.sn)
            hand = "right"
            if hasattr(info, "hand_side"):
                hand = "left" if info.hand_side == HandSide.LEFT else "right"
            elif hasattr(wave, "get_hand_side"):
                hs = wave.get_hand_side()
                hand = "left" if hs == HandSide.LEFT else "right"

            if not self._init_wave_hand(wave, self.speed_coeff, self.current_coeff):
                continue

            err = wave.set_enable_state(True)
            if err.code != 0:
                logger.warning(f"SDK set_enable_state for {info.sn}: {err.message}")

            if not wave.start():
                logger.error(f"SDK start() failed for {info.sn}")
                continue

            self.wave_by_hand[hand] = wave
            logger.info(f"Sharpa SDK: {info.sn} ready as {hand} hand")

        if not self.wave_by_hand:
            raise RuntimeError("Sharpa SDK: no hand started successfully")

        # Zero pose with interpolation (same idea as sharpa_wave_example main)
        enable_interp = True
        for hand, wave in self.wave_by_hand.items():
            wave.set_joint_position([0.0] * 22, enable_interp)

        self._started = True
        print(f"Sharpa SDK backend started, hands: {list(self.wave_by_hand.keys())}")

    def send_joint_angles_rad(self, hand_key, joint_angles):
        """Send 22 joint angles in radians (matches set_joint_position in SDK examples)."""
        if not self._started:
            return False
        wave = self.wave_by_hand.get(hand_key)
        if wave is None:
            return False
        if len(joint_angles) != 22:
            return False
        try:
            positions = [float(x) for x in joint_angles]
            err = wave.set_joint_position(positions, True)
            return err.code == 0
        except Exception as e:
            logger.error(f"Sharpa SDK set_joint_position ({hand_key}): {e}")
            return False

    def get_joint_angles_degree(self, hand_key):
        """Read current joint feedback angles (degree) for one hand."""
        if not self._started:
            return None
        wave = self.wave_by_hand.get(hand_key)
        if wave is None:
            return None
        try:
            error, angles = wave.get_joint_position_degree()
            if error.code != 0:
                logger.debug(f"Sharpa SDK get_joint_position_degree ({hand_key}) failed: {error.message}")
                return None
            return [float(a) for a in angles]
        except Exception as e:
            logger.debug(f"Sharpa SDK get_joint_position_degree ({hand_key}) exception: {e}")
            return None

    def stop(self):
        for hand_key, wave in list(self.wave_by_hand.items()):
            try:
                wave.set_enable_state(False)
                wave.stop()
                if hasattr(wave, "destroy"):
                    wave.destroy()
            except Exception as e:
                logger.warning(f"Sharpa SDK stop {hand_key}: {e}")
        self.wave_by_hand.clear()
        try:
            if self.manager is not None:
                SharpaWaveManager.get_instance().disconnect_all()
        except Exception as e:
            logger.warning(f"Sharpa SDK disconnect_all: {e}")
        self.manager = None
        self._started = False
        print("Sharpa SDK backend stopped")


class DualHandRetargetingSystem:
    """Dual hand retargeting system, with optional left/right input"""
    
    def __init__(self, hand_serial='WAVE', filter_alpha=0.2, 
                 mocap_address="tcp://localhost:6667", 
                 hand_action_address="tcp://localhost:6668",
                 wave=False, sdk=False,
                 glove_left_ip='192.168.10.110', glove_right_ip='192.168.10.100',
                 plot_enabled=False, plot_orientation=False, debug_print=False,
                 smoother_hz=250.0, smoother_w=25.0, smoother_z=0.8,
                 sdk_speed_coeff=0.3, sdk_current_coeff=0.6,
                 haptics=False, haptic_pub_address="tcp://*:2045",
                 haptic_publish_hz=30.0, haptic_debug=False,
                 haptics_source="auto", haptic_force_sub=None,
                 haptic_config=None, latency=False):
        
        self.hand_serial = hand_serial
        self.filter_alpha = filter_alpha
        self.mocap_address = mocap_address
        self.hand_action_address = hand_action_address
        self.wave = wave
        self.sdk = sdk
        self.glove_left_ip = glove_left_ip
        self.glove_right_ip = glove_right_ip
        self.plot_enabled = plot_enabled
        self.plot_orientation = plot_orientation
        self.debug_print = debug_print
        self.smoother_hz = smoother_hz
        self.smoother_w = smoother_w
        self.smoother_z = smoother_z
        self.sdk_speed_coeff = sdk_speed_coeff
        self.sdk_current_coeff = sdk_current_coeff
        self.sdk_backend = None
        self.haptics = haptics
        self.haptic_pub_address = haptic_pub_address
        self.haptic_publish_hz = haptic_publish_hz
        self.haptic_debug = haptic_debug
        self.haptics_source = haptics_source
        self.haptic_force_sub = haptic_force_sub
        self.haptic_config = haptic_config
        self.haptic_bridge = None
        self.latency = latency
        self._latency_cmd_ms = []
        self._latency_udp_ms = []
        self._latency_opt_ms = []
        self._latency_manus_age_ms = []
        self._last_latency_print_time = 0.0
        self._latency_lock = threading.Lock()
        self._latest_recv_ts = None

        if self.wave and self.sdk:
            raise ValueError("Use only one of wave (UDP) or sdk (SharpaWave SDK), not both")

        if hand_serial != 'WAVE':
            raise ValueError(f"Unsupported hand model: {hand_serial}, only WAVE (Sharpa Wave URDF) is supported")
        
        self.hand_models = init_hand_model(hand_serial)
        self.hand_action_sender = self._create_zmq_components()
        
        self.mocap_queue = mp.Queue(maxsize=10)
        self.mocap_subscriber_process = None
        
        self.multiprocess_manager = MultiprocessOptimizationManager(
            self.hand_models, 
            filter_alpha=filter_alpha, 
            hand_serial=hand_serial
        )
        
        # Wave controllers and Smoothers
        self.left_wave_controller = None
        self.right_wave_controller = None
        self.left_smoother = None
        self.right_smoother = None

        self._use_hardware_output = False

        if self.sdk and not SDK_MODE_AVAILABLE:
            print("Sharpa SDK requested (-sdk) but Python SDK module failed to load; disabling sdk mode")
            self.sdk = False

        if self.sdk and SDK_MODE_AVAILABLE:
            try:
                self.sdk_backend = SharpaSdkDualBackend(
                    speed_coeff=self.sdk_speed_coeff,
                    current_coeff=self.sdk_current_coeff,
                )
                self.sdk_backend.connect_and_start()

                self.left_smoother = JointSmoother(
                    send_callback=lambda angles: self.sdk_backend.send_joint_angles_rad("left", angles),
                    hz=self.smoother_hz,
                    w=self.smoother_w,
                    z=self.smoother_z,
                )
                self.right_smoother = JointSmoother(
                    send_callback=lambda angles: self.sdk_backend.send_joint_angles_rad("right", angles),
                    hz=self.smoother_hz,
                    w=self.smoother_w,
                    z=self.smoother_z,
                )
                print("Sharpa SDK smoothers initialized (joint commands in radians)")
                self._use_hardware_output = True
            except Exception as e:
                print(f"Failed to start Sharpa SDK backend: {e}")
                self.sdk = False
                self.sdk_backend = None
                self.left_smoother = None
                self.right_smoother = None

        if self.wave and WAVE_MODE_AVAILABLE:
            try:
                # 1. Initialize Low-level Wave Controllers
                self.left_wave_controller = WaveController(
                    target_ip=self.glove_left_ip, 
                    hand_type=HandType.LEFT
                )
                self.right_wave_controller = WaveController(
                    target_ip=self.glove_right_ip, 
                    hand_type=HandType.RIGHT
                )
                self.left_wave_controller.start()
                self.right_wave_controller.start()

                # 2. Initialize Joint Smoothers
                # We connect the smoother's callback directly to the wave controller's send method
                self.left_smoother = JointSmoother(
                    send_callback=self.left_wave_controller.send_joint_angles,
                    hz=self.smoother_hz,  # High frequency sending to hardware
                    w=self.smoother_w,    # Response speed
                    z=self.smoother_z     # Damping
                )
                self.right_smoother = JointSmoother(
                    send_callback=self.right_wave_controller.send_joint_angles,
                    hz=self.smoother_hz,
                    w=self.smoother_w,
                    z=self.smoother_z
                )

                print("Wave controllers and Smoothers initialized")
                self._use_hardware_output = True
            except Exception as e:
                print(f"Failed to start Wave controllers: {e}")
                self.wave = False
                self._use_hardware_output = self.sdk and self.sdk_backend is not None

        self._wire_latency_callbacks()

        if self.haptics:
            if not HAPTICS_BRIDGE_AVAILABLE:
                print("WARN: -haptics requested but tactile_haptic_bridge unavailable")
                self.haptics = False
            else:
                self.haptic_bridge = TactileHapticBridge(
                    pub_address=self.haptic_pub_address,
                    publish_hz=self.haptic_publish_hz,
                    debug=self.haptic_debug,
                    tactile_source=self.haptics_source,
                    upstream_sub_address=self.haptic_force_sub,
                    haptic_config_path=self.haptic_config,
                )

        self.visualizer = None
        if self.plot_enabled and not self._use_hardware_output:
            try:
                self.visualizer = DualHandVisualizer(display_mode='both', plot_orientation=self.plot_orientation)
                print("Visualizer initialized")
            except Exception as e:
                print(f"Failed to initialize visualizer: {e}")
                self.visualizer = None
        
        if self.debug_print:
            self.table, self.console = self._create_table()
        else:
            self.table, self.console = None, None
        
        self.running = False
        self.frame_count = 0
        self._last_sdk_state_print_time = 0.0

    def _wire_latency_callbacks(self):
        if not self.latency:
            return

        def _on_udp(side):
            def _cb(latency_s):
                with self._latency_lock:
                    self._latency_udp_ms.append(latency_s * 1000.0)
            return _cb

        if self.left_smoother:
            self.left_smoother.set_latency_callback(_on_udp('left'))
        if self.right_smoother:
            self.right_smoother.set_latency_callback(_on_udp('right'))
        print("Latency probe enabled: prints avg/p50/p95 every 1s (recv→cmd / recv→UDP)")

    @staticmethod
    def _pct(samples, p):
        if not samples:
            return float('nan')
        arr = np.sort(np.asarray(samples, dtype=np.float64))
        idx = int(np.clip(round((p / 100.0) * (len(arr) - 1)), 0, len(arr) - 1))
        return float(arr[idx])

    def _record_pipeline_latency(self, recv_ts, mocap_msg, left_result, right_result):
        if not self.latency or recv_ts is None:
            return
        now = time.time()
        with self._latency_lock:
            self._latency_cmd_ms.append((now - recv_ts) * 1000.0)
            if left_result is not None and getattr(left_result, 'optimization_time', None) is not None:
                self._latency_opt_ms.append(left_result.optimization_time * 1000.0)
            if right_result is not None and getattr(right_result, 'optimization_time', None) is not None:
                self._latency_opt_ms.append(right_result.optimization_time * 1000.0)
            try:
                stamp = mocap_msg.header.stamp
                manus_t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                # Integrated Core usually uses host-ish epoch ns; ignore nonsense ages.
                age_ms = (recv_ts - manus_t) * 1000.0
                if 0.0 <= age_ms <= 1000.0:
                    self._latency_manus_age_ms.append(age_ms)
            except Exception:
                pass

        if now - self._last_latency_print_time < 1.0:
            return
        self._last_latency_print_time = now
        with self._latency_lock:
            cmd = self._latency_cmd_ms
            udp = self._latency_udp_ms
            opt = self._latency_opt_ms
            age = self._latency_manus_age_ms
            self._latency_cmd_ms = []
            self._latency_udp_ms = []
            self._latency_opt_ms = []
            self._latency_manus_age_ms = []

        def _fmt(name, samples):
            if not samples:
                return f"{name}: n/a"
            return (
                f"{name}: n={len(samples)} avg={np.mean(samples):.1f} "
                f"p50={self._pct(samples, 50):.1f} p95={self._pct(samples, 95):.1f} ms"
            )

        print(
            "[Latency] "
            + " | ".join([
                _fmt("recv→cmd", cmd),
                _fmt("recv→UDP", udp),
                _fmt("opt", opt),
                _fmt("manus→recv", age),
            ])
        )
    
    def _create_zmq_components(self):
        hand_action_sender = HandActionSender(address=self.hand_action_address)
        return hand_action_sender
    
    def _start_mocap_subscriber_process(self):
        if self.mocap_subscriber_process is None or not self.mocap_subscriber_process.is_alive():
            self.mocap_subscriber_process = mp.Process(
                target=zmq_mocap_subscriber_process,
                args=(self.mocap_queue, self.mocap_address)
            )
            self.mocap_subscriber_process.daemon = True
            self.mocap_subscriber_process.start()
            print(f"Mocap data receiving process started, address: {self.mocap_address}")
    
    def _stop_mocap_subscriber_process(self):
        if self.mocap_subscriber_process and self.mocap_subscriber_process.is_alive():
            self.mocap_subscriber_process.terminate()
            self.mocap_subscriber_process.join(timeout=1.0)
            if self.mocap_subscriber_process.is_alive():
                self.mocap_subscriber_process.kill()
            print("Mocap data receiving process stopped")
    
    def _create_table(self):
        """Create display table"""
        table = Table(show_header=False, box=None)
        table.add_column("Left Hand", style="cyan")
        table.add_column("Right Hand", style="magenta")
        table.add_row("Waiting for data...", "Waiting for data...")
        table.add_row("Cost value: ", "Cost value: ")
        table.add_row("Optimization time: ", "Optimization time: ")
        table.add_row("tip_pos_loss: ", "tip_pos_loss: ")
        table.add_row("tip_ori_loss: ", "tip_ori_loss: ")
        table.add_row("finger_ori_loss: ", "finger_ori_loss: ")
        table.add_row("pinch_loss: ", "pinch_loss: ")
        table.add_row("pinch_ori_loss: ", "pinch_ori_loss: ")
        table.add_row("dq_loss: ", "dq_loss: ")
        table.add_row("thumb_ddq_loss: ", "thumb_ddq_loss: ")
        table.add_row("thumb_mcp_loss: ", "thumb_mcp_loss: ")
        table.add_row("dip_ori_loss: ", "dip_ori_loss: ")
        table.add_row("pip_pinch_loss: ", "pip_pinch_loss: ")
        table.add_row("fist_loss: ", "fist_loss: ")
        table.add_row("pip_gap_loss: ", "pip_gap_loss: ")
        
        console = Console()
        return table, console
    
    def _protobuf_to_numpy(self, mocap_msg):
        left_keypoints = np.zeros((25, 7))
        right_keypoints = np.zeros((25, 7))
        
        for i, pose in enumerate(mocap_msg.left_mocap_pose):
            if i < 25: 
                left_keypoints[i, 0] = pose.position.x
                left_keypoints[i, 1] = pose.position.y
                left_keypoints[i, 2] = pose.position.z
                left_keypoints[i, 3] = pose.orientation.w
                left_keypoints[i, 4] = pose.orientation.x
                left_keypoints[i, 5] = pose.orientation.y
                left_keypoints[i, 6] = pose.orientation.z
        
        for i, pose in enumerate(mocap_msg.right_mocap_pose):
            if i < 25:
                right_keypoints[i, 0] = pose.position.x
                right_keypoints[i, 1] = pose.position.y
                right_keypoints[i, 2] = pose.position.z
                right_keypoints[i, 3] = pose.orientation.w
                right_keypoints[i, 4] = pose.orientation.x
                right_keypoints[i, 5] = pose.orientation.y
                right_keypoints[i, 6] = pose.orientation.z
        return left_keypoints, right_keypoints

    @staticmethod
    def _keypoints_usable(keypoints, min_bone_len=1e-4):
        """Reject frames that make CasADi cost Inf/NaN (empty, non-finite, collapsed bones, zero quats)."""
        if keypoints is None or keypoints.shape != (25, 7):
            return False
        if not np.isfinite(keypoints).all():
            return False
        if np.all(keypoints[:, :3] == 0):
            return False
        # WAVE optimizer finger chains: [0..4],[5..9],... — skip near-zero bones
        for finger in range(5):
            base = finger * 5
            for j in range(4):
                a = keypoints[base + j, :3]
                b = keypoints[base + j + 1, :3]
                if np.linalg.norm(b - a) < min_bone_len:
                    return False
        quat_norm = np.linalg.norm(keypoints[:, 3:7], axis=1)
        if np.any(quat_norm < 1e-6):
            return False
        return True
    
    def _update_table(self, frame_idx, left_result, right_result):
        if not self.debug_print or self.table is None:
            return
            
        if left_result:
            self.table.columns[0]._cells[0] = f"Left hand frame {left_result.frame_index + 1} optimization completed"
            self.table.columns[0]._cells[1] = f"Cost value: {left_result.cost_value:.6f}"
            self.table.columns[0]._cells[2] = f"Optimization time: {left_result.optimization_time:.6f}s"
            # ... (truncated for brevity, same as original) ...
            
        if right_result:
            self.table.columns[1]._cells[0] = f"Right hand frame {right_result.frame_index + 1} optimization completed"
            self.table.columns[1]._cells[1] = f"Cost value: {right_result.cost_value:.6f}"
            self.table.columns[1]._cells[2] = f"Optimization time: {right_result.optimization_time:.6f}s"
            # ... (truncated for brevity, same as original) ...

    def _send_hand_action(self, left_result, right_result, recv_ts=None):
        """
        Send joint command message
        
        Args:
            left_result: Left optimization result
            right_result: Right optimization result
            recv_ts: Host time when this mocap frame was received over ZMQ
        """
        try:
            # === Hardware: UDP Wave (degrees) or Sharpa SDK (radians) ===
            if self.wave:
                if left_result and left_result.filtered_angles is not None and self.left_smoother:
                    left_angles_deg = np.rad2deg(left_result.filtered_angles)
                    self.left_smoother.update(left_angles_deg, recv_ts=recv_ts)
                if right_result and right_result.filtered_angles is not None and self.right_smoother:
                    right_angles_deg = np.rad2deg(right_result.filtered_angles)
                    self.right_smoother.update(right_angles_deg, recv_ts=recv_ts)
            elif self.sdk and self.sdk_backend is not None:
                if left_result and left_result.filtered_angles is not None and self.left_smoother:
                    self.left_smoother.update(
                        np.asarray(left_result.filtered_angles, dtype=np.float32),
                        recv_ts=recv_ts,
                    )
                if right_result and right_result.filtered_angles is not None and self.right_smoother:
                    self.right_smoother.update(
                        np.asarray(right_result.filtered_angles, dtype=np.float32),
                        recv_ts=recv_ts,
                    )
                self._print_sdk_state_once_per_second()
            
            # === ZMQ Mode (Visualization / Data Recording) ===
            # This continues to send the "Target" (Optimized) values to ZMQ for visualizer
            
            # Get joint names
            left_joint_names = self.hand_models['left'].joint_names if self.hand_models['left'] else []
            right_joint_names = self.hand_models['right'].joint_names if self.hand_models['right'] else []
            
            # Convert to degrees for ZMQ (Standard is usually degrees for Unity/Unreal)
            left_positions = np.array(left_result.filtered_angles).tolist() if left_result and left_result.filtered_angles is not None else [0.0] * len(left_joint_names)
            right_positions = np.array(right_result.filtered_angles).tolist() if right_result and right_result.filtered_angles is not None else [0.0] * len(right_joint_names)
            
            # Send ZMQ message
            success = self.hand_action_sender.send_hand_action(
                left_joint_names, left_positions,
                right_joint_names, right_positions
            )
            
            if not success:
                print("[HandAction Sender] Failed to send joint commands")
                
        except Exception as e:
            print(f"[HandAction Sender] Error sending joint commands: {e}")

    def _print_sdk_state_once_per_second(self):
        """In -sdk mode, print hardware feedback joint state at 1 Hz."""
        now = time.time()
        if now - self._last_sdk_state_print_time < 1.0:
            return
        self._last_sdk_state_print_time = now

        left_angles = self.sdk_backend.get_joint_angles_degree("left") if self.sdk_backend else None
        right_angles = self.sdk_backend.get_joint_angles_degree("right") if self.sdk_backend else None

        if left_angles is not None:
            print("[SDK State][left][deg] " + ",".join(f"{x:.2f}" for x in left_angles))
        if right_angles is not None:
            print("[SDK State][right][deg] " + ",".join(f"{x:.2f}" for x in right_angles))
    
    def start(self):
        """Start system"""
        try:
            self._start_mocap_subscriber_process()
            self.multiprocess_manager.start()

            if self.haptics and self.haptic_bridge is not None:
                try:
                    if not self.haptic_bridge.start():
                        print("WARN: Haptic bridge failed to start (check SharpaWave SDK / WAVE network)")
                        self.haptics = False
                    else:
                        print(
                            f"Haptics bridge active: PUB {self.haptic_pub_address} @ {self.haptic_publish_hz}Hz "
                            f"(Manus Client SUB tcp://127.0.0.1:2045)"
                        )
                        if not self.wave and not self.sdk:
                            print(
                                "NOTE: -haptics only publishes tactile->Manus; "
                                "add -wave or -sdk to also drive WAVE joint motion."
                            )
                except Exception as exc:
                    print(f"WARN: Haptic bridge error (retargeting continues): {exc}")
                    self.haptics = False

            # Start Smoothers (Threads)
            if self.wave or (self.sdk and self.sdk_backend is not None):
                if self.left_smoother:
                    self.left_smoother.start()
                if self.right_smoother:
                    self.right_smoother.start()
            
            time.sleep(1)
            
            if self.visualizer is not None and self.plot_enabled and not self._use_hardware_output:
                try:
                    self.visualizer.start_animation(interval=100)
                    print("Visualizer started")
                except Exception as e:
                    print(f"Failed to start visualizer: {e}")
            
            self.running = True
            print("Dual hand retargeting system started")
            
        except Exception as e:
            print(f"Failed to start system: {e}")
            raise e
    
    def run(self):
        """Run main loop"""
        if not self.running:
            raise RuntimeError("System not started, please call start() method first")
        
        try:
            left_keypoints= None
            right_keypoints = None
            left_keypoints_last = None
            right_keypoints_last = None
            
            if self.debug_print and self.table is not None:
                live_context = Live(self.table, refresh_per_second=100, console=self.console, screen=False)
            else:
                from contextlib import nullcontext
                live_context = nullcontext()
            
            with live_context:
                while self.running:
                    try:
                        item = self.mocap_queue.get_nowait()
                        if isinstance(item, tuple):
                            mocap_msg, recv_ts = item
                        else:
                            mocap_msg, recv_ts = item, time.time()
                        
                        if mocap_msg is not None:
                            left_keypoints, right_keypoints = self._protobuf_to_numpy(mocap_msg)
                            if left_keypoints is None or np.all(left_keypoints == 0) or not self._keypoints_usable(left_keypoints):
                                left_keypoints = left_keypoints_last
                            if right_keypoints is None or np.all(right_keypoints == 0) or not self._keypoints_usable(right_keypoints):
                                right_keypoints = right_keypoints_last

                            if left_keypoints is not None and self._keypoints_usable(left_keypoints):
                                left_keypoints_last = left_keypoints
                            if right_keypoints is not None and self._keypoints_usable(right_keypoints):
                                right_keypoints_last = right_keypoints
                            
                            left_updated = False
                            right_updated = False
                            
                            if left_keypoints is not None and self._keypoints_usable(left_keypoints):
                                left_updated = self.multiprocess_manager.update_process_keypoints('left', left_keypoints, self.frame_count)
                                while not left_updated and self.running: 
                                    left_updated = self.multiprocess_manager.update_process_keypoints('left', left_keypoints, self.frame_count)
                                    time.sleep(0.001)
                            
                            if right_keypoints is not None and self._keypoints_usable(right_keypoints):
                                right_updated = self.multiprocess_manager.update_process_keypoints('right', right_keypoints, self.frame_count)
                                while not right_updated and self.running:
                                    right_updated = self.multiprocess_manager.update_process_keypoints('right', right_keypoints, self.frame_count)
                                    time.sleep(0.001)
                            
                            left_result = self.multiprocess_manager.get_result('left')  if left_updated else None
                            right_result = self.multiprocess_manager.get_result('right')  if right_updated else None

                            if self.visualizer is not None and self.plot_enabled and not self._use_hardware_output:
                                try:
                                    left_hand_keypoints = left_result.keypoints if left_result else None
                                    right_hand_keypoints = right_result.keypoints if right_result else None
                                    
                                    self.visualizer.update(
                                        left_raw_keypoints=left_keypoints,
                                        left_hand_keypoints=left_hand_keypoints,
                                        right_raw_keypoints=right_keypoints,
                                        right_hand_keypoints=right_hand_keypoints
                                    )
                                    self.visualizer.update_display()
                                except KeyboardInterrupt:
                                    raise 
                                except Exception as e:
                                    print(f"Error updating visualizer: {e}")
                            
                            self._update_table(self.frame_count, left_result, right_result)
                            
                            # Send joint commands (Now utilizes Smoothers inside)
                            self._send_hand_action(left_result, right_result, recv_ts=recv_ts)
                            self._record_pipeline_latency(recv_ts, mocap_msg, left_result, right_result)
                            
                            self.frame_count += 1
                            
                    except queue.Empty:
                        pass
                    except KeyboardInterrupt:
                        raise
                    except Exception as e:
                        print(f"Error processing message: {e}, {traceback.format_exc()}")
                        time.sleep(0.01)
                    
                    time.sleep(0.001)
                        
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Error occurred during program execution: {e}")
            raise e
    
    def stop(self):
        """Stop system"""
        print("Stopping system...")
        
        self.running = False
        
        self._stop_mocap_subscriber_process()
        
        if hasattr(self, 'multiprocess_manager') and self.multiprocess_manager:
            try:
                self.multiprocess_manager.cleanup()
            except Exception as e:
                print(f"Multiprocess cleanup error: {e}")
        
        if hasattr(self, 'hand_action_sender') and self.hand_action_sender:
            try:
                self.hand_action_sender.close()
            except:
                pass
        
        # Stop Smoothers BEFORE stopping WaveControllers
        # This ensures threads don't try to send to closed sockets
        if self.left_smoother:
            self.left_smoother.stop()
        if self.right_smoother:
            self.right_smoother.stop()

        if self.left_wave_controller:
            try:
                self.left_wave_controller.stop()
            except:
                pass
        if self.right_wave_controller:
            try:
                self.right_wave_controller.stop()
            except:
                pass

        if self.sdk_backend is not None:
            try:
                self.sdk_backend.stop()
            except Exception as e:
                logger.warning(f"SDK backend stop: {e}")
            self.sdk_backend = None

        if self.haptic_bridge is not None:
            try:
                self.haptic_bridge.stop()
            except Exception as e:
                print(f"Haptic bridge stop error: {e}")
            self.haptic_bridge = None

        if self.visualizer is not None:
            try:
                if hasattr(self.visualizer, 'left_ani') and self.visualizer.left_ani is not None:
                    self.visualizer.left_ani.event_source.stop()
                if hasattr(self.visualizer, 'right_ani') and self.visualizer.right_ani is not None:
                    self.visualizer.right_ani.event_source.stop()
                plt.close('all')
                plt.ioff()
            except:
                pass

        print("System stopped")

def main():
    try:
        mp.set_start_method('spawn', force=True)
    except RuntimeError:
        pass

    # Before spawning optimizer workers: hide expected Inf trial-point spam.
    install_casadi_inf_warning_filter()
    
    import argparse
    parser = argparse.ArgumentParser(description='Dual hand retargeting system')
    parser.add_argument('-hand_serial', type=str, default='WAVE', choices=['WAVE'], 
                       help='Hand kinematic model: Sharpa Wave URDF (default: WAVE)')
    parser.add_argument('-filter_alpha', type=float, default=1.0, 
                       help='Filter parameter (0-1, default: 0.8)')
    parser.add_argument('-mocap_address', type=str, default="tcp://localhost:2044",help='Mocap data receiving address')
    parser.add_argument('-hand_action_address', type=str, default="tcp://*:6668",help='Joint command sending address')
    hw = parser.add_mutually_exclusive_group()
    hw.add_argument('-wave', action='store_true',
                       help='UDP Wave glove protocol (MockGlove / heartbeat)')
    hw.add_argument('-sdk', action='store_true',
                       help='Use SharpaWave SDK, see include/SharpaWaveSDK_5.0.4/sample/python/sharpa_wave_example.py')
    parser.add_argument('-glove_left_ip', type=str, default='192.168.10.99',
                       help='Left hand Wave controller target IP')
    parser.add_argument('-glove_right_ip', type=str, default='192.168.10.100',
                       help='Right hand Wave controller target IP')
    parser.add_argument('-plot', action='store_true', default=False,
                       help='Enable plot function')
    parser.add_argument('-plot_orientation', action='store_true', default=False,
                       help='Display orientation arrows for key points')
    parser.add_argument('-debug_print', action='store_true', default=False,
                       help='Enable debug print mode')
    parser.add_argument('-smoother_hz', type=float, default=250.0,
                       help='Joint smoother sending frequency in Hz (default: 250.0)')
    parser.add_argument('-smoother_w', type=float, default=100.0,
                       help='Joint smoother response speed (default: 100.0)')
    parser.add_argument('-smoother_z', type=float, default=0.2,
                       help='Joint smoother damping (default: 0.2)')
    parser.add_argument('-sdk_speed_coeff', type=float, default=0.3,
                       help='Sharpa SDK speed coeff (default: 0.3, only with -sdk)')
    parser.add_argument('-sdk_current_coeff', type=float, default=0.6,
                       help='Sharpa SDK current coeff (default: 0.6, only with -sdk)')
    parser.add_argument('-haptics', action='store_true',
                       help='Enable WAVE tactile -> Manus haptic ZMQ (requires SharpaWave SDK)')
    parser.add_argument('-haptic_pub_address', type=str, default='tcp://*:2045',
                       help='ZMQ PUB bind for HapticFeedback (default: tcp://*:2045)')
    parser.add_argument('-haptic_publish_hz', type=float, default=30.0,
                       help='HapticFeedback publish rate in Hz (default: 30)')
    parser.add_argument('-haptic_debug', action='store_true',
                       help='Print per-finger force magnitudes from tactile bridge')
    parser.add_argument('-haptics_source', type=str, default='auto',
                       choices=['auto', 'sdk', 'upstream'],
                       help='Tactile source: auto (detect 50001 conflict), sdk, or upstream ZMQ')
    parser.add_argument('-haptic_force_sub', type=str, default=None,
                       help='ZMQ SUB for proto.HapticFeedback from host software (use with upstream)')
    parser.add_argument('-haptic_config', type=str, default=None,
                       help='haptic_feedback.yaml path (thresholds/PWM for Manus Client; bridge timing)')
    parser.add_argument('-latency', action='store_true',
                       help='Print pipeline latency every 1s: recv→cmd / recv→UDP / opt / manus→recv')
    
    args = parser.parse_args()
    
    hand_serial = args.hand_serial
    filter_alpha = args.filter_alpha
    mocap_address = args.mocap_address
    hand_action_address = args.hand_action_address
    wave = args.wave
    sdk = args.sdk
    glove_left_ip = args.glove_left_ip
    glove_right_ip = args.glove_right_ip
    plot_enabled = args.plot
    plot_orientation = args.plot_orientation
    debug_print = args.debug_print
    smoother_hz = args.smoother_hz
    smoother_w = args.smoother_w
    smoother_z = args.smoother_z
    sdk_speed_coeff = args.sdk_speed_coeff
    sdk_current_coeff = args.sdk_current_coeff
    haptics = args.haptics
    haptic_pub_address = args.haptic_pub_address
    haptic_publish_hz = args.haptic_publish_hz
    haptic_debug = args.haptic_debug
    haptics_source = args.haptics_source
    haptic_force_sub = args.haptic_force_sub
    haptic_config = args.haptic_config
    latency = args.latency
    
    if hand_serial != 'WAVE':
        raise ValueError(f"Unsupported hand model: {hand_serial}, only WAVE (Sharpa Wave URDF) is supported")
    
    print(f"System configuration:")
    print(f"  Hand model: {hand_serial}")
    print(f"  Filter alpha: {filter_alpha}")
    print(f"  Mocap address: {mocap_address}")
    print(f"  Wave mode (UDP): {'Enabled' if wave else 'Disabled'}")
    print(f"  Sharpa SDK mode: {'Enabled' if sdk else 'Disabled'}")
    print(f"  Latency probe: {'Enabled' if latency else 'Disabled'}")
    print(f"  Haptics bridge: {'Enabled' if haptics else 'Disabled'}")
    if haptics:
        from tactile_haptic_bridge import default_haptic_config_path
        cfg_path = haptic_config or default_haptic_config_path()
        print(f"  Haptic PUB: {haptic_pub_address} @ {haptic_publish_hz}Hz")
        print(f"  Haptics source: {haptics_source}")
        print(f"  Haptic config YAML: {cfg_path}")
        print(f"  (Manus Client: export HAPTIC_FEEDBACK_CONFIG=\"{cfg_path}\" then restart)")
        if haptic_force_sub:
            print(f"  Haptic force SUB (host): {haptic_force_sub}")
    
    system = DualHandRetargetingSystem(
        hand_serial=hand_serial,
        filter_alpha=filter_alpha,
        mocap_address=mocap_address,
        hand_action_address=hand_action_address,
        wave=wave,
        sdk=sdk,
        glove_left_ip=glove_left_ip,
        glove_right_ip=glove_right_ip,
        plot_enabled=plot_enabled,
        plot_orientation=plot_orientation,
        debug_print=debug_print,
        smoother_hz=smoother_hz,
        smoother_w=smoother_w,
        smoother_z=smoother_z,
        sdk_speed_coeff=sdk_speed_coeff,
        sdk_current_coeff=sdk_current_coeff,
        haptics=haptics,
        haptic_pub_address=haptic_pub_address,
        haptic_publish_hz=haptic_publish_hz,
        haptic_debug=haptic_debug,
        haptics_source=haptics_source,
        haptic_force_sub=haptic_force_sub,
        haptic_config=haptic_config,
        latency=latency,
    )
    
    try:
        system.start()
        system.run()
    except KeyboardInterrupt:
        print("\nUser interrupted program execution")
    except Exception as e:
        print(f"Program execution error: {e}")
        raise e
    finally:
        system.stop()
        print("Program exit")

if __name__ == "__main__":
    main()

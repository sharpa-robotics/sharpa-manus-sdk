import sys
# After `sudo dpkg -i sharpa-wave-sdk_*.deb`, native module is under /opt/sharpa-wave-sdk/python/
sys.path.insert(0, '/opt/sharpa-wave-sdk/python')
import math
import time
from sharpa import (
    SharpaWave,
    SharpaWaveManager,
    ControlMode,
    ControlSource,
    HandSide,
    ErrorCode
)

print("Sharpa Wave Example - Starting")

# Joint names (indices 0..21)
JOINT_NAMES = [
    "Thumb CMC Flexion/Extension",
    "Thumb CMC Abduction/Adduction", 
    "Thumb MCP Flexion/Extension",
    "Thumb MCP Abduction/Adduction",
    "Thumb DIP Flexion/Extension",
    "Index MCP Flexion/Extension",
    "Index MCP Abduction/Adduction",
    "Index PIP Flexion/Extension", 
    "Index DIP Flexion/Extension",
    "Middle MCP Flexion/Extension",
    "Middle MCP Abduction/Adduction",
    "Middle PIP Flexion/Extension",
    "Middle DIP Flexion/Extension", 
    "Ring MCP Flexion/Extension",
    "Ring MCP Abduction/Adduction",
    "Ring PIP Flexion/Extension",
    "Ring DIP Flexion/Extension",
    "Pinky CMC Flexion/Extension",
    "Pinky MCP Flexion/Extension",
    "Pinky MCP Abduction/Adduction", 
    "Pinky PIP Flexion/Extension",
    "Pinky DIP Flexion/Extension",
]

# Angle ranges (in degrees)
ANGLE_RANGES = [
    (0, 50),   # Thumb CMC Flexion/Extension
    (0, 10),     # Thumb CMC Abduction/Adduction
    (0, 30),   # Thumb MCP Flexion/Extension
    (0, 10),     # Thumb MCP Abduction/Adduction
    (0, 40),     # Thumb DIP Flexion/Extension
    (0, 20),   # Index MCP Flexion/Extension
    (-20, 20),   # Index MCP Abduction/Adduction
    (0, 20),     # Index PIP Flexion/Extension
    (0, 20),     # Index DIP Flexion/Extension
    (0, 20),   # Middle MCP Flexion/Extension
    (-20, 20),   # Middle MCP Abduction/Adduction
    (0, 20),     # Middle PIP Flexion/Extension
    (0, 20),     # Middle DIP Flexion/Extension
    (0, 20),   # Ring MCP Flexion/Extension
    (-20, 20),   # Ring MCP Abduction/Adduction
    (0, 20),     # Ring PIP Flexion/Extension
    (0, 20),     # Ring DIP Flexion/Extension
    (0, 10),     # Pinky CMC Flexion/Extension
    (0, 20),   # Pinky MCP Flexion/Extension
    (-20, 20),   # Pinky MCP Abduction/Adduction
    (0, 20),     # Pinky PIP Flexion/Extension
    (0, 20),     # Pinky DIP Flexion/Extension
]

def auto_detect_hand() -> None:
    """Automatically detect device and return device and device serial number"""
    print("Searching for devices...")
    
    try:
        manager = SharpaWaveManager.get_instance()
        time.sleep(1)  # Wait for 1 seconds for device discovery to complete
        while True:
            devices = manager.get_all_device_sn()
            if not devices:
                print("No available devices found")
                time.sleep(1)
                continue
            else:
                print(f"Device found: {devices[0]}")
                return manager.connect(devices[0])
    except Exception as e:
        print(f"Failed to connect to device: {str(e)}")
        exit(1)
    
def initialize(hand) -> bool:
    error = hand.set_control_mode(ControlMode.POSITION)
    if error.code != 0:
        print(f"Failed to set control mode: {error.message}")
        return False
    error = hand.set_speed_coeff(0.3)
    if error.code != 0:
        print(f"Failed to set speed coeff: {error.message}")
        return False

    error = hand.set_current_coeff(0.6)
    if error.code != 0:
        print(f"Failed to set current coeff: {error.message}")
        return False
    error = hand.set_control_source(ControlSource.SDK)
    if error.code != 0:
        print(f"Failed to set control source: {error.message}")
        return False
    return True

def run_cycles(hand):
    cycle_duration_s = 1.
    dt = 0.01
    total_tick = int(cycle_duration_s / dt)
    cycles = 3
    angle_list = [0.0] * 22
    SKIP_MOVE_JOINT_INDEX = [6, 10, 14, 17, 19] # index of MCP joints
    for cycle in range(cycles):
        for tick in range(total_tick):
            for i in range(22):
                if i in SKIP_MOVE_JOINT_INDEX:
                    continue
                lo, hi = ANGLE_RANGES[i]
                target_angle = (lo + hi) / 2 + (hi - lo) / 2 * math.sin(2 * math.pi * tick / total_tick)
                angle_list[i] = math.radians(target_angle)
            error = hand.set_joint_position(angle_list, True)
            if error.code != 0:
                print(f"Failed to set joint position(rad): {error.message}")
            time.sleep(dt)
        error, angles = hand.get_joint_position_degree()
        if error.code != 0:
            print(f"Failed to get joint position(degree): {error.message}")
        print("angles", ",".join(f"{angle:.2f}" for angle in angles))

if __name__ == "__main__":
    print("Sharpa Wave Example - Starting")
    # Try to automatically detect device
    hand = auto_detect_hand()
    if hand is None:
        print("Error: No available device found")
        exit(1)
    print("Sharpa Wave Example - Init Hand Running Mode")
    if not initialize(hand):
        print("Error: Failed to initialize hand")
        exit(1)

    print("Sharpa Wave Example - Run Demo Gestures")
    hand.start()
    # reset hand to init position with interpolation mode (move smoothly)
    enable_interpolation_mode = True
    hand.set_joint_position([0.0] * 22, enable_interpolation_mode)
    run_cycles(hand)
    hand.set_joint_position([0.0] * 22, enable_interpolation_mode)

    print("Sharpa Wave Example - Stop Hand Running Mode")
    hand.stop()
    SharpaWaveManager.get_instance().disconnect_all()
    print("Sharpa Wave Example - Stopped")
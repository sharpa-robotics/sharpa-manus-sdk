# SharpaWave ROS2 Interface

This directory provides the ROS2 bridge server (Server) and control client (Client) for the SharpaWave dexterous hand.

## File Description


| File                 | Description                                                                                                                                                               |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `wave_ros_server.py` | **Server** — Connects to the dexterous hand device, publishes tactile data / joint states to ROS2, receives joint control commands, and auto-reconnects on device failure |
| `wave_ros_client.py` | **Client** — Controls the dexterous hand via ROS2 topics, supports multiple operation modes                                                                               |
| `requirements.txt`   | Dependency description                                                                                                                                                    |


## Environment Setup

```bash
# 1. Install ROS2 dependency packages
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  python3-colcon-common-extensions

# 2. Verify CMake version
cmake --version
# Expected output: cmake version 3.22.1

# 3. Build SharpaWaveSDK
cd /path/to/sharpa-wave-sdk && bash build.sh

# 4. Python dependencies
pip install numpy opencv-python


```

## Connect the Hand

After connecting the hand, go to Network Settings and select "Wired"; set IPv4 method to "Manual".
Address: 192.168.10.240, Subnet mask: 192.168.10.240

## Quick Start

Note: Sharpa-pilot and wave_ros_client.py cannot run simultaneously. Close Sharpa-pilot before starting.

Two terminals are required: one for the Server, one for the Client.

```bash
bash build.sh --clear
bash tools/install/install.sh --no-conda
```

### Terminal 1: Start the Server

```bash
source /opt/ros/humble/setup.bash
cd sharpa-wave-sdk/sample/ROS
python3 wave_ros_server.py
```

The Server will automatically: discover devices -> detect hand side (left/right) -> set control source to SDK -> enable motors -> start watchdog -> start publishing tactile and joint data on per-hand topics.

When you see `WaveRosServer running...`, it is ready. The built-in watchdog will periodically check device health and automatically reconnect if a device becomes unresponsive (e.g., after power cycling).

### Terminal 2: Start the Client

```bash
source /opt/ros/humble/setup.bash
cd sharpa-wave-sdk/sample/ROS
```

The Client has **4 operation modes**. Choose as needed:

---

## Client Operation Modes

### Mode 1: Demo Mode (recommended for first use)

Automatically controls a finger DIP joint (fingertip) from 0 deg to 60 deg and back. Left and right hands use different fingers so you can tell them apart:

- **Right hand** — moves `index_DIP` (index fingertip)
- **Left hand** — moves `middle_DIP` (middle fingertip)

```bash
python3 wave_ros_client.py --demo                 # both hands move simultaneously (default)
python3 wave_ros_client.py --hand right --demo    # right hand only: index finger bends
python3 wave_ros_client.py --hand left --demo     # left hand only: middle finger bends
```

**Expected result**: The target fingertip joint(s) slowly bend, hold for 2 seconds, then slowly return to the original position.
The terminal will print the angle during motion, the actual feedback angle for each hand, and the measured latency (ms).

### Mode 2: Send Preset Pose

A single command makes the hand smoothly transition to a preset pose (approximately 1.5 seconds), then automatically exits.

```bash
python3 wave_ros_client.py --pose open                     # Open both hands
python3 wave_ros_client.py --hand right --pose open        # Open right hand only
python3 wave_ros_client.py --hand left --pose open         # Open left hand only
```

### Mode 3: Send Custom Joint Angles

Directly specify 22 joint angles (radians, comma-separated), then automatically exit.

```bash
# Example: bend only index MCP joint to 0.35 rad (approx. 20 deg), others set to 0
python3 wave_ros_client.py --hand right --joints "0,0,0,0,0,0.35,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
```

See the "Joint Definitions" table below for joint index-to-name mapping.

### Mode 4: Continuous Listening

Start without any action arguments to continuously receive and print tactile data and joint state feedback from both hands. Useful for observing device status.

```bash
python3 wave_ros_client.py
python3 wave_ros_client.py --quiet   # reduce log output
```

Press `Ctrl+C` to exit.

---

## Client Parameters


| Parameter           | Description                                                                     | Example              |
| ------------------- | ------------------------------------------------------------------------------- | -------------------- |
| `--hand`            | Target hand: `left` / `right` / `both` (default: both)                          | `--hand left`        |
| `--demo`            | Run demo (right: index finger, left: middle finger)                             | `--demo`             |
| `--pose <name>`     | Send preset pose then exit (open)                                               | `--pose open`        |
| `--joints <values>` | Send 22 joint angles (radians) then exit                                        | `--joints "0,0,..."` |
| `--quiet`           | Quiet mode, reduce log output                                                   | `--quiet`            |
| No arguments        | Continuous listening mode (print tactile data and joint states from both hands) |                      |


## Server Parameters


| Parameter             | Default | Description                                                                           |
| --------------------- | ------- | ------------------------------------------------------------------------------------- |
| `--feedback_hz`       | `10.0`  | Joint state publishing frequency (Hz), set 0 to disable                               |
| `--channels`          | `0-9`   | Tactile channel range (0-4: right hand, 5-9: left hand)                               |
| `--watchdog_interval` | `3.0`   | Device health check interval in seconds; auto-reconnects after 5 consecutive failures |


## Architecture

```
┌──────────────┐                                    ┌──────────────┐
│  SharpaWave  │         ┌────────────────┐         │  WaveRos     │
│  Dexterous   │<- SDK ->│ WaveRosServer  │<- ROS ->│  Client      │
│  Hand Device │         │ (Bridge)       │         │  (Control)   │
└──────────────┘         └────────────────┘         └──────────────┘
```

**Data Flow**:

- **Server -> Client**: Tactile data (images, force), joint state feedback with timestamp (per hand)
- **Client -> Server**: Joint control commands with timestamp (per hand)

Latency is measured automatically: each message carries a `header.stamp` timestamp set by the sender; the receiver computes the difference to report per-message and average latency in milliseconds.

## ROS2 Topic Description

All topics are namespaced by hand side (`left` / `right`), so left and right hand data is fully separated.

### Topics Published by Server


| Topic                                  | Message Type                  | Description                       |
| -------------------------------------- | ----------------------------- | --------------------------------- |
| `wave/{hand}/tactile/{finger}/raw`     | `sensor_msgs/Image`           | Tactile raw image (mono8)         |
| `wave/{hand}/tactile/{finger}/force6d` | `geometry_msgs/WrenchStamped` | 6-axis force/torque               |
| `wave/{hand}/tactile/{finger}/deform`  | `sensor_msgs/Image`           | Deformation image (64FC1)         |
| `wave/{hand}/joint_states`             | `sensor_msgs/JointState`      | 22 joint angle feedback (radians) |


Where `{hand}` is: `left`, `right`
Where `{finger}` is: `thumb`, `index`, `middle`, `ring`, `pinky`

### Topics Published by Client


| Topic                        | Message Type             | Description                                  |
| ---------------------------- | ------------------------ | -------------------------------------------- |
| `wave/{hand}/joint_commands` | `sensor_msgs/JointState` | Joint position commands (radians, 22 joints) |


## Joint Definitions

22 degrees of freedom (FE = Flexion/Extension, AA = Abduction/Adduction):


| Index | Name          | Finger | Description                          |
| ----- | ------------- | ------ | ------------------------------------ |
| 0     | thumb_CMC_FE  | Thumb  | CMC flexion/extension (0~50 deg)     |
| 1     | thumb_CMC_AA  | Thumb  | CMC abduction/adduction (0~10 deg)   |
| 2     | thumb_MCP_FE  | Thumb  | MCP flexion/extension (0~30 deg)     |
| 3     | thumb_MCP_AA  | Thumb  | MCP abduction/adduction (0~10 deg)   |
| 4     | thumb_DIP     | Thumb  | DIP flexion/extension (0~40 deg)     |
| 5     | index_MCP_FE  | Index  | MCP flexion/extension (0~20 deg)     |
| 6     | index_MCP_AA  | Index  | MCP abduction/adduction (-20~20 deg) |
| 7     | index_PIP     | Index  | PIP flexion/extension (0~20 deg)     |
| 8     | index_DIP     | Index  | DIP flexion/extension (0~20 deg)     |
| 9     | middle_MCP_FE | Middle | MCP flexion/extension (0~20 deg)     |
| 10    | middle_MCP_AA | Middle | MCP abduction/adduction (-20~20 deg) |
| 11    | middle_PIP    | Middle | PIP flexion/extension (0~20 deg)     |
| 12    | middle_DIP    | Middle | DIP flexion/extension (0~20 deg)     |
| 13    | ring_MCP_FE   | Ring   | MCP flexion/extension (0~20 deg)     |
| 14    | ring_MCP_AA   | Ring   | MCP abduction/adduction (-20~20 deg) |
| 15    | ring_PIP      | Ring   | PIP flexion/extension (0~20 deg)     |
| 16    | ring_DIP      | Ring   | DIP flexion/extension (0~20 deg)     |
| 17    | pinky_CMC_FE  | Pinky  | CMC flexion/extension (0~10 deg)     |
| 18    | pinky_MCP_FE  | Pinky  | MCP flexion/extension (0~20 deg)     |
| 19    | pinky_MCP_AA  | Pinky  | MCP abduction/adduction (-20~20 deg) |
| 20    | pinky_PIP     | Pinky  | PIP flexion/extension (0~20 deg)     |
| 21    | pinky_DIP     | Pinky  | DIP flexion/extension (0~20 deg)     |


## Testing with ros2 Command Line

You can also operate directly with `ros2` commands without the Client script (Server must be running):

```bash
# List all wave topics
ros2 topic list | grep wave

# View right hand joint states
ros2 topic echo /wave/right/joint_states

# View left hand joint states
ros2 topic echo /wave/left/joint_states

# View right index finger force sensor
ros2 topic echo /wave/right/tactile/index/force6d

# View left thumb force sensor
ros2 topic echo /wave/left/tactile/thumb/force6d

# Send a one-time all-zero joint command to right hand (open hand)
ros2 topic pub --once /wave/right/joint_commands sensor_msgs/JointState \
  "{position: [0,0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,0]}"
```

## FAQ

**Q: Segmentation fault (core dumped)**

The SDK's `libfmt.so.10` conflicts with ROS2's `libfmt.so.8`. `wave_ros_server.py` has a built-in workaround, no extra steps needed. If you write your own script that uses both sharpa and rclpy, add this before imports:

```python
import ctypes
ctypes.CDLL("libfmt.so.8", mode=ctypes.RTLD_GLOBAL)
```

**Q: `ModuleNotFoundError: No module named 'sharpa'`**

Install the **sharpa-wave-sdk** deb. The server expects the native module under `/opt/sharpa-wave-sdk/python/` (`sharpa.cpython-*-linux-gnu.so`). For a local build tree, add that directory (or `build/install/python`) to `PYTHONPATH` instead.

**Q: `ros2` command not found**

```bash
source /opt/ros/humble/setup.bash
```

**Q: Device disconnected after power cycling**

The Server's built-in watchdog automatically detects unresponsive devices and reconnects when they come back online. You should see log messages like:

```
[WARN] [Watchdog] Device SN... unresponsive (5 failures), attempting reconnect...
[INFO] [Watchdog] Waiting for devices to come back online...
[INFO] [Watchdog] Reconnect complete  |  hands: [...]
```

No manual restart is needed. Adjust the check interval with `--watchdog_interval`.

**Q: Device connection timeout**

- Check that the device is powered on and connected
- Confirm you are on the same subnet as the device (usually 192.168.10.x)
- Use `ping <device_IP>` to verify network reachability

**Q: Fingers not moving after sending commands**

- Confirm Server log shows `control_source -> SDK` and `enable_state -> True`
- Confirm joint angles are within valid range (see Joint Definitions table)
- Check Server log for `[Joint] ... hand` output
- Make sure the `--hand` argument matches the connected device

## License

SharpaWave SDK © Sharpa Robotics
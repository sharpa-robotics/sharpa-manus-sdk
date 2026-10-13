# SharpaWave with Manus (Pro Haptics)

Real-time teleoperation stack for **Sharpa Wave** dexterous hands driven by **Manus Metaglove Pro Haptics**, including **WAVE fingertip tactile → Manus glove vibration** feedback.

This package is the **haptics-enabled** line of the Sharpa–Manus integration. Usage docs that previously lived in *SharpaWave with Manus User Manual.pdf* are consolidated here.

---

## Compatibility (important)

| Glove model | This package (current / haptics line) | What to use instead |
|-------------|----------------------------------------|---------------------|
| **Manus Metaglove Pro Haptics** | Supported (motion + vibration feedback) | — |
| **Manus Metaglove Pro** (non-haptics) | **Not supported** for this release line | Use release **[v1.0.1](https://github.com/sharpa-robotics/sharpa-manus-sdk/releases/tag/v1.0.1)** |
| Metaglove Pro Precision | No fingertip motors; do not expect vibration | Use **v1.0.1** for non-haptic teleop |

> **Summary:** This codebase targets **Manus Pro Haptics** only. If you have standard **Manus Pro** (no haptic motors), please use **version 1.0.1** instead of this branch/package.

---

## Overview

| Component | Role |
|-----------|------|
| `client/` | Linux Manus SDK client (C++). Reads glove RawSkeleton, converts to Sharpa keypoints, publishes via ZMQ/Protobuf; optionally subscribes to haptic forces and drives glove motors. |
| `retargeting_alg_release_V5.0/` | Python retargeting. Maps 25 keypoints → 22 Wave joint angles; can command hardware via UDP glove protocol (`-wave`) or SharpaWave SDK (`-sdk`); with `-haptics` bridges WAVE tactile F6 to Manus. |

**Data path (motion):**

```text
Manus Glove → SharpaManusClient → ZMQ MocapKeypoints (tcp://127.0.0.1:2044)
    → retargeting_manus_demo_multiprocess.py → Wave joints (-wave UDP / -sdk)
```

**Data path (haptics, optional):**

```text
WAVE tactile (UDP ~30 Hz/channel)
    → retargeting (-haptics) → ZMQ HapticFeedback (tcp://*:2045)
    → SharpaManusClient → CoreSdk_VibrateFingers (Metaglove Pro Haptics)
```

> **Glove requirement:** **Metaglove Pro Haptics** only. Standard Manus Pro is unsupported on this line (use **v1.0.1**). Pro Precision has no fingertip motors; Client will skip vibration with a warning.

---

## Repository layout

```text
sharpa-manus-sdk/
├── client/                              # Manus Linux client + CalibrationGUI
│   ├── SharpaManusClient.cpp / .hpp
│   ├── HapticFeedbackHandler.*          # force → PWM tiers → motors
│   ├── HapticConfig.*
│   ├── include/haptic_feedback.yaml     # thresholds / PWM (Client loads this)
│   ├── CalibrationGUI/
│   ├── ManusSDK/
│   └── Makefile
├── retargeting_alg_release_V5.0/        # Retargeting + WAVE bridge
│   ├── retargeting_manus_demo_multiprocess.py
│   ├── include/
│   │   ├── hand_retargeting_optimizer.so
│   │   ├── tactile_haptic_bridge.py
│   │   ├── haptic_feedback.yaml         # bridge + shared deadzone
│   │   ├── SharpaWaveSDK_5.0.4/         # bundled WAVE SDK (optional path)
│   │   └── proto_hand/
│   ├── urdf/
│   ├── environment.yml
│   └── requirements.txt
├── License
├── NOTICE.txt
└── README.md
```

---

## Hardware requirements

- **Manus Metaglove Pro Haptics** (dongle + SDK license key). Standard Manus Pro is **not** supported here — use **v1.0.1**.
- Sharpa Wave left/right hands on Ethernet.
- Recommended PC: Ubuntu 20.04 / 22.04 / 24.04 (64-bit), ≥32 GB RAM, modern Intel CPU.

### Network (typical)

| Device / role | Example IP |
|---------------|------------|
| PC wired NIC (tactile unicast target) | `192.168.10.240` |
| Wave left hand | `192.168.10.10` |
| Wave right hand | `192.168.10.20` |
| `-glove_left_ip` / `-glove_right_ip` (UDP control endpoints used by `-wave`) | same subnet, e.g. `192.168.10.98` / `192.168.10.99` |

Configure Wave IPs in **Sharpa Pilot** if needed. PC IPv4 must match the tactile target configured on the hands (default `192.168.10.240`) and must not collide with hand IPs.

---

## Software requirements

- **Sharpa Manus Client** — glove mocap + optional haptic motors.
- **Retargeting V5.0** — keypoints → Wave joints; optional tactile bridge.
- **Sharpa Pilot** — monitor devices; switch control source to **Glove** before teleop.
- Build deps: `g++`, Manus SDK (bundled under `client/ManusSDK`), Protobuf (client Makefile can vendor it), ZeroMQ, Eigen, libusb, etc.
- Python 3.10 conda env (`environment.yml`).

---

## Quick start (teleop + haptics)

### 1. Hardware

1. Power Wave hands and connect Ethernet.
2. Plug Manus dongle + license key; power gloves (LED solid blue when linked).
3. Set PC wired IP (e.g. `192.168.10.240`).

### 2. Build & run Manus Client

```bash
# System packages (Ubuntu)
sudo apt-get install -y \
  build-essential libusb-1.0-0-dev zlib1g-dev libudev-dev \
  gdb libncurses5-dev libeigen3-dev libglfw3-dev \
  libgl1-mesa-dev libglu1-mesa-dev libzmq3-dev \
  libprotobuf-dev protobuf-compiler libprotoc-dev

# udev rules for Manus HID
sudo tee /etc/udev/rules.d/70-manus-hid.rules > /dev/null <<'EOF'
# HIDAPI/libusb
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="83fd", MODE:="0666"
# HIDAPI/hidraw
KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"
EOF
# Reboot (or reload udev) after creating the rules

cd client
make
./SharpaManusClient.out
```

Healthy mocap stream looks like:

```text
INFO: Frame[xx] skeleton: Right is published. - System time: xx.xxs
INFO: Frame[xx] skeleton: Left is published. - System time: xx.xxs
```

Client binds **mocap PUB** at `tcp://127.0.0.1:2044` and connects **haptic SUB** to `tcp://127.0.0.1:2045`.

**Client keyboard (haptics):**

| Key | Action |
|-----|--------|
| `H` | Short index-finger motor pulse test |
| `J` | Synthetic Low/Mid/High tier test (no WAVE needed) |
| `T` | Toggle WAVE→Manus haptic path on/off |
| `D` | Toggle haptic debug print (force N + tier) |
| `Space` | Exit |

Config file: `client/include/haptic_feedback.yaml` (auto-loaded). Override with:

```bash
export HAPTIC_FEEDBACK_CONFIG=/path/to/haptic_feedback.yaml
```

Restart the Client after editing thresholds/PWM.

### 3. Retargeting environment

```bash
cd retargeting_alg_release_V5.0
conda env create -n sharpa -f environment.yml   # or: conda env update -n sharpa -f environment.yml
conda activate sharpa
```

### 4. Run retargeting with Wave + haptics

**Close Sharpa Pilot (or anything else holding tactile UDP ports, usually `50001` / `50011`) before `-haptics`**, otherwise the bridge cannot bind and Manus motors stay silent.

```bash
conda activate sharpa
cd retargeting_alg_release_V5.0

python3 retargeting_manus_demo_multiprocess.py \
  -wave -haptics -haptic_debug \
  -glove_left_ip 192.168.10.98 \
  -glove_right_ip 192.168.10.99
```

Adjust `-glove_*_ip` to your Wave / glove-protocol endpoints on the `192.168.10.x` subnet.

### 5. Pilot control source

1. Start **Sharpa Pilot** only for monitoring / mode switch if it does not steal tactile ports; if haptics fail with “port occupied”, stop Pilot and rely on SDK/`-wave` path.
2. Set Wave control source to **Glove**.
3. Teleoperate; pressing fingertips should produce vibration on Pro Haptics gloves when `-haptics` is running.

### Recommended startup order

1. `./SharpaManusClient.out`
2. `python3 retargeting_manus_demo_multiprocess.py -wave -haptics ...`
3. Pilot → Glove mode (if used)

---

## Calibration (Manus glove)

Per-user, per-glove calibration is required. Each operator–glove pair needs its own `.mcal` files.

```bash
cd client/CalibrationGUI
make
./CalibrationGUI.out
```

Typical flow (also matches Manus Knowledge Center):

1. Confirm glove status at the top of the GUI.
2. Place the hand flat as instructed → **F5** → wait ~5 s.
3. Make a fist → **F9** → wait ~5 s → **F9** for next step.
4. Fingertips to palm base repeatedly → **F9** when prompted.
5. Thumb to pinky base / extend outward → finish when “Calibration Complete” → **F9**.
6. Repeat for the other glove.
7. Copy `Calibration_left.mcal` / `Calibration_right.mcal` into `client/` (overwrite), then **restart** `SharpaManusClient.out`.

---

## Retargeting CLI reference

Entry: `retargeting_alg_release_V5.0/retargeting_manus_demo_multiprocess.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `-wave` | off | Send joints via UDP Wave glove protocol (MockGlove / heartbeat) |
| `-sdk` | off | Control via bundled SharpaWave SDK (mutually exclusive with `-wave`) |
| `-glove_left_ip` | `192.168.10.99` | Left UDP / glove target IP |
| `-glove_right_ip` | `192.168.10.100` | Right UDP / glove target IP |
| `-mocap_address` | `tcp://localhost:2044` | Subscribe Client mocap |
| `-hand_action_address` | `tcp://*:6668` | Optional ZMQ joint PUB (non-`-wave` paths) |
| `-haptics` | off | WAVE tactile → ZMQ `HapticFeedback` for Client |
| `-haptic_pub_address` | `tcp://*:2045` | Haptic ZMQ PUB bind |
| `-haptic_publish_hz` | `30` | Bridge publish rate |
| `-haptic_debug` | off | Print per-finger force magnitudes |
| `-haptics_source` | `auto` | `auto` / `sdk` / `upstream` (tactile source) |
| `-haptic_force_sub` | — | Upstream ZMQ SUB when using host-published forces |
| `-haptic_config` | — | Path to `haptic_feedback.yaml` |
| `-smoother_hz` | `250` | Joint command send rate after smoother |
| `-smoother_w` / `-smoother_z` | `100` / `0.2` | Smoother dynamics |
| `-filter_alpha` | `1.0` | Post-opt filter (0–1) |
| `-plot` / `-plot_orientation` | off | Matplotlib visualization |
| `-latency` | off | Print pipeline latency every 1 s |
| `-debug_print` | off | Extra debug |

SDK-only extras: `-sdk_speed_coeff`, `-sdk_current_coeff`.

---

## Haptics details

### Force → vibration

1. Bridge reads WAVE tactile **F6** per finger (~30 Hz), publishes `proto.HapticFeedback` (`left_force_mag` / `right_force_mag`, Newtons, order: thumb → pinky).
2. Client maps force to **Low / Mid / High** PWM tiers with hysteresis (`haptic_feedback.yaml`).
3. Motors run at `pwm_tick_hz` (default 50 Hz) with per-finger phase offset.

### WAVE tactile channels (SDK order)

| Channel | Hand | Finger (pinky → thumb) |
|---------|------|-------------------------|
| 0–4 | Right | pinky, ring, middle, index, thumb |
| 5–9 | Left | pinky, ring, middle, index, thumb |

Bridge remaps to Manus motor order: **thumb, index, middle, ring, pinky**.

### Port conflicts

Tactile UDP (often `50001` right / `50011` left on the PC) can only be owned by **one** process. If Pilot or another SDK app holds the port, `-haptics` will fail to start tactile. Use `ss -ulnp | grep -E '50001|50011'` to check.

### Tuning

Edit `client/include/haptic_feedback.yaml` (Client) and/or `retargeting_alg_release_V5.0/include/haptic_feedback.yaml` (bridge section). Keep threshold/PWM blocks consistent when both are used.

---

## Developer notes

### Mocap keypoints

- Shape: left/right `(25, 7)` — `[x, y, z, qw, qx, qy, qz]`.
- Position in meters; orientation quaternion.
- Wrist (index 0): position `0,0,0`, orientation `1,0,0,0`.
- Only **fingertip** orientations are used by the optimizer; other orientations may be identity.
- Order: wrist → thumb (4) → index (5) → middle (5) → ring (5) → pinky (5). See Manus RawSkeleton docs: [Manus SDK Client skeletons](https://docs.manus-meta.com/latest/Plugins/SDK/SDK%20Client/#skeletons).
- Use **RawSkeleton**, not Manus “Retargeting Skeleton”.

### Joint outputs

- 22 joints per hand, radians.
- Order example (right): thumb CMC_FE/AA, MCP_FE/AA, IP → index MCP_FE/AA, PIP, DIP → middle → ring → pinky CMC, MCP_FE/AA, PIP, DIP.
- Closed-source optimizers: `include/hand_retargeting_optimizer.so`, `include/hand_kinematic_casadi.so`.

### ZMQ topics (this package)

| Stream | Default | Direction |
|--------|---------|-----------|
| `MocapKeypoints` | `tcp://127.0.0.1:2044` | Client → Retargeting |
| `HapticFeedback` | `tcp://127.0.0.1:2045` | Retargeting → Client |
| Joint PUB (optional) | `tcp://*:6668` | Retargeting → subscribers |

Protobuf: `client/sharpa_hand.proto` / `retargeting_alg_release_V5.0/include/proto_hand/`.

---

## Installation notes (from legacy manual)

1. **protoc / protobuf version**  
   Client build expects protobuf tooling compatible with the vendored/`third_party` flow in `client/Makefile`. If a system `protoc` older than 3.12.4 breaks the build, temporarily move it aside, rebuild with the Makefile’s protobuf, then restore. See `client/download_protobuf.sh` and `client/protobuf_problem.txt`.

2. **Ubuntu 20.04 cppzmq**  
   Distro headers may be too old. Install header-only cppzmq ≥ 4.7 (e.g. 4.10.0) into `/usr/local/include` (`zmq.hpp`, `zmq_addon.hpp`).

3. **libfmt / ROS**  
   If you also run WAVE ROS samples (`include/SharpaWaveSDK_*/sample/ROS`), preload system `libfmt.so.8` before mixing ROS Humble with the WAVE SDK (see SDK sample README). **This Manus teleop stack does not require ROS.**

---

## Troubleshooting

| Symptom | What to check |
|---------|----------------|
| No skeleton published | Dongle/license, udev rules, glove pairing, Client logs |
| Wave not moving | `-wave`/`-sdk`, glove IPs, Pilot control source = Glove, network |
| No vibration | Pro Haptics glove? `-haptics` running? Port `50001`/`50011` free? Client `T` enabled? |
| CasADi Inf spam | Bad/zero mocap frames; Client pairing; single-hand zeros are skipped when valid |
| High latency | `-latency` flag; reduce load; `-smoother_hz` |
| Haptics sticky after release | `finger_stale_timeout_sec` in yaml; ensure tactile callbacks still flowing |

---

## License

See `License` and `NOTICE.txt`. Manus SDK binaries and headers under `client/ManusSDK/` remain subject to Manus license terms. SharpaWave SDK under `retargeting_alg_release_V5.0/include/SharpaWaveSDK_*` is subject to Sharpa terms.

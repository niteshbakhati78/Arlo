# ArloBot SLAM — ROS 2 Autonomous Mobile Robot

A ROS 2 package for autonomous navigation and SLAM (Simultaneous Localization and Mapping) on an Arlo differential-drive robot platform, powered by a **Xilinx KV260 FPGA** for low-level motor control and encoder reading.

---

## Overview

This project integrates:

- **Custom FPGA firmware** (via PYNQ) for PWM motor control and quadrature encoder decoding
- **Wheel odometry** from encoder tick counts
- **RPLIDAR** for 2D laser scanning
- **SLAM Toolbox** for online graph-based mapping
- **Nav2** for full autonomous navigation (path planning, AMCL localization, costmaps)

---

## Hardware

| Component | Details |
|-----------|---------|
| Robot chassis | Arlo differential-drive platform |
| Compute | Xilinx KV260 FPGA SoM (running Ubuntu + PYNQ) |
| FPGA bitstream | `kv260_v4.bit` (motor PWM + encoder decoder IPs) |
| LiDAR | RPLIDAR A2/A3 — connected via `/dev/ttyUSB0` @ 115200 baud |
| Camera | USB webcam — V4L2, 640×480 @ 5 fps |
| Wheels | 6-inch diameter (0.1524 m), 35 encoder counts/revolution |
| Wheel base | ~0.39 m |

**Robot dimensions (from URDF):**
- Base radius: 22.45 cm
- LiDAR mount: 12.5 cm forward, 13.5 cm above base_link
- Camera mount: 20.2 cm forward, 28.5 cm above base_link

---

## Software Stack

| Layer | Tool |
|-------|------|
| Middleware | ROS 2 (Humble) |
| Language | Python 3.10 |
| FPGA interface | PYNQ |
| SLAM | SLAM Toolbox (sync online mapper) |
| Navigation | Nav2 (AMCL, DWA planner, BT navigator) |
| Camera | OpenCV / cv_bridge |

---

## Package Structure

```
arlobot/
├── arlobot/                        # Python nodes
│   ├── motor_controller.py         # /cmd_vel → FPGA PWM
│   ├── encoder_logger.py           # FPGA encoder → ROS topics
│   ├── odom_node.py                # Encoder ticks → /odom + TF
│   ├── camera_pub.py               # USB camera → /image_raw
│   └── calibration_tester.py       # Velocity calibration + CSV logging
├── config/
│   ├── slam_toolbox_params.yaml    # SLAM Toolbox parameters
│   ├── nav2_params.yaml            # Nav2 full stack config
│   └── costmap_*.yaml              # Global/local costmap configs
├── launch/
│   ├── arlobot_slam_launch.py      # SLAM pipeline (mapping mode)
│   ├── arlobot_navigation_launch.py # Full Nav2 navigation
│   └── localization_launch.py      # Localization only (AMCL)
└── urdf/
    └── arlobot.urdf                # Full robot model
```

---

## Prerequisites

### System
- Ubuntu 22.04 on KV260 (PYNQ image)
- ROS 2 Humble

### ROS 2 Dependencies

```bash
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-nav2-bringup \
                 ros-humble-rplidar-ros \
                 ros-humble-robot-state-publisher \
                 ros-humble-tf2-ros \
                 python3-opencv
```

### Python Dependencies (PYNQ environment)

```bash
# PYNQ is pre-installed on KV260; ensure the venv is accessible
/usr/local/share/pynq-venv/bin/pip install pynq
```

### FPGA Bitstream

Place the custom bitstream at:

```
/home/ubuntu/arlo/kv260_v4.bit
```

This bitstream exposes two IPs:
- `motor_pwm_0` — dual-channel PWM output (register `0x00` = left, `0x04` = right)
- Two quadrature decoder IPs — one per wheel, mapped to encoder tick counts

---

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url> .

cd ~/ros2_ws
colcon build --packages-select arlobot
source install/setup.bash
```

---

## Running the Robot

Open three terminals. **Terminal 1 must run as root** because PYNQ requires direct FPGA hardware access.

### Terminal 1 — Launch the full system (on the KV260)

```bash
sudo su
source /etc/profile.d/pynq_venv.sh
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
cd arlobot_ws/
colcon build
source install/setup.bash
ros2 launch arlobot navigation_launch.py
```

This starts the motor controller, encoder logger, odometry node, RPLIDAR driver, and the Nav2 navigation stack.

### Terminal 2 — Keyboard teleoperation

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use keyboard keys to drive the robot and explore the environment while SLAM builds the map.

### Terminal 3 — RViz2 visualization (on a remote machine or the same host)

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
rviz2
```

In RViz2, add the following displays: **Map**, **LaserScan** (`/scan`), **Odometry** (`/odom`), and **RobotModel**. Use **2D Pose Estimate** to initialize AMCL localization and **Nav2 Goal** to send autonomous navigation targets.

> **Note:** `ROS_DOMAIN_ID=0` and `ROS_LOCALHOST_ONLY=0` must match across all terminals (and machines) for ROS 2 topics to be visible over the network.

---

## Usage

### 1. SLAM — Build a Map

Launches the robot hardware nodes, RPLIDAR, and SLAM Toolbox in online mapping mode:

```bash
ros2 launch arlobot arlobot_slam_launch.py
```

Drive the robot around to build a map. Then save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 2. Navigation — Use a Saved Map

Launches Nav2 with AMCL localization on a previously saved map:

```bash
ros2 launch arlobot arlobot_navigation_launch.py map:=~/maps/my_map.yaml
```

Use RViz2 to set an initial pose (2D Pose Estimate) and send navigation goals (Nav2 Goal).

### 3. Localization Only

If you only want AMCL localization without the full Nav2 stack:

```bash
ros2 launch arlobot localization_launch.py map:=~/maps/my_map.yaml
```

### 4. Calibration

Run the calibration tester to log encoder/odometry data to CSV for tuning:

```bash
ros2 run arlobot calibration_tester
```

---

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (input to motor controller) |
| `/left_ticks` | `std_msgs/Int32` | Left wheel encoder tick count |
| `/right_ticks` | `std_msgs/Int32` | Right wheel encoder tick count |
| `/odom` | `nav_msgs/Odometry` | Computed wheel odometry |
| `/scan` | `sensor_msgs/LaserScan` | RPLIDAR scan data |
| `/image_raw` | `sensor_msgs/Image` | USB camera frames |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM-generated map |

---

## TF Tree

```
map
 └── odom              (published by SLAM Toolbox)
      └── base_link    (published by odom_node)
           └── laser   (static: -0.125m x, +0.135m z)
```

---

## SLAM Configuration

Key parameters in [config/slam_toolbox_params.yaml](arlobot/config/slam_toolbox_params.yaml):

| Parameter | Value | Notes |
|-----------|-------|-------|
| `resolution` | 0.03 m | 3 cm grid cells |
| `min_laser_range` | 0.2 m | Ignore returns closer than this |
| `max_laser_range` | 3.5 m | Effective RPLIDAR range |
| `do_loop_closing` | true | Corrects accumulated drift |
| `scan_buffer_size` | 10 | Scans held for matching |
| `map_update_interval` | 0.3 s | Map refresh rate |

---

## Motor PWM Calibration

The motor controller maps `geometry_msgs/Twist` linear velocity to PWM microseconds:

| State | PWM (µs) |
|-------|----------|
| Stop | 1500 (50%) |
| Slow forward | 1650 |
| Full forward | 1800 |
| Slow reverse | 1450 |
| Full reverse | 1300 |

Max configured speed: **0.3 m/s**. Tune `PWM_FWD_MIN/MAX` and `PWM_REV_MIN/MAX` in [motor_controller.py](arlobot/arlobot/motor_controller.py) to match your motors.

---

## Troubleshooting

**Motor controller fails to start**
- Verify the FPGA bitstream exists at `/home/ubuntu/arlo/kv260_v4.bit`
- Confirm PYNQ is available: `python3 -c "from pynq import Overlay"`

**RPLIDAR not detected**
- Check `/dev/ttyUSB0` is present: `ls /dev/ttyUSB*`
- Add your user to the `dialout` group: `sudo usermod -aG dialout $USER`

**SLAM map drifts or is inaccurate**
- Run `calibration_tester` and verify the actual vs. expected distance/rotation
- Tune wheel diameter and wheel base in `odom_node.py`

**TF errors / missing transforms**
- Ensure all nodes in the launch file started successfully
- Check: `ros2 run tf2_tools view_frames`

---

## License

This project does not currently include a license. Add one to the `package.xml` and repository before publishing.

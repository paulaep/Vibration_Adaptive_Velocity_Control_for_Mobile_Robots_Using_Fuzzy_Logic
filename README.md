# Vibration-Adaptive Velocity Control for Mobile Robots Using Fuzzy Logic

This repository contains the source code, launch configuration, and recorded experimental data accompanying the MSc thesis *Vibration-Adaptive Velocity Control for Mobile Robots Using Fuzzy Logic: Online IMU-Based Approach*, University of Turku, 2026.

The system implements a shared-control teleoperation architecture in which a human operator commands the trajectory of an iRobot Create 3 mobile robot through a PS4 wireless game controller, and a Mamdani fuzzy logic controller modulates the magnitude of the commanded velocity in response to vibration measured by an onboard MPU-9250 inertial measurement unit. The architectural and methodological details are documented in the thesis; this repository documents the implementation only.

## System overview

The system is distributed across three hosts communicating over ROS 2 Humble with CycloneDDS:

- **Raspberry Pi 4** (mounted on the robot): hosts the IMU acquisition node, the joystick driver, and the teleoperation node.
- **Development laptop**: hosts the vibration feature node, the fuzzy controller node, and the command fusion node.
- **iRobot Create 3**: receives the modulated velocity command via its native ROS 2 interface.

The four custom ROS 2 nodes and their topic interfaces are summarised below; each maps to a specific section of the thesis.

| Node                       | Host          | Subscribes to                 | Publishes to            |
|----------------------------|---------------|-------------------------------|-------------------------|
| `imu_reader_node`          | Raspberry Pi  | —                             | `/imu/raw`              |
| `vibration_feature_node`   | Laptop        | `/imu/raw`                    | `/vibration_feature`    |
| `fuzzy_alpha_node`         | Laptop        | `/vibration_feature`          | `/alpha`                |
| `cmd_vel_fusion_node`      | Laptop        | `/cmd_vel_user`, `/alpha`     | `/S2R5/cmd_vel`         |

## Repository structure

```
.
├── laptop_ws/
│   └── src/control_pkg/
│       ├── control_pkg/
│       │   ├── __init__.py
│       │   ├── vibration_feature_node.py
│       │   ├── fuzzy_alpha_node.py
│       │   └── cmd_vel_fusion_node.py
│       ├── launch/
│       │   └── adaptive_control_launch.py
│       ├── package.xml
│       └── setup.py
├── pi_ws/
│   └── src/imu_pkg/
│       ├── imu_pkg/
│       │   ├── __init__.py
│       │   └── imu_reader_node.py
│       ├── launch/
│       │   └── pi_teleop_imu_launch.py
│       ├── config/
│       │   └── ps4_teleop.yaml
│       ├── package.xml
│       └── setup.py
├── bags/                   (recorded experimental data; nine runs)
├── analysis/               (Python scripts used to generate the figures of Chapter 5)
├── LICENSE
└── README.md
```

## Software environment

The system was developed and tested with the following versions. Other versions may work but are not guaranteed to.

- Ubuntu 22.04 LTS (Server on the Pi, Desktop on the laptop)
- ROS 2 Humble Hawksbill
- CycloneDDS as the ROS Middleware implementation
- Python 3.10
- `numpy`, `smbus`, `rclpy`

The iRobot Create 3 firmware must be at a release certified for ROS 2 Humble.

## Build and installation

On both the laptop and the Raspberry Pi, set the following environment variables before any ROS 2 command (add them to `~/.bashrc` to make them persistent):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=78
export ROS_LOCALHOST_ONLY=0
```

### On the laptop

```bash
cd laptop_ws
colcon build
source install/setup.bash
```

### On the Raspberry Pi

```bash
cd pi_ws
colcon build
source install/setup.bash
```

Enable the I2C interface on the Pi (one-time setup via `raspi-config`).

## Running the system

Start the Pi side first, so that `/imu/raw` and `/cmd_vel_user` are present on the network when the laptop side starts.

### On the Raspberry Pi

```bash
ros2 launch imu_pkg pi_teleop_imu_launch.py
```

This starts `imu_reader_node`, `joy_node`, and `teleop_twist_joy`.

### On the laptop

For a **fuzzy-ON** run (the controller is active):

```bash
ros2 launch control_pkg adaptive_control_launch.py fuzzy_enabled:=true
```

For a **fuzzy-OFF** run (the controller is bypassed and the operator's command is passed through unchanged):

```bash
ros2 launch control_pkg adaptive_control_launch.py fuzzy_enabled:=false
```

To record a bag during the run, add `record_bag:=true bag_name:=<your_name>` to the launch command.

## Experimental data

The `bags/` directory contains the recorded ROS 2 bag files for the nine experimental runs reported in Chapter 5 of the thesis. Each bag is named `<condition>_<repetition>` (for example, `fuzzy_on_01`, `fuzzy_off_03`).

Bag files are stored using Git LFS due to their size. To access them after cloning, install Git LFS and run `git lfs pull`.

## Reproducibility notes

The numerical results reported in Chapter 5 of the thesis can be reproduced from the bag files in `bags/` using the analysis scripts in `analysis/`. The fuzzy controller is deterministic and stateless, so the `/alpha` traces shown in the thesis figures are mathematically identical whether they are read from the live `/alpha` topic in the bag or reconstructed offline by re-applying the inference to the recorded `/vibration_feature`.

The membership-function parameter values hard-coded in `fuzzy_alpha_node.py` correspond to the final tuning iteration documented in Section 4.4.2 of the thesis. Modifying these values will change the controller's behaviour but will not change the architecture.

## License

This work is released under the MIT License. See `LICENSE` for the full text.

## Author and citation

Paula Estepa Pérez, University of Turku, 2026.

If you use this work, please cite the thesis:

> Estepa Pérez, P. (2026). *Vibration-Adaptive Velocity Control for Mobile Robots Using Fuzzy Logic: Online IMU-Based Approach.* MSc thesis, University of Turku.

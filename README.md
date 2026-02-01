# Unitree Low Level

This project provides a low-level controller for Unitree Go2 and G1 robots.

## Features

- Acts as a hardware interface: reads sensor data, calls the high-level controller, and sends commands to the robot.
- Emergency stop (E-stop) support.
- Provides IK + PD control for robot initialization.

## Prerequisites

### [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2.git)
Used to communicate with Unitree Go2/G1 robots via DDS. Since ROS 2 also uses DDS as the underlying transport, they are naturally connected and accessible as long as the `ROS_DOMAIN_ID` and network interface match.

### [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2.git)
Provides ROS 2 message packages so nodes can decode the Unitree messages. Otherwise, `ros2 topic list` can see topics, but `ros2 topic echo` cannot decode them. On Foxy, the default ROS 2 DDS implementation differs from `unitree_sdk2`, so you must build Cyclone DDS following the `unitree_ros2` instructions (this is included in [scripts/colcon-config.sh](scripts/colcon-config.sh)).

### [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco.git)
Used for simulation validation before hardware deployment. Unitree MuJoCo provides the same API as the hardware, so you can switch between simulation and hardware by setting `ROS_DOMAIN_ID` and the network interface.

Note: Unitre Mujoco use src/unitree_mujoco/simulate/config.yaml to config, remeber set use_joystick = 1 if you need joystick.
```bash
source src/unitree_lowlevel/scripts/setup.sh <network-interface> $ROS_DISTRO
./src/unitree_mujoco/simulate/build/unitree_mujoco -i 0 -n $NetworkInterface
```
Note: By design, Unitree uses `ROS_DOMAIN_ID=0` on hardware and suggests `ROS_DOMAIN_ID=1` for simulation. Because simulation and hardware often run on different interfaces (e.g., `lo` and `eth0`), this repo uses `ROS_DOMAIN_ID=0` for both settings.

### Network Connection
- Go2 MCU
    - IP: 192.168.123.161
- Jetson
    - IP: 192.168.123.18
    - Username: unitree
    - Password: 123

### Visualization on Jetson with NoMachine
```bash
sudo service gdm3 stop
sudo init 3
sudo /etc/NX/nxserver --restart
```

## Installation

### Docker Installation

You have to install [Docker](https://docs.docker.com/engine/install/ubuntu/) and [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) first. 

#### Host Machine
```bash
docker compose up -d --build       # build inage and start container
docker exec -it unitree_ws bash     # attach container
```

### Dependencies (Skip this if you use docker)
```bash
sudo apt install -y python-is-python3 libopenblas-dev python3-dev python3-vcstool libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev libfmt-dev
```

### Build
```bash
mkdir -p unitree_ws/src
cd unitree_ws/src
git clone https://github.com/Renkunzhao/unitree_lowlevel.git
cd unitree_lowlevel
./scripts/colcon-config.sh $ROS_DISTRO <Release|Debug>
```

### Run
```bash
# Run test and stop the default controller (run once per boot)
source src/unitree_lowlevel/scripts/setup.sh <network-interface> $ROS_DISTRO
./build/unitree_sdk2/bin/go2_stand_example $NetworkInterface

# Low level controller
ros2 run unitree_lowlevel go2_lowlevel_node $NetworkInterface $WORKSPACE/src/unitree_lowlevel/config/go2.yaml
```

## Usage

### States & Keys

!!! L2 + B -> E-stop


IDLE (initial state, zero torque)
- L2 + A -> STAND

STAND (IK + PD)
- ly, lx, ry, rx -> Height, Roll, Pitch, Yaw
- L2 + A -> IDLE
- START -> High-level controller (needs user implementation; not included here)

High-level controller
- SELECT -> STAND


For a demonstration of how to implement a high-level controller, see [legged_rl_deploy](https://github.com/Renkunzhao/legged_rl_deploy.git).


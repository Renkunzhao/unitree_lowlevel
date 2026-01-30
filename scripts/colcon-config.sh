#!/bin/bash

# Default build type is Release
ROS_DISTRO="${1:-humble}"
BUILD_TYPE="${2:-Release}"

echo "Building with ROS_DISTRO=$ROS_DISTRO and CMAKE_BUILD_TYPE=$BUILD_TYPE"

# Get project root directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"/../../../

# Clone repos
vcs import $PROJECT_DIR/src < $PROJECT_DIR/src/unitree_lowlevel/scripts/deps.repos --debug -w $(nproc)

# Checkout correct ROS distro branches
cd $PROJECT_DIR/src/rmw_cyclonedds
git checkout $ROS_DISTRO

# Build
cd $PROJECT_DIR
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to manif unitree_sdk2
source install/setup.bash
colcon build --packages-up-to unitree_lowlevel --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE 

# MUJOCO + UNITREE MUJOCO
echo "=== Downloading Mujoco (Simulation Mode) ==="
mkdir -p $HOME/.mujoco
cd $HOME/.mujoco
wget -nc https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-$(uname -m).tar.gz
tar -xvf mujoco-3.3.6-linux-$(uname -m).tar.gz

echo "=== Building Unitree Mujoco ==="
cd $PROJECT_DIR/src/unitree_mujoco/simulate
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
mkdir build
cd build
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=$BUILD_TYPE 
make -j$(nproc)

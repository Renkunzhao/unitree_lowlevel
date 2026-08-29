#!/bin/bash

# Default build type is Release
ROS_DISTRO="${1:-humble}"
BUILD_TYPE="${2:-Release}"

echo "Building with ROS_DISTRO=$ROS_DISTRO and CMAKE_BUILD_TYPE=$BUILD_TYPE"

# Get project root directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"/../../../
echo "Project directory: $PROJECT_DIR"
mkdir -p $PROJECT_DIR/lib
mkdir -p $PROJECT_DIR/src

# Clone repos
vcs import $PROJECT_DIR/lib < $PROJECT_DIR/src/unitree_lowlevel/scripts/lib.repos --debug -w $(nproc)
vcs import $PROJECT_DIR/src < $PROJECT_DIR/src/unitree_lowlevel/scripts/src.repos --debug -w $(nproc)

# Checkout correct ROS distro branches
cd $PROJECT_DIR/lib/rmw_cyclonedds
git checkout $ROS_DISTRO

# unitree_sdk2
cd $PROJECT_DIR/lib/unitree_sdk2
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
make -j$(nproc)
sudo make install

# unitree_mujoco
echo "=== Downloading Mujoco (Simulation Mode) ==="
mkdir -p $HOME/.mujoco
cd $HOME/.mujoco
wget -nc https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-$(uname -m).tar.gz
tar -xvf mujoco-3.3.6-linux-$(uname -m).tar.gz

echo "=== Building Unitree Mujoco ==="
cd $PROJECT_DIR/src/unitree_mujoco/simulate
if [ ! -e mujoco ]; then
    ln -s ~/.mujoco/mujoco-3.3.6 mujoco
fi

# Build
cd $PROJECT_DIR
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to unitree_lowlevel unitree_mujoco --cmake-args \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DPython_EXECUTABLE=/usr/bin/python \
    -DPython3_EXECUTABLE=/usr/bin/python3

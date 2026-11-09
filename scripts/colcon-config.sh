#!/bin/bash
set -e

# Default build type is Release
ROS_DISTRO="${1:-humble}"
BUILD_TYPE="${2:-Release}"
SIM_MODE="${3:-}"
BUILD_PACKAGES=(unitree_lowlevel)

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

if [ "$SIM_MODE" = "sim" ]; then
    # unitree_mujoco
    echo "=== Downloading Mujoco (Simulation Mode) ==="
    MUJOCO_VERSION=4.0.0
    MUJOCO_ARCHIVE="mujoco-${MUJOCO_VERSION}-linux-$(uname -m).tar.gz"
    mkdir -p "$HOME/.mujoco"
    cd "$HOME/.mujoco"
    if [ ! -f "$MUJOCO_ARCHIVE" ]; then
        wget "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_ARCHIVE}" -O "${MUJOCO_ARCHIVE}.part"
        mv "${MUJOCO_ARCHIVE}.part" "$MUJOCO_ARCHIVE"
    fi
    tar -xf "$MUJOCO_ARCHIVE"

    echo "=== Building Unitree Mujoco ==="
    cd "$PROJECT_DIR/src/unitree_mujoco/simulate"
    if [ -e mujoco ] && [ ! -L mujoco ]; then
        echo "Expected simulate/mujoco to be a symlink; move the existing directory first." >&2
        exit 1
    fi
    ln -sfnT "$HOME/.mujoco/mujoco-${MUJOCO_VERSION}" mujoco
    BUILD_PACKAGES+=(unitree_mujoco)
fi

# Build
cd $PROJECT_DIR
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to "${BUILD_PACKAGES[@]}" --cmake-args \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DPython_EXECUTABLE=/usr/bin/python \
    -DPython3_EXECUTABLE=/usr/bin/python3

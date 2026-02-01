# Ubuntu 22.04 image based on ROS Humble (supports both x86_64 and arm64)
FROM ros:humble-perception-jammy

ENV ROS_DISTRO=humble

# Disable interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    # APT / certificates / repository management / GPG keys
    # (apt-transport-https is usually unnecessary on Ubuntu 22.04)
    apt-get install -y --no-install-recommends \
      ca-certificates gnupg lsb-release software-properties-common && \
    \
    # Networking and troubleshooting tools
    apt-get install -y --no-install-recommends \
      iputils-ping iproute2 wget curl net-tools && \
    \
    # Build / debug / code management tools
    apt-get install -y --no-install-recommends \
      build-essential cmake gdb git git-lfs vim python3-vcstool && \
    \
    # Common C++ math and utility libraries
    apt-get install -y --no-install-recommends \
      libboost-all-dev libeigen3-dev libyaml-cpp-dev && \
    \
    # JSON processing / code formatting / miscellaneous utilities
    apt-get install -y --no-install-recommends \
      jq clang-format unzip ncdu && \
    \
    # GUI support (kept since GUI is required; gnome-terminal is large but convenient)
    apt-get install -y --no-install-recommends \
      dbus-x11 gnome-terminal && \
    \
    # Hardware / USB / joystick support
    apt-get install -y --no-install-recommends \
      udev usbutils joystick && \
    \
    # Python toolchain (pip)
    apt-get install -y --no-install-recommends \
      python3-pip && \
    \
    git lfs install

    
    # ROS dependencies
RUN apt-get install -y --no-install-recommends \
        ros-humble-plotjuggler-ros ros-humble-pinocchio

    # Install additional dependencies
    # unitree_sdk2
RUN apt-get install -y --no-install-recommends \
        libspdlog-dev libfmt-dev && \
    \
    # unitree_ros2
    apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl && \
    \
    # unitree_mujoco
    apt-get install -y --no-install-recommends \
        libglfw3-dev

# Copy workspace setup script into the container
COPY workspace-config.sh /workspace-config.sh

# Use workspace-config.sh as the container entrypoint
ENTRYPOINT ["/workspace-config.sh"]

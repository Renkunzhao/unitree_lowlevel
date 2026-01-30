#!/bin/bash

if [ $# -ne 2 ]; then
  echo "Usage: $0 <network_interface> <ros_distro>"
  echo "Example: $0 eth0 foxy"
  exit 1
fi

IFACE="$1"
ROS_DISTRO="${2}"

echo "Setup unitree ros2 environment (interface: ${IFACE})"

# Get project root directory
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"/../../../

export WORKSPACE=$PROJECT_DIR
export NetworkInterface=${IFACE}

source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash

sudo ip link set dev lo multicast on

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\" />
</Interfaces></General></Domain></CycloneDDS>"

ros2 topic list

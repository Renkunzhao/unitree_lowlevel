# [Installation](https://github.com/realsenseai/librealsense/releases) 
[Linux](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)

[Jetson](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)

```bash
# install realsense-ros
sudo apt install ros-<ROS_DISTRO>-realsense2-* ros-<ROS_DISTRO>-rqt-image-view
```

# Command
```bash
# check profile
rs-enumerate-devices -c -o
realsense-viewer

# https://github.com/realsenseai/realsense-ros
ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -r __ns:=/camera \
    -p depth_module.profile:=848x480x90 \
    -p enable_color:=false

ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=848x480x90 \
  enable_color:=false

ros2 run rqt_image_view rqt_image_view

ros2 bag record -o D435i \
  $(ros2 topic list | awk '/^\/camera\//')

ros2 bag record -o D435i /camera/depth/image_rect_raw /bridge_mjlab_raw_depth/depth_image /lowstate /lowcmd

env -u CYCLONEDDS_URI RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 bag play ~/D435i

rsync -avP  unitree@100.88.41.38:/home/unitree/D435i ~/

tar -I 'zstd -9 -T0' -cf D435i.tar.zst D435i/
rsync -avP  unitree@100.88.41.38:/home/unitree/D435i.tar.zst ~/
rsync -avP  unitree@100.88.41.38:/home/unitree/code/unitree_ws/D435i.tar.zst ~/code/unitree_ws/data/
tar -I zstd -xf data/D435i.tar.zst


```

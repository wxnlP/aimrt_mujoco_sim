#!/bin/bash

set -e

export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/ros2_log}"
mkdir -p "$ROS_LOG_DIR"

# 相机位于 (0, -2, 0.9)，向下倾斜 20 度；这里发布光学坐标系（x 右、y 下、z 前）
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y -2 --z 0.9 \
  --yaw 0 --pitch 0 --roll -1.919862 \
  --frame-id map --child-frame-id camera

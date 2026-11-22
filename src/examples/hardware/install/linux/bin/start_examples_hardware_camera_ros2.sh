#!/bin/bash

set -e

source install/share/aimrt_msgs/local_setup.bash

./aimrt_main --cfg_file_path=./cfg/examples_hardware_camera_ros2_cfg.yaml

# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

add_subdirectory(hardware/sensor)

if(AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)
  add_subdirectory(hardware/sensor_ros2)
endif()

if(AIMRT_MUJOCO_SIM_BUILD_EXAMPLES)
  add_subdirectory(examples/inverted_pendulum)
endif()

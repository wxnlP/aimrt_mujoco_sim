# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

if(AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA)
  add_subdirectory(sensor)
endif()

if(AIMRT_MUJOCO_SIM_BUILD_EXAMPLES)
  add_subdirectory(examples/inverted_pendulum)
endif()

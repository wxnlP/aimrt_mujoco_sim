// Copyright (c) 2026, wxnlP
// All rights reserved.

#pragma once

#include <functional>
#include <utility>

#include "glfw_adapter.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::camera {

class CaptureAdapter : public mujoco::GlfwAdapter {
 public:
  explicit CaptureAdapter(std::function<void(mjrContext&)> on_frame)
      : on_frame_(std::move(on_frame)) {}

  void SwapBuffers() override {
    on_frame_(mjr_context());
    mujoco::GlfwAdapter::SwapBuffers();
  }

 private:
  std::function<void(mjrContext&)> on_frame_;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::camera

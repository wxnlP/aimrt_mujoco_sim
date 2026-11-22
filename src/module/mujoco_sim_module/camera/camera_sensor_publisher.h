// Copyright (c) 2026, wxnlP
// All rights reserved.

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "mujoco/mujoco.h"
#include "mujoco_sim_module/global.h"
#include "yaml-cpp/yaml.h"

#include "aimrt_module_cpp_interface/channel/channel_handle.h"
#include "aimrt_module_cpp_interface/executor/executor.h"

#include "simulate.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::camera {

struct CameraFrame {
  std::vector<uint8_t> color;
  std::vector<uint16_t> depth;
  uint64_t timestamp_ns = 0;
};

class CameraSensorPublisherBase {
 public:
  struct Options {
    std::string bind_camera;
    uint32_t width = 640;
    uint32_t height = 480;
  };

 public:
  CameraSensorPublisherBase() = default;
  virtual ~CameraSensorPublisherBase() = default;

  CameraSensorPublisherBase(const CameraSensorPublisherBase&) = delete;
  CameraSensorPublisherBase& operator=(const CameraSensorPublisherBase&) = delete;

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual std::string_view Type() const noexcept = 0;

  void Start() {}
  void Shutdown();

  void SetMj(mjModel* m, mjData* d);
  void SetSim(mujoco::Simulate* sim) { sim_ = sim; }
  void SetPublisherHandles(aimrt::channel::PublisherRef color_publisher,
                           aimrt::channel::PublisherRef depth_publisher,
                           aimrt::channel::PublisherRef info_publisher);
  void SetExecutor(aimrt::executor::ExecutorRef executor) { executor_ = executor; }
  void SetFreq(uint32_t freq) { channel_frq_ = freq; }

  void OnRenderFrame(mjrContext& con);

  uint32_t Width() const { return options_.width; }
  uint32_t Height() const { return options_.height; }

 protected:
  void InitializeBase(YAML::Node options_node);
  void ConvertDepth(const std::vector<float>& raw, std::vector<uint16_t>& depth) const;
  virtual void PublishFrame(std::shared_ptr<CameraFrame> frame) = 0;

 protected:
  Options options_;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  mujoco::Simulate* sim_ = nullptr;

  aimrt::channel::PublisherRef color_publisher_;
  aimrt::channel::PublisherRef depth_publisher_;
  aimrt::channel::PublisherRef info_publisher_;
  aimrt::executor::ExecutorRef executor_;

  uint32_t channel_frq_ = 30;
  std::chrono::nanoseconds period_{};
  std::chrono::system_clock::time_point next_frame_tp_{};

  int32_t camera_id_ = -1;
  mjvCamera cam_{};
  mjvOption opt_{};
  mjvScene scn_{};
  std::atomic_bool enabled_{true};

  std::array<double, 9> k_{};
  std::array<double, 9> r_{};
  std::array<double, 12> p_{};
};

class CameraSensorPublisher : public CameraSensorPublisherBase {
 public:
  CameraSensorPublisher() = default;
  ~CameraSensorPublisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "camera_sensor"; }

 protected:
  void PublishFrame(std::shared_ptr<CameraFrame> frame) override;
};

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
class CameraSensorRos2Publisher : public CameraSensorPublisherBase {
 public:
  CameraSensorRos2Publisher() = default;
  ~CameraSensorRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "camera_sensor_ros2"; }

 protected:
  void PublishFrame(std::shared_ptr<CameraFrame> frame) override;
};
#endif

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::camera

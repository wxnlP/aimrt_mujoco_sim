// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <memory>

#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "mujoco_sim_module/publisher/publisher_base.h"
#include "mujoco_sim_module/subscriber/subscriber_base.h"

#include "glfw_adapter.h"
#include "mujoco/mujoco.h"
#include "simulate.h"
#include "yaml-cpp/yaml.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  #include "mujoco_sim_module/camera/camera_sensor_publisher.h"
#endif

namespace aimrt_mujoco_sim::mujoco_sim_module {

class MujocoSimModule : public aimrt::ModuleBase {
 public:
  struct Options {
    std::string simulation_model_path;
    std::string sim_executor;
    std::string gui_executor;

    struct SubscriberOption {
      std::string topic;
      std::string type;
      YAML::Node options;
    };
    std::vector<SubscriberOption> subscriber_options;

    struct PublisherOption {
      std::string topic;
      uint32_t frequency;
      std::string executor;
      std::string type;
      YAML::Node options;
    };
    std::vector<PublisherOption> publisher_options;

    struct CameraOption {
      uint32_t frequency;
      std::string executor;
      std::string type;
      std::string color_topic;
      std::string depth_topic;
      std::string info_topic;
      YAML::Node options;
    };
    std::vector<CameraOption> camera_options;
  };

 public:
  MujocoSimModule() = default;
  ~MujocoSimModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "MujocoSimModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  void RegisterSubscriberGenFunc();
  void RegisterPublisherGenFunc();
#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  void RegisterCameraGenFunc();
#endif

  aimrt::co::Task<void> GuiLoop();
  aimrt::co::Task<void> SimLoop();

 private:
  aimrt::CoreRef core_;

  Options options_;

  aimrt::executor::ExecutorRef gui_executor_;
  aimrt::executor::ExecutorRef sim_executor_;

  std::shared_ptr<mujoco::Simulate> sim_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  aimrt::co::AsyncScope scope_;
  std::atomic_bool run_flag_ = true;

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  // key: color topic
  std::unordered_map<std::string, std::unique_ptr<camera::CameraSensorPublisherBase>> camera_map_;
#endif

  // key:type
  using SubscriberGenFunc = std::function<std::unique_ptr<subscriber::SubscriberBase>()>;
  std::unordered_map<std::string, SubscriberGenFunc> subscriber_gen_func_map_;

  using PublisherGenFunc = std::function<std::unique_ptr<publisher::PublisherBase>()>;
  std::unordered_map<std::string, PublisherGenFunc> publisher_gen_func_map_;

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  using CameraGenFunc = std::function<std::unique_ptr<camera::CameraSensorPublisherBase>()>;
  std::unordered_map<std::string, CameraGenFunc> camera_gen_func_map_;
#endif

  // key:topic
  std::unordered_map<std::string, std::unique_ptr<subscriber::SubscriberBase>> subscriber_map_;
  std::unordered_map<std::string, std::unique_ptr<publisher::PublisherBase>> publisher_map_;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module

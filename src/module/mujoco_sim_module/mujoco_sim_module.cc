// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/mujoco_sim_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/imu_sensor_publisher.h"
#include "mujoco_sim_module/publisher/joint_sensor_publisher.h"
#include "mujoco_sim_module/publisher/touch_sensor_publisher.h"
#include "mujoco_sim_module/subscriber/joint_actuator_subscriber.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  #include "mujoco_sim_module/camera/camera_sensor_publisher.h"
  #include "mujoco_sim_module/camera/capture_adapter.h"
#endif

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::MujocoSimModule::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::MujocoSimModule::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["simulation_model_path"] = rhs.simulation_model_path;
    node["auto_detach"] = rhs.auto_detach;
    node["auto_detach_eq_prefix"] = rhs.auto_detach_eq_prefix;
    node["auto_detach_gripper_joint"] = rhs.auto_detach_gripper_joint;
    node["auto_detach_gripper_body_prefix"] = rhs.auto_detach_gripper_body_prefix;
    node["auto_detach_close_thresh"] = rhs.auto_detach_close_thresh;
    node["auto_detach_hold_steps"] = rhs.auto_detach_hold_steps;
    node["sim_executor"] = rhs.sim_executor;
    node["gui_executor"] = rhs.gui_executor;

    node["subscriber_options"] = YAML::Node();
    for (const auto& subscriber_option : rhs.subscriber_options) {
      Node subscriber_option_node;
      subscriber_option_node["topic"] = subscriber_option.topic;
      subscriber_option_node["type"] = subscriber_option.type;
      subscriber_option_node["options"] = subscriber_option.options;
      node["subscriber_options"].push_back(subscriber_option_node);
    }

    node["publisher_options"] = YAML::Node();
    for (const auto& publisher_option : rhs.publisher_options) {
      Node publisher_option_node;
      publisher_option_node["topic"] = publisher_option.topic;
      publisher_option_node["frequency"] = publisher_option.frequency;
      publisher_option_node["executor"] = publisher_option.executor;
      publisher_option_node["type"] = publisher_option.type;
      publisher_option_node["options"] = publisher_option.options;
      node["publisher_options"].push_back(publisher_option_node);
    }

    node["camera_options"] = YAML::Node();
    for (const auto& camera_option : rhs.camera_options) {
      Node camera_option_node;
      camera_option_node["frequency"] = camera_option.frequency;
      camera_option_node["executor"] = camera_option.executor;
      camera_option_node["type"] = camera_option.type;
      camera_option_node["color_topic"] = camera_option.color_topic;
      camera_option_node["depth_topic"] = camera_option.depth_topic;
      camera_option_node["info_topic"] = camera_option.info_topic;
      camera_option_node["options"] = camera_option.options;
      node["camera_options"].push_back(camera_option_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.simulation_model_path = node["simulation_model_path"].as<std::string>();
    if (node["auto_detach"]) rhs.auto_detach = node["auto_detach"].as<bool>();
    if (node["auto_detach_eq_prefix"]) rhs.auto_detach_eq_prefix = node["auto_detach_eq_prefix"].as<std::string>();
    if (node["auto_detach_gripper_joint"]) rhs.auto_detach_gripper_joint = node["auto_detach_gripper_joint"].as<std::string>();
    if (node["auto_detach_gripper_body_prefix"]) rhs.auto_detach_gripper_body_prefix = node["auto_detach_gripper_body_prefix"].as<std::string>();
    if (node["auto_detach_close_thresh"]) rhs.auto_detach_close_thresh = node["auto_detach_close_thresh"].as<double>();
    if (node["auto_detach_hold_steps"]) rhs.auto_detach_hold_steps = node["auto_detach_hold_steps"].as<int>();
    rhs.sim_executor = node["sim_executor"].as<std::string>();
    rhs.gui_executor = node["gui_executor"].as<std::string>();

    if (node["subscriber_options"] && node["subscriber_options"].IsSequence()) {
      for (const auto& subscriber_option_node : node["subscriber_options"]) {
        auto subscriber_options = Options::SubscriberOption{
            .topic = subscriber_option_node["topic"].as<std::string>(),
            .type = subscriber_option_node["type"].as<std::string>()};

        if (subscriber_option_node["options"])
          subscriber_options.options = subscriber_option_node["options"];
        else
          subscriber_options.options = YAML::Node(YAML::NodeType::Null);

        rhs.subscriber_options.emplace_back(std::move(subscriber_options));
      }
    }

    if (node["publisher_options"] && node["publisher_options"].IsSequence()) {
      for (const auto& publisher_option_node : node["publisher_options"]) {
        auto publisher_options = Options::PublisherOption{
            .topic = publisher_option_node["topic"].as<std::string>(),
            .frequency = publisher_option_node["frequency"].as<uint32_t>(),
            .executor = publisher_option_node["executor"].as<std::string>(),
            .type = publisher_option_node["type"].as<std::string>()};

        if (publisher_option_node["options"])
          publisher_options.options = publisher_option_node["options"];
        else
          publisher_options.options = YAML::Node(YAML::NodeType::Null);

        rhs.publisher_options.emplace_back(std::move(publisher_options));
      }
    }

    if (node["camera_options"] && node["camera_options"].IsSequence()) {
      for (const auto& camera_option_node : node["camera_options"]) {
        auto camera_options = Options::CameraOption{
            .frequency = camera_option_node["frequency"].as<uint32_t>(),
            .executor = camera_option_node["executor"].as<std::string>(),
            .type = camera_option_node["type"].as<std::string>(),
            .color_topic = camera_option_node["color_topic"].as<std::string>(),
            .depth_topic = camera_option_node["depth_topic"].as<std::string>(),
            .info_topic = camera_option_node["info_topic"].as<std::string>()};

        if (camera_option_node["options"]) {
          camera_options.options = camera_option_node["options"];
        } else {
          camera_options.options = YAML::Node(YAML::NodeType::Null);
        }

        rhs.camera_options.emplace_back(std::move(camera_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module {

bool MujocoSimModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  SetLogger(core_.GetLogger());

  // Read cfg
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  auto yaml_node = YAML::LoadFile(std::string(file_path));
  options_ = yaml_node.as<Options>();

  // Get executor handle
  gui_executor_ = core_.GetExecutorManager().GetExecutor(options_.gui_executor);
  AIMRT_CHECK_ERROR_THROW(gui_executor_, "Get executor '{}' failed.", options_.gui_executor);

  sim_executor_ = core_.GetExecutorManager().GetExecutor(options_.sim_executor);
  AIMRT_CHECK_ERROR_THROW(sim_executor_, "Get executor '{}' failed.", options_.sim_executor);
  AIMRT_CHECK_ERROR_THROW(sim_executor_.SupportTimerSchedule(),
                          "Sim executor '{}' do not support time schedule.", options_.sim_executor);

  // load model
  m_ = mj_loadXML(options_.simulation_model_path.c_str(), nullptr, nullptr, 0);
  AIMRT_CHECK_ERROR_THROW(m_ != nullptr, "Load model failed, model path: '{}'.", options_.simulation_model_path);

  d_ = mj_makeData(m_);
  AIMRT_CHECK_ERROR_THROW(d_ != nullptr, "Make data failed.");

  // --- 夹住即脱落：建立索引（weld 名前缀 -> {eq_id, 果实 body}；夹爪 body 集合）---
  if (options_.auto_detach) {
    detach_grip_joint_ = mj_name2id(m_, mjOBJ_JOINT, options_.auto_detach_gripper_joint.c_str());
    for (int b = 1; b < m_->nbody; ++b) {
      const char* bn = mj_id2name(m_, mjOBJ_BODY, b);
      if (bn && std::string(bn).rfind(options_.auto_detach_gripper_body_prefix, 0) == 0)
        gripper_bodies_.insert(b);
    }
    const std::string& pfx = options_.auto_detach_eq_prefix;
    for (int e = 0; e < m_->neq; ++e) {
      const char* en = mj_id2name(m_, mjOBJ_EQUALITY, e);
      if (!en || std::string(en).rfind(pfx, 0) != 0) continue;
      if (m_->eq_type[e] != mjEQ_WELD) continue;
      detach_welds_.emplace_back(e, m_->eq_obj2id[e]);
      detach_hits_.push_back(0);
    }
    AIMRT_INFO("auto detach enabled: {} welds, {} gripper bodies, gripper joint id {}",
               detach_welds_.size(), gripper_bodies_.size(), detach_grip_joint_);
  }

  // register subscriber gen func
  RegisterSubscriberGenFunc();

  // create subscriber
  for (auto& item : options_.subscriber_options) {
    auto finditr = subscriber_gen_func_map_.find(item.type);
    AIMRT_CHECK_ERROR_THROW(finditr != subscriber_gen_func_map_.end(),
                            "Invalid type '{}' for subscriber.", item.type);

    auto ptr = finditr->second();

    ptr->SetMj(m_, d_);
    ptr->SetSubscriberHandle(core_.GetChannelHandle().GetSubscriber(item.topic));

    ptr->Initialize(item.options);

    subscriber_map_.emplace(item.topic, std::move(ptr));
  }

  // register publisher gen func
  RegisterPublisherGenFunc();

  // create publisher
  for (auto& item : options_.publisher_options) {
    auto finditr = publisher_gen_func_map_.find(item.type);
    AIMRT_CHECK_ERROR_THROW(finditr != publisher_gen_func_map_.end(),
                            "Invalid type '{}' for publisher.", item.type);

    auto executor = core_.GetExecutorManager().GetExecutor(item.executor);
    AIMRT_CHECK_ERROR_THROW(executor, "Can not get executor '{}' for publisher topic '{}'.",
                            item.executor, item.topic);

    auto ptr = finditr->second();

    ptr->SetMj(m_, d_);
    ptr->SetPublisherHandle(core_.GetChannelHandle().GetPublisher(item.topic));
    ptr->SetExecutor(executor);
    ptr->SetFreq(item.frequency);

    ptr->Initialize(item.options);

    publisher_map_.emplace(item.topic, std::move(ptr));
  }

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  // register camera gen func
  RegisterCameraGenFunc();

  // create camera
  for (auto& item : options_.camera_options) {
    auto finditr = camera_gen_func_map_.find(item.type);
    AIMRT_CHECK_ERROR_THROW(finditr != camera_gen_func_map_.end(),
                            "Invalid type '{}' for camera.", item.type);

    auto executor = core_.GetExecutorManager().GetExecutor(item.executor);
    AIMRT_CHECK_ERROR_THROW(executor, "Can not get executor '{}' for camera.", item.executor);

    auto ptr = finditr->second();

    ptr->SetMj(m_, d_);
    ptr->SetPublisherHandles(
        core_.GetChannelHandle().GetPublisher(item.color_topic),
        core_.GetChannelHandle().GetPublisher(item.depth_topic),
        core_.GetChannelHandle().GetPublisher(item.info_topic));
    ptr->SetExecutor(executor);
    ptr->SetFreq(item.frequency);

    ptr->Initialize(item.options);

    camera_map_.emplace(item.color_topic, std::move(ptr));
  }
#else
  AIMRT_CHECK_ERROR_THROW(options_.camera_options.empty(), "Camera support is not built.");
#endif

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool MujocoSimModule::Start() {
  AIMRT_INFO("Start succeeded.");

  scope_.spawn(aimrt::co::On(aimrt::co::InlineScheduler(), GuiLoop()));
  scope_.spawn(aimrt::co::On(aimrt::co::InlineScheduler(), SimLoop()));

  for (auto& itr : subscriber_map_) {
    itr.second->Start();
  }

  for (auto& itr : publisher_map_) {
    itr.second->Start();
  }

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  for (auto& itr : camera_map_) {
    itr.second->Start();
  }
#endif

  return true;
}

void MujocoSimModule::Shutdown() {
  run_flag_ = false;

  for (auto& itr : publisher_map_) {
    itr.second->Shutdown();
  }

  for (auto& itr : subscriber_map_) {
    itr.second->Shutdown();
  }

  if (sim_)
    sim_->exitrequest.store(1);

  aimrt::co::SyncWait(scope_.complete());

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  for (auto& itr : camera_map_) {
    itr.second->Shutdown();
  }
  camera_map_.clear();
#endif

  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);

  AIMRT_INFO("Shutdown succeeded.");
}

void MujocoSimModule::RegisterSubscriberGenFunc() {
  auto generator = [this]<typename T>(std::string_view name) {
    subscriber_gen_func_map_.emplace(
        name,
        []() -> std::unique_ptr<subscriber::SubscriberBase> {
          return std::make_unique<T>();
        });
  };

  generator.template operator()<subscriber::JointActuatorSubscriber>("joint_actuator");

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  generator.template operator()<subscriber::JointActuatorRos2Subscriber>("joint_actuator_ros2");
#endif
}

void MujocoSimModule::RegisterPublisherGenFunc() {
  auto generator = [this]<typename T>(std::string_view name) {
    publisher_gen_func_map_.emplace(
        name,
        []() -> std::unique_ptr<publisher::PublisherBase> {
          return std::make_unique<T>();
        });
  };

  generator.template operator()<publisher::JointSensorPublisher>("joint_sensor");
  generator.template operator()<publisher::ImuSensorPublisher>("imu_sensor");
  generator.template operator()<publisher::TouchSensorPublisher>("touch_sensor");
#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  generator.template operator()<publisher::ImuSensorRos2Publisher>("imu_sensor_ros2");
  generator.template operator()<publisher::TouchSensorRos2Publisher>("touch_sensor_ros2");
  generator.template operator()<publisher::JointSensorRos2Publisher>("joint_sensor_ros2");
#endif
}

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
void MujocoSimModule::RegisterCameraGenFunc() {
  auto generator = [this]<typename T>(std::string_view name) {
    camera_gen_func_map_.emplace(
        name,
        []() -> std::unique_ptr<camera::CameraSensorPublisherBase> {
          return std::make_unique<T>();
        });
  };

  generator.template operator()<camera::CameraSensorPublisher>("camera_sensor");
#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  generator.template operator()<camera::CameraSensorRos2Publisher>("camera_sensor_ros2");
#endif
}
#endif

aimrt::co::Task<void> MujocoSimModule::GuiLoop() {
  auto gui_scheduler = aimrt::co::AimRTScheduler(gui_executor_);
  co_await aimrt::co::Schedule(gui_scheduler);

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  std::unique_ptr<mujoco::PlatformUIAdapter> adapter;
#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  if (!camera_map_.empty()) {
    adapter = std::make_unique<camera::CaptureAdapter>([this](mjrContext& con) {
      for (auto& itr : camera_map_) {
        itr.second->OnRenderFrame(con);
      }
    });
  }
#endif
  if (!adapter) {
    adapter = std::make_unique<mujoco::GlfwAdapter>();
  }

  sim_ = std::make_shared<mujoco::Simulate>(std::move(adapter), &cam, &opt, &pert, false);

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_CAMERA
  for (auto& itr : camera_map_) {
    itr.second->SetSim(sim_.get());
  }
#endif

  sim_->RenderLoop();

  AIMRT_INFO("GuiLoop exit.");

  co_return;
}

aimrt::co::Task<void> MujocoSimModule::SimLoop() {
  auto sim_scheduler = aimrt::co::AimRTScheduler(sim_executor_);

  while (!sim_)
    co_await aimrt::co::ScheduleAfter(sim_scheduler, std::chrono::milliseconds(500));
  co_await aimrt::co::ScheduleAfter(sim_scheduler, std::chrono::milliseconds(100));

  sim_->Load(m_, d_, options_.simulation_model_path.c_str());

  // loop
  auto next_sche_tp = sim_executor_.Now();
  std::chrono::nanoseconds dt(static_cast<uint64_t>(m_->opt.timestep * 1e9));

  while (!sim_->exitrequest.load()) {
    next_sche_tp += dt;

    co_await aimrt::co::ScheduleAt(sim_scheduler, next_sche_tp);

    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

      // apply ctrl data
      for (auto& itr : subscriber_map_) {
        itr.second->ApplyCtrlData();
      }

      // read sensor data
      for (auto& itr : publisher_map_) {
        itr.second->PublishSensorData();
      }

      // 夹住即脱落（本地临时补丁）
      if (options_.auto_detach) ApplyAutoDetach();

      // step
      if (sim_->run) {
        mj_step(m_, d_);
        sim_->AddToHistory();
      } else {
        mj_forward(m_, d_);
      }
    }
  }

  AIMRT_INFO("SimLoop exit.");

  co_return;
}

void MujocoSimModule::ApplyAutoDetach() {
  if (detach_grip_joint_ < 0 || detach_welds_.empty()) return;
  // 夹爪是否已闭合到阈值（0.0 = 夹紧，越大越开）
  double grip = d_->qpos[m_->jnt_qposadr[detach_grip_joint_]];
  if (grip > options_.auto_detach_close_thresh) return;

  for (size_t i = 0; i < detach_welds_.size(); ++i) {
    auto& [eq, fbody] = detach_welds_[i];
    if (d_->eq_active[eq] == 0) continue;
    bool hit = false;
    for (int c = 0; c < d_->ncon && !hit; ++c) {
      const mjContact& con = d_->contact[c];
      int b0 = m_->geom_bodyid[con.geom[0]];
      int b1 = m_->geom_bodyid[con.geom[1]];
      hit = (b0 == fbody && gripper_bodies_.count(b1)) ||
            (b1 == fbody && gripper_bodies_.count(b0));
    }
    if (!hit) { detach_hits_[i] = 0; continue; }
    if (++detach_hits_[i] < options_.auto_detach_hold_steps) continue;
    d_->eq_active[eq] = 0;
    detach_hits_[i] = 0;
    const char* en = mj_id2name(m_, mjOBJ_EQUALITY, eq);
    const char* fn = mj_id2name(m_, mjOBJ_BODY, fbody);
    AIMRT_INFO("auto detach: '{}' released (fruit body '{}', grip {:.4f})",
               en ? en : "?", fn ? fn : "?", grip);
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module

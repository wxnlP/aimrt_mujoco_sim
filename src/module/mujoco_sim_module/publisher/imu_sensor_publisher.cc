// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/imu_sensor_publisher.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher::ImuSensorPublisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher::ImuSensorPublisher::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["bind_site"] = rhs.bind_site;
    node["bind_framequat"] = rhs.bind_framequat;
    node["bind_gyro"] = rhs.bind_gyro;
    node["bind_accelerometer"] = rhs.bind_accelerometer;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.bind_site = node["bind_site"].as<std::string>();

    if (node["bind_framequat"]) {
      rhs.bind_framequat = node["bind_framequat"].as<std::string>();
    }
    if (node["bind_gyro"]) {
      rhs.bind_gyro = node["bind_gyro"].as<std::string>();
    }
    if (node["bind_accelerometer"]) {
      rhs.bind_accelerometer = node["bind_accelerometer"].as<std::string>();
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

void ImuSensorPublisher::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);

  RegisterSensorAddr();

  options_node = options_;

  bool ret = aimrt::channel::RegisterPublishType<aimrt::protocols::sensor::ImuState>(publisher_);

  AIMRT_CHECK_ERROR_THROW(ret, "Register publish type failed.");
}

void ImuSensorPublisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;

  if (counter_++ < avg_interval_) return;

  auto state_array = std::make_unique<SensorStateGroup>();

  CopySensorData(sensor_addr_group_.framequat_addr, state_array->orientation, 4);
  CopySensorData(sensor_addr_group_.gyro_addr, state_array->angular_velocity, 3);
  CopySensorData(sensor_addr_group_.accelerometer_addr, state_array->linear_acceleration, 3);

  executor_.Execute([this, state_array = std::move(state_array)]() {
    aimrt::protocols::sensor::ImuState state;

    state.mutable_header()->set_time_stamp(std::chrono::system_clock::now().time_since_epoch().count());

    auto* orientation = state.mutable_orientation();
    orientation->set_x(state_array->orientation.x);
    orientation->set_y(state_array->orientation.y);
    orientation->set_z(state_array->orientation.z);
    orientation->set_w(state_array->orientation.w);

    state.mutable_orientation_covariance()->Resize(9, 0.0);

    auto* angular_velocity = state.mutable_angular_velocity();
    angular_velocity->set_x(state_array->angular_velocity.x);
    angular_velocity->set_y(state_array->angular_velocity.y);
    angular_velocity->set_z(state_array->angular_velocity.z);

    state.mutable_angular_velocity_covariance()->Resize(9, 0.0);

    auto* linear_acceleration = state.mutable_linear_acceleration();
    linear_acceleration->set_x(state_array->linear_acceleration.x);
    linear_acceleration->set_y(state_array->linear_acceleration.y);
    linear_acceleration->set_z(state_array->linear_acceleration.z);

    state.mutable_linear_acceleration_covariance()->Resize(9, 0.0);

    aimrt::channel::Publish(publisher_, state);
  });

  avg_interval_ += avg_interval_base_;

  // avoid overflow
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

void ImuSensorPublisher::RegisterSensorAddr() {
  sensor_addr_group_.framequat_addr = GetSensorAddr(m_, options_.bind_framequat),
  sensor_addr_group_.gyro_addr = GetSensorAddr(m_, options_.bind_gyro),
  sensor_addr_group_.accelerometer_addr = GetSensorAddr(m_, options_.bind_accelerometer);
}

void ImuSensorPublisher::CopySensorData(int addr, auto& dest, size_t n) {
  if (addr >= 0) {
    std::memcpy(&dest, &d_->sensordata[addr], n * sizeof(double));
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/imu_sensor_publisher.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/utils.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher::ImuSensorPublisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher::ImuSensorPublisher::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["imus"] = YAML::Node();
    for (const auto& imu : rhs.imus) {
      Node imu_node;
      imu_node["name"] = imu.name;
      imu_node["bind_site"] = imu.bind_site;
      imu_node["bind_framequat"] = imu.bind_framequat;
      imu_node["bind_gyro"] = imu.bind_gyro;
      imu_node["bind_accelerometer"] = imu.bind_accelerometer;
      node["imus"].push_back(imu_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["imus"] && node["imus"].IsSequence()) {
      for (const auto& imu_node : node["imus"]) {
        auto imu_node_options = Options::Imu{
            .name = imu_node["name"].as<std::string>(),
            .bind_site = imu_node["bind_site"].as<std::string>(),
            .bind_framequat = imu_node["bind_framequat"].as<std::string>(),
            .bind_gyro = imu_node["bind_gyro"].as<std::string>(),
            .bind_accelerometer = imu_node["bind_accelerometer"].as<std::string>()};

        rhs.imus.emplace_back(std::move(imu_node_options));
      }
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

void ImuSensorPublisher::Start() {
}

void ImuSensorPublisher::Shutdown() {
}

void ImuSensorPublisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;

  if (counter_++ < avg_interval_) return;

  std::unique_ptr<SensorStateGroup[]> state_array(new SensorStateGroup[imu_num_]);

  for (size_t i = 0; i < imu_num_; i++) {
    const auto& addr = sensor_addr_vec_[i];

    if (addr.framequat_addr >= 0) {
      state_array[i].orientation = {
          .w = d_->sensordata[addr.framequat_addr],
          .x = d_->sensordata[addr.framequat_addr + 1],
          .y = d_->sensordata[addr.framequat_addr + 2],
          .z = d_->sensordata[addr.framequat_addr + 3]};
    }

    if (addr.gyro_addr >= 0) {
      state_array[i].angular_velocity = {
          .x = d_->sensordata[addr.gyro_addr],
          .y = d_->sensordata[addr.gyro_addr + 1],
          .z = d_->sensordata[addr.gyro_addr + 2]};
    }

    if (addr.accelerometer_addr >= 0) {
      state_array[i].linear_acceleration = {
          .x = d_->sensordata[addr.accelerometer_addr],
          .y = d_->sensordata[addr.accelerometer_addr + 1],
          .z = d_->sensordata[addr.accelerometer_addr + 2]};
    }
  }

  executor_.Execute([this, state_array = std::move(state_array)]() {
    aimrt::protocols::sensor::ImuState state;
    for (int i = 0; i < imu_num_; ++i) {
      auto* data = state.add_data();

      data->set_name(name_vec_[i]);

      auto* orientation = data->mutable_orientation();
      orientation->set_w(state_array[i].orientation.w);
      orientation->set_x(state_array[i].orientation.x);
      orientation->set_y(state_array[i].orientation.y);
      orientation->set_z(state_array[i].orientation.z);

      auto* angular_velocity = data->mutable_angular_velocity();
      angular_velocity->set_x(state_array[i].angular_velocity.x);
      angular_velocity->set_y(state_array[i].angular_velocity.y);
      angular_velocity->set_z(state_array[i].angular_velocity.z);

      auto* linear_acceleration = data->mutable_linear_acceleration();
      linear_acceleration->set_x(state_array[i].linear_acceleration.x);
      linear_acceleration->set_y(state_array[i].linear_acceleration.y);
      linear_acceleration->set_z(state_array[i].linear_acceleration.z);
    }

    aimrt::channel::Publish(publisher_, state);
  });

  avg_interval_ += avg_interval_base_;

  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

void ImuSensorPublisher::RegisterSensorAddr() {
  for (const auto& imu : options_.imus) {
    sensor_addr_vec_.emplace_back(SensorAddrGroup{
        .framequat_addr = GetSensorAddr(m_, imu.bind_framequat),
        .gyro_addr = GetSensorAddr(m_, imu.bind_gyro),
        .accelerometer_addr = GetSensorAddr(m_, imu.bind_accelerometer)});

    name_vec_.emplace_back(imu.name);
  }
  imu_num_ = sensor_addr_vec_.size();
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
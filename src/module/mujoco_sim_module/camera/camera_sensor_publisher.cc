// Copyright (c) 2026, wxnlP
// All rights reserved.

#include "mujoco_sim_module/camera/camera_sensor_publisher.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <mutex>

#include "simulate.h"

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "camera.pb.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include <sensor_msgs/msg/camera_info.hpp>
  #include <sensor_msgs/msg/image.hpp>
  #include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#endif

namespace camera_pb = aimrt_mujoco_sim::protocols::sensor;

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::camera::CameraSensorPublisherBase::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::camera::CameraSensorPublisherBase::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["bind_camera"] = rhs.bind_camera;
    node["width"] = rhs.width;
    node["height"] = rhs.height;

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) {
      return false;
    }

    rhs.bind_camera = node["bind_camera"].as<std::string>();

    if (node["width"]) {
      rhs.width = node["width"].as<uint32_t>();
    }
    if (node["height"]) {
      rhs.height = node["height"].as<uint32_t>();
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::camera {

namespace {

template <typename T>
void FlipRows(std::vector<T>& data, uint32_t width, uint32_t height, uint32_t channels) {
  for (uint32_t row = 0; row < height / 2; ++row) {
    auto top = data.begin() + static_cast<std::ptrdiff_t>(row * width * channels);
    auto bottom = data.begin() + static_cast<std::ptrdiff_t>((height - 1 - row) * width * channels);
    std::swap_ranges(top, top + width * channels, bottom);
  }
}

}  // namespace

void CameraSensorPublisherBase::Shutdown() {
  enabled_.store(false);
  mjv_freeScene(&scn_);
}

void CameraSensorPublisherBase::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void CameraSensorPublisherBase::SetPublisherHandles(aimrt::channel::PublisherRef color_publisher,
                                                    aimrt::channel::PublisherRef depth_publisher,
                                                    aimrt::channel::PublisherRef info_publisher) {
  color_publisher_ = color_publisher;
  depth_publisher_ = depth_publisher;
  info_publisher_ = info_publisher;
}

void CameraSensorPublisherBase::InitializeBase(YAML::Node options_node) {
  if (options_node && !options_node.IsNull()) {
    options_ = options_node.as<Options>();
  }

  AIMRT_CHECK_ERROR_THROW((options_.width > 0 && options_.height > 0), "Invalid camera resolution {}x{}.",
                          options_.width, options_.height);
  AIMRT_CHECK_ERROR_THROW(channel_frq_ > 0, "Invalid camera frequency {}.", channel_frq_);
  period_ = std::chrono::nanoseconds(static_cast<uint64_t>(1e9 / channel_frq_));

  camera_id_ = mj_name2id(m_, mjOBJ_CAMERA, options_.bind_camera.c_str());
  AIMRT_CHECK_ERROR_THROW(camera_id_ >= 0, "Invalid camera name: {}.", options_.bind_camera);

  mjv_defaultCamera(&cam_);
  cam_.type = mjCAMERA_FIXED;
  cam_.fixedcamid = camera_id_;

  mjv_defaultOption(&opt_);

  mjv_defaultScene(&scn_);
  mjv_makeScene(m_, &scn_, mujoco::Simulate::kMaxGeom);

  const double fovy = m_->cam_fovy[camera_id_] * mjPI / 180.0;
  const double fy = 0.5 * options_.height / std::tan(fovy / 2.0);
  const double fx = fy;
  const double cx = 0.5 * options_.width;
  const double cy = 0.5 * options_.height;

  k_ = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  r_ = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  p_ = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
}

void CameraSensorPublisherBase::OnRenderFrame(mjrContext& con) {
  if (!enabled_.load() || !sim_ || !sim_->m_) {
    return;
  }

  const auto now = std::chrono::system_clock::now();
  if (now < next_frame_tp_) {
    return;
  }
  next_frame_tp_ = now + period_;

  if (con.offWidth < static_cast<int>(options_.width) || con.offHeight < static_cast<int>(options_.height)) {
    mjr_resizeOffscreen(static_cast<int>(options_.width), static_cast<int>(options_.height), &con);
  }

  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    mjv_updateScene(sim_->m_, sim_->d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
  }

  auto frame = std::make_shared<CameraFrame>();
  frame->color.resize(static_cast<size_t>(options_.width) * options_.height * 3);
  frame->depth.resize(static_cast<size_t>(options_.width) * options_.height);
  frame->timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

  std::vector<float> depth_raw(static_cast<size_t>(options_.width) * options_.height);
  const mjrRect viewport = {0, 0, static_cast<int>(options_.width), static_cast<int>(options_.height)};
  const int previous = con.currentBuffer;
  mjr_setBuffer(mjFB_OFFSCREEN, &con);
  mjr_render(viewport, &scn_, &con);
  mjr_readPixels(frame->color.data(), depth_raw.data(), viewport, &con);
  mjr_setBuffer(previous, &con);

  FlipRows(frame->color, options_.width, options_.height, 3);
  FlipRows(depth_raw, options_.width, options_.height, 1);
  ConvertDepth(depth_raw, frame->depth);

  PublishFrame(std::move(frame));
}

void CameraSensorPublisherBase::ConvertDepth(const std::vector<float>& raw, std::vector<uint16_t>& depth) const {
  const double extent = m_->stat.extent;
  const double znear = m_->vis.map.znear * extent;
  const double zfar = m_->vis.map.zfar * extent;
  const double scale = znear * zfar;
  const double range = zfar - znear;

  for (size_t i = 0; i < raw.size(); ++i) {
    const double mm = scale / (zfar - raw[i] * range) * 1000.0;
    depth[i] = mm > 65535.0 ? 65535 : static_cast<uint16_t>(mm);
  }
}

void CameraSensorPublisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<camera_pb::Image>(color_publisher_),
                          "Register publish type failed.");
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<camera_pb::Image>(depth_publisher_),
                          "Register publish type failed.");
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<camera_pb::CameraInfo>(info_publisher_),
                          "Register publish type failed.");
}

void CameraSensorPublisher::PublishFrame(std::shared_ptr<CameraFrame> frame) {
  executor_.Execute([this, frame = std::move(frame)]() {
    camera_pb::Image color_image;
    auto* color_header = color_image.mutable_header();
    color_header->set_time_stamp(frame->timestamp_ns);
    color_header->set_frame_id(options_.bind_camera);
    color_image.set_height(options_.height);
    color_image.set_width(options_.width);
    color_image.set_encoding("rgb8");
    color_image.set_is_bigendian(0);
    color_image.set_step(options_.width * 3);
    color_image.set_data(frame->color.data(), frame->color.size());
    aimrt::channel::Publish(color_publisher_, color_image);

    camera_pb::Image depth_image;
    auto* depth_header = depth_image.mutable_header();
    depth_header->set_time_stamp(frame->timestamp_ns);
    depth_header->set_frame_id(options_.bind_camera);
    depth_image.set_height(options_.height);
    depth_image.set_width(options_.width);
    depth_image.set_encoding("16UC1");
    depth_image.set_is_bigendian(0);
    depth_image.set_step(static_cast<uint32_t>(options_.width * sizeof(uint16_t)));
    depth_image.set_data(frame->depth.data(), frame->depth.size() * sizeof(uint16_t));
    aimrt::channel::Publish(depth_publisher_, depth_image);

    camera_pb::CameraInfo camera_info;
    auto* info_header = camera_info.mutable_header();
    info_header->set_time_stamp(frame->timestamp_ns);
    info_header->set_frame_id(options_.bind_camera);
    camera_info.set_height(options_.height);
    camera_info.set_width(options_.width);
    camera_info.set_distortion_model("plumb_bob");
    for (int i = 0; i < 5; ++i) {
      camera_info.add_d(0.0);
    }
    for (double value : k_) {
      camera_info.add_k(value);
    }
    for (double value : r_) {
      camera_info.add_r(value);
    }
    for (double value : p_) {
      camera_info.add_p(value);
    }
    camera_info.set_binning_x(0);
    camera_info.set_binning_y(0);
    aimrt::channel::Publish(info_publisher_, camera_info);
  });
}

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
void CameraSensorRos2Publisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<sensor_msgs::msg::Image>(color_publisher_),
                          "Register publish type failed.");
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<sensor_msgs::msg::Image>(depth_publisher_),
                          "Register publish type failed.");
  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<sensor_msgs::msg::CameraInfo>(info_publisher_),
                          "Register publish type failed.");
}

void CameraSensorRos2Publisher::PublishFrame(std::shared_ptr<CameraFrame> frame) {
  executor_.Execute([this, frame = std::move(frame)]() {
    sensor_msgs::msg::Image color_image;
    color_image.header.stamp.sec = static_cast<int32_t>(frame->timestamp_ns / 1000000000);
    color_image.header.stamp.nanosec = static_cast<uint32_t>(frame->timestamp_ns % 1000000000);
    color_image.header.frame_id = options_.bind_camera;
    color_image.height = options_.height;
    color_image.width = options_.width;
    color_image.encoding = "rgb8";
    color_image.is_bigendian = 0;
    color_image.step = static_cast<uint32_t>(options_.width * 3);
    color_image.data = frame->color;
    aimrt::channel::Publish(color_publisher_, color_image);

    sensor_msgs::msg::Image depth_image;
    depth_image.header = color_image.header;
    depth_image.height = options_.height;
    depth_image.width = options_.width;
    depth_image.encoding = "16UC1";
    depth_image.is_bigendian = 0;
    depth_image.step = static_cast<uint32_t>(options_.width * sizeof(uint16_t));
    depth_image.data.resize(frame->depth.size() * sizeof(uint16_t));
    std::memcpy(depth_image.data.data(), frame->depth.data(), depth_image.data.size());
    aimrt::channel::Publish(depth_publisher_, depth_image);

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = color_image.header;
    camera_info.height = options_.height;
    camera_info.width = options_.width;
    camera_info.distortion_model = "plumb_bob";
    camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info.k = k_;
    camera_info.r = r_;
    camera_info.p = p_;
    camera_info.binning_x = 0;
    camera_info.binning_y = 0;
    aimrt::channel::Publish(info_publisher_, camera_info);
  });
}
#endif

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::camera

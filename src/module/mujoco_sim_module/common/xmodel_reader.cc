// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/common/xmodel_reader.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::common {
std::optional<std::string> GetJointvelNameByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t jointId = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (jointId < 0) return std::nullopt;

  for (int32_t i = 0; i < m->nsensor; i++) {
    if (m->sensor_type[i] == mjSENS_JOINTVEL && m->sensor_objid[i] == jointId) {
      return mj_id2name(m, mjOBJ_SENSOR, i);
    }
  }
  return std::nullopt;
}

std::optional<std::string> GetJointposNameByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t jointId = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (jointId < 0) return std::nullopt;

  for (int32_t i = 0; i < m->nsensor; i++) {
    if (m->sensor_type[i] == mjSENS_JOINTPOS && m->sensor_objid[i] == jointId) {
      return mj_id2name(m, mjOBJ_SENSOR, i);
    }
  }
  return std::nullopt;
}

std::optional<std::string> GetJointactfrcNameByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t jointId = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (jointId < 0) return std::nullopt;

  for (int32_t i = 0; i < m->nsensor; i++) {
    if (m->sensor_type[i] == mjSENS_JOINTACTFRC && m->sensor_objid[i] == jointId) {
      return mj_id2name(m, mjOBJ_SENSOR, i);
    }
  }
  return std::nullopt;
}

std::optional<int32_t> GetJointvelIdByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t jointId = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (jointId < 0) return std::nullopt;

  for (int32_t i = 0; i < m->nsensor; i++) {
    if (m->sensor_type[i] == mjSENS_JOINTVEL && m->sensor_objid[i] == jointId) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<int32_t> GetJointposIdByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t jointId = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (jointId < 0) return std::nullopt;

  for (int32_t i = 0; i < m->nsensor; i++) {
    if (m->sensor_type[i] == mjSENS_JOINTPOS && m->sensor_objid[i] == jointId) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<int32_t> GetJointactfrcIdByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t jointId = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (jointId < 0) return std::nullopt;

  for (int32_t i = 0; i < m->nsensor; i++) {
    if (m->sensor_type[i] == mjSENS_JOINTACTFRC && m->sensor_objid[i] == jointId) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<int32_t> GetJointActIdByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t jointId = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (jointId < 0) return std::nullopt;

  for (int i = 0; i < m->nu; i++) {
    if (m->actuator_trntype[i] == mjTRN_JOINT && m->actuator_trnid[static_cast<ptrdiff_t>(i * 2)] == jointId) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<std::string> GetJointActNameByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t actuatorId = GetJointActIdByJointName(m, joint_name.data()).value_or(-1);
  if (actuatorId < 0) return std::nullopt;

  return mj_id2name(m, mjOBJ_ACTUATOR, actuatorId);
}

std::optional<std::string> GetJointActTypeByJointName(const mjModel* m, std::string_view joint_name) {
  if (!m || !joint_name.data()) return std::nullopt;
  for (int32_t i = 0; i < m->nu; i++) {
    int32_t targetId = m->actuator_trnid[static_cast<ptrdiff_t>(i * 2)];
    if (m->actuator_trntype[i] == mjTRN_JOINT) {
      const char* targetName = mj_id2name(m, mjOBJ_JOINT, targetId);
      if (targetName && strcmp(targetName, joint_name.data()) == 0) {
        // position
        if (m->actuator_gaintype[i] == mjGAIN_FIXED &&
            m->actuator_biastype[i] == mjBIAS_AFFINE &&
            (m->actuator_dyntype[i] == mjDYN_NONE ||
             m->actuator_dyntype[i] == mjDYN_FILTEREXACT)) {
          if (m->actuator_biasprm[i * mjNBIAS + 1] == -m->actuator_gainprm[static_cast<ptrdiff_t>(i * mjNGAIN)]) {
            return "position";
          }
        }
        // velocity
        if (m->actuator_gaintype[i] == mjGAIN_FIXED &&
            m->actuator_biastype[i] == mjBIAS_AFFINE &&
            m->actuator_dyntype[i] == mjDYN_NONE) {
          if (m->actuator_biasprm[i * mjNBIAS + 2] == -m->actuator_gainprm[static_cast<ptrdiff_t>(i * mjNGAIN)]) {
            return "velocity";
          }
        }
        // motor
        if (m->actuator_dyntype[i] == mjDYN_NONE &&
            m->actuator_gaintype[i] == mjGAIN_FIXED &&
            m->actuator_biastype[i] == mjBIAS_NONE) {
          if (m->actuator_gainprm[i * mjNGAIN] == 1) {
            return "motor";
          }
        }
        // other (todo:(hj) adapt to other types of actuators)
        return std::nullopt;
      }
    }
  }
  // cannot find the joint name in the actuators
  return std::nullopt;
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::common
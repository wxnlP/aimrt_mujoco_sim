// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <string>
#include "mujoco/mujoco.h"
#include "mujoco_sim_module/global.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

inline void CheckFrequency(const uint32_t channel_frq,
                           double& avg_interval_base) {
  constexpr static uint32_t kMaxSimFreq = 1000;
  constexpr static double kErrorRate = 0.05;

  AIMRT_CHECK_ERROR_THROW((channel_frq <= kMaxSimFreq && channel_frq > 0),
                          "Invalid channel frequency {}, exceeds the maximum frequency ({} Hz)",
                          channel_frq, kMaxSimFreq);
  avg_interval_base = static_cast<double>(kMaxSimFreq) / static_cast<double>(channel_frq);

  if (kMaxSimFreq % channel_frq == 0) return;

  const uint32_t lower_interval = kMaxSimFreq / channel_frq;
  const uint32_t upper_interval = lower_interval + 1;

  const double lower_error = std::abs(lower_interval - avg_interval_base) / avg_interval_base;
  const double upper_error = std::abs(upper_interval - avg_interval_base) / avg_interval_base;

  AIMRT_CHECK_ERROR_THROW((lower_error <= kErrorRate && upper_error <= kErrorRate),
                          "Invalid channel frequency {}, which cauess the frequency error is more than {} ",
                          channel_frq, kErrorRate);
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
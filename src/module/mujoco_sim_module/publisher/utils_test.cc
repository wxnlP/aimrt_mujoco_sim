// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/utils.h"
#include <gtest/gtest.h>

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

TEST(CheckFrequencyTest, BoundaryValidation) {
  double avg_interval = 0.0;

  EXPECT_ANY_THROW(CheckFrequency(0, avg_interval));
  EXPECT_ANY_THROW(CheckFrequency(1001, avg_interval));
}

TEST(CheckFrequencyTest, ExactFrequency) {
  const std::vector<std::pair<uint32_t, double>> cases = {
      {1, 1000.0}, {100, 10.0}, {500, 2.0}, {1000, 1.0}};

  for (const auto& [freq, expected] : cases) {
    double avg_interval = 0.0;
    EXPECT_NO_THROW(CheckFrequency(freq, avg_interval));
    EXPECT_DOUBLE_EQ(avg_interval, expected);
  }
}

TEST(CheckFrequencyTest, ApproximateFrequency) {
  // valid cases
  {
    const std::vector<uint32_t> valid_cases = {10, 48};
    for (auto freq : valid_cases) {
      double avg_interval = 0.0;
      EXPECT_NO_THROW(CheckFrequency(freq, avg_interval));
      EXPECT_NEAR(avg_interval, 1000.0 / freq, 1e-9);
    }
  }

  // invalid cases
  {
    const std::vector<uint32_t> invalid_cases = {999};
    for (auto freq : invalid_cases) {
      double avg_interval = 0.0;
      EXPECT_ANY_THROW(CheckFrequency(freq, avg_interval));
    }
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher

// Copyright 2026 physical_ai_runtime
// SPDX-License-Identifier: Apache-2.0
//
// Offline tests: write-path command filter (END_EFFECTOR_CONVENTIONS §6).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "pika_gripper_hardware_interface/pika_protocol.hpp"

namespace protocol = pika_gripper_hardware_interface::protocol;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

protocol::CommandFilterConfig default_cfg()
{
  protocol::CommandFilterConfig cfg;
  cfg.min_width = 0.0;
  cfg.max_width = 0.045;
  cfg.max_speed = 0.1;   // m/s
  cfg.deadband = 5e-4;   // m
  return cfg;
}

}  // namespace

TEST(CommandFilter, RejectsNonFinite)
{
  const auto cfg = default_cfg();
  EXPECT_FALSE(protocol::filter_command(cfg, kNaN, 0.02, 0.02, 0.01).has_value());
  EXPECT_FALSE(protocol::filter_command(
      cfg, std::numeric_limits<double>::infinity(), 0.02, 0.02, 0.01).has_value());
}

TEST(CommandFilter, ClampsToWidthLimits)
{
  const auto cfg = default_cfg();
  const auto high = protocol::filter_command(cfg, 0.5, 0.02, 0.02, 10.0);
  ASSERT_TRUE(high.has_value());
  EXPECT_DOUBLE_EQ(*high, cfg.max_width);

  const auto low = protocol::filter_command(cfg, -0.5, 0.02, 0.02, 10.0);
  ASSERT_TRUE(low.has_value());
  EXPECT_DOUBLE_EQ(*low, cfg.min_width);
}

TEST(CommandFilter, FirstWriteStartsFromMeasured)
{
  const auto cfg = default_cfg();
  const auto out = protocol::filter_command(cfg, 0.045, kNaN, 0.02, 0.01);
  ASSERT_TRUE(out.has_value());
  EXPECT_NEAR(*out, 0.021, 1e-12);
}

TEST(CommandFilter, PerCycleStepLimit)
{
  const auto cfg = default_cfg();
  const auto up = protocol::filter_command(cfg, 0.045, 0.02, 0.02, 0.01);
  ASSERT_TRUE(up.has_value());
  EXPECT_NEAR(*up, 0.021, 1e-12);

  const auto down = protocol::filter_command(cfg, 0.0, 0.02, 0.02, 0.01);
  ASSERT_TRUE(down.has_value());
  EXPECT_NEAR(*down, 0.019, 1e-12);
}

TEST(CommandFilter, DeadbandSkipsUnchangedTarget)
{
  const auto cfg = default_cfg();
  EXPECT_FALSE(protocol::filter_command(cfg, 0.0202, 0.020, 0.020, 0.01).has_value());
  EXPECT_TRUE(protocol::filter_command(cfg, 0.021, 0.020, 0.020, 0.01).has_value());
}

TEST(CommandFilter, FirstWriteClampsMeasuredBaseline)
{
  const auto cfg = default_cfg();
  const auto from_low = protocol::filter_command(cfg, 0.045, kNaN, -0.01, 0.01);
  ASSERT_TRUE(from_low.has_value());
  EXPECT_NEAR(*from_low, 0.001, 1e-12);

  const auto from_high = protocol::filter_command(cfg, 0.0, kNaN, 0.08, 0.01);
  ASSERT_TRUE(from_high.has_value());
  EXPECT_NEAR(*from_high, 0.044, 1e-12);
}

TEST(ClampFingerTravel, ClampsAndMapsNonFiniteToMin)
{
  const auto cfg = default_cfg();
  EXPECT_DOUBLE_EQ(protocol::clamp_finger_travel(cfg, -0.01), 0.0);
  EXPECT_DOUBLE_EQ(protocol::clamp_finger_travel(cfg, 0.08), 0.045);
  EXPECT_DOUBLE_EQ(protocol::clamp_finger_travel(cfg, 0.02), 0.02);
  EXPECT_DOUBLE_EQ(protocol::clamp_finger_travel(cfg, kNaN), cfg.min_width);
}

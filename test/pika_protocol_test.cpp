// Copyright 2026 physical_ai_runtime
// SPDX-License-Identifier: Apache-2.0
//
// Offline tests: command encoding, frame assembly/parsing, kinematics.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <string>

#include "pika_gripper_hardware_interface/pika_protocol.hpp"

namespace protocol = pika_gripper_hardware_interface::protocol;

// ── Command encoding ─────────────────────────────────────────────────────────

TEST(EncodeCommand, PositionCommandLayout)
{
  const float value = 1.5F;
  const auto cmd = protocol::encode_command(protocol::CommandType::kPosition, value);

  EXPECT_EQ(cmd[0], 22u);
  float decoded{};
  std::memcpy(&decoded, cmd.data() + 1, 4);
  EXPECT_FLOAT_EQ(decoded, value);
  EXPECT_EQ(cmd[5], '\r');
  EXPECT_EQ(cmd[6], '\n');
}

TEST(EncodeCommand, EnableDisableTypes)
{
  EXPECT_EQ(protocol::encode_command(protocol::CommandType::kEnable, 0.0F)[0], 11u);
  EXPECT_EQ(protocol::encode_command(protocol::CommandType::kDisable, 0.0F)[0], 10u);
  EXPECT_EQ(protocol::encode_command(protocol::CommandType::kSetZero, 0.0F)[0], 12u);
}

// ── Frame assembly ───────────────────────────────────────────────────────────

namespace
{

void feed_string(protocol::FrameAssembler & assembler, const std::string & s)
{
  assembler.feed(reinterpret_cast<const uint8_t *>(s.data()), s.size());
}

const char kFullFrame[] =
  "{\r\n\"motor\":{\r\n\"Speed\":0.50,\r\n\"Current\":120,\r\n\"Position\":1.00\r\n}\r\n,\r\n"
  "\"motorstatus\":{\r\n\"Voltage\":24.1,\r\n\"DriverTemp\":31,\r\n\"MotorTemp\":35,\r\n"
  "\"Status\":\"0x00\",\r\n\"BusCurrent\":80\r\n}\r\n\r\n}";

// Quiescent stream observed on a real device before ENABLE.
const char kEmptyFrame[] = "{\r\n\"motor\":,\r\n\"motorstatus\":\r\n}";

}  // namespace

TEST(FrameAssembler, ExtractsSingleFrame)
{
  protocol::FrameAssembler assembler;
  feed_string(assembler, kFullFrame);
  const auto frame = assembler.next_frame();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(*frame, std::string(kFullFrame));
  EXPECT_FALSE(assembler.next_frame().has_value());
}

TEST(FrameAssembler, HandlesFragmentation)
{
  protocol::FrameAssembler assembler;
  const std::string full(kFullFrame);
  for (std::size_t i = 0; i + 1 < full.size(); ++i) {
    feed_string(assembler, full.substr(i, 1));
    EXPECT_FALSE(assembler.next_frame().has_value());
  }
  feed_string(assembler, full.substr(full.size() - 1));
  const auto frame = assembler.next_frame();
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(*frame, full);
}

TEST(FrameAssembler, ResyncsAfterGarbage)
{
  protocol::FrameAssembler assembler;
  feed_string(assembler, "###garbage###" + std::string(kEmptyFrame) + "!!!" + kFullFrame);
  const auto first = assembler.next_frame();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, std::string(kEmptyFrame));
  const auto second = assembler.next_frame();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(*second, std::string(kFullFrame));
}

TEST(FrameAssembler, BoundsBufferGrowth)
{
  protocol::FrameAssembler assembler;
  const std::string junk(1024, 'x');
  for (int i = 0; i < 100; ++i) {
    feed_string(assembler, junk);
    EXPECT_FALSE(assembler.next_frame().has_value());
  }
  feed_string(assembler, kFullFrame);
  EXPECT_TRUE(assembler.next_frame().has_value());
}

// ── Frame parsing ────────────────────────────────────────────────────────────

TEST(ParseFrame, FullFrame)
{
  const auto fb = protocol::parse_frame(kFullFrame);
  ASSERT_TRUE(fb.has_motor);
  EXPECT_DOUBLE_EQ(fb.speed_rad_s, 0.5);
  EXPECT_DOUBLE_EQ(fb.current_ma, 120.0);
  EXPECT_DOUBLE_EQ(fb.position_rad, 1.0);
  ASSERT_TRUE(fb.has_status);
  EXPECT_DOUBLE_EQ(fb.voltage_v, 24.1);
  EXPECT_DOUBLE_EQ(fb.driver_temp_c, 31.0);
  EXPECT_DOUBLE_EQ(fb.motor_temp_c, 35.0);
  EXPECT_EQ(fb.status_bits, 0u);
  EXPECT_DOUBLE_EQ(fb.bus_current_ma, 80.0);
}

TEST(ParseFrame, EmptyQuiescentFrame)
{
  const auto fb = protocol::parse_frame(kEmptyFrame);
  EXPECT_FALSE(fb.has_motor);
  EXPECT_FALSE(fb.has_status);
}

TEST(ParseFrame, NegativeValuesAndFaultBits)
{
  const std::string frame =
    "{\"motor\":{\"Speed\":-0.25,\"Current\":-600,\"Position\":0.75},"
    "\"motorstatus\":{\"Voltage\":23.9,\"DriverTemp\":40,\"MotorTemp\":45,"
    "\"Status\":\"0x42\",\"BusCurrent\":150}}";
  const auto fb = protocol::parse_frame(frame);
  ASSERT_TRUE(fb.has_motor);
  EXPECT_DOUBLE_EQ(fb.speed_rad_s, -0.25);
  EXPECT_DOUBLE_EQ(fb.current_ma, -600.0);
  ASSERT_TRUE(fb.has_status);
  EXPECT_EQ(fb.status_bits, 0x42u);
}

TEST(ParseFrame, PartialFrameMotorOnly)
{
  const std::string frame = "{\"motor\":{\"Speed\":0.0,\"Current\":0,\"Position\":0.1}}";
  const auto fb = protocol::parse_frame(frame);
  EXPECT_TRUE(fb.has_motor);
  EXPECT_FALSE(fb.has_status);
}

// ── Kinematics ───────────────────────────────────────────────────────────────

TEST(Kinematics, ClosedAtZero)
{
  EXPECT_NEAR(protocol::width_from_angle(0.0), 0.0, 1e-12);
}

TEST(Kinematics, MonotonicIncreasing)
{
  double prev = protocol::width_from_angle(0.0);
  for (double a = 0.05; a <= protocol::max_motor_angle(); a += 0.05) {
    const double w = protocol::width_from_angle(a);
    EXPECT_GT(w, prev) << "not monotonic at angle " << a;
    prev = w;
  }
}

TEST(Kinematics, MaxWidthPlausible)
{
  EXPECT_GT(protocol::max_width(), 0.08);
  EXPECT_LT(protocol::max_width(), 0.12);
}

TEST(Kinematics, RoundTripWidthAngle)
{
  for (double w = 0.0; w <= protocol::max_width(); w += 0.005) {
    const double angle = protocol::angle_from_width(w);
    EXPECT_NEAR(protocol::width_from_angle(angle), w, 1e-5);
  }
}

TEST(Kinematics, InverseClampsOutOfRange)
{
  EXPECT_NEAR(protocol::angle_from_width(-0.01), 0.0, 1e-6);
  EXPECT_NEAR(protocol::angle_from_width(1.0), protocol::max_motor_angle(), 1e-6);
}

TEST(Kinematics, VelocitySignMatchesSpeed)
{
  const double angle = 1.0;
  EXPECT_GT(protocol::width_velocity(angle, 1.0), 0.0);
  EXPECT_LT(protocol::width_velocity(angle, -1.0), 0.0);
  EXPECT_DOUBLE_EQ(protocol::width_velocity(angle, 0.0), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

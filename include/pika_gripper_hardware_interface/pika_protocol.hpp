// Copyright 2026 physical_ai_runtime
// SPDX-License-Identifier: Apache-2.0
//
// Pure protocol / kinematics logic for the AgileX Pika gripper serial
// interface. No ROS, no I/O — everything here is offline-testable.
//
// Protocol source of truth: agilexrobotics/pika_sdk (pika/serial_comm.py,
// pika/gripper.py) at commit observed 2026-07; verified against a live
// device stream (460800 8N1, CH340 usb-serial).

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace pika_gripper_hardware_interface::protocol
{

// ── Command encoding ─────────────────────────────────────────────────────────
//
// Wire format: [type:u8][value:f32 little-endian][\r][\n]

enum class CommandType : uint8_t
{
  kDisable = 10,
  kEnable = 11,
  kSetZero = 12,
  kCurrent = 15,
  kEffort = 20,
  kVelocity = 21,
  kPosition = 22,  // value = motor angle [rad]
};

std::array<uint8_t, 7> encode_command(CommandType type, float value);

// ── Feedback frames ──────────────────────────────────────────────────────────
//
// The device streams pseudo-JSON frames terminated by balanced braces:
//   {\r\n"motor":{\r\n"Speed":...,\r\n"Current":...,\r\n"Position":...\r\n}\r\n,
//    \r\n"motorstatus":{...}\r\n\r\n}
// Quirks the parser must tolerate:
//   - trailing commas before '}' / ']'
//   - EMPTY values while the motor is disabled: {"motor":,"motorstatus":}
//   - fragmentation at arbitrary byte boundaries
//   - garbage between frames (resync on next '{')

struct MotorFeedback
{
  // "motor" block — valid only when has_motor is true.
  bool has_motor{false};
  double speed_rad_s{0.0};    // Speed    [rad/s]
  double current_ma{0.0};     // Current  [mA]
  double position_rad{0.0};   // Position [rad], motor angle

  // "motorstatus" block — valid only when has_status is true.
  bool has_status{false};
  double voltage_v{0.0};      // Voltage    [V]
  double driver_temp_c{0.0};  // DriverTemp [degC]
  double motor_temp_c{0.0};   // MotorTemp  [degC]
  unsigned status_bits{0};    // Status "0xNN" parsed as hex
  double bus_current_ma{0.0}; // BusCurrent [mA]
};

// Accumulates raw serial bytes and extracts balanced-brace frames.
class FrameAssembler
{
public:
  void feed(const uint8_t * data, std::size_t len);

  // Returns the next complete frame (including braces), or nullopt.
  // Call repeatedly until nullopt after each feed().
  std::optional<std::string> next_frame();

private:
  std::string buffer_;
  static constexpr std::size_t kMaxBuffer = 4096;
};

// Parses one extracted frame. Missing/empty fields leave has_motor /
// has_status false; never throws.
MotorFeedback parse_frame(const std::string & frame);

// ── Width <-> motor-angle kinematics ─────────────────────────────────────────
//
// Linkage model from pika_sdk Gripper.get_distance(); all lengths in meters.
// width_from_angle(0) == 0 (closed); width grows monotonically with angle.

// Upper motor-angle bound of the monotonic working range [rad] (2.0; the
// linkage peaks near 2.21 rad and folds back beyond it).
double max_motor_angle();

// Full opening width [m] for a motor angle [rad].
double width_from_angle(double angle_rad);

// Inverse mapping, clamped to [0, max_motor_angle()]. Bisection, ~1e-6 m.
double angle_from_width(double width_m);

// Maximum commandable opening width [m] (~0.108 at kAngleUpper).
double max_width();

// Opening-width velocity [m/s] given motor angle [rad] and speed [rad/s].
double width_velocity(double angle_rad, double speed_rad_s);

// ── Write-path command filter ────────────────────────────────────────────────
//
// Pure guard logic mirrored from marvin/piper conventions
// (docs/END_EFFECTOR_CONVENTIONS.md §6). Values are ROS finger travel [m]
// (full opening on the wire ≈ 2 × travel).

struct CommandFilterConfig
{
  double min_width{0.0};
  double max_width{0.045};  // finger travel [m]
  double max_speed{0.15};   // [m/s] per-cycle step limit (finger travel)
  double deadband{5e-4};    // [m] skip re-sending unchanged targets
};

// Returns the finger travel to send this cycle, or nullopt to skip the send.
// last_sent: previously dispatched travel, NaN if never sent (then the
// measured travel is the step baseline). Non-finite targets are rejected.
// Target and measured baseline are clamped to [min_width, max_width].
std::optional<double> filter_command(
  const CommandFilterConfig & cfg, double target, double last_sent,
  double measured, double period_s);

// Clamp ROS finger travel to [min_width, max_width]. Non-finite → min_width.
double clamp_finger_travel(const CommandFilterConfig & cfg, double travel);

}  // namespace pika_gripper_hardware_interface::protocol

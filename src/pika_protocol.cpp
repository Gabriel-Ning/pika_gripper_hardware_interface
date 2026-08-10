// Copyright 2026 physical_ai_runtime
// SPDX-License-Identifier: Apache-2.0

#include "pika_gripper_hardware_interface/pika_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace pika_gripper_hardware_interface::protocol
{

// ── Command encoding ─────────────────────────────────────────────────────────

std::array<uint8_t, 7> encode_command(CommandType type, float value)
{
  std::array<uint8_t, 7> out{};
  out[0] = static_cast<uint8_t>(type);
  static_assert(sizeof(float) == 4);
  // Device expects little-endian IEEE754 (pika_sdk struct.pack('<f', v)).
  std::memcpy(out.data() + 1, &value, 4);
  out[5] = '\r';
  out[6] = '\n';
  return out;
}

// ── Frame assembly ───────────────────────────────────────────────────────────

void FrameAssembler::feed(const uint8_t * data, std::size_t len)
{
  buffer_.append(reinterpret_cast<const char *>(data), len);
  if (buffer_.size() > kMaxBuffer) {
    // Corrupt or unframed stream — drop everything before the last '{'
    // so we can resync without unbounded growth.
    const auto last_open = buffer_.rfind('{');
    if (last_open == std::string::npos) {
      buffer_.clear();
    } else {
      buffer_.erase(0, last_open);
    }
  }
}

std::optional<std::string> FrameAssembler::next_frame()
{
  const auto start = buffer_.find('{');
  if (start == std::string::npos) {
    buffer_.clear();
    return std::nullopt;
  }

  int depth = 0;
  for (std::size_t i = start; i < buffer_.size(); ++i) {
    if (buffer_[i] == '{') {
      ++depth;
    } else if (buffer_[i] == '}') {
      if (--depth == 0) {
        std::string frame = buffer_.substr(start, i - start + 1);
        buffer_.erase(0, i + 1);
        return frame;
      }
    }
  }
  // Incomplete frame — drop leading garbage, keep the partial frame.
  buffer_.erase(0, start);
  return std::nullopt;
}

// ── Frame parsing ────────────────────────────────────────────────────────────

namespace
{

// Finds `"key":` and returns the index just past the colon, or npos.
std::size_t find_value(const std::string & frame, const char * key)
{
  const std::string needle = std::string("\"") + key + "\"";
  auto pos = frame.find(needle);
  if (pos == std::string::npos) {return std::string::npos;}
  pos = frame.find(':', pos + needle.size());
  if (pos == std::string::npos) {return std::string::npos;}
  return pos + 1;
}

// Parses a numeric value at/after `pos`, skipping whitespace/CRLF.
// Returns false when the value is empty (",", "}", CRLF) or malformed.
bool parse_number(const std::string & frame, std::size_t pos, double & out)
{
  while (pos < frame.size() &&
    (frame[pos] == ' ' || frame[pos] == '\r' || frame[pos] == '\n' || frame[pos] == '\t'))
  {
    ++pos;
  }
  if (pos >= frame.size()) {return false;}
  const char c = frame[pos];
  if (c != '-' && c != '+' && !(c >= '0' && c <= '9')) {return false;}

  const char * begin = frame.c_str() + pos;
  char * end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin || !std::isfinite(value)) {return false;}
  out = value;
  return true;
}

// Parses a quoted "0xNN" hex status string.
bool parse_hex_status(const std::string & frame, std::size_t pos, unsigned & out)
{
  const auto quote = frame.find('"', pos);
  if (quote == std::string::npos) {return false;}
  const auto end_quote = frame.find('"', quote + 1);
  if (end_quote == std::string::npos) {return false;}
  const std::string token = frame.substr(quote + 1, end_quote - quote - 1);
  if (token.empty()) {return false;}
  out = static_cast<unsigned>(std::strtoul(token.c_str(), nullptr, 16));
  return true;
}

}  // namespace

MotorFeedback parse_frame(const std::string & frame)
{
  MotorFeedback fb;

  double speed{}, current{}, position{};
  const auto p_speed = find_value(frame, "Speed");
  const auto p_current = find_value(frame, "Current");
  const auto p_position = find_value(frame, "Position");
  if (p_speed != std::string::npos && p_current != std::string::npos &&
    p_position != std::string::npos &&
    parse_number(frame, p_speed, speed) &&
    parse_number(frame, p_current, current) &&
    parse_number(frame, p_position, position))
  {
    fb.has_motor = true;
    fb.speed_rad_s = speed;
    fb.current_ma = current;
    fb.position_rad = position;
  }

  double voltage{}, driver_temp{}, motor_temp{}, bus_current{};
  unsigned status{};
  const auto p_voltage = find_value(frame, "Voltage");
  const auto p_driver_temp = find_value(frame, "DriverTemp");
  const auto p_motor_temp = find_value(frame, "MotorTemp");
  const auto p_status = find_value(frame, "Status");
  const auto p_bus = find_value(frame, "BusCurrent");
  if (p_voltage != std::string::npos && p_driver_temp != std::string::npos &&
    p_motor_temp != std::string::npos && p_status != std::string::npos &&
    p_bus != std::string::npos &&
    parse_number(frame, p_voltage, voltage) &&
    parse_number(frame, p_driver_temp, driver_temp) &&
    parse_number(frame, p_motor_temp, motor_temp) &&
    parse_hex_status(frame, p_status, status) &&
    parse_number(frame, p_bus, bus_current))
  {
    fb.has_status = true;
    fb.voltage_v = voltage;
    fb.driver_temp_c = driver_temp;
    fb.motor_temp_c = motor_temp;
    fb.status_bits = status;
    fb.bus_current_ma = bus_current;
  }

  return fb;
}

// ── Kinematics ───────────────────────────────────────────────────────────────

namespace
{

// Linkage constants from pika_sdk Gripper.get_distance() [meters].
constexpr double kCrank = 0.0325;
constexpr double kCoupler = 0.058;
constexpr double kHeightOffset = 0.01456;
const double kAngleZero = (180.0 - 43.99) / 180.0 * M_PI;
// The linkage width peaks near 2.21 rad and decreases beyond it; cap the
// working range to 2.0 rad (width ~0.108 m) so width_from_angle stays
// strictly monotonic and the bisection inverse is well-defined. The physical
// device opens ~0.09 m, well inside this bound.
constexpr double kAngleUpper = 2.0;

// Half opening distance of one finger [m] for a motor angle [rad].
double half_distance(double angle_rad)
{
  const double a = kAngleZero - angle_rad;
  const double height = kCrank * std::sin(a);
  const double width_d = kCrank * std::cos(a);
  const double dy = height - kHeightOffset;
  return std::sqrt(kCoupler * kCoupler - dy * dy) + width_d;
}

}  // namespace

double max_motor_angle() {return kAngleUpper;}

double width_from_angle(double angle_rad)
{
  return 2.0 * (half_distance(angle_rad) - half_distance(0.0));
}

double max_width() {return width_from_angle(kAngleUpper);}

double angle_from_width(double width_m)
{
  const double clamped = std::clamp(width_m, 0.0, max_width());
  double low = 0.0;
  double high = kAngleUpper;
  // width_from_angle is monotonically increasing on [0, kAngleUpper].
  for (int i = 0; i < 60 && (high - low) > 1e-9; ++i) {
    const double mid = 0.5 * (low + high);
    if (width_from_angle(mid) < clamped) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return 0.5 * (low + high);
}

double width_velocity(double angle_rad, double speed_rad_s)
{
  constexpr double kEps = 1e-5;
  const double dwidth_dangle =
    (width_from_angle(angle_rad + kEps) - width_from_angle(angle_rad - kEps)) / (2.0 * kEps);
  return dwidth_dangle * speed_rad_s;
}

// ── Command filter ───────────────────────────────────────────────────────────

std::optional<double> filter_command(
  const CommandFilterConfig & cfg, double target, double last_sent,
  double measured, double period_s)
{
  if (!std::isfinite(target)) {return std::nullopt;}

  double clamped = std::clamp(target, cfg.min_width, cfg.max_width);

  const bool first_write = !std::isfinite(last_sent);
  const double baseline = first_write ? measured : last_sent;

  const double max_step = cfg.max_speed * period_s;
  clamped = std::clamp(clamped, baseline - max_step, baseline + max_step);

  if (!first_write && std::fabs(clamped - last_sent) < cfg.deadband) {
    return std::nullopt;
  }
  return clamped;
}

}  // namespace pika_gripper_hardware_interface::protocol

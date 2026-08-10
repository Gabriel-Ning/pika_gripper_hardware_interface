// Copyright 2026 physical_ai_runtime
// SPDX-License-Identifier: Apache-2.0

#include "pika_gripper_hardware_interface/pika_gripper_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace pika_gripper_hardware_interface
{

namespace
{

// ROS joint uses single-finger travel [m]; vendor protocol uses full opening.
// Same factor as PiperGripperInterface.
constexpr double kFingerTravelPerOpeningWidth = 0.5;
constexpr double kDefaultMaxFingerTravelM = 0.045;

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double parse_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, double fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {return fallback;}
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return fallback;
  }
}

bool parse_bool_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, bool fallback)
{
  const auto it = params.find(key);
  if (it == params.end()) {return fallback;}
  return it->second == "true" || it->second == "True" || it->second == "1";
}

}  // namespace

PikaGripperInterface::~PikaGripperInterface()
{
  // Safety net: on_deactivate/on_cleanup may never run on a crash.
  if (port_.is_open()) {
    const auto disable = protocol::encode_command(protocol::CommandType::kDisable, 0.0F);
    port_.write_all(disable.data(), disable.size());
  }
  stop_threads();
  port_.close();
}

hardware_interface::CallbackReturn PikaGripperInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(logger_, "Expected exactly 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & joint = info_.joints[0];
  joint_name_ = joint.name;

  if (joint.command_interfaces.size() != 1 ||
    joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(logger_, "Joint '%s' must declare exactly one 'position' command interface",
      joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hw_params = info_.hardware_parameters;
  const auto port_it = hw_params.find("serial_port");
  if (port_it != hw_params.end() && !port_it->second.empty()) {
    serial_port_ = port_it->second;
  }
  baudrate_ = static_cast<int>(parse_param(hw_params, "baudrate", 460800.0));
  enable_on_activate_ = parse_bool_param(hw_params, "enable_on_activate", true);
  disable_on_deactivate_ = parse_bool_param(hw_params, "disable_on_deactivate", true);
  activation_timeout_s_ = parse_param(hw_params, "activation_timeout_s", 2.0);
  stale_warn_s_ = parse_param(hw_params, "stale_warn_s", 0.1);
  stale_error_s_ = parse_param(hw_params, "stale_error_s", 0.5);

  filter_cfg_.min_width = parse_param(hw_params, "min_width", 0.0);
  // User-facing max_width is ROS finger travel [m]; never exceed half of the
  // linkage opening ceiling so the wire conversion stays in range.
  const double linkage_finger_max = protocol::max_width() * kFingerTravelPerOpeningWidth;
  filter_cfg_.max_width = std::min(
    parse_param(hw_params, "max_width", kDefaultMaxFingerTravelM), linkage_finger_max);
  filter_cfg_.max_speed = parse_param(hw_params, "max_speed", 0.15);
  filter_cfg_.deadband = parse_param(hw_params, "command_deadband", 5e-4);

  if_voltage_ = joint_name_ + "/voltage";
  if_driver_temp_ = joint_name_ + "/driver_temp";
  if_motor_temp_ = joint_name_ + "/motor_temp";
  if_fault_ = joint_name_ + "/fault";

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::InterfaceDescription>
PikaGripperInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> descriptions;
  for (const char * name : {"voltage", "driver_temp", "motor_temp", "fault"}) {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    descriptions.emplace_back(joint_name_, info);
  }
  return descriptions;
}

hardware_interface::CallbackReturn PikaGripperInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!port_.open(serial_port_, baudrate_)) {
    RCLCPP_ERROR(logger_, "Failed to open %s @ %d: %s",
      serial_port_.c_str(), baudrate_, port_.error().c_str());
    return hardware_interface::CallbackReturn::FAILURE;
  }
  start_threads();
  RCLCPP_INFO(logger_, "Connected to Pika gripper on %s @ %d", serial_port_.c_str(), baudrate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PikaGripperInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (enable_on_activate_) {
    if (!send_simple_command(protocol::CommandType::kEnable)) {
      RCLCPP_ERROR(logger_, "Failed to send ENABLE command");
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }

  // The device streams empty frames until the motor reports; require valid
  // feedback before exposing command interfaces so activation never starts
  // from a made-up position.
  if (!wait_for_feedback(activation_timeout_s_)) {
    RCLCPP_ERROR(logger_,
      "No valid motor feedback within %.1f s after %s — is the gripper powered?",
      activation_timeout_s_, enable_on_activate_ ? "ENABLE" : "activation");
    if (enable_on_activate_) {
      send_simple_command(protocol::CommandType::kDisable);
    }
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Sync command to measured finger travel to prevent jumps on activation.
  const double width = cached_width_.load();
  set_command(joint_name_ + "/" + hardware_interface::HW_IF_POSITION, width);
  set_state(joint_name_ + "/" + hardware_interface::HW_IF_POSITION, width);
  last_sent_width_ = std::numeric_limits<double>::quiet_NaN();
  writer_io_failed_.store(false);
  activated_ = true;

  RCLCPP_INFO(logger_, "Activated; measured finger travel %.4f m", width);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PikaGripperInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  activated_ = false;
  if (disable_on_deactivate_) {
    send_simple_command(protocol::CommandType::kDisable);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PikaGripperInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_threads();
  port_.close();
  has_feedback_.store(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PikaGripperInterface::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR(logger_, "Hardware error from state '%s' — safing gripper",
    previous_state.label().c_str());
  activated_ = false;
  send_simple_command(protocol::CommandType::kDisable);
  stop_threads();
  port_.close();
  has_feedback_.store(false);
  // SUCCESS -> Unconfigured: a later configure can reconnect.
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PikaGripperInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!has_feedback_.load()) {
    // Before the first valid frame there is nothing to report (INACTIVE
    // bring-up); do not fail the loop.
    return hardware_interface::return_type::OK;
  }

  set_state(joint_name_ + "/" + hardware_interface::HW_IF_POSITION, cached_width_.load());
  set_state(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY, cached_velocity_.load());
  set_state(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT, cached_current_ma_.load());
  set_state(if_voltage_, cached_voltage_.load());
  set_state(if_driver_temp_, cached_driver_temp_.load());
  set_state(if_motor_temp_, cached_motor_temp_.load());
  set_state(if_fault_, static_cast<double>(cached_status_bits_.load()));

  const double stale_s = static_cast<double>(steady_now_ns() - last_frame_ns_.load()) * 1e-9;
  if (activated_ && stale_s > stale_error_s_) {
    RCLCPP_ERROR(logger_, "Feedback stale for %.3f s (> %.3f s) — erroring out",
      stale_s, stale_error_s_);
    return hardware_interface::return_type::ERROR;
  }
  if (activated_ && stale_s > stale_warn_s_) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
      "Feedback stale for %.3f s", stale_s);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PikaGripperInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!activated_) {
    return hardware_interface::return_type::OK;
  }
  if (writer_io_failed_.load()) {
    RCLCPP_ERROR(logger_, "Writer thread reported serial I/O failure");
    return hardware_interface::return_type::ERROR;
  }

  const double target =
    get_command(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  const auto filtered = protocol::filter_command(
    filter_cfg_, target, last_sent_width_, cached_width_.load(), period.seconds());
  if (!filtered.has_value()) {
    return hardware_interface::return_type::OK;
  }

  last_sent_width_ = *filtered;
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    writer_target_width_ = *filtered;
    writer_has_cmd_ = true;
  }
  writer_cv_.notify_one();
  return hardware_interface::return_type::OK;
}

// ── Background threads ───────────────────────────────────────────────────────

void PikaGripperInterface::start_threads()
{
  reader_stop_.store(false);
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    writer_stop_ = false;
    writer_has_cmd_ = false;
  }
  reader_thread_ = std::thread(&PikaGripperInterface::reader_thread_fn, this);
  writer_thread_ = std::thread(&PikaGripperInterface::writer_thread_fn, this);
}

void PikaGripperInterface::stop_threads()
{
  reader_stop_.store(true);
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    writer_stop_ = true;
  }
  writer_cv_.notify_one();
  if (reader_thread_.joinable()) {reader_thread_.join();}
  if (writer_thread_.joinable()) {writer_thread_.join();}
}

void PikaGripperInterface::reader_thread_fn()
{
  protocol::FrameAssembler assembler;
  uint8_t buffer[512];

  while (!reader_stop_.load()) {
    const ssize_t n = port_.read_some(buffer, sizeof(buffer));
    if (n < 0) {
      // Port error (e.g. unplugged). Stale detection in read() reports it;
      // avoid a hot loop while the port is broken.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    if (n == 0) {continue;}  // 100 ms timeout tick — lets us poll reader_stop_

    assembler.feed(buffer, static_cast<std::size_t>(n));
    while (auto frame = assembler.next_frame()) {
      const auto fb = protocol::parse_frame(*frame);
      if (fb.has_motor) {
        const double opening = protocol::width_from_angle(fb.position_rad);
        cached_width_.store(opening * kFingerTravelPerOpeningWidth);
        cached_velocity_.store(
          protocol::width_velocity(fb.position_rad, fb.speed_rad_s) *
          kFingerTravelPerOpeningWidth);
        cached_current_ma_.store(fb.current_ma);
        last_frame_ns_.store(steady_now_ns());
        has_feedback_.store(true);
      }
      if (fb.has_status) {
        cached_voltage_.store(fb.voltage_v);
        cached_driver_temp_.store(fb.driver_temp_c);
        cached_motor_temp_.store(fb.motor_temp_c);
        cached_status_bits_.store(fb.status_bits);
      }
    }
  }
}

void PikaGripperInterface::writer_thread_fn()
{
  while (true) {
    double finger_travel{};
    {
      std::unique_lock<std::mutex> lock(writer_mutex_);
      writer_cv_.wait(lock, [this] {return writer_has_cmd_ || writer_stop_;});
      if (writer_stop_) {return;}
      finger_travel = writer_target_width_;
      writer_has_cmd_ = false;
    }

    const double opening = finger_travel / kFingerTravelPerOpeningWidth;
    const double angle = protocol::angle_from_width(opening);
    const auto cmd = protocol::encode_command(
      protocol::CommandType::kPosition, static_cast<float>(angle));
    if (!port_.write_all(cmd.data(), cmd.size())) {
      writer_io_failed_.store(true);
    }
  }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

bool PikaGripperInterface::send_simple_command(protocol::CommandType type)
{
  // Serialize against the writer thread — both write to the same fd.
  std::lock_guard<std::mutex> lock(writer_mutex_);
  const auto cmd = protocol::encode_command(type, 0.0F);
  return port_.write_all(cmd.data(), cmd.size());
}

bool PikaGripperInterface::wait_for_feedback(double timeout_s)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  const int64_t start_ns = steady_now_ns();
  while (std::chrono::steady_clock::now() < deadline) {
    if (has_feedback_.load() && last_frame_ns_.load() > start_ns) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace pika_gripper_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pika_gripper_hardware_interface::PikaGripperInterface,
  hardware_interface::SystemInterface)

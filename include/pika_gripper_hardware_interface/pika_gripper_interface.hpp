// Copyright 2026 physical_ai_runtime
// SPDX-License-Identifier: Apache-2.0
//
// ros2_control SystemInterface for the AgileX Pika gripper (serial/USB).
//
// The gripper streams pseudo-JSON feedback and accepts binary commands over
// one 460800-baud serial port (see pika_protocol.hpp). Per
// docs/END_EFFECTOR_CONVENTIONS.md §5, the RT read()/write() callbacks never
// touch the port:
//   - reader thread: reads the port, parses frames, publishes the latest
//     state into std::atomic caches. read() copies the caches.
//   - writer thread: waits on a condition variable; write() posts a filtered
//     finger-travel target, the writer converts travel → opening → motor angle
//     and sends the POSITION command.
//
// Exposes one joint (URDF-declared):
//   - command: position [m]  (single-finger travel; opening ≈ 2×)
//   - state:   position [m], velocity [m/s],
//              effort [motor phase current, mA — grip-force proxy, see README]
// plus unlisted diagnostic states: voltage [V], driver_temp [degC],
// motor_temp [degC], fault (status bits as double).
//
// Compatible with parallel_gripper_action_controller/GripperActionController.

#pragma once

#include <atomic>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pika_gripper_hardware_interface/pika_protocol.hpp"
#include "pika_gripper_hardware_interface/pika_serial_port.hpp"

namespace pika_gripper_hardware_interface
{

class PikaGripperInterface final : public hardware_interface::SystemInterface
{
public:
  PikaGripperInterface() = default;
  ~PikaGripperInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void start_threads();
  void stop_threads();
  void reader_thread_fn();
  void writer_thread_fn();
  bool send_simple_command(protocol::CommandType type);
  bool wait_for_feedback(double timeout_s);

  // ── Hardware parameters (URDF <hardware><param>) ────────────────────────
  std::string serial_port_{"/dev/ttyUSB0"};
  int baudrate_{460800};
  bool enable_on_activate_{true};
  bool disable_on_deactivate_{true};
  double activation_timeout_s_{2.0};
  double stale_warn_s_{0.1};
  double stale_error_s_{0.5};
  protocol::CommandFilterConfig filter_cfg_{};

  std::string joint_name_;

  // ── Transport (owned by background threads after start_threads) ─────────
  PikaSerialPort port_;

  // ── Reader thread -> RT read() caches ────────────────────────────────────
  // Finger travel [m] (= vendor opening width × 0.5), matching URDF / Piper.
  std::atomic<double> cached_width_{0.0};
  std::atomic<double> cached_velocity_{0.0};
  std::atomic<double> cached_current_ma_{0.0};
  std::atomic<double> cached_voltage_{0.0};
  std::atomic<double> cached_driver_temp_{0.0};
  std::atomic<double> cached_motor_temp_{0.0};
  std::atomic<unsigned> cached_status_bits_{0};
  std::atomic<bool> has_feedback_{false};
  // steady_clock nanoseconds of the last valid motor frame.
  std::atomic<int64_t> last_frame_ns_{0};

  std::atomic<bool> reader_stop_{false};
  std::thread reader_thread_;

  // ── RT write() -> writer thread handoff ──────────────────────────────────
  std::mutex writer_mutex_;
  std::condition_variable writer_cv_;
  double writer_target_width_{0.0};
  bool writer_has_cmd_{false};
  bool writer_stop_{false};
  std::thread writer_thread_;
  std::atomic<bool> writer_io_failed_{false};

  // RT-thread-only write bookkeeping. NaN = never sent since activation.
  double last_sent_width_{std::numeric_limits<double>::quiet_NaN()};

  // ── Unlisted diagnostic interface names (fully qualified) ────────────────
  std::string if_voltage_, if_driver_temp_, if_motor_temp_, if_fault_;

  bool activated_{false};

  rclcpp::Logger logger_{rclcpp::get_logger("PikaGripperInterface")};
};

}  // namespace pika_gripper_hardware_interface

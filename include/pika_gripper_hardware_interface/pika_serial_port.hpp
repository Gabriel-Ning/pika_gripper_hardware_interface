// Copyright 2026 physical_ai_runtime
// SPDX-License-Identifier: Apache-2.0
//
// Minimal termios wrapper for the Pika gripper serial link (raw 8N1).
// Reads are used only from the background reader thread; writes only from
// the background writer thread. Not thread-safe by itself.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace pika_gripper_hardware_interface
{

class PikaSerialPort
{
public:
  PikaSerialPort() = default;
  ~PikaSerialPort();

  PikaSerialPort(const PikaSerialPort &) = delete;
  PikaSerialPort & operator=(const PikaSerialPort &) = delete;

  // Opens and configures the port (raw, 8N1, given baud). Returns false on
  // any failure; error() then describes the cause.
  bool open(const std::string & device, int baudrate);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Blocking read with ~100 ms timeout (VMIN=0, VTIME=1). Returns bytes
  // read, 0 on timeout, -1 on error.
  ssize_t read_some(uint8_t * buffer, std::size_t capacity);

  // Writes the full buffer. Returns false on short write or error.
  bool write_all(const uint8_t * data, std::size_t len);

  const std::string & error() const {return error_;}

private:
  int fd_{-1};
  std::string error_;
};

}  // namespace pika_gripper_hardware_interface

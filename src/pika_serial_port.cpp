// Copyright 2026 physical_ai_runtime
// SPDX-License-Identifier: Apache-2.0

#include "pika_gripper_hardware_interface/pika_serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace pika_gripper_hardware_interface
{

namespace
{

bool to_speed(int baudrate, speed_t & out)
{
  switch (baudrate) {
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
    case 460800: out = B460800; return true;
    case 921600: out = B921600; return true;
    default: return false;
  }
}

}  // namespace

PikaSerialPort::~PikaSerialPort() {close();}

bool PikaSerialPort::open(const std::string & device, int baudrate)
{
  close();

  speed_t speed{};
  if (!to_speed(baudrate, speed)) {
    error_ = "unsupported baudrate " + std::to_string(baudrate);
    return false;
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    error_ = "open(" + device + "): " + std::strerror(errno);
    return false;
  }

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;
  // Blocking read with 100 ms timeout: lets the reader thread poll its
  // stop flag without busy-waiting.
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void PikaSerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t PikaSerialPort::read_some(uint8_t * buffer, std::size_t capacity)
{
  if (fd_ < 0) {return -1;}
  const ssize_t n = ::read(fd_, buffer, capacity);
  if (n < 0 && (errno == EAGAIN || errno == EINTR)) {return 0;}
  return n;
}

bool PikaSerialPort::write_all(const uint8_t * data, std::size_t len)
{
  if (fd_ < 0) {return false;}
  std::size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0) {
      if (errno == EINTR) {continue;}
      error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
}

}  // namespace pika_gripper_hardware_interface

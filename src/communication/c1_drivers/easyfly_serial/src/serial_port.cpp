// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_serial/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
#include "easyfly_serial/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

namespace easyfly_serial
{

// ---- 波特率 int → speed_t 转换 ----
static speed_t baudrate_to_speed(int baudrate)
{
  switch (baudrate) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 576000:  return B576000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    default:      return B0;
  }
}

// ---- 构造 / 析构 ----

SerialPort::SerialPort()
  : fd_(-1)
  , baudrate_(0)
{}

SerialPort::~SerialPort()
{
  close();
}

// ---- open ----

bool SerialPort::open(const std::string & device, int baudrate)
{
  // 先关闭已打开的串口
  if (fd_ >= 0) {
    close();
  }

  // --- 1. 打开设备 ---
  // O_RDWR  |  读写
  // O_NOCTTY |  不作为控制终端
  // O_NDELAY |  非阻塞打开（避免 DCD 信号阻塞）
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) {
    return false;
  }

  // --- 2. 获取当前属性 ---
  struct termios options;
  if (tcgetattr(fd_, &options) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // --- 3. 设置波特率 ---
  speed_t speed = baudrate_to_speed(baudrate);
  if (speed == B0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  // --- 4. 控制模式 ---
  // 8 数据位, 无校验, 1 停止位 (8N1)
  options.c_cflag &= ~PARENB;   // 无校验
  options.c_cflag &= ~CSTOPB;   // 1 停止位
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;       // 8 数据位
  options.c_cflag |= CLOCAL;    // 忽略调制解调器控制线
  options.c_cflag |= CREAD;     // 启用接收

  // --- 5. 本地模式 ---
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 原始模式
  options.c_iflag &= ~(IXON | IXOFF | IXANY);           // 无软件流控
  options.c_iflag &= ~IGNBRK;
  options.c_iflag &= ~(INLCR | ICRNL);                  // 不做换行转换
  options.c_oflag &= ~OPOST;                             // 不做输出处理

  // --- 6. 超时 (VTIME / VMIN) ---
  // 由 read() 的 select() 来控制超时，这里设为非阻塞底层读
  options.c_cc[VMIN]  = 0;
  options.c_cc[VTIME] = 0;

  // --- 7. 刷入配置 ---
  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // --- 8. 清空缓冲区 ---
  tcflush(fd_, TCIOFLUSH);

  baudrate_ = baudrate;
  return true;
}

// ---- close ----

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
    baudrate_ = 0;
  }
}

// ---- is_open ----

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

// ---- read (带 select 超时) ----

size_t SerialPort::read(uint8_t * buf, size_t len, int timeout_ms)
{
  if (fd_ < 0 || buf == nullptr || len == 0) {
    return 0;
  }

  // 用 select() 等待数据可读或超时
  if (timeout_ms >= 0) {
    fd_set readset;
    FD_ZERO(&readset);
    FD_SET(fd_, &readset);

    struct timeval tv;
    struct timeval * ptv = nullptr;
    if (timeout_ms > 0) {
      tv.tv_sec  = timeout_ms / 1000;
      tv.tv_usec = (timeout_ms % 1000) * 1000;
      ptv = &tv;
    }
    // timeout_ms == 0: ptv = nullptr → select 立即返回

    int ret = select(fd_ + 1, &readset, nullptr, nullptr, ptv);
    if (ret < 0) {
      return 0;   // select 出错
    }
    if (ret == 0) {
      return 0;   // 超时，无数据
    }
  }
  // timeout_ms == -1: 跳过 select，直接阻塞 read，无限等待

  ssize_t n = ::read(fd_, buf, len);
  if (n < 0) {
    return 0;
  }
  return static_cast<size_t>(n);
}

// ---- write ----

size_t SerialPort::write(const uint8_t * buf, size_t len)
{
  if (fd_ < 0 || buf == nullptr || len == 0) {
    return 0;
  }

  ssize_t n = ::write(fd_, buf, len);
  if (n < 0) {
    return 0;
  }
  return static_cast<size_t>(n);
}

// ---- flush ----

void SerialPort::flush()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIOFLUSH);
  }
}

// ---- set_baudrate ----

bool SerialPort::set_baudrate(int baudrate)
{
  if (fd_ < 0) {
    return false;
  }

  speed_t speed = baudrate_to_speed(baudrate);
  if (speed == B0) {
    return false;
  }

  struct termios options;
  if (tcgetattr(fd_, &options) != 0) {
    return false;
  }
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    return false;
  }
  baudrate_ = baudrate;
  return true;
}

}  // namespace easyfly_serial

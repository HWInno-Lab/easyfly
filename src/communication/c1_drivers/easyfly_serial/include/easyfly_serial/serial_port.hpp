// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_serial/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
#ifndef EASYFLY_SERIAL_SERIAL_PORT_HPP_
#define EASYFLY_SERIAL_SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace easyfly_serial
{

/**
 * @brief 通用串口操作类，封装 POSIX termios
 *
 * 零 ROS2 依赖，纯 C++17 标准库 + POSIX 系统调用。
 * 用于所有需要直连串口外设的模块（云台、载荷、直连接收机等）。
 *
 * 用法：
 * @code
 *   SerialPort sp;
 *   if (!sp.open("/dev/ttyUSB0", 921600)) { ... }
 *   uint8_t buf[256];
 *   size_t n = sp.read(buf, sizeof(buf), 100);  // 100ms 超时
 *   sp.write(buf, n);
 *   sp.close();
 * @endcode
 */
class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  // 禁止拷贝
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /**
   * @brief 打开串口设备
   * @param device  设备路径，如 "/dev/ttyUSB0"
   * @param baudrate 波特率 (9600, 57600, 115200, 921600 等)
   * @return true 成功, false 失败
   */
  bool open(const std::string & device, int baudrate);

  /**
   * @brief 关闭串口
   */
  void close();

  /**
   * @brief 串口是否已打开
   */
  bool is_open() const;

  /**
   * @brief 读取数据（阻塞，带超时）
   * @param buf        接收缓冲区
   * @param len        期望读取的最大字节数
   * @param timeout_ms 超时毫秒。0=立即返回, -1=无限阻塞
   * @return 实际读取的字节数，出错或超时返回 0
   */
  size_t read(uint8_t * buf, size_t len, int timeout_ms);

  /**
   * @brief 写入数据
   * @param buf  发送缓冲区
   * @param len  写入字节数
   * @return 实际写入的字节数，出错返回 0
   */
  size_t write(const uint8_t * buf, size_t len);

  /**
   * @brief 清空收发缓冲区
   */
  void flush();

  /**
   * @brief 动态修改波特率（串口需已打开）
   * @return true 成功
   */
  bool set_baudrate(int baudrate);

private:
  int fd_;            // 文件描述符, -1 = 未打开
  int baudrate_;
};

}  // namespace easyfly_serial

#endif  // EASYFLY_SERIAL_SERIAL_PORT_HPP_

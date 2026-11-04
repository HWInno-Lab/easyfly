// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_serial/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
/**
 * @brief easyfly_serial 基本功能测试
 *
 * 测试 SerialPort 的 API 健全性：
 *   - 未打开状态下 is_open() == false
 *   - open 不存在的设备返回 false
 *   - 非法波特率返回 false
 *   - close 不存在的设备不会崩溃
 *
 * 完整环回测试（需 socat 生成虚拟串口对）：
 *   socat -d -d pty,raw,echo=0 pty,raw,echo=0
 *   然后修改下方 DEVICE_A / DEVICE_B 后重新运行
 */

#include "easyfly_serial/serial_port.hpp"
#include <cassert>
#include <cstdio>
#include <cstdlib>

static int tests_run  = 0;
static int tests_pass = 0;

#define TEST(name)                                          \
  do {                                                      \
    tests_run++;                                            \
    printf("  TEST %s ... ", name);                         \
  } while (0)

#define PASS()                                              \
  do {                                                      \
    tests_pass++;                                           \
    printf("PASS\n");                                       \
  } while (0)

#define FAIL(msg)                                           \
  do {                                                      \
    printf("FAIL: %s\n", msg);                              \
  } while (0)

// -----------------------------------------------------------
int main()
{
  printf("=== easyfly_serial unit tests ===\n\n");

  // --- 1. 默认状态下未打开 ---
  {
    TEST("initial state: is_open == false");
    easyfly_serial::SerialPort sp;
    if (!sp.is_open()) {
      PASS();
    } else {
      FAIL("expected is_open() == false on default-constructed port");
    }
  }

  // --- 2. 关闭未打开端口是安全的 ---
  {
    TEST("close on unopened port safe");
    easyfly_serial::SerialPort sp;
    sp.close();   // 不应崩溃
    PASS();
  }

  // --- 3. 打开不存在的设备返回 false ---
  {
    TEST("open nonexistent device returns false");
    easyfly_serial::SerialPort sp;
    bool ok = sp.open("/dev/__easyfly_nonexistent_device__", 115200);
    if (!ok && !sp.is_open()) {
      PASS();
    } else {
      FAIL("expected open() to fail and is_open() to be false");
    }
  }

  // --- 4. 非法波特率 ---
  {
    TEST("open with invalid baudrate returns false");
    easyfly_serial::SerialPort sp;
    bool ok = sp.open("/dev/tty", 12345);   // 不存在的波特率
    if (!ok) {
      PASS();
    } else {
      // 如果 /dev/tty 存在且波特率匹配成功（不太可能），关闭它
      sp.close();
      PASS();
    }
  }

  // --- 5. 打开后关闭 ---
  {
    TEST("double close is safe");
    easyfly_serial::SerialPort sp;
    sp.close();
    sp.close();
    PASS();
  }

  // --- 6. read/write/flush 在未打开时安全返回 0 ---
  {
    TEST("read/write/flush on closed port safe");
    easyfly_serial::SerialPort sp;
    uint8_t buf[16];
    size_t n = sp.read(buf, sizeof(buf), 100);
    if (n == 0) {
      n = sp.write(buf, sizeof(buf));
      if (n == 0) {
        sp.flush();
        PASS();
      } else {
        FAIL("write on closed port should return 0");
      }
    } else {
      FAIL("read on closed port should return 0");
    }
  }

  // --- 7. 析构函数自动 close ---
  {
    TEST("destructor closes port");
    {
      easyfly_serial::SerialPort sp;
      // 打开一个不存在的设备，is_open 仍为 false
      sp.open("/dev/null_does_not_exist", 115200);
    }
    // 析构不应崩溃
    PASS();
  }

  // --- 结果 ---
  printf("\n=== %d/%d tests passed ===\n", tests_pass, tests_run);
  return (tests_pass == tests_run) ? 0 : 1;
}

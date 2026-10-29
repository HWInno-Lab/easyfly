#ifndef QUADROTOR_MSGS__ENCODE_MSGS_H_
#define QUADROTOR_MSGS__ENCODE_MSGS_H_

#include <stdint.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_msgs/Gains.h>
#include <vector>

namespace quadrotor_msgs
{

void encodeSO3Command(
  const quadrotor_msgs::SO3Command & so3_command,
  std::vector<uint8_t> & output);
void encodeTRPYCommand(
  const quadrotor_msgs::TRPYCommand & trpy_command,
  std::vector<uint8_t> & output);

void encodePPRGains(
  const quadrotor_msgs::Gains & gains,
  std::vector<uint8_t> & output);
}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__ENCODE_MSGS_H_

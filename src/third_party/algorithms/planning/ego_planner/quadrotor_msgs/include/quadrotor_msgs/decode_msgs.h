#ifndef QUADROTOR_MSGS__DECODE_MSGS_H_
#define QUADROTOR_MSGS__DECODE_MSGS_H_

#include <stdint.h>
#include <quadrotor_msgs/OutputData.h>
#include <quadrotor_msgs/StatusData.h>
#include <quadrotor_msgs/PPROutputData.h>
#include <vector>

namespace quadrotor_msgs
{

bool decodeOutputData(
  const std::vector<uint8_t> & data,
  quadrotor_msgs::OutputData & output);

bool decodeStatusData(
  const std::vector<uint8_t> & data,
  quadrotor_msgs::StatusData & status);

bool decodePPROutputData(
  const std::vector<uint8_t> & data,
  quadrotor_msgs::PPROutputData & output);
}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__DECODE_MSGS_H_

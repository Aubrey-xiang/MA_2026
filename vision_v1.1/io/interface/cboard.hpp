#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{   

struct __attribute__((packed)) Vision_receive_q //10byte
{
  uint8_t head = 0x21;
  uint8_t q[8];    // wxyz顺序
  uint8_t crc8;
};

class CBoard 
{
public:
  CBoard(const std::string & config_path);
  ~CBoard();
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;
  IMUData data_ahead_;
  IMUData data_behind_;

  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex;
  Vision_receive_q qdata_;

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();

};

}// namespace io

#endif// IO__CBoard_HPP
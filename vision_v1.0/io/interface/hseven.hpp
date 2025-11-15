#ifndef IO__HSEVEN_HPP
#define IO__HSEVEN_HPP

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
struct __attribute__((packed)) Vision_receive //16字节
{
  uint8_t head = 0xff;
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  uint8_t q[8];  
  float bullet_speed;
  uint16_t crc16;
};

struct __attribute__((packed)) Vision_send  //12字节
{
  uint8_t head = 0xff;
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw; //4字节
  float pitch;//4字节
  uint16_t crc16;//2字节
};

enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

class Hseven 
{
public:
  double bullet_speed;
  Mode mode;
  Hseven(const std::string & config_path);
  ~Hseven();
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  void send(Command command);

private:
  std::string serial_name;

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
  Vision_receive rxdata_;
  Vision_send tx_data_;

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();

};

}// namespace io

#endif// IO__HSEVEN_HPP
#include "cboard.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <iomanip>
namespace io
{
CBoard::CBoard(const std::string & config_path) : queue_(5000)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "serial_cboard");
  serial::Timeout t = serial::Timeout::simpleTimeout(20);
  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200);
    serial_.setTimeout(t);
    serial_.open();
    } catch (const std::exception & e) {
        tools::logger()->error("[CBoard] Failed to open serial: {}", e.what());
        exit(1);
    } 
    thread_ = std::thread(&CBoard::read_thread, this);
    queue_.pop();
    tools::logger()->info("[CBoard] First q received.");
}


CBoard::~CBoard()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}


Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}


bool CBoard::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[CBoard] Failed to read serial: {}", e.what());
    return false;
  }
}


void CBoard::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[CBoard] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[CBoard] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[CBoard] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}



void CBoard::read_thread()
{
  tools::logger()->info("[CBoard] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[CBoard] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&qdata_), sizeof(qdata_.head))) {
      error_count++;
      continue;
    }

    if (qdata_.head != 0x21) continue;

    if (!read(
          reinterpret_cast<uint8_t *>(&qdata_) + sizeof(qdata_.head),
          sizeof(qdata_) - sizeof(qdata_.head))) {
          error_count++;
      continue;
    }

    if (!tools::check_crc8(reinterpret_cast<uint8_t *>(&qdata_), sizeof(qdata_))) {
      tools::logger()->debug("[CBoard] CRC8 check failed.");
      continue;
    }

    error_count = 0;
    std::lock_guard<std::mutex> lock(mutex);

    auto timestamp = std::chrono::steady_clock::now();
    auto w = (int16_t)(qdata_.q[0] << 8 | qdata_.q[1]) / 1e3;
    auto x = (int16_t)(qdata_.q[2] << 8 | qdata_.q[3]) / 1e3;
    auto y = (int16_t)(qdata_.q[4] << 8 | qdata_.q[5]) / 1e3;
    auto z = (int16_t)(qdata_.q[6] << 8 | qdata_.q[7]) / 1e3;
 
    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      continue;
    }
    queue_.push({{w,x,y,z},timestamp});

  }

  tools::logger()->info("[CBoarb] read_thread stopped.");
  
}
}
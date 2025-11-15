#include "hseven.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <iomanip>
namespace io
{
Hseven::Hseven(const std::string & config_path)
: mode(Mode::idle), 
  bullet_speed(0),
  queue_(5000)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  serial::Timeout t = serial::Timeout::simpleTimeout(20);
  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200);
    serial_.setTimeout(t);
    serial_.open();
    } catch (const std::exception & e) {
        tools::logger()->error("[hseven] Failed to open serial: {}", e.what());
        exit(1);
    } 
    thread_ = std::thread(&Hseven::read_thread, this);
    tools::logger()->info("[heaven] successful received.");
}


Hseven::~Hseven()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

Eigen::Quaterniond Hseven::imu_at(std::chrono::steady_clock::time_point timestamp)
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

void Hseven::send(Command command)
{
  tx_data_.mode = command.control ? (command.shoot ? 2 : 1) : 0;
  tx_data_.yaw = command.yaw;
  tx_data_.pitch = command.pitch;
  tx_data_.crc16 = tools::get_crc16(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[heaven] Failed to write serial: {}", e.what());
  }
  
}


bool Hseven::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[heaven] Failed to read serial: {}", e.what());
    return false;
  }
}


void Hseven::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[heaven] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      tools::logger()->info("[heaven] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[heaven] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

void Hseven::read_thread()
{
  tools::logger()->info("[heaven] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[heaven] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rxdata_), sizeof(rxdata_.head))) {
      error_count++;
      continue;
    }

    if (rxdata_.head != 0xff) continue;


    if (!read(
          reinterpret_cast<uint8_t *>(&rxdata_) + sizeof(rxdata_.head),
          sizeof(rxdata_) - sizeof(rxdata_.head))) {
          error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rxdata_), sizeof(rxdata_))) {
      tools::logger()->debug("[heaven] CRC16 check failed.");
      continue;
    }

    error_count = 0;

    std::lock_guard<std::mutex> lock(mutex);
   
    auto timestamp = std::chrono::steady_clock::now();
    auto w = (int16_t)(rxdata_.q[0] << 8 | rxdata_.q[1]) / 1e3;
    auto x = (int16_t)(rxdata_.q[2] << 8 | rxdata_.q[3]) / 1e3;
    auto y = (int16_t)(rxdata_.q[4] << 8 | rxdata_.q[5]) / 1e3;
    auto z = (int16_t)(rxdata_.q[6] << 8 | rxdata_.q[7]) / 1e3;
 
    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      continue;
    }
    queue_.push({{w,x,y,z},timestamp});

    bullet_speed = rxdata_.bullet_speed;
    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[heaven] Bullet speed: {:.2f} m/s, Mode: {}",
        bullet_speed, MODES[mode]);
      last_log_time = now;
    }

    switch (rxdata_.mode) {
      case 0:
        mode = Mode::idle;
        break;
      case 1:
        mode = Mode::auto_aim;
        break;
      case 2:
        mode = Mode::small_buff;
        break;
      case 3:
        mode = Mode::big_buff;
        break;
      case 4:
        mode = Mode::outpost;
        break;
      default:
        mode = Mode::idle;
        tools::logger()->warn("[heaven] Invalid mode: {}", rxdata_.mode);
        break;
    }

  }

  tools::logger()->info("[heaven] read_thread stopped.");

}

}
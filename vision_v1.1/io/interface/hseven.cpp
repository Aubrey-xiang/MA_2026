#include "hseven.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <iomanip>
namespace io
{
Hseven::Hseven(const std::string & config_path) : mode(Mode::idle), bullet_speed(0)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "hseven_serial");
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
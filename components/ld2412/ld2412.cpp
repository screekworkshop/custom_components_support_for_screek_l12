#include "ld2412.h"

#define highbyte(val) (uint8_t)((val) >> 8)
#define lowbyte(val) (uint8_t)((val) & 0xff)

namespace esphome
{
  namespace ld2412
  {

    static const char *const TAG = "ld2412";

// 模拟激活编辑器
#ifdef __INTELLISENSE__
#define USE_SENSOR
#define USE_BINARY_SENSOR
#endif

#define SHOW_LOT_DEV_LOGS // 显示大量的开发日志

    void LD2412Component::dump_config()
    {
      ESP_LOGCONFIG(TAG, "LD2412:");
#ifdef USE_BINARY_SENSOR
      LOG_BINARY_SENSOR("  ", "HasTargetSensor", this->target_binary_sensor_);
      LOG_BINARY_SENSOR("  ", "MovingSensor", this->moving_binary_sensor_);
      LOG_BINARY_SENSOR("  ", "StillSensor", this->still_binary_sensor_);
#endif
#ifdef USE_SENSOR
      LOG_SENSOR("  ", "Moving Distance", this->moving_target_distance_sensor_);
      LOG_SENSOR("  ", "Still Distance", this->still_target_distance_sensor_);
      LOG_SENSOR("  ", "Moving Energy", this->moving_target_energy_sensor_);
      LOG_SENSOR("  ", "Still Energy", this->still_target_energy_sensor_);
      LOG_SENSOR("  ", "Detection Distance", this->detection_distance_sensor_);

      // 回显光线模块设置(23年3月13日_17时28分_)
      LOG_SENSOR("  ", "Light", this->light_sensor_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "VersionTextSensor", this->version_text_sensor_);
  LOG_TEXT_SENSOR("  ", "MacTextSensor", this->mac_text_sensor_);
#endif
      this->read_all_info();
      ESP_LOGCONFIG(TAG, "  MAC Address : %s", const_cast<char *>(this->mac_.c_str()));
      ESP_LOGCONFIG(TAG, "  Firmware Version : %s", const_cast<char *>(this->version_.c_str()));
    }

    void LD2412Component::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up LD2412...");
      // ESP_LOGCONFIG(TAG, "Apply screek patch...");
      // this->read_all_info();

      // 开启工程模式！(23年3月13日_18时03分_)
      // 因为会自动开启，这里就不太合适继续需要了。
      // this->factory_mode(true);

      // ESP_LOGCONFIG(TAG, "Mac Address : %s", const_cast<char *>(this->mac_.c_str()));
      // ESP_LOGCONFIG(TAG, "Firmware Version : %s", const_cast<char *>(this->version_.c_str()));
      ESP_LOGCONFIG(TAG, "LD2412 setup complete.");
    }

  void LD2412Component::read_all_info() {
    this->set_config_mode_(true);
    delay(10); // 加入延时，保证工作可以稳定
    this->get_version_();
    delay(10); // 加入延时，保证工作可以稳定
    this->get_mac_();
    //this->get_distance_resolution_();
    //this->query_parameters_();
    delay(10);
    this->set_config_mode_(false);
  }

  void LD2412Component::restart_and_read_all_info() {
    this->reboot();
    this->set_timeout(1000, [this]() { this->read_all_info(); });
  }


    void LD2412Component::loop()
    {
      const int max_line_length = 80;
      static uint8_t buffer[max_line_length];

      while (available())
      {
        this->readline_(read(), buffer, max_line_length);
      }
    }

    // 发送命令给雷达！(23年3月13日_14时46分_)
    void LD2412Component::send_command_(uint8_t command, uint8_t *command_value, int command_value_len)
    {
      // lastCommandSuccess->publish_state(false);

      // frame start bytes
      this->write_array(CMD_FRAME_HEADER, 4);
      // length bytes
      int len = 2;
      if (command_value != nullptr)
        len += command_value_len;
      this->write_byte(lowbyte(len));
      this->write_byte(highbyte(len));

      // command
      this->write_byte(lowbyte(command));
      this->write_byte(highbyte(command));

      // command value bytes
      if (command_value != nullptr)
      {
        for (int i = 0; i < command_value_len; i++)
        {
          this->write_byte(command_value[i]);
        }
      }
      // frame end bytes
      this->write_array(CMD_FRAME_END, 4);

      // 发送出去？
      // this->write_str("\n");

      // this->flush();
      //  FIXME to remove
      delay(50); // NOLINT, 默认50ms。
    }

    /*
    // 引入Arduino的Map函数，重新映射数值。(23年3月14日_16时34分_)
    // https://cdn.arduino.cc/reference/en/language/functions/math/map/
    long map(long x, long in_min, long in_max, long out_min, long out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    */

    void LD2412Component::readline_(int readch, uint8_t *buffer, int len)
    {
      static int pos = 0;

      if (readch >= 0)
      {
        if (pos < len - 1)
        {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
        else
        {
          pos = 0;
        }

        if (pos >= 4)
        {
          if (buffer[pos - 4] == 0xF8 && buffer[pos - 3] == 0xF7 && buffer[pos - 2] == 0xF6 && buffer[pos - 1] == 0xF5)
          {

            // DEBUG: 打印缓冲区的所有内容
            // char log_buffer[256];
            // memset(log_buffer, 0, sizeof(log_buffer));
            // for (int i = 0; i < pos; ++i) {
            //   char byte_str[4]; // 每个字节最多需要3个字符 + 空字符
            //   snprintf(byte_str, sizeof(byte_str), "%02X ", buffer[i]);
            //   strcat(log_buffer, byte_str);
            // }
            // ESP_LOGV(TAG, "缓冲区内容: %s", log_buffer);

            ESP_LOGV(TAG, "Will handle Periodic Data");
            this->handle_periodic_data_(buffer, pos);
            pos = 0; // Reset position index ready for next time
          }
          else if (buffer[pos - 4] == 0x04 && buffer[pos - 3] == 0x03 && buffer[pos - 2] == 0x02 &&
                   buffer[pos - 1] == 0x01)
          {
            if (this->handle_ack_data_(buffer, pos)) {
                  pos = 0;  // Reset position index ready for next time
                } else {
                  ESP_LOGV(TAG, "ACK Data incomplete");
                }
              }
            }
      }
    }

    void LD2412Component::handle_periodic_data_(uint8_t *buffer, int len)
    {
      char target_state1 = buffer[TARGET_STATES];

#ifdef SHOW_LOT_DEV_LOGS
      // 打印目标状态的十六进制值
      // ESP_LOGI("LD2412", "Target State: 0x%02X", target_state1);
      // 检查目标状态是否等于 0x00 或 0x01
      if (target_state1 == 0x03)
      {
        ESP_LOGI("LD2412", "复合状态数据到来！");
      }
#endif

      // 最小需要12字节！(23年3月13日_16时53分_)
      if (len < 12)
      {
#ifdef SHOW_LOT_DEV_LOGS
        ESP_LOGI("LD2412", "长度丢失");
#endif

        return; // 4 frame start bytes + 2 length bytes + 1 data end byte + 1 crc byte + 4 frame end bytes
      }

      // 效验开头符合要求。(23年3月13日_16时54分_)
      if (buffer[0] != 0xF4 || buffer[1] != 0xF3 || buffer[2] != 0xF2 || buffer[3] != 0xF1)
      { // check 4 frame start bytes
        return;
      }

      if (buffer[7] != HEAD)
      { // Check data head
        return;
      }

      if (buffer[len - 6] != END)
      { // Check data end
        return;
      }

      if (buffer[len - 5] != CHECK)
      {
        if (target_state1 != 0x03)
        {
          // 实际上是速度显示了。
#ifdef SHOW_LOT_DEV_LOGS
          ESP_LOGI("LD2412", "CRC检查错误，接收到的CRC值是: 0x%02X", (unsigned int)buffer[len - 5]);
#endif

          return; // 复合运动状态会出错
        }
#ifdef SHOW_LOT_DEV_LOGS
        ESP_LOGI("LD2412", "混合模式忽略CRC效验。");
#endif
      }
      /*
        Data Type: 6th
        0x01: Engineering mode
        0x02: Normal mode
      */
      // char data_type = buffer[DATA_TYPES];
      /*
        Target states: 9th
        0x00 = No target
        0x01 = Moving targets
        0x02 = Still targets
        0x03 = Moving+Still targets
      */

      char data_type = buffer[DATA_TYPES];

      // DEBUG：打印数据类型
      //  ESP_LOGI("LD2412", "Data Type: 0x%02X", data_type);
      //  switch (data_type) {
      //      case 0x01:
      //          ESP_LOGI("LD2412", "Data Type: Engineering Mode (0x01)");
      //          break;
      //      case 0x02:
      //          ESP_LOGI("LD2412", "Data Type: Normal Mode (0x02)");
      //          break;
      //      default:
      //          ESP_LOGI("LD2412", "Data Type: Unknown (0x%02X)", data_type);
      //          break;
      //  }

#ifdef USE_BINARY_SENSOR
      char target_state = buffer[TARGET_STATES];

      if (this->target_binary_sensor_ != nullptr)
      {
        this->target_binary_sensor_->publish_state(target_state != 0x00);
      }
#endif

      /*
        Reduce data update rate to prevent home assistant database size grow fast
      */

      // 这里优化了HA的上报间隔，未来的话，这里看起来可以单独设置呢！(23年3月13日_16时48分_)
      uint32_t current_millis = millis();
      // if (current_millis - last_periodic_millis < 500) // 这里可以加速显示的
      //   return;
      // last_periodic_millis = current_millis;

#ifdef USE_SENSOR

      /* 读取光感尝试(23年3月13日_16时57分_) */
      // 仅仅开启了光感这个设置才启用，否则就是不启用的。(23年3月13日_21时16分_)
      if (this->light_sensor_ != nullptr)
      {
        // 需要有传感器的时候才发射，不然很麻烦。(23年3月13日_17时47分_)
        int new_light = -1;

        if (data_type == 0x01)
        {
          if (len > 45)
          {
            new_light = buffer[45];
            // ESP_LOGD(TAG, "LD2412 luminance: %d%", new_light);
          }
        }
        else
        {
          uint32_t now_millis = millis();
          // TODO: 留下一个设置在里面。(23年3月13日_18时09分_)
          /// 2秒才能改变一次啊！(23年3月13日_18时07分_)
          if (now_millis - last_change_fatory_mode_millis > 2000)
          {
            ESP_LOGD(TAG, "Normal mode no light, change to factory mode");
            // 重置工程模式。(23年3月13日_18时08分_)

            // TODO: 等待恢复这里
            this->factory_mode(true);

            // this -> set_config_mode_(true);

            last_change_fatory_mode_millis = now_millis;
          }
        }
        uint32_t current_light_millis = millis(); // 获取当前时间（毫秒）
        if (current_light_millis - last_update_light_millis >= 1000)
        {
          if (this->light_sensor_->get_state() != new_light)
          {
            this->light_sensor_->publish_state(new_light);
          }
          last_update_light_millis = current_light_millis;
        }
      }
#endif

      // 剩下的需要在基本模式里完成
      if (data_type != 0x02 && data_type != 0x01)
      { // 非基本模式，请离开。
        return;
      }

#ifdef USE_BINARY_SENSOR
      if (this->moving_binary_sensor_ != nullptr)
      {
        this->moving_binary_sensor_->publish_state(CHECK_BIT(target_state, 0));
      }
      if (this->still_binary_sensor_ != nullptr)
      {
        this->still_binary_sensor_->publish_state(CHECK_BIT(target_state, 1));
      }
#endif
      /*
        Moving target distance: 10~11th bytes
        Moving target energy: 12th byte
        Still target distance: 13~14th bytes
        Still target energy: 15th byte
      */

      // for (int i = 0; i < 20; ++i) {
      //   ESP_LOGD("LD2412", "位置 %d: 0x%02X", i + 1, buffer[i]);
      // }

#ifdef USE_SENSOR
      if (this->moving_target_distance_sensor_ != nullptr)
      {
        int new_moving_target_distance = this->two_byte_to_int_(buffer[MOVING_TARGET_LOW], buffer[MOVING_TARGET_HIGH]);
        if (this->moving_target_distance_sensor_->get_state() != new_moving_target_distance)
          this->moving_target_distance_sensor_->publish_state(new_moving_target_distance);
      }
      if (this->moving_target_energy_sensor_ != nullptr)
      {
        int new_moving_target_energy = buffer[MOVING_ENERGY];
        if (this->moving_target_energy_sensor_->get_state() != new_moving_target_energy)
          this->moving_target_energy_sensor_->publish_state(new_moving_target_energy);
      }
      if (this->still_target_distance_sensor_ != nullptr)
      {
        int new_still_target_distance = this->two_byte_to_int_(buffer[STILL_TARGET_LOW], buffer[STILL_TARGET_HIGH]);
        if (this->still_target_distance_sensor_->get_state() != new_still_target_distance)
          this->still_target_distance_sensor_->publish_state(new_still_target_distance);
      }
      if (this->still_target_energy_sensor_ != nullptr)
      {
        int new_still_target_energy = buffer[STILL_ENERGY];
        if (this->still_target_energy_sensor_->get_state() != new_still_target_energy)
          this->still_target_energy_sensor_->publish_state(new_still_target_energy);
      }

#endif
    }

    const char VERSION_FMT[] = "%u.%02X.%02X%02X%02X%02X";

    std::string format_version(uint8_t *buffer) {
      std::string::size_type version_size = 256;
      std::string version;
      do {
        version.resize(version_size + 1);
        version_size = std::snprintf(&version[0], version.size(), VERSION_FMT, buffer[13], buffer[12], buffer[17],
                                    buffer[16], buffer[15], buffer[14]);
      } while (version_size + 1 > version.size());
      version.resize(version_size);
      return version;
    }

    const char MAC_FMT[] = "%02X:%02X:%02X:%02X:%02X:%02X";

    const std::string UNKNOWN_MAC("unknown");
    const std::string NO_MAC("08:05:04:03:02:01");

    std::string format_mac(uint8_t *buffer) {
      std::string::size_type mac_size = 256;
      std::string mac;
      do {
        mac.resize(mac_size + 1);
        mac_size = std::snprintf(&mac[0], mac.size(), MAC_FMT, buffer[10], buffer[11], buffer[12], buffer[13], buffer[14],
                                buffer[15]);
      } while (mac_size + 1 > mac.size());
      mac.resize(mac_size);
      if (mac == NO_MAC) {
        return UNKNOWN_MAC;
      }
      return mac;
    }

    bool LD2412Component::handle_ack_data_(uint8_t *buffer, int len)
    {
      
#ifdef SHOW_LOT_DEV_LOGS
      // 打印返回的 buffer 内容
      std::string buffer_str;
      for (int i = 0; i < len; i++)
      {
        char hex_str[4];
        sprintf(hex_str, "%02X", buffer[i]);
        buffer_str += hex_str;
        if (i < len - 1)
        {
          buffer_str += ":";
        }
      }
      ESP_LOGV(TAG, "Buffer: %s", buffer_str.c_str());
#endif

      if (len < 10)
      {
        ESP_LOGE(TAG, "Error with last command : incorrect length");
        return true;
      }

      ESP_LOGV(TAG, "Handling ACK DATA for COMMAND %02X", buffer[COMMAND]);

      if (buffer[0] != 0xFD || buffer[1] != 0xFC || buffer[2] != 0xFB || buffer[3] != 0xFA)
      { // check 4 frame start bytes
        ESP_LOGE(TAG, "Error with last command : incorrect Header");
        return true;
      }
      if (buffer[COMMAND_STATUS] != 0x01)
      {
        ESP_LOGE(TAG, "Error with last command : status != 0x01");
        return true;
      }
      if (this->two_byte_to_int_(buffer[8], buffer[9]) != 0x00)
      {
        ESP_LOGE(TAG, "Error with last command , last buffer was: %u , %u", buffer[8], buffer[9]);
        return true;
      }

      switch (buffer[COMMAND])
      {
      case lowbyte(CMD_ENABLE_CONF):
        ESP_LOGV(TAG, "Handled Enable conf command");
        break;
      case lowbyte(CMD_DISABLE_CONF):
        ESP_LOGV(TAG, "Handled Disabled conf command");
        break;
      case lowbyte(CMD_VERSION):
        this->version_ = format_version(buffer);
        ESP_LOGV(TAG, "FW Version is: %s", const_cast<char *>(this->version_.c_str()));
  #ifdef USE_TEXT_SENSOR
        if (this->version_text_sensor_ != nullptr) {
          if (this->version_text_sensor_->get_state() != this->version_){
            // 检查不一样再发布！
            this->version_text_sensor_->publish_state(this->version_);
          }
        }
  #endif
      case lowbyte(CMD_MAC):
      if (len < 20) {
        return false;
      }
      this->mac_ = format_mac(buffer);
      ESP_LOGV(TAG, "MAC Address is: %s", const_cast<char *>(this->mac_.c_str()));
#ifdef USE_TEXT_SENSOR
        if (this->mac_text_sensor_ != nullptr) {
          if (this->mac_text_sensor_->get_state() != this->mac_){
            // 检查不一样再发布！
            this->mac_text_sensor_->publish_state(this->mac_);
          }
        }
#endif
      case lowbyte(CMD_GATE_SENS):
        ESP_LOGV(TAG, "Handled sensitivity command");
        break;
      case lowbyte(CMD_QUERY): // Query parameters response
      {
        if (buffer[10] != 0xAA)
          return true; // value head=0xAA
                  /*
                    Moving distance range: 13th byte
                    Still distance range: 14th byte
                  */
                  // TODO
                  // maxMovingDistanceRange->publish_state(buffer[12]);
                  // maxStillDistanceRange->publish_state(buffer[13]);
                  /*
                    Moving Sensitivities: 15~23th bytes
                    Still Sensitivities: 24~32th bytes
                  */

        // 显示参数设置在这里！

        // for (int i = 0; i < 9; i++) {
        //   moving_sensitivities[i] = buffer[14 + i];
        // }
        // for (int i = 0; i < 9; i++) {
        //   still_sensitivities[i] = buffer[23 + i];
        // }
        /*
          None Duration: 33~34th bytes
        */
        // noneDuration->publish_state(this->two_byte_to_int_(buffer[32], buffer[33]));
      }
      break;
      default:
        break;
      }

      return true;
    }

    void LD2412Component::set_config_mode_(bool enable)
    {
      uint8_t cmd = enable ? CMD_ENABLE_CONF : CMD_DISABLE_CONF;
      uint8_t cmd_value[2] = {0x01, 0x00};
      this->send_command_(cmd, enable ? cmd_value : nullptr, 2);
    }

    // 额外增加的命令到这里去(23年3月13日_14时57分_)
    void LD2412Component::factoryReset()
    {
      this->set_config_mode_(true);

      uint8_t cmd = 0x00A2;
      this->send_command_(cmd, nullptr, 0);
    }

    void LD2412Component::reboot()
    {
      ESP_LOGD(TAG, "reboot ld2412b...");
      this->set_config_mode_(true);
      this->send_command_(CMD_RESTART, nullptr, 0);
      // not need to exit config mode because the ld2412 will reboot automatically
    }

    void LD2412Component::query_parameters_() { this->send_command_(CMD_QUERY, nullptr, 0); }
    void LD2412Component::get_version_() { this->send_command_(CMD_VERSION, nullptr, 0); }
    void LD2412Component::get_mac_() {
      uint8_t cmd_value[2] = {0x01, 0x00};
      this->send_command_(CMD_MAC, cmd_value, 2);
    }
    void LD2412Component::get_distance_resolution_() { this->send_command_(CMD_QUERY_DISTANCE_RESOLUTION, nullptr, 0); }


    void LD2412Component::ble_control(bool enable)
    {
      // 启用配置模式(23年3月13日_15时24分_)
      this->set_config_mode_(true);
      uint8_t CMD_BLE_CONF = 0x00A4;

      uint8_t cmd_value[2] = {0x00, 0x00};

      if (enable)
      {
        cmd_value[0] = 0x01;

        ESP_LOGD(TAG, "Enable BLE...");
      }
      else
      {
        ESP_LOGD(TAG, "Disable BLE...");
      }

      // TODO: 获取返回结果？ack？(23年3月13日_15时26分_)
      // 如果不指定最后的字节长度，就会命令无效。(23年3月13日_16时02分_)
      this->send_command_(CMD_BLE_CONF, cmd_value, 2);

      this->set_config_mode_(false);

      this->reboot();
    }

    /// @brief 工厂模式调节
    /// @param enable 开启与否
    void LD2412Component::factory_mode(bool enable)
    {
      uint8_t CMD_FACTORY_ON = 0x0062;
      uint8_t CMD_FACTORY_OFF = 0x0063;
      this->set_config_mode_(true);

      uint8_t cmd = enable ? CMD_FACTORY_ON : CMD_FACTORY_OFF;

      this->send_command_(cmd, nullptr, 0);

      this->set_config_mode_(false);
    }

  } // namespace ld2412
} // namespace esphome

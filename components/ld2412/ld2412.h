#pragma once
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

namespace esphome
{
  namespace ld2412
  {

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

    // Commands
    static const uint8_t CMD_ENABLE_CONF = 0x00FF;
    static const uint8_t CMD_DISABLE_CONF = 0x00FE;
    static const uint8_t CMD_MAXDIST_DURATION = 0x0060;
    static const uint8_t CMD_QUERY = 0x0061;
    static const uint8_t CMD_GATE_SENS = 0x0064;
    static const uint8_t CMD_VERSION = 0x00A0;
    static const uint8_t CMD_MAC = 0x00A5;
    static const uint8_t CMD_RESET = 0x00A2;
    static const uint8_t CMD_RESTART = 0x00A3;
    static const uint8_t CMD_QUERY_DISTANCE_RESOLUTION = 0x00AB;

    // Commands values
    static const uint8_t CMD_MAX_MOVE_VALUE = 0x0000;
    static const uint8_t CMD_MAX_STILL_VALUE = 0x0001;
    static const uint8_t CMD_DURATION_VALUE = 0x0002;
    // Command Header & Footer
    static const uint8_t CMD_FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
    static const uint8_t CMD_FRAME_END[4] = {0x04, 0x03, 0x02, 0x01};
    // Data Header & Footer
    static const uint8_t DATA_FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
    static const uint8_t DATA_FRAME_END[4] = {0xF8, 0xF7, 0xF6, 0xF5};
    /*
    Data Type: 6th byte
    Target states: 9th byte
        Moving target distance: 10~11th bytes
        Moving target energy: 12th byte
        Still target distance: 13~14th bytes
        Still target energy: 15th byte
        Detect distance: 16~17th bytes
    */
    enum PeriodicDataStructure : uint8_t
    {
      DATA_TYPES = 6,
      TARGET_STATES = 8,
      MOVING_TARGET_LOW = 9,
      MOVING_TARGET_HIGH = 10,
      MOVING_ENERGY = 11,
      STILL_TARGET_LOW = 12,
      STILL_TARGET_HIGH = 13,
      STILL_ENERGY = 14,
    };
    // 上报数据的固定结构。(23年3月13日_16时54分_)
    enum PeriodicDataValue : uint8_t
    {
      HEAD = 0XAA,
      END = 0x55,
      CHECK = 0x00
    };

    enum AckDataStructure : uint8_t
    {
      COMMAND = 6,
      COMMAND_STATUS = 7
    };

    //  char cmd[2] = {enable ? 0xFF : 0xFE, 0x00};
    class LD2412Component : public Component, public uart::UARTDevice
    {
#ifdef USE_SENSOR
      SUB_SENSOR(moving_target_distance)
      SUB_SENSOR(still_target_distance)
      SUB_SENSOR(moving_target_energy)
      SUB_SENSOR(still_target_energy)
      SUB_SENSOR(detection_distance)
      SUB_SENSOR(light)
#endif
#ifdef USE_TEXT_SENSOR
      SUB_TEXT_SENSOR(version)
      SUB_TEXT_SENSOR(mac)
#endif
    public:
      void setup() override;
      void dump_config() override;
      void loop() override;

#ifdef USE_BINARY_SENSOR
      void set_target_sensor(binary_sensor::BinarySensor *sens) { this->target_binary_sensor_ = sens; };
      void set_moving_target_sensor(binary_sensor::BinarySensor *sens) { this->moving_binary_sensor_ = sens; };
      void set_still_target_sensor(binary_sensor::BinarySensor *sens) { this->still_binary_sensor_ = sens; };
#endif

      // 从新的LD2410库里引入。(24年8月11日_21时33分_)
      void read_all_info();
      void restart_and_read_all_info();

      /// @brief 上次改变工程模式的时间，用来防止改变频率太高了。(23年3月13日_18时07分_)
      uint32_t last_change_fatory_mode_millis = 0;

      // 增加一些额外的命令，比如重启，开关蓝牙。(23年3月13日_14时44分_)
      // https://github.com/rain931215/ESPHome-LD2412/blob/main/ld2412_uart.h

      /// 恢复出厂设置！(23年3月13日_14时48分_)
      void factoryReset();

      // 重启(23年3月13日_14时48分_)
      void reboot();

      /// @brief  蓝牙开关
      /// (23年3月13日_15时22分_)
      void ble_control(bool enable);

      /// @brief 开关工程模式
      /// @param enable 开关
      /// 用来快速切换工程模式用的。(23年3月13日_17时59分_)
      void factory_mode(bool enable);

    protected:
#ifdef USE_BINARY_SENSOR
      binary_sensor::BinarySensor *target_binary_sensor_{nullptr};
      binary_sensor::BinarySensor *moving_binary_sensor_{nullptr};
      binary_sensor::BinarySensor *still_binary_sensor_{nullptr};
#endif
      std::vector<uint8_t> rx_buffer_;
      int two_byte_to_int_(char firstbyte, char secondbyte) { return (int16_t)(secondbyte << 8) + firstbyte; }
      void send_command_(uint8_t command_str, uint8_t *command_value, int command_value_len);

      void set_config_mode_(bool enable);
      void handle_periodic_data_(uint8_t *buffer, int len);
      bool handle_ack_data_(uint8_t *buffer, int len);
      void readline_(int readch, uint8_t *buffer, int len);
      void query_parameters_();
      void get_version_();
      void get_mac_();
      void get_distance_resolution_();

      uint32_t last_periodic_millis = millis();
      uint32_t last_update_light_millis = millis();
      std::string version_;
      std::string mac_;
    };

  } // namespace ld2412
} // namespace esphome

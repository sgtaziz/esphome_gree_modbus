/**
 * Gree AC Modbus RTU ESPHome Component
 *
 * Controls Gree commercial AC units (U-Match, GFH series) via RS485 Modbus RTU
 *
 * Tested with: Gree GFH36K3FI / GUHD36NK3FO
 *
 * Registers (from TapHome template + discoveries):
 *   2  - On/Off (85=Off, 170=On)
 *   3  - Current temperature from wired controller × 10 (used when reg 39 = 3)
 *   4  - Ambient temperature × 10 (IDU return air sensor)
 *   5  - IDU Address
 *   17 - Mode (1=Cool, 2=Heat, 3=Dry, 4=Fan, 5=Auto)
 *   19 - Fan Speed (0=Auto, 1-5=Speed, 6=Turbo)
 *   20 - Set Temperature (whole degrees)
 *   22 - Vertical Swing (Up/Down)
 *   23 - Horizontal Swing (Left/Right)
 *   24 - Fresh air valve status
 *   25 - Sleep mode
 *   34 - Contamination grade
 *   39 - Ambient temp sensor selection
 *   42 - Set Temperature × 10 (supports 0.5°C precision)
 *   49 - Outdoor temperature
 *   77 - DRED function
 *   82 - Ambient temp at return air port × 10
 *   83 - Ambient temp of light board × 10
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/button/button.h"
#include <array>
#include <vector>

namespace esphome {
namespace gree_ac {

static const char *const TAG = "gree_ac";
static const char *const VERSION = "1.0.0";

// Modbus settings
static const uint32_t MODBUS_BAUD = 9600;
static const uint8_t MODBUS_DEFAULT_SLAVE_ID = 1;

// Temperature limits (from XML)
static const float MIN_TEMPERATURE = 16.0f;
static const float MAX_TEMPERATURE = 30.0f;
static const float TEMPERATURE_STEP = 1.0f;

// Modbus register addresses
namespace registers {
  static const uint16_t ON_OFF = 2;
  static const uint16_t TEMP_SENSOR_3 = 3;
  static const uint16_t AMBIENT_TEMP = 4;
  static const uint16_t IDU_ADDRESS = 5;
  static const uint16_t MODE = 17;
  static const uint16_t FAN_SPEED = 19;
  static const uint16_t SET_TEMP = 20;
  static const uint16_t VERTICAL_SWING = 22;
  static const uint16_t HORIZONTAL_SWING = 23;
  static const uint16_t FRESH_AIR_VALVE = 24;
  static const uint16_t SLEEP_MODE = 25;
  static const uint16_t CONTAMINATION = 34;
  static const uint16_t TEMP_SENSOR_SELECT = 39;
  static const uint16_t SET_TEMP_PRECISE = 42;
  static const uint16_t OUTDOOR_TEMP = 49;
  static const uint16_t DRED_FUNCTION = 77;
  static const uint16_t AMBIENT_RETURN_AIR = 82;
  static const uint16_t AMBIENT_LIGHT_BOARD = 83;

  // Debug register sweep range (inclusive). Cache has MAX- MIN + 1 slots.
  static const uint16_t MIN_DEBUG_ADDR = 0;
  static const uint16_t MAX_DEBUG_ADDR = 92;
  static const uint16_t DEBUG_REG_COUNT = MAX_DEBUG_ADDR - MIN_DEBUG_ADDR + 1;
}

// On/Off values
static const uint16_t AC_ON = 170;   // 0xAA
static const uint16_t AC_OFF = 85;   // 0x55

// Mode values
namespace modes {
  static const uint16_t COOL = 1;
  static const uint16_t HEAT = 2;
  static const uint16_t DRY = 3;
  static const uint16_t FAN_ONLY = 4;
  static const uint16_t AUTO = 5;
}

// Fan speed values
namespace fan_speeds {
  static const uint16_t AUTO = 0;
  static const uint16_t SPEED_1 = 1;
  static const uint16_t SPEED_2 = 2;
  static const uint16_t SPEED_3 = 3;
  static const uint16_t SPEED_4 = 4;
  static const uint16_t SPEED_5 = 5;
  static const uint16_t TURBO = 6;
}

// Selectable sources for the climate's current_temperature value
enum class CurrentTempSource : uint8_t {
  WIRED_CONTROLLER = 0,  // reg 3 (TEMP_SENSOR_3), ÷10, signed
  IDU_RETURN_AIR,        // reg 4 (AMBIENT_TEMP), ÷1
  RETURN_AIR_PORT,       // reg 82 (AMBIENT_RETURN_AIR), ÷10, signed
  LIGHT_BOARD,           // reg 83 (AMBIENT_LIGHT_BOARD), ÷10, signed
};

// Options for the current-temp source select (index matches CurrentTempSource enum)
static const std::vector<std::string> CURRENT_TEMP_SOURCE_OPTIONS = {
    "Wired Controller",
    "IDU Return Air",
    "Return Air Port",
    "Light Board",
};

// Vertical swing options (from XML)
static const std::vector<std::string> VERTICAL_SWING_OPTIONS = {
  "Off",
  "Full Swing",
  "Position 1 (Up)",
  "Position 2",
  "Position 3 (Middle)",
  "Position 4",
  "Position 5 (Down)",
  "Swing Upper",
  "Swing Middle",
  "Swing Lower",
  "Swing Upper-Middle",
  "Swing Middle-Lower"
};

// Horizontal swing options (from XML)
static const std::vector<std::string> HORIZONTAL_SWING_OPTIONS = {
  "Off",
  "Full Swing",
  "Position 1 (Left)",
  "Position 2",
  "Position 3 (Middle)",
  "Position 4",
  "Position 5 (Right)",
  "Swing Left",
  "Swing Right"
};

class GreeAC : public Component, public uart::UARTDevice, public climate::Climate {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration setters
  void set_slave_id(uint8_t slave_id) { this->slave_id_ = slave_id; }
  void set_update_interval(uint32_t interval) { this->update_interval_ = interval; }
  void set_flow_control_pin(GPIOPin *pin) { this->flow_control_pin_ = pin; }

  // Optional component setters (implementations in cpp with callbacks)
  void set_outdoor_temperature_sensor(sensor::Sensor *sensor) { this->outdoor_temp_sensor_ = sensor; }
  void set_vertical_swing_select(select::Select *select);
  void set_horizontal_swing_select(select::Select *select);
  void set_sleep_switch(switch_::Switch *sw);
  void set_turbo_switch(switch_::Switch *sw);
  void set_fresh_air_switch(switch_::Switch *sw);
  void set_current_temp_source_select(select::Select *select);

  // Feature flags
  void set_expose_sensors(bool v) { this->expose_sensors_ = v; }
  void set_debug_mode(bool v) { this->debug_mode_ = v; }

  // Exposed-sensor setters (each optional)
  void set_set_point_sensor(sensor::Sensor *s) { this->set_point_sensor_ = s; }
  void set_current_temp_sensor(sensor::Sensor *s) { this->current_temp_sensor_ = s; }
  void set_mode_sensor(sensor::Sensor *s) { this->mode_sensor_ = s; }
  void set_fan_speed_sensor(sensor::Sensor *s) { this->fan_speed_sensor_ = s; }
  void set_on_off_sensor(sensor::Sensor *s) { this->on_off_sensor_ = s; }
  void set_sleep_sensor(sensor::Sensor *s) { this->sleep_sensor_ = s; }
  void set_turbo_sensor(sensor::Sensor *s) { this->turbo_sensor_ = s; }
  void set_fresh_air_sensor(sensor::Sensor *s) { this->fresh_air_sensor_ = s; }
  void set_contamination_sensor(sensor::Sensor *s) { this->contamination_sensor_ = s; }
  void set_set_temp_precise_sensor(sensor::Sensor *s) { this->set_temp_precise_sensor_ = s; }
  void set_ambient_return_air_sensor(sensor::Sensor *s) { this->ambient_return_air_sensor_ = s; }
  void set_ambient_light_board_sensor(sensor::Sensor *s) { this->ambient_light_board_sensor_ = s; }

  // Debug entity setters
  void set_debug_text_sensor(text_sensor::TextSensor *s) { this->debug_text_sensor_ = s; }
  void set_debug_address_number(number::Number *n);
  void set_debug_value_number(number::Number *n);
  void set_debug_write_button(button::Button *b);

  // Modbus communication
  bool read_register(uint16_t reg_addr, uint16_t *value);
  bool write_register(uint16_t reg_addr, uint16_t value);

  // Control methods (called by switches/selects)
  void set_vertical_swing(uint16_t value);
  void set_horizontal_swing(uint16_t value);
  void set_sleep_mode(bool enabled);
  void set_turbo_mode(bool enabled);
  void set_fresh_air(bool enabled);

 protected:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // Modbus helpers
  uint16_t calculate_crc(uint8_t *data, uint8_t len);
  bool send_modbus_request(uint8_t *request, uint8_t req_len, uint8_t *response, uint8_t *resp_len);

  // State update
  void update_state();
  void read_all_registers();
  void read_next_register();  // Non-blocking: reads one register per call (from read_plan_)
  void build_read_plan();     // Assembles read_plan_ based on enabled features

  // Async Modbus methods
  void send_read_request(uint16_t reg_addr);  // Non-blocking: sends request
  void check_response();  // Non-blocking: checks for response data
  void advance_read_plan();  // Wraps read_plan_index_ and publishes on cycle wrap
  void process_register_response(uint16_t reg_addr, uint16_t value);  // Handles received data

  // Register cache helpers (indexed by register address)
  void store_register(uint16_t addr, uint16_t value);
  float compute_current_temp_from_cache() const;
  void recompute_current_temp();
  void publish_exposed_sensors();
  void publish_debug_dump();
  void handle_debug_write();

  // Configuration
  uint8_t slave_id_{MODBUS_DEFAULT_SLAVE_ID};
  uint32_t update_interval_{5000};  // 5 seconds default
  GPIOPin *flow_control_pin_{nullptr};  // DE/RE pin for MAX485 modules
  bool expose_sensors_{false};
  bool debug_mode_{false};

  // Timing
  uint32_t last_update_{0};
  uint32_t last_request_{0};

  // Dynamic read plan (replaces the old fixed 10-index switch)
  std::vector<uint16_t> read_plan_;
  size_t read_plan_index_{0};

  // Async Modbus state machine
  enum class ModbusState : uint8_t {
    IDLE,
    WAITING_RESPONSE,
  };
  ModbusState modbus_state_{ModbusState::IDLE};
  uint32_t request_start_time_{0};
  uint8_t response_buffer_[64];
  uint8_t response_index_{0};
  uint32_t last_byte_time_{0};
  uint16_t pending_register_{0};  // Register we're waiting for response from

  // Optional sensors
  sensor::Sensor *outdoor_temp_sensor_{nullptr};

  // Exposed sensors (gated by expose_sensors_)
  sensor::Sensor *set_point_sensor_{nullptr};
  sensor::Sensor *current_temp_sensor_{nullptr};
  sensor::Sensor *mode_sensor_{nullptr};
  sensor::Sensor *fan_speed_sensor_{nullptr};
  sensor::Sensor *on_off_sensor_{nullptr};
  sensor::Sensor *sleep_sensor_{nullptr};
  sensor::Sensor *turbo_sensor_{nullptr};
  sensor::Sensor *fresh_air_sensor_{nullptr};
  sensor::Sensor *contamination_sensor_{nullptr};
  sensor::Sensor *set_temp_precise_sensor_{nullptr};
  sensor::Sensor *ambient_return_air_sensor_{nullptr};
  sensor::Sensor *ambient_light_board_sensor_{nullptr};

  // Optional selects
  select::Select *vertical_swing_select_{nullptr};
  select::Select *horizontal_swing_select_{nullptr};
  select::Select *current_temp_source_select_{nullptr};

  // Optional switches
  switch_::Switch *sleep_switch_{nullptr};
  switch_::Switch *turbo_switch_{nullptr};
  switch_::Switch *fresh_air_switch_{nullptr};

  // Debug entities
  text_sensor::TextSensor *debug_text_sensor_{nullptr};
  number::Number *debug_address_number_{nullptr};
  number::Number *debug_value_number_{nullptr};
  button::Button *debug_write_button_{nullptr};

  // Raw register cache (indexed by register address, 0..MAX_DEBUG_ADDR)
  std::array<uint16_t, registers::DEBUG_REG_COUNT> register_cache_{};
  std::array<bool, registers::DEBUG_REG_COUNT> register_valid_{};  // false until first successful read

  // Cached state
  bool ac_on_{false};
  uint16_t mode_{modes::AUTO};
  uint16_t fan_speed_{fan_speeds::AUTO};
  float target_temp_{24.0f};
  float current_temp_{0.0f};
  float outdoor_temp_{0.0f};
  uint16_t vertical_swing_{0};
  uint16_t horizontal_swing_{0};
  bool sleep_mode_{false};
  bool turbo_mode_{false};
  bool fresh_air_{false};
  CurrentTempSource current_temp_source_{CurrentTempSource::WIRED_CONTROLLER};
};

}  // namespace gree_ac
}  // namespace esphome

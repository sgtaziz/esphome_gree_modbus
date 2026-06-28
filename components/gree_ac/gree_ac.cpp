/**
 * Gree AC Modbus RTU ESPHome Component - Implementation
 */

#include "gree_ac.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>
#include <cstdio>
#include <set>

namespace esphome {
namespace gree_ac {

// CRC-16 Modbus lookup table
static const uint16_t CRC_TABLE[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t GreeAC::calculate_crc(uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc = (crc >> 8) ^ CRC_TABLE[(crc ^ data[i]) & 0xFF];
  }
  return crc;
}

void GreeAC::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Gree AC Modbus...");

  // Initialize flow control pin (DE/RE for MAX485 modules)
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
    this->flow_control_pin_->digital_write(false);  // Start in receive mode
  }

  // Assemble the register read plan based on which features are enabled.
  this->build_read_plan();

  // Don't read registers here - it blocks too long and triggers watchdog
  // First read will happen in loop() after setup completes
}

void GreeAC::build_read_plan() {
  // Front-load the core control registers, then add feature-driven ones.
  // Dedupe via a set so registers shared by multiple features are read once.
  std::vector<uint16_t> plan;
  std::set<uint16_t> seen;

  auto add = [&](uint16_t reg) {
    if (seen.insert(reg).second)
      plan.push_back(reg);
  };

  // Core control registers (always polled, in priority order)
  add(registers::ON_OFF);
  add(registers::MODE);
  add(registers::TEMP_SENSOR_3);
  add(registers::SET_TEMP);
  add(registers::FAN_SPEED);
  add(registers::VERTICAL_SWING);
  add(registers::HORIZONTAL_SWING);
  add(registers::SLEEP_MODE);
  add(registers::FRESH_AIR_VALVE);

  // Outdoor temp: polled if the outdoor sensor or an exposed-sensor view is configured
  if (this->outdoor_temp_sensor_ != nullptr || this->expose_sensors_) {
    add(registers::OUTDOOR_TEMP);
  }

  // Current-temp source select needs all four candidate registers cached
  if (this->current_temp_source_select_ != nullptr) {
    add(registers::AMBIENT_TEMP);          // reg 4
    add(registers::AMBIENT_RETURN_AIR);    // reg 82
    add(registers::AMBIENT_LIGHT_BOARD);   // reg 83
  }

  // Exposed sensors need a few extra decoded registers
  if (this->expose_sensors_) {
    add(registers::CONTAMINATION);        // reg 34
    add(registers::SET_TEMP_PRECISE);     // reg 42
    add(registers::AMBIENT_RETURN_AIR);   // reg 82
    add(registers::AMBIENT_LIGHT_BOARD);  // reg 83
  }

  // Debug mode: sweep every address in range (debug reads the rest of 0..92)
  if (this->debug_mode_) {
    for (uint16_t a = registers::MIN_DEBUG_ADDR; a <= registers::MAX_DEBUG_ADDR; ++a) {
      add(a);
    }
  }

  this->read_plan_ = std::move(plan);
  this->read_plan_index_ = 0;
}

void GreeAC::loop() {
  uint32_t now = millis();

  // Async state machine for Modbus communication
  switch (this->modbus_state_) {
    case ModbusState::IDLE: {
      // Time to send next register request?
      uint32_t read_interval = this->update_interval_ / 10;
      if (read_interval < 100) read_interval = 100;
      // In debug mode we sweep many more registers; slow the cadence to be
      // gentle on the bus (full cycle ~14-18s instead of ~9s).
      if (this->debug_mode_ && read_interval < 200) read_interval = 200;

      if (now - this->last_update_ >= read_interval) {
        this->last_update_ = now;
        this->read_next_register();  // This now just sends the request
      }
      break;
    }

    case ModbusState::WAITING_RESPONSE: {
      this->check_response();  // Non-blocking check for response
      break;
    }
  }
}

void GreeAC::dump_config() {
  ESP_LOGCONFIG(TAG, "Gree AC Modbus:");
  ESP_LOGCONFIG(TAG, "  Version: %s", VERSION);
  ESP_LOGCONFIG(TAG, "  Slave ID: %d", this->slave_id_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %d ms", this->update_interval_);
  ESP_LOGCONFIG(TAG, "  Read plan size: %zu", this->read_plan_.size());
  if (this->flow_control_pin_ != nullptr) {
    LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  }
  if (this->current_temp_source_select_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Current temp source: %s",
                  CURRENT_TEMP_SOURCE_OPTIONS[static_cast<uint8_t>(this->current_temp_source_)].c_str());
  }
  if (this->expose_sensors_)
    ESP_LOGCONFIG(TAG, "  Expose sensors: ON");
  if (this->debug_mode_)
    ESP_LOGCONFIG(TAG, "  Debug mode: ON");
  LOG_CLIMATE("", "Gree AC", this);
  if (this->outdoor_temp_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Outdoor Temperature", this->outdoor_temp_sensor_);
  }
}

climate::ClimateTraits GreeAC::traits() {
  auto traits = climate::ClimateTraits();

  traits.set_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION);
  traits.set_visual_min_temperature(MIN_TEMPERATURE);
  traits.set_visual_max_temperature(MAX_TEMPERATURE);
  traits.set_visual_temperature_step(TEMPERATURE_STEP);

  // Supported modes
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY,
    climate::CLIMATE_MODE_HEAT_COOL,  // Auto mode
  });

  // Use only custom fan modes (like sinclair_ac UART implementation)
  // "0 - Auto" prefix prevents HA from treating it as standard FAN_AUTO
  traits.set_supported_custom_fan_modes({
    "0 - Auto",
    "1 - Speed 1",
    "2 - Speed 2",
    "3 - Speed 3",
    "4 - Speed 4",
    "5 - Speed 5",
    "6 - Turbo",
  });

  // Swing modes
  traits.set_supported_swing_modes({
    climate::CLIMATE_SWING_OFF,
    climate::CLIMATE_SWING_VERTICAL,
    climate::CLIMATE_SWING_HORIZONTAL,
    climate::CLIMATE_SWING_BOTH,
  });

  return traits;
}

void GreeAC::control(const climate::ClimateCall &call) {
  // Handle mode changes
  if (call.get_mode().has_value()) {
    climate::ClimateMode mode = *call.get_mode();

    if (mode == climate::CLIMATE_MODE_OFF) {
      // Turn off
      this->write_register(registers::ON_OFF, AC_OFF);
      this->ac_on_ = false;
    } else {
      // Turn on if not already
      if (!this->ac_on_) {
        this->write_register(registers::ON_OFF, AC_ON);
        this->ac_on_ = true;
      }

      // Set mode
      uint16_t modbus_mode = modes::AUTO;
      switch (mode) {
        case climate::CLIMATE_MODE_COOL:
          modbus_mode = modes::COOL;
          break;
        case climate::CLIMATE_MODE_HEAT:
          modbus_mode = modes::HEAT;
          break;
        case climate::CLIMATE_MODE_DRY:
          modbus_mode = modes::DRY;
          break;
        case climate::CLIMATE_MODE_FAN_ONLY:
          modbus_mode = modes::FAN_ONLY;
          break;
        case climate::CLIMATE_MODE_HEAT_COOL:
          modbus_mode = modes::AUTO;
          break;
        default:
          break;
      }
      this->write_register(registers::MODE, modbus_mode);
      this->mode_ = modbus_mode;
    }
    this->mode = mode;
  }

  // Handle target temperature
  if (call.get_target_temperature().has_value()) {
    float temp = *call.get_target_temperature();
    // Clamp to valid range
    temp = std::max(MIN_TEMPERATURE, std::min(MAX_TEMPERATURE, temp));

    // Register 20 is for whole degrees (register 42 is read-only for precise display)
    uint16_t raw_temp = static_cast<uint16_t>(temp + 0.5f);  // Round to nearest whole degree
    ESP_LOGD(TAG, "Setting temperature: %.1f (rounded: %d)", temp, raw_temp);
    this->write_register(registers::SET_TEMP, raw_temp);
    this->target_temp_ = static_cast<float>(raw_temp);
    this->target_temperature = this->target_temp_;
  }

  // Handle custom fan mode
  auto custom_fan = call.get_custom_fan_mode();
  if (!custom_fan.empty()) {
    uint16_t modbus_fan = fan_speeds::AUTO;

    ESP_LOGD(TAG, "Setting fan mode: %s", custom_fan.c_str());

    if (custom_fan == "0 - Auto") modbus_fan = fan_speeds::AUTO;
    else if (custom_fan == "1 - Speed 1") modbus_fan = fan_speeds::SPEED_1;
    else if (custom_fan == "2 - Speed 2") modbus_fan = fan_speeds::SPEED_2;
    else if (custom_fan == "3 - Speed 3") modbus_fan = fan_speeds::SPEED_3;
    else if (custom_fan == "4 - Speed 4") modbus_fan = fan_speeds::SPEED_4;
    else if (custom_fan == "5 - Speed 5") modbus_fan = fan_speeds::SPEED_5;
    else if (custom_fan == "6 - Turbo") modbus_fan = fan_speeds::TURBO;

    this->write_register(registers::FAN_SPEED, modbus_fan);
    this->fan_speed_ = modbus_fan;
    this->set_custom_fan_mode_(custom_fan);
  }

  // Handle swing mode
  if (call.get_swing_mode().has_value()) {
    climate::ClimateSwingMode swing = *call.get_swing_mode();

    uint16_t v_swing = 0;
    uint16_t h_swing = 0;

    switch (swing) {
      case climate::CLIMATE_SWING_OFF:
        v_swing = 0;
        h_swing = 0;
        break;
      case climate::CLIMATE_SWING_VERTICAL:
        v_swing = 1;  // Full swing
        h_swing = 0;
        break;
      case climate::CLIMATE_SWING_HORIZONTAL:
        v_swing = 0;
        h_swing = 1;  // Full swing
        break;
      case climate::CLIMATE_SWING_BOTH:
        v_swing = 1;
        h_swing = 1;
        break;
    }

    this->write_register(registers::VERTICAL_SWING, v_swing);
    this->write_register(registers::HORIZONTAL_SWING, h_swing);
    this->vertical_swing_ = v_swing;
    this->horizontal_swing_ = h_swing;
    this->swing_mode = swing;
  }

  this->publish_state();
}

bool GreeAC::send_modbus_request(uint8_t *request, uint8_t req_len, uint8_t *response, uint8_t *resp_len) {
  // Clear RX buffer
  while (this->available()) {
    this->read();
  }

  // Enable transmit mode for MAX485 modules
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
    delayMicroseconds(50);  // Small delay for transceiver to switch
  }

  // Send request
  this->write_array(request, req_len);
  this->flush();

  // Switch back to receive mode
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }

  // Wait for response with timeout
  uint32_t start = millis();
  uint8_t idx = 0;
  uint32_t last_byte = 0;

  while (millis() - start < 500) {
    if (this->available()) {
      if (idx < 64) {
        response[idx++] = this->read();
        last_byte = millis();
      }
    } else if (idx > 0 && millis() - last_byte > 10) {
      // Frame complete (silence detected)
      break;
    }
    yield();
  }

  *resp_len = idx;
  return idx > 0;
}

void GreeAC::send_read_request(uint16_t reg_addr) {
  // Clear RX buffer
  while (this->available()) {
    this->read();
  }

  // Build read request (function 0x03)
  uint8_t request[8];
  request[0] = this->slave_id_;
  request[1] = 0x03;  // Read holding registers
  request[2] = reg_addr >> 8;
  request[3] = reg_addr & 0xFF;
  request[4] = 0x00;  // Count high
  request[5] = 0x01;  // Count low (1 register)

  uint16_t crc = this->calculate_crc(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  // Enable transmit mode for MAX485 modules
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
    delayMicroseconds(50);
  }

  // Send request
  this->write_array(request, 8);
  this->flush();

  // Switch back to receive mode
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }

  // Set up async state
  this->pending_register_ = reg_addr;
  this->response_index_ = 0;
  this->request_start_time_ = millis();
  this->last_byte_time_ = 0;
  this->modbus_state_ = ModbusState::WAITING_RESPONSE;
}

void GreeAC::check_response() {
  uint32_t now = millis();

  // Check for timeout (500ms)
  if (now - this->request_start_time_ > 500) {
    ESP_LOGW(TAG, "No response reading register %d", this->pending_register_);
    if (this->pending_register_ <= registers::MAX_DEBUG_ADDR)
      this->register_valid_[this->pending_register_] = false;
    this->modbus_state_ = ModbusState::IDLE;
    this->advance_read_plan();
    return;
  }

  // Read available bytes (non-blocking)
  while (this->available() && this->response_index_ < 64) {
    this->response_buffer_[this->response_index_++] = this->read();
    this->last_byte_time_ = now;
  }

  // Check for frame complete (silence after receiving data)
  if (this->response_index_ > 0 && now - this->last_byte_time_ > 10) {
    bool handled = false;

    // Validate response
    if (this->response_index_ >= 5 &&
        this->response_buffer_[0] == this->slave_id_ &&
        (this->response_buffer_[1] & 0x80) == 0x80) {
      // Modbus exception response (function | 0x80). Common when sweeping
      // debug registers that don't exist on the unit.
      uint8_t exception = this->response_buffer_[2];
      ESP_LOGD(TAG, "Register %u returned Modbus exception %d", this->pending_register_, exception);
      if (this->pending_register_ <= registers::MAX_DEBUG_ADDR)
        this->register_valid_[this->pending_register_] = false;
      handled = true;
    } else if (this->response_index_ >= 7 &&
               this->response_buffer_[0] == this->slave_id_ &&
               this->response_buffer_[1] == 0x03) {
      // Verify CRC
      uint16_t rx_crc = this->response_buffer_[this->response_index_ - 2] |
                        (this->response_buffer_[this->response_index_ - 1] << 8);
      uint16_t calc_crc = this->calculate_crc(this->response_buffer_, this->response_index_ - 2);

      if (rx_crc == calc_crc) {
        // Extract value and process
        uint16_t value = (this->response_buffer_[3] << 8) | this->response_buffer_[4];
        this->process_register_response(this->pending_register_, value);
      } else {
        ESP_LOGW(TAG, "CRC mismatch reading register %d", this->pending_register_);
        if (this->pending_register_ <= registers::MAX_DEBUG_ADDR)
          this->register_valid_[this->pending_register_] = false;
      }
      handled = true;
    } else {
      ESP_LOGW(TAG, "Invalid response reading register %d: %d bytes", this->pending_register_, this->response_index_);
    }

    (void) handled;

    // Move to next register (advance_read_plan handles cycle-wrap publishing)
    this->modbus_state_ = ModbusState::IDLE;
    this->advance_read_plan();
  }
}

void GreeAC::advance_read_plan() {
  if (this->read_plan_.empty()) {
    // Nothing to poll; nothing to do.
    return;
  }

  this->read_plan_index_ = (this->read_plan_index_ + 1) % this->read_plan_.size();

  // On cycle wrap: refresh derived climate action/state, exposed sensors, and debug dump.
  if (this->read_plan_index_ == 0) {
    // Determine action based on state
    if (!this->ac_on_) {
      this->action = climate::CLIMATE_ACTION_OFF;
    } else if (this->mode_ == modes::FAN_ONLY) {
      this->action = climate::CLIMATE_ACTION_FAN;
    } else if (this->mode_ == modes::DRY) {
      this->action = climate::CLIMATE_ACTION_DRYING;
    } else if (this->mode_ == modes::COOL) {
      if (this->current_temp_ > this->target_temp_) {
        this->action = climate::CLIMATE_ACTION_COOLING;
      } else {
        this->action = climate::CLIMATE_ACTION_IDLE;
      }
    } else if (this->mode_ == modes::HEAT) {
      if (this->current_temp_ < this->target_temp_) {
        this->action = climate::CLIMATE_ACTION_HEATING;
      } else {
        this->action = climate::CLIMATE_ACTION_IDLE;
      }
    } else if (this->mode_ == modes::AUTO) {
      // Auto mode: determine if heating or cooling based on temperature difference
      if (this->current_temp_ > this->target_temp_) {
        this->action = climate::CLIMATE_ACTION_COOLING;
      } else if (this->current_temp_ < this->target_temp_) {
        this->action = climate::CLIMATE_ACTION_HEATING;
      } else {
        this->action = climate::CLIMATE_ACTION_IDLE;
      }
    } else {
      this->action = climate::CLIMATE_ACTION_IDLE;
    }
    this->publish_state();

    this->publish_exposed_sensors();
    this->publish_debug_dump();
  }
}

void GreeAC::process_register_response(uint16_t reg_addr, uint16_t value) {
  // Cache the raw value for every successful read (powers debug dump, the
  // current-temp source select, and exposed ambient sensors).
  this->store_register(reg_addr, value);

  switch (reg_addr) {
    case registers::ON_OFF:
      this->ac_on_ = (value == AC_ON);
      break;

    case registers::MODE:
      this->mode_ = value;
      switch (value) {
        case modes::COOL:
          this->mode = climate::CLIMATE_MODE_COOL;
          break;
        case modes::HEAT:
          this->mode = climate::CLIMATE_MODE_HEAT;
          break;
        case modes::DRY:
          this->mode = climate::CLIMATE_MODE_DRY;
          break;
        case modes::FAN_ONLY:
          this->mode = climate::CLIMATE_MODE_FAN_ONLY;
          break;
        case modes::AUTO:
          this->mode = climate::CLIMATE_MODE_HEAT_COOL;
          break;
      }
      if (!this->ac_on_) {
        this->mode = climate::CLIMATE_MODE_OFF;
      }
      break;

    // Any of the four candidate current-temp registers may supply
    // current_temperature, depending on the selected source. Recompute from
    // the cache only when this read matches the active source.
    case registers::TEMP_SENSOR_3:
    case registers::AMBIENT_TEMP:
    case registers::AMBIENT_RETURN_AIR:
    case registers::AMBIENT_LIGHT_BOARD:
      this->recompute_current_temp();
      break;

    case registers::SET_TEMP:
      this->target_temp_ = static_cast<float>(value);
      this->target_temperature = this->target_temp_;
      break;

    case registers::FAN_SPEED:
      this->fan_speed_ = value;
      switch (value) {
        case fan_speeds::AUTO:
          this->set_custom_fan_mode_("0 - Auto");
          this->turbo_mode_ = false;
          break;
        case fan_speeds::SPEED_1:
          this->set_custom_fan_mode_("1 - Speed 1");
          this->turbo_mode_ = false;
          break;
        case fan_speeds::SPEED_2:
          this->set_custom_fan_mode_("2 - Speed 2");
          this->turbo_mode_ = false;
          break;
        case fan_speeds::SPEED_3:
          this->set_custom_fan_mode_("3 - Speed 3");
          this->turbo_mode_ = false;
          break;
        case fan_speeds::SPEED_4:
          this->set_custom_fan_mode_("4 - Speed 4");
          this->turbo_mode_ = false;
          break;
        case fan_speeds::SPEED_5:
          this->set_custom_fan_mode_("5 - Speed 5");
          this->turbo_mode_ = false;
          break;
        case fan_speeds::TURBO:
          this->set_custom_fan_mode_("6 - Turbo");
          this->turbo_mode_ = true;
          break;
      }
      break;

    case registers::VERTICAL_SWING:
      this->vertical_swing_ = value;
      if (this->vertical_swing_select_ != nullptr && value < VERTICAL_SWING_OPTIONS.size()) {
        this->vertical_swing_select_->publish_state(VERTICAL_SWING_OPTIONS[value]);
      }
      break;

    case registers::HORIZONTAL_SWING:
      this->horizontal_swing_ = value;
      if (this->horizontal_swing_select_ != nullptr && value < HORIZONTAL_SWING_OPTIONS.size()) {
        this->horizontal_swing_select_->publish_state(HORIZONTAL_SWING_OPTIONS[value]);
      }
      // Update combined swing mode
      if (this->vertical_swing_ > 0 && this->horizontal_swing_ > 0) {
        this->swing_mode = climate::CLIMATE_SWING_BOTH;
      } else if (this->vertical_swing_ > 0) {
        this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      } else if (this->horizontal_swing_ > 0) {
        this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
      } else {
        this->swing_mode = climate::CLIMATE_SWING_OFF;
      }
      break;

    case registers::OUTDOOR_TEMP:
      if (this->outdoor_temp_sensor_ != nullptr) {
        float temp;
        if (value & 0x8000) {
          temp = -static_cast<float>(value & 0x7FFF);
        } else {
          temp = static_cast<float>(value);
        }
        this->outdoor_temp_ = temp;
        this->outdoor_temp_sensor_->publish_state(temp);
      }
      break;

    case registers::SLEEP_MODE:
      this->sleep_mode_ = (value > 0);
      if (this->sleep_switch_ != nullptr) {
        this->sleep_switch_->publish_state(this->sleep_mode_);
      }
      break;

    case registers::FRESH_AIR_VALVE:
      this->fresh_air_ = (value > 0);
      if (this->fresh_air_switch_ != nullptr) {
        this->fresh_air_switch_->publish_state(this->fresh_air_);
      }
      break;
  }
}

void GreeAC::store_register(uint16_t addr, uint16_t value) {
  if (addr <= registers::MAX_DEBUG_ADDR) {
    this->register_cache_[addr] = value;
    this->register_valid_[addr] = true;
  }
}

float GreeAC::compute_current_temp_from_cache() const {
  uint16_t reg = registers::TEMP_SENSOR_3;
  float scale = 10.0f;
  bool sign = true;
  switch (this->current_temp_source_) {
    case CurrentTempSource::WIRED_CONTROLLER:
      reg = registers::TEMP_SENSOR_3;      scale = 10.0f; sign = true;  break;
    case CurrentTempSource::IDU_RETURN_AIR:
      reg = registers::AMBIENT_TEMP;       scale = 1.0f;  sign = false; break;
    case CurrentTempSource::RETURN_AIR_PORT:
      reg = registers::AMBIENT_RETURN_AIR; scale = 10.0f; sign = true;  break;
    case CurrentTempSource::LIGHT_BOARD:
      reg = registers::AMBIENT_LIGHT_BOARD; scale = 10.0f; sign = true;  break;
  }
  if (reg > registers::MAX_DEBUG_ADDR || !this->register_valid_[reg])
    return this->current_temp_;  // keep last known value when source data is missing
  uint16_t v = this->register_cache_[reg];
  if (sign && (v & 0x8000))
    return -static_cast<float>(v & 0x7FFF) / scale;
  return static_cast<float>(v) / scale;
}

void GreeAC::recompute_current_temp() {
  float temp = this->compute_current_temp_from_cache();
  if (temp == this->current_temp_)
    return;
  this->current_temp_ = temp;
  this->current_temperature = temp;
}

void GreeAC::publish_exposed_sensors() {
  // Only published when at least one exposed sensor is configured.
  if (this->set_point_sensor_ == nullptr && this->current_temp_sensor_ == nullptr &&
      this->mode_sensor_ == nullptr && this->fan_speed_sensor_ == nullptr &&
      this->on_off_sensor_ == nullptr && this->sleep_sensor_ == nullptr &&
      this->turbo_sensor_ == nullptr && this->fresh_air_sensor_ == nullptr &&
      this->contamination_sensor_ == nullptr && this->set_temp_precise_sensor_ == nullptr &&
      this->ambient_return_air_sensor_ == nullptr && this->ambient_light_board_sensor_ == nullptr)
    return;

  auto publish_raw = [&](sensor::Sensor *s, uint16_t reg) {
    if (s != nullptr && reg <= registers::MAX_DEBUG_ADDR && this->register_valid_[reg])
      s->publish_state(static_cast<float>(this->register_cache_[reg]));
  };
  // Temperature registers stored ×10 get scaled to degrees here.
  auto publish_scaled = [&](sensor::Sensor *s, uint16_t reg) {
    if (s != nullptr && reg <= registers::MAX_DEBUG_ADDR && this->register_valid_[reg])
      s->publish_state(static_cast<float>(this->register_cache_[reg]) / 10.0f);
  };

  if (this->set_point_sensor_) this->set_point_sensor_->publish_state(this->target_temp_);
  if (this->current_temp_sensor_) this->current_temp_sensor_->publish_state(this->current_temp_);
  if (this->mode_sensor_) this->mode_sensor_->publish_state(static_cast<float>(this->mode_));
  if (this->fan_speed_sensor_) this->fan_speed_sensor_->publish_state(static_cast<float>(this->fan_speed_));
  if (this->on_off_sensor_) this->on_off_sensor_->publish_state(this->ac_on_ ? 1.0f : 0.0f);
  if (this->sleep_sensor_) this->sleep_sensor_->publish_state(this->sleep_mode_ ? 1.0f : 0.0f);
  if (this->turbo_sensor_) this->turbo_sensor_->publish_state(this->turbo_mode_ ? 1.0f : 0.0f);
  if (this->fresh_air_sensor_) this->fresh_air_sensor_->publish_state(this->fresh_air_ ? 1.0f : 0.0f);
  publish_raw(this->contamination_sensor_, registers::CONTAMINATION);
  publish_scaled(this->set_temp_precise_sensor_, registers::SET_TEMP_PRECISE);
  publish_scaled(this->ambient_return_air_sensor_, registers::AMBIENT_RETURN_AIR);
  publish_scaled(this->ambient_light_board_sensor_, registers::AMBIENT_LIGHT_BOARD);
}

void GreeAC::publish_debug_dump() {
  if (this->debug_text_sensor_ == nullptr)
    return;
  std::string s = "{";
  char buf[24];
  for (uint16_t a = registers::MIN_DEBUG_ADDR; a <= registers::MAX_DEBUG_ADDR; ++a) {
    if (a > registers::MIN_DEBUG_ADDR)
      s += ",";
    if (this->register_valid_[a])
      snprintf(buf, sizeof(buf), "\"%u\":%u", a, this->register_cache_[a]);
    else
      snprintf(buf, sizeof(buf), "\"%u\":null", a);
    s += buf;
  }
  s += "}";
  this->debug_text_sensor_->publish_state(s);
}

void GreeAC::handle_debug_write() {
  if (this->debug_address_number_ == nullptr || this->debug_value_number_ == nullptr)
    return;
  if (!this->debug_address_number_->has_state() || !this->debug_value_number_->has_state())
    return;
  uint16_t addr = static_cast<uint16_t>(this->debug_address_number_->state);
  uint16_t val = static_cast<uint16_t>(this->debug_value_number_->state);
  ESP_LOGW(TAG, "DEBUG WRITE register %u = %u", addr, val);
  bool ok = this->write_register(addr, val);
  if (ok)
    ESP_LOGI(TAG, "Debug write OK (reg %u)", addr);
  else
    ESP_LOGE(TAG, "Debug write FAILED (reg %u)", addr);
}

bool GreeAC::read_register(uint16_t reg_addr, uint16_t *value) {
  uint8_t request[8];
  uint8_t response[64];
  uint8_t resp_len = 0;

  // Build read request (function 0x03)
  request[0] = this->slave_id_;
  request[1] = 0x03;  // Read holding registers
  request[2] = reg_addr >> 8;
  request[3] = reg_addr & 0xFF;
  request[4] = 0x00;  // Count high
  request[5] = 0x01;  // Count low (1 register)

  uint16_t crc = this->calculate_crc(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  if (!this->send_modbus_request(request, 8, response, &resp_len)) {
    ESP_LOGW(TAG, "No response reading register %d", reg_addr);
    return false;
  }

  // Validate response
  if (resp_len < 7) {
    ESP_LOGW(TAG, "Short response reading register %d: %d bytes", reg_addr, resp_len);
    return false;
  }

  if (response[0] != this->slave_id_ || response[1] != 0x03) {
    ESP_LOGW(TAG, "Invalid response reading register %d", reg_addr);
    return false;
  }

  // Verify CRC
  uint16_t rx_crc = response[resp_len - 2] | (response[resp_len - 1] << 8);
  uint16_t calc_crc = this->calculate_crc(response, resp_len - 2);
  if (rx_crc != calc_crc) {
    ESP_LOGW(TAG, "CRC mismatch reading register %d", reg_addr);
    return false;
  }

  // Extract value
  *value = (response[3] << 8) | response[4];
  return true;
}

bool GreeAC::write_register(uint16_t reg_addr, uint16_t value) {
  uint8_t request[11];
  uint8_t response[64];
  uint8_t resp_len = 0;

  // Build write request (function 0x10 - Write Multiple Registers)
  // The Gree AC only supports function 0x10, not 0x06
  request[0] = this->slave_id_;
  request[1] = 0x10;  // Write multiple registers
  request[2] = reg_addr >> 8;
  request[3] = reg_addr & 0xFF;
  request[4] = 0x00;  // Number of registers high
  request[5] = 0x01;  // Number of registers low (1)
  request[6] = 0x02;  // Byte count
  request[7] = value >> 8;
  request[8] = value & 0xFF;

  uint16_t crc = this->calculate_crc(request, 9);
  request[9] = crc & 0xFF;
  request[10] = crc >> 8;

  if (!this->send_modbus_request(request, 11, response, &resp_len)) {
    ESP_LOGW(TAG, "No response writing register %d", reg_addr);
    return false;
  }

  // Validate response (should echo back slave, function, register, count)
  if (resp_len >= 8 && response[0] == this->slave_id_ && response[1] == 0x10) {
    ESP_LOGD(TAG, "Successfully wrote register %d = %d", reg_addr, value);
    return true;
  }

  ESP_LOGW(TAG, "Failed to write register %d", reg_addr);
  return false;
}

void GreeAC::read_all_registers() {
  uint16_t value;

  // Read On/Off state
  if (this->read_register(registers::ON_OFF, &value)) {
    this->ac_on_ = (value == AC_ON);
  }

  // Read mode
  if (this->read_register(registers::MODE, &value)) {
    this->mode_ = value;
    switch (value) {
      case modes::COOL:
        this->mode = climate::CLIMATE_MODE_COOL;
        break;
      case modes::HEAT:
        this->mode = climate::CLIMATE_MODE_HEAT;
        break;
      case modes::DRY:
        this->mode = climate::CLIMATE_MODE_DRY;
        break;
      case modes::FAN_ONLY:
        this->mode = climate::CLIMATE_MODE_FAN_ONLY;
        break;
      case modes::AUTO:
        this->mode = climate::CLIMATE_MODE_HEAT_COOL;
        break;
    }
  }

  // Override mode if AC is off
  if (!this->ac_on_) {
    this->mode = climate::CLIMATE_MODE_OFF;
  }

  // Read current temperature from wired controller sensor (register 3, value ÷ 10)
  // Register 39 = 3 means "use register 3 for ambient temp" (wired controller's sensor)
  if (this->read_register(registers::TEMP_SENSOR_3, &value)) {
    // Handle negative temperatures (bit 15 is sign)
    if (value & 0x8000) {
      this->current_temp_ = -static_cast<float>(value & 0x7FFF) / 10.0f;
    } else {
      this->current_temp_ = static_cast<float>(value) / 10.0f;
    }
    this->current_temperature = this->current_temp_;
  }

  // Read set temperature (register 20, whole degrees)
  if (this->read_register(registers::SET_TEMP, &value)) {
    this->target_temp_ = static_cast<float>(value);
    this->target_temperature = this->target_temp_;
  }

  // Read fan speed
  if (this->read_register(registers::FAN_SPEED, &value)) {
    this->fan_speed_ = value;
    switch (value) {
      case fan_speeds::AUTO:
        this->set_custom_fan_mode_("0 - Auto");
        this->turbo_mode_ = false;
        break;
      case fan_speeds::SPEED_1:
        this->set_custom_fan_mode_("1 - Speed 1");
        this->turbo_mode_ = false;
        break;
      case fan_speeds::SPEED_2:
        this->set_custom_fan_mode_("2 - Speed 2");
        this->turbo_mode_ = false;
        break;
      case fan_speeds::SPEED_3:
        this->set_custom_fan_mode_("3 - Speed 3");
        this->turbo_mode_ = false;
        break;
      case fan_speeds::SPEED_4:
        this->set_custom_fan_mode_("4 - Speed 4");
        this->turbo_mode_ = false;
        break;
      case fan_speeds::SPEED_5:
        this->set_custom_fan_mode_("5 - Speed 5");
        this->turbo_mode_ = false;
        break;
      case fan_speeds::TURBO:
        this->set_custom_fan_mode_("6 - Turbo");
        this->turbo_mode_ = true;
        break;
    }
  }

  // Read swing modes
  if (this->read_register(registers::VERTICAL_SWING, &value)) {
    this->vertical_swing_ = value;
    // Update vertical swing select if present
    if (this->vertical_swing_select_ != nullptr && value < VERTICAL_SWING_OPTIONS.size()) {
      this->vertical_swing_select_->publish_state(VERTICAL_SWING_OPTIONS[value]);
    }
  }

  if (this->read_register(registers::HORIZONTAL_SWING, &value)) {
    this->horizontal_swing_ = value;
    // Update horizontal swing select if present
    if (this->horizontal_swing_select_ != nullptr && value < HORIZONTAL_SWING_OPTIONS.size()) {
      this->horizontal_swing_select_->publish_state(HORIZONTAL_SWING_OPTIONS[value]);
    }
  }

  // Determine combined swing mode
  if (this->vertical_swing_ > 0 && this->horizontal_swing_ > 0) {
    this->swing_mode = climate::CLIMATE_SWING_BOTH;
  } else if (this->vertical_swing_ > 0) {
    this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
  } else if (this->horizontal_swing_ > 0) {
    this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  } else {
    this->swing_mode = climate::CLIMATE_SWING_OFF;
  }

  // Read outdoor temperature if sensor is configured
  if (this->outdoor_temp_sensor_ != nullptr) {
    if (this->read_register(registers::OUTDOOR_TEMP, &value)) {
      // Handle negative temperatures
      float temp;
      if (value & 0x8000) {
        temp = -static_cast<float>(value & 0x7FFF);
      } else {
        temp = static_cast<float>(value);
      }
      this->outdoor_temp_ = temp;
      this->outdoor_temp_sensor_->publish_state(temp);
    }
  }

  // Read sleep mode
  if (this->read_register(registers::SLEEP_MODE, &value)) {
    this->sleep_mode_ = (value > 0);
    if (this->sleep_switch_ != nullptr) {
      this->sleep_switch_->publish_state(this->sleep_mode_);
    }
  }

  // Read fresh air valve
  if (this->read_register(registers::FRESH_AIR_VALVE, &value)) {
    this->fresh_air_ = (value > 0);
    if (this->fresh_air_switch_ != nullptr) {
      this->fresh_air_switch_->publish_state(this->fresh_air_);
    }
  }

  // Determine action based on state
  if (!this->ac_on_) {
    this->action = climate::CLIMATE_ACTION_OFF;
  } else if (this->mode_ == modes::FAN_ONLY) {
    this->action = climate::CLIMATE_ACTION_FAN;
  } else if (this->mode_ == modes::DRY) {
    this->action = climate::CLIMATE_ACTION_DRYING;
  } else if (this->mode_ == modes::COOL) {
    if (this->current_temp_ > this->target_temp_) {
      this->action = climate::CLIMATE_ACTION_COOLING;
    } else {
      this->action = climate::CLIMATE_ACTION_IDLE;
    }
  } else if (this->mode_ == modes::HEAT) {
    if (this->current_temp_ < this->target_temp_) {
      this->action = climate::CLIMATE_ACTION_HEATING;
    } else {
      this->action = climate::CLIMATE_ACTION_IDLE;
    }
  } else if (this->mode_ == modes::AUTO) {
    // Auto mode: determine if heating or cooling based on temperature difference
    if (this->current_temp_ > this->target_temp_) {
      this->action = climate::CLIMATE_ACTION_COOLING;
    } else if (this->current_temp_ < this->target_temp_) {
      this->action = climate::CLIMATE_ACTION_HEATING;
    } else {
      this->action = climate::CLIMATE_ACTION_IDLE;
    }
  } else {
    this->action = climate::CLIMATE_ACTION_IDLE;
  }

  this->publish_state();
}

void GreeAC::read_next_register() {
  // Dispatch an async read request for the current entry in the read plan.
  // Response is handled by check_response() -> process_register_response().
  if (this->read_plan_.empty())
    return;
  this->send_read_request(this->read_plan_[this->read_plan_index_]);
}

void GreeAC::set_vertical_swing(uint16_t value) {
  this->write_register(registers::VERTICAL_SWING, value);
  this->vertical_swing_ = value;
}

void GreeAC::set_horizontal_swing(uint16_t value) {
  this->write_register(registers::HORIZONTAL_SWING, value);
  this->horizontal_swing_ = value;
}

void GreeAC::set_sleep_mode(bool enabled) {
  this->write_register(registers::SLEEP_MODE, enabled ? 1 : 0);
  this->sleep_mode_ = enabled;
}

void GreeAC::set_turbo_mode(bool enabled) {
  if (enabled) {
    this->write_register(registers::FAN_SPEED, fan_speeds::TURBO);
    this->fan_speed_ = fan_speeds::TURBO;
  } else {
    // Revert to auto
    this->write_register(registers::FAN_SPEED, fan_speeds::AUTO);
    this->fan_speed_ = fan_speeds::AUTO;
  }
  this->turbo_mode_ = enabled;
}

void GreeAC::set_fresh_air(bool enabled) {
  this->write_register(registers::FRESH_AIR_VALVE, enabled ? 1 : 0);
  this->fresh_air_ = enabled;
}

// Callback-based setters (like sinclair_ac pattern)
void GreeAC::set_vertical_swing_select(select::Select *vertical_swing_select) {
  this->vertical_swing_select_ = vertical_swing_select;
  this->vertical_swing_select_->add_on_state_callback([this](size_t index) {
    if (index == this->vertical_swing_)
      return;
    this->set_vertical_swing(static_cast<uint16_t>(index));
  });
}

void GreeAC::set_horizontal_swing_select(select::Select *horizontal_swing_select) {
  this->horizontal_swing_select_ = horizontal_swing_select;
  this->horizontal_swing_select_->add_on_state_callback([this](size_t index) {
    if (index == this->horizontal_swing_)
      return;
    this->set_horizontal_swing(static_cast<uint16_t>(index));
  });
}

void GreeAC::set_sleep_switch(switch_::Switch *sleep_switch) {
  this->sleep_switch_ = sleep_switch;
  this->sleep_switch_->add_on_state_callback([this](bool state) {
    if (state == this->sleep_mode_)
      return;
    this->set_sleep_mode(state);
  });
}

void GreeAC::set_turbo_switch(switch_::Switch *turbo_switch) {
  this->turbo_switch_ = turbo_switch;
  this->turbo_switch_->add_on_state_callback([this](bool state) {
    if (state == this->turbo_mode_)
      return;
    this->set_turbo_mode(state);
  });
}

void GreeAC::set_fresh_air_switch(switch_::Switch *fresh_air_switch) {
  this->fresh_air_switch_ = fresh_air_switch;
  this->fresh_air_switch_->add_on_state_callback([this](bool state) {
    if (state == this->fresh_air_)
      return;
    this->set_fresh_air(state);
  });
}

void GreeAC::set_current_temp_source_select(select::Select *select) {
  this->current_temp_source_select_ = select;
  // The select reports the chosen option as its index, which maps 1:1 to the
  // CurrentTempSource enum order. Recompute current_temperature instantly from
  // the cached register values on change.
  this->current_temp_source_select_->add_on_state_callback([this](size_t index) {
    if (index >= CURRENT_TEMP_SOURCE_OPTIONS.size())
      return;
    CurrentTempSource new_source = static_cast<CurrentTempSource>(index);
    if (new_source == this->current_temp_source_)
      return;
    this->current_temp_source_ = new_source;
    ESP_LOGI(TAG, "Current temp source set to %s", CURRENT_TEMP_SOURCE_OPTIONS[index].c_str());
    this->recompute_current_temp();
    this->publish_state();
  });
}

void GreeAC::set_debug_address_number(number::Number *address_number) {
  this->debug_address_number_ = address_number;
}

void GreeAC::set_debug_value_number(number::Number *value_number) {
  this->debug_value_number_ = value_number;
}

void GreeAC::set_debug_write_button(button::Button *write_button) {
  this->debug_write_button_ = write_button;
  this->debug_write_button_->add_on_press_callback([this]() { this->handle_debug_write(); });
}

}  // namespace gree_ac
}  // namespace esphome

#include "sen6x.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cinttypes>

namespace esphome {
namespace sen6x {

static const char *const TAG = "sen6x";

static const uint16_t SEN6X_CMD_GET_DATA_READY_STATUS = 0x0202;
static const uint16_t SEN6X_CMD_GET_FIRMWARE_VERSION = 0xD100;
static const uint16_t SEN6X_CMD_GET_PRODUCT_NAME = 0xD014;
static const uint16_t SEN6X_CMD_GET_SERIAL_NUMBER = 0xD033;

static const uint16_t SEN6X_CMD_READ_MEASUREMENT = 0x0300;  // SEN66 only!
static const uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN62 = 0x04A3;
static const uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN63C = 0x0471;
static const uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN65 = 0x0446;
static const uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN68 = 0x0467;
static const uint16_t SEN6X_CMD_READ_MEASUREMENT_SEN69C = 0x04B5;

static const uint16_t SEN6X_CMD_START_MEASUREMENTS = 0x0021;
static const uint16_t SEN6X_CMD_STOP_MEASUREMENTS = 0x0104;
static const uint16_t SEN6X_CMD_RESET = 0xD304;

static inline void set_read_command_and_words(SEN6XComponent::Sen6xType type, uint16_t &read_cmd, uint8_t &read_words) {
  read_cmd = SEN6X_CMD_READ_MEASUREMENT;
  read_words = 9;
  switch (type) {
    case SEN6XComponent::SEN62:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN62;
      read_words = 6;
      break;
    case SEN6XComponent::SEN63C:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN63C;
      read_words = 7;
      break;
    case SEN6XComponent::SEN65:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN65;
      read_words = 8;
      break;
    case SEN6XComponent::SEN66:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT;
      read_words = 9;
      break;
    case SEN6XComponent::SEN68:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN68;
      read_words = 9;
      break;
    case SEN6XComponent::SEN69C:
      read_cmd = SEN6X_CMD_READ_MEASUREMENT_SEN69C;
      read_words = 10;
      break;
    default:
      break;
  }
}

void SEN6XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up sen6x...");

  // the sensor needs 100 ms to enter the idle state
  this->set_timeout(100, [this]() {
    // Check if measurement is ready before reading the value
    if (!this->write_command(SEN6X_CMD_GET_DATA_READY_STATUS)) {
      ESP_LOGE(TAG, "Failed to write data ready status command");
      this->mark_failed();
      return;
    }

    uint16_t raw_read_status;
    if (!this->read_data(raw_read_status)) {
      ESP_LOGE(TAG, "Failed to read data ready status");
      this->mark_failed();
      return;
    }

    // In order to query the device periodic measurement must be ceased => use reset!
    if (raw_read_status) {
      ESP_LOGD(TAG, "Sensor has data available, stopping periodic measurement / reset");

      if (!this->write_command(SEN6X_CMD_STOP_MEASUREMENTS)) {
        ESP_LOGE(TAG, "Failed to stop measurements");
        this->mark_failed();
        return;
      }
    }

    this->set_timeout(20, [this]() {
      uint16_t raw_serial_number[16];
      if (!this->get_register(SEN6X_CMD_GET_SERIAL_NUMBER, raw_serial_number, 16, 20)) {
        ESP_LOGE(TAG, "Failed to read serial number");
        this->error_code_ = SERIAL_NUMBER_IDENTIFICATION_FAILED;
        this->mark_failed();
        return;
      }
      this->serial_number_.clear();
      this->serial_number_.reserve(32);
      for (uint8_t i = 0; i < 16; i++) {
        const uint16_t word = raw_serial_number[i];
        const char c1 = static_cast<char>(word >> 8);
        const char c2 = static_cast<char>(word & 0xFF);
        if (c1 == '\0')
          break;
        this->serial_number_.push_back(c1);
        if (c2 == '\0')
          break;
        this->serial_number_.push_back(c2);
      }
      ESP_LOGD(TAG, "Serial number %s", this->serial_number_.c_str());

      uint16_t raw_product_name[16];
      if (!this->get_register(SEN6X_CMD_GET_PRODUCT_NAME, raw_product_name, 16, 20)) {
        ESP_LOGE(TAG, "Failed to read product name");
        this->error_code_ = PRODUCT_NAME_FAILED;
        this->mark_failed();
        return;
      }

      this->product_name_.clear();
      // 2 ASCII bytes are encoded in an int
      const uint16_t *current_int = raw_product_name;
      char current_char;
      uint8_t max = 16;
      do {
        // first char
        current_char = *current_int >> 8;
        if (current_char) {
          this->product_name_.push_back(current_char);
          // second char
          current_char = *current_int & 0xFF;
          if (current_char)
            this->product_name_.push_back(current_char);
        }
        current_int++;
      } while (current_char && --max);

      this->sen6x_type_ = UNKNOWN;
      if (this->product_name_ == "SEN62") {
        this->sen6x_type_ = SEN62;
      } else if (this->product_name_ == "SEN63C") {
        this->sen6x_type_ = SEN63C;
      } else if (this->product_name_ == "SEN65") {
        this->sen6x_type_ = SEN65;
      } else if (this->product_name_ == "SEN66") {
        this->sen6x_type_ = SEN66;
      } else if (this->product_name_ == "SEN68") {
        this->sen6x_type_ = SEN68;
      } else if (this->product_name_ == "SEN69C") {
        this->sen6x_type_ = SEN69C;
      } else if (this->product_name_ == "") {  // empty name
        ESP_LOGD(TAG, "Productname empty, falling back to SEN66");
        this->sen6x_type_ = SEN66;
      }
      ESP_LOGD(TAG, "Productname %s", this->product_name_.c_str());

      uint16_t raw_firmware_version = 0;
      if (!this->get_register(SEN6X_CMD_GET_FIRMWARE_VERSION, raw_firmware_version, 20)) {
        ESP_LOGE(TAG, "Failed to read firmware version");
        this->error_code_ = FIRMWARE_FAILED;
        this->mark_failed();
        return;
      }
      this->firmware_version_major_ = (raw_firmware_version >> 8) & 0xFF;
      this->firmware_version_minor_ = raw_firmware_version & 0xFF;
      ESP_LOGD(TAG, "Firmware version %u.%u", this->firmware_version_major_, this->firmware_version_minor_);

      this->finish_setup_();
    });
  });
}

bool SEN6XComponent::is_measurement_running() const { return this->measurement_started_; }

void SEN6XComponent::finish_setup_() {
  if (!this->write_command(SEN6X_CMD_START_MEASUREMENTS)) {
    ESP_LOGE(TAG, "Error starting continuous measurements.");

    this->error_code_ = MEASUREMENT_INIT_FAILED;
    this->mark_failed();
    return;
  }

  const uint32_t now = App.get_loop_component_start_time();

  this->measurement_started_ = true;
  this->startup_stable_after_ = now + this->startup_delay_ms_;
  this->initialized_ = true;
  ESP_LOGD(TAG, "Sensor initialized");
}

void SEN6XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "sen6x:");
  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    switch (this->error_code_) {
      case COMMUNICATION_FAILED:
        ESP_LOGW(TAG, "Communication failed! Is the sensor connected?");
        break;
      case MEASUREMENT_INIT_FAILED:
        ESP_LOGW(TAG, "Measurement Initialization failed!");
        break;
      case SERIAL_NUMBER_IDENTIFICATION_FAILED:
        ESP_LOGW(TAG, "Unable to read sensor serial id");
        break;
      case PRODUCT_NAME_FAILED:
        ESP_LOGW(TAG, "Unable to read product name");
        break;
      case FIRMWARE_FAILED:
        ESP_LOGW(TAG, "Unable to read sensor firmware version");
        break;
      default:
        ESP_LOGW(TAG, "Unknown setup error!");
        break;
    }
  }
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Startup delay: %u ms", this->startup_delay_ms_);
  LOG_SENSOR("  ", "PM  1.0", this->pm_1_0_sensor_);
  LOG_SENSOR("  ", "PM  2.5", this->pm_2_5_sensor_);
  LOG_SENSOR("  ", "PM  4.0", this->pm_4_0_sensor_);
  LOG_SENSOR("  ", "PM 10.0", this->pm_10_0_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  LOG_SENSOR("  ", "VOC", this->voc_sensor_);
  LOG_SENSOR("  ", "NOx", this->nox_sensor_);
  LOG_SENSOR("  ", "HCHO", this->hcho_sensor_);
  LOG_SENSOR("  ", "CO2", this->co2_sensor_);
}

void SEN6XComponent::update() {
  if (!this->initialized_) {
    return;
  }
  const uint32_t now = App.get_loop_component_start_time();
  if (this->last_stop_ms_ != 0 && (now - this->last_stop_ms_) < 1400) {
    const uint32_t wait_ms = 1400 - (now - this->last_stop_ms_);
    this->set_timeout(wait_ms, [this]() { this->update(); });
    return;
  }
  if (!this->measurement_started_) {
    if (this->has_last_values_) {
      if (this->pm_1_0_sensor_ != nullptr)
        this->pm_1_0_sensor_->publish_state(this->last_pm_1_0_);
      if (this->pm_2_5_sensor_ != nullptr)
        this->pm_2_5_sensor_->publish_state(this->last_pm_2_5_);
      if (this->pm_4_0_sensor_ != nullptr)
        this->pm_4_0_sensor_->publish_state(this->last_pm_4_0_);
      if (this->pm_10_0_sensor_ != nullptr)
        this->pm_10_0_sensor_->publish_state(this->last_pm_10_0_);
      if (this->temperature_sensor_ != nullptr)
        this->temperature_sensor_->publish_state(this->last_temperature_);
      if (this->humidity_sensor_ != nullptr)
        this->humidity_sensor_->publish_state(this->last_humidity_);
      if (this->voc_sensor_ != nullptr)
        this->voc_sensor_->publish_state(this->last_voc_);
      if (this->nox_sensor_ != nullptr)
        this->nox_sensor_->publish_state(this->last_nox_);
      if (this->hcho_sensor_ != nullptr)
        this->hcho_sensor_->publish_state(this->last_hcho_);
      if (this->co2_sensor_ != nullptr)
        this->co2_sensor_->publish_state(this->last_co2_);
      this->status_clear_warning();
    }
    return;
  }

  uint16_t read_cmd;
  uint8_t read_words;
  set_read_command_and_words(this->sen6x_type_, read_cmd, read_words);

  const uint8_t poll_retries = 24;
  auto poll_ready = std::make_shared<std::function<void(uint8_t)>>();
  *poll_ready = [this, poll_ready, poll_retries, read_cmd, read_words](uint8_t retries_left) {
    const uint8_t attempt = static_cast<uint8_t>(poll_retries - retries_left + 1);
    ESP_LOGV(TAG, "Data ready polling attempt %u", attempt);
    uint16_t raw_read_status;
    if (!this->get_register(SEN6X_CMD_GET_DATA_READY_STATUS, raw_read_status, 20)) {
      this->status_set_warning();
      ESP_LOGD(TAG, "read data ready status error (%d)", this->last_error_);
      return;
    }

    if ((raw_read_status & 0x0001) == 0) {
      if (retries_left == 0) {
        this->status_set_warning();
        ESP_LOGD(TAG, "data not ready in time");
        return;
      }
      this->set_timeout(50, [this, poll_ready, retries_left]() { (*poll_ready)(retries_left - 1); });
      return;
    }

    if (!this->write_command(read_cmd)) {
      this->status_set_warning();
      ESP_LOGD(TAG, "write error read measurement (%d)", this->last_error_);
      return;
    }

    this->set_timeout(20, [this, read_words]() {
      uint16_t measurements[10];

      if (!this->read_data(measurements, read_words)) {
        this->status_set_warning();
        ESP_LOGD(TAG, "read data error (%d)", this->last_error_);
        return;
      }
      int8_t voc_index = -1;
      int8_t nox_index = -1;
      int8_t hcho_index = -1;
      int8_t co2_index = -1;
      bool co2_uint16 = false;
      switch (this->sen6x_type_) {
        case SEN62:
          break;
        case SEN63C:
          co2_index = 6;
          break;
        case SEN65:
          voc_index = 6;
          nox_index = 7;
          break;
        case SEN66:
          voc_index = 6;
          nox_index = 7;
          co2_index = 8;
          co2_uint16 = true;
          break;
        case SEN68:
          voc_index = 6;
          nox_index = 7;
          hcho_index = 8;
          break;
        case SEN69C:
          voc_index = 6;
          nox_index = 7;
          hcho_index = 8;
          co2_index = 9;
          break;
        default:
          break;
      }

      float pm_1_0 = measurements[0] / 10.0f;
      if (measurements[0] == 0xFFFF)
        pm_1_0 = NAN;
      float pm_2_5 = measurements[1] / 10.0f;
      if (measurements[1] == 0xFFFF)
        pm_2_5 = NAN;
      float pm_4_0 = measurements[2] / 10.0f;
      if (measurements[2] == 0xFFFF)
        pm_4_0 = NAN;
      float pm_10_0 = measurements[3] / 10.0f;
      if (measurements[3] == 0xFFFF)
        pm_10_0 = NAN;
      float humidity = static_cast<int16_t>(measurements[4]) / 100.0f;
      if (measurements[4] == 0x7FFF)
        humidity = NAN;
      float temperature = static_cast<int16_t>(measurements[5]) / 200.0f;
      if (measurements[5] == 0x7FFF)
        temperature = NAN;

      float voc = NAN;
      float nox = NAN;
      float hcho = NAN;
      float co2 = NAN;

      if (voc_index >= 0) {
        voc = static_cast<int16_t>(measurements[voc_index]) / 10.0f;
        if (measurements[voc_index] == 0x7FFF)
          voc = NAN;
      }
      if (nox_index >= 0) {
        nox = static_cast<int16_t>(measurements[nox_index]) / 10.0f;
        if (measurements[nox_index] == 0x7FFF)
          nox = NAN;
      }

      if (hcho_index >= 0) {
        const uint16_t hcho_raw = measurements[hcho_index];
        hcho = hcho_raw / 10.0f;
        if (hcho_raw == 0xFFFF)
          hcho = NAN;
      }

      if (co2_index >= 0) {
        if (co2_uint16) {
          const uint16_t co2_raw = measurements[co2_index];
          co2 = static_cast<float>(co2_raw);
          if (co2_raw == 0xFFFF)
            co2 = NAN;
        } else {
          const int16_t co2_raw = static_cast<int16_t>(measurements[co2_index]);
          co2 = static_cast<float>(co2_raw);
          if (co2_raw == 0x7FFF)
            co2 = NAN;
        }
      }

      this->last_pm_1_0_ = pm_1_0;
      this->last_pm_2_5_ = pm_2_5;
      this->last_pm_4_0_ = pm_4_0;
      this->last_pm_10_0_ = pm_10_0;
      this->last_temperature_ = temperature;
      this->last_humidity_ = humidity;
      this->last_voc_ = voc;
      this->last_nox_ = nox;
      this->last_hcho_ = hcho;
      this->last_co2_ = co2;
      this->has_last_values_ = true;

      const uint32_t check_time = App.get_loop_component_start_time();
      if (check_time < this->startup_stable_after_) {
        ESP_LOGV(TAG, "Startup stabilization in progress, skipping publish");
        const uint32_t remaining_ms = this->startup_stable_after_ - check_time;
        const uint32_t remaining_ms_clamped = remaining_ms < 1000 ? 0 : remaining_ms;
        ESP_LOGD(TAG, "Startup delay active (%u ms left), ignored values from sensor",
                 static_cast<unsigned>(remaining_ms_clamped));
        this->status_clear_warning();
        return;
      }

      if (this->pm_1_0_sensor_ != nullptr)
        this->pm_1_0_sensor_->publish_state(pm_1_0);
      if (this->pm_2_5_sensor_ != nullptr)
        this->pm_2_5_sensor_->publish_state(pm_2_5);
      if (this->pm_4_0_sensor_ != nullptr)
        this->pm_4_0_sensor_->publish_state(pm_4_0);
      if (this->pm_10_0_sensor_ != nullptr)
        this->pm_10_0_sensor_->publish_state(pm_10_0);
      if (this->temperature_sensor_ != nullptr)
        this->temperature_sensor_->publish_state(temperature);
      if (this->humidity_sensor_ != nullptr)
        this->humidity_sensor_->publish_state(humidity);
      if (this->voc_sensor_ != nullptr && voc_index >= 0)
        this->voc_sensor_->publish_state(voc);
      if (this->nox_sensor_ != nullptr && nox_index >= 0)
        this->nox_sensor_->publish_state(nox);
      if (this->hcho_sensor_ != nullptr && hcho_index >= 0)
        this->hcho_sensor_->publish_state(hcho);
      if (this->co2_sensor_ != nullptr && co2_index >= 0)
        this->co2_sensor_->publish_state(co2);

      this->status_clear_warning();
    });
  };

  (*poll_ready)(poll_retries);
}

bool SEN6XComponent::reset_device() {
  const uint32_t now = App.get_loop_component_start_time();
  if (this->last_stop_ms_ != 0 && (now - this->last_stop_ms_) < 50) {
    const uint32_t wait_ms = 50 - (now - this->last_stop_ms_);
    this->set_timeout(wait_ms, [this]() { this->reset_device(); });
    return true;
  }
  if (!this->write_command(SEN6X_CMD_RESET)) {
    this->status_set_warning();
    ESP_LOGE(TAG, "write error device reset (%d)", this->last_error_);
    return false;
  }
  this->set_timeout(1200, [this]() { ESP_LOGD(TAG, "Reset complete"); });
  return true;
}

bool SEN6XComponent::start_measurement() {
  const uint32_t now = App.get_loop_component_start_time();
  if (this->last_stop_ms_ != 0 && (now - this->last_stop_ms_) < 1400) {
    const uint32_t wait_ms = 1400 - (now - this->last_stop_ms_);
    this->set_timeout(wait_ms, [this]() { this->start_measurement(); });
    return true;
  }
  if (!this->write_command(SEN6X_CMD_START_MEASUREMENTS)) {
    this->status_set_warning();
    ESP_LOGE(TAG, "write error start measurement (%d)", this->last_error_);
    return false;
  }
  this->measurement_started_ = true;
  this->startup_stable_after_ = now + this->startup_delay_ms_;
  ESP_LOGD(TAG, "Measurement started");
  return true;
}

bool SEN6XComponent::stop_measurement() {
  if (!this->write_command(SEN6X_CMD_STOP_MEASUREMENTS)) {
    this->status_set_warning();
    ESP_LOGE(TAG, "write error stop measurement (%d)", this->last_error_);
    return false;
  }
  this->measurement_started_ = false;
  this->last_stop_ms_ = App.get_loop_component_start_time();
  ESP_LOGD(TAG, "Measurement stopped");
  return true;
}

}  // namespace sen6x
}  // namespace esphome

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sensirion_common/i2c_sensirion.h"
#include "esphome/core/application.h"

namespace esphome {
namespace sen6x {

enum ERRORCODE {
  COMMUNICATION_FAILED,
  SERIAL_NUMBER_IDENTIFICATION_FAILED,
  MEASUREMENT_INIT_FAILED,
  PRODUCT_NAME_FAILED,
  FIRMWARE_FAILED,
  UNKNOWN
};

class SEN6XComponent : public PollingComponent, public sensirion_common::SensirionI2CDevice {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;
  void update() override;

  enum Sen6xType { SEN62, SEN63C, SEN65, SEN66, SEN68, SEN69C, UNKNOWN };

  void set_pm_1_0_sensor(sensor::Sensor *pm_1_0) { pm_1_0_sensor_ = pm_1_0; }
  void set_pm_2_5_sensor(sensor::Sensor *pm_2_5) { pm_2_5_sensor_ = pm_2_5; }
  void set_pm_4_0_sensor(sensor::Sensor *pm_4_0) { pm_4_0_sensor_ = pm_4_0; }
  void set_pm_10_0_sensor(sensor::Sensor *pm_10_0) { pm_10_0_sensor_ = pm_10_0; }

  void set_voc_sensor(sensor::Sensor *voc_sensor) { voc_sensor_ = voc_sensor; }
  void set_nox_sensor(sensor::Sensor *nox_sensor) { nox_sensor_ = nox_sensor; }
  void set_hcho_sensor(sensor::Sensor *hcho_sensor) { hcho_sensor_ = hcho_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_co2_sensor(sensor::Sensor *co2) { co2_sensor_ = co2; }
  void set_startup_delay(uint32_t delay_ms) { startup_delay_ms_ = delay_ms; }
  bool is_measurement_running() const;
  const std::string &get_product_name() const { return this->product_name_; }
  const std::string &get_serial_number() const { return this->serial_number_; }
  uint8_t get_firmware_version_major() const { return this->firmware_version_major_; }
  uint8_t get_firmware_version_minor() const { return this->firmware_version_minor_; }
  bool get_state() const { return this->measurement_started_; }
  bool reset_device();
  bool start_measurement();
  bool stop_measurement();

 protected:
  void finish_setup_();

  ERRORCODE error_code_;
  bool initialized_{false};
  sensor::Sensor *pm_1_0_sensor_{nullptr};
  sensor::Sensor *pm_2_5_sensor_{nullptr};
  sensor::Sensor *pm_4_0_sensor_{nullptr};
  sensor::Sensor *pm_10_0_sensor_{nullptr};
  // SEN54 and SEN55 only
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *voc_sensor_{nullptr};
  // SEN55 only
  sensor::Sensor *nox_sensor_{nullptr};
  sensor::Sensor *hcho_sensor_{nullptr};
  sensor::Sensor *co2_sensor_{nullptr};
  std::string product_name_;
  Sen6xType sen6x_type_{UNKNOWN};
  std::string serial_number_;
  uint8_t firmware_version_major_{0};
  uint8_t firmware_version_minor_{0};
  bool measurement_started_{false};
  uint32_t startup_delay_ms_{60000};
  uint32_t startup_stable_after_{0};
  uint32_t last_stop_ms_{0};
  bool has_last_values_{false};
  float last_pm_1_0_{NAN};
  float last_pm_2_5_{NAN};
  float last_pm_4_0_{NAN};
  float last_pm_10_0_{NAN};
  float last_temperature_{NAN};
  float last_humidity_{NAN};
  float last_voc_{NAN};
  float last_nox_{NAN};
  float last_hcho_{NAN};
  float last_co2_{NAN};
};

}  // namespace sen6x
}  // namespace esphome

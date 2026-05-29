#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace icm20948 {

// Expected WHO_AM_I value for the ICM-20948
static const uint8_t ICM20948_WHO_AM_I = 0xEA;

// --- Register map (bank in high nibble of REG_BANK_SEL bits [5:4]) ---
static const uint8_t REG_BANK_SEL = 0x7F;
// Bank 0
static const uint8_t REG_WHO_AM_I = 0x00;
static const uint8_t REG_PWR_MGMT_1 = 0x06;
static const uint8_t REG_PWR_MGMT_2 = 0x07;
static const uint8_t REG_ACCEL_XOUT_H = 0x2D;  // 6 bytes: X/Y/Z high+low, big-endian
// Bank 2
static const uint8_t REG_ACCEL_CONFIG = 0x14;

enum AccelRange : uint8_t {
  ACCEL_RANGE_2G = 0,
  ACCEL_RANGE_4G = 1,
  ACCEL_RANGE_8G = 2,
  ACCEL_RANGE_16G = 3,
};

// Persisted across reboots
struct ICM20948CalibrationData {
  float pitch_offset;
  float roll_offset;
};

class ICM20948 : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_pitch_sensor(sensor::Sensor *s) { this->pitch_sensor_ = s; }
  void set_roll_sensor(sensor::Sensor *s) { this->roll_sensor_ = s; }
  void set_level_sensor(binary_sensor::BinarySensor *s) { this->level_sensor_ = s; }
  void set_accel_range(AccelRange r) { this->accel_range_ = r; }
  void set_level_threshold(float deg) { this->level_threshold_ = deg; }

  // Call these from a YAML lambda (e.g. a template button):
  //   id(imu).calibrate();          -> store current orientation as "level"
  //   id(imu).reset_calibration();  -> clear stored offsets
  void calibrate();
  void reset_calibration();

 protected:
  bool select_bank_(uint8_t bank);
  bool read_accel_(float &ax, float &ay, float &az);
  bool read_angles_(float &pitch, float &roll);
  float accel_scale_();  // LSB per g

  sensor::Sensor *pitch_sensor_{nullptr};
  sensor::Sensor *roll_sensor_{nullptr};
  binary_sensor::BinarySensor *level_sensor_{nullptr};

  AccelRange accel_range_{ACCEL_RANGE_2G};
  float level_threshold_{1.5f};

  float pitch_offset_{0.0f};
  float roll_offset_{0.0f};

  uint8_t current_bank_{0xFF};
  ESPPreferenceObject pref_;
};

}  // namespace icm20948
}  // namespace esphome

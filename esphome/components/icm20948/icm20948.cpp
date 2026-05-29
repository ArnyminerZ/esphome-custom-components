#include "icm20948.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cmath>

namespace esphome {
namespace icm20948 {

static const char *const TAG = "icm20948";
static const float RAD_TO_DEG = 57.29577951308232f;

bool ICM20948::select_bank_(uint8_t bank) {
  if (bank == this->current_bank_)
    return true;
  // Bank number occupies bits [5:4] of REG_BANK_SEL
  if (this->write_byte(REG_BANK_SEL, (uint8_t) (bank << 4))) {
    this->current_bank_ = bank;
    return true;
  }
  return false;
}

float ICM20948::accel_scale_() {
  switch (this->accel_range_) {
    case ACCEL_RANGE_2G:
      return 16384.0f;
    case ACCEL_RANGE_4G:
      return 8192.0f;
    case ACCEL_RANGE_8G:
      return 4096.0f;
    case ACCEL_RANGE_16G:
      return 2048.0f;
  }
  return 16384.0f;
}

bool ICM20948::read_accel_(float &ax, float &ay, float &az) {
  if (!this->select_bank_(0))
    return false;

  uint8_t raw[6];
  if (!this->read_bytes(REG_ACCEL_XOUT_H, raw, 6))
    return false;

  int16_t rx = (int16_t) ((raw[0] << 8) | raw[1]);
  int16_t ry = (int16_t) ((raw[2] << 8) | raw[3]);
  int16_t rz = (int16_t) ((raw[4] << 8) | raw[5]);

  const float scale = this->accel_scale_();
  ax = rx / scale;
  ay = ry / scale;
  az = rz / scale;
  return true;
}

bool ICM20948::read_angles_(float &pitch, float &roll) {
  float ax, ay, az;
  if (!this->read_accel_(ax, ay, az))
    return false;

  // Tilt of the gravity vector, in degrees.
  // Z is assumed roughly "up" when the board lies flat.
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  roll = atan2f(ay, az) * RAD_TO_DEG;
  return true;
}

void ICM20948::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ICM20948...");
  this->current_bank_ = 0xFF;  // force first bank write

  if (!this->select_bank_(0)) {
    ESP_LOGE(TAG, "I2C bank select failed");
    this->mark_failed();
    return;
  }

  uint8_t who = 0;
  if (!this->read_byte(REG_WHO_AM_I, &who) || who != ICM20948_WHO_AM_I) {
    ESP_LOGE(TAG, "WHO_AM_I check failed (got 0x%02X, expected 0x%02X)", who, ICM20948_WHO_AM_I);
    this->mark_failed();
    return;
  }

  // Reset the device, then wait for it to come back.
  this->write_byte(REG_PWR_MGMT_1, 0x80);
  delay(100);  // NOLINT
  this->current_bank_ = 0xFF;
  this->select_bank_(0);

  // Wake up (clear SLEEP) and auto-select the best clock source.
  this->write_byte(REG_PWR_MGMT_1, 0x01);
  delay(10);  // NOLINT
  // Enable all accel + gyro axes.
  this->write_byte(REG_PWR_MGMT_2, 0x00);

  // Configure accelerometer: low-pass filter (~5.7 Hz) + full-scale range.
  // ACCEL_CONFIG = [5:3]=DLPFCFG, [2:1]=FS_SEL, [0]=FCHOICE(1=enable DLPF)
  this->select_bank_(2);
  uint8_t accel_cfg = (uint8_t) ((0x06 << 3) | (this->accel_range_ << 1) | 0x01);
  this->write_byte(REG_ACCEL_CONFIG, accel_cfg);

  this->select_bank_(0);

  // Restore tare offsets from flash.
  this->pref_ = global_preferences->make_preference<ICM20948CalibrationData>(fnv1_hash("icm20948_calibration"));
  ICM20948CalibrationData data{};
  if (this->pref_.load(&data)) {
    this->pitch_offset_ = data.pitch_offset;
    this->roll_offset_ = data.roll_offset;
    ESP_LOGCONFIG(TAG, "Restored calibration: pitch_offset=%.2f roll_offset=%.2f", this->pitch_offset_,
                  this->roll_offset_);
  }
}

void ICM20948::update() {
  float pitch, roll;
  if (!this->read_angles_(pitch, roll)) {
    ESP_LOGW(TAG, "Failed to read accelerometer");
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();

  pitch -= this->pitch_offset_;
  roll -= this->roll_offset_;

  if (this->pitch_sensor_ != nullptr)
    this->pitch_sensor_->publish_state(pitch);
  if (this->roll_sensor_ != nullptr)
    this->roll_sensor_->publish_state(roll);
}

void ICM20948::calibrate() {
  float pitch, roll;
  if (!this->read_angles_(pitch, roll)) {
    ESP_LOGW(TAG, "Calibration failed: could not read accelerometer");
    return;
  }
  // Store the current raw orientation as the new "level" reference.
  this->pitch_offset_ = pitch;
  this->roll_offset_ = roll;

  ICM20948CalibrationData data{this->pitch_offset_, this->roll_offset_};
  this->pref_.save(&data);
  ESP_LOGI(TAG, "Calibrated. New offsets: pitch=%.2f roll=%.2f", this->pitch_offset_, this->roll_offset_);
}

void ICM20948::reset_calibration() {
  this->pitch_offset_ = 0.0f;
  this->roll_offset_ = 0.0f;
  ICM20948CalibrationData data{0.0f, 0.0f};
  this->pref_.save(&data);
  ESP_LOGI(TAG, "Calibration reset to zero");
}

void ICM20948::dump_config() {
  ESP_LOGCONFIG(TAG, "ICM20948:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Accel range: +/-%dG", 2 << this->accel_range_);
  ESP_LOGCONFIG(TAG, "  Calibration offsets: pitch=%.2f roll=%.2f", this->pitch_offset_, this->roll_offset_);
  LOG_SENSOR("  ", "Pitch", this->pitch_sensor_);
  LOG_SENSOR("  ", "Roll", this->roll_sensor_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Communication with ICM20948 failed!");
  }
}

}  // namespace icm20948
}  // namespace esphome

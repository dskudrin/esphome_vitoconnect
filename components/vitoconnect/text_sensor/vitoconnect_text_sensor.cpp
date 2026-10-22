#include "vitoconnect_text_sensor.h"

#include "esphome/core/log.h"

#include <cstdio>

namespace esphome {
namespace vitoconnect {

static const char *TAG = "vitoconnect.text_sensor";

OPTOLINKTextSensor::OPTOLINKTextSensor() {
  // empty
}

OPTOLINKTextSensor::~OPTOLINKTextSensor() {
  // empty
}

uint8_t OPTOLINKTextSensor::bcd_to_dec_(uint8_t value) {
  return ((value >> 4) * 10) + (value & 0x0F);
}

bool OPTOLINKTextSensor::valid_bcd_(uint8_t value) {
  return ((value >> 4) <= 9) && ((value & 0x0F) <= 9);
}

std::string OPTOLINKTextSensor::hex_(uint8_t *data, uint8_t length) {
  std::string out;
  char byte_text[4];

  for (uint8_t i = 0; i < length; i++) {
    if (i != 0) out += ":";
    std::snprintf(byte_text, sizeof(byte_text), "%02X", data[i]);
    out += byte_text;
  }

  return out;
}

bool OPTOLINKTextSensor::datetime_(uint8_t *data, uint8_t length, std::string &out) {
  if (length != 8) return false;

  for (uint8_t i = 0; i < 8; i++) {
    if (!valid_bcd_(data[i])) return false;
  }

  const int year = bcd_to_dec_(data[0]) * 100 + bcd_to_dec_(data[1]);
  const int month = bcd_to_dec_(data[2]);
  const int day = bcd_to_dec_(data[3]);
  const int hour = bcd_to_dec_(data[5]);
  const int minute = bcd_to_dec_(data[6]);
  const int second = bcd_to_dec_(data[7]);

  if (month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 59) {
    return false;
  }

  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
                year, month, day, hour, minute, second);
  out = buffer;
  return true;
}

void OPTOLINKTextSensor::decode(uint8_t *data, uint8_t length, Datapoint *dp) {
  if (!dp) dp = this;

  if (length < this->_length) {
    ESP_LOGW(TAG, "Received only %u bytes for datapoint 0x%04X, expected %u",
             length, this->_address, this->_length);
    return;
  }

  if (this->format_ == "hex") {
    this->publish_state(hex_(data, this->_length));
    return;
  }

  if (this->format_ == "datetime") {
    std::string value;
    if (datetime_(data, this->_length, value)) {
      this->publish_state(value);
    } else {
      ESP_LOGW(TAG, "Invalid datetime payload for datapoint 0x%04X: %s",
               this->_address, hex_(data, this->_length).c_str());
    }
    return;
  }

  if (this->format_ == "error_code") {
    if (this->_length < 1) return;

    char buffer[3];
    std::snprintf(buffer, sizeof(buffer), "%02X", data[0]);
    this->publish_state(buffer);
    return;
  }

  if (this->format_ == "error") {
    if (this->_length != 9) {
      ESP_LOGW(TAG, "Error format requires 9 bytes for datapoint 0x%04X, got %u",
               this->_address, this->_length);
      return;
    }

    char code[3];
    std::snprintf(code, sizeof(code), "%02X", data[0]);

    std::string timestamp;
    if (!datetime_(&data[1], 8, timestamp)) {
      this->publish_state(std::string(code) + "|" + hex_(&data[1], 8));
      return;
    }

    this->publish_state(std::string(code) + "|" + timestamp);
    return;
  }

  ESP_LOGW(TAG, "Unknown text sensor format '%s' for datapoint 0x%04X",
           this->format_.c_str(), this->_address);
}

}  // namespace vitoconnect
}  // namespace esphome

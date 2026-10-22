#pragma once

#include "esphome/components/text_sensor/text_sensor.h"
#include "../vitoconnect_datapoint.h"

#include <string>

namespace esphome {
namespace vitoconnect {

class OPTOLINKTextSensor : public text_sensor::TextSensor, public Datapoint {
 public:
  OPTOLINKTextSensor();
  ~OPTOLINKTextSensor();

  void set_format(const std::string &format) { this->format_ = format; }

  void decode(uint8_t *data, uint8_t length, Datapoint *dp = nullptr) override;

 protected:
  std::string format_{"hex"};

  static uint8_t bcd_to_dec_(uint8_t value);
  static bool valid_bcd_(uint8_t value);
  static std::string hex_(uint8_t *data, uint8_t length);
  static bool datetime_(uint8_t *data, uint8_t length, std::string &out);
};

}  // namespace vitoconnect
}  // namespace esphome

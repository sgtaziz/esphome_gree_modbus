#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

namespace esphome {
namespace gree_ac {

// Thin Number subclass used only so ESPHome codegen can register a number
// entity. Real value handling is driven by the parent GreeAC component.
class GreeACNumber : public number::Number, public Component {
 protected:
  void control(float value) override { this->publish_state(value); }
};

}  // namespace gree_ac
}  // namespace esphome

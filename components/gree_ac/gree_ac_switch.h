#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace gree_ac {

class GreeACSwitch : public switch_::Switch, public Component {
 protected:
  void write_state(bool state) override { this->publish_state(state); }
};

}  // namespace gree_ac
}  // namespace esphome

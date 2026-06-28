#pragma once

#include "esphome/components/button/button.h"
#include "esphome/core/component.h"

namespace esphome {
namespace gree_ac {

// Thin Button subclass used only so ESPHome codegen can register a button
// entity. The press action is wired to the parent GreeAC component via
// add_on_press_callback.
class GreeACButton : public button::Button, public Component {
 protected:
  void press_action() override {}
};

}  // namespace gree_ac
}  // namespace esphome

#include "constant_fan_switch.h"
#include "../air_conditioner.h"

#include "esphome/core/log.h"

namespace esphome {
namespace midea {
namespace ac {

static const char *const TAG = "midea_xye.switch";

void ConstantFanSwitch::setup() {
  bool initial_state = this->parent_->vrf_protocol_available();
  auto restored_state = this->get_initial_state_with_restore_mode();
  if (restored_state.has_value()) {
    initial_state = restored_state.value();
  }
  this->parent_->set_constant_fan(initial_state);
}

void ConstantFanSwitch::dump_config() { LOG_SWITCH("", "Midea XYE Constant Fan", this); }

void ConstantFanSwitch::write_state(bool state) { this->parent_->set_constant_fan(state); }

}  // namespace ac
}  // namespace midea
}  // namespace esphome

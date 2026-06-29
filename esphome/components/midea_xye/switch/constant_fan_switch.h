#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "../air_conditioner.h"

namespace esphome {
namespace midea {
namespace ac {

class ConstantFanSwitch : public switch_::Switch, public Component, public Parented<AirConditioner> {
 public:
  ConstantFanSwitch() = default;

  void setup() override;
  void dump_config() override;

 protected:
  void write_state(bool state) override;
};

}  // namespace ac
}  // namespace midea
}  // namespace esphome

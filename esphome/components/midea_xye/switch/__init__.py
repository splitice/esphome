import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from ..climate import AirConditioner, midea_ac_ns

UseFahrenheitSwitch = midea_ac_ns.class_("UseFahrenheitSwitch", switch.Switch)
ConstantFanSwitch = midea_ac_ns.class_("ConstantFanSwitch", switch.Switch, cg.Component)

CONF_MIDEA_AC_ID = "midea_ac_id"
CONF_CONSTANT_FAN = "constant_fan"
CONF_USE_FAHRENHEIT = "use_fahrenheit"
ICON_FAN = "mdi:fan"
ICON_THERMOMETER = "mdi:thermometer"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MIDEA_AC_ID): cv.use_id(AirConditioner),
        cv.Optional(CONF_USE_FAHRENHEIT): switch.switch_schema(
            UseFahrenheitSwitch,
            icon=ICON_THERMOMETER,
            entity_category=ENTITY_CATEGORY_CONFIG,
            default_restore_mode="DISABLED",
        ),
        cv.Optional(CONF_CONSTANT_FAN): switch.switch_schema(
            ConstantFanSwitch,
            icon=ICON_FAN,
            entity_category=ENTITY_CATEGORY_CONFIG,
            default_restore_mode="RESTORE_DEFAULT_ON",
        ).extend(cv.COMPONENT_SCHEMA),
    }
).add_extra(cv.has_at_least_one_key(CONF_USE_FAHRENHEIT, CONF_CONSTANT_FAN))


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MIDEA_AC_ID])
    if CONF_USE_FAHRENHEIT in config:
        sw_var = await switch.new_switch(config[CONF_USE_FAHRENHEIT])
        await cg.register_parented(sw_var, parent)
        cg.add(parent.set_use_fahrenheit_switch(sw_var))
    if CONF_CONSTANT_FAN in config:
        sw_var = await switch.new_switch(config[CONF_CONSTANT_FAN])
        await cg.register_component(sw_var, config[CONF_CONSTANT_FAN])
        await cg.register_parented(sw_var, parent)
        cg.add(parent.set_constant_fan_switch(sw_var))

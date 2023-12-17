import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, sensirion_common
from esphome.const import (
    CONF_ID,
    CONF_ILLUMINANCE,
    CONF_COLOR_TEMPERATURE,
    CONF_GLASS_ATTENUATION_FACTOR,
    CONF_INTEGRATION_TIME,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
    UNIT_KELVIN,
    ICON_LIGHTBULB,
    ICON_THERMOMETER,
    DEVICE_CLASS_ILLUMINANCE,
)

CODEOWNERS = ["@airlytix"]
DEPENDENCIES = ["i2c"]

veml6040_ns = cg.esphome_ns.namespace("veml6040")

VEML6040Component = veml6040_ns.class_(
    "VEML6040Component", cg.PollingComponent, i2c.I2CDevice
)


VEML6040IntegrationTime = veml6040_ns.enum("VEML6040IntegrationTime")
VEML6040_INTEGRATION_TIMES = {
    "40ms": VEML6040IntegrationTime.VEML6040_INTEGRATION_TIME_40MS,
    "80ms": VEML6040IntegrationTime.VEML6040_INTEGRATION_TIME_80MS,
    "160ms": VEML6040IntegrationTime.VEML6040_INTEGRATION_TIME_160MS,
    "320ms": VEML6040IntegrationTime.VEML6040_INTEGRATION_TIME_320MS,
    "640ms": VEML6040IntegrationTime.VEML6040_INTEGRATION_TIME_640MS,
    "1280ms": VEML6040IntegrationTime.VEML6040_INTEGRATION_TIME_1280MS,
}


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VEML6040Component),
            cv.Optional(CONF_ILLUMINANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_LUX,
                icon=ICON_LIGHTBULB,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COLOR_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_KELVIN,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_INTEGRATION_TIME, default="80ms"): cv.enum(
                VEML6040_INTEGRATION_TIMES, lower=True
            ),
            cv.Optional(CONF_GLASS_ATTENUATION_FACTOR, default=0.5): cv.float_range(
                min=0.5
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x10))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_integration_time(config[CONF_INTEGRATION_TIME]))
    cg.add(var.set_glass_attenuation_factor(config[CONF_GLASS_ATTENUATION_FACTOR]))

    if CONF_ILLUMINANCE in config:
        sens = await sensor.new_sensor(config[CONF_ILLUMINANCE])
        cg.add(var.set_illuminance_sensor(sens))
    if CONF_COLOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_COLOR_TEMPERATURE])
        cg.add(var.set_color_temperature_sensor(sens))
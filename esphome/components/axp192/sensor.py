import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_BATTERY_VOLTAGE,
    CONF_TEMPERATURE,
    UNIT_CELSIUS,
    UNIT_VOLT,
    ICON_THERMOMETER,
    ICON_BATTERY,
)

DEPENDENCIES = ["i2c"]

axp192_ns = cg.esphome_ns.namespace("axp192")

AXP192Component = axp192_ns.class_(
    "AXP192Component", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AXP192Component),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                icon=ICON_BATTERY,
                accuracy_decimals=2,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x77))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_BATTERY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE])
        cg.add(var.set_battery_voltage_sensor(sens))
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_internal_temperature_sensor(sens))

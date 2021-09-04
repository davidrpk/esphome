import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c  # , sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c"]

axp192_ns = cg.esphome_ns.namespace("axp192")

AXP192Component = axp192_ns.class_(
    "AXP192Component", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(AXP192Component)})
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x77))
)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

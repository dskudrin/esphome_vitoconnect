import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ADDRESS, CONF_LENGTH

from .. import CONF_VITOCONNECT_ID, VitoConnect, vitoconnect_ns

DEPENDENCIES = ["vitoconnect"]

CONF_FORMAT = "format"

OPTOLINKTextSensor = vitoconnect_ns.class_("OPTOLINKTextSensor", text_sensor.TextSensor)

FORMATS = {
    "hex": "hex",
    "datetime": "datetime",
    "error": "error",
    "error_code": "error_code",
}

CONFIG_SCHEMA = text_sensor.text_sensor_schema(OPTOLINKTextSensor).extend(
    {
        cv.GenerateID(): cv.declare_id(OPTOLINKTextSensor),
        cv.GenerateID(CONF_VITOCONNECT_ID): cv.use_id(VitoConnect),
        cv.Required(CONF_ADDRESS): cv.uint16_t,
        cv.Required(CONF_LENGTH): cv.uint8_t,
        cv.Optional(CONF_FORMAT, default="hex"): cv.one_of(*FORMATS, lower=True),
    }
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)

    cg.add(var.setAddress(config[CONF_ADDRESS]))
    cg.add(var.setLength(config[CONF_LENGTH]))
    cg.add(var.set_format(config[CONF_FORMAT]))

    hub = await cg.get_variable(config[CONF_VITOCONNECT_ID])
    cg.add(hub.register_datapoint(var))

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_DEGREES,
    ICON_SCREEN_ROTATION,
)

CODEOWNERS = ["@ArnyminerZ"]

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

CONF_PITCH = "pitch"
CONF_ROLL = "roll"
CONF_ACCEL_RANGE = "accel_range"

icm20948_ns = cg.esphome_ns.namespace("icm20948")
ICM20948 = icm20948_ns.class_("ICM20948", cg.PollingComponent, i2c.I2CDevice)

AccelRange = icm20948_ns.enum("AccelRange")
ACCEL_RANGES = {
    "2G": AccelRange.ACCEL_RANGE_2G,
    "4G": AccelRange.ACCEL_RANGE_4G,
    "8G": AccelRange.ACCEL_RANGE_8G,
    "16G": AccelRange.ACCEL_RANGE_16G,
}

angle_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ICM20948),
            cv.Optional(CONF_PITCH): angle_schema,
            cv.Optional(CONF_ROLL): angle_schema,
            cv.Optional(CONF_ACCEL_RANGE, default="2G"): cv.enum(
                ACCEL_RANGES, upper=True
            ),
        }
    )
    .extend(cv.polling_component_schema("250ms"))
    .extend(i2c.i2c_device_schema(0x68))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_accel_range(config[CONF_ACCEL_RANGE]))

    if CONF_PITCH in config:
        sens = await sensor.new_sensor(config[CONF_PITCH])
        cg.add(var.set_pitch_sensor(sens))
    if CONF_ROLL in config:
        sens = await sensor.new_sensor(config[CONF_ROLL])
        cg.add(var.set_roll_sensor(sens))
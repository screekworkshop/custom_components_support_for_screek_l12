import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_ENERGY,
    UNIT_CENTIMETER,
    UNIT_PERCENT,
    STATE_CLASS_NONE, # 光线加入(23年3月13日_17时20分_)
    ICON_BRIGHTNESS_5, # 光线的图标。(*23年4月17日*17时57分)
    UNIT_EMPTY,
    UNIT_LUX,

)
from . import CONF_LD2412_ID, LD2412Component

DEPENDENCIES = ["ld2412"]
CONF_MOVING_DISTANCE = "moving_distance"
CONF_STILL_DISTANCE = "still_distance"
CONF_MOVING_ENERGY = "moving_energy"
CONF_STILL_ENERGY = "still_energy"
CONF_LIGHT = "luminance"

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_LD2412_ID): cv.use_id(LD2412Component),
    cv.Optional(CONF_MOVING_DISTANCE): sensor.sensor_schema(
        device_class=DEVICE_CLASS_DISTANCE, unit_of_measurement=UNIT_CENTIMETER
    ),
    cv.Optional(CONF_STILL_DISTANCE): sensor.sensor_schema(
        device_class=DEVICE_CLASS_DISTANCE, unit_of_measurement=UNIT_CENTIMETER
    ),
    cv.Optional(CONF_MOVING_ENERGY): sensor.sensor_schema(
        device_class=STATE_CLASS_NONE, unit_of_measurement=UNIT_PERCENT
    ),
    cv.Optional(CONF_STILL_ENERGY): sensor.sensor_schema(
        device_class=STATE_CLASS_NONE, unit_of_measurement=UNIT_PERCENT
    ),
    # 引入光线单位。(23年3月13日_17时33分_)
    cv.Optional(CONF_LIGHT): sensor.sensor_schema(
        device_class="illuminance",  # 设置为光线传感器类别
        icon=ICON_BRIGHTNESS_5,      # 保持亮度图标
        unit_of_measurement=UNIT_LUX # 设置单位为lux
    ),
}

async def to_code(config):
    ld2412_component = await cg.get_variable(config[CONF_LD2412_ID])
    if CONF_MOVING_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_MOVING_DISTANCE])
        cg.add(ld2412_component.set_moving_target_distance_sensor(sens))
    if CONF_STILL_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_STILL_DISTANCE])
        cg.add(ld2412_component.set_still_target_distance_sensor(sens))
    if CONF_MOVING_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_MOVING_ENERGY])
        cg.add(ld2412_component.set_moving_target_energy_sensor(sens))
    if CONF_STILL_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_STILL_ENERGY])
        cg.add(ld2412_component.set_still_target_energy_sensor(sens))
    # 光线读取(23年3月13日_17时24分_)
    if CONF_LIGHT in config:
        sens = await sensor.new_sensor(config[CONF_LIGHT])
        cg.add(ld2412_component.set_light_sensor(sens))
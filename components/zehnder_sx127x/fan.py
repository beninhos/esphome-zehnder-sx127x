import esphome.codegen as cg
from esphome.components import binary_sensor, fan, sensor
from esphome.components.sx127x import CONF_SX127X_ID, SX127x, SX127xListener
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL_MILLIWATT,
)

DEPENDENCIES = ["sx127x"]
AUTO_LOAD = ["binary_sensor", "sensor"]

CONF_NETWORK_ID = "network_id"
CONF_FAN_ID = "fan_id"
CONF_REMOTE_ID = "remote_id"
CONF_QUERY_INTERVAL = "query_interval"
CONF_CONFIRMED = "confirmed"
CONF_SIGNAL = "signal"
CONF_NOISE = "noise"

RSSI_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

zehnder_sx127x_ns = cg.esphome_ns.namespace("zehnder_sx127x")
ZehnderSX127x = zehnder_sx127x_ns.class_(
    "ZehnderSX127x", fan.Fan, cg.Component, SX127xListener
)

CONFIG_SCHEMA = (
    fan.fan_schema(ZehnderSX127x, default_restore_mode="NO_RESTORE")
    .extend(
        {
            cv.GenerateID(CONF_SX127X_ID): cv.use_id(SX127x),
            # Network address and fan ID as seen on air; read them from a verbose log while pressing the RFZ.
            cv.Required(CONF_NETWORK_ID): cv.hex_uint32_t,
            cv.Required(CONF_FAN_ID): cv.hex_uint8_t,
            # Our own remote ID: any value 0x01-0xFE that no RFZ in the house uses. No pairing needed.
            cv.Required(CONF_REMOTE_ID): cv.All(cv.hex_uint8_t, cv.Range(min=0x01, max=0xFE)),
            cv.Optional(
                CONF_QUERY_INTERVAL, default="60s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CONFIRMED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY
            ),
            # Signal strength of the fan's replies here, and the background level between frames (every 10 s).
            cv.Optional(CONF_SIGNAL): RSSI_SCHEMA,
            cv.Optional(CONF_NOISE): RSSI_SCHEMA,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await fan.new_fan(config)
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_SX127X_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_network_id(config[CONF_NETWORK_ID]))
    cg.add(var.set_fan_id(config[CONF_FAN_ID]))
    cg.add(var.set_remote_id(config[CONF_REMOTE_ID]))
    cg.add(var.set_query_interval(config[CONF_QUERY_INTERVAL]))
    if CONF_CONFIRMED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONFIRMED])
        cg.add(var.set_confirmed_binary_sensor(sens))
    if CONF_SIGNAL in config:
        sens = await sensor.new_sensor(config[CONF_SIGNAL])
        cg.add(var.set_signal_sensor(sens))
    if CONF_NOISE in config:
        sens = await sensor.new_sensor(config[CONF_NOISE])
        cg.add(var.set_noise_sensor(sens))

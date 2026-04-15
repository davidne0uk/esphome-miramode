import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, sensor, switch, number, button
from esphome.const import (
    CONF_ID, CONF_NAME,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

DEPENDENCIES  = ["ble_client"]
AUTO_LOAD     = ["sensor", "switch", "number", "button"]
MULTI_CONF    = True

miramode_ns   = cg.esphome_ns.namespace("miramode")

MiraModeDevice = miramode_ns.class_(
    "MiraModeDevice", cg.PollingComponent, ble_client.BLEClientNode
)
MiraModeSwitch = miramode_ns.class_("MiraModeSwitch", switch.Switch, cg.Component)
MiraModeSensor = miramode_ns.class_("MiraModeSensor", sensor.Sensor, cg.Component)
MiraModeNumber = miramode_ns.class_("MiraModeNumber", number.Number, cg.Component)
MiraModeButton = miramode_ns.class_("MiraModeButton", button.Button, cg.Component)

CONF_CLIENT_NAME = "client_name"
CONF_OUTLET1     = "outlet1"
CONF_OUTLET2     = "outlet2"
CONF_ACTUAL_TEMP = "actual_temperature"
CONF_TARGET_TEMP = "target_temperature"
CONF_PAIR_BUTTON = "pair_button"
CONF_MIN_TEMP    = "min_value"
CONF_MAX_TEMP    = "max_value"
CONF_STEP        = "step"

_OUTLET_SCHEMA = switch.switch_schema(MiraModeSwitch)

_ACTUAL_TEMP_SCHEMA = sensor.sensor_schema(
    MiraModeSensor,
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

_TARGET_TEMP_SCHEMA = (
    number.number_schema(MiraModeNumber)
    .extend({
        cv.Optional(CONF_MIN_TEMP, default=20.0): cv.float_,
        cv.Optional(CONF_MAX_TEMP, default=48.0): cv.float_,
        cv.Optional(CONF_STEP,     default=0.5):  cv.positive_float,
    })
)

_PAIR_BUTTON_SCHEMA = button.button_schema(MiraModeButton)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(MiraModeDevice),
        cv.Required(CONF_NAME): cv.string,
        cv.Optional(CONF_CLIENT_NAME, default="ESPHome"): cv.string,
        cv.Optional(CONF_OUTLET1):     _OUTLET_SCHEMA,
        cv.Optional(CONF_OUTLET2):     _OUTLET_SCHEMA,
        cv.Optional(CONF_ACTUAL_TEMP): _ACTUAL_TEMP_SCHEMA,
        cv.Optional(CONF_TARGET_TEMP): _TARGET_TEMP_SCHEMA,
        cv.Optional(CONF_PAIR_BUTTON): _PAIR_BUTTON_SCHEMA,
    })
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.polling_component_schema("30s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_client_name(config[CONF_CLIENT_NAME]))
    # nvs_key is derived from the component's unique YAML id (str form)
    # so each instance gets a distinct NVS namespace regardless of name
    cg.add(var.set_nvs_key(str(config[CONF_ID])))

    if CONF_OUTLET1 in config:
        sw = await switch.new_switch(config[CONF_OUTLET1])
        cg.add(sw.set_parent(var))
        cg.add(sw.set_outlet(1))
        cg.add(var.set_outlet1_switch(sw))

    if CONF_OUTLET2 in config:
        sw = await switch.new_switch(config[CONF_OUTLET2])
        cg.add(sw.set_parent(var))
        cg.add(sw.set_outlet(2))
        cg.add(var.set_outlet2_switch(sw))

    if CONF_ACTUAL_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_ACTUAL_TEMP])
        cg.add(sens.set_parent(var))
        cg.add(var.set_actual_temp_sensor(sens))

    if CONF_TARGET_TEMP in config:
        cfg = config[CONF_TARGET_TEMP]
        num = await number.new_number(
            cfg,
            min_value=cfg[CONF_MIN_TEMP],
            max_value=cfg[CONF_MAX_TEMP],
            step=cfg[CONF_STEP],
        )
        cg.add(num.set_parent(var))
        cg.add(var.set_target_temp_number(num))

    if CONF_PAIR_BUTTON in config:
        btn = await button.new_button(config[CONF_PAIR_BUTTON])
        cg.add(btn.set_parent(var))
        cg.add(var.set_pair_button(btn))

"""
Schema validation tests for miramode __init__.py.
Tests schema acceptance/rejection using ESPHome's cv validators directly.
Does not call to_code() — schema-only validation.

Note: ESPHome's ble_client component imports esp32_ble_tracker at module level,
which queries CORE.data for target_framework. We pre-seed CORE before any
esphome component imports to avoid a KeyError during collection.
"""
import sys
import os
import pytest

# Add project root to path so components.miramode can be imported
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# Pre-seed ESPHome CORE to satisfy esp32_ble_tracker import-time check.
# Without this, importing ble_client raises KeyError('core') before any
# test runs. We declare esp-idf; the value doesn't affect schema validation.
from esphome.core import CORE, KEY_CORE, KEY_TARGET_FRAMEWORK  # noqa: E402
CORE.data.setdefault(KEY_CORE, {})[KEY_TARGET_FRAMEWORK] = "esp-idf"

import esphome.config_validation as cv
from components.miramode import (
    CONFIG_SCHEMA,
    CONF_CLIENT_NAME,
    CONF_OUTLET1,
    CONF_OUTLET2,
    CONF_ACTUAL_TEMP,
    CONF_TARGET_TEMP,
    CONF_PAIR_BUTTON,
    CONF_MIN_TEMP,
    CONF_MAX_TEMP,
    CONF_STEP,
)


def valid_base():
    """Minimal valid config (only required fields)."""
    return {
        "name": "Master Shower",
        "ble_client_id": "shower_ble",
        "update_interval": "30s",
    }


def test_name_required():
    cfg = valid_base()
    del cfg["name"]
    with pytest.raises((cv.Invalid, Exception)):
        CONFIG_SCHEMA(cfg)


def test_client_name_defaults_to_esphome():
    cfg = valid_base()
    result = CONFIG_SCHEMA(cfg)
    assert result[CONF_CLIENT_NAME] == "ESPHome"


def test_outlet1_optional():
    result = CONFIG_SCHEMA(valid_base())
    assert CONF_OUTLET1 not in result


def test_outlet2_optional():
    result = CONFIG_SCHEMA(valid_base())
    assert CONF_OUTLET2 not in result


def test_target_temp_defaults():
    cfg = valid_base()
    cfg[CONF_TARGET_TEMP] = {"name": "Target Temp"}
    result = CONFIG_SCHEMA(cfg)
    assert result[CONF_TARGET_TEMP][CONF_MIN_TEMP] == 20.0
    assert result[CONF_TARGET_TEMP][CONF_MAX_TEMP] == 48.0
    assert result[CONF_TARGET_TEMP][CONF_STEP]     == 0.5


def test_target_temp_custom_range():
    cfg = valid_base()
    cfg[CONF_TARGET_TEMP] = {
        "name": "Target Temp",
        CONF_MIN_TEMP: 25.0,
        CONF_MAX_TEMP: 45.0,
        CONF_STEP: 1.0,
    }
    result = CONFIG_SCHEMA(cfg)
    assert result[CONF_TARGET_TEMP][CONF_MIN_TEMP] == 25.0


def test_step_must_be_positive():
    cfg = valid_base()
    cfg[CONF_TARGET_TEMP] = {"name": "Target Temp", CONF_STEP: -1.0}
    with pytest.raises((cv.Invalid, Exception)):
        CONFIG_SCHEMA(cfg)


def test_multi_conf_two_showers():
    cfg1 = {**valid_base(), "name": "Ensuite Shower", "ble_client_id": "ble_1"}
    cfg2 = {**valid_base(), "name": "Master Shower",  "ble_client_id": "ble_2"}
    r1 = CONFIG_SCHEMA(cfg1)
    r2 = CONFIG_SCHEMA(cfg2)
    assert r1["name"] == "Ensuite Shower"
    assert r2["name"] == "Master Shower"

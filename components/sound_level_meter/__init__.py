
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_SENSORS,
    CONF_FILTERS,
    CONF_WINDOW_SIZE,
    CONF_UPDATE_INTERVAL,
    CONF_TYPE,
    UNIT_DECIBEL,
    STATE_CLASS_MEASUREMENT
)
from esphome.components.i2s_audio import (
    CONF_I2S_DIN_PIN,
    CONF_RIGHT,
    I2SAudioIn,
    i2s_audio_component_schema,
    i2s_audio_ns,
    register_i2s_audio_component,
)


CODEOWNERS = ["@mikenorgate"]
DEPENDENCIES = ["esp32", "i2s_audio"]
AUTO_LOAD = ["sensor"]
MULTI_CONF = True

sound_level_meter_ns = cg.esphome_ns.namespace("sound_level_meter")
SoundLevelMeter = sound_level_meter_ns.class_("SoundLevelMeter", cg.Component)
SoundLevelMeterSensor = sound_level_meter_ns.class_("SoundLevelMeterSensor", sensor.Sensor)
SoundLevelMeterSensorEq = sound_level_meter_ns.class_("SoundLevelMeterSensorEq", SoundLevelMeterSensor, sensor.Sensor)
SoundLevelMeterSensorMax = sound_level_meter_ns.class_("SoundLevelMeterSensorMax", SoundLevelMeterSensor, sensor.Sensor)
SoundLevelMeterSensorMin = sound_level_meter_ns.class_("SoundLevelMeterSensorMin", SoundLevelMeterSensor, sensor.Sensor)
SoundLevelMeterSensorPeak = sound_level_meter_ns.class_(
    "SoundLevelMeterSensorPeak", SoundLevelMeterSensor, sensor.Sensor)
SensorGroup = sound_level_meter_ns.class_("SensorGroup")
Filter = sound_level_meter_ns.class_("Filter")
SOS_Filter = sound_level_meter_ns.class_("SOS_Filter", Filter)

CONF_GROUPS = "groups"
CONF_EQ = "eq"
CONF_MAX = "max"
CONF_MIN = "min"
CONF_PEAK = "peak"
CONF_SOS = "sos"
CONF_COEFFS = "coeffs"
CONF_WARMUP_INTERVAL = "warmup_interval"
CONF_TASK_STACK_SIZE = "task_stack_size"
CONF_TASK_PRIORITY = "task_priority"
CONF_TASK_CORE = "task_core"
CONF_MIC_SENSITIVITY = "mic_sensitivity"
CONF_MIC_SENSITIVITY_REF = "mic_sensitivity_ref"
CONF_OFFSET = "offset"
CONF_PDM = "pdm"

ICON_WAVEFORM = "mdi:waveform"

CONFIG_SENSOR_SCHEMA = cv.typed_schema(
    {
        CONF_EQ: sensor.sensor_schema(
            SoundLevelMeterSensorEq,
            unit_of_measurement=UNIT_DECIBEL,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_WAVEFORM
        ).extend({
            cv.Optional(CONF_UPDATE_INTERVAL): cv.positive_time_period_milliseconds
        }),
        CONF_MAX: sensor.sensor_schema(
            SoundLevelMeterSensorMax,
            unit_of_measurement=UNIT_DECIBEL,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_WAVEFORM
        ).extend({
            cv.Optional(CONF_UPDATE_INTERVAL): cv.positive_time_period_milliseconds,
            cv.Required(CONF_WINDOW_SIZE): cv.positive_time_period_milliseconds
        }),
        CONF_MIN: sensor.sensor_schema(
            SoundLevelMeterSensorMin,
            unit_of_measurement=UNIT_DECIBEL,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_WAVEFORM
        ).extend({
            cv.Optional(CONF_UPDATE_INTERVAL): cv.positive_time_period_milliseconds,
            cv.Required(CONF_WINDOW_SIZE): cv.positive_time_period_milliseconds
        }),
        CONF_PEAK: sensor.sensor_schema(
            SoundLevelMeterSensorPeak,
            unit_of_measurement=UNIT_DECIBEL,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_WAVEFORM
        ).extend({
            cv.Optional(CONF_UPDATE_INTERVAL): cv.positive_time_period_milliseconds
        })
    }
)

CONFIG_FILTER_SCHEMA = cv.typed_schema(
    {
        CONF_SOS: cv.Schema({
            cv.GenerateID(): cv.declare_id(SOS_Filter),
            cv.Required(CONF_COEFFS): [[cv.float_]]
        })
    }
)

def config_group_schema(value):
    return CONFIG_GROUP_SCHEMA(value)

CONFIG_GROUP_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(SensorGroup),
        cv.Optional(CONF_FILTERS): [CONFIG_FILTER_SCHEMA],
        cv.Optional(CONF_SENSORS): [CONFIG_SENSOR_SCHEMA],
        cv.Optional(CONF_GROUPS): [config_group_schema]
    })
)


BASE_SCHEMA = i2s_audio_component_schema(
        SoundLevelMeter,
        default_sample_rate=48000,
        default_channel=CONF_RIGHT,
        default_bits_per_sample="24bit",
    ).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA =  BASE_SCHEMA.extend(
    {
        cv.Required(CONF_I2S_DIN_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_PDM, default=False): cv.boolean,
        cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_WARMUP_INTERVAL, default="500ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_TASK_STACK_SIZE, default=4096): cv.positive_not_null_int,
        cv.Optional(CONF_TASK_PRIORITY, default=2): cv.uint8_t,
        cv.Optional(CONF_TASK_CORE, default=1): cv.int_range(0, 1),
        cv.Optional(CONF_MIC_SENSITIVITY): cv.decibel,
        cv.Optional(CONF_MIC_SENSITIVITY_REF): cv.decibel,
        cv.Optional(CONF_OFFSET): cv.decibel,
        cv.Required(CONF_GROUPS): [CONFIG_GROUP_SCHEMA]
    }
)

async def groups_to_code(config, component, parent):
    for gc in config:
        g = cg.new_Pvariable(gc[CONF_ID])
        cg.add(g.set_parent(component))
        cg.add(parent.add_group(g))
        if CONF_FILTERS in gc:
            for fc in gc[CONF_FILTERS]:
                f = None
                if fc[CONF_TYPE] == CONF_SOS:
                    f = cg.new_Pvariable(fc[CONF_ID], fc[CONF_COEFFS])
                if f is not None:
                    cg.add(g.add_filter(f))
        if CONF_GROUPS in gc:
            await groups_to_code(gc[CONF_GROUPS], component, g)
        if CONF_SENSORS in gc:
            for sc in gc[CONF_SENSORS]:
                s = await sensor.new_sensor(sc)
                cg.add(s.set_parent(component))
                if CONF_WINDOW_SIZE in sc:
                    cg.add(s.set_window_size(sc[CONF_WINDOW_SIZE]))
                if CONF_UPDATE_INTERVAL in sc:
                    cg.add(s.set_update_interval(sc[CONF_UPDATE_INTERVAL]))
                cg.add(g.add_sensor(s))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await register_i2s_audio_component(var, config)
    cg.add(var.set_din_pin(config[CONF_I2S_DIN_PIN]))
    cg.add(var.set_pdm(config[CONF_PDM]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_warmup_interval(config[CONF_WARMUP_INTERVAL]))
    cg.add(var.set_task_stack_size(config[CONF_TASK_STACK_SIZE]))
    cg.add(var.set_task_priority(config[CONF_TASK_PRIORITY]))
    cg.add(var.set_task_core(config[CONF_TASK_CORE]))
    cg.add(var.set_max_groups_depth(get_groups_depth(config[CONF_GROUPS]) + 1))
    if CONF_MIC_SENSITIVITY in config:
        cg.add(var.set_mic_sensitivity(config[CONF_MIC_SENSITIVITY]))
    if CONF_MIC_SENSITIVITY_REF in config:
        cg.add(var.set_mic_sensitivity_ref(config[CONF_MIC_SENSITIVITY_REF]))
    if CONF_OFFSET in config:
        cg.add(var.set_offset(config[CONF_OFFSET]))
    await groups_to_code(config[CONF_GROUPS], var, var)

def get_groups_depth(cur):
    max_d = 0
    for g in cur:
        d = int(CONF_FILTERS in g)
        if CONF_GROUPS in g:
            d += get_groups_depth(g[CONF_GROUPS])
        max_d = max(d, max_d)
    return max_d
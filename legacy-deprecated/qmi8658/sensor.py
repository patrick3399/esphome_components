import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ACCELERATION_X,
    CONF_ACCELERATION_Y,
    CONF_ACCELERATION_Z,
    CONF_GYROSCOPE_X,
    CONF_GYROSCOPE_Y,
    CONF_GYROSCOPE_Z,
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_ACCELERATION_X,
    ICON_ACCELERATION_Y,
    ICON_ACCELERATION_Z,
    ICON_GYROSCOPE_X,
    ICON_GYROSCOPE_Y,
    ICON_GYROSCOPE_Z,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREE_PER_SECOND,
    UNIT_METER_PER_SECOND_SQUARED,
)

CODEOWNERS = ["@patrick3399"]
DEPENDENCIES = ["i2c"]

CONF_ACCEL_RANGE = "accelerometer_range"
CONF_GYRO_RANGE = "gyroscope_range"
CONF_ACCEL_ODR = "accelerometer_odr"
CONF_GYRO_ODR = "gyroscope_odr"
CONF_ACCEL_LPF = "accelerometer_lpf_mode"
CONF_GYRO_LPF = "gyroscope_lpf_mode"

qmi8658_ns = cg.esphome_ns.namespace("qmi8658")
QMI8658Component = qmi8658_ns.class_(
    "QMI8658Component", cg.PollingComponent, i2c.I2CDevice
)

AccelRange = qmi8658_ns.enum("AccelRange")
ACCEL_RANGES = {
    "2G": AccelRange.ACCEL_RANGE_2G,
    "4G": AccelRange.ACCEL_RANGE_4G,
    "8G": AccelRange.ACCEL_RANGE_8G,
    "16G": AccelRange.ACCEL_RANGE_16G,
}

GyroRange = qmi8658_ns.enum("GyroRange")
GYRO_RANGES = {
    "16DPS": GyroRange.GYRO_RANGE_16DPS,
    "32DPS": GyroRange.GYRO_RANGE_32DPS,
    "64DPS": GyroRange.GYRO_RANGE_64DPS,
    "128DPS": GyroRange.GYRO_RANGE_128DPS,
    "256DPS": GyroRange.GYRO_RANGE_256DPS,
    "512DPS": GyroRange.GYRO_RANGE_512DPS,
    "1024DPS": GyroRange.GYRO_RANGE_1024DPS,
    "2048DPS": GyroRange.GYRO_RANGE_2048DPS,
}

AccelODR = qmi8658_ns.enum("AccelODR")
ACCEL_ODRS = {
    "1000HZ": AccelODR.ACCEL_ODR_1000HZ,
    "500HZ": AccelODR.ACCEL_ODR_500HZ,
    "250HZ": AccelODR.ACCEL_ODR_250HZ,
    "125HZ": AccelODR.ACCEL_ODR_125HZ,
    "62HZ": AccelODR.ACCEL_ODR_62HZ,
    "31HZ": AccelODR.ACCEL_ODR_31HZ,
    "128HZ_LP": AccelODR.ACCEL_ODR_128HZ_LP,
    "21HZ_LP": AccelODR.ACCEL_ODR_21HZ_LP,
    "11HZ_LP": AccelODR.ACCEL_ODR_11HZ_LP,
    "3HZ_LP": AccelODR.ACCEL_ODR_3HZ_LP,
}

GyroODR = qmi8658_ns.enum("GyroODR")
GYRO_ODRS = {
    "7174HZ": GyroODR.GYRO_ODR_7174HZ,
    "3587HZ": GyroODR.GYRO_ODR_3587HZ,
    "1793HZ": GyroODR.GYRO_ODR_1793HZ,
    "896HZ": GyroODR.GYRO_ODR_896HZ,
    "448HZ": GyroODR.GYRO_ODR_448HZ,
    "224HZ": GyroODR.GYRO_ODR_224HZ,
    "112HZ": GyroODR.GYRO_ODR_112HZ,
    "56HZ": GyroODR.GYRO_ODR_56HZ,
    "28HZ": GyroODR.GYRO_ODR_28HZ,
}

LpfMode = qmi8658_ns.enum("LpfMode")
LPF_MODES = {
    "DISABLED": LpfMode.LPF_DISABLED,
    "0": LpfMode.LPF_MODE_0,
    "1": LpfMode.LPF_MODE_1,
    "2": LpfMode.LPF_MODE_2,
    "3": LpfMode.LPF_MODE_3,
}

accel_schema = {
    "unit_of_measurement": UNIT_METER_PER_SECOND_SQUARED,
    "accuracy_decimals": 2,
    "state_class": STATE_CLASS_MEASUREMENT,
}
gyro_schema = {
    "unit_of_measurement": UNIT_DEGREE_PER_SECOND,
    "accuracy_decimals": 2,
    "state_class": STATE_CLASS_MEASUREMENT,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(QMI8658Component),
            cv.Optional(CONF_ACCELERATION_X): sensor.sensor_schema(
                icon=ICON_ACCELERATION_X,
                **accel_schema,
            ),
            cv.Optional(CONF_ACCELERATION_Y): sensor.sensor_schema(
                icon=ICON_ACCELERATION_Y,
                **accel_schema,
            ),
            cv.Optional(CONF_ACCELERATION_Z): sensor.sensor_schema(
                icon=ICON_ACCELERATION_Z,
                **accel_schema,
            ),
            cv.Optional(CONF_GYROSCOPE_X): sensor.sensor_schema(
                icon=ICON_GYROSCOPE_X,
                **gyro_schema,
            ),
            cv.Optional(CONF_GYROSCOPE_Y): sensor.sensor_schema(
                icon=ICON_GYROSCOPE_Y,
                **gyro_schema,
            ),
            cv.Optional(CONF_GYROSCOPE_Z): sensor.sensor_schema(
                icon=ICON_GYROSCOPE_Z,
                **gyro_schema,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ACCEL_RANGE, default="4G"): cv.enum(
                ACCEL_RANGES, upper=True
            ),
            cv.Optional(CONF_GYRO_RANGE, default="256DPS"): cv.enum(
                GYRO_RANGES, upper=True
            ),
            cv.Optional(CONF_ACCEL_ODR, default="250HZ"): cv.enum(
                ACCEL_ODRS, upper=True
            ),
            cv.Optional(CONF_GYRO_ODR, default="224HZ"): cv.enum(
                GYRO_ODRS, upper=True
            ),
            cv.Optional(CONF_ACCEL_LPF, default="DISABLED"): cv.enum(
                LPF_MODES, upper=True
            ),
            cv.Optional(CONF_GYRO_LPF, default="DISABLED"): cv.enum(
                LPF_MODES, upper=True
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x6B))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_accel_range(config[CONF_ACCEL_RANGE]))
    cg.add(var.set_gyro_range(config[CONF_GYRO_RANGE]))
    cg.add(var.set_accel_odr(config[CONF_ACCEL_ODR]))
    cg.add(var.set_gyro_odr(config[CONF_GYRO_ODR]))
    cg.add(var.set_accel_lpf_mode(config[CONF_ACCEL_LPF]))
    cg.add(var.set_gyro_lpf_mode(config[CONF_GYRO_LPF]))

    for d in ["x", "y", "z"]:
        accel_key = f"acceleration_{d}"
        if accel_key in config:
            sens = await sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))

        gyro_key = f"gyroscope_{d}"
        if gyro_key in config:
            sens = await sensor.new_sensor(config[gyro_key])
            cg.add(getattr(var, f"set_gyro_{d}_sensor")(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

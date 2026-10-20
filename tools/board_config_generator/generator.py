"""Validate board JSON and emit deterministic C headers (Python standard library only)."""
import argparse
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BOARD = "MP_V3"
SLEW_RATES = (53.2, 34.0, 18.3, 13.0, 10.8, 7.9, 5.3, 2.6)
DRV8873_CURRENT_THRESHOLDS_MA = (4000, 5400, 6500, 7000)
DRV8873_OVERCURRENT_MODES = ("latched_shutdown", "automatic_retry", "report_only", "disabled")
# These pins are always configured by lib/usb/usbd_conf.c, outside the board JSON.
FIRMWARE_RESERVED_PINS = {"PA11": "USB_OTG_FS_DM", "PA12": "USB_OTG_FS_DP"}
# Timer routes used by supported boards. Extend deliberately when adding hardware.
PWM_PINS = {
    ("TIM1", 1, False): {"PA8"}, ("TIM1", 2, False): {"PA9"},
    ("TIM1", 4, False): {"PE14"}, ("TIM1", 1, True): {"PE8"},
    ("TIM1", 2, True): {"PE10"}, ("TIM1", 3, True): {"PE12"},
    ("TIM2", 1, False): {"PA5", "PA15"}, ("TIM2", 2, False): {"PB3"},
    ("TIM2", 3, False): {"PB10"}, ("TIM2", 4, False): {"PB11"},
    ("TIM3", 1, False): {"PA6"}, ("TIM3", 2, False): {"PA7"},
    ("TIM4", 1, False): {"PB6"}, ("TIM4", 2, False): {"PB7"},
    **{("TIM5", c, False): {f"PA{c - 1}"} for c in range(1, 5)},
    **{("TIM8", c, False): {f"PC{c + 5}"} for c in range(1, 5)},
    ("TIM12", 1, False): {"PB14"}, ("TIM12", 2, False): {"PB15"},
}


class ConfigError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise ConfigError(message)


def fields(value, required, optional=(), at="config"):
    require(isinstance(value, dict), f"{at}: expected an object")
    missing = set(required) - value.keys()
    unknown = value.keys() - set(required) - set(optional)
    require(not missing, f"{at}: missing fields {sorted(missing)}")
    require(not unknown, f"{at}: unknown fields {sorted(unknown)}")


def integer(value, low, high, at):
    require(type(value) is int and low <= value <= high,
            f"{at}: expected integer {low}..{high}")


def boolean(value, at):
    require(type(value) is bool, f"{at}: expected true or false")


def entries(value, maximum, at, minimum=1):
    require(isinstance(value, list) and minimum <= len(value) <= maximum,
            f"{at}: expected {minimum}..{maximum} entries")
    for index, item in enumerate(value):
        require(isinstance(item, dict) and type(item.get("id")) is int and item["id"] == index,
                f"{at}: IDs must be unique, contiguous and ordered from 0")
    return value


def validate(board):
    fields(board, ("schema_version", "board", "mcu", "buses", "motors", "encoders", "gpio", "status_leds"),
           ("shared_pins", "temperature_sensors", "imu_sensors"))
    integer(board["schema_version"], 1, 1, "schema_version")
    identity = board["board"]
    fields(identity, ("model", "revision", "build_define"), at="board")
    integer(identity["model"], 1, 255, "board.model")
    require(isinstance(identity["revision"], str) and re.fullmatch(r"(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)", identity["revision"]),
            "board.revision: expected major.minor.patch")
    require(all(int(v) <= 255 for v in identity["revision"].split(".")), "board.revision: components must fit uint8")
    require(isinstance(identity["build_define"], str) and re.fullmatch(r"MP_V[1-9][0-9]*", identity["build_define"]),
            "board.build_define: expected MP_V<number>")
    fields(board["mcu"], ("part",), at="mcu")
    require(board["mcu"]["part"] in ("STM32F405RGT6", "STM32F405VGT6"), "mcu.part: unsupported MCU")
    pins, timer_channels, encoder_timers = {}, set(), set()

    def pin(value, user):
        require(isinstance(value, str) and re.fullmatch(r"P[A-E](?:[0-9]|1[0-5])", value),
                f"{user}: invalid pin {value!r}")
        if board["mcu"]["part"] == "STM32F405RGT6":
            require(value[1] in "ABC" or value == "PD2", f"{user}: pin unavailable on STM32F405RGT6")
        if value in FIRMWARE_RESERVED_PINS:
            raise ConfigError(f"{user}: pin {value} is reserved by firmware for {FIRMWARE_RESERVED_PINS[value]}")
        pins.setdefault(value, []).append(user)

    def route(timer, channel, complementary, value, user):
        require(isinstance(timer, str), f"{user}: expected a timer name")
        integer(channel, 1, 4, user + ".channel")
        boolean(complementary, user + ".complementary")
        pin(value, user)
        require(value in PWM_PINS.get((timer, channel, complementary), set()),
                f"{user}: unsupported timer/pin route {timer} CH{channel}{'N' if complementary else ''} on {value}")

    def pwm(value, user):
        fields(value, ("pin", "timer", "channel", "complementary"), at=user)
        route(value["timer"], value["channel"], value["complementary"], value["pin"], user)
        key = (value["timer"], value["channel"])
        require(key not in timer_channels, f"{user}: timer channel already used")
        timer_channels.add(key)

    require(isinstance(board["buses"], dict), "buses: expected an object")
    peripherals, used_buses = set(), set()
    for name, bus in board["buses"].items():
        require(re.fullmatch(r"[a-z][a-z0-9_]*", name), "buses: invalid bus name")
        require(isinstance(bus, dict), f"buses.{name}: expected an object")
        if bus.get("peripheral") == "I2C3":
            fields(bus, ("peripheral", "scl", "sda", "frequency_hz"), at=f"buses.{name}")
            require(bus["scl"] == "PA8" and bus["sda"] == "PC9",
                    f"buses.{name}: supported I2C route is I2C3 on PA8/PC9")
            integer(bus["frequency_hz"], 100000, 100000, f"buses.{name}.frequency_hz")
            keys = ("scl", "sda")
        else:
            fields(bus, ("peripheral", "sck", "miso", "mosi"), at=f"buses.{name}")
            require(bus in ({"peripheral": "SPI3", "sck": "PC10", "miso": "PC11", "mosi": "PC12"},
                            {"peripheral": "SPI2", "sck": "PB13", "miso": "PC2", "mosi": "PC3"}),
                    f"buses.{name}: supported SPI routes are SPI3 on PC10/PC11/PC12 or SPI2 on PB13/PC2/PC3")
            keys = ("sck", "miso", "mosi")
        require(bus["peripheral"] not in peripherals, f"buses.{name}: peripheral already used")
        peripherals.add(bus["peripheral"])
        for key in keys:
            pin(bus[key], f"buses.{name}.{key}")

    addresses = set()
    for sensor in entries(board.get("temperature_sensors", []), 4, "temperature_sensors", minimum=0):
        at = f"temperature_sensors.{sensor['id']}"
        fields(sensor, ("id", "type", "bus", "address"), at=at)
        require(sensor["type"] == "TMP102", at + ".type: unsupported sensor")
        require(isinstance(sensor["bus"], str) and sensor["bus"] in board["buses"], at + ".bus: unknown bus")
        require(board["buses"][sensor["bus"]]["peripheral"] == "I2C3", at + ".bus: TMP102 requires I2C3")
        integer(sensor["address"], 0x48, 0x4B, at + ".address")
        require(sensor["address"] not in addresses, at + ".address: duplicate I2C address")
        addresses.add(sensor["address"])
        used_buses.add(sensor["bus"])

    for sensor in entries(board.get("imu_sensors", []), 4, "imu_sensors", minimum=0):
        at = f"imu_sensors.{sensor['id']}"
        fields(sensor, ("id", "type", "bus", "accel_chip_select", "gyro_chip_select",
                        "accel_interrupt", "gyro_interrupt"), at=at)
        require(sensor["type"] == "BMI088", at + ".type: unsupported sensor")
        require(isinstance(sensor["bus"], str) and sensor["bus"] in board["buses"], at + ".bus: unknown bus")
        require(board["buses"][sensor["bus"]]["peripheral"] == "SPI2", at + ".bus: BMI088 requires SPI2")
        for key in ("accel_chip_select", "gyro_chip_select", "accel_interrupt", "gyro_interrupt"):
            pin(sensor[key], at + "." + key)
        used_buses.add(sensor["bus"])

    motors = board["motors"]
    fields(motors, ("control_model", "pwm_frequency_hz", "channels"), ("driver",), at="motors")
    require(motors["control_model"] in ("pwm", "enable_direction"), "motors.control_model: unsupported model")
    integer(motors["pwm_frequency_hz"], 100000, 100000, "motors.pwm_frequency_hz")
    enable_direction = motors["control_model"] == "enable_direction"
    if enable_direction:
        require("driver" in motors, "motors.driver: required for enable_direction")
        driver = motors["driver"]
        fields(driver, ("type", "bus", "slew_rate_v_per_us"), at="motors.driver")
        require(driver["type"] == "DRV8873S", "motors.driver.type: unsupported driver")
        require(isinstance(driver["bus"], str) and driver["bus"] in board["buses"], "motors.driver.bus: unknown bus")
        require(board["buses"][driver["bus"]]["peripheral"] == "SPI3", "motors.driver.bus: DRV8873S requires SPI3")
        require(type(driver["slew_rate_v_per_us"]) in (int, float) and driver["slew_rate_v_per_us"] in SLEW_RATES,
                "motors.driver.slew_rate_v_per_us: unsupported DRV8873 slew rate")
        used_buses.add(driver["bus"])
    else:
        require("driver" not in motors, "pwm: SPI driver/buses are not supported")
    require(set(board["buses"]) == used_buses, "buses: unused bus definition")
    for motor in entries(motors["channels"], 4, "motors.channels"):
        user = f"motors.{motor['id']}"
        if enable_direction:
            fields(motor, ("id", "enable", "direction", "disable", "sleep", "fault", "chip_select", "current_sense", "hardware_current_limit", "overcurrent_protection"), at=user)
            pwm(motor["enable"], user + ".enable")
            for key in ("direction", "disable", "sleep", "fault", "chip_select"):
                pin(motor[key], user + "." + key)
            sense = motor["current_sense"]
            require(isinstance(sense, dict), user + ".current_sense: expected an object")
            require("type" in sense, user + ".current_sense: missing field 'type'")
            require(sense["type"] == "adc",
                    user + ".current_sense.type: unsupported type; only 'adc' configuration is supported")
            fields(sense, ("type", "pin", "sense_resistor_ohm", "filter_capacitance_nf"), at=user + ".current_sense")
            pin(sense["pin"], user + ".current_sense")
            require(sense["pin"] in {f"PA{n}" for n in range(8)} | {"PB0", "PB1"} | {f"PC{n}" for n in range(6)},
                    user + ".current_sense: expected an ADC-capable pin")
            integer(sense["sense_resistor_ohm"], 1, 65535, user + ".sense_resistor_ohm")
            integer(sense["filter_capacitance_nf"], 1, 65535, user + ".filter_capacitance_nf")
            limit = motor["hardware_current_limit"]
            fields(limit, ("enabled", "threshold_ma", "supported_thresholds_ma"), at=user + ".hardware_current_limit")
            boolean(limit["enabled"], user + ".hardware_current_limit.enabled")
            integer(limit["threshold_ma"], 1, 65535, user + ".hardware_current_limit.threshold_ma")
            thresholds = limit["supported_thresholds_ma"]
            at = user + ".hardware_current_limit.supported_thresholds_ma"
            require(isinstance(thresholds, list) and 1 <= len(thresholds) <= len(DRV8873_CURRENT_THRESHOLDS_MA),
                    at + ": expected 1..4 entries")
            for value in thresholds:
                integer(value, 1, 65535, at)
                require(value in DRV8873_CURRENT_THRESHOLDS_MA, at + ": unsupported DRV8873S threshold")
            require(thresholds == sorted(set(thresholds)), at + ": expected unique values in ascending order")
            require(limit["threshold_ma"] in thresholds,
                    user + ".hardware_current_limit.threshold_ma: must be listed in supported_thresholds_ma")
            # Metadata only until IC4 programming is implemented. Never silently
            # accept a requested threshold that the firmware will not apply.
            require(limit["enabled"] and limit["threshold_ma"] == 6500,
                    user + ".hardware_current_limit: only the enabled 6500 mA factory default is implemented")
            protection = motor["overcurrent_protection"]
            at = user + ".overcurrent_protection"
            fields(protection, ("mode", "supported_modes"), at=at)
            require(isinstance(protection["mode"], str) and protection["mode"] in DRV8873_OVERCURRENT_MODES,
                    at + ".mode: unsupported DRV8873S mode")
            modes = protection["supported_modes"]
            require(isinstance(modes, list) and 1 <= len(modes) <= len(DRV8873_OVERCURRENT_MODES),
                    at + ".supported_modes: expected 1..4 entries")
            require(all(isinstance(mode, str) and mode in DRV8873_OVERCURRENT_MODES for mode in modes),
                    at + ".supported_modes: unsupported DRV8873S mode")
            require(len(set(modes)) == len(modes), at + ".supported_modes: expected unique values")
            require(protection["mode"] in modes, at + ".mode: must be listed in supported_modes")
            # Metadata only until IC2 programming is implemented.
            require(protection["mode"] == "latched_shutdown",
                    at + ": only the latched_shutdown factory default is implemented")
        else:
            fields(motor, ("id", "pwm1", "pwm2"), at=user)
            pwm(motor["pwm1"], user + ".pwm1")
            pwm(motor["pwm2"], user + ".pwm2")
            require(motor["pwm1"]["timer"] == motor["pwm2"]["timer"], user + ": both PWM outputs must use the same timer")
    for encoder in entries(board["encoders"], 4, "encoders"):
        user = f"encoders.{encoder['id']}"
        fields(encoder, ("id", "timer", "a", "b"), at=user)
        for channel, key in enumerate(("a", "b"), 1):
            route(encoder["timer"], channel, False, encoder[key], user + "." + key)
        require(encoder["timer"] in ("TIM1", "TIM2", "TIM3", "TIM4", "TIM5", "TIM8"), user + ": unsupported encoder timer")
        require(encoder["timer"] not in encoder_timers and all(t != encoder["timer"] for t, _ in timer_channels), user + ": timer already used")
        encoder_timers.add(encoder["timer"])
    for item in entries(board["gpio"], 255, "gpio"):
        fields(item, ("id", "pin"), at="gpio")
        pin(item["pin"], f"gpio.{item['id']}")
    for item in entries(board["status_leds"], 255, "status_leds"):
        fields(item, ("id", "pin", "active_low"), at="status_leds")
        pin(item["pin"], f"status_leds.{item['id']}")
        boolean(item["active_low"], f"status_leds.{item['id']}.active_low")
    shared = board.get("shared_pins", [])
    require(isinstance(shared, list), "shared_pins: expected an array")
    declared = set()
    for item in shared:
        fields(item, ("pin", "users", "reason"), at="shared_pins")
        require(isinstance(item["pin"], str) and item["pin"] not in declared, "shared_pins: invalid or repeated pin")
        require(isinstance(item["users"], list) and all(isinstance(u, str) for u in item["users"]), "shared_pins.users: expected strings")
        actual = pins.get(item["pin"], [])
        require(len(actual) > 1 and sorted(item["users"]) == sorted(actual), f"shared_pins.{item['pin']}: must list exactly the conflicting users")
        require(isinstance(item["reason"], str) and bool(item["reason"].strip()), "shared_pins.reason: required")
        declared.add(item["pin"])
    for value, users in pins.items():
        require(len(users) == 1 or value in declared, f"pin {value} is shared by {', '.join(users)}; declare shared_pins explicitly")
    return board


def unique_object(pairs):
    value = {}
    for key, item in pairs:
        require(key not in value, f"duplicate JSON key: {key}")
        value[key] = item
    return value


def load_boards(directory):
    boards = []
    for path in sorted(Path(directory).glob("*.json")):
        try:
            board = validate(json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=unique_object))
        except (ValueError, TypeError) as error:
            raise ConfigError(f"{path.name}: {error}") from error
        boards.append(board)
    require(boards, f"{directory}: no board JSON files found")
    defines = [b["board"]["build_define"] for b in boards]
    identities = [(b["board"]["model"], b["board"]["revision"]) for b in boards]
    require(len(set(defines)) == len(defines), "duplicate board.build_define")
    require(len(set(identities)) == len(identities), "duplicate board model/revision")
    require(DEFAULT_BOARD in defines, f"default board {DEFAULT_BOARD} is missing")
    return sorted(boards, key=lambda b: b["board"]["build_define"])


def select_build_board(boards, defines, mcu):
    known = {board["board"]["build_define"] for board in boards}
    requested = {name for name in defines if name.startswith("MP_V")}
    require(not requested - known, f"Unsupported board selector: {sorted(requested - known)}")
    selected = requested or {DEFAULT_BOARD}
    require(len(selected) == 1, "Select exactly one supported board revision")
    board = next(b for b in boards if b["board"]["build_define"] in selected)
    part = board["mcu"]["part"].lower()
    require(mcu.lower() in (part, part[:-2]),
            f"Board {board['board']['revision']} needs {board['mcu']['part']}; build selects {mcu}")
    return board


def gpio(value):
    return "{GPIO%s, GPIO_PIN_%s}" % (value[1], value[2:])


def pwm(value):
    return "{TIM_CHANNEL_%d, GPIO%s, GPIO_PIN_%s, %s}" % (
        value["channel"], value["pin"][1], value["pin"][2:], str(value["complementary"]).lower())


def client_profile(board):
    """Project hardware definitions into public, language-neutral feature metadata."""
    return {
        "board": {"model": board["board"]["model"], "revision": board["board"]["revision"]},
        "resources": {
            "motors": len(board["motors"]["channels"]), "encoders": len(board["encoders"]),
            "gpio": len(board["gpio"]), "status_leds": len(board["status_leds"]),
        },
        "motor_features": [{
            "id": motor["id"],
            "current_sense": "current_sense" in motor,
            "current_limit": "hardware_current_limit" in motor,
        } for motor in board["motors"]["channels"]],
        "sensors": {
            "temperature": [{"id": s["id"]} for s in board.get("temperature_sensors", [])],
            "imu": [{"id": s["id"], "accelerometer": True, "gyroscope": True}
                    for s in board.get("imu_sensors", [])],
        },
    }


def render(boards):
    banner = "// Generated by tools/board_config_generator/generator.py. DO NOT EDIT.\n// Source: boards/*.json\n#pragma once\n"
    identity = [banner]
    defines = [b["board"]["build_define"] for b in boards]
    identity += ["#if " + " && ".join(f"!defined({d})" for d in defines),
                 f"#define {DEFAULT_BOARD}", "#endif",
                 "#if (" + " + ".join(f"defined({d})" for d in defines) + ") != 1",
                 '#error "Select exactly one supported board revision"', "#endif", ""]
    hardware = [banner, '#include "stm32f4xx_hal.h"', '#include "hw_config_structs.h"', '#include "board_revision.h"', ""]
    for index, board in enumerate(boards):
        selection = f"#{'if' if index == 0 else 'elif'} defined({board['board']['build_define']})"
        identity += [selection, f"#define KINISI_BOARD_MODEL {board['board']['model']}",
                     f'#define KINISI_MCU_PART "{board["mcu"]["part"]}"']
        for key, number in zip(("MAJOR", "MINOR", "PATCH"), board["board"]["revision"].split(".")):
            identity.append(f"#define KINISI_BOARD_VERSION_{key} {number}")
        motors = board["motors"]
        model = motors["control_model"].upper()
        hardware += [selection, f"#define MOTOR_CONTROL_MODEL MOTOR_CONTROL_MODEL_{model}",
                     f"#define MOTOR_PWM_FREQUENCY_HZ {motors['pwm_frequency_hz']}U"]
        for count, items in (("MOTORS", motors["channels"]), ("ENCODERS", board["encoders"]),
                             ("GPIO_PINS", board["gpio"]), ("STATUS_LEDS", board["status_leds"]),
                             ("TEMPERATURE_SENSORS", board.get("temperature_sensors", [])),
                             ("IMU_SENSORS", board.get("imu_sensors", []))):
            hardware.append(f"#define NUMBER_{count} {len(items)}")
        sensors = board.get("temperature_sensors", [])
        if sensors:
            bus = board["buses"][sensors[0]["bus"]]
            hardware += [f"#define TEMPERATURE_I2C_CLK_ENABLE() __HAL_RCC_{bus['peripheral']}_CLK_ENABLE()",
                         "static const i2c_bus_info_t temperature_i2c_info = {",
                         f"    .instance = {bus['peripheral']}, .alternate = GPIO_AF4_I2C3, .frequencyHz = {bus['frequency_hz']}U,",
                         f"    .scl = {gpio(bus['scl'])}, .sda = {gpio(bus['sda'])}", "};",
                         "static const temperature_sensor_info_t temperature_sensor_info[NUMBER_TEMPERATURE_SENSORS] = {"]
            hardware += [f"    [{s['id']}] = {{.type = TEMPERATURE_SENSOR_TMP102, .address = 0x{s['address']:02X}U}}," for s in sensors]
            hardware.append("};")
        imus = board.get("imu_sensors", [])
        if imus:
            bus = board["buses"][imus[0]["bus"]]
            hardware += [f"#define IMU_SPI_CLK_ENABLE() __HAL_RCC_{bus['peripheral']}_CLK_ENABLE()",
                         "static const spi_bus_info_t imu_spi_info = {",
                         f"    .instance = {bus['peripheral']}, .alternate = GPIO_AF5_SPI2,",
                         f"    .sck = {gpio(bus['sck'])}, .miso = {gpio(bus['miso'])}, .mosi = {gpio(bus['mosi'])}", "};",
                         "static const imu_sensor_info_t imu_sensor_info[NUMBER_IMU_SENSORS] = {"]
            for sensor in imus:
                hardware += [f"    [{sensor['id']}] = {{.type = IMU_SENSOR_BMI088,",
                             f"        .accelChipSelect = {gpio(sensor['accel_chip_select'])}, .gyroChipSelect = {gpio(sensor['gyro_chip_select'])},",
                             f"        .accelInterrupt = {gpio(sensor['accel_interrupt'])}, .gyroInterrupt = {gpio(sensor['gyro_interrupt'])}}},"]
            hardware.append("};")
        if model == "ENABLE_DIRECTION":
            driver = motors["driver"]
            bus = board["buses"][driver["bus"]]
            hardware += [f"#define DRV8873_SLEW_RATE_BITS {SLEW_RATES.index(driver['slew_rate_v_per_us'])}U",
                         f"#define MOTOR_SPI_CLK_ENABLE() __HAL_RCC_{bus['peripheral']}_CLK_ENABLE()",
                         "static const spi_bus_info_t motor_spi_info = {",
                         f"    .instance = {bus['peripheral']}, .alternate = GPIO_AF6_SPI3,",
                         f"    .sck = {gpio(bus['sck'])}, .miso = {gpio(bus['miso'])}, .mosi = {gpio(bus['mosi'])}", "};"]
            for item in motors["channels"]:
                thresholds = ", ".join(str(v) for v in item["hardware_current_limit"]["supported_thresholds_ma"])
                hardware.append(f"static const uint16_t motor_{item['id']}_current_limit_thresholds_ma[] = {{{thresholds}}};")
                modes = ", ".join(f"OVERCURRENT_PROTECTION_{mode.upper()}" for mode in item["overcurrent_protection"]["supported_modes"])
                hardware.append(f"static const overcurrent_protection_mode_t motor_{item['id']}_overcurrent_modes[] = {{{modes}}};")
        hardware += ["static const motor_info_t motor_info[NUMBER_MOTORS] = {"]
        for item in motors["channels"]:
            channel = item["enable"] if model == "ENABLE_DIRECTION" else item["pwm1"]
            hardware += [f"    [{item['id']}] = {{", f"        .controlModel = MOTOR_CONTROL_{model}, .timer = {channel['timer']},"]
            if model == "ENABLE_DIRECTION":
                hardware.append(f"        .enable = {pwm(item['enable'])},")
                for source, target in (("direction", "direction"), ("disable", "disable"), ("sleep", "sleep"), ("fault", "fault"), ("chip_select", "chipSelect")):
                    hardware.append(f"        .{target} = {gpio(item[source])},")
                sense, limit = item["current_sense"], item["hardware_current_limit"]
                protection = item["overcurrent_protection"]
                hardware += ["        // Circuit/default metadata; ADC sampling and IC2/IC4 programming are not implemented.",
                             f"        .currentSense = {{.available = true, .type = CURRENT_SENSE_{sense['type'].upper()}, .input = {gpio(sense['pin'])},",
                             f"            .senseResistorOhm = {sense['sense_resistor_ohm']}, .filterCapacitanceNf = {sense['filter_capacitance_nf']}}},",
                             f"        .hardwareCurrentLimit = {{.available = true, .enabled = {str(limit['enabled']).lower()}, .thresholdMa = {limit['threshold_ma']},",
                             f"            .supportedThresholdsMa = motor_{item['id']}_current_limit_thresholds_ma, .supportedThresholdCount = {len(limit['supported_thresholds_ma'])}}},",
                             f"        .overcurrentProtection = {{.available = true, .mode = OVERCURRENT_PROTECTION_{protection['mode'].upper()},",
                             f"            .supportedModes = motor_{item['id']}_overcurrent_modes, .supportedModeCount = {len(protection['supported_modes'])}}},"]
            else:
                hardware += [f"        .pwmChannel1 = {pwm(item['pwm1'])},", f"        .pwmChannel2 = {pwm(item['pwm2'])},"]
            hardware.append("    },")
        hardware += ["};", "static const encoder_info_t encoder_info[NUMBER_ENCODERS] = {"]
        for item in board["encoders"]:
            hardware.append(f"    [{item['id']}] = {{{item['timer']}, GPIO{item['a'][1]}, GPIO_PIN_{item['a'][2:]}, GPIO{item['b'][1]}, GPIO_PIN_{item['b'][2:]}}},")
        hardware += ["};", "static const gpio_info_t gpio_info[NUMBER_GPIO_PINS] = {"]
        hardware += [f"    [{item['id']}] = {gpio(item['pin'])}," for item in board["gpio"]]
        hardware += ["};", "static const status_led_info_t status_led_info[NUMBER_STATUS_LEDS] = {"]
        hardware += [f"    [{item['id']}] = {{.gpio = {gpio(item['pin'])}, .activeLow = {str(item['active_low']).lower()}}}," for item in board["status_leds"]]
        hardware += ["};", ""]
    identity.append("#endif\n")
    hardware.append("#endif\n")
    profiles = {"schema_version": 1, "boards": [client_profile(b) for b in boards]}
    return {"board_revision.h": "\n".join(identity), "hw_config.h": "\n".join(hardware),
            "board_profiles.json": json.dumps(profiles, indent=2) + "\n"}


def generate(boards_dir=ROOT / "boards", output_dir=ROOT / "include", check=False):
    outputs = render(load_boards(boards_dir))  # Validate everything before touching output.
    stale = []
    for name, content in outputs.items():
        path = Path(output_dir) / name
        if not path.exists() or path.read_text(encoding="utf-8") != content:
            stale.append(name)
            if not check:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8", newline="\n")
    if check and stale:
        raise ConfigError("stale generated board files: " + ", ".join(stale) + "; run tools/board_config_generator/generator.py")
    return stale


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boards-dir", type=Path, default=ROOT / "boards")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "include")
    parser.add_argument("--check", action="store_true", help="fail if generated headers or client profiles differ; do not write files")
    args = parser.parse_args()
    try:
        changed = generate(args.boards_dir, args.output_dir, args.check)
    except (ConfigError, OSError) as error:
        parser.exit(1, f"Board configuration error: {error}\n")
    print("Board configuration: " + ("updated " + ", ".join(changed) if changed else "up to date"))


if __name__ == "__main__":
    main()

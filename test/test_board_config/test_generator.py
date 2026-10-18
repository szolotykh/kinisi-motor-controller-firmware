import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("generator", ROOT / "tools/board_config_generator/generator.py")
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)


class BoardConfigTests(unittest.TestCase):
    def setUp(self):
        self.boards = g.load_boards(ROOT / "boards")
        self.v4 = copy.deepcopy(self.boards[3])

    def reject(self, board, message):
        with self.assertRaisesRegex(g.ConfigError, message):
            g.validate(board)

    def test_generation_is_current_deterministic_and_does_not_rewrite(self):
        g.generate(check=True)
        with tempfile.TemporaryDirectory() as tmp:
            g.generate(output_dir=tmp)
            before = {p.name: (p.read_bytes(), p.stat().st_mtime_ns) for p in Path(tmp).glob("*.h")}
            self.assertEqual(g.generate(output_dir=tmp), [])
            self.assertEqual(before, {p.name: (p.read_bytes(), p.stat().st_mtime_ns) for p in Path(tmp).glob("*.h")})
            header = Path(tmp) / "hw_config.h"
            header.write_text("stale", encoding="utf-8")
            with self.assertRaisesRegex(g.ConfigError, "stale generated headers"):
                g.generate(output_dir=tmp, check=True)
            self.assertEqual(header.read_text(), "stale")

    def test_invalid_input_never_overwrites_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            boards, output = Path(tmp) / "boards", Path(tmp) / "output"
            boards.mkdir(); output.mkdir()
            (boards / "bad.json").write_text('{"schema_version":1,"schema_version":2}')
            (output / "hw_config.h").write_text("keep me")
            with self.assertRaisesRegex(g.ConfigError, "duplicate JSON key"):
                g.generate(boards, output)
            self.assertEqual((output / "hw_config.h").read_text(), "keep me")

    def test_unknown_fields_and_schema_fail(self):
        self.v4["motors"]["channels"][0]["enable"]["complimentary"] = True
        self.reject(self.v4, "unknown fields")
        self.v4 = copy.deepcopy(self.boards[3])
        self.v4["schema_version"] = 2
        self.reject(self.v4, "schema_version")

    def test_strict_boolean_ids_and_frequency(self):
        for value in (0, "false", None):
            board = copy.deepcopy(self.v4)
            board["motors"]["channels"][0]["enable"]["complementary"] = value
            self.reject(board, "true or false")
        self.v4["gpio"][1]["id"] = 0
        self.reject(self.v4, "IDs must be")
        self.v4 = copy.deepcopy(self.boards[3])
        self.v4["motors"]["pwm_frequency_hz"] = 20000
        self.reject(self.v4, "pwm_frequency_hz")

    def test_motor_mode_specific_fields(self):
        self.v4["motors"]["control_model"] = "pwm"
        self.reject(self.v4, "SPI driver/buses")
        self.v4 = copy.deepcopy(self.boards[3])
        del self.v4["motors"]["channels"][0]["enable"]
        self.reject(self.v4, "missing fields")

    def test_unsupported_and_shared_timer_routes_fail(self):
        self.v4["motors"]["channels"][0]["enable"]["complementary"] = True
        self.reject(self.v4, "unsupported timer/pin route")
        self.v4 = copy.deepcopy(self.boards[3])
        self.v4["motors"]["channels"][1]["enable"] = copy.deepcopy(self.v4["motors"]["channels"][0]["enable"])
        self.reject(self.v4, "timer channel already used")
        board = copy.deepcopy(self.boards[2])
        board["encoders"][0].update(timer="TIM8", a="PC6", b="PC7")
        self.reject(board, "timer already used")
        board = copy.deepcopy(self.boards[0])
        board["motors"]["channels"][0]["pwm2"] = dict(pin="PA9", timer="TIM1", channel=2, complementary=False)
        self.reject(board, "same timer")

    def test_legacy_shared_pins_must_be_declared_exactly(self):
        board = copy.deepcopy(self.boards[0])
        del board["shared_pins"]
        self.reject(board, "pin PA2 is shared")
        board = copy.deepcopy(self.boards[0])
        board["shared_pins"][0]["users"].pop()
        self.reject(board, "exactly the conflicting users")
        self.v4["status_leds"][0]["pin"] = "PE14"
        self.reject(self.v4, "pin PE14 is shared")

    def test_bus_reference_and_package(self):
        self.v4["motors"]["driver"]["bus"] = "missing"
        self.reject(self.v4, "unknown bus")
        self.v4 = copy.deepcopy(self.boards[3])
        self.v4["mcu"]["part"] = "STM32F405RGT6"
        self.reject(self.v4, "pin unavailable")

    def test_usb_pins_are_reserved_on_all_boards(self):
        for original in self.boards:
            for pin in ("PA11", "PA12"):
                for resource in ("gpio", "status_leds"):
                    with self.subTest(board=original["board"]["revision"], pin=pin, resource=resource):
                        board = copy.deepcopy(original)
                        board[resource][0]["pin"] = pin
                        self.reject(board, f"pin {pin} is reserved by firmware for USB_OTG_FS")

    def test_usb_pins_cannot_be_used_for_motor_signals(self):
        for pin in ("PA11", "PA12"):
            for signal in ("direction", "disable", "sleep", "fault", "chip_select"):
                with self.subTest(pin=pin, signal=signal):
                    board = copy.deepcopy(self.v4)
                    board["motors"]["channels"][0][signal] = pin
                    self.reject(board, f"motors.0.{signal}: pin {pin} is reserved by firmware for USB_OTG_FS")

    def test_reserved_usb_pins_cannot_be_declared_shared(self):
        for pin in ("PA11", "PA12"):
            with self.subTest(pin=pin):
                board = copy.deepcopy(self.v4)
                board["gpio"][0]["pin"] = pin
                board["status_leds"][0]["pin"] = pin
                board["shared_pins"] = [{
                    "pin": pin, "users": ["gpio.0", "status_leds.0"],
                    "reason": "Attempt to reuse a firmware-reserved USB pin."
                }]
                self.reject(board, f"pin {pin} is reserved by firmware for USB_OTG_FS")

    def test_unimplemented_current_settings_fail(self):
        self.v4["motors"]["channels"][0]["hardware_current_limit"]["threshold_ma"] = 5400
        self.reject(self.v4, "factory default")
        self.v4["motors"]["channels"][0]["hardware_current_limit"].update(enabled=False, threshold_ma=6500)
        self.reject(self.v4, "factory default")

    def test_current_sense_type_is_required(self):
        del self.v4["motors"]["channels"][0]["current_sense"]["type"]
        self.reject(self.v4, "current_sense: missing field 'type'")

    def test_overcurrent_protection_schema(self):
        del self.v4["motors"]["channels"][0]["overcurrent_protection"]
        self.reject(self.v4, "missing fields.*overcurrent_protection")
        for value, message in ((None, "expected an object"), ({}, "missing fields"),
                               ({"mode": "latched_shutdown"}, "missing fields.*supported_modes"),
                               ({"supported_modes": ["latched_shutdown"]}, "missing fields.*mode")):
            with self.subTest(value=value):
                board = copy.deepcopy(self.boards[3])
                board["motors"]["channels"][0]["overcurrent_protection"] = value
                self.reject(board, "overcurrent_protection: " + message)
        board = copy.deepcopy(self.boards[3])
        board["motors"]["channels"][0]["overcurrent_protection"]["enabled"] = True
        self.reject(board, "overcurrent_protection: unknown fields.*enabled")

    def test_invalid_overcurrent_supported_modes_fail(self):
        for modes in (None, {}, "latched_shutdown", [], [None], [True], [0], [[]],
                      ["unknown"], ["latched_shutdown", "latched_shutdown"],
                      list(g.DRV8873_OVERCURRENT_MODES) + ["unknown"]):
            with self.subTest(modes=modes):
                board = copy.deepcopy(self.v4)
                board["motors"]["channels"][0]["overcurrent_protection"]["supported_modes"] = modes
                self.reject(board, "overcurrent_protection.supported_modes:")

    def test_overcurrent_mode_must_be_supported(self):
        self.v4["motors"]["channels"][0]["overcurrent_protection"]["supported_modes"] = ["automatic_retry"]
        self.reject(self.v4, "overcurrent_protection.mode: must be listed in supported_modes")

    def test_invalid_or_unimplemented_overcurrent_modes_fail(self):
        for mode in (None, True, 0, [], {}, "unknown", "LATCHED_SHUTDOWN"):
            with self.subTest(mode=mode):
                board = copy.deepcopy(self.v4)
                board["motors"]["channels"][0]["overcurrent_protection"]["mode"] = mode
                self.reject(board, "overcurrent_protection.mode: unsupported DRV8873S mode")
        for mode in ("automatic_retry", "report_only", "disabled"):
            with self.subTest(mode=mode):
                board = copy.deepcopy(self.v4)
                board["motors"]["channels"][0]["overcurrent_protection"]["mode"] = mode
                self.reject(board, "overcurrent_protection: only the latched_shutdown factory default")

    def test_current_limit_supported_thresholds_are_required(self):
        del self.v4["motors"]["channels"][0]["hardware_current_limit"]["supported_thresholds_ma"]
        self.reject(self.v4, "missing fields.*supported_thresholds_ma")

    def test_invalid_current_limit_supported_thresholds_fail(self):
        for values in (None, "6500", {}, [], [4000, 5400, 6500, 7000, 8000],
                       [True, 6500], [6500.0], ["6500"], [[6500]], [0, 6500],
                       [6500, 65536], [6500, 8000], [6500, 6500], [7000, 6500]):
            with self.subTest(values=values):
                board = copy.deepcopy(self.v4)
                board["motors"]["channels"][0]["hardware_current_limit"]["supported_thresholds_ma"] = values
                self.reject(board, "hardware_current_limit.supported_thresholds_ma:")

    def test_selected_current_limit_must_be_supported(self):
        self.v4["motors"]["channels"][0]["hardware_current_limit"]["supported_thresholds_ma"] = [4000, 5400, 7000]
        self.reject(self.v4, "threshold_ma: must be listed in supported_thresholds_ma")

    def test_unsupported_current_sense_types_fail(self):
        for kind in ("i2c", "spi", "ADC", "", None, True, 1, {}):
            with self.subTest(kind=kind):
                board = copy.deepcopy(self.v4)
                board["motors"]["channels"][0]["current_sense"]["type"] = kind
                self.reject(board, "current_sense.type: unsupported type")
        self.v4["motors"]["channels"][0]["current_sense"] = {
            "type": "i2c", "device": "motor_current_sensor", "channel": 0
        }
        self.reject(self.v4, "current_sense.type: unsupported type")

    def test_adc_current_sense_requires_adc_fields(self):
        sense = self.v4["motors"]["channels"][0]["current_sense"]
        del sense["pin"]
        self.reject(self.v4, "missing fields.*pin")
        sense["pin"] = "PE0"
        self.reject(self.v4, "ADC-capable pin")
        sense["pin"] = "PC4"
        sense["device"] = "motor_current_sensor"
        self.reject(self.v4, "unknown fields.*device")

    def test_build_selection_checks_mcu_and_conflicting_defines(self):
        self.assertEqual(g.select_build_board(self.boards, set(), "stm32f405rg")["board"]["revision"], "0.3.0")
        for board in self.boards:
            self.assertIs(g.select_build_board(self.boards, {board["board"]["build_define"]}, board["mcu"]["part"]), board)
        for defines, mcu, message in [({"MP_V1", "MP_V4"}, "stm32f405vg", "exactly one"),
                                      ({"MP_V9"}, "stm32f405vg", "Unsupported"),
                                      ({"MP_V4"}, "stm32f405rg", "needs STM32F405VGT6")]:
            with self.assertRaisesRegex(g.ConfigError, message):
                g.select_build_board(self.boards, defines, mcu)

    def test_multiple_status_leds_and_driver_slew_render(self):
        self.v4["status_leds"].append(dict(id=1, pin="PE0", active_low=True))
        self.v4["motors"]["driver"]["slew_rate_v_per_us"] = 10.8
        g.validate(self.v4)
        output = g.render([self.v4])["hw_config.h"]
        self.assertIn("#define NUMBER_STATUS_LEDS 2", output)
        self.assertIn(".activeLow = true", output)
        self.assertIn("#define DRV8873_SLEW_RATE_BITS 4U", output)

    def test_duplicate_board_identity_or_selector_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i, board in enumerate(self.boards):
                (Path(tmp) / f"{i}.json").write_text(json.dumps(board))
            (Path(tmp) / "copy.json").write_text(json.dumps(self.boards[3]))
            with self.assertRaisesRegex(g.ConfigError, "duplicate board.build_define"):
                g.load_boards(tmp)
            board = copy.deepcopy(self.boards[3])
            board["board"]["build_define"] = "MP_V5"
            (Path(tmp) / "copy.json").write_text(json.dumps(board))
            with self.assertRaisesRegex(g.ConfigError, "duplicate board model/revision"):
                g.load_boards(tmp)


if __name__ == "__main__":
    unittest.main()

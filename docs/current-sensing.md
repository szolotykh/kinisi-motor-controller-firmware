# Motor current sensing

`GET_MOTOR_CURRENT(motor_index)` reads a nonnegative current estimate in amperes.
`SUBSCRIBE(MOTOR_CURRENT, motor_index, interval_ms)` streams the same measurement
with its ADC read completion timestamp and clock metadata. Motor indices are
0–3; the minimum delivery interval is 20 ms. Both require protocol 2.7 within
major 2. Subscriptions also require READY and enabled heartbeat monitoring.

Reading or subscribing does not initialize, enable, or change a motor. Boards
without a fitted sensor return `SENSOR_NOT_AVAILABLE`. ADC failures return
`INTERNAL_ERROR`; saturation returns `SAMPLE_OUT_OF_RANGE`. A failed scheduled
read pauses delivery until recovery, without publishing an old sample.

## Measurement

The v0.4 DRV8873S high-side current mirrors share a 330 ohm sense resistor and
1 uF filter per motor. The nominal mirror ratio is 1100:1. ADC1 reads PC4, PC5,
PC0 and PC1 for motors 0, 1, 2 and 3 respectively. VREF+ connects to the nominal
3.3 V rail. Conversion parameters come from each motor's private board JSON;
the public profile exposes only whether current sensing is present.

The driver discards one conversion after selecting a channel and averages 16
12-bit conversions with long acquisition time. Conversion uses:

```text
amps = mean_adc_count / 4095 × reference_voltage_mv / 1000
       × current_mirror_ratio / sense_resistor_ohm
```

This is a filtered estimate of high-side current. PWM decay intervals affect
the reading; it is neither signed winding current nor battery current. No
duty-cycle compensation or calibration is applied. A zero reading can occur
with an unpowered or sleeping driver. See the [DRV8873 datasheet, Current Sense](https://www.ti.com/lit/ds/symlink/drv8873.pdf)
for mirror behavior and accuracy, and the [STM32F405 datasheet](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf)
for ADC characteristics.

Current measurement does not configure the driver's hardware current limit or
overcurrent protection.

## Validation

Run `python test/test_current_sense/run_tests.py` with a host C compiler in `CC`.
The host tests cover conversion, channel isolation, saturation, failure recovery,
and absence on older boards. Connection and SDK tests cover wire data and stream
lifecycle. Physical accuracy still requires comparison against a measured load.

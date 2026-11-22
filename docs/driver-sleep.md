# Motor-driver sleep

On v0.4.0 with protocol 2.15, a ready USB CDC or external I²C client can put all four motor drivers to sleep. The MCU and communication peripherals continue running. There is no timeout or implicit wake on motion, connect, or reconnect.

| Command | Result |
| --- | --- |
| `SET_DRIVER_SLEEP(1)` | Cancel active motor and platform control, coast the motors, inhibit new drive requests and assert all four driver sleep pins. Wait for the board ACK. |
| `GET_DRIVER_SLEEP()` | Read the shared inhibit. A result of 1 applies to all clients. A result of 0 permits motor commands. An individual uninitialized channel can still be asleep. |
| `SET_DRIVER_SLEEP(0)` | Wake only previously initialized channels, with bridges disabled. Verify driver configuration and the last verified current-limit setting on each channel before clearing the inhibit. Wait for ACK, then issue a fresh motion command when wanted. |

Entry from an active controller cancels its target. Platform geometry, encoder setup, motor directions and PWM frequency remain in RAM. Initialize or start a closed-loop controller again before sending it new targets. Existing subscriptions and heartbeat rules continue to apply. Fault reads can show the physical sleep pin and unavailable register data. Requests that would initialize, drive, brake, arm a controller, clear driver faults or access current-limit registers return `DRIVERS_ASLEEP` while inhibited.

The sleep request is volatile. Reconnecting sees the current request. Reset clears it and follows normal boot policy, including restoring an explicitly saved configuration with outputs stopped. Driver sleep and wake do not write FRAM. If sleep entry cannot verify the GPIO levels, the transition returns an error and the inhibit stays set. Retry sleep to check the pins again. A failed wake returns an error, keeps the inhibit set and requests sleep on all channels. Correct the cause and explicitly retry. A repeated wake on an already awake board does not stop a new motion command.

## Original v0.3 and Nano

Protocol 2.17 adds the same SET/GET_DRIVER_SLEEP commands on model 1 revisions 0.3.0 and 0.3.1 (Nano). Both DRV8870 inputs low select coast. Sustained coast also lets the chip enter sleep on its own. Explicit sleep adds a board-wide software inhibit. Entry cancels motion and holds all eight inputs low through the specified entry interval. The state query reports this inhibit. An independent host can reconnect, read the flag and explicitly clear it. Clearing the flag leaves each input pair low. A fresh drive or an explicitly requested brake can then wake the chip. A valid BRAKE_MOTOR or BRAKE_PLATFORM applies ordinary short braking to the selected motor or platform channels by setting both inputs high. A preceding nonzero drive is unnecessary. A subsequent coast returns both inputs low. An inhibited or superseded pre-wake brake is rejected just like a stale drive. TI specifies a maximum sleep-entry time of 1.5 ms, and describes a qualifying high input of at least 5 µs followed by up to 50 µs for wake. The release ACK measures the software transition. Physical first-drive timing and supply current require bench measurements. [TI DRV8870, electrical characteristics and section 7.3.2](https://www.ti.com/lit/ds/symlink/drv8870.pdf)

| Firmware motor | Driver | IN1 (pin 3) | IN2 (pin 2) |
| --- | --- | --- | --- |
| 0 | U301 | PB14, TIM12 CH1 | PB15, TIM12 CH2 |
| 1 | U303 | PC6, TIM8 CH1 | PC7, TIM8 CH2 |
| 2 | U302 | PC8, TIM8 CH3 | PC9, TIM8 CH4 |
| 3 | U304 | PA8, TIM1 CH1 | v0.3.0 uses PA9, TIM1 CH2. Nano uses PA10, TIM1 CH3. |

All eight routes use regular active-high timer outputs. Nano dedicates PA9 to USB VBUS sensing. Build `v0_3` (or the default `genericSTM32F405RG`) for original hardware and `genericSTM32F405RG_v0_3_1` for Nano. The board identity is compiled into the image. Select the correct image for the physical PCB. Neither profile exposes board idle. Compare explicit inhibit with the same board already in ordinary low/low coast, and measure both motor supply and USB input before attributing any electrical savings.

## Revision and wake-source feasibility

| Hardware | Driver control | MCU and peripherals |
| --- | --- | --- |
| v0.1.0 | The v0.1 schematic (`ae91214`) uses DRV8871 and paired PWM. Holding both inputs low can put the chip to sleep. The next drive input wakes it. There is no independent sleep pin or SPI readback. [DRV8871 datasheet](https://www.ti.com/lit/ds/symlink/drv8871.pdf) | STM32F405RGT6. Shallow CPU sleep can leave peripheral clocks running. This profile exposes no board temperature sensor or IMU. |
| v0.2.0, v0.3.0 and Nano | The v0.2 and v0.3 schematics (`e1089c2` and `40d90ca`) use DRV8870 and paired PWM. Both inputs held low can cause internal driver sleep. Only original v0.3 and Nano expose the protocol 2.17 inhibit. All three lack an independent sleep pin and SPI readback. | STM32F405RGT6. The same MCU sleep modes are available. These profiles expose no board temperature sensor or IMU. |
| v0.4.0 | Four DRV8873S drivers each have nSLEEP, DISABLE and SPI. Their sleep pins are PB15, PD2, PB4 and PB8, in motor index order. | STM32F405VGT6. Protocol 2.16 supports separate communication-preserving board idle. The motor and USB supplies feed the logic regulator through hardware ORing. Firmware has no identified switch that disconnects every rail or external load. |

All five board profiles support shallow Sleep with interrupt wake and SRAM retention. The explicit firmware board-idle capability is available on v0.4.0. Firmware services its USB and external I²C queues at 1 ms intervals and keeps the TIM14 timebase, clock synchronization and heartbeat running during board idle. [Board idle](board-idle.md)

STM32 STOP retains SRAM but halts the main clocks. Candidate wake sources include configured EXTI, RTC and USB resume from bus suspend. Resuming normal operation requires restoring the clock tree and reconciling elapsed time and session deadlines. An ordinary CDC payload or a successful addressed I²C transaction is not an established STOP wake source for this board. STANDBY loses normal SRAM and boots on wake. A reset or RTC wake would require a fresh session. On v0.4 the dedicated PA0 wake pin also serves encoder 1. A dedicated external wake signal or switched peripheral rail would need its own wiring and board review. STOP and STANDBY are deferred. [STM32F405 datasheet](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf) · [DRV8870 datasheet](https://www.ti.com/lit/ds/symlink/drv8870.pdf) · [DRV8873 datasheet](https://www.ti.com/lit/ds/symlink/drv8873.pdf)

## Provisional bench targets

The product owner approved these targets for a future bench comparison. Virtual-board results cover software behavior only.

1. Record board and firmware revisions, ambient temperature, instrument accuracy, sample interval and load wiring. Compare four initialized, unloaded and coasting channels against those same channels explicitly asleep. Match motor supply (13.5 V), USB connection and load, peripheral configuration, telemetry, and settling and averaging windows. Record motor-input and VBUS voltage and current simultaneously. Compare both rails and the sum of their input watts across paired runs.
2. Target a reduction of at least 10 mA at the motor input exceeding three times the combined measurement uncertainty of the difference. TI specifies chip operating current under a different DISABLE condition from the board's coast baseline. A datasheet estimate cannot establish the measured difference.
3. From an established READY session with all four channels previously initialized, issue a fresh wake request on each cycle. Time from sending the request through receiving the verified ACK on that same transport. Target p95 at or below 50 ms over 100 sleep/wake cycles each for USB CDC and I²C. Record failures, retries, maximum and bus configuration. Capture PWM and DISABLE across entry and wake and check the outputs remain off until a subsequent fresh drive command. Measure reconnect/handshake separately. A future board-idle power comparison uses driver-only sleep as its baseline.

The four driver checks run in sequence. Firmware inserts millisecond delays between SPI transactions in addition to the driver's wake interval. Include that overhead, the scheduler and host transport in the latency result.

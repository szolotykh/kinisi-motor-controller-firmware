# Communication-preserving board idle

On v0.4.0 with protocol 2.16, a ready USB CDC or external I²C client can explicitly idle the controller and wake it over either transport. Check the exact public board profile for `power.board_idle`. v0.1–v0.3 and v0.4 firmware through protocol 2.15 do not expose this mode.

| Request | Expected result |
| --- | --- |
| `SET_BOARD_IDLE(1)` | Stop active control, clear motion targets, coast all motors and verify driver sleep. Pause subscription sampling and delivery. Wait for ACK. |
| `GET_BOARD_IDLE()` | Read the shared idle request, including after reconnect. A value of 1 also covers a failed transition during which the board stays inhibited. |
| `SET_BOARD_IDLE(0)` | If the drivers were awake for use before entry, restore their previously initialized subset and verify their settings. If the operator had put the drivers to sleep separately, preserve that inhibit. Resume subscription acquisition. Wait for ACK. |

A successful wake leaves motor outputs stopped. A new motion request is required to drive, and closed-loop control must be started again. A failed transition returns an error and holds the idle and motion inhibits. Correct the cause and explicitly retry. Repeating a confirmed transition is safe. Use `GET_DRIVER_SLEEP` to check the driver inhibit separately. Motion during idle returns `DRIVERS_ASLEEP`; attempting to change driver sleep during board idle returns `BOARD_IDLE`.

## Connections, time and measurements

- USB CDC and external I²C remain usable. A second ready client can query state or wake the board. The ordinary watchdog, disconnect, INIT, READY and clock-sync rules still apply. Connection loss leaves board idle requested. Reconnect and perform the usual handshake to interrogate it. Boot clears the volatile power requests and follows the existing stopped configuration-restore path.
- Existing subscriptions stay registered as long as their session remains valid. The board pauses autonomous acquisition and emission, including event-triggered and on-change sampling. New subscriptions are rejected with `BOARD_IDLE`. Unsubscribe remains available. Polling during idle does not release a cached event. Normal session loss or reinitialization clears that connection's subscriptions.
- After wake the firmware reacquires current data before delivering a resumed stream. It sends no backlog of missed samples. The first valid sample can be delivered even if its value equals the last value before idle. A reading already delivered to a host is historical; its timestamp stays unchanged. Explicit sensor GET commands retain their normal cadence, validity and error rules. A host that polls them continues to generate work.
- The encoder and odometry timebase continues to account for external motion. Saved platform geometry, directions and PWM frequency survive idle. Neither transition writes FRAM. Peripheral supply rails remain powered.

## Scheduling and bench comparison

The firmware services both transport queues at 1 ms intervals, with a 1 kHz RTOS tick and TIM14 HAL time. In explicit board idle the v0.4 FreeRTOS idle task can execute a shallow wait-for-interrupt instruction between ready tasks. Normal interrupts resume execution, allowing the same command sessions, scheduler and monotonic clock to operate throughout. Wake of the CPU to handle an interrupt does not lift the board's idle request. The code leaves PLL and peripheral clocks in their normal configuration. STOP and STANDBY require a different wake and session design. [STM32F405/407 reference manual](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf)

To compare scheduled work, use identical configurations, already-inhibited motor drivers, subscriptions, heartbeat traffic and elapsed time. Count actual acquisitions, emitted events and served requests in comparable windows under driver-only sleep and board idle. Check a direct read while idle separately and verify fresh publication after wake. The native harness models firmware execution and hardware calls. CPU sleep residency and electrical consumption require board instrumentation.

For a future physical comparison, record board and firmware revisions, motor supply and VBUS voltage/current simultaneously, ambient temperature, attached loads and measurement uncertainty. Keep driver state, sensor configuration, USB host or I²C traffic, subscriptions, settling time and averaging window identical. Compare total input watts and integrated energy across matched intervals. Record results with and without subscriptions. For each transport, measure a fresh wake request on an established READY session through its verified ACK, time to the first fresh subscribed sample, and any failures. Record retained-driver and verified-driver-wake cases separately. Capture PWM and DISABLE through the transition. Physical power reduction, CPU sleep residency and wake latency are unmeasured.

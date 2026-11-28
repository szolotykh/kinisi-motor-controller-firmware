# Nano USB VBUS detection

Nano (model 1, hardware 0.3.1, `MP_V3_1`) routes USB VBUS through the
R107/R108 divider to PA9. Firmware configures PA9 as a regular GPIO input with
no internal pull. The USB peripheral's embedded VBUS sensing is disabled.
This follows the divider guidance in [ST AN4879, section 2.6](https://www.st.com/resource/en/application_note/an4879-usb-hardware-and-pcb-guidelines-stmicroelectronics.pdf).
[RM0090's OTG_FS GCCFG register](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
describes disabling the embedded comparator and monitoring VBUS in software
for device-only applications.

The USB stack initializes without waiting for VBUS. With VBUS absent, its
pull-up remains disconnected and external I2C initialization proceeds. With
VBUS present at boot, USB starts immediately after stack initialization.
`MX_USB_DEVICE_Process()` samples PA9 from the existing command task, which
normally delays 1 ms between iterations. A high sample starts USB; a low sample
after attachment synchronously stops shared motion and invalidates sessions
before stopping USB, even if CDC had never reached configuration. Repeated samples do not restart the
stack. Original v0.3 retains PA9 as motor PWM; other profiles retain their
existing USB startup behavior and do not read PA9 in this service.

Detach runs before queued commands. The command-manager quiesce path stops
shared outputs, cancels controllers, clears complete and incomplete commands
on both transports and invalidates both sessions before USB teardown can
wait or fail. Reconnection requires a fresh handshake and does not resume motion.
CDC deinitialization/deconfiguration continues to request the same cleanup.
Only the USB IRQ is masked across task-side start/stop and class teardown;
the timebase remains available for HAL timeouts. Nano's stop path discards RX
FIFO data and endpoint interrupt state, clears pending USB interrupt status,
and resets USB configuration/address state before another attachment.

Healthy USB attachment preserves an existing independent I2C session. This
relies on the installed STM32 HAL's `HAL_PCD_Start` being bounded: it locks the
handle, enables USB interrupts and connects the pull-up without a wait loop.
If start returns an error, firmware quiesces shared motion immediately, before
attempting USB cleanup. A start, stop or FIFO-flush failure latches USB
unavailable until MCU reset and leaves its IRQ disabled. It does not enter the
global error handler or retry on every task tick/cable edge. The command task,
timebase and I2C remain running; I2C requires a new INIT/READY handshake before
commands can resume. Recovery does not restore motion targets. A future HAL
change that adds a blocking start path requires revisiting this ordering.

Run the native lifecycle regression with:

```powershell
python test/test_usb_vbus/run_tests.py
```

Set `CC` to a host GCC or Clang if it is not on `PATH`. The harness executes
production `usb_device.c`, command-manager quiesce, queue and session logic
with fake GPIO, NVIC, motor/controller and USB-stack boundaries. It
covers absent/present boot, repeated samples, pre-enumeration and configured
detach, reconnect, IRQ-mask preservation, failed start, and isolation of all
five board profiles. Injected start, stop and low-level flush failures assert
that all output/control state is stopped and both sessions/queues are invalid
before failing teardown, that no retry storm occurs, and that I2C completes a
fresh handshake and read without restarting motion. The configuration suite checks that the command-task
hook remains compatible with both transports. The Nano PlatformIO build
compiles the real STM32 HAL and USB low-level integration.

These checks do not qualify physical USB behavior. Bench verification remains
required for divider input levels over supply/component tolerances, powered
and unpowered cable insertion, enumeration/re-enumeration, suspend/unplug,
motor stopping on detach, and I2C operation with USB absent. Polling is not an
edge latch: a low pulse entirely between service calls can be missed, and the
1 ms task delay is not a measured worst-case detach latency. No physical USB,
VBUS threshold, or timing measurements are claimed by the native tests.

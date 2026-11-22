// Production USB lifecycle with host fakes at the GPIO/NVIC and USB-stack boundary.
#include "usb_device.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
#include "board_revision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
USBD_DescriptorsTypeDef FS_Desc;
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
int USBD_CDC;
static unsigned starts, stops, gpio_reads, clears, barriers;
extern unsigned losses;
void test_prepare_running(void);
void test_assert_running(void);
void test_assert_quiesced(void);
void test_i2c_recover(void);
static unsigned initialized, registered, interface_registered;
static int present, irq_enabled = 1, fail_start, fail_stop, fail_flush;

uint32_t NVIC_GetEnableIRQ(int irq) { assert(irq == OTG_FS_IRQn); return irq_enabled; }
void HAL_NVIC_DisableIRQ(int irq) { assert(irq == OTG_FS_IRQn); irq_enabled = 0; }
void HAL_NVIC_EnableIRQ(int irq) { assert(irq == OTG_FS_IRQn); irq_enabled = 1; }
void HAL_NVIC_ClearPendingIRQ(int irq) { assert(irq == OTG_FS_IRQn && !irq_enabled); ++clears; }
void __DSB(void) { assert(!irq_enabled); ++barriers; }
void __ISB(void) { assert(!irq_enabled); ++barriers; }
int HAL_GPIO_ReadPin(void *port, uint16_t pin) {
    assert(port == GPIOA && pin == GPIO_PIN_9); ++gpio_reads; return present;
}
void Error_Handler(void) { assert(!"USB runtime faults must not globally halt"); }
int USBD_Init(USBD_HandleTypeDef *device, USBD_DescriptorsTypeDef *desc, uint8_t id) {
    assert(desc == &FS_Desc && id == DEVICE_FS);
    memset(device, 0, sizeof(*device)); device->dev_state = USBD_STATE_DEFAULT;
    ++initialized; return USBD_OK;
}
int USBD_RegisterClass(USBD_HandleTypeDef *device, void *klass) {
    assert(device == &hUsbDeviceFS && initialized && klass == &USBD_CDC);
    ++registered; return USBD_OK;
}
int USBD_CDC_RegisterInterface(USBD_HandleTypeDef *device, USBD_CDC_ItfTypeDef *itf) {
    assert(device == &hUsbDeviceFS && registered && itf == &USBD_Interface_fops_FS);
    ++interface_registered; return USBD_OK;
}
int USBD_Start(USBD_HandleTypeDef *device) {
    assert(device == &hUsbDeviceFS && interface_registered);
#if KINISI_USB_VBUS_SENSING
    assert(present && !irq_enabled && barriers >= 2);
#endif
    ++starts; return fail_start ? USBD_FAIL : USBD_OK;
}
int USBD_Stop(USBD_HandleTypeDef *device) {
    assert(!irq_enabled && losses > stops);
    // The real command-manager safety path must complete before either a
    // bounded stop failure or a FIFO timeout can occur.
    test_assert_quiesced();
    device->pClassData = NULL; ++stops;
    // Deliberately no CDC DeInit notification: covers a loss before enumeration.
    if (fail_flush) MX_USB_DEVICE_Fault(); // Core USBD_Stop discards LL status.
    return fail_stop ? USBD_FAIL : USBD_OK;
}
int main(int argc, char **argv) {
    present = argc > 1 && strcmp(argv[1], "present") == 0;
    MX_USB_DEVICE_Init();
    assert(initialized == 1 && registered == 1 && interface_registered == 1);
#if KINISI_USB_VBUS_SENSING
    assert(starts == (unsigned)present && !stops && !losses && irq_enabled);
    if (argc > 2) {
        test_prepare_running();
        if (!strcmp(argv[2], "start-fail")) {
            fail_start = 1; present = 1;
        } else {
            present = 1; MX_USB_DEVICE_Process(); test_assert_running();
            fail_stop = !strcmp(argv[2], "stop-fail");
            fail_flush = !strcmp(argv[2], "flush-fail");
            present = 0;
        }
        MX_USB_DEVICE_Process();
        test_assert_quiesced();
        assert(!irq_enabled && losses == 1 && stops == 1);
        unsigned attempts = starts;
        // No retry storm on stable high or cable bounce; USB is latched off.
        for (int i = 0; i < 100; ++i) { present = i % 2; MX_USB_DEVICE_Process(); }
        assert(starts == attempts && stops == 1 && losses == 1 && !irq_enabled);
        test_i2c_recover();
        puts("PASS synchronous safe outputs/session/queue ordering before USB failure; no retry storm");
        return 0;
    }
    if (!present) {
        for (int i = 0; i < 100; ++i) MX_USB_DEVICE_Process();
        assert(!starts && !stops && !losses); // I2C/task can run without USB.
        test_prepare_running();
        present = 1; MX_USB_DEVICE_Process(); test_assert_running();
    }
    assert(starts == 1 && irq_enabled);
    for (int i = 0; i < 10; ++i) MX_USB_DEVICE_Process();
    assert(starts == 1);
    // Unconfigured cable loss still invalidates; a second low sample is inert.
    test_prepare_running();
    present = 0; MX_USB_DEVICE_Process(); MX_USB_DEVICE_Process();
    assert(stops == 1 && losses == 1 && clears == 1 && irq_enabled);
    present = 1; MX_USB_DEVICE_Process();
    assert(starts == 2 && hUsbDeviceFS.dev_state == USBD_STATE_DEFAULT);
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.dev_config = 1; hUsbDeviceFS.dev_address = 23;
    hUsbDeviceFS.pClassData = &hUsbDeviceFS;
    // A pre-existing USB IRQ mask must remain masked on both edges.
    test_prepare_running();
    irq_enabled = 0; present = 0; MX_USB_DEVICE_Process();
    assert(!irq_enabled && losses == 2 && stops == 2 && clears == 2);
    assert(hUsbDeviceFS.dev_state == USBD_STATE_DEFAULT && !hUsbDeviceFS.dev_config &&
           !hUsbDeviceFS.dev_address && !hUsbDeviceFS.pClassData);
    present = 1; MX_USB_DEVICE_Process(); assert(starts == 3 && !irq_enabled);
    test_prepare_running();
    irq_enabled = 1; present = 0; MX_USB_DEVICE_Process();
    test_prepare_running();
    fail_start = 1; present = 1;
    MX_USB_DEVICE_Process(); test_assert_quiesced();
    assert(!irq_enabled); // Never expose a partially failed start to IRQs.
#else
    assert(starts == 1 && !gpio_reads);
    for (int i = 0; i < 100; ++i) { present = i % 2; MX_USB_DEVICE_Process(); }
    assert(starts == 1 && !stops && !losses && !gpio_reads && irq_enabled);
#endif
    puts("PASS USB boot/attach/detach/reconnect, explicit loss, IRQ fencing and profile isolation");
    return 0;
}

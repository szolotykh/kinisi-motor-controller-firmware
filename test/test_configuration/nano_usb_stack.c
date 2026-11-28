// Only the vendor USB stack, NVIC and PA9 are faked here; usb_device.c
// performs the normal Nano VBUS polling, startup, loss and reattachment.
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include <assert.h>
#include <stdbool.h>
#include <string.h>

USBD_DescriptorsTypeDef FS_Desc;
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
int USBD_CDC;
unsigned usb_starts, usb_stops;
bool vbus_present = true;
static bool irq_enabled = true;
uint32_t NVIC_GetEnableIRQ(int irq) { assert(irq == OTG_FS_IRQn); return irq_enabled; }
void HAL_NVIC_DisableIRQ(int irq) { assert(irq == OTG_FS_IRQn); irq_enabled = false; }
void HAL_NVIC_EnableIRQ(int irq) { assert(irq == OTG_FS_IRQn); irq_enabled = true; }
void HAL_NVIC_ClearPendingIRQ(int irq) { assert(irq == OTG_FS_IRQn && !irq_enabled); }
void __DSB(void) { assert(!irq_enabled); }
void __ISB(void) { assert(!irq_enabled); }
int HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    assert(port == GPIOA && pin == GPIO_PIN_9);
    return vbus_present;
}
void Error_Handler(void) { assert(!"unexpected USB failure"); }
int USBD_Init(USBD_HandleTypeDef *device, USBD_DescriptorsTypeDef *desc, uint8_t id)
{
    assert(desc == &FS_Desc && id == DEVICE_FS);
    memset(device, 0, sizeof(*device));
    return USBD_OK;
}
int USBD_RegisterClass(USBD_HandleTypeDef *d, void *k) { assert(d && k == &USBD_CDC); return USBD_OK; }
int USBD_CDC_RegisterInterface(USBD_HandleTypeDef *d, USBD_CDC_ItfTypeDef *i)
{ assert(d && i == &USBD_Interface_fops_FS); return USBD_OK; }
int USBD_Start(USBD_HandleTypeDef *device)
{
    assert(vbus_present && !irq_enabled);
    ++usb_starts;
    device->dev_state = USBD_STATE_CONFIGURED;
    device->dev_address = 20 + usb_starts;
    device->pClassData = device;
    return USBD_OK;
}
int USBD_Stop(USBD_HandleTypeDef *device)
{
    assert(!vbus_present && !irq_enabled);
    ++usb_stops;
    device->pClassData = NULL;
    return USBD_OK;
}

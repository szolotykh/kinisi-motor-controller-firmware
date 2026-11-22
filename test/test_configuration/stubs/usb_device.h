#pragma once
typedef struct { unsigned dev_state; void *pClassData; } USBD_HandleTypeDef;
#define USBD_STATE_CONFIGURED 1
void MX_USB_DEVICE_Init(void);
void MX_USB_DEVICE_Process(void);

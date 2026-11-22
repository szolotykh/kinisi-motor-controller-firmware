#pragma once
#ifdef KINISI_CONFIGURATION_REAL_USB
#include "usbd_def.h"
#else
typedef struct { unsigned dev_state; void *pClassData; } USBD_HandleTypeDef;
#define USBD_STATE_CONFIGURED 1
#endif
void MX_USB_DEVICE_Init(void);
void MX_USB_DEVICE_Process(void);

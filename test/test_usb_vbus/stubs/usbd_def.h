#pragma once
#include <stdint.h>
#define USBD_OK 0
#define USBD_FAIL 1
#define USBD_STATE_DEFAULT 1
#define USBD_STATE_CONFIGURED 3
#define DEVICE_FS 0
typedef struct { uint8_t dev_state, dev_config, dev_address; void *pClassData; } USBD_HandleTypeDef;
typedef struct { int unused; } USBD_DescriptorsTypeDef;

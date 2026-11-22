#pragma once
#include "usbd_def.h"
typedef struct { int unused; } USBD_CDC_ItfTypeDef;
extern int USBD_CDC;
int USBD_CDC_RegisterInterface(USBD_HandleTypeDef *, USBD_CDC_ItfTypeDef *);

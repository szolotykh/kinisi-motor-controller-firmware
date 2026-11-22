/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "board_revision.h"
#include "commands_manager.h"

void Error_Handler(void);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

#if KINISI_USB_VBUS_SENSING
static uint8_t vbus_attached;
static uint8_t vbus_faulted;
#endif

void MX_USB_DEVICE_Fault(void)
{
#if KINISI_USB_VBUS_SENSING
  vbus_faulted = 1;
#endif
}

/** Service Nano's divided VBUS as a regular GPIO, from the command task only. */
void MX_USB_DEVICE_Process(void)
{
#if KINISI_USB_VBUS_SENSING
  if (vbus_faulted) return; // No retry storm; I2C remains serviceable until reset.
  uint8_t present = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET;
  if (present == vbus_attached) return;

  /* CDC callbacks and endpoint IRQs must not race task-side class teardown.
     Keep the timebase running: HAL FIFO waits need interrupts for timeouts. */
  uint32_t irq_enabled = NVIC_GetEnableIRQ(OTG_FS_IRQn);
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  __DSB();
  __ISB();
  if (present) {
    /* HAL_PCD_Start only locks, enables IRQs and connects the pull-up; it has
       no wait loop. Healthy attachment must not interrupt independent I2C. */
    if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
      commands_manager_usb_quiesce();
      MX_USB_DEVICE_Fault();
      (void)USBD_Stop(&hUsbDeviceFS);
      hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;
      hUsbDeviceFS.dev_config = 0;
      hUsbDeviceFS.dev_address = 0;
      HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
      return; // Quarantine USB only; do not halt with global interrupts disabled.
    }
  } else {
    /* Coasting cannot depend on CDC callbacks or successful FIFO teardown. */
    commands_manager_usb_quiesce();
    if (USBD_Stop(&hUsbDeviceFS) != USBD_OK) MX_USB_DEVICE_Fault();
    hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;
    hUsbDeviceFS.dev_config = 0;
    hUsbDeviceFS.dev_address = 0;
    HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
  }
  vbus_attached = present;
  if (irq_enabled && !vbus_faulted) HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
#endif
}

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
#if KINISI_USB_VBUS_SENSING
  /* PCD initialization leaves the pull-up disconnected. Do not wait for VBUS;
     external I2C must start even when no USB cable is present. */
  vbus_attached = 0;
  vbus_faulted = 0;
  MX_USB_DEVICE_Process();
#else
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }
#endif

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

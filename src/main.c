//------------------------------------------------------------
// File name: main.c
//------------------------------------------------------------
#include "main.h"
#include "stm32f4xx.h"
#include "hw_gpio.h"
#include "usb_device.h"
#include <cmsis_os.h>
#include "commands_manager.h"

#define HAL_PCD_MODULE_ENABLED
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void OTG_FS_IRQHandler(void);

commands_manager_t commandsManager;

int main(void)
{ 
    HAL_Init();
    SystemClock_Config();
    initialize_status_led();
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    osKernelInitialize();
    commands_manager_init(&commandsManager);
    osKernelStart();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) {}
}


void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void DebugMon_Handler(void)
{
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
// Interrupt handler for USB
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
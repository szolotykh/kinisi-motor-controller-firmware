//------------------------------------------------------------
// File name: main.c
//------------------------------------------------------------
#include "main.h"
#include "stm32f4xx.h"
#include "hw_gpio.h"
#include "hw_motor.h"
#include "hw_temperature.h"
#include "hw_imu.h"
#include "platform_fusion.h"
#if BOARD_HAS_FUSION_EXPERIMENT
#include "cmsis_os.h"

static void imu_acquire(void *unused)
{
    (void)unused;
    for (;;) {
        uint32_t epoch = platform_fusion_imu_epoch();
        if (epoch && !hw_motor_drivers_sleeping()) {
            hw_imu_poll_observations(epoch);
            osDelay(2);
        } else osDelay(20);
    }
}

static bool start_imu_acquisition(void)
{
    static osThreadId_t thread;
    if (!thread) thread = osThreadNew(imu_acquire, NULL, &(osThreadAttr_t){
        .name = "ImuAcquisition", .priority = osPriorityBelowNormal, .stack_size = 1536});
    return thread != NULL;
}
#endif
#include "hw_current_sense.h"
#include "usb_device.h"
#include "os_interface.h"
#include "commands_manager.h"

#define HAL_PCD_MODULE_ENABLED
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void OTG_FS_IRQHandler(void);

int main(void)
{ 
    HAL_Init();
    SystemClock_Config();
    // The hardware layer selects driver preparation from the board configuration.
    hw_motor_init();
    hw_temperature_init();
    hw_imu_init();
    hw_current_sense_init();
    
    // Initialize GPIO interface
    const gpio_interface_t* gpio = get_gpio_interface();
    gpio->init_status_led();
    
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    const os_interface_t* os = get_os_interface();
    os->kernel_init();
#if BOARD_HAS_FUSION_EXPERIMENT
    platform_fusion_install_imu(hw_imu_read_before, start_imu_acquisition);
#endif
    commands_manager_start();
    os->kernel_start();
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

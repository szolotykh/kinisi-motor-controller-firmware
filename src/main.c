#include "stm32f4xx.h"
#include "gpio.h"
#include "motor.h"
#include "usb_device.h"
#include "encoder.h"
#include "commands.h"
#include "main.h"
#include "platform.h"

#include <stdio.h>

#define HAL_PCD_MODULE_ENABLED
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void SystemClock_Config(void);
void OTG_FS_IRQHandler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    // Init USB
    MX_USB_DEVICE_Init();

    while (1)
    {
        if(commandBuffer[0] != '\0')
        {
        switch(commandBuffer[0])
        {
            case INITIALIZE_MOTOR:
                initialize_motor(commandBuffer[1]);
            break;

            case SET_MOTOR_SPEED:
                {
                signed short speed = commandBuffer[3] | commandBuffer[4]<<8;
                signed short direction = commandBuffer[2];
                set_motor_speed(commandBuffer[1], direction, speed);
                }
            break;

            case INITIALIZE_ENCODER:
                initialize_encoder(commandBuffer[1]);
            break;

            case GET_ENCODER_VALUE:
                {
                unsigned int value = get_encoder_value(commandBuffer[1]);
                char buf[4];
                buf[3] = (value >> 24) & 0xFF;
                buf[2] = (value >> 16) & 0xFF;
                buf[1] = (value >> 8) & 0xFF;
                buf[0] = value & 0xFF;
                CDC_Transmit_FS(buf, 4);
                }
            break;

            case STATUS_LED_TOGGLE:
                gpio_toggle_status_led();
            break;

            case PLATFORM_INITIALIZE:
                init_platform();
            break;

            case PLATFORM_SET_VELOCITY_INPUT:
                set_velocity_input(commandBuffer[1], commandBuffer[2], commandBuffer[3]);
            break;
            }

            // Reset command buffer
            memset(commandBuffer, '\0', 64);
        }
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
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

void SVC_Handler(void)
{
}


void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

void SystemClock_Config(void)
{
    // Endbling clock on port H since using cristal for HSE
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

// Interrupt handler for USB
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
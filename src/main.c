#include "stm32f4xx.h"
#include "hw_gpio.h"
#include "hw_motor.h"
#include "usb_device.h"
#include "hw_encoder.h"
#include "commands.h"
#include "main.h"
#include "platform.h"

#include "cmsis_os.h"
#include <FreeRTOS.h>
#include "task.h"
#include <stdio.h>

#define HAL_PCD_MODULE_ENABLED
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void SystemClock_Config(void);
void OTG_FS_IRQHandler(void);

char commandBuffer[64];

TIM_HandleTypeDef htim14;


osThreadId_t CommandsTaskHandle;
const osThreadAttr_t CommandsTask_attributes = {
  .name = "CommandsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControllerTask */
osThreadId_t ControllerTaskHandle;
const osThreadAttr_t ControllerTask_attributes = {
  .name = "ControllerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommandsQueue01 */
osMessageQueueId_t CommandsQueue01Handle;
const osMessageQueueAttr_t CommandsQueue01_attributes = {
  .name = "CommandsQueue01"
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void Callback01(void *argument);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    initialize_gpio();

    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    // Init USB
    // MX_USB_DEVICE_Init();  ???

    osKernelInitialize();
    myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);
    CommandsQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &CommandsQueue01_attributes);
    CommandsTaskHandle = osThreadNew(StartDefaultTask, NULL, &CommandsTask_attributes);
    ControllerTaskHandle = osThreadNew(StartTask02, NULL, &ControllerTask_attributes);

    osKernelStart();

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

/*
void SysTick_Handler(void)
{
    HAL_IncTick();
}
*/
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
/*
void SVC_Handler(void)
{
}
*/

void DebugMon_Handler(void)
{
}
/*
void PendSV_Handler(void)
{
}
*/
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  /*Configure the TIM14 IRQ priority */
  HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, TickPriority ,0);

  /* Enable the TIM14 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
  /* Enable TIM14 clock */
  __HAL_RCC_TIM14_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM14 clock */
  uwTimclock = 2*HAL_RCC_GetPCLK1Freq();
  /* Compute the prescaler value to have TIM14 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM14 */
  htim14.Instance = TIM14;

  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM14CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim14.Init.Period = (1000000U / 1000U) - 1U;
  htim14.Init.Prescaler = uwPrescalerValue;
  htim14.Init.ClockDivision = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htim14) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim14);
  }

  /* Return function status */
  return HAL_ERROR;
}

// Suspend Tick increment.
// Disable the tick increment by disabling TIM14 update interrupt.
void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim14, TIM_IT_UPDATE);
}

// Resume Tick increment.
// Enable the tick increment by Enabling TIM14 update interrupt.
void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim14);
}


void StartDefaultTask(void *argument)
{
  MX_USB_DEVICE_Init();
  for(;;)
  {
    uint8_t *data = "Hello World!\n";
	CDC_Transmit_FS((uint8_t*) data, strlen(data));
    osDelay(1000);
  }
}

void StartTask02(void *argument)
{
  for(;;)
  {
    gpio_toggle_status_led();
    osDelay(500);
  }
}

/* Callback01 function */
void Callback01(void *argument)
{
}
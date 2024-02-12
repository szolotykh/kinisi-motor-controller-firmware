//------------------------------------------------------------
// File name: encoder.c
//------------------------------------------------------------

#include "stdbool.h"
#include "hw_encoder.h"
#include "hw_timers.h"
#include "hw_config.h"
#include "stm32f4xx_hal.h"

typedef struct
{
	bool is_initialized;
	double resolution; // Ticks per revolution
	bool is_reversed;
}encoder_status_t;

static encoder_status_t encoder_status[NUMBER_ENCODERS] = {0};

static void initialize_encoder_timer(TIM_HandleTypeDef *htim, TIM_TypeDef *typeDef);

void initialize_encoder(encoder_index_t index, double encoder_resolution, uint8_t is_reversed) {
	if(!encoder_status[index].is_initialized){
		TIM_HandleTypeDef *htim = get_timer_handeler(encoder_info[index].timer);
		initialize_encoder_timer(htim, encoder_info[index].timer);
		HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
		encoder_status[index].resolution = encoder_resolution;
		encoder_status[index].is_reversed = is_reversed;
		encoder_status[index].is_initialized = true;
	}
}

uint8_t encoder_is_initialized(encoder_index_t index){
	return encoder_status[index].is_initialized;
}

uint16_t get_encoder_value(encoder_index_t index){

	TIM_HandleTypeDef *htim = get_timer_handeler(encoder_info[index].timer);
	if(encoder_status[index].is_reversed){
		return 65535 - htim->Instance->CNT;
	}
	return htim->Instance->CNT;
}

uint8_t get_encoder_direction(encoder_index_t index){

	// TODO: implement base on velocity
	return 0;
}

static void initialize_encoder_timer(TIM_HandleTypeDef *htim, TIM_TypeDef *typeDef)
{
	if(HAL_TIM_Base_GetState(htim) == HAL_TIM_STATE_READY){
		return;
	}

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim->Instance = typeDef;
	htim->Init.Prescaler = 0;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = 65535; // This value can't be 4294967295 since some of the timeers are 16 bit.
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(htim, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

encoder_info_t get_encoder_info(TIM_TypeDef* timTypeDef)
{
	for(int i = 0; i < NUMBER_ENCODERS; i++ ){
		if(timTypeDef == encoder_info[i].timer){
			return encoder_info[i];
		}
	}

	// TODO: Add assert here.
	encoder_info_t encoder_configuration;
	return encoder_configuration;
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
	encoder_info_t encoder_configuration = get_encoder_info(htim_encoder->Instance);

	GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
	rcc_tim_clk_enable(encoder_configuration.timer);

    rcc_gpio_clk_enable(encoder_configuration.portA);
    GPIO_InitStruct.Pin = encoder_configuration.pinA;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = get_alternate_function_mapping(encoder_configuration.timer);
    HAL_GPIO_Init(encoder_configuration.portA, &GPIO_InitStruct);

    if(encoder_configuration.portA != encoder_configuration.portB)
    {
    	rcc_gpio_clk_enable(encoder_configuration.portB);
    }
    GPIO_InitStruct.Pin = encoder_configuration.pinB;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = get_alternate_function_mapping(encoder_configuration.timer);
    HAL_GPIO_Init(encoder_configuration.portB, &GPIO_InitStruct);
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{
    encoder_info_t encoder_configuration = get_encoder_info(htim_encoder->Instance);
    
    if(htim_encoder->Instance == encoder_configuration.timer)
    {
        // Disable the peripheral clock
        rcc_tim_clk_disable(encoder_configuration.timer);

        // DeInit GPIO for Channel A and B
        HAL_GPIO_DeInit(encoder_configuration.portA, encoder_configuration.pinA);
    	HAL_GPIO_DeInit(encoder_configuration.portB, encoder_configuration.pinB);
    }
}
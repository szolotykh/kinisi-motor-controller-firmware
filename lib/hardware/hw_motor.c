//------------------------------------------------------------
// Board-selected motor control: dual PWM or enable/direction.
//------------------------------------------------------------
#include "hw_motor.h"
#include "hw_config.h"
#include "hw_timers.h"
#include <math.h>

extern void Error_Handler(void);

#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
#include "hw_drv8873.h"
#elif MOTOR_CONTROL_MODEL != MOTOR_CONTROL_MODEL_PWM
#error "Unsupported motor control model"
#endif

typedef struct {
    bool isInitialized;
    bool timerInitialized;
    bool isReversed;
} motor_status_t;
static motor_status_t motor_status[NUMBER_MOTORS];

static bool init_motor_timer(const motor_info_t *info);
static void stop_motor(motorIndex index);

static void initialize_motor(motorIndex index, bool reverse) {
    if (index >= NUMBER_MOTORS) return;
    const motor_info_t *info = &motor_info[index];
    // Each board compiles a single model; reject a mismatched motor entry.
    if (info->controlModel != MOTOR_CONTROL_MODEL) return;
    motor_status[index].isReversed = reverse;
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    // VM can cycle while USB keeps the MCU alive. Every explicit initialization
    // must revalidate the driver, with the bridge disabled and its duty cleared.
    stop_motor(index);
    motor_status[index].isInitialized = false;
    if (!hw_drv8873_initialize(index)) return;
#else
    if (motor_status[index].isInitialized) return;
#endif
    // A failed driver reinitialization does not reset its running timer channel.
    if (!motor_status[index].timerInitialized)
        motor_status[index].timerInitialized = init_motor_timer(info);
    motor_status[index].isInitialized = motor_status[index].timerInitialized;
}

static uint8_t motor_is_initialized(motorIndex index) {
    return index < NUMBER_MOTORS && motor_status[index].isInitialized;
}
static uint8_t motor_is_reversed(motorIndex index) {
    return index < NUMBER_MOTORS && motor_status[index].isReversed;
}

static void set_motor_speed(motorIndex index, double pwm) {
    if (!motor_is_initialized(index)) return;
    if (!isfinite(pwm)) { stop_motor(index); return; }
    if (pwm > 100.0) pwm = 100.0;
    if (pwm < -100.0) pwm = -100.0;
    const motor_info_t *info = &motor_info[index];
    TIM_HandleTypeDef *timer = get_timer_handeler(info->timer);
    bool direction = (pwm > 0) ^ motor_status[index].isReversed;
    uint16_t speed = fabs(pwm) * MOTOR_MAX_SPEED / 100.0;
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    if (pwm == 0 || hw_drv8873_fault_active(index)) {
        stop_motor(index);
        return;
    }
    HAL_GPIO_WritePin(info->disable.port, info->disable.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(info->direction.port, info->direction.pin,
                      direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // Enable compare preload is disabled, so no stale duty is released.
    __HAL_TIM_SET_COMPARE(timer, info->enable.timerChannel, speed);
    HAL_GPIO_WritePin(info->disable.port, info->disable.pin, GPIO_PIN_RESET);
#else
    speed = MOTOR_MAX_SPEED - speed;
    __HAL_TIM_SET_COMPARE(timer, info->pwmChannel1.timerChannel, direction ? speed : MOTOR_MAX_SPEED);
    __HAL_TIM_SET_COMPARE(timer, info->pwmChannel2.timerChannel, direction ? MOTOR_MAX_SPEED : speed);
#endif
}

static void stop_motor(motorIndex index) {
    if (!motor_is_initialized(index)) return;
    const motor_info_t *info = &motor_info[index];
    TIM_HandleTypeDef *timer = get_timer_handeler(info->timer);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    HAL_GPIO_WritePin(info->disable.port, info->disable.pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(timer, info->enable.timerChannel, 0);
#else
    __HAL_TIM_SET_COMPARE(timer, info->pwmChannel1.timerChannel, 0);
    __HAL_TIM_SET_COMPARE(timer, info->pwmChannel2.timerChannel, 0);
#endif
}

static void brake_motor(motorIndex index) {
    if (!motor_is_initialized(index)) return;
    const motor_info_t *info = &motor_info[index];
    TIM_HandleTypeDef *timer = get_timer_handeler(info->timer);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    HAL_GPIO_WritePin(info->disable.port, info->disable.pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(timer, info->enable.timerChannel, 0);
    // In DRV8873 PH/EN mode, EN low brakes both outputs high.
    HAL_GPIO_WritePin(info->disable.port, info->disable.pin, GPIO_PIN_RESET);
#else
    __HAL_TIM_SET_COMPARE(timer, info->pwmChannel1.timerChannel, MOTOR_MAX_SPEED);
    __HAL_TIM_SET_COMPARE(timer, info->pwmChannel2.timerChannel, MOTOR_MAX_SPEED);
#endif
}

static const hw_motor_interface_t default_motor_interface = {
    .initialize = initialize_motor, .is_reversed = motor_is_reversed,
    .is_initialized = motor_is_initialized, .set_speed = set_motor_speed,
    .stop = stop_motor, .brake = brake_motor
};
static const hw_motor_interface_t *current_motor_interface = &default_motor_interface;
const hw_motor_interface_t *get_motor_interface(void) { return current_motor_interface; }
void hw_motor_set_interface(const hw_motor_interface_t *interface) { current_motor_interface = interface; }
void hw_motor_init(void) {
    current_motor_interface = &default_motor_interface;
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    hw_drv8873_prepare();
#endif
}

static bool init_channel(TIM_HandleTypeDef *timer, const pwm_channel_info_t *channel) {
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    // The complementary pin must have the same physical duty as the regular pin.
    oc.OCNPolarity = channel->complementary ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &oc, channel->timerChannel) != HAL_OK) return false;
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    __HAL_TIM_DISABLE_OCxPRELOAD(timer, channel->timerChannel);
#endif
    rcc_gpio_clk_enable(channel->port);
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = channel->pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = get_alternate_function_mapping(timer->Instance);
    HAL_GPIO_Init(channel->port, &gpio);
    return HAL_OK == (channel->complementary ?
        HAL_TIMEx_PWMN_Start(timer, channel->timerChannel) : HAL_TIM_PWM_Start(timer, channel->timerChannel));
}

static bool init_motor_timer(const motor_info_t *info) {
    TIM_HandleTypeDef *timer = get_timer_handeler(info->timer);
    if (HAL_TIM_Base_GetState(timer) != HAL_TIM_STATE_READY) {
        rcc_tim_clk_enable(info->timer);
        timer->Instance = info->timer;
        // TIM1/TIM8 are on APB2; the other configured motor timers are on APB1.
        // STM32F405 timers run at 2x PCLK when the corresponding APB is divided.
        RCC_ClkInitTypeDef clocks = {0};
        uint32_t latency;
        HAL_RCC_GetClockConfig(&clocks, &latency);
        bool apb2 = info->timer == TIM1 || info->timer == TIM8;
        uint32_t timer_clock = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
        uint32_t divider = apb2 ? clocks.APB2CLKDivider : clocks.APB1CLKDivider;
        if (divider != RCC_HCLK_DIV1) timer_clock *= 2U;
        const uint32_t counter_clock = MOTOR_PWM_FREQUENCY_HZ * MOTOR_MAX_SPEED;
        if (timer_clock < counter_clock || timer_clock % counter_clock != 0U ||
            timer_clock / counter_clock > 65536U) {
            Error_Handler();
            return false;
        }
        // At the standard clocks: PSC=0 for 84 MHz, PSC=1 for 168 MHz.
        // Both produce 100 kHz with the same 840-step duty scale.
        timer->Init.Prescaler = timer_clock / counter_clock - 1U;
        timer->Init.CounterMode = TIM_COUNTERMODE_UP;
        timer->Init.Period = MOTOR_MAX_SPEED - 1;
        timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        TIM_ClockConfigTypeDef clock = {0};
        clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        TIM_MasterConfigTypeDef master = {0};
        master.MasterOutputTrigger = TIM_TRGO_RESET;
        master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIM_Base_Init(timer) != HAL_OK ||
            HAL_TIM_ConfigClockSource(timer, &clock) != HAL_OK ||
            HAL_TIM_PWM_Init(timer) != HAL_OK ||
            HAL_TIMEx_MasterConfigSynchronization(timer, &master) != HAL_OK) {
            Error_Handler();
            return false;
        }
    }
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    bool success = init_channel(timer, &info->enable);
#else
    bool success = init_channel(timer, &info->pwmChannel1) && init_channel(timer, &info->pwmChannel2);
#endif
    if (!success) Error_Handler();
    return success;
}

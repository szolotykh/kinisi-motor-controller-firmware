#include <assert.h>
#include <stdio.h>
#include "hw_config.h"
#include "hw_gpio.h"

GPIO_TypeDef test_ports[5], unused_ports[2];
TIM_TypeDef test_timers[12];
static unsigned initialized;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    if (state) port->ODR |= pin; else port->ODR &= ~pin;
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin) { port->ODR ^= pin; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config) {
    const status_led_info_t *led = &status_led_info[initialized++];
    assert(port == led->gpio.port && config->Pin == led->gpio.pin);
    assert(config->Mode == GPIO_MODE_OUTPUT_PP);
    // Output latch must be off before enabling output mode, including active-low LEDs.
    assert(((port->ODR & config->Pin) != 0) == led->activeLow);
}
static bool on(unsigned i) {
    const status_led_info_t *led = &status_led_info[i];
    return ((led->gpio.port->ODR & led->gpio.pin) != 0) != led->activeLow;
}
int main(void) {
    const gpio_interface_t *gpio = get_gpio_interface();
    gpio->init_status_led();
    assert(initialized == NUMBER_STATUS_LEDS);
    for (unsigned i = 0; i < NUMBER_STATUS_LEDS; ++i) assert(!on(i));
    gpio->set_status_led(1); assert(on(0));
    gpio->toggle_status_led(); assert(!on(0));
    gpio->toggle_status_led(); assert(on(0));
    gpio->set_status_led(0); assert(!on(0));
    for (unsigned i = 1; i < NUMBER_STATUS_LEDS; ++i) assert(!on(i));
    puts("Status LED initialization, polarity and primary-LED commands passed");
    return 0;
}

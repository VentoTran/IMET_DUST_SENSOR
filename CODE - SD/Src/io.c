/*
 * io.c
 *
 *  Created on: Dec 3, 2021
 *      Author: manht
 */

#include "io.h"

static gpio_p gpio_proc = {
		gpio_get_state,
		gpio_set_state
};

static void(*io_events)(uint32_t pin) = NULL;

void initIO() {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    (void)io_events;
}

void deinitIO() {
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOA_CLK_DISABLE();
}

void registerIOIntr(void(*callback)(uint32_t pin)) {
    io_events = callback;
}

gpio_t* gpio_create(void* port, uint32_t pin, uint32_t mode, uint32_t speed, uint32_t pull) {
    return NULL;
}

gpio_t* gpio_create_static(void* port, uint32_t pin, uint32_t mode, uint32_t speed, uint32_t pull) {
    static gpio_t gpio = {0};
    gpio.port = port;
    gpio.pin = pin;
    gpio.mode = mode;
    gpio.speed = speed;
    gpio.pull = pull;
    gpio.proc = &gpio_proc;
    return &gpio;
}

gpio_t* gpio_create_base(gpio_t* init, void* port, uint32_t pin, uint32_t mode, uint32_t speed, uint32_t pull) {
    if (init != NULL) {
		init->pin = pin;
		init->port = port;
        init->mode = mode;
        init->speed = speed;
		init->proc = &gpio_proc;
	}
	return init;
}

bool gpio_get_state(gpio_t* gpio) {
    if (gpio->port != 0) {
        return ((((GPIO_TypeDef*)(gpio->port))->IDR & gpio->pin) != (uint32_t)GPIO_PIN_RESET) ? true : false;
    }
    return 0;
}

void gpio_set_state(gpio_t* gpio, bool state) {
    if (gpio->port != 0) {
        if (state)
            ((GPIO_TypeDef*)(gpio->port))->BSRR = (uint32_t)gpio->pin;
        else
            ((GPIO_TypeDef*)(gpio->port))->BRR = (uint32_t)gpio->pin;
    }

}

void gpio_config(gpio_t* gpio){
	GPIO_InitTypeDef gpio_struct_init;
	gpio_struct_init.Mode = gpio->mode;
	gpio_struct_init.Pin = gpio->pin;
	gpio_struct_init.Pull = gpio->pull;
	gpio_struct_init.Speed = gpio->speed;
	HAL_GPIO_Init((GPIO_TypeDef*)(gpio->port), &gpio_struct_init);
}


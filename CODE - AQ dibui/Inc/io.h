/*
 * io.h
 *
 *  Created on: Dec 3, 2021
 *      Author: manht
 */

#ifndef USER_PORTS_IO_IO_H_
#define USER_PORTS_IO_IO_H_

#include <stdbool.h>
#include "main.h"
typedef struct gpio_t gpio_t;

typedef struct gpio_p {
	bool(*get_state)(gpio_t*);
	void(*set_state)(gpio_t*, bool);
} gpio_p;

struct gpio_t {
	const gpio_p* proc;
	GPIO_TypeDef* port;
	uint32_t pin;
	uint32_t mode;
	uint32_t speed;
	uint32_t pull;
};
void initIO(void);
void deinitIO(void);
void registerIOIntr(void(*callback)(uint32_t pin));

gpio_t* gpio_create(void* port, uint32_t pin, uint32_t mode, uint32_t speed, uint32_t pull);
gpio_t* gpio_create_static(void* port, uint32_t pin, uint32_t mode, uint32_t speed, uint32_t pull);
gpio_t* gpio_create_base(gpio_t* init, void* port, uint32_t pin, uint32_t mode, uint32_t speed, uint32_t pull);

bool gpio_get_state(gpio_t* gpio);
void gpio_set_state(gpio_t* gpio, bool state);
void gpio_config(gpio_t* gpio);

uint32_t gpio_get_port_state(gpio_t* gpio);

#endif /* USER_PORTS_IO_IO_H_ */

/*
 * spi.h
 *
 *  Created on: Dec 14, 2021
 *      Author: manht
 */

#ifndef USER_PORTS_SPI_SPI_H_
#define USER_PORTS_SPI_SPI_H_
#include "main.h"

uint8_t spi_init(SPI_HandleTypeDef *hspi, SPI_TypeDef *SPIx);
uint8_t spi_tx_byte(SPI_HandleTypeDef *hspi, uint8_t data);
#endif /* USER_PORTS_SPI_SPI_H_ */

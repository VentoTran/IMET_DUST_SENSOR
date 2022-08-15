/*
 * spi.c
 *
 *  Created on: Dec 14, 2021
 *      Author: manht
 */

#include "spi.h"
uint8_t spi_init(SPI_HandleTypeDef *hspi, SPI_TypeDef *SPIx){
	hspi->Instance = SPIx;
	hspi->Init.Mode = SPI_MODE_MASTER;
	hspi->Init.Direction = SPI_DIRECTION_2LINES;
	hspi->Init.DataSize = SPI_DATASIZE_8BIT;
	hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi->Init.NSS = SPI_NSS_SOFT;
	hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi->Init.TIMode = SPI_TIMODE_DISABLE;
	hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi->Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(hspi) != HAL_OK) return 0;
	__HAL_SPI_ENABLE(hspi);
	return 1;
}
uint8_t spi_tx_byte(SPI_HandleTypeDef *hspi, uint8_t data){
	while(__HAL_SPI_GET_FLAG(hspi,SPI_FLAG_TXE) == RESET );
	hspi->Instance->DR = data;
	while(__HAL_SPI_GET_FLAG(hspi,SPI_FLAG_RXNE) == RESET );
	return hspi->Instance->DR;
}

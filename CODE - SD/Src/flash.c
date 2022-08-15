#include <flash.h>
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
#include "main.h"

#define FLASH_TYPEERASE_PAGES    0x00U //xóa theo page
#define FLASH_TYPEERASE_MASSERASE 0x02U //xóa toàn b? chip
FLASH_EraseInitTypeDef EraseInitStruct; // struct c?u hình xóa d? li?u
uint16_t lengthPage;
uint32_t PAGEError; // bi?n ch?a d?a ch? l?i

void Flash_Lock()
{
	HAL_FLASH_Lock();
}

void Flash_Unlock()
{
	HAL_FLASH_Unlock();
}

void Flash_Erase(uint32_t addr)
{
 EraseInitStruct.Banks = 1;
 EraseInitStruct.TypeErase  = FLASH_TYPEERASE_PAGES;
 EraseInitStruct.PageAddress = addr; // d?a ch? page 64
 EraseInitStruct.NbPages    = 1;
 HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
}


void Flash_Write_array(uint32_t addr, uint8_t* data)
{
	Flash_Unlock();
	Flash_Erase(addr);
	int i;
  //FLASH->CR |= FLASH_CR_PG;
	lengthPage = 2048;
  for(i=0; i<lengthPage; i+=4)
  {
		uint32_t temp_data = (uint32_t)(data[i] << 24) | (uint32_t)(data[i+1] << 16) | (uint32_t)(data[i+2] << 8) | data[i+3];
		HAL_FLASH_Program(0x02u, addr + i, temp_data);
  }
	Flash_Lock();
}
void Flash_Write_32(uint32_t addr, uint32_t data)
{
	Flash_Unlock();
	Flash_Erase(addr);
  //FLASH->CR |= FLASH_CR_PG;
	HAL_FLASH_Program(0x02u, addr, data);
	Flash_Lock();
}










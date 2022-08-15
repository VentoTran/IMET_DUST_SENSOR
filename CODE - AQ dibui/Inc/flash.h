#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "string.h"

void deleteBuffer(char* data);
void 	Flash_Lock(void);
void 	Flash_Unlock(void);
void 	Flash_Erase(uint32_t addr);
void Flash_Write_array(uint32_t addr, uint8_t* data);
void Flash_Write_32(uint32_t addr, uint32_t data);

#include "Flash.h"
#include "mbed.h"

bool Flash::write(uint32_t addr, uint8_t* dat, uint16_t size)
{
  int i;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;
  HAL_StatusTypeDef r;
 
  r = HAL_FLASH_Unlock();
  if( r != HAL_OK ){
    return false;
  }
  EraseInitStruct.TypeErase = TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = addr;
  EraseInitStruct.NbPages = 1;
  r = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
  if ( r != HAL_OK )
  {
    return false;
  }
  for(i = 0; i < size; i += 2){
    r = HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, addr+i, *(uint16_t*)(dat+i));
    if( r != HAL_OK ){
      return false;
    }
  }
  r = HAL_FLASH_Lock();
  if( r != HAL_OK ){
    return false;
  }
  return true;
}

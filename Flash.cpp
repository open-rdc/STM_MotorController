#include "Flash.h"
#include "mbed.h"

typedef struct {
    uint32_t base_address;
    uint32_t sector_size;
    uint32_t sector_count;
} flash_layout_t;

#define FLASH_FLAG_PGSERR FLASH_FLAG_ERSERR

static const flash_layout_t flash_layout[] = {
    { 0x08000000, 0x08000, 4 },
    { 0x08020000, 0x20000, 1 },
    { 0x08040000, 0x40000, 3 },
};

uint32_t flash_get_sector_info(uint32_t addr, uint32_t *start_addr, uint32_t *size) {
    if (addr >= flash_layout[0].base_address) {
        uint32_t sector_index = 0;
        for (int i = 0; i < sizeof(flash_layout)/sizeof(flash_layout_t); ++i) {
            for (int j = 0; j < flash_layout[i].sector_count; ++j) {
                uint32_t sector_start_next = flash_layout[i].base_address
                    + (j + 1) * flash_layout[i].sector_size;
                if (addr < sector_start_next) {
                    if (start_addr != NULL) {
                        *start_addr = flash_layout[i].base_address
                            + j * flash_layout[i].sector_size;
                    }
                    if (size != NULL) {
                        *size = flash_layout[i].sector_size;
                    }
                    return sector_index;
                }
                ++sector_index;
            }
        }
    }
    return 0;
}

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
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);  
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = flash_get_sector_info(addr, NULL, NULL);
  EraseInitStruct.NbSectors = flash_get_sector_info(addr + size - 1, NULL, NULL) - EraseInitStruct.Sector + 1;
  r = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
  if ( r != HAL_OK )
  {
    return false;
  }
  for(i = 0; i < size; i += 2){
    r = HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, addr+i, *(uint16_t*)(dat+i));
    if( r != HAL_OK ){
      HAL_FLASH_Lock();
      return false;
    }
  }
  r = HAL_FLASH_Lock();
  if( r != HAL_OK ){
    return false;
  }
  return true;
}

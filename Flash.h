// Chiba Institute of Technology

#ifndef FLASH_H
#define FLASH_H

/** Class to communicate with RS485
 *
 * Exmaple
 * @code
 *
 * #include "mbed.h" 
 * #include "Flash.h" 
 * 
 * DigitalOut led1(LED1);
 * Flash flash;
 * 
 * int main() {
 *   uint8_t dat[2] = {'a', 'b'};
 *   uint8_t *p = (uint8_t *)0x08010000;
 *   flash.write((uint32_t)p, dat, 2); 
 *   if (*p == 'a') led1 = 1;
 * }
 */

#include "mbed.h"

class Flash
{
public:
  Flash(){};

  bool write(uint32_t addr, uint8_t* dat, uint16_t size);
};

#endif


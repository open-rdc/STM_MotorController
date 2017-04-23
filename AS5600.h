// Chiba Institute of Technology

#ifndef ASS5600_H
#define ASS5600_H

#include "mbed.h"

/** Class to measure angle using an absolute encoder AS5600
 *
 * Exmaple
 * @code
 *
 * // Display Angle (Button 1: Reset Angle)
 * #include "mbed.h" 
 * #include "AS5600.h" 
 *
 * BusOut led(LED1, LED2, LED3, LED4);
 * BusIn sw(SW1, SW2);
 * AS5600 as5600(I2C_SDA, I2C_SCL);
 * 
 * int main() {
 *   led = 0;
 *   sw.mode(PullUp);
 *   while(1) {
 *     float angle = as5600;
 *     printf("%f\r\n", angle);
 *     if (sw[0] == 0) as5600 = 0;
 *     wait(0.5);
 *   }
 * }
 */

class AS5600
{
public:
  AS5600(PinName i2c_sda, PinName i2c_scl);

  void updateAngle();

  float getAngleRad();

  float getAngleDeg();

  int getError();

  void resetError();

void write(float value);

  float read();

  AS5600& operator= (double value) {
    write(value);
    return *this;
  }

  operator float(){
    return read();
  }

private:
  I2C i2c;
  float angle;
  float angle0;
  int error;
};

#endif

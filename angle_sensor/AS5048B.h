// Chiba Institute of Technology

#ifndef ASS5048B_H
#define ASS5048B_H

#include "mbed.h"
#include "AngleSensor.h"

/** Class to measure angle using an absolute encoder AS5048B
 *
 * Exmaple
 * @code
 *
 * // Display Angle (Button 1: Reset Angle)
 * #include "mbed.h" 
 * #include "AS5048B.h" 
 *
 * BusOut led(LED1, LED2, LED3, LED4);
 * BusIn sw(SW1, SW2);
 * AS5048B as5048b(I2C_SDA, I2C_SCL);
 * 
 * int main() {
 *   led = 0;
 *   sw.mode(PullUp);
 *   while(1) {
 *     float angle = as5048b;
 *     printf("%f\r\n", angle);
 *     if (sw[0] == 0) as5048b = 0;
 *     wait(0.5);
 *   }
 * }
 */

class AS5048B : public AngleSensor
{
public:
  AS5048B(PinName i2c_sda, PinName i2c_scl);

  virtual void updateAngle();

  virtual float getAngleRad();

  virtual float getAngleDeg();

  virtual int getError();

  virtual void resetError();

  virtual void write(float value);

  virtual float read();

  AS5048B& operator= (double value) {
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
  bool is_first;
};

#endif

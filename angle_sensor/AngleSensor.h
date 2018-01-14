// Chiba Institute of Technology

#ifndef ANGLE_SENSOR_H
#define ANGLE_SENSOR_H

#include "mbed.h"

/** Class to measure angle using an absolute encoder
 *
 * Exmaple
 * @code
 *
 * // Display Angle (Button 1: Reset Angle)
 * #include "mbed.h" 
 * #include "AngleSensor.h"
 * #include "AS5600.h" 
 *
 * AS5048B as5048b(I2C_SDA, I2C_SCL);
 * 
 * int main() {
 *   while(1) {
 *			AngleSensor *as = &as5048b;
 *     float angle = as->getAngleRad();
 *     printf("%f\r\n", angle);
 *     wait(0.5);
 *   }
 * }
 */

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

class AngleSensor
{
public:
  AngleSensor() {};
		
	virtual ~AngleSensor() {};

  virtual void updateAngle() {};

  virtual float getAngleRad() { return 0.0f; };

  virtual float getAngleDeg() { return 0.0f; };

  virtual int getError() { return 0; };

  virtual void resetError() {};

  virtual void write(float value) {};

  virtual float read() { return 0.0f; };

  AngleSensor& operator= (float value) {
    write(value);
    return *this;
  }

  operator float(){
    return read();
  }
};

#endif

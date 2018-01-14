#include "mbed.h"
#include "AS5048B.h"

#define SLAVE_ADRESS  0x40

AS5048B::AS5048B(PinName i2c_sda, PinName i2c_scl):
  i2c(i2c_sda, i2c_scl), angle0(0), error(0)
{
}

void AS5048B::updateAngle()
{
  char cmd[1];
  char out[2];
  cmd[0] = 0xFF;
  error |= i2c.write(SLAVE_ADRESS << 1, cmd, 1);
  error |= i2c.read(SLAVE_ADRESS << 1, out, 2);
	int raw_data = ((unsigned short)out[0] << 6) + (out[1] & 0x3f);
  if (error == 0) angle = raw_data * 0.021973997f * M_PI / 180.0f - angle0;
  while (angle > M_PI) angle -= 2.0f * M_PI;
  while (angle < -M_PI) angle += 2.0f * M_PI;
  angle = -angle;
}

float AS5048B::getAngleRad()
{
  return angle;
}

float AS5048B::getAngleDeg()
{
  return angle / M_PI * 180.0f;
}

int AS5048B::getError()
{
  return error;
}

void AS5048B::resetError()
{
  error = 0;
}

void AS5048B::write(float value)
{
  updateAngle();
  angle0 = (angle + angle0) - value;
}

float AS5048B::read()
{  
  updateAngle();
  return angle;
}

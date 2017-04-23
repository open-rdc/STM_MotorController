#include "mbed.h"
#include "AS5600.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

#define SLAVE_ADRESS  0x36

AS5600::AS5600(PinName i2c_sda, PinName i2c_scl):
  i2c(i2c_sda, i2c_scl), angle0(0), error(0)
{
}

void AS5600::updateAngle()
{
  char cmd[1];
  char out[2];
  cmd[0] = 0x0E;
  error |= i2c.write(SLAVE_ADRESS << 1, cmd, 1);
  error |= i2c.read(SLAVE_ADRESS << 1, out, 2);
  if (error == 0) angle = ((out[0] << 8) + out[1]) * 0.087912087f * M_PI / 180.0f - angle0;
  while (angle > M_PI) angle -= 2.0f * M_PI;
  while (angle < -M_PI) angle += 2.0f * M_PI;
}

float AS5600::getAngleRad()
{
  return angle;
}

float AS5600::getAngleDeg()
{
  return (float)(angle / M_PI * 180.0f);
}

int AS5600::getError()
{
  return error;
}

void AS5600::resetError()
{
  error = 0;
}

void AS5600::write(float value)
{
  updateAngle();
  angle0 = (angle + angle0) - value;
}

float AS5600::read()
{  
  updateAngle();
  return angle;
}

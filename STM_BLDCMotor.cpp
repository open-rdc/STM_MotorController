#include "mbed.h"
#include "STM_BLDCMotor.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define HOLE_STATE0   0x05  // 101  ( 0deg - 60deg)
#define HOLE_STATE1   0x04  // 100  ( 60deg - 120deg)
#define HOLE_STATE2   0x06  // 110  ( 120deg - 180deg)
#define HOLE_STATE3   0x02  // 010  ( 180deg - 240deg)
#define HOLE_STATE4   0x03  // 011  ( 240deg - 300deg)
#define HOLE_STATE5   0x01  // 001  ( 300deg - 360deg)

#define MIN_PWM 0.10
#define PWM_FREQUENCY 20000.0
#define SAMPLING_TIME 0.0001

int STM_BLDCMotor::switching_table[6] [3] = {
    { 0, -1, 1 }, // STATE1
    { 1, -1, 0 }, // STATE2
    { 1, 0, -1 }, // STATE3
    { 0, 1, -1 }, // STATE4
    { -1, 1, 0 }, // STATE5
    { -1, 0, 1 }, // STATE6
};

STM_BLDCMotor::STM_BLDCMotor() :
  uh_(MOTOR_UH), ul_(MOTOR_UL), vh_(MOTOR_VH), vl_(MOTOR_VL), wh_(MOTOR_WH), wl_(MOTOR_WL),
  st_(), max_ratio_(0.5), enable_(false), hole_state_no(0), hole_state0_angle_(3.068711), 
  angle_sensor_(I2C_SDA, I2C_SCL)  
{
  st_.attach(callback(this, &STM_BLDCMotor::status_changed), SAMPLING_TIME);
  setPwmPeriod(1.0 / PWM_FREQUENCY);
  ul_ = uh_ = vl_ = vh_ = wl_ = wh_ = 0;
  
  this->write(0);
}

void STM_BLDCMotor::servoOn(void)
{
  enable_ = true;
}

void STM_BLDCMotor::servoOff(void)
{
  enable_ = false;
}

void STM_BLDCMotor::setMaxDutyRatio(float max_ratio)
{
  max_ratio_ = max(min(max_ratio, 1.0f), 0.0f);
}

void STM_BLDCMotor::setPwmPeriod(double seconds)
{
  period_sec_ = seconds;
  uh_.period(period_sec_);
  vh_.period(period_sec_);
  wh_.period(period_sec_);
}

void STM_BLDCMotor::write(double value)
{
  value_ = max(min(value, max_ratio_), -max_ratio_);
}

float STM_BLDCMotor::read()
{
  return value_;
}

int STM_BLDCMotor::getHoleState()
{
  const float angle_width = 2 * M_PI / 42;
  float angle = hole_state0_angle_ + angle_width/2 + 2 * M_PI - angle_sensor_;
  int min_no = (int)(angle / angle_width);
  hole_state_no = min_no % 6;

  return hole_state_no;
}

int STM_BLDCMotor::getState()
{
  return hole_state_no;
}

void STM_BLDCMotor::status_changed(void)
{
  hole_state_no = 0;
  int dir = (value_ > 0.0) ? 1 : (value_ < 0.0) ? -2 : 0;
  
  getHoleState();
  int next_state = (hole_state_no + dir + 6) % 6;
  hole_state_no = next_state;

  if (enable_){
    drive(switching_table[next_state][0], switching_table[next_state][1], switching_table[next_state][2]);
  } else {
    drive(0, 0, 0);
  }
	st_.attach(callback(this, &STM_BLDCMotor::status_changed), SAMPLING_TIME);
}

/*!
 * @brief drive for three phase motor
 * @param[in] u switch u line (1:High, 0: NC, -1: Low)
 * @param[in] v switch v line (1:High, 0: NC, -1: Low)
 * @param[in] w switch w line (1:High, 0: NC, -1: Low)
 */
void STM_BLDCMotor::drive(int u, int v, int w)
{
  // prevent through current
  double val = fabs(value_);

  uh_ = (u == 1) ? val : 0.0;
  ul_ = (u == -1) ? 1 : 0;
  vh_ = (v == 1) ? val : 0.0;
  vl_ = (v == -1) ? 1 : 0;
  wh_ = (w == 1) ? val : 0.0;
  wl_ = (w == -1) ? 1 : 0;
}

//#include "RS485.h"
//RS485 rs485(RS485_TX, RS485_RX, RS485_SELECT);

void STM_BLDCMotor::detectHoleState0(float duty_ratio)
{
  st_.detach();
  value_ = duty_ratio;
  for(int i = 5; i >= 0; i --) {
    drive(switching_table[i][0], switching_table[i][1], switching_table[i][2]);
    wait(0.3);
  }
  wait(2.0);
  hole_state0_angle_ = 0;
  for(int i = 0; i < 100; i ++) {
    float angle = angle_sensor_;
    hole_state0_angle_ += angle;
//    rs485.printf("%f\r\n", angle);
    wait(0.001);
  }
  hole_state0_angle_ /= 100;
  value_ = 0.0;
  drive(0, 0, 0);
  st_.attach(callback(this, &STM_BLDCMotor::status_changed), SAMPLING_TIME);
}

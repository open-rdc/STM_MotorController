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

STM_BLDCMotor::STM_BLDCMotor()
  :  uh_(MOTOR_UH), ul_(MOTOR_UL), vh_(MOTOR_VH), vl_(MOTOR_VL), wh_(MOTOR_WH), wl_(MOTOR_WL),
    st_(), max_ratio_(0.5), enable_(false), as5600_(I2C_SDA, I2C_SCL)
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
	static double hole_angle[] = {
		3.068711, 2.917832333, 2.760306, 2.596640667, 2.48361, 2.313296667,
		2.153212333, 2.020234667, 1.857592667, 1.705180333, 1.542538333, 1.437691,
		1.251522333, 1.108827667, 0.964087, 0.808605333, 0.655169667, 0.498154,
    0.381031667, 0.209183667, 0.049610667, -0.083366333, -0.241916667, -0.389215,
		-0.553901667,  -0.657726667, -0.837246667, -0.988124333, -1.129285333, -1.278118,
		-1.439736667, -1.597264, -1.707225667, -1.885723, -2.038135333, -2.169578667,
		-2.327105667, -2.469800667, -2.632953667, -2.744450333, -2.917321333, -3.061039
	};
	float angle = as5600_;
	float min_val = 1.0f;
	int min_no = 0;
	for(int i = 0; i < 42; i ++) {
		float val = fabs(hole_angle[i] - angle);
		if (val < -3.14159*2.0) val =+ 3.14159*2.0;
		if (val > 3.14159*2.0) val =- 3.14159*2.0;
		if (val < min_val){
			min_val = val;
			min_no = i;
		}
	}
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
  int dir = (value_ >= 0.0) ? 1 : -2;
  
  getHoleState();
  int next_state = (hole_state_no + dir + 6) % 6;

  if (enable_){
    drive(switching_table[next_state][0],
            switching_table[next_state][1],
            switching_table[next_state][2]);
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

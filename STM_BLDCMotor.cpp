#include "mbed.h"
#include "STM_BLDCMotor.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define HOLE_STATE0 	0x05	// 101  ( 0deg - 60deg)
#define HOLE_STATE1 	0x04	// 100  ( 60deg - 120deg)
#define HOLE_STATE2 	0x06	// 110  ( 120deg - 180deg)
#define HOLE_STATE3 	0x02	// 010  ( 180deg - 240deg)
#define HOLE_STATE4 	0x03	// 011  ( 240deg - 300deg)
#define HOLE_STATE5 	0x01	// 001  ( 300deg - 360deg)

#define MIN_PWM 0.10
#define PWM_FREQUENCY 20000.0

int STM_BLDCMotor::switching_table[6] [3] = {
		{ 0, -1, 1 }, // STATE1
		{ 1, -1, 0 }, // STATE2
		{ 1, 0, -1 }, // STATE3
		{ 0, 1, -1 }, // STATE4
		{ -1, 1, 0 }, // STATE5
		{ -1, 0, 1 }, // STATE6
};

STM_BLDCMotor::STM_BLDCMotor(PinName Ppwm)
	: hole1_(MOTOR_HOLE1), hole2_(MOTOR_HOLE2), hole3_(MOTOR_HOLE3),
		uh(MOTOR_UH), ul(MOTOR_UL), vh(MOTOR_VH), vl(MOTOR_VL), wh(MOTOR_WH), wl(MOTOR_WL),
		max_ratio_(0.5), enable_(false)
{
	// hole‚ÌŠ„ž‚Ý‚Ì—Dæ‡ˆÊ‚ðã‚°‚é
	hole1_.mode(PullUp);
	hole2_.mode(PullUp);
	hole3_.mode(PullUp);
	
	setPwmPeriod(1.0 / PWM_FREQUENCY);
	ul = uh = vl = vh = wl = wh = 0;
	
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
	max_ratio_ = max(min(max_ratio, 1.0), 0.0);
}

void STM_BLDCMotor::setPwmPeriod(double seconds)
{
  period_sec_ = seconds;
	uh.period(period_sec_);
	ul.period(period_sec_);
	vh.period(period_sec_);
	vl.period(period_sec_);
	wh.period(period_sec_);
	wl.period(period_sec_);
}

void STM_BLDCMotor::write(double value)
{
	value_ = max(min(value, 1.0), -1.0);
//	pwm_ = min(fabs(max_ratio_ * value_), 1.0);
}

float STM_BLDCMotor::read()
{
  return value_;
}

int STM_BLDCMotor::getHoleState()
{
	int h1 = hole1_;
	int h2 = hole2_;
	int h3 = hole3_;
	
	hole_state = (h1 << 2) + (h2 << 1) + h3;

	return hole_state;
}

int STM_BLDCMotor::getState()
{
	return hole_state_no;
}

void STM_BLDCMotor::status_changed(void)
{
	hole_state_no = 0;
	int dir = (value_ >= 0.0) ? 1 : -2;
	
	switch(hole_state){
		case HOLE_STATE0:
			hole_state_no = 0; break;
		case HOLE_STATE1:
			hole_state_no = 1; break;
		case HOLE_STATE2:
			hole_state_no = 2; break;
		case HOLE_STATE3:
			hole_state_no = 3; break;
		case HOLE_STATE4:
			hole_state_no = 4; break;
		case HOLE_STATE5:
			hole_state_no = 5; break;
	}
	int next_state = (hole_state_no + dir + 6) % 6;

	if (enable_){
		drive(switching_table[next_state][0],
						switching_table[next_state][1],
						switching_table[next_state][2]);
	} else {
		drive(0, 0, 0);
	}
}

/*!
 * @brief drive for three phase motor
* @param[in] u switch u line (1:High, 0: NC, -1: Low)
* @param[in] v switch v line (1:High, 0: NC, -1: Low)
* @param[in] w switch w line (1:High, 0: NC, -1: Low)
 */
void STM_BLDCMotor::drive(int u, int v, int w)
{
	// ŠÑ’Ê“d—¬‚Ì–â‘è
	float val = value_;
	uh = (u == 1) ? val : 0.0;
	ul = (u == -1) ? val : 0.0;
	vh = (v == 1) ? val : 0.0;
	vl = (v == -1) ? val : 0.0;
	wh = (w == 1) ? val : 0.0;
	wl = (w == -1) ? val : 0.0;
}

#include "mbed.h"
#include "MakisumiACMotor.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define HOLE_STATE0 	0x05	// 101  ( 0deg - 60deg)
#define HOLE_STATE1 	0x04	// 100  ( 60deg - 120deg)
#define HOLE_STATE2 	0x06	// 110  ( 120deg - 180deg)
#define HOLE_STATE3 	0x02	// 010  ( 180deg - 240deg)
#define HOLE_STATE4 	0x03	// 011  ( 240deg - 300deg)
#define HOLE_STATE5 	0x01	// 001  ( 300deg - 360deg)

#define MIN_PWM 0.10
#define OFFEST_PWM 0.05

int MakisumiACMotor::switching_table[6] [3] = {
		{ 0, -1, 1 }, // STATE1
		{ 1, -1, 0 }, // STATE2
		{ 1, 0, -1 }, // STATE3
		{ 0, 1, -1 }, // STATE4
		{ -1, 1, 0 }, // STATE5
		{ -1, 0, 1 }, // STATE6
};

DigitalOut uh(MOTOR_UH);
DigitalOut ul(MOTOR_UL);
DigitalOut vh(MOTOR_VH);
DigitalOut vl(MOTOR_VL);
DigitalOut wh(MOTOR_WH);
DigitalOut wl(MOTOR_WL);

MakisumiACMotor::MakisumiACMotor(PinName Ppwm)
		: pwm_int_(Ppwm), pwm_(Ppwm), 
		hole1_(MOTOR_HOLE1), hole2_(MOTOR_HOLE2), hole3_(MOTOR_HOLE3),
		max_ratio_(0.5), enable_(false), underChanging(false)
{
	LPC_IOCON -> SWDIO_PIO1_3 |= 0x01; 
	for(int i = 0; i < 32; i ++)
		NVIC_SetPriority((IRQn_Type)i, 10);
	NVIC_SetPriority(TIMER_16_0_IRQn, 1);
	NVIC_SetPriority(TIMER_16_1_IRQn, 1);
	NVIC_SetPriority(UART_IRQn, 2);
	
	hole1_.mode(PullUp);
	hole2_.mode(PullUp);
	hole3_.mode(PullUp);

	pwm_int_.rise(this, &MakisumiACMotor::pwmRise);
	pwm_int_.fall(this, &MakisumiACMotor::pwmFall);
	pwm_.period(0.0002);		// 1kHz
	pwm_ = 0.0;
	
	this->write(0);
}

void MakisumiACMotor::servoOn(void)
{
	enable_ = true;
}

void MakisumiACMotor::servoOff(void)
{
	enable_ = false;
}

void MakisumiACMotor::setMaxDutyRatio(float max_ratio)
{
	max_ratio_ = max(min(max_ratio, 1.0), 0.0);
}

void MakisumiACMotor::setPwmPeriod(double seconds)
{
	period_sec_ = seconds;
	pwm_.period(seconds);
}

void MakisumiACMotor::write(double value)
{
	value_ = max(min(value, 1.0), -1.0);
	pwm_ = min(fabs(max_ratio_ * value_) + OFFEST_PWM, 1.0);
}

float MakisumiACMotor::read()
{
  return value_;
}

int MakisumiACMotor::getHoleState()
{
	int h1 = (LPC_GPIO0->DATA >> 3) & 1;	// P0_3
	int h2 = (LPC_GPIO1->DATA >> 0) & 1;	// P1_0
	int h3 = (LPC_GPIO1->DATA >> 1) & 1;	// P1_1
	
	hole_state = (h1 << 2) + (h2 << 1) + h3;

	return hole_state;
}

int MakisumiACMotor::getState()
{
	return hole_state_no;
}

void MakisumiACMotor::status_changed(void)
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

	if (enable_ && pwm_ >= MIN_PWM){
		drive(switching_table[next_state][0],
						switching_table[next_state][1],
						switching_table[next_state][2]);
	} else {
		drive(-1, -1, -1);
	}
}

/*!
 * @brief drive for three phase motor
* @param[in] u switch u line (1:High, 0: NC, -1: Low)
* @param[in] v switch v line (1:High, 0: NC, -1: Low)
* @param[in] w switch w line (1:High, 0: NC, -1: Low)
 */
void MakisumiACMotor::drive(int u, int v, int w)
{
	underChanging = true;

	on_swtiching_ptn[UH] = (u == 1) ? 1 : 0;
	on_swtiching_ptn[UL] = (u == -1) ? 1 : 0;
	on_swtiching_ptn[VH] = (v == 1) ? 1 : 0;
	on_swtiching_ptn[VL] = (v == -1) ? 1 : 0;
	on_swtiching_ptn[WH] = (w == 1) ? 1 : 0;
	on_swtiching_ptn[WL] = (w == -1) ? 1 : 0;

	off_swtiching_ptn[UH] = 0;
	off_swtiching_ptn[UL] = (u == -1) ? 1 : 0;
	off_swtiching_ptn[VH] = 0;
	off_swtiching_ptn[VL] = (v == -1) ? 1 : 0;
	off_swtiching_ptn[WH] = 0;
	off_swtiching_ptn[WL] = (w == -1) ? 1 : 0;

	underChanging = false;
}

void MakisumiACMotor::pwmRise(void)
{
	if (!underChanging)
	{
		if (on_swtiching_ptn[UH]){
			LPC_GPIO3->DATA &= ~(1<<0);	//UL
			LPC_GPIO1->DATA |= (1<<2);	//UH
		}
		else	LPC_GPIO1->DATA &= ~(1<<2);	//UH
		if (on_swtiching_ptn[VH]){
			LPC_GPIO1->DATA &= ~(1<<10);	//VL
			LPC_GPIO1->DATA |= (1<<11);	//VH
		}
		else	LPC_GPIO1->DATA &= ~(1<<11);	//VH
		if (on_swtiching_ptn[WH]){
			LPC_GPIO1->DATA &= ~(1<<3);	//WL
			LPC_GPIO1->DATA |= (1<<4);	//WH
		}
		else	LPC_GPIO1->DATA &= ~(1<<4);	//WH
	}

	if (on_swtiching_ptn[UL])	LPC_GPIO3->DATA |= (1<<0);	//UL
	else	LPC_GPIO3->DATA &= ~(1<<0);	//UL
	if (on_swtiching_ptn[VL])	LPC_GPIO1->DATA |= (1<<10);	//VL
	else	LPC_GPIO1->DATA &= ~(1<<10);	//VL
	if (on_swtiching_ptn[WL])	LPC_GPIO1->DATA |= (1<<3);	//WL
	else	LPC_GPIO1->DATA &= ~(1<<3);	//WL
}

void MakisumiACMotor::pwmFall(void)
{
	static int mask = ~(1<<2) & ~(1<<11) & ~(1<<4);
	LPC_GPIO1->DATA &= mask;
//	LPC_GPIO1->DATA &= ~(1<<2);	//UH
//	LPC_GPIO1->DATA &= ~(1<<11);	//VH
//	LPC_GPIO1->DATA &= ~(1<<4);	//WH
}

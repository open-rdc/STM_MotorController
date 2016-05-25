#include "mbed.h"
#include "AS5600.h"
#include "RS485.h"
#include "Parser.h"
#include "STM_BLDCMotor.h"

#define MAX_ANGLE 0
#define MIN_ANGLE -45
#define GAIN 10.0
#define BAUDRATE 57600

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

DigitalOut blink_led(LED1);
BusIn sw(SW1, SW2);
STM_BLDCMotor motor;
AS5600 as5600(I2C_SDA, I2C_SCL);
RS485 rs485(RS485_TX, RS485_RX, RS485_SELECT);
Parser commnand_parser(0);
Timer t;

const int MAX_COMMAND_LEN = 100; 
unsigned char command_data[MAX_COMMAND_LEN];
int command_len = 0;
const int LED_COUNT_MAX = 100;

struct RobotStatus{
  float target_angle;
  float gain;
  float max_torque;
  bool is_servo_on;
  bool change_target;
  bool isWakeupMode;
  int led_state;
  int led_count;
} status;

void initialize()
{
  status.target_angle = 0;
  status.gain = GAIN;
  status.max_torque = 1.0;
  status.is_servo_on = false;
  status.led_state = 0;
  status.led_count = 0;
  status.change_target = false;
  status.isWakeupMode = true;
}

void isrRx() {
	while(rs485.readable()){
    command_data[command_len ++] = rs485.getc();
  }
  if (command_len > 0){
    commnand_parser.setCommand(command_data, command_len);
  }
}

int main() {
	int counter = 25;
	initialize();
	blink_led = 0;
	sw.mode(PullUp);
  
	rs485.attach(isrRx, Serial::RxIrq);
	rs485.baud(BAUDRATE);
	
	as5600 = as5600;
  t.reset();
  t.start();
	while(1){
		float angle = as5600;
		if (as5600.getError()) break;
		float val = max(min(status.gain * (angle - status.target_angle),
      status.max_torque), -status.max_torque);
		motor = (double)val;
		if (status.change_target){
			motor.status_changed();
			status.change_target = false;
		}
		status.led_count ++;
		if (status.led_count >= LED_COUNT_MAX){
			status.led_state ^= 1;
			blink_led = status.led_state;
			status.led_count = 0;
		}
	}
	motor = 0;
	motor.status_changed();
	blink_led = 0;
}

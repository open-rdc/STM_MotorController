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

struct RobotStatus{
  float target_angle;
  bool is_servo_on;
  bool change_target;
} status;

void initialize()
{
  status.target_angle = 0;
  status.is_servo_on = false;
  status.change_target = false;
}

void isrRx() {  
	while(rs485.readable()){
    command_data[command_len ++] = rs485.getc();
  }
  if (command_len > 0){
    if (commnand_parser.setCommand(command_data, command_len))
    {
      status.change_target = true;
    }
  }
}

int main() {
	int previous_hole_state = 6;
	float gain = GAIN, max_value = 1.0;
	int counter = 25, led_counter  = 100, led_state = 0;
	bool is_low_gain = true;
	initialize();
	blink_led = 0;
	sw.mode(PullUp);
	rs485.attach(isrRx, Serial::RxIrq);
	rs485.baud(BAUDRATE);
	
	as5600 = as5600;
	while(1){
		float angle = as5600;
//		printf("%f\r\n", angle);
		if (as5600.getError()) break;
		if (is_low_gain){
			if(fabs(angle - status.target_angle) < 0.1){
        motor.setMaxDutyRatio(0.5);
				is_low_gain = false;
			}
		}
		float val = max(min(gain * (angle - status.target_angle), max_value), -max_value);
		motor = (double)val;
		if (status.change_target){
			motor.status_changed();
			status.change_target = false;
		}
		if (sw[0] == 0 && counter == 0){
			status.target_angle += 0.1;
			counter = 25;
		}
		if (sw[1] == 0 && counter == 0){
			status.target_angle -= 0.1;
			counter = 25;
		}
		if (counter > 0) counter --;
		if (led_counter == 0){
			led_state ^= 1;
			blink_led = led_state;
			led_counter = 100;
		}	
		led_counter --;
		t.reset();
		t.start();
		while(t.read() < 0.020f){
			if (motor.getHoleState() != previous_hole_state){
				motor.status_changed();
				previous_hole_state = motor.getHoleState();
//				printf("%d\r\n", previous_hole_state);
			}
		}
	}
	motor = 0;
	motor.status_changed();
	blink_led = 0x02;
	while(1);
}

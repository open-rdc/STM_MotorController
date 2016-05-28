#include "mbed.h"
#include "AS5600.h"
#include "RS485.h"
#include "Parser.h"
#include "STM_BLDCMotor.h"
#include "Flash.h"
#include "b3m.h"

#define GAIN 10.0
#define PUNCH 0.15
#define MARGIN 0.003
#define MAX_ANGLE 30.0
#define MIN_ANGLE -30.0
#define OFFSET_ANGLE 2.3
#define BAUDRATE 115200
#define OFFSET 2.3;
#define FLASH_ADDRESS 0x08010000

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

DigitalOut blink_led(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
BusIn sw(SW1, SW2);
STM_BLDCMotor motor;
AS5600 as5600(I2C_SDA, I2C_SCL);
RS485 rs485(RS485_TX, RS485_RX, RS485_SELECT);
Parser commnand_parser(0);
Flash flash;
Timer t;

const int MAX_COMMAND_LEN = 256; 
unsigned char command_data[MAX_COMMAND_LEN];
int command_len = 0;
const int LED_COUNT_MAX = 500;

struct RobotStatus {
  float target_angle;
  float current_angle;
  float gain;
  float punch;
  float margin;
  float max_torque;
  float max_angle;
  float min_angle;
  float offset_angle;
  bool is_servo_on;
  bool change_target;
  bool isWakeupMode;
  int led_state;
  int led_count;
} status;

Property_T property;

void initialize()
{
  status.target_angle = OFFSET;
  status.current_angle = 0;
  status.gain = GAIN;
  status.punch  = PUNCH;
  status.margin = MARGIN;
  status.max_torque = 1.0;
  status.max_angle = MAX_ANGLE;
  status.min_angle = MIN_ANGLE;
  status.offset_angle = OFFSET_ANGLE;
  status.is_servo_on = false;
  status.led_state = 0;
  status.led_count = 0;
  status.change_target = false;
  status.isWakeupMode = true;
}

int main() {
	initialize();
	blink_led = 0;
	sw.mode(PullUp);
  rs485.baud(BAUDRATE);
	as5600 = as5600;
  t.reset();
  t.start();
  motor.servoOn();
  memcpy(&property, (void *)FLASH_ADDRESS, sizeof(property));
  
  while(1){
    status.led_count ++;
    if (status.led_count > LED_COUNT_MAX){
      status.led_state ^= 1;
      blink_led = status.led_state;
      status.led_count = 0;
    }

    command_len = rs485.read(command_data, MAX_COMMAND_LEN);
    int command = commnand_parser.setCommand(command_data, command_len);
    command_len = 0;
    
    if (command == B3M_CMD_WRITE){
      led2 = led2 ^ 1;
      int address, data;
      if (commnand_parser.getNextCommand(&address, &data) > 0){
        data = max(min(data, status.max_angle * 100), status.min_angle * 100);
        status.target_angle = (float)data * M_PI / 18000.0  + status.offset_angle;
        motor.status_changed();
      }
    } else if (command == B3M_CMD_SAVE){
      flash.write(FLASH_ADDRESS, (uint8_t *)&property, sizeof(property));
    }

		status.current_angle = as5600;
		if (as5600.getError()) break;
    float error = status.current_angle - status.target_angle;
    float pwm = 0;
    if (fabs(error) > status.margin){
      if (error > 0){
        error -= status.margin;
        pwm = status.gain * error + status.punch;
      } else {
        error += status.margin;
        pwm = status.gain * error - status.punch;
      }
    }
		float val = max(min(pwm, status.max_torque), -status.max_torque);
		motor = val;
    
    wait(0.001);
  }
	motor = 0;
	blink_led = 0;
}

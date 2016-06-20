char version[4] = {16, 06, 20, 1};

#include "mbed.h"
#include "AS5600.h"
#include "RS485.h"
#include "Parser.h"
#include "STM_BLDCMotor.h"
#include "Flash.h"
#include "b3m.h"

#define GAIN 10.0
#define GAIN_I 0.0
#define PUNCH 0.20
#define DEAD_BAND_WIDTH 0.2
#define MAX_ANGLE 60.0
#define MIN_ANGLE -60.0
#define OFFSET_ANGLE 2.3
#define BAUDRATE 115200
#define OFFSET 2.3;
#define FLASH_ADDRESS 0x08010000

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define USE_WAKEUP_MODE

extern Property property;

DigitalOut blink_led(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
BusIn sw(SW1, SW2);
STM_BLDCMotor motor;
AS5600 as5600(I2C_SDA, I2C_SCL);
RS485 rs485(RS485_TX, RS485_RX, RS485_SELECT);
Parser commnand_parser;
Flash flash;
Timer t;

const int MAX_COMMAND_LEN = 256; 
unsigned char command_data[MAX_COMMAND_LEN];
int command_len = 0;
const int LED_COUNT_MAX = 500;
unsigned char send_buf[256];

struct RobotStatus {
  float target_angle;
  bool is_servo_on;
  bool change_target;
  bool isWakeupMode;
  int led_state;
  int led_count;
  float err_i;
} status;

float deg100_2rad(float deg){
  return deg * M_PI / 18000.0;
}

float deg2rad(float deg){
  return deg * M_PI / 180.0;
}

float rad2deg100(float rad){
  return rad * 18000.0 / M_PI;
}

void initialize()
{
  status.target_angle = as5600;
  status.is_servo_on = false;
  status.led_state = 0;
  status.led_count = 0;
  status.change_target = false;
  status.isWakeupMode = false;
  status.err_i = 0.0;
  
  memset((void *)&property, 0, sizeof(property));
  property.ID = 0;
  property.Baudrate = 115200;
  property.PositionMinLimit = MIN_ANGLE * 100;
  property.PositionMaxLimit = MAX_ANGLE * 100;
  property.PositionCenterOffset = rad2deg100(status.target_angle);
  property.TorqueLimit = 100;
  property.DeadBandWidth = DEAD_BAND_WIDTH * 100;
  property.Kp0 = GAIN * 100;
  property.Ki0 = GAIN_I * 100;
  property.StaticFriction0 = PUNCH *100;
  property.FwVersion = (version[0] << 24) + (version[1] << 16) + (version[2] << 8) + version[3];
}

int main() {
	initialize();
  bool is_status_changed = false;
	blink_led = 0;
	sw.mode(PullUp);
  rs485.baud(BAUDRATE);
	as5600 = as5600;
  t.reset();
  motor.servoOn();
  memcpy((void *)&property, (void *)FLASH_ADDRESS, sizeof(property));
  property.FwVersion = (version[0] << 24) + (version[1] << 16) + (version[2] << 8) + version[3];
  
  while(1){
    status.led_count ++;
    if (status.led_count > LED_COUNT_MAX){
      status.led_state ^= 1;
      blink_led = status.led_state;
      status.led_count = 0;
    } 
#ifdef USE_WAKEUP_MODE
    status.isWakeupMode = (t.read() < 5) ? true : false;
#endif
    command_len = rs485.read(command_data, MAX_COMMAND_LEN);
    int command = commnand_parser.setCommand(command_data, command_len);
    command_len = 0;
    
    if (command == B3M_CMD_WRITE){
      led2 = led2 ^ 1;
    } else if (command == B3M_CMD_SAVE){
      flash.write(FLASH_ADDRESS, (uint8_t *)&property, sizeof(property));
    } else if (command == B3M_CMD_LOAD){
      memcpy((void *)&property, (void *)FLASH_ADDRESS, sizeof(property));
    } else if (command == B3M_CMD_RESET){      
      initialize();
      led2 = led3 = led4 = 1;
      wait(1);
      led2 = led3 = led4 = 0;
    }
    
    int address, data;
    int com_num = commnand_parser.getNextCommand(&address, &data);
    if (com_num > 0){
      switch(address){
        case B3M_SYSTEM_ID:
          property.ID = data;
          break;
        case B3M_SYSTEM_POSITION_MIN:
          property.PositionMinLimit = data;
          break;
        case B3M_SYSTEM_POSITION_MAX:
          property.PositionMaxLimit = data;
          break;
        case B3M_SYSTEM_POSITION_CENTER:
          property.PositionCenterOffset = data;
          break;
        case B3M_SYSTEM_DEADBAND_WIDTH:
          property.DeadBandWidth = data;
          break;
        case B3M_SYSTEM_TORQUE_LIMIT:
          property.TorqueLimit = data;
          break;
        case B3M_SERVO_DESIRED_POSITION:
          data = max(min(data, property.PositionMaxLimit), property.PositionMinLimit);
          status.target_angle = deg100_2rad(data)  + deg100_2rad(property.PositionCenterOffset);
          is_status_changed = true;
          break;
        case B3M_CONTROL_KP0:
          property.Kp0 = data;
          break;
        case B3M_CONTROL_KI0:
          property.Ki0 = data;
          status.err_i = 0;
          break;
        case B3M_CONTROL_STATIC_FRICTION0:
          property.StaticFriction0 = data;
          break;
        case B3M_SERVO_SERVO_MODE:
          t.reset();
          status.is_servo_on = (data == 0) ? true : false;
          led3 = (status.is_servo_on) ? 1 : 0;
          status.target_angle = as5600;
          property.DesiredPosition = rad2deg100(status.target_angle);
          if (status.is_servo_on) t.start();
          break;
      }
    }

    property.CurrentPosition = rad2deg100(as5600);
		if (as5600.getError()) break;
    float error = deg100_2rad(property.CurrentPosition) - status.target_angle;
    while(error > M_PI) error -= 2.0 * M_PI;
    while(error < -M_PI) error += 2.0 * M_PI;
    status.err_i += error * 0.001f;
    status.err_i = max(min(status.err_i, 1.0f), -1.0f); 

    float gain = property.Kp0 / 100.0f;
    float gain_i = property.Ki0 / 100.0f;
    float punch = property.StaticFriction0 / 100.0f;
    float pwm = gain_i * status.err_i;
    float margin = deg100_2rad(property.DeadBandWidth);
    if (fabs(error) > margin){
      if (error > 0){
        error -= margin;
        pwm += gain * error + punch;
      } else {
        error += margin;
        pwm += gain * error - punch;
      }
    }
    
    float max_torque = property.TorqueLimit / 100.0f;
    float val = max(min(pwm, max_torque), -max_torque);
    if (status.isWakeupMode) val *= 0.3;
    
		if (status.is_servo_on) motor = val;
    else motor = 0;
    {
      int len = commnand_parser.getReply(send_buf);
      if (len > 0){
        wait_us(30);
        for(int i = 0; i < len; i ++) rs485.putc(send_buf[i]);
      }
    }
    if (is_status_changed){
      motor.status_changed();
      is_status_changed = false;
    }
    wait(0.001);
  }
	motor = 0;
  while(1){
      blink_led = led2 = led3 = led4 = 1;
      wait(0.2);
      blink_led = led2 = led3 = led4 = 0;
      wait(0.2);
  }
}

// version { year, month, day, no }
char version[4] = { 17, 04, 23, 1 };

#include "mbed.h"
#include "AS5600.h"
#include "RS485.h"
#include "Parser.h"
#include "STM_BLDCMotor.h"
#include "Flash.h"
#include "b3m.h"

#define GAIN 10.0
#define GAIN_I 0.0
#define PUNCH 0.0
#define DEAD_BAND_WIDTH 0.1
#define MAX_ANGLE 60.0
#define MIN_ANGLE -60.0
#define BAUDRATE 115200
#define FLASH_ADDRESS 0x08010000

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define USE_WAKEUP_MODE

extern Property property;

DigitalOut blink_led(LED1);   // toggle LED every control loop
DigitalOut led2(LED2);        // toggle LED every send message
DigitalOut led3(LED3);        // servo on
DigitalOut led4(LED4);        // no assign
BusIn sw(SW1, SW2);
STM_BLDCMotor motor;
AS5600 as5600(I2C_SDA, I2C_SCL);
RS485 rs485(RS485_TX, RS485_RX, RS485_SELECT);
Parser commnand_parser;
Flash flash;
Timer t, loop_timer, position_read_timer;

const int MAX_COMMAND_LEN = 256; 
unsigned char command_data[MAX_COMMAND_LEN];
int command_len = 0;
const int LED_TOGGLE_COUNT = 500;
unsigned char send_buf[256];
unsigned char send_buf_len = 0;

const int stocked_number = 1000;
const int period_ms = 5;
short stocked_target_position[stocked_number];
short stocked_encoder_position[stocked_number];
short stocked_motor_position[stocked_number];
short stocked_pwm_duty[stocked_number];

struct RobotStatus {
  float target_angle;
  float initial_angle;
  bool is_servo_on;
  bool change_target;
  bool isWakeupMode;
  int led_state;
  int led_count;
  float err_i;
  int pulse_per_rotate;
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

int initialize()
{
  status.initial_angle = status.target_angle = as5600;   // read angle
  if (as5600.getError()) return -1;
  status.is_servo_on = false;
  status.led_state = 0;
  status.led_count = 0;
  status.change_target = false;
  status.isWakeupMode = false;
  status.err_i = 0.0;
  
  memset((void *)&property, 0, sizeof(property));
  property.ID = 0;
  property.Baudrate = BAUDRATE;
  property.PositionMinLimit = MIN_ANGLE * 100;
  property.PositionMaxLimit = MAX_ANGLE * 100;
  property.PositionCenterOffset = rad2deg100(status.target_angle);
  property.TorqueLimit = 100;
  property.DeadBandWidth = DEAD_BAND_WIDTH * 100;
  property.Kp0 = GAIN * 100;
  property.Ki0 = GAIN_I * 100;
  property.StaticFriction0 = PUNCH *100;
  property.FwVersion = (version[0] << 24) + (version[1] << 16) + (version[2] << 8) + version[3];
  return 0;
}

int main() {
  bool is_status_changed = false;
  int time_from_last_update = 0;
  int stocked_count = stocked_number;
  int sub_count = period_ms;
  
  if (initialize() == -1) goto error;
  blink_led = 0;
  sw.mode(PullUp);
  as5600 = as5600;    // ??
  t.reset();
  memcpy((void *)&property, (void *)FLASH_ADDRESS, sizeof(property));
  property.FwVersion = (version[0] << 24) + (version[1] << 16) + (version[2] << 8) + version[3];
  status.pulse_per_rotate = property.MCUTempLimit;
  if (status.pulse_per_rotate <= 0) status.pulse_per_rotate = 2000.0f;
  rs485.baud(property.Baudrate);
  motor.servoOn();
  loop_timer.start();
  position_read_timer.start();
  
  while(1){         // main loop
    status.led_count ++;
    if (status.led_count > LED_TOGGLE_COUNT){
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
    } else if (command == B3M_CMD_DATA_STOCK){
      stocked_count = 0;
    } else if (command == B3M_CMD_DATA_PLAY){
      led4 = 1;
      for(int i = 0; i < stocked_number; i ++){
        while(!rs485.isEnableSend()) wait_us(1);
        rs485.printf("%d, %d, %d, %d\r\n", 
          stocked_target_position[i], stocked_encoder_position[i],
          stocked_motor_position[i], stocked_pwm_duty[i]);
        wait_ms(10);
      }
      led4 = 0;
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
        case B3M_SYSTEM_MCU_TEMP_LIMIT:
          property.MCUTempLimit = data;
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
          property.DesiredPosition = rad2deg100(status.target_angle);
          is_status_changed = true;
          break;
        case B3M_CONTROL_KP0:
          property.Kp0 = data;
          break;
        case B3M_CONTROL_KD0:
          property.Kd0 = data;
          break;
        case B3M_CONTROL_KI0:
          property.Ki0 = data;
          status.err_i = 0;
          break;
        case B3M_CONTROL_STATIC_FRICTION0:
          property.StaticFriction0 = data;
          break;
        case B3M_CONTROL_KP1:
          property.Kp1 = data;
          break;
        case B3M_SERVO_SERVO_MODE:
          t.reset();
          status.is_servo_on = (data == 0) ? true : false;
          led3 = (status.is_servo_on) ? 1 : 0;
          
          status.initial_angle = status.target_angle = as5600;
          if (as5600.getError()) break;
          motor.resetHoleSensorCount();
          property.DesiredPosition = rad2deg100(status.target_angle);
          if (status.is_servo_on) t.start();
          break;
      }
    }

    property.PreviousPosition = property.CurrentPosition;
    short current_position = rad2deg100(- 2.0 * M_PI * (double)motor.getHoleSensorCount() / status.pulse_per_rotate + status.initial_angle);
    
    property.CurrentPosition = current_position;
    float period = position_read_timer.read();
    position_read_timer.reset();
    property.CurrentVelosity = property.CurrentVelosity * 0.9 + (property.CurrentPosition - property.PreviousPosition) / period * 0.1;
    
    float error = deg100_2rad(property.CurrentPosition) - status.target_angle;
    while(error > M_PI) error -= 2.0 * M_PI;
    while(error < -M_PI) error += 2.0 * M_PI;
    status.err_i += error * 0.001f;
    status.err_i = max(min(status.err_i, 0.001f), -0.001f); 
    
    float gain = property.Kp0 / 100.0f;
    float gain1 = property.Kp1 / 100.0f;
    float gain_d = property.Kd0 / 100.0f;
    float gain_i = property.Ki0 / 100.0f;
    float punch = property.StaticFriction0 / 100.0f;
    float pwm = gain_i * status.err_i;
    pwm += gain_d * deg100_2rad(property.CurrentVelosity);
    float margin = deg100_2rad(property.DeadBandWidth);
    if (fabs(error) > margin){
      if (error > 0){
        error -= margin;
        pwm += gain * error + punch;
      } else {
        error += margin;
        pwm += gain * error - punch;
      }
    } else {
        pwm += gain1 * error;
    }
    
    float max_torque = property.TorqueLimit / 100.0f;
    float val = max(min(pwm, max_torque), -max_torque);
    if (status.isWakeupMode) val *= 0.3;
    
    if (status.is_servo_on) motor = val;
    else motor = 0;
    property.PwmDuty = motor * 100;
    
    if (send_buf_len == 0){
      send_buf_len = commnand_parser.getReply(send_buf);
    }
    if (send_buf_len > 0){
      if (rs485.isEnableSend()){
        for(int i = 0; i < send_buf_len; i ++) rs485.putc(send_buf[i]);
        send_buf_len = 0;
      }
    }
    
    if ((is_status_changed)||(time_from_last_update >= 10)){
      motor.status_changed();
      is_status_changed = false;
      time_from_last_update = 0;
    }
    time_from_last_update ++;
    
    if (stocked_count < stocked_number){
      sub_count --;
      if (sub_count <= 0){
        sub_count = period_ms;
        float angle = as5600;
        if (as5600.getError()) angle = 0;
        stocked_target_position[stocked_count] = property.DesiredPosition - property.PositionCenterOffset;
        stocked_encoder_position[stocked_count] = rad2deg100(angle) - property.PositionCenterOffset;
        stocked_motor_position[stocked_count] = property.CurrentPosition - property.PositionCenterOffset;
        stocked_pwm_duty[stocked_count] = property.PwmDuty;
        stocked_count ++;
        led4 = 1;
      }
    } else {
      led4 = 0;
    }
    
    while(loop_timer.read_us() < 1000) wait_us(1);
    loop_timer.reset();
    loop_timer.start();
  }
error:
  motor = 0;
  while(1){   // error mode
      blink_led = led2 = led3 = led4 = 1;
      wait(0.2);
      blink_led = led2 = led3 = led4 = 0;
      wait(0.2);
  }
}

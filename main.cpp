//#define R2

#include "mbed.h"
#include "STM_BLDCMotor.h"
#include "AS5600.h"

#if defined(L1)
#define ID 'a'
#define OFFSET -0.985056

#elif defined(L2)
#define ID 'b'
#define OFFSET 0.549299

#elif defined(R1)
#define ID 'c'
#define OFFSET -1.360973

#elif defined(R2)
#define ID 'd'
#define OFFSET 2.458037

#else
#define ID 'a'
//#define OFFSET 1.89
#define OFFSET 2.20
#endif

#define MAX_ANGLE 0
#define MIN_ANGLE -45
#define GAIN 10.0
#define BAUDRATE 57600

#ifndef M_PI
#define M_PI           3.14159265358979323846f
#endif

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

BusOut led(LED2, LED3, LED4);
BusIn sw(SW1, SW2);
STM_BLDCMotor acmotor(LED1);
AS5600 as5600(I2C_SDA, I2C_SCL);
Serial serial(USBTX, USBRX);
Timer t;
float target_angle = 0;
bool is_servo_on = false;
bool change_target = false;

void initialize()
{
}

void isrRx() {
	static char buf[10];
	static int n = 0;
	while(serial.readable()){
		buf[n ++] = serial.getc();
		if (n >= 10) n = 0;
		if (buf[n-1] == '\r'){
			int t_angle;
			float f_angle;
			n = 0;
			if (buf[0] != ID) return;
			if (strlen(&buf[1] )!= 5) return;
			sscanf(&buf[1], "%d", &t_angle);
			f_angle = (float)t_angle / 10.0f;
//			printf("%d\r\n", t_angle);
			memset(buf, 0, sizeof(buf));
			if ((f_angle > MAX_ANGLE) || (f_angle < MIN_ANGLE)) return;
			target_angle = f_angle * M_PI / 180.0f;
			if (is_servo_on == false){
				acmotor.servoOn();
				is_servo_on = true;
			}
			change_target = true;
		}
	}
}

int main() {
	int previous_hole_state = 6;
	float gain = GAIN, max_value = 1.0;
	int counter = 25, led_counter  = 100, led_state = 0;
	bool is_low_gain = true;
	initialize();
	led = 0;
	sw.mode(PullUp);
	serial.attach(isrRx, Serial::RxIrq);
	serial.baud(BAUDRATE);
	
	as5600 = as5600 - OFFSET;
	while(1){
		float angle = as5600;
//		printf("%f\r\n", angle);
		if (as5600.getError()) break;
		if (is_low_gain){
			if(fabs(angle - target_angle) < 0.1){
				acmotor.setMaxDutyRatio(0.5);
				is_low_gain = false;
			}
		}
		float val = max(min(gain * (angle - target_angle), max_value), -max_value);
#if defined(L1) || defined(L2) || defined(R1) || defined(R2)
		acmotor = -val;
#else
		acmotor = val;
#endif
		if (change_target){
			acmotor.status_changed();
			change_target = false;
		}
		if (sw[0] == 0 && counter == 0){
			target_angle += 0.1;
			counter = 25;
		}
		if (sw[1] == 0 && counter == 0){
			target_angle -= 0.1;
			counter = 25;
		}
		if (counter > 0) counter --;
		if (led_counter == 0){
			led_state ^= 1;
			led[0] = led_state;
			led_counter = 100;
		}	
		led_counter --;
		t.reset();
		t.start();
		while(t.read() < 0.020f){
			if (acmotor.getHoleState() != previous_hole_state){
				acmotor.status_changed();
				previous_hole_state = acmotor.getHoleState();
//				printf("%d\r\n", previous_hole_state);
			}
		}
	}
	acmotor = 0;
	acmotor.status_changed();
	led = 0x02;
	while(1);
}

// Chiba Institute of Technology

#ifndef STM_BLDCMOTOR_H
#define STM_BLDCMOTOR_H

#define USE_AS5048B

#include "mbed.h"
#include "rtos.h"
#include "AngleSensor.h"

/** Class to control a motor on any pin, without using pwm pin
 *
 * Example:
 * @code
 * // STM_BLDCMotor Control
 * #include "mbed.h"
 * #include "STM_BLDCMotor.h"
 *
 * STM_BLDCMotor motor;
 *
 * int main(){
 *   motor.servoOn();
 *   motor = 0.1;    // duty ratio
 * }
 * @endcode
 */

class STM_BLDCMotor
{
public:
    /** Create a new SoftwarePWM object on any mbed pin
      *
      * @param Pin Pin on mbed to connect PWM device to
     */
    STM_BLDCMotor(AngleSensor *angle_sensor);

    void servoOn(void);

    void servoOff(void);

    void setMaxDutyRatio(float max_ratio);

    void setPwmPeriod(double seconds);

    void write(double value);

    float read();
    
    int getHoleState();

    int getState();

    void status_changed(void);

    void resetHoleSensorCount();
		
    void detectHoleState0(float duty_ratio);

//#ifdef MBED_OPERATORS
    /** A operator shorthand for write()
     */
    STM_BLDCMotor& operator= (float value) {
        write(value);
        return *this;
    }

    STM_BLDCMotor& operator= (STM_BLDCMotor& rhs) {
        write(rhs.read());
        return *this;
    }

    /** An operator shorthand for read()
     */
    operator float() {
        return read();
    }
//#endif

private:
    PwmOut uh_;
    DigitalOut ul_;
    PwmOut vh_;
    DigitalOut vl_;
    PwmOut wh_;
    DigitalOut wl_;
    Timeout st_;

    double value_;
    double period_sec_;
    double max_ratio_;
    bool enable_;
    int hole_state_no;
    float hole_state0_angle_;
    AngleSensor *_angle_sensor;
    float _angle;
    Thread *_thread;
    static void angle_read_thread(void const *argument);
    static int switching_table[6][3];
    void drive(int u, int v, int w);

    enum h_bridge{
      UH = 0,
      UL,
      VH,
      VL,
      WH,
      WL,
    };
};

#endif

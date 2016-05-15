// Chiba Institute of Technology

#ifndef MAKISUMI_ACMOTOR_H
#define MAKISUMI_ACMOTOR_H

#include "mbed.h"

/** Class to control a motor on any pin, without using pwm pin
 *
 * Example:
 * @code
 * // MakisumiACMotor Control
 * #include "mbed.h"
 * #include "MakisumiACMotor.h"
 *
 * MakisumiACMotor acmotor();
 *
 * int main(){
 * int previous_hole_state = 6;
 *   acmotor.servoOn();
 *   acmotor = 0.1;		// duty ratio
 *   while(1){
 *	    if (acmotor.getHoleState() != previous_hole_state){
 *       acmotor.status_changed();
 *       previous_hole_state = acmotor.getHoleState();
 *		  }
 *		}
 * }
 * @endcode
 */


class MakisumiACMotor
{
public:
    /** Create a new SoftwarePWM object on any mbed pin
      *
      * @param Pin Pin on mbed to connect PWM device to
     */
    MakisumiACMotor(PinName Ppwm);

    void servoOn(void);

    void servoOff(void);

		void setMaxDutyRatio(float max_ratio);

    void setPwmPeriod(double seconds);

    void write(double value);

    float read();
    
    int getHoleState();

    int getState();

    void status_changed(void);

#ifdef MBED_OPERATORS
    /** A operator shorthand for write()
     */
    MakisumiACMotor& operator= (float value) {
        write(value);
        return *this;
    }

    MakisumiACMotor& operator= (MakisumiACMotor& rhs) {
        write(rhs.read());
        return *this;
    }

    /** An operator shorthand for read()
     */
    operator float() {
        return read();
    }
#endif

private:
		InterruptIn pwm_int_;
		PwmOut pwm_;

		InterruptIn hole1_;
		InterruptIn hole2_;
		InterruptIn hole3_;

		double value_;
    double period_sec_;
		double max_ratio_;
    bool enable_;
		int hole_state;
		int hole_state_no;

    static int switching_table[6][3];
    void drive(int u, int v, int w);
    void pwmRise(void);
    void pwmFall(void);

		int on_swtiching_ptn[6];
		int off_swtiching_ptn[6];
		enum h_bridge{
			UH = 0,
			UL,
			VH,
			VL,
			WH,
			WL,
		};
		bool underChanging;
};

#endif

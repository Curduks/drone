#include "PID.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "MPU6050.h"

#define dt 0.001

extern volatile double roll,pitch;
extern volatile double angle_gx,angle_gy;

float roll_target_angle = 0.0;
float roll_angle_in;
float roll_rate_in;
float roll_stabilize_kp = 1;
float roll_stabilize_ki = 0;
float roll_rate_kp = 1;
float roll_rate_ki = 0;
float roll_stabilize_iterm;
float roll_rate_iterm;
float roll_output;

float pitch_target_angle = 0.0;
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp = 1;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 1;
float pitch_rate_ki = 0;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_output;


void Init_PID(){
    TCCR2 = 0x04;
    TCNT2 = 6;
    TIMSK |= (1<<TOIE2);
    sei();
}

ISR(TIMER2_OVF_vect){
    TCNT2 = 6;
    calcPRtoDualPID();
}

void calcPRtoDualPID(){
    roll_angle_in = roll;
    roll_rate_in = angle_gy;

    dualPID(roll_target_angle,roll_angle_in,roll_rate_in,
            roll_stabilize_kp,roll_stabilize_ki,roll_rate_kp,
            roll_rate_ki,roll_stabilize_iterm,roll_rate_iterm,roll_output);
    
    pitch_angle_in = pitch;
    pitch_rate_in = angle_gx;

    dualPID(pitch_target_angle,pitch_angle_in,pitch_rate_in,
            pitch_stabilize_kp,pitch_stabilize_ki,pitch_rate_kp,
            pitch_rate_ki,pitch_stabilize_iterm,pitch_rate_iterm,pitch_output);

}

void dualPID(float target_angle,float angle_in,
             float rate_in,float stabilize_kp,
             float stabilize_ki,float rate_kp,
             float rate_ki,float stabilize_iterm,
             float rate_iterm,float output)
             {
    float angle_error;
    float desired_rate;
    float rate_error;
    float stabilize_pterm, rate_pterm;

    angle_error = target_angle - angle_in;

    stabilize_pterm = stabilize_kp * angle_error;
    stabilize_iterm += stabilize_ki * angle_error * dt;

    desired_rate = stabilize_pterm;

    rate_error = desired_rate - rate_in;

    rate_pterm = rate_ki * rate_error;
    rate_iterm += rate_ki * rate_error * dt;

    output = rate_pterm + rate_iterm + stabilize_iterm;

}

void reset_stabilize_iterm(){
    roll_stabilize_iterm = 0;
    roll_rate_iterm = 0;
    pitch_stabilize_iterm = 0;
    pitch_rate_iterm = 0;
}
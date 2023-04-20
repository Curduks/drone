#include "PID.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "MPU6050.h"
#include "UART1.h"

#define dt 0.001
#define roll_stabilize_kp pitch_stabilize_kp
#define roll_stabilize_ki pitch_stabilize_ki
#define roll_rate_kp pitch_rate_kp
#define roll_rate_ki pitch_rate_ki
#define roll_rate_kd pitch_rate_kd

extern volatile double roll,pitch,yaw;
extern volatile double angle_gx,angle_gy,angle_gz;
extern volatile double g_x,g_y,g_z;
extern int memory_speed;

float roll_target_angle = 0.0;
float roll_angle_in;
float roll_rate_in;
//float roll_stabilize_kp = 1;
//float roll_stabilize_ki = 0;
//float roll_rate_kp = 2;
//float roll_rate_ki = 1;
float roll_stabilize_iterm;
float roll_rate_iterm;
float roll_output;
float roll_rate_error_last = 0;

float pitch_target_angle = 0.0;
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp = 16;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 2;
float pitch_rate_ki = 0;
float pitch_rate_kd = 2;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_output;
float pitch_rate_error_last = 0;

float yaw_target_angle = 0.0;
float yaw_angle_in;
float yaw_rate_in;
float yaw_stabilize_kp = 1;
float yaw_stabilize_ki = 0;
float yaw_rate_kp = 4;
float yaw_rate_ki = 0;
float yaw_stabilize_iterm;
float yaw_rate_iterm;
float yaw_output;

float Pitch_P;
float Pitch_PID;

float PID_I_MAX = 5;

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
    roll_rate_in = g_y;
    // PID_control(roll_target_angle,roll_angle_in,roll_rate_in,
    //         roll_stabilize_kp,roll_stabilize_ki,roll_rate_kp,
    //         roll_rate_ki,&roll_stabilize_iterm,&roll_rate_iterm,&roll_output,&roll_rate_error_last,roll_rate_kd);
    dualPID(roll_target_angle,roll_angle_in,roll_rate_in,
            roll_stabilize_kp,roll_stabilize_ki,roll_rate_kp,
            roll_rate_ki,&roll_stabilize_iterm,&roll_rate_iterm,&roll_output);

    pitch_angle_in = pitch;
    pitch_rate_in = g_x;
    // PID_control(pitch_target_angle,pitch_angle_in,pitch_rate_in,
    //         pitch_stabilize_kp,pitch_stabilize_ki,pitch_rate_kp,
    //         pitch_rate_ki,&pitch_stabilize_iterm,&pitch_rate_iterm,&pitch_output,&pitch_rate_error_last,pitch_rate_kd);
    dualPID(pitch_target_angle,pitch_angle_in,pitch_rate_in,
            pitch_stabilize_kp,pitch_stabilize_ki,pitch_rate_kp,
            pitch_rate_ki,&pitch_stabilize_iterm,&pitch_rate_iterm,&pitch_output);

    // yaw_angle_in = yaw;
    // yaw_rate_in = g_z;

    // dualPID(yaw_target_angle,yaw_angle_in,yaw_rate_in,
    //         yaw_stabilize_kp,yaw_stabilize_ki,yaw_rate_kp,
    //         yaw_rate_ki,&yaw_stabilize_iterm,&yaw_rate_iterm,&yaw_output);

}

void dualPID(float target_angle,float angle_in,
             float rate_in,float stabilize_kp,
             float stabilize_ki,float rate_kp,
             float rate_ki,float *stabilize_iterm,
             float *rate_iterm,float *output)
             {
    float angle_error;
    float desired_rate;
    float rate_error;
    float stabilize_pterm, rate_pterm;

    angle_error = target_angle - angle_in;

    stabilize_pterm = stabilize_kp * angle_error;
    *stabilize_iterm += stabilize_ki * angle_error * dt;

    desired_rate = stabilize_pterm;

    rate_error = desired_rate - rate_in;

    rate_pterm = rate_kp * rate_error;
    *rate_iterm += rate_ki * rate_error * dt;

    *output = rate_pterm + (*rate_iterm) + (*stabilize_iterm);

    if(memory_speed == 0){
        *stabilize_iterm = 0;
        *rate_iterm = 0;
        reset_stabilize_iterm();
        *output = 0;
    }

    if(angle_error == 0){
        reset_stabilize_iterm();
    }
    //*output *= 10;

}

void PID_control(float target_angle,float angle_in,
             float rate_in,float stabilize_kp,
             float stabilize_ki,float rate_kp,
             float rate_ki,float *stabilize_iterm,
             float *rate_iterm,float *output,
             float *rate_error_last,float rate_kd)
             {
    float angle_error;
    float desired_rate;
    float rate_error;
    float stabilize_pterm, rate_pterm, rate_dterm;

    angle_error = angle_in - target_angle;
    stabilize_pterm = angle_error * stabilize_kp; //각도 p 제어
    desired_rate = stabilize_pterm; 

    rate_error = desired_rate + rate_in; //angular rate
    rate_pterm = rate_error * rate_kp; //각속도 p 제어

    *rate_iterm += rate_ki * rate_error; //각속도 I 제어
    if(*rate_iterm > PID_I_MAX) *rate_iterm = PID_I_MAX;
    else if(*rate_iterm < PID_I_MAX * (-1)) *rate_iterm = PID_I_MAX * (-1);

    rate_dterm = (rate_error - *rate_error_last) * rate_kd;

    *rate_error_last = rate_error;

    *output = rate_pterm + *rate_iterm + rate_dterm;
    
    if(memory_speed == 0) *output = 0;

}

void reset_stabilize_iterm(){
    roll_stabilize_iterm = 0;
    roll_rate_iterm = 0;
    pitch_stabilize_iterm = 0;
    pitch_rate_iterm = 0;
}

void set_pitch(float data){
    pitch_target_angle = data;
}
void set_roll(float data){
    roll_target_angle = data;
}
void set_yaw(float data){
    yaw_target_angle = data;
}


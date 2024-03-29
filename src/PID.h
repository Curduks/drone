#ifndef PID_H_
#define PID_H_

void Init_PID();
void calcPRtoDualPID();
void reset_stabilize_iterm();
void dualPID(float target_angle,
             float angle_in,
             float rate_in,
             float stabilize_kp,
             float stabilize_ki,
             float rate_kp,
             float rate_ki,
             float *stabilize_iterm,
             float *rate_iterm,
             float *output);
void PID_control(float target_angle,float angle_in,
             float rate_in,float stabilize_kp,
             float stabilize_ki,float rate_kp,
             float rate_ki,float *stabilize_iterm,
             float *rate_iterm,float *output,
             float *rate_error_last,float rate_kd);
void set_pitch(float data);
void set_roll(float data);
void set_yaw(float data);

#endif /* PID_H_ */
/*
 * controller.c
 *
 * Created: 2023-02-14 오전 1:24:53
 *  Author: Hwabok
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "controller.h"
#include "timer.h"
#include "MPU6050.h"
#include "PID.h"

extern float roll_output;
extern float pitch_output;

extern volatile double yaw;

extern int memory_speed;
extern volatile int Motor_speed;
extern volatile int front_left_speed;
extern volatile int front_right_speed;
extern volatile int back_left_speed;
extern volatile int back_right_speed;

static unsigned char rotation_state = 0;
static unsigned char move_state = 0;

volatile int temp_front_left;
volatile int temp_front_right;
volatile int temp_back_left;
volatile int temp_back_right;

volatile unsigned char move_type = 0;

struct Status move_status;

void throttle(){
	//Motor_speed = 2000+(2000.0*height/100);
	if(memory_speed == 0){
		reset_stabilize_iterm();
		front_left_speed = 2000;
		front_right_speed = 2000;
		back_left_speed = 2000;
		back_right_speed = 2000;
	}else{
		front_left_speed = 2000+(2000.0*memory_speed/100) + pitch_output ;
		front_right_speed = 2000+(2000.0*memory_speed/100) + pitch_output;
		back_left_speed = 2000+(2000.0*memory_speed/100) - pitch_output;
		back_right_speed = 2000+(2000.0*memory_speed/100) - pitch_output;
		/*
		front_left_speed = 2000+(2000.0*memory_speed/100) + pitch_output - roll_output;
		front_right_speed = 2000+(2000.0*memory_speed/100) + pitch_output + roll_output;
		back_left_speed = 2000+(2000.0*memory_speed/100) - pitch_output - roll_output;
		back_right_speed = 2000+(2000.0*memory_speed/100) - pitch_output + roll_output;*/

		// front_left_speed = 2000+(2000.0*memory_speed/100) + roll_output;
		// front_right_speed = 2000+(2000.0*memory_speed/100) - roll_output;
		// back_left_speed = 2000+(2000.0*memory_speed/100) - roll_output ;
		// back_right_speed = 2000+(2000.0*memory_speed/100) + roll_output;
	}

	if(front_left_speed < 2000){
		front_left_speed = 2000;
	}if(front_left_speed > 4000){
		front_left_speed = 4000;
	}if(front_right_speed < 2000){
		front_right_speed = 2000;
	}if(front_right_speed > 4000){
		front_right_speed = 4000;
	}if(back_left_speed < 2000){
		back_left_speed = 2000;
	}if(back_left_speed > 4000){
		back_left_speed = 4000;
	}if(back_right_speed < 2000){
		back_right_speed = 2000;
	}if(back_right_speed > 4000){
		back_right_speed = 4000;
	}



}

void drone_rotation(int rotation){
	int temp = rotation;
	if(temp > 0){
		if(temp > 10) temp = 10;
		set_yaw(temp+yaw);
	}
	else {
		temp = -temp;
		if(temp > 10) temp = 10;
		set_yaw(yaw-temp); 
	}
}

void drone_move_angle(int angle){
	if(angle >= 22.5 && angle <= 67.5){
		move_type = 1; //북동
	}else if(angle >=67.5 && angle <= 112.5){
		move_type = 2; //북
	}else if(angle >= 112.5 && angle <= 157.5){
		move_type = 3; //북서
	}else if(angle >= 157.5 && angle <= 202.5){
		move_type = 4; //서
	}else if(angle >= 202.5 && angle <= 247.5){
		move_type = 5; //남서
	}else if(angle >= 247.5 && angle <= 292.5){
		move_type = 6; //남
	}else if(angle >= 292.5 && angle <= 337.5){
		move_type = 7; //남동
	}else if(angle == 0){
		move_type = 0;
		move_state = 0;
	}
	else{
		move_type = 8; //동
	}
	
}

void drone_move_power(int power){
	if(!move_state){
		int temp = power;
		switch (move_type){
		case 0:
			set_pitch(0);
			set_roll(0);
			break;
		case 1: // 북동
			if(temp > 20) temp = 20;
			set_pitch(-(temp/2));
			set_roll(-(temp/2));
			break;
		case 2: //북
			if(temp > 20) temp = 20;
			set_pitch(-temp);
			set_roll(0);
			break;
		case 3: //북서
			if(temp > 20) temp = 20;
			set_pitch(-(temp/2));
			set_roll(temp/2);
			break;
		case 4: //서
			if(temp > 20) temp = 20;
			set_pitch(0);
			set_roll(temp);
			break;
		case 5: //남서
			if(temp > 20) temp = 20;
			set_pitch(temp/2);
			set_roll(temp/2);
			break;
		case 6: //남
			if(temp > 20) temp = 20;
			set_pitch(temp);
			set_roll(0);
			break;
		case 7: //남동
			if(temp > 20) temp = 20;
			set_pitch(temp/2);
			set_roll(-(temp/2));
			break;
		case 8: //동
			if(temp > 20) temp = 20;
			set_pitch(0);
			set_roll(-temp);
			break;
		}
	}

}

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
#include "PID.h"

extern float roll_output;
extern float pitch_output;

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

void throttle(int height){
	//Motor_speed = 2000+(2000.0*height/100);
	front_left_speed = 2000+(2000.0*height/100) + pitch_output - roll_output;
	front_right_speed = 2000+(2000.0*height/100) + pitch_output + roll_output;
	back_left_speed = 2000+(2000.0*height/100) - pitch_output - roll_output;
	back_right_speed = 2000+(2000.0*height/100) - pitch_output + roll_output;

	if(height == 0){
		reset_stabilize_iterm();
	}
}

void drone_rotation(int rotation){
	if(!rotation_state){
		temp_front_left = front_left_speed;
		temp_front_right = front_right_speed;
		temp_back_left = back_left_speed;
		temp_back_right = back_right_speed;
		rotation_state = 1;
	}else{
		if(rotation > 0){
			front_left_speed = temp_front_left - rotation*3;
			back_right_speed = temp_back_right -  rotation*3;
		}else if(rotation < 0){
			front_right_speed = temp_front_right + rotation*3;
			back_left_speed = temp_back_left + rotation*3; 
		}else if(rotation == 0){
			rotation_state = 0;
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;
		}
	}
}
/*
void drone_move_angle(int angle){
	if(!move_state){
		temp_front_left = front_left_speed;
		temp_front_right = front_right_speed;
		temp_back_left = back_left_speed;
		temp_back_right = back_right_speed;
		move_state = 1;
	}else{
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
}

void drone_move_power(int power){
	if(!move_state){
		switch (move_type){
		case 0:
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;
			break;
		case 1: // 북동
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;
			move_north_east(power);
			break;
		case 2: //북
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;		
			move_north(power);
			break;
		case 3: //북서
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;
			move_north_west(power);
			break;
		case 4: //서
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;		
			move_west(power);
			break;
		case 5: //남서
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;		
			move_south_west(power);
			break;
		case 6: //남
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;		
			move_south(power);
			break;
		case 7: //남동
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;		
			move_south_east(power);
			break;
		case 8: //동
			front_left_speed = temp_front_left;
			front_right_speed = temp_front_right;
			back_left_speed = temp_back_left;
			back_right_speed = temp_back_right;		
			move_east(power);
			break;
		}
	}

}

void move_north(int power){
	if(power > 0 && power <=10){
		front_left_speed = temp_front_left - 10;
		front_right_speed = temp_front_right - 10;
	}else if(power > 10 && power <= 20){
		front_left_speed = temp_front_left - 20;
		front_right_speed = temp_front_right - 20;	
	}else if(power > 20 && power <= 30){
		front_left_speed = temp_front_left - 30;
		front_right_speed = temp_front_right - 30;	
	}else if(power > 30 && power <= 40){
		front_left_speed = temp_front_left - 40;
		front_right_speed = temp_front_right - 40;	
	}else if(power > 40 && power <= 50){
		front_left_speed = temp_front_left - 50;
		front_right_speed = temp_front_right - 50;	
	}else if(power > 50 && power <= 60){
		front_left_speed = temp_front_left - 60;
		front_right_speed = temp_front_right - 60;	
	}else if(power > 60 && power <= 70){
		front_left_speed = temp_front_left - 70;
		front_right_speed = temp_front_right - 70;	
	}else if(power > 70 && power <= 80){
		front_left_speed = temp_front_left - 80;
		front_right_speed = temp_front_right - 80;	
	}else if(power > 80 && power <= 90){
		front_left_speed = temp_front_left - 90;
		front_right_speed = temp_front_right - 90;	
	}else if(power > 90 && power <= 100){
		front_left_speed = temp_front_left - 100;
		front_right_speed = temp_front_right - 100;	
	}
}

void move_north_west(int power){
	if(power > 0 && power <=10){
		front_left_speed = temp_front_left - 10;
	}else if(power > 10 && power <= 20){
		front_left_speed = temp_front_left - 20;
	}else if(power > 20 && power <= 30){
		front_left_speed = temp_front_left - 30;
	}else if(power > 30 && power <= 40){
		front_left_speed = temp_front_left - 40;
	}else if(power > 40 && power <= 50){
		front_left_speed = temp_front_left - 50;
	}else if(power > 50 && power <= 60){
		front_left_speed = temp_front_left - 60;
	}else if(power > 60 && power <= 70){
		front_left_speed = temp_front_left - 70;
	}else if(power > 70 && power <= 80){
		front_left_speed = temp_front_left - 80;
	}else if(power > 80 && power <= 90){
		front_left_speed = temp_front_left - 90;
	}else if(power > 90 && power <= 100){
		front_left_speed = temp_front_left - 100;
	}
}
void move_west(int power){
	if(power > 0 && power <=10){
		front_left_speed = temp_front_left - 10;
		back_left_speed = temp_back_left - 10;
	}else if(power > 10 && power <= 20){
		front_left_speed = temp_front_left - 20;
		back_left_speed = temp_back_left - 20;	
	}else if(power > 20 && power <= 30){
		front_left_speed = temp_front_left - 30;
		back_left_speed = temp_back_left - 30;	
	}else if(power > 30 && power <= 40){
		front_left_speed = temp_front_left - 40;
		back_left_speed = temp_back_left - 40;	
	}else if(power > 40 && power <= 50){
		front_left_speed = temp_front_left - 50;
		back_left_speed = temp_back_left - 50;	
	}else if(power > 50 && power <= 60){
		front_left_speed = temp_front_left - 60;
		back_left_speed = temp_back_left - 60;	
	}else if(power > 60 && power <= 70){
		front_left_speed = temp_front_left - 70;
		back_left_speed = temp_back_left - 70;	
	}else if(power > 70 && power <= 80){
		front_left_speed = temp_front_left - 80;
		back_left_speed = temp_back_left - 80;	
	}else if(power > 80 && power <= 90){
		front_left_speed = temp_front_left - 90;
		back_left_speed = temp_back_left - 90;	
	}else if(power > 90 && power <= 100){
		front_left_speed = temp_front_left - 100;
		back_left_speed = temp_back_left - 100;	
	}
}
void move_south_west(int power){
	if(power > 0 && power <=10){
		back_left_speed = temp_back_left - 10;
	}else if(power > 10 && power <= 20){
		back_left_speed = temp_back_left - 20;	
	}else if(power > 20 && power <= 30){
		back_left_speed = temp_back_left - 30;	
	}else if(power > 30 && power <= 40){
		back_left_speed = temp_back_left - 40;	
	}else if(power > 40 && power <= 50){
		back_left_speed = temp_back_left - 50;	
	}else if(power > 50 && power <= 60){
		back_left_speed = temp_back_left - 60;	
	}else if(power > 60 && power <= 70){
		back_left_speed = temp_back_left - 70;	
	}else if(power > 70 && power <= 80){
		back_left_speed = temp_back_left - 80;	
	}else if(power > 80 && power <= 90){
		back_left_speed = temp_back_left - 90;	
	}else if(power > 90 && power <= 100){
		back_left_speed = temp_back_left - 100;	
	}
}
void move_south(int power){
	if(power > 0 && power <=10){
		back_right_speed = temp_back_right - 10;
		back_left_speed = temp_back_left - 10;
	}else if(power > 10 && power <= 20){
		back_right_speed = temp_back_right - 20;
		back_left_speed = temp_back_left - 20;	
	}else if(power > 20 && power <= 30){
		back_right_speed = temp_back_right - 30;
		back_left_speed = temp_back_left - 30;	
	}else if(power > 30 && power <= 40){
		back_right_speed = temp_back_right - 40;
		back_left_speed = temp_back_left - 40;	
	}else if(power > 40 && power <= 50){
		back_right_speed = temp_back_right - 50;
		back_left_speed = temp_back_left - 50;	
	}else if(power > 50 && power <= 60){
		back_right_speed = temp_back_right - 60;
		back_left_speed = temp_back_left - 60;	
	}else if(power > 60 && power <= 70){
		back_right_speed = temp_back_right - 70;
		back_left_speed = temp_back_left - 70;	
	}else if(power > 70 && power <= 80){
		back_right_speed = temp_back_right - 80;
		back_left_speed = temp_back_left - 80;	
	}else if(power > 80 && power <= 90){
		back_right_speed = temp_back_right - 90;
		back_left_speed = temp_back_left - 90;	
	}else if(power > 90 && power <= 100){
		back_right_speed = temp_back_right - 100;
		back_left_speed = temp_back_left - 100;	
	}
}
void move_south_east(int power){
	if(power > 0 && power <=10){
		back_right_speed = temp_back_right - 10;
	}else if(power > 10 && power <= 20){
		back_right_speed = temp_back_right - 20;
	}else if(power > 20 && power <= 30){
		back_right_speed = temp_back_right - 30;
	}else if(power > 30 && power <= 40){
		back_right_speed = temp_back_right - 40;
	}else if(power > 40 && power <= 50){
		back_right_speed = temp_back_right - 50;
	}else if(power > 50 && power <= 60){
		back_right_speed = temp_back_right - 60;
	}else if(power > 60 && power <= 70){
		back_right_speed = temp_back_right - 70;
	}else if(power > 70 && power <= 80){
		back_right_speed = temp_back_right - 80;
	}else if(power > 80 && power <= 90){
		back_right_speed = temp_back_right - 90;
	}else if(power > 90 && power <= 100){
		back_right_speed = temp_back_right - 100;
	}
}
void move_east(int power){
	if(power > 0 && power <=10){
		back_right_speed = temp_back_right - 10;
		front_right_speed = temp_front_right - 10;
	}else if(power > 10 && power <= 20){
		back_right_speed = temp_back_right - 20;
		front_right_speed = temp_front_right - 20;	
	}else if(power > 20 && power <= 30){
		back_right_speed = temp_back_right - 30;
		front_right_speed = temp_front_right - 30;	
	}else if(power > 30 && power <= 40){
		back_right_speed = temp_back_right - 40;
		front_right_speed = temp_front_right - 40;	
	}else if(power > 40 && power <= 50){
		back_right_speed = temp_back_right - 50;
		front_right_speed = temp_front_right - 50;	
	}else if(power > 50 && power <= 60){
		back_right_speed = temp_back_right - 60;
		front_right_speed = temp_front_right - 60;	
	}else if(power > 60 && power <= 70){
		back_right_speed = temp_back_right - 70;
		front_right_speed = temp_front_right - 70;	
	}else if(power > 70 && power <= 80){
		back_right_speed = temp_back_right - 80;
		front_right_speed = temp_front_right - 80;	
	}else if(power > 80 && power <= 90){
		back_right_speed = temp_back_right - 90;
		front_right_speed = temp_front_right - 90;	
	}else if(power > 90 && power <= 100){
		back_right_speed = temp_back_right - 100;
		front_right_speed = temp_front_right - 100;	
	}
}
void move_north_east(int power){
	if(power > 0 && power <=10){
		front_right_speed = temp_front_right - 10;
	}else if(power > 10 && power <= 20){
		front_right_speed = temp_front_right - 20;	
	}else if(power > 20 && power <= 30){
		front_right_speed = temp_front_right - 30;	
	}else if(power > 30 && power <= 40){
		front_right_speed = temp_front_right - 40;	
	}else if(power > 40 && power <= 50){
		front_right_speed = temp_front_right - 50;	
	}else if(power > 50 && power <= 60){
		front_right_speed = temp_front_right - 60;	
	}else if(power > 60 && power <= 70){
		front_right_speed = temp_front_right - 70;	
	}else if(power > 70 && power <= 80){
		front_right_speed = temp_front_right - 80;	
	}else if(power > 80 && power <= 90){
		front_right_speed = temp_front_right - 90;	
	}else if(power > 90 && power <= 100){
		front_right_speed = temp_front_right - 100;	
	}
}
*/
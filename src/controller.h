/*
 * controller.h
 *
 * Created: 2023-02-14 오전 12:36:26
 *  Author: Hwabok
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

struct Status{
	volatile int power_status;
	volatile int rotation_status;
	volatile int angle_status;
	volatile int strength_status;
};

void throttle(int height);
void drone_rotation(int rotation);
/*
void drone_move_angle(int angle);
void drone_move_power(int power);
void move_north(int power);
void move_north_west(int power);
void move_west(int power);
void move_south_west(int power);
void move_south(int power);
void move_south_east(int power);
void move_east(int power);
void move_north_east(int power);
*/
#endif /* CONTROLLER_H_ */
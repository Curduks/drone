/*
 * DroneMake.c
 *
 * Created: 2023-01-22 오후 11:48:33
 * Author : Hwabok
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "UART0.h"
#include "UART1.h"
#include "timer.h"
#include "MPU6050.h"
#include "PID.h"
#include "controller.h"
#include <util/delay.h>
#define F_CPU 16000000UL

extern volatile double roll;
extern volatile double pitch;
extern volatile double yaw;
extern unsigned char UDR0_data;
extern volatile int Motor_speed;
extern struct Status move_status;

extern float roll_output;
extern float pitch_output;


int main(void){
	Init_Timer3();
	move_status.angle_status = 0;
	move_status.power_status = 0;
	move_status.rotation_status = 0;
	move_status.strength_status = 0;
	//Init_Timer1();
	//Motor_speed = 2000;
	UART0_initialize();
	UART1_initialize();
	Init_MPU6050();
	calibrate();
	Init_PID();
    while (1){
		
		if(get_is_receive()){
			//throttle(get_received_value());
			if(move_status.power_status == 1){
				throttle(get_received_value());
				move_status.power_status = 0;
			}	
			if(move_status.rotation_status == 1){
				drone_rotation(get_received_value());
				move_status.rotation_status = 0;
			}
			if(move_status.angle_status == 1){
				//drone_move_angle(get_received_value());
				move_status.angle_status = 0;
			}
			if(move_status.strength_status == 1){
				//drone_move_power(get_received_value());
				move_status.strength_status = 0;
			}
			USART_Transmit_init4(get_received_value());
			Str_Clear();
			set_is_receive();
			set_received_value();
		}
		
		get_raw_data();
		calculate();

		USART1_Transmit_init4(roll);
		UART1_TX('\t');
		USART1_Transmit_init4(pitch);
		UART1_TX('\t');
		USART1_Transmit_init4(yaw);
		UART1_TX('\n');
		UART1_TX('\r');
    }
}


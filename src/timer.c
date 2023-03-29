/*
 * timer.c
 *
 * Created: 2023-01-22 오후 11:52:48
 *  Author: Hwabok
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>

#define Motor_mode 2000 //모터시작2000 or 셋업4000
volatile int Motor_speed = 2200; //셋업할시 2000 or 2200 = 10%출력 (최대 4000)
volatile int front_left_speed = 2000; //front_right ocr3a
volatile int front_right_speed = 2000;//back_right ocr3b
volatile int back_left_speed = 2000; //front_left ocr3c
volatile int back_right_speed = 2000; //back_left ocr1c

void Init_Timer3(){
	DDRE = 0b00111000;
	TCCR3A = 0b10101010;
	TCCR3B = 0b00011010;
	ICR3 = 40000; //20ms주기 파형
	SREG |= 0x80;
	ETIMSK |= 0b00000100;
	OCR3A = Motor_mode; //초기값 모터시작2000 or 셋업4000
	OCR3B = Motor_mode; //초기값
	OCR3C = Motor_mode; //초기값

	DDRB = 0b10000000;
	TCCR1A = 0b00001010;
	TCCR1B = 0b00011010;
	ICR1 = 40000;
	OCR1C = Motor_mode;
	TIMSK |= 0b00000100; //a
}

void Init_Timer1(){
	DDRB = 0b10000000;
	TCCR1A = 0b00001010;
	TCCR1B = 0b00011010;
	ICR1 = 40000;
	OCR1C = back_left_speed;
	TIMSK |= 0b00000100; //a
}

unsigned int i,k = 0;


ISR(TIMER3_OVF_vect){
	if(i >= 250){
		OCR3A = front_right_speed;
		OCR3B = back_right_speed;
		OCR3C = front_left_speed;
		OCR1C = back_left_speed;
	}
	i+=1;
}

ISR(TIMER1_OVF_vect){
	if(k >= 250){
		//OCR1C = back_left_speed;
	}
	k+=1;
}

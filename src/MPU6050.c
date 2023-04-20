/*
 * MPUU6050.c
 *
 * Created: 2023-02-03 오후 10:43:56
 *  Author: Hwabok
 */ 

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include "MPU6050.h"
#include "UART1.h"
#include <math.h>
#define FS_SEL 131

volatile double dt = 0.000;
volatile int temp;
volatile unsigned char a_x_l,a_x_h,a_y_l,a_y_h,a_z_l,a_z_h;
volatile unsigned char g_x_l,g_x_h,g_y_l,g_y_h,g_z_l,g_z_h;
volatile double bas_a_x,bas_a_y,bas_a_z;
volatile double bas_g_x,bas_g_y,bas_g_z;
volatile double a_x,a_y,a_z;
volatile double g_x,g_y,g_z;
volatile double las_angle_gx,las_angle_gy,las_angle_gz;
volatile double angle_ax,angle_ay,angle_az;
volatile double angle_gx,angle_gy,angle_gz;
volatile double roll,pitch,yaw;
volatile double alpha;

void Init_MPU6050(){
	TWCR = (1<<TWEN);
	TWBR = 12;
	TCCR0 = (1<<CS02)|(1<<CS01);
	TCNT0 = 256-125;
	TIMSK |= (1<<TOIE0);
	twi_write(0x6B,0x00);
	_delay_ms(1);
	twi_write(0x1A,0x05);
	SREG = 0x80;
}

ISR(TIMER0_OVF_vect){
	dt += 0.002;
	
	TCNT0 = 256-125;
}

void twi_write(unsigned char address,unsigned char data){
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x08);
	
	TWDR = 0b11010000;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x18);
	
	TWDR = address;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);
	
	TWDR = data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);	
	
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

unsigned char twi_read(char address){
	unsigned char data;
	
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x08);
	
	TWDR = 0b11010000;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x18);
	
	TWDR = address;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x28);
	
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);		
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x10);
	
	TWDR = 0b11010001;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x40);	
	
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR&0xF8) != 0x58);	
	
	data = TWDR;
	
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	
	return data;
}

void calibrate(){
	int cal = 10;
	
	for(int i = 0; i < cal; i++){
		get_raw_data();
		
		temp = (a_x_h<<8) | a_x_l;
		a_x += - temp - 16383;
		temp = (a_y_h<<8) | a_y_l;
		a_y += - temp;
		temp = (a_z_h<<8) | a_z_l;
		a_z += temp;
		temp = (g_x_h<<8) | g_x_l;
		g_x += temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y += temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z += temp;
		
		_delay_ms(100);
	}
	
	a_x /= cal;
	a_y /= cal;
	a_z /= cal;
	g_x /= cal;
	g_y /= cal;
	g_z /= cal;
	
	bas_a_x = a_x;
	bas_a_y = a_y;
	bas_a_z = a_z;
	bas_g_x = g_x;
	bas_g_y = g_y;
	bas_g_z = g_z;
}

void get_raw_data(){
	a_x_h = twi_read(0x3b);
	_delay_us(10);
	a_x_l = twi_read(0x3c);
	_delay_us(10);
	a_y_h = twi_read(0x3D);
	_delay_us(10);
	a_y_l = twi_read(0x3E);
	_delay_us(10);
	a_z_h = twi_read(0x3F);
	_delay_us(10);
	a_z_l = twi_read(0x40);
	_delay_us(10);
	g_x_h = twi_read(0x43);
	_delay_us(10);
	g_x_l = twi_read(0x44);
	_delay_us(10);
	g_y_h = twi_read(0x45);
	_delay_us(10);
	g_y_l = twi_read(0x46);
	_delay_us(10);
	g_z_h = twi_read(0x47);
	_delay_us(10);
	g_z_l = twi_read(0x48);
	_delay_us(10);
}

/**/
void calculate(){
	las_angle_gx = roll;
	las_angle_gy = pitch;
	las_angle_gz = yaw;
	
	temp = (a_x_h<<8) | a_x_l;
	a_x = -temp - 16383;
	temp = (a_y_h<<8) | a_y_l;
	a_y = -temp;
	temp = (a_z_h<<8) | a_z_l;
	a_z = temp;
	temp = (g_x_h<<8) | g_x_l;
	g_x = temp;
	temp = (g_y_h<<8) | g_y_l;
	g_y = temp;
	temp = (g_z_h<<8) | g_z_l;
	g_z = temp;
	
	g_x = (g_x - bas_g_x)/FS_SEL;
	g_y = (g_y - bas_g_y)/FS_SEL;
	g_z = (g_z - bas_g_z)/FS_SEL;
	
	angle_ax = atan(-1.000*a_y/sqrt(pow(a_x,2) + pow(a_z,2)))*180/3.141592 - 1; // -1 오프셋
	angle_ay = atan(a_x/sqrt(pow(a_y,2) + pow(a_z,2)))*180/3.141592 + 36; //36은 오프셋
	
	angle_gx = g_x*dt + las_angle_gx;
	angle_gy = g_y*dt + las_angle_gy;
	angle_gz = g_z*dt + las_angle_gz;
	
	dt = 0.000;

	alpha = 0.96;
	roll = alpha*angle_gx + (1.000 - alpha)*angle_ax;
	pitch = alpha*angle_gy + (1.000 - alpha)*angle_ay;
	//yaw = alpha*angle_gz + (1.000 - alpha)*angle_az;
	yaw = angle_gz;
	
	
}
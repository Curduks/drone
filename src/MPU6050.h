/*
 * MPU6050.h
 *
 * Created: 2023-02-03 오후 10:39:25
 *  Author: Hwabok
 */ 


#ifndef MPU6050_H_
#define MPU6050_H_

void Init_MPU6050();
void twi_write(unsigned char address,unsigned char data);
unsigned char twi_read(char address);
void get_raw_data();
void calibrate();
void calculate();

#endif /* MPU6050_H_ */
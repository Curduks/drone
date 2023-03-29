/*
 * UART0.h
 *
 * Created: 2023-02-01 오후 9:22:15
 *  Author: Hwabok
 */ 


#ifndef UART0_H_
#define UART0_H_

void UART0_initialize();
void UART0_TX(unsigned char data);
void UART0_TX_string(unsigned char* str);
char UART0_RX();
unsigned char* DEC_TO_CHAR(long data);
void USART_Transmit_init4(int data);
unsigned char* USART_RX_int();
void Str_Clear();
int get_received_value();
int get_is_receive();
void set_received_value();
void set_is_receive();
void trans_data();
unsigned char UDR0_data;
int UART0_data;


#endif /* UART0_H_ */
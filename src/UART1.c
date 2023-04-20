#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include "UART1.h"

unsigned char str[16];

void UART1_initialize(){
	UCSR1A = 0x00;
	UCSR1B = 0x98;
    UCSR1C = 0x06;
	UBRR1H = 0x00;
	UBRR1L = 103;
	SREG |= 0x80;
}

void UART1_TX(unsigned char data){
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1=data;
}

void UART1_TX_string(unsigned char* str){
	while(*str){
		while((UCSR1A & 0x20)==0);
		UDR1 = *str;
		str++;
	}
}

char UART1_RX(){
	while(!(UCSR1A & 0x80));
	return UDR1;
}

void USART1_Transmit_init4(int data)
{
	if(data < 0)
	{
		data = -data;
		UART1_TX('-');
		//UART1_TX(' ');
	}
	else
	UART1_TX(' ');


	int temp = 0;
	temp = data/10000;
	UART1_TX(temp+48);
	temp = (data%10000)/1000;
	UART1_TX(temp+48);
	temp = (data%1000)/100;
	UART1_TX(temp+48);
	temp = (data%100)/10;
	UART1_TX(temp+48);
	temp = data%10;
	UART1_TX(temp+48);
}

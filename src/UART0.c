/*
 * UART0.c
 *
 * Created: 2023-02-01 오후 9:24:19
 *  Author: Hwabok
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include "UART1.h"
#include "UART0.h"
#include "controller.h"

unsigned char UDR0_data=0;
int UART0_data = 0;
static unsigned char save_data[15] = {0};

unsigned char str[16];
volatile static unsigned char is_receivce = 0;
volatile static int received_value = 0;
unsigned char UDR0_data;
extern struct Status move_status;

unsigned char get_Data[16];
unsigned char ch = 0;

void UART0_initialize(){
	UCSR0A = 0x00;
	UCSR0B = (1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) |
			 (1<<TXEN0)|(0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	UCSR0C = (0<<UMSEL0) | (0<<UPM01) | (0<<UPM00) |(0<<USBS0) |
			 (1<<UCSZ01) | (1<<UCSZ00);
	UBRR0H = 0x00;
	UBRR0L = 103;
	SREG |= 0x80;
}

void UART0_TX(unsigned char data){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0=data;
}

void UART0_TX_string(unsigned char* str){
	while(*str){
		while((UCSR0A & 0x20)==0);
		UDR0 = *str;
		str++;
	}
}

char UART0_RX(){
	while(!(UCSR0A & 0x80));
	return UDR0;
}

unsigned char* DEC_TO_CHAR(long data){
	unsigned char i , j, temp;
	for(i=0; i<10;i++)
		save_data[i] = 0;
	
	if(data>=0){
		save_data[0] = '+';
		for(i=1;i<10;i++){
			save_data[i] = (data % 10) + '0';
			data/=10;
			if(data==0) break;
		}
	}else{
		data*=-1;
		save_data[0] = '-';
		for(i=1;i<10;i++){
			save_data[i] = (data % 10) + '0';
			data/=10;
			if(data==0) break;
		}
	}
	i++;
	for(j=1;j<=(i/2);j++){
		temp=save_data[j];
		save_data[j]=save_data[i-j];
		save_data[i-j] = temp;
	}
	return save_data;
	
}

void USART_Transmit_init4(int data)
{
	if(data < 0)
	{
		data = -data;
		UART0_TX('-');
	}
	else
	UART0_TX(' ');

	int temp = 0;
	temp = data/10000;
	UART0_TX(temp+48);
	temp = (data%10000)/1000;
	UART0_TX(temp+48);
	temp = (data%1000)/100;
	UART0_TX(temp+48);
	temp = (data%100)/10;
	UART0_TX(temp+48);
	temp = data%10;
	UART0_TX(temp+48);
}

unsigned char* USART_RX_int(){
	return atoi(str);
}

void Str_Clear(){
	for(unsigned char i = 0; i < 16; i++) get_Data[i] = 0;
	ch = 0;
}

int get_received_value(){
	return received_value;
}

int get_is_receive(){
	return is_receivce;
}

void set_received_value(){
	received_value = 0;
}

void set_is_receive(){
	is_receivce = 0;
}
/*
ISR(USART0_RX_vect){
	//unsigned char temp = UDR0;
	unsigned char received_char = UDR0;
	if(received_char >= '0' && received_char <= '9'){
		int digit = received_char - '0';
		received_value = received_value * 10 + digit;
	}
	else if(received_char == '\n' || received_char == '\r'){
		is_receivce = 1;
	}
	switch (received_char)
	{
	case 'p':
		UART0_TX(received_char);
		move_status.power_status = 1;
		break;
	case 'a':
		UART0_TX(received_char);
		move_status.angle_status = 1;
		break;
	case 's':
		UART0_TX(received_char);
		move_status.strength_status = 1;
		break;
	case 'r':
		UART0_TX(received_char);
		move_status.rotation_status = 1;
		break;
	default:
		break;
	}
		
}
*/
 // "abc"
ISR(USART0_RX_vect){
	unsigned char received_char = UDR0;
	get_Data[ch++] = received_char;
	if(received_char == '\n' || received_char == '\r'){
		is_receivce = 1;
		trans_data();
		if(get_Data[1] == '-')
			received_value *= -1;
	}
}

void trans_data(){
	for(unsigned char i = 1; i < ch-1; i++){
		int digit = get_Data[i] - '0';
		received_value = received_value * 10 + digit;
	}
	switch (get_Data[0])
	{
	case 'p':
		UART0_TX(get_Data[0]);
		move_status.power_status = 1;
		break;
	case 'a':
		UART0_TX(get_Data[0]);
		move_status.angle_status = 1;
		break;
	case 's':
		UART0_TX(get_Data[0]);
		move_status.strength_status = 1;
		break;
	case 'r':
		UART0_TX(get_Data[0]);
		move_status.rotation_status = 1;
		break;
	}
	ch = 0;
}
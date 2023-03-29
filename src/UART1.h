#ifndef UART1_H_
#define UART1_H_

void UART1_initialize();
void UART1_TX(unsigned char data);
void UART1_TX_string(unsigned char* str);
void USART1_Transmit_init4(int data);
char UART1_RX();

#endif /* UART0_H_ */
/*
* GccApplication11.c
*
* Created: 24-05-2018 15:20:43
* Author : user
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define F_CPU 20000000UL
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(20000000.0 * 64 / (16 * (float)BAUD_RATE)) + 0.5)
void uart_init();
void uart_recv_data();
void uart_send_data(char);
void pwm0_init();
void clk_init();
char data1,data;
                                            
int main(void)
{
	clk_init();
	uart_init();
	pwm0_init();
	PORTD.OUTSET|=PIN0_bm; //STDBY
	
	while (1)
	{
		uart_recv_data();
		data1=(char)data;
		data='\0';
		switch(data1)
		{
			case 'F':
						
						uart_send_data('F');
						PORTC.OUT &=0XF0;
						PORTC.OUT|=1<<1|0<<0;
						_delay_ms(100000);
						PORTC.OUT|=1<<1|1<<0;
						data1='\0';
						break;
			case 'B':
						//PORTA.OUTSET|=PIN4_bm; //STDBY
						//PORTA.OUTSET|=PIN4_bm; //STDBY
						uart_send_data('B');
						PORTC.OUT &=0XF0;
						PORTC.OUT|=0<<1|1<<0 |1<<2|0<<3;
						_delay_ms(3000);
						PORTC.OUT|=1<<1|1<<0|1<<2|1<<3;
						data1='\0';
						break;
			case 'L':
						//PORTA.OUTSET|=PIN4_bm; //STDBY
						//PORTA.OUTSET|=PIN4_bm; //STDBY
						uart_send_data('B');
						PORTC.OUT &=0XF0;
						PORTC.OUT|=0<<1|1<<0 |0<<2|1<<3;
						_delay_ms(30000);
						PORTC.OUT|=1<<1|1<<0|1<<2|1<<3;
						data1='\0';
						break;
			case 'R':
						//PORTA.OUTSET|=PIN4_bm; //STDBY
						//PORTA.OUTSET|=PIN4_bm; //STDBY
						uart_send_data('B');
						PORTC.OUT &=0XF0;
						PORTC.OUT|=1<<1|0<<0 |0<<2|1<<3;
						_delay_ms(30000);
						PORTC.OUT|=1<<1|1<<0|1<<2|1<<3;
						data1='\0';
						break;			
			default:
						PORTA.OUT=1<<3;
						PORTA.OUT=1<<2;
						break;
			
		}
	}


}
void uart_recv_data()
{

	{
		if (USART0_STATUS & USART_RXCIF_bm)
		{
			data=USART0_RXDATAL;

		}

		_delay_ms(1000);
	}
}
void uart_init()
{
	PORTA.DIRCLR|=PIN1_bm; //PA0 AND PA1
	PORTA.PIN1CTRL&=PIN1_bp;
	PORTA_DIRSET|=PIN0_bm;
	PORTA.OUTCLR|=PIN0_bm;
	USART0_CTRLC&=0x00;
	USART0_CTRLC|=0X03;
	USART0_BAUD=(uint16_t)USART0_BAUD_RATE(9600);
	USART0_CTRLB|=0XC0;
}
void pwm0_init()
{
	PORTMUX.TCAROUTEA|=0X01;
	TCA0.SINGLE.CTRLA|=0X05;
	TCA0.SINGLE.CTRLB|=0X75;
	TCA0.SINGLE.CTRLESET&=0X00;
	TCA0.SINGLE.INTCTRL|=0X41;
	TCA0.SINGLE.CNT&=0X0000;
	TCA0.SINGLE.PER|=0XFFFF;
	TCA0.SINGLE.CMP1=0X7FFF;
	TCA0.SINGLE.CMP0=0X7FFF;
	//SET THE PORT AS OUTPUT
	PORTB.DIRSET|=PIN1_bm;		//PWM pins PB0,PB1
	PORTB.DIRSET|=PIN0_bm;
	PORTC.DIRSET|=0X03;
	PORTD.DIRSET|=PIN0_bm;
	/* Replace with your application code */
}
void clk_init()
{
	CCP|=CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB&=0X00;
	SREG|=0X80;
}
void uart_send_data(char msg)
{
	if (USART0_STATUS & USART_DREIF_bm)
	{
		USART0_TXDATAL=(uint8_t)msg;		
	}
}



/*
* Battery_Indicator.c
*
* Created: 21.05.2019 13:14:24
* Author : Denis Denk
*/

#include "main.h"

/*
// ���������� ���������� ������
ISR(INT0_vect) {
	cli();
	if (GIFR & (1<<INTF0)) {
	}
	sei();
}

// ���������� ���������� �������0
ISR(TIMER0_OVF_vect) {
}
*/


int main(void) {
	// ����������� ��� ���������
	PORT_Init();
	ADC_Init();
	// PWM_Init();
	USART_Init();
	// TIM_Init();
	// INT_Init();
	sei();
	
	while(1) {
		// ������ ������, ������� ������������
		while(bit_is_clear(PIND, 2)) {
			_delay_ms(50);
			counter++;
		}
		// ���� ���������� �������
		if(counter > 15) {
			// ������ ���� ����� ������, ���������� ����
			STR_UART_Send("Long press!\r\n");
			longPush = true;
			counter = 0;
		}
		// ���� �������� �������
		if(counter > 0 && counter < 5) {
			// ���� ����� ��� ���� ������� - ����������/��������� �������
			if(longPush) {
				// ���� �������� - ��������� � ��������
				if(!run) {
					//for (i = 0; i < 255; i++) {
						//OCR1A++;
						//_delay_ms(2);
						// run = true;
					//}
					run = true;
					PORTB |= (1 << PB1);
					STR_UART_Send("Work!\r\n");
				}
				else {
					//OCR1A = 0;
					PORTB &= ~(1 << PB1);
					run = false;
					STR_UART_Send("Work done!\r\n");
				}
				// ���������� ���� �������� �������
				longPush = false;
			}
			// �����������, �������� �������, ���������� ����������
			else {
				//STR_UART_Send("Short press!\r\n");
				counter = 0;
				ShowVoltage();
			}
		}
	}
}

void PORT_Init(void) {
	// PD0 � PD1 - ������ ��� USART
	// PB1 - ����� � ���
	DDRB |= (1 << PB1);

	DDRD |= (1 << PD5) | (1 << PD6)| (1 << PD7); // ������� - ������ - ������ 
	DDRB |= (1 << PB2) | (1 << PB3); // ������� - �������
	
	// ����������� ���������� �� ������� ������ PD2 (INT0)
	// ���� �������� � �������, ������ �������� ����� �� �����.
	DDRD &= ~(1 << PD2);
	PORTD |= (1 << PD2);	
}

void INT_Init(void) {
	// ��������� ���������� �� INT0
	GICR |= (1 << INT0);
	// ���������� �� ������������ ������ ������� �� INT0
	MCUCR |= (1 << ISC01);
}

void USART_Init() {
	// ��������� ����������: 8 ��� ������, 1 �������� ���, ��� �������� ��������
	// USART ��������: �������
	// USART ����������: �������
	// USART �����: �����������
	UBRRL = (F_CPU/BAUDRATE/16-1); // ��������� �������� ������ �������
	UBRRH = (F_CPU/BAUDRATE/16-1) >> 8;
	UCSRB |= (1 << RXCIE)| // ��������� ���������� �� ���������� ������ ������
	(1 << RXEN)|(1 << TXEN); // �������� �������� � ����������
	UCSRC |= (1 << URSEL)| // ��� ������� � �������� UCSRC ���������� ��� URSEL
	(1 << UCSZ1)|(1 << UCSZ0); // ������ ������� � ����� 8 ���
}

void ADC_Init(void) {
	// ��������� ���// ������������ ��������������� �� 8
	ADCSRA |= (1 << ADEN) | (1 << ADPS1)|(1 << ADPS0);
	// ������� ��� // ���� PC0
	ADMUX |= (0 << REFS1) | (0 << REFS0) |(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3);
}

void PWM_Init(void) {
	// �� ������ OC1A �������, ����� OCR1A==TCNT1, ������������ ���
	TCCR1A = (1 << COM1A1) | (1 << WGM10);
	// �������� = / 1
	TCCR1B = (1 << CS10);
	// ��������� ��������, ������ ���� ����� �������� �� 0 �� 255 ��� ��������� ����������
	OCR1A=0x00;
}

void TIM_Init(void) {
	// ������������ = 1024
	TCCR0 |= (1 << CS02) | (1 << CS00);
	//��������� ���������� �� ������������ ������� 0
	TIMSK |= (1 << TOIE0);
	//��������� ����������
	sei();
}

void ShowVoltage(void) {
	/* ��������� ������ � ��� */
	uint16_t ADCdata[5] = {0};
	uint16_t ADC_ExpVal = 0;
	// ������� ���������� 4 ���� � ������� ��� �������� ��� ��������
	for(uint8_t i = 0; i < 4; i++) {
		// �������� ��������������
		ADCSRA |= (1 << ADSC);
		// ���� ����� ��������� ��������������
		while ((ADCSRA&(1 << ADIF))== 0);
		// ���������  ADC
		ADCdata[i] = (ADCL|ADCH << 8);
	}
	ADC_ExpVal = (ADCdata[0] + ADCdata[1] + ADCdata[2] + ADCdata[3]) / 4;
	// ������� ���������� 5�, �������� �������� �� 1024 => ����������� = 5/1024=0.0048828
	float voltage = ADC_ExpVal * 0.0048828;
	// �������� � ���� 30�.
	voltage = map(voltage, 0, 5, 0, 25.2);
	// �������� �������� � UART ��� �������
	char buf[50];
	sprintf(buf, "ADC Data: %d\tVoltage: %f\r\n", ADC_ExpVal, voltage);
	STR_UART_Send(buf);

	/* 100% ���������� ����������� = 25,2�.
	* 0% = 19,8�.
	* ������� 5,4�. => 20% ��� 1.08�.
	*/
	
	/*
		DDRD |= (1 << PD5) | (1 << PD6)| (1 << PD7); // ������� - ������ - ������
		DDRB |= (1 << PB2) | (1 << PB3) // ������� - �������
	*/

	// 100%
	if(voltage >= 24.12) {
		PORTD |= (1 << PD5) | (1 << PD6)| (1 << PD7);
		PORTB |= (1 << PB2) | (1 << PB3);
		// ������ 2 �������, ����� ��� �����
		_delay_ms(500);
		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);
		PORTD &= ~(1 << PD5);
		PORTD &= ~(1 << PD6);
		PORTD &= ~(1 << PD7);
	}
	// 80%
	else if(voltage >= 23.04 && voltage < 24.12) {
		PORTD |= (1 << PD5) | (1 << PD6)| (1 << PD7);
		PORTB |= (1 << PB2);
		PORTB &= ~(1 << PB3);
		// ������ 2 �������, ����� ��� �����
		_delay_ms(500);
		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);
		PORTD &= ~(1 << PD5);
		PORTD &= ~(1 << PD6);
		PORTD &= ~(1 << PD7);
	}
	// 60%
	else if(voltage >= 21.96 && voltage < 23.04) {
		PORTD |= (1 << PD5) | (1 << PD6)| (1 << PD7);
		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);
		// ������ 2 �������, ����� ��� �����
		_delay_ms(500);
		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);
		PORTD &= ~(1 << PD5);
		PORTD &= ~(1 << PD6);
		PORTD &= ~(1 << PD7);
	}
	// 40%
	else if(voltage >= 20.88 && voltage < 21.96) {
		PORTD |= (1 << PD5) | (1 << PD6);
		PORTD &= ~(1 << PD7);
		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);
		// ������ 2 �������, ����� ��� �����
		_delay_ms(500);
		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);
		PORTD &= ~(1 << PD5);
		PORTD &= ~(1 << PD6);
		PORTD &= ~(1 << PD7);
	}
	// 20%
	else if(voltage >= 19.8 && voltage < 20.88) {
		PORTD |= (1 << PD5);
		PORTD &= ~(1 << PD6);
		PORTD &= ~(1 << PD7);
		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);
		// ������ 2 �������, ����� ��� �����
		_delay_ms(500);
		PORTB &= ~(1 << PB2);
		PORTB &= ~(1 << PB3);
		PORTD &= ~(1 << PD5);
		PORTD &= ~(1 << PD6);
		PORTD &= ~(1 << PD7);
	}
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void UART_Send(char data) {
	while(!( UCSRA & (1 << UDRE)));   // ������� ����� ��������� ����� ��������
	UDR = data; // �������� ������ � �����, �������� ��������
}

// ������� �������� ������ �� USART
void STR_UART_Send(char *string) {
	while(*string != '\0') {
		UART_Send(*string);
		string++;
	}
}




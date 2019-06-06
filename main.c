/*
* Battery_Indicator.c
*
* Created: 21.05.2019 13:14:24
* Author : Denis Denk
*/

#include "main.h"

/*
// Обработчик прерывания кнопки
ISR(INT0_vect) {
	cli();
	if (GIFR & (1<<INTF0)) {
	}
	sei();
}

// Обработчик прерывания таймера0
ISR(TIMER0_OVF_vect) {
}
*/


int main(void) {
	// Настраиваем всю перефирию
	PORT_Init();
	ADC_Init();
	// PWM_Init();
	USART_Init();
	// TIM_Init();
	// INT_Init();
	sei();
	
	while(1) {
		// Нажали кнопку, считаем длительность
		while(bit_is_clear(PIND, 2)) {
			_delay_ms(50);
			counter++;
		}
		// Если длительное нажатие
		if(counter > 15) {
			// Кнопка была долго нажата, выставляем флаг
			STR_UART_Send("Long press!\r\n");
			longPush = true;
			counter = 0;
		}
		// Если короткое нажатие
		if(counter > 0 && counter < 5) {
			// Если перед ним было длинное - подключаем/отключаем питание
			if(longPush) {
				// Если работало - отключаем и наоборот
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
				// Сбрасываем флаг длинного нажатия
				longPush = false;
			}
			// Однократное, короткое нажатие, показываем напряжение
			else {
				//STR_UART_Send("Short press!\r\n");
				counter = 0;
				ShowVoltage();
			}
		}
	}
}

void PORT_Init(void) {
	// PD0 и PD1 - выводы для USART
	// PB1 - выход с ШИМ
	DDRB |= (1 << PB1);

	DDRD |= (1 << PD5) | (1 << PD6)| (1 << PD7); // Красный - Желтый - Желтый 
	DDRB |= (1 << PB2) | (1 << PB3); // Зеленый - Зеленый
	
	// Настраиваем прерывания по нажатию кнопки PD2 (INT0)
	// Порт подтянут к питанию, кнопка замыкает вывод на Землю.
	DDRD &= ~(1 << PD2);
	PORTD |= (1 << PD2);	
}

void INT_Init(void) {
	// Разрешаем прерывание по INT0
	GICR |= (1 << INT0);
	// Прерывание по ниспадающему фронту сигнала на INT0
	MCUCR |= (1 << ISC01);
}

void USART_Init() {
	// Параметры соединения: 8 бит данные, 1 стоповый бит, нет контроля четности
	// USART Приемник: Включен
	// USART Передатчик: Включен
	// USART Режим: Асинхронный
	UBRRL = (F_CPU/BAUDRATE/16-1); // Вычисляем скорость обмена данными
	UBRRH = (F_CPU/BAUDRATE/16-1) >> 8;
	UCSRB |= (1 << RXCIE)| // Разрешаем прерывание по завершению приема данных
	(1 << RXEN)|(1 << TXEN); // Включаем приемник и передатчик
	UCSRC |= (1 << URSEL)| // Для доступа к регистру UCSRC выставляем бит URSEL
	(1 << UCSZ1)|(1 << UCSZ0); // Размер посылки в кадре 8 бит
}

void ADC_Init(void) {
	// Включение АЦП// предделитель преобразователя на 8
	ADCSRA |= (1 << ADEN) | (1 << ADPS1)|(1 << ADPS0);
	// внешний ИОН // вход PC0
	ADMUX |= (0 << REFS1) | (0 << REFS0) |(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3);
}

void PWM_Init(void) {
	// На выводе OC1A единица, когда OCR1A==TCNT1, восьмибитный ШИМ
	TCCR1A = (1 << COM1A1) | (1 << WGM10);
	// Делитель = / 1
	TCCR1B = (1 << CS10);
	// Начальное значение, вообще сюда пишем значение от 0 до 255 для изменнеия скважности
	OCR1A=0x00;
}

void TIM_Init(void) {
	// Предделитель = 1024
	TCCR0 |= (1 << CS02) | (1 << CS00);
	//Разрешить прерывание по переполнению таймера 0
	TIMSK |= (1 << TOIE0);
	//Разрешить прерывания
	sei();
}

void ShowVoltage(void) {
	/* Получение данных с АЦП */
	uint16_t ADCdata[5] = {0};
	uint16_t ADC_ExpVal = 0;
	// Считаем напряжение 4 раза и считаем мат ожидание для точности
	for(uint8_t i = 0; i < 4; i++) {
		// Начинаем преобразование
		ADCSRA |= (1 << ADSC);
		// Ждем флага окончания преобразования
		while ((ADCSRA&(1 << ADIF))== 0);
		// Считываем  ADC
		ADCdata[i] = (ADCL|ADCH << 8);
	}
	ADC_ExpVal = (ADCdata[0] + ADCdata[1] + ADCdata[2] + ADCdata[3]) / 4;
	// Опорное напряжение 5В, значение регистра до 1024 => коэффициент = 5/1024=0.0048828
	float voltage = ADC_ExpVal * 0.0048828;
	// Приводим к виду 30В.
	voltage = map(voltage, 0, 5, 0, 25.2);
	// Отправим значения в UART для отладки
	char buf[50];
	sprintf(buf, "ADC Data: %d\tVoltage: %f\r\n", ADC_ExpVal, voltage);
	STR_UART_Send(buf);

	/* 100% заряженный аккумулятор = 25,2В.
	* 0% = 19,8В.
	* Разница 5,4В. => 20% это 1.08В.
	*/
	
	/*
		DDRD |= (1 << PD5) | (1 << PD6)| (1 << PD7); // Красный - Желтый - Желтый
		DDRB |= (1 << PB2) | (1 << PB3) // Зеленый - Зеленый
	*/

	// 100%
	if(voltage >= 24.12) {
		PORTD |= (1 << PD5) | (1 << PD6)| (1 << PD7);
		PORTB |= (1 << PB2) | (1 << PB3);
		// светим 2 секунды, потом все тушим
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
		// светим 2 секунды, потом все тушим
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
		// светим 2 секунды, потом все тушим
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
		// светим 2 секунды, потом все тушим
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
		// светим 2 секунды, потом все тушим
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
	while(!( UCSRA & (1 << UDRE)));   // Ожидаем когда очистится буфер передачи
	UDR = data; // Помещаем данные в буфер, начинаем передачу
}

// Функция передачи строки по USART
void STR_UART_Send(char *string) {
	while(*string != '\0') {
		UART_Send(*string);
		string++;
	}
}




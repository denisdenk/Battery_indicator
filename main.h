/*
 * main.h
 *
 *  Created on: Mar 14, 2018
 *      Author: Denis Denk
 *      Mail: denisdenk@live.ru
 */

#ifndef INC_MAIN_H_
#define INC_MAIN_H_

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "string.h"
#include "stdio.h"
#include "stdbool.h"

#define BAUDRATE 1200L  // Скорость обмена данными по UART
#define PWM_Pin PB1	// Пин для одключения Mosfet

uint8_t counter = 0;
uint8_t timer = 0;
uint8_t i = 0;

bool longPush = false;
bool run = false;


void PORT_Init(void);
void INT_Init(void);
void USART_Init();
void ADC_Init(void);
void PWM_Init(void);
void TIM_Init(void);
void ShowVoltage(void);
void UART_Send(char data);
void STR_UART_Send(char *string);
float map(float x, float in_min, float in_max, float out_min, float out_max);


#endif /* INC_MAIN_H_ */

/*
 * PWM1.h
 *
 * Created: 12/04/2024 11:45:48 a. m.
 *  Author: josei
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

#ifndef PWM1_H_
#define PWM1_H_

#include <avr/io.h>
#include <stdint.h>

#define invertido 1
#define no_invertido 0

//Función para configurar PWM1 Modo Fast, Canal A
void initPWM1FastA(uint8_t inverted, uint16_t prescaler);

//Función para configurar PWM1 Modo Fast, Canal B
void initPWM1FastB(uint8_t inverted, uint16_t prescaler);

void updateDutyCycleA(uint8_t duty);

void updateDutyCycleB(uint8_t duty);

#endif /* PWM1_H_ */
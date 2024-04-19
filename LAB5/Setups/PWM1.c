/*
 * PWM1.c
 *
 * Created: 18/04/2024 09:14:13 a. m.
 *  Author: josei
 */ 

#include "PWM1.h"


void initPWM1FastA(uint8_t inverted, uint16_t prescaler){
	DDRB |= (1<<DDB1);		//PIN B1 COMO SALIDA
	//TIMER 1
	TCCR1A = 0;							//RESET
	TCCR1B = 0;							//RESET
	if(inverted){
		TCCR1A |= (1<<COM1A1)|(1<<COM1A0);	//OC1A INVERTIDO
	}else{
		TCCR1A |= (1<<COM1A1);				//OC1A NO INVERTIDO	
	}
	
	TCCR1A |= (1<<WGM10);				//MODO PWM FAST, 8 bits
	TCCR1B |= (1<<WGM12);				//MODO PWM FAST, 8 bits
	if(prescaler==1024){
	TCCR1B |= (1<<CS12)|(1<<CS10);		//PRESCALER DE 1024	
	}
	//TCCR1B |= (1<<CS12)|(1<<CS10);		//PRESCALER DE 1024
	
}

void initPWM1FastB(uint8_t inverted, uint16_t prescaler){
	DDRB |= (1<<DDB2);		//PIN B2 COMO SALIDA
	//TIMER 1
	TCCR1A = 0;							//RESET
	TCCR1B = 0;							//RESET
	if(inverted){
		TCCR1A |= (1<<COM1B1)|(1<<COM1B0);	//OC1A INVERTIDO
	}else{
		TCCR1A |= (1<<COM1B1);				//OC1A NO INVERTIDO
		
	}
	
	TCCR1A |= (1<<WGM10);				//MODO PWM FAST, 8 bits
	TCCR1B |= (1<<WGM12);				//MODO PWM FAST, 8 bits
	if(prescaler==1024){
		TCCR1B |= (1<<CS12)|(1<<CS10);		//PRESCALER DE 1024
	}
	//TCCR1B |= (1<<CS12)|(1<<CS10);		//PRESCALER DE 1024
}

void updateDutyCycleA(uint8_t duty){
	OCR0A = duty;
	
}
void updateDutyCycleB(uint8_t duty){
	OCR0B = duty;
	
}
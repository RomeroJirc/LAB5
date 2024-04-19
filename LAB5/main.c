/*
 * LAB5.c
 *
 * Created: 12/04/2024 05:39:43 a. m.
 * Author : josei
 */ 


//LIBRERIAS
#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "Setups/PWM1.h"

//VARIABLES
uint8_t ADCvalue1 = 0;
uint8_t ADCvalue2 = 0;
uint8_t ADCvalue3 = 0;
uint8_t suich = 0;
uint8_t periodo = 0;

//PROTOTIPOS
void initADC(void);
void initTimer0(void);

int main(void)
{
	cli();
	initADC();
	initPWM1FastA(no_invertido, 1024);
	DDRB |= (1<<DDB3);		//PIN B3 COMO SALIDA
	DDRD |=(1<<DDD4);		//PIN D4 COMO SALIDA
	
	//TIMER 2
	TCCR2A = 0;										//RESET
	TCCR2B = 0;										//RESET
	TCCR2A |= (1<<COM2A1);							//OC1A NO INVERTIDO
	TCCR2A |= (1<<WGM21)|(1<<WGM20);				//MODO PWM FAST, 8 bits
	TCCR2B |= (1<<CS20)|(1<<CS21)|(1<<CS22);		//PRESCALER DE 1024
	
	initTimer0();									//LLAMAR A LA INICIALIZACIÓN DEL TIMER1
										//LLAMAR A LA INICIALIZACIÓN DEL ADC
	
	sei();
	
    while (1) 
    {
	periodo++;
	OCR1A = ADCvalue1/6;
	OCR2A = ADCvalue2/6;
	if (periodo==0)
	{
		PORTD &= ~(1<<PORTD4);
	}
	else if (periodo == (ADCvalue3))
	{
		PORTD |= (1<<PORTD4);
	}
}
}

void initADC(void){
	ADMUX = 0;
	ADMUX |= (1<<ADLAR);								//JUSTIFICACIÓN A LA IZQUIERDA
	ADMUX |= (1<<REFS0);								//REFERENCIA AVCC = 5V
	DIDR0 |= (1<<ADC0D);								//DESHABILITAR PIN DIGITAL
	ADMUX |= ((1<<MUX2) | (1<<MUX1));					//AL COLOCAR 0110 INDICAMOS EL ADC6 COMO EL RECEPTOR DE LA SEÑAL ANALÓGICA
	
	ADCSRA = 0;
	ADCSRA |= (1<<ADIE);								//HABILITaR INTERRUPCIONES DE ADC
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);		//PRESCALER DE 128 ----> 125kHz
	ADCSRA |= (1<<ADEN);								//HABILITANDO EL ADC
	
}

void initTimer0(void){
	TCCR0B |= (1<<CS02)|(1<<CS00);;				//CONFIGURAR PRESCALER DE 1024
	TCNT0 = 100;								//CARGAR VALOR DE DESBORDAMIENTO
	TIMSK0 |= (1 << TOIE0);
}

ISR (TIMER0_OVF_vect){
		if(suich==0)
		{
		suich=1;
		ADMUX &= ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2));
		ADMUX |= ((1<<MUX1)|(1<<MUX2));
		}
		else if (suich==1)
		{
		suich=2;
		ADMUX &= ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2));
		ADMUX |= ((1<<MUX0)|(1<<MUX1)|(1<<MUX2));
		}
		else if (suich==2)
		{
		suich =0;
		ADMUX &= ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2));
		}

		
	ADCSRA |= (1<<ADSC);				//ADC START CONVERSION
	TCNT0 = 100;						//CARGAR VALOR DE DESBORDAMIENTO
	TIFR0 |= (1 << TOV0);				//APAGAR BANDERA DEL TIMER1			
}

ISR (ADC_vect){
	if(suich==0)
	{
		ADCvalue1 = ADCH;					//ACTUALIZAR VALOR DEL ADC
	}
	else if (suich==1)
	{
		ADCvalue2 = ADCH;					//ACTUALIZAR VALOR DEL ADC
	}
	else if (suich==2)
		{
		ADCvalue3 = ADCH;
		}
	ADCSRA |= (1<<ADIF);				//APAGAR BANDERA DE INTERRUPCIÓN
}
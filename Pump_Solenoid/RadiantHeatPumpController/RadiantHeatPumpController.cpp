/*
 * RadiantHeatPumpController.cpp
 *
 * Created: 11/21/2014 9:59:20 PM
 *  Author: Jacob
 
 PUMP : PA7
 Solenoid : PA3
 
 */ 

#define F_CPU 20000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define Default_Ramp_Time 5000

#define USI_SS 2 //PA2
#define USI_DO 5 //PA5
#define USI_DI 6 //PA6
#define USI_CLK 4 //PA4


#define Command_Emegency_OFF 0
#define Command_OFF 10
#define Command_High 255
#define Command_Low 13
#define Command_Medium 127
#define Command_CustomSpeed 150 // Only command that requires two bytes, next byte from USI is the desired speed

#define PUMP_PrimeSpeed 210
#define PUMP_MaxSpeed 254
#define PUMP_IdleSpeed 180
#define PUMP_Off 0
#define PumpSpeed(speed) (OCR0B = (speed))

static char USI_Incomming_Command = 1;

void QuickDelay(uint8_t delayTime)
{	
	for(int i = 0; i < delayTime; i++)
	{
		_delay_ms(1);
	}	
}



void SlowPumpRampUp(uint16_t rampTime_ms, uint8_t rampUpToSpeed)
{
	uint8_t startingSpeed = OCR0B;
	uint8_t rampLoopCount = rampUpToSpeed - startingSpeed;
	uint8_t rampLoopTime = rampTime_ms / rampLoopCount;
	
	for(int i = 0; i < rampLoopCount; i++)
	{
		QuickDelay(rampLoopTime);
		if(OCR0B < rampUpToSpeed)
		{
			OCR0B += 1;
		}
	}	
}

void SlowPumpRampDown(uint16_t rampTime_ms, uint8_t rampDownToSpeed)
{
	uint8_t startingSpeed = OCR0B;
	uint8_t rampLoopCount = startingSpeed - rampDownToSpeed;
	uint8_t rampLoopTime = rampTime_ms / rampLoopCount;
	
	for(int i = 0; i < rampLoopCount; i++)
	{
		QuickDelay(rampLoopTime);
		if(OCR0B > rampDownToSpeed)
		{
			OCR0B -= 1;
		}
	}	
}

void SlowPumpRamp(uint16_t rampTime_ms, uint8_t rampSpeed)
{
	uint8_t currentSpeed = OCR0B;
	if((currentSpeed - rampSpeed) > 0)
	{
		SlowPumpRampUp(rampTime_ms, rampSpeed);
	}
	else
	{
		SlowPumpRampDown(rampTime_ms, rampSpeed);
	}
	
}

void ConfigurePumpPWM_Timer0_OCR0B()
{
	DDRA |= (1<<7); // PUMP
	TCCR0A |= (1<<COM0B1) | (1<<WGM00);
	TCCR0B |= (1<<CS02) | (1<<CS00);
	OCR0B = 0;
	PORTA |= (1<<7);
}

void ConfigureSPI()
{
	DDRA |= (1<<USI_DO);
	DDRA &= ~(1<<USI_DI) | ~(1<<USI_CLK);
	
	PORTA |= (1<<USI_DI) | (1<<USI_CLK); // Enable Pull-Ups on incomming Lines	
	USICR |= (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1); // Three wire mode (SPI)
	USISR = _BV(USIOIF); // Make sure the Overflow Flag is clear
	sei(); // Enable Global Interupts (So we start listening for data)
}


int main(void)
{
	DDRB = 0;
	DDRA = 0;
	PORTB = 0;
	PORTA = 0;
	
	// Configuring Solenoid as OUTPUT and making sure its off
	DDRA |= (1<<3);
	PORTA &= ~(1<<3);
	

	//Configure PWM for Pump control output OC0B
	ConfigurePumpPWM_Timer0_OCR0B();
	

	
	
    for(;;)
    {
		switch(USI_Incomming_Command)
		{
			case Command_Emegency_OFF:
				OCR0B = 0;
				//Solenoid OFF			
			break;
			case Command_OFF:			
				SlowPumpRamp(Default_Ramp_Time, PUMP_Off);
				//Close Solenoid			
			break;
			case Command_High:
				//Make sure solenoid is open... Maybe move this to the rampUp function
				//Maybe open Solenoid part way until pump is up to speed
				SlowPumpRamp(Default_Ramp_Time, PUMP_MaxSpeed);
			break;
			case Command_Medium:
				//Make sure solenoid is open... Maybe move this to the rampUp function
				//Maybe open Solenoid part way until pump is up to speed
				SlowPumpRamp(Default_Ramp_Time, PUMP_PrimeSpeed);
			
			break;
			case Command_Low:
				//Make sure solenoid is open... Maybe move this to the rampUp function
				//Maybe open Solenoid part way until pump is up to speed
				SlowPumpRamp(Default_Ramp_Time, PUMP_IdleSpeed);
			break;
			case Command_CustomSpeed:
			
			break;
			
		}
		_delay_ms(5000);		
		SlowPumpRampUp(5000,PUMP_MaxSpeed);
		_delay_ms(2500);
		SlowPumpRampDown(5000,PUMP_Off);
		
		// If pump is above a certain speed
		
		// Open Solenoid
		
		

    }
}


ISR(USI_OVF_vect)
{
	
}


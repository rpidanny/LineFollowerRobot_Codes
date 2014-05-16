/*
 * PIDlinefollower.c
 *
 * Created: 12/30/2012 2:47:29 PM
 *  Author: Danny
 */ 

#define F_CPU 12000000UL

#define kp 1 //0.09 //1/20
#define kd 0.9 //3/2
#define ki 1/10000

//#include "lcdm.h"
#include <avr/io.h>
#include <util/delay.h>

int l1=0,l2=0,c=0,r1=0,r2=0,n=0;
int position=0,error=0,previouserror=0,correction=0,derivative=0;
int integral=0;
const int maxspeed=255;

	int rightflag=0;
	int leftflag=0;




void turn_left()
{
	OCR0=255;
	OCR2=255;
	//PORTB=0b00001001;
	PORTC=0b00001001;
	_delay_ms(550);    //1550 for 3.3 V
}

void turn_right()
{
	OCR0=255;
	OCR2=255;
//	PORTB=0b00000110;
	PORTC=0b00000110;
	_delay_ms(550);
}



void inicialize()
{

	TCCR0 = 0b01100011;
	TCCR2 = 0b01100100;
	

	DDRA=0x00;// PORTA = 0;			// input port
	DDRC=0xff; PORTC=0b00000101;	//motor port
	
	DDRB=0xff;						// PWM0 port
	DDRD=0xff;						//pwm2 port
	
}


int pos()
{
	l2=bit_is_set(PINA,0) >> 0;
	l1=bit_is_set(PINA,1) >> 1;
	c=bit_is_set(PINA,2)  >> 2;
	r2=bit_is_set(PINA,3) >> 3;
	r1=bit_is_set(PINA,4) >> 4;
	
		
	n=l2+l1+c+r2+r1;
	if (n==0)
	{
	//	previouserror=0;
		//PORTD|=(1<PIND1);
		set_motor(0,0);
		//return 4500;
		
	}
	else
	{
		//PORTD|=(0<PIND1);
		return ((2*l2 + 3*l1 + 4*c + 5*r2 + 6*r1)*1000)/n;
	}	
}


int PIDcontrol()
{
	position=pos();
	
	error=position- 4000;
		
	derivative=error-previouserror;
	
	if (integral==30000 || integral==(-30000) )
	{
		integral=0;
	}
		
	integral+=error;
		
	previouserror=error;
		
	//return ((error*kp)+(derivative*kd));
	return ((error*kp)+(integral*ki)+(derivative*kd));
}


void set_motor(int a, int b)
{
	OCR0=a;
	OCR2=b;
}



int main(void)
{
	inicialize();
	
    while(1)
    {
		
			if ((PINA&0b00100000)==0b00100000 && (PINA&0b01000000)==0b01000000 )
			{
				//PORTC=0xff;
				rightflag=0;
				leftflag=0;
				
				//_delay_ms(100);
			//	break;
			}
			else if((PINA&0b00100000)==0b00100000)
			{
				rightflag=1;
				//PORTC=0x10;
				leftflag=0;
			}
			else if ((PINA&0b01000000)==0b01000000)
			{
				leftflag=1;
				//PORTC=0x01;
				rightflag=0;
			}
			
		correction=PIDcontrol();
		
		if (correction>maxspeed)
		{
			correction=maxspeed;
		}
		if (correction<-maxspeed)
		{
			correction= -maxspeed;
		}
		
		
		if (correction<0)
		{
			set_motor(maxspeed+correction,maxspeed);
		}
		else
		{
			set_motor(maxspeed,maxspeed-correction);
		}
		
	
	
	
	if(l1 && l2 && c && leftflag)  //r2
	{
		turn_left();   //turn left
		leftflag=0;
	}
	else if (r1 && r2 && c && rightflag) //l1
	{
		turn_right();  //turn right
		rightflag=0;
	}
	 
   }
	
		
    
}
/*
 * Linefollower.c
 *
 * Created: 12/16/2012 3:47:05 PM
 *  Author: Abhishek
 */ 

#define F_CPU 12000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int count=0;
int pos,no;

	int l1,l2,c,r1,r2;
	int previouspos=0;

int rt,lt;
/*
void right(int a)
{
	OCR0=a;OCR2=255;
	PORTC=0b00001010;	
	//PORTB=0b00001110;
	//PORTC=0b00001110;
}

void left(int a)
{
	OCR0=255;OCR2=a;
	PORTC=0b00001010;	
	//PORTB=0b00001011;
	//PORTC=0b00001011;
} */

void backward(int a,int b)
{
	OCR0=a;OCR2=b;
	//PORTB=0b00000101;
	PORTC=0b00001010;	
} 

void forward(int a,int b)
{
	OCR0=a;OCR2=b;
	//PORTB=0b00001010;	
	//PORTC=0b00000101;

}
/*
ISR(TIMER1_OVF_vect)
{
	
	count++;
	
	if (count==14)
	{
		PORTB ^= (1<<PINB4);
		PORTB ^=(1<<PINB3);
		count=0;
	}
	
}

*/
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(void)
{

	TCCR0 = 0b01100011;
	TCCR2 = 0b01100100;
	
	DDRB=0xff;						// PWM0 port
	DDRA=0x00; // PORTA =0x00;		// input port
	DDRC=0xff;			PORTC=0b00000101;			//motor port
	DDRD=0xff;						//pwm2 port
	


    while(1)
    {
		//PORTD=PINA;
		
		l2=bit_is_set(PINA,0);
		l1=bit_is_set(PINA,1) >> 1;
		c=bit_is_set(PINA,2) >> 2;
		r2=bit_is_set(PINA,3) >> 3;
		r1=bit_is_set(PINA,4) >> 4;
no=l2+l1+c+r2+r1;

		 if(no!=0)
		{
			pos=((2*l2 + 3*l1 + 4*c + 5*r2 + 6*r1)*10)/no;
		}
		else
		{
				pos=70;
		} 
 		
		 PORTB=pos;
		if(pos==40)
		{
				//PORTB=0x00;
			
			forward(255,255);
			previouspos=1;
		}
		
	
	
		else if(pos>40 && pos<<60)
		{
			rt=map((pos-40),20,0,0,100);
			forward(rt,255);
			previouspos=2;
		}
		
		
		else if(pos<40 && pos!=0)
		{
			lt=map((40-pos),20,0,0,100);
			forward(255,lt);
			previouspos=3;
		}
		
		else
		{
			 if (previouspos==0)
			{
				forward(255,255);
			} 
			else if (previouspos=1)
			{
				//backward(255,255);
				forward(0,0);
			} 
			if (previouspos=2)
			{
				forward(255,rt);
			}
			else if (previouspos=3)
			{
				forward(lt,255);
			}
		}
       
    }
}

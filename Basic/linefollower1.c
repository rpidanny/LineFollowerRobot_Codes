/*
 * Linefollower.c
 *
 * Created: 12/16/2012 3:47:05 PM
 *  Author: Abhishek
 */ 

#define F_CPU 12000000UL
#include <avr/io.h>
#include <util/delay.h>


int l1,l2,c,r1,r2,rb,lb;
int previouspos=0;
	
	int rightflag=0;
	int leftflag=0;
	
int no;	
int count=0;
double del=100;
int pos;

void right()
{
		
	PORTB=0b00001110;
	//PORTC=0b00001110;
}

void left()
{
	PORTB=0b00001011;
	//PORTC=0b00001011;
} 

void forward()
{
	PORTB=0b00001010;	
	//PORTC=0b00001010;

}

void backward()
{
	PORTB=0b00000101;	
	//PORTC=0b00000101;

}

void turn_left()
{
	
	PORTB=0b00001001;
	//PORTC=0b00001001;
	_delay_ms(1500);    //1550 for 3.3 V
}

void turn_right()
{
	PORTB=0b00000110;
	//PORTC=0b00000110;
	_delay_ms(1500);
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(void)
{
	
	
	DDRB=0xff;						// motor port
	DDRD=0x00;						//INPUT port
	DDRC=0xff;						//led port

    while(1)
    {
		
		
			if ((PIND&0b00100000)==0b00100000 && (PIND&0b01000000)==0b01000000 )
			{		
				rightflag=0;
				leftflag=0;
			}
			else if((PIND&0b00100000)==0b00100000)
			{
				rightflag=1;
				
				leftflag=0;
				PORTC=0xff;
				_delay_ms(1);
				PORTC=0x00;
			}
			else if ((PIND&0b01000000)==0b01000000)
			{
				leftflag=1;
				rightflag=0;
				PORTC=0xff;
				_delay_ms(1);
				PORTC=0x00;
			}
    
			
		
 		
	
		l2=bit_is_set(PIND,0);
		l1=bit_is_set(PIND,1) >> 1;
		c=bit_is_set(PIND,2) >> 2;
		r2=bit_is_set(PIND,3) >> 3;
		r1=bit_is_set(PIND,4) >> 4;
		
		/*rb=bit_is_set(PIND,5) >> 5;
		lb=bit_is_set(PIND,6) >> 6;
		*/
		
		 no=l2+l1+c+r2+r1;
 		
		 if(no!=0)
		{
			pos=((2*l2 + 3*l1 + 4*c + 5*r2 + 6*r1)*10)/no;
		}
		else
		{
				pos=70;
		} 		
	
	
		if(pos==40)
		{
			
			forward();
			previouspos=1;
		}
		
	
			
		else if(pos>40 && pos<=60)
		{
			right();
			previouspos=2;
		}
		
		
		else if(pos<40 && pos!=0)
		{
			left();
			previouspos=3;
		}
		
		else
		{
			 if (previouspos==0)
			{
				forward();
			} 
			else if (previouspos==1)
			{
				backward();
			} 
			else if (previouspos==2)
			{
				left();
			}
			else if (previouspos==3)
			{
				right();
			}
		}
		
		
	
	if(l1 && l2 && c && r2 && leftflag)
	{
		turn_left();   //turn left
		leftflag=0;
		PORTC=0xff;
				_delay_ms(10);
				PORTC=0x00;
	}
	else if (r1 && r2 && c && l1 && rightflag)
	{
		turn_right();  //turn right
		rightflag=0;
		PORTC=0xff;
				_delay_ms(10);
				PORTC=0x00;
	}
	 
   }
}



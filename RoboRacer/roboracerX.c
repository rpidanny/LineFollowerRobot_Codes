/*
 * Created: 10/3/2013 2:04:17 PM
 *  Author: Danny
 */ 

#define F_CPU 12000000UL
#define LCD_DATA PORTB		
#define ctrl PORTD
#define en PD2		
#define rw PD1		
#define rs PD0

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

int sensor[8]={0,0,0,0,0,0,0,0};
unsigned long int total=0,pos=0,weight=0;

uint16_t result;
int count=0;


int l2=0,l1=0,c2=0,c1=0,r1=0,r2=0;



void LCD_cmd(unsigned char cmd);
void LCDGotoXY(int x,int y);
void init_LCD(void);
void LCD_write(unsigned char data);
void writeChar(int x,int y,int n);

void lcd_string_write(int x,int y, unsigned char *string);
void clearlcd();
void shift(char);
void adc_init();
int map(int x, int in_min, int in_max, int out_min, int out_max);
uint16_t adc_con(uint8_t ch);
void LCDWriteInt(int x,int y,int val,unsigned int field_length);
void detectline();
void minimum();
void turnLeft();
void turnRight();
void forward();
void backward();
void stop();
void manual();

int main()
{
	DDRB=0xff;	
	DDRC=0xff;	
	DDRD=0b11110111;		
	init_LCD();		
	_delay_ms(50);	
	
	clearlcd();
	lcd_string_write(0,0,"The Mighty Zeus!");
	lcd_string_write(0,1," DannyTech");
	_delay_ms(1000);
	for (int i=0;i<16;i++)
	{
		shift('r');
		_delay_ms(400);
	}
	clearlcd();

	lcd_string_write(10,0,"Pos:");
	
	LCDWriteInt(0,0,1,1);
	LCDWriteInt(1,0,2,1);
	LCDWriteInt(2,0,3,1);
	LCDWriteInt(3,0,4,1);
	LCDWriteInt(4,0,5,1);
	LCDWriteInt(5,0,6,1);
	LCDWriteInt(6,0,7,1);
	LCDWriteInt(7,0,8,1);
	
	TCCR1A=(1<<COM1A1) | (1<<WGM10)|(1<<COM1B1); 
	TCCR1B=(1<<WGM12)|(1<<CS12)|(1<<CS10);  
	OCR1A=0;
	OCR1B=0;

	TCCR2|=(1<<CS22)|(1<<CS20)|(1<<CS21);
	TIMSK|=(1<<TOIE2);
	TCNT2=0;
	adc_init();
	
    while(1)
	{	
	}
 }
 
 
void LCD_cmd(unsigned char cmd)
{
	LCD_DATA=cmd;
	ctrl =(0<<rs)|(0<<rw)|(1<<en);	
	_delay_us(500);
	ctrl =(0<<rs)|(0<<rw)|(0<<en);	
	_delay_us(500); //50
	return;
}

 void LCDGotoXY(int x,int y)
{
 if(x<40)
 {
  if(y) x|=0b01000000;
  x|=0b10000000;
  LCD_cmd(x);
  }
}

void init_LCD(void)
{
	LCD_cmd(0x38);		
	_delay_ms(1);
 
	LCD_cmd(0x01);		
	_delay_ms(1);
 
	LCD_cmd(0x0E);		
	_delay_ms(1);
 
	LCD_cmd(0x80);		
	_delay_ms(1);
}
 
void LCD_write(unsigned char data)
{
	LCD_DATA= data;
	ctrl = (1<<rs)|(0<<rw)|(1<<en);	
	_delay_us(500);
	ctrl = (1<<rs)|(0<<rw)|(0<<en);	
	_delay_us(500);			//50			 
	//return ;
}
 
 void writeChar(int x,int y,int n)
 {
	LCDGotoXY(x,y);
	LCD_write(n);  
	_delay_us(500); 
 }
 


void lcd_string_write(int x,int y, unsigned char *string)
{
	LCDGotoXY(x,y);
	while (*string)
	LCD_write(*string++);
}

void clearlcd()
{
		LCD_cmd(1);
		LCD_cmd(12);
}

void shift(char s)
{
	if (s=='r')
	{
		LCD_cmd(30);
	}
	else if (s=='l')
	{
		LCD_cmd(24);
	}
}

void adc_init()		
{
	ADCSRA |= 1<<ADPS2;  
	ADMUX = 0b01000000;
	ADCSRA |= 1<<ADIE;
	ADCSRA |= 1<<ADEN;
	sei();
	ADCSRA |= 1<<ADSC;
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void LCDWriteInt(int x,int y,int val,unsigned int field_length)
{
	
LCDGotoXY(x,y);
	char str[5]={0,0,0,0,0};
	int i=4,j=0;
	while(val)
	{
	str[i]=val%10;
	val=val/10;
	i--;
	}
	if(field_length==-1)
		while(str[j]==0) j++;
	else
		j=5-field_length;

	if(val<0) LCD_write('-');
	for(i=j;i<5;i++)
	{
	LCD_write(48+str[i]);
	}
}

ISR(ADC_vect) 
{
	
		uint8_t theLow = ADCL;
		uint16_t theTenBitResult = ADCH<<8 | theLow;
		
		switch (ADMUX)
		{
				case 0b01000000:
					LCDWriteInt(0, 0, theTenBitResult, 4);
					
					writeChar(0,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[0]=1023-theTenBitResult;
					ADMUX = 0b01000001;
				break;
				case 0b01000001:
					LCDWriteInt(5, 0, theTenBitResult, 4);
					
					writeChar(1,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[1]=1023-theTenBitResult;
					ADMUX = 0b01000010;
				break;
				case 0b01000010:
					LCDWriteInt(10, 0, theTenBitResult, 4);
					
					writeChar(2,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[2]=1023-theTenBitResult;
					ADMUX = 0b01000011;
				break;
				case 0b01000011:
					LCDWriteInt(0, 1, theTenBitResult, 4);
					
					writeChar(3,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[3]=1023-theTenBitResult;
					ADMUX = 0b01000100;
				break;
				case 0b01000100:
					LCDWriteInt(5, 1, theTenBitResult, 4);
					
					writeChar(4,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[4]=1023-theTenBitResult;
					ADMUX = 0b01000101;
				break;
				case 0b01000101:
					LCDWriteInt(10, 1, theTenBitResult, 4);
					
					writeChar(5,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[5]=1023-theTenBitResult;
					detectline();				
					ADMUX = 0b01000000;
				break;
				default:
					
				break;
		} 
					
		ADCSRA |= 1<<ADSC;
}

void detectline()
{
	if(sensor[0]<=250){r2=1;} else{ r2=0;}
	if(sensor[1]<=350){r1=1;} else{ r1=0;}
	if(sensor[2]<=350){c2=1;} else{ c2=0;}
	if(sensor[3]<=300){c1=1;} else{ c1=0;}
	if(sensor[4]<=350){l2=1;} else{ l2=0;}
	if(sensor[5]<=70){l1=1;} else{ l1=0;}
		
		weight=l1+l2+c2+c1+c2+r1;
		if (weight!=0)
		{
			total = ( r2*10 + r1*20 + c2*30 + c1*40 + l2*50 + l1*60 );
		
			pos=total/weight;
			
			LCDWriteInt(11,1,pos,2);
			
			if (pos==35)
			{
				forward(255,255);
			}
			else if (pos<35)
			{
				turnLeft();
			}
			else if (pos>35)
			{
				turnRight();
			}
			
		}
		else
		{
			backward(200,200);
			_delay_us(300);
		}
		
	
	
	
}

void minimum()
{
	int min=sensor[0];
	int temp;
	for (int i=1;i<6;i++)
	{
		if(sensor[i]<min)
		min=sensor[i];
	}
	for (int i=0;i<6;i++)
	{
		sensor[i]-=min;
	}
}


void turnLeft()
{
	OCR1B=150;
	OCR1A=150;
	PORTC&=0b11110000;
	PORTC|=0b00001000;
}
void turnRight()
{
	OCR1B=150;
	OCR1A=150;
	PORTC&=0b11110000;
	PORTC|=0b00000010;
}
void forward(int a,int b)
{
	OCR1B=a;
	OCR1A=b;
	PORTC&=0b11110000;
	PORTC|=0b00001010;
}

void backward(int a,int b)
{
	OCR1B=a;
	OCR1A=b;
	PORTC&=0b11110000;
	PORTC|=0b00000101;
	
}

void stop()
{
	OCR1B=0;
	OCR1A=0;
	PORTC&=0b11110000;
	//PORTC=0x00;
}



ISR(TIMER2_OVF_vect)
{

   count++;
   if(count==61)
   {
      //PORTB^=0b01000001; //Invert the Value of PORTC
	 // PORTD^=0b00010000;
	 PORTC^=0b11000000;
      count=0;
   }
}


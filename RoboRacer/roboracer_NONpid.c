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

#define kp 0.4 //0.5 //0.09 //1/20
#define kd 2 //0.5 //3/2
#define ki 1/10000

int sensor[8]={0,0,0,0,0,0,0,0};
unsigned long int total=0,pos=0,weight=0;

uint16_t result;
int count=0;

int error=0,previouserror=0,correction=0,derivative=0;
int integral=0;
const int maxspeed=255;

uint8_t data[9][8]={{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000},
 {0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b11111111},
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b11111111,0b11111111},
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b11111111,0b11111111,0b11111111},
{0b00000000,0b00000000,0b00000000,0b00000000,0b11111111,0b11111111,0b11111111,0b11111111},
{0b00000000,0b00000000,0b00000000,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111},
{0b00000000,0b00000000,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111},
{0b00000000,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111},
{0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111}};

void LCD_cmd(unsigned char cmd);
void LCDGotoXY(int x,int y);
void init_LCD(void);
void LCD_write(unsigned char data);
void writeChar(int x,int y,int n);
void customcharinit();
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
	customcharinit();
	clearlcd();
	//lcd_string_write(0,0,"The Mighty Zeus!");
	//lcd_string_write(0,1," DannyTech");
	//_delay_ms(5000);
	//for (int i=0;i<16;i++)
	//{
	//	shift('r');
	//	_delay_ms(400);
	//}
	clearlcd();

	lcd_string_write(10,0,"Pos:");
	
	/*LCDWriteInt(0,0,1,1);
	LCDWriteInt(1,0,2,1);
	LCDWriteInt(2,0,3,1);
	LCDWriteInt(3,0,4,1);
	LCDWriteInt(4,0,5,1);
	LCDWriteInt(5,0,6,1);
	LCDWriteInt(6,0,7,1);
	LCDWriteInt(7,0,8,1);
	*/
	
	TCCR1A=(1<<COM1A1) | (1<<WGM10)|(1<<COM1B1); //(1<<WGM11)| // pwm channel 1A
	TCCR1B=(1<<WGM12)|(1<<CS12)|(1<<CS10); //(1<<WGM13)| //pwm channel 1B //cs10 = no prescalar //cs12= 1/250 prescalar 
	OCR1A=0;
	OCR1B=0;
	//heartbeat LED
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
	LCD_cmd(0x38);		// initialization of 16X2 LCD in 8bit mode
	_delay_ms(1);
 
	LCD_cmd(0x01);		// clear LCD
	_delay_ms(1);
 
	LCD_cmd(0x0E);		// cursor ON
	_delay_ms(1);
 
	LCD_cmd(0x80);		// ---8 go to first line and --0 is for 0th position
	_delay_ms(1);
	//return;
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
	LCD_write(n);  // Displaying the character created at address 0x64 
	_delay_us(500); //10ms
 }
 
void customcharinit(){
	
	for (int j=0;j<9;j++)
	{
		LCD_cmd(64+j*8);  // Address where customized character is to be stored
		for(int i=0;i<8;i++)
		{
			LCD_write(data[j][i]);		
		}
	}
	
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
	ADCSRA |= 1<<ADPS2;  //adps2
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
					//LCDWriteInt(0, 1, theTenBitResult, 4);
					
					//writeChar(0,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[0]=1023-theTenBitResult;
					ADMUX = 0b01000001;
				break;
				case 0b01000001:
					//LCDWriteInt(5, 1, theTenBitResult, 4);
					
					//writeChar(1,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[1]=1023-theTenBitResult;
					ADMUX = 0b01000010;
				break;
				case 0b01000010:
					//LCDWriteInt(5, 1, theTenBitResult, 4);
					
					//writeChar(2,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[2]=1023-theTenBitResult;
					ADMUX = 0b01000011;
				break;
				case 0b01000011:
					//LCDWriteInt(5, 1, theTenBitResult, 4);
					
					//writeChar(3,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[3]=1023-theTenBitResult;
					ADMUX = 0b01000100;
				break;
				case 0b01000100:
					//LCDWriteInt(5, 1, theTenBitResult, 4);
					
					//writeChar(4,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[4]=1023-theTenBitResult;
					ADMUX = 0b01000101;
				break;
				case 0b01000101:
					//LCDWriteInt(5, 1, theTenBitResult, 4);
					
					//writeChar(5,1,map(1023-theTenBitResult,0,1023,0,7));
					sensor[5]=1023-theTenBitResult;

					ADMUX = 0b01000110;
				break;
				case 0b01000110:
					//LCDWriteInt(5, 1, theTenBitResult, 4);
					
					//writeChar(6,1,map(theTenBitResult,0,1023,0,7));
					sensor[6]=theTenBitResult;
					ADMUX = 0b01000111;
				break;
				case 0b01000111:
					//LCDWriteInt(5, 1, theTenBitResult, 4);
					
					//writeChar(7,1,map(theTenBitResult,0,1023,0,7));
					sensor[7]=theTenBitResult;
						detectline();				
					ADMUX = 0b01000000;
				break;
				default:
					//Default code
				break;
		} 
					
		ADCSRA |= 1<<ADSC;
}

void detectline()
{
	if (sensor[5]>=700 && sensor[4]>=700 && sensor[3]>=700 && sensor[0]>=700 && sensor[1]>=700 && sensor[2]>=700)
	{
		stop();
		_delay_ms(4000);
	}
	else if (sensor[0]>=700 && sensor[1]>=700 && sensor[2]>=700)
	{
		turnLeft();
		_delay_ms(50);
	}
	else if (sensor[5]>=700 && sensor[4]>=700 && sensor[3]>=700)
	{
		turnRight();
		_delay_ms(50);
	}

else
{
	minimum();
	for (int i=0;i<6;i++)
	{
		total+=sensor[i]*(i+1);
		weight+=sensor[i];
	}
	if ((weight/100)!=0)
	{
		pos=((total*1000)/weight);
	
	error=pos- 3500;
		
	
	LCDWriteInt(11,1,pos,3);
	}
	else
	{
		stop();
	}
	
	total=0;
	weight=0;
	
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
	PORTC|=0b00001001;
}
void turnRight()
{
	OCR1B=150;
	OCR1A=150;
	PORTC&=0b11110000;
	PORTC|=0b00000110;
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


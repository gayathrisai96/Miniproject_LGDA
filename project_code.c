//Header files 
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h> 
#include <avr/interrupt.h>

//macro
#define set(PORT,BIT) PORT|=(1<<BIT)
#define clr(PORT,BIT) PORT&=~(1<<BIT)

//global declaration
volatile int flag=0;
volatile unsigned int count=0;
//volatile unsigned int c=0; 
//volatile unsigned int d=0;

//Ultrasonic distance finder
long readUltrasonicDistance(int pin)
{
  set(DDRD,pin); 
  clr(PORTD,pin);
  _delay_ms(2);
  set(PORTD,pin);
  _delay_ms(10);
  clr(PORTD,pin);
  clr(DDRD,pin); 
  return pulseIn(pin, HIGH);
}
 
 
int main()
{
  Serial.begin(9600);
  unsigned int inches = 0;
  unsigned int cm = 0;

  set(DDRB,PB5);   //OUTPUT-Red light for OD 
  set(DDRB,PB4);   //OUTPUT-Blue light for OD 
  set(DDRB,PB3);   //OUTPUT-Green light for OD 
  set(DDRB,PB2);   //OUTPUT-Buzzer for OD 
  set(DDRB,PB1);   //OUTPUT-Red light for TPMS
  set(DDRB,PB0);   //OUTPUT-Blue light for TPMS
  clr(DDRD,PD7);   //INPUT-Ultrasonic sensor
  set(DDRD,PD6);   //OUTPUT-Motor for CAS
  set(DDRD,PD5);   //OUTPUT-Servomotor for AHL 
  clr(PORTD,PD5);
  set(DDRD,PD4);   //OUTPUT-Green light for TPMS
  clr(DDRD,PD2);   //INPUT-Interupt(Ignition Switch) 
  set(DDRD,PD3);   //OUTPUT-Buzzer for TPMS  
  clr(PORTD,PD3);
  
  //Local Interupt
  EICRA|=(1<<ISC00);
  EICRA&=~(1<<ISC01);
  EIMSK|=(1<<INT0);  //Enable of local interupt
 
  //Timer 0 as PWM for AHL
  TCNT0=0x00;
  TCCR0A=0x00;
  TCCR0B|=((1<<CS00)|(1<<CS02));
  TCCR0B&=~((1<<CS01));  
  OCR0B=255;
  
  sei(); //Global interupt enable
  
  while(1)
   {
    if(flag==1)
    { 
      TIMSK0 |= ((1<<OCIE0A)|(1<<OCIE0B));
      //AHL
        //ADC conversion
        ADCSRA|=(1<<ADEN);  //ADC enable
        ADCSRA|=((1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2));
        ADCSRA|=(1<<ADSC);  //starting conversion
        ADMUX&=~((1<<MUX0)|(1<<MUX1)|(1<<MUX2)|(1<<MUX3)|(1<<REFS1)); //selecting the port A0
        ADMUX|=(1<<REFS0);
        Serial.println(ADC);
        Serial.println("ADC1");
  
   if((ADC>=450)&&(ADC<=550))     //pot centre
   {
    lookup(0);  
   }  
   else if ((ADC>=900)&&(ADC<=1024))  //pot left extreme
   {
   lookup(1);
   }
   else if ((ADC>=656)&&(ADC<=899))  //pot left mid
   {
   lookup(2);
   }   
   else if ((ADC>=551)&&(ADC<=655))  //pot left low
   {
   lookup(3);
   }
   else if ((ADC>=0)&&(ADC<=150))  //pot right extreme
   {
   lookup(4);
   }
   else if ((ADC>=151)&&(ADC<=250)) //pot right mid
   {
   lookup(5);
   }
   else if ((ADC>=251)&&(ADC<=449)) //pot right low
   {
   lookup(6);
   }

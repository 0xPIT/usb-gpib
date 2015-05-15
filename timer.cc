/* #------------------------------------------------------------------------#
   |                                                                        |
   |   timer.cc                                                             |
   |                                                                        |
   |   Shared 1ms timer.                                                    |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include "gpib.h"

SharedTimer *volatile SharedTimer::First=0;

inline void TimerInterrupt()
{
   SharedTimer *timer=SharedTimer::First;
   while(timer)
   {  timer->Tick();
      timer=timer->Next;
   }
}

SharedTimer::SharedTimer()
{
   cli();
   if((Next=First)==0)
   {
      TCCR0A=(1<<WGM01)|(1<<WGM00);          // fast pwm mode (but no PWM output)
      TCCR0B=(1<<WGM02)|(1<<CS01)|(1<<CS00); // prescaler=64
      OCR0A=250-1;                           // 250*64=16000 cycles, 1ms
      OCR0B=255;
      TIMSK0=(1<<TOIE0);                     // enable overflow interrupt
   }
   First=this;
   sei();
}

void SharedTimer::Tick()
{ }

ISR(TIMER0_OVF_vect)
{  TimerInterrupt();
}

Timer::Timer()
   :SharedTimer(),Count(0),Reload(0),TimedOut(false)
{
}

void Timer::Tick()
{
   uint16_t count=Count;
   if(count>0 && --count==0)
   {  TimedOut=true;
      Count=Reload;
   }
   else
      Count=count;
}

uint16_t Timer::Read() const
{
   cli();
   uint16_t r=Count;
   sei();
   return r;
}

void Timer::Single(uint16_t time)
{
   cli();
   TimedOut=false;
   Reload=0;
   Count=time;
   sei();
}

void Timer::Repeat(uint16_t time)
{
   cli();
   TimedOut=false;
   Reload=Count=time;
   sei();
}

void Timer::Clear()
{
   cli();
   TimedOut=false;
   Reload=Count=0;
   sei();
}

void Blinky::Tick()
{
   if(--Count==0)
   {  Count=Reload;
      Pattern=(Pattern>>1) | (Pattern<<15);
      GPIB.LED(Pattern&1);
   }
}

Blinky::Blinky()
   :SharedTimer(),Count(0),Reload(0),Pattern(0)
{}

void Blinky::operator()(uint8_t period,uint16_t pattern)
{  Count=Reload=period;
   Pattern=pattern;
}

void Blinky::operator()(uint16_t pattern)
{  Pattern=pattern;
}

Blinky blinky;

/* ----- EOF timer.cc ----- */

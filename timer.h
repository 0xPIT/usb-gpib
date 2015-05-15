/* #------------------------------------------------------------------------#
   |                                                                        |
   |   timer.h                                                              |
   |                                                                        |
   |   Shared 1ms timer.                                                    |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#ifndef TIMER_H_
#define TIMER_H_

void TimerInterrupt();

class SharedTimer
{
   friend void TimerInterrupt();
   static SharedTimer *volatile First;
   SharedTimer *Next;
protected:
   virtual void Tick();
   SharedTimer();
};

class Timer:public SharedTimer
{
   uint16_t Count,
            Reload;
   bool TimedOut;

protected:
   virtual void Tick();

public:
   Timer();
   void Single(uint16_t time);
   void Repeat(uint16_t time);
   void Clear();
   uint16_t Read() const;
   bool Timeout() const
   {  return TimedOut;
   }
};

class Blinky:public SharedTimer
{
   uint8_t Count,
           Reload;
   uint16_t Pattern;

protected:
   virtual void Tick();

public:
   Blinky();
   void operator()(uint8_t period,uint16_t pattern);
   void operator()(uint16_t pattern);
};

extern Blinky blinky;

#endif /* TIMER_H_ */

/* ----- EOF timer.h ----- */


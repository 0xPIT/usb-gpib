/* #------------------------------------------------------------------------#
   |                                                                        |
   |   serial.h                                                             |
   |                                                                        |
   |   Debugging (transmit only) serial port.                               |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "formatting.h"
//#define static_assert(COND) do { typedef char static_assertion[(COND)?1:-1]; } while(0)

class DebugSerial:public Formatting<DebugSerial>
{
public:
   // public methods
   DebugSerial();
   static void Initialize();

   static __attribute__((noinline)) void Send_(char c);
private:
   static void delay();
};

extern const DebugSerial Serial;

#endif /* SERIAL_H_ */

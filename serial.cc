/* #------------------------------------------------------------------------#
   |                                                                        |
   |   serial.cc                                                            |
   |                                                                        |
   |   Debugging (transmit only) serial port.                               |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "serial.h"

#if !defined(SERIAL_PORT) || !defined(SERIAL_TXD)
   #ifdef __AVR_ATtiny85__
      #define SERIAL_PORT PORTB
      #define SERIAL_TXD  PB4
   #elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      #define SERIAL_PORT PORTA
      #define SERIAL_TXD  PA7
   #elif defined(__AVR_ATtiny441__) || defined(__AVR_ATtiny841__)
      // These devices have hardware UART, not implemented here
      #define SERIAL_PORT PORTA
      #define SERIAL_TXD  PA7
      #define SERIAL_RXD  PB2
   #elif defined __AVR_ATmega32U4__
      // These devices have hardware UART but using a spare port pin here
      #define SERIAL_PORT PORTD
      #define SERIAL_TXD  PD5    // near switch
   #endif
#endif

#if !defined(SERIAL_BAUD)
   #define SERIAL_BAUD 38400
#endif

// Note on ABI: function calls  R18-R27, R30, R31 clobbered
//                              R2-R17, R28, R29 saved

inline static void set_high()
{  static const uint8_t port=_SFR_IO_ADDR(SERIAL_PORT);
   static const uint8_t bit=SERIAL_TXD;
   __asm__ __volatile__ ("sbi %0,%1"::"I"(port),"I"(bit):"memory");
}

inline static void set_low()
{  static const uint8_t port=_SFR_IO_ADDR(SERIAL_PORT);
   static const uint8_t bit=SERIAL_TXD;
   __asm__ __volatile__ ("cbi %0,%1"::"I"(port),"I"(bit):"memory");
}

void __attribute__((noinline)) DebugSerial::delay()
{
   // 38400bps -> 26us
   // 19200bps -> 52us
   // 9600bps -> 104us
   // 7=fudge factor for call and loop overhead, below
#if SERIAL_BAUD==38400
   uint8_t count=F_CPU*26/3-(F_CPU>1?8:6);
#elif SERIAL_BAUD==19200
   uint8_t count=F_CPU*52/3-6;
#else // SERIAL_BAUD==9600
   uint8_t count=F_CPU*104/3-7;
#endif
   char dummy;
   __asm__ __volatile__ (
      "1: subi %0,1   \n"
      "   brne 1b     \n"
   #if SERIAL_BAUD==38400 && F_CPU==1
      "   breq 2f     \n"
      "2:             \n"
   #endif
      : "=r"(dummy)
      : "0"(count)
      : "0"
   );
}

void __attribute__((noinline)) DebugSerial::Send_(char byte)
{
   static const uint8_t port=_SFR_IO_ADDR(SERIAL_PORT);
   static const uint8_t bit=SERIAL_TXD;

   cli();

   // start bit
   set_low();

   for(uint8_t i=0;i<9;i++)
   {
      delay();

      __asm__ __volatile__(
      "   asr %0       \n"
      "   brcs 1f      \n"
      "   nop          \n"
      "   cbi %2,%3    \n"
      "   brcc 2f      \n"
      "1: sbi %2,%3    \n"
      "   nop          \n"
      "2: \n"
      :"=r"(byte)
      :"0"(byte),"I"(port),"I"(bit));
   }

   // final data bit (MSB)
   delay();

   // safe to enable interrupts now, no problem if stop bit is a bit longer.
   sei();
   set_high();
   delay();
#ifdef  SERIAL_TWO_STOP_BITS
   // two stop bits
   delay();
#endif
}

DebugSerial::DebugSerial()
{  Initialize();
}

void DebugSerial::Initialize()
{  static const uint8_t port=_SFR_IO_ADDR(SERIAL_PORT)-1;
   static const uint8_t bit=SERIAL_TXD;
   __asm__ __volatile__ ("sbi %0,%1"::"I"(port),"I"(bit));
   set_high();
}

const DebugSerial __attribute__((__progmem__)) Serial;

/* ----- EOF serial.cc ----- */

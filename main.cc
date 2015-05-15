/* #------------------------------------------------------------------------#
   |                                                                        |
   |   main.cc                                                              |
   |                                                                        |
   |   USB-GPIB startup and main loop.                                      |
   |                                                                        |
   |   Based in part on Simple example for Teensy USB Development Board.    |
   |   http://www.pjrc.com/teensy/  Copyright (c) 2008 PJRC.COM, LLC        |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include "usb_serial.h"
#include "serial.h"
#include "gpib.h"

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

uint8_t recv_str(char *buf, uint8_t size);
const UsbSerial __attribute__((__progmem__)) USB;


/* -------------------------------------------------------------------------- */

// Command line state machine
enum State_t { UNCONNECTED, IDLE,
               // source handshake
               SH_START, SH_DATA, SH_WAIT_NRFD, SH_DAV, SH_WAIT_NDAC, SH_N_DAV,
               // acceptor handshake
               AH_START, AH_N_ATN, AH_READY, AH_WAIT_DAV, AH_NRFD, AH_N_NDAC,
               AH_WAIT_NDAV, AH_NDAC, AH_N_NDAC_LAST, AH_WAIT_NDAV_LAST,
               // become active controller
               CAC_START, CAC_WAIT, CAC_COMPLETE,
               // send interface clear
               SIC_START, SIC_WAIT, SIC_COMPLETE,
               // parallel poll
               PP_START, PP_COMPLETE
               // serial poll
             };
State_t State;

void Version();
int16_t Command(char *);
bool Echo=true;

int main(void)
{
   uint8_t n;
   static char buf[100];

   // set for 16 MHz clock, and turn on the LED
   CPU_PRESCALE(0);

   GPIB.Initialize();
   Serial.Initialize();
   blinky(255,1);

   // initialize the USB, and then wait for the host
   // to set configuration.  If the Teensy is powered
   // without a PC connected to the USB port, this
   // will wait forever.
   USB.Initialize();
   while (!usb_configured())
      { }
   _delay_ms(1000);

   while (1)
   {
      blinky(255,1);

      // wait for the user to run their terminal emulator program
      // which sets DTR to indicate it is ready to receive.
      while (!(usb_serial_get_control() & USB_SERIAL_DTR))
         { }

      blinky(100,3);

      // discard anything that was received prior.  Sometimes the
      // operating system or other software will send a modem
      // "AT command", which can still be buffered.
      usb_serial_flush_input();

      // print a nice welcome message
      Version();

      // state
      bool DirectMode=false;

      // and then listen for commands and process them
      while(1)
      {
         //GPIB.Report();
         if(Echo)
            USB<<ROMS("++");

         blinky(5);

         n = recv_str(buf, sizeof(buf));
         if(n == 255)
            break;
         buf[n]=0;

         blinky(7);

         if(n>3 && buf[0]=='+' && buf[1]=='+')
         {
            if(Echo)
               USB<<endl;
            Command(buf+2);
         }
         else if(!DirectMode)
         {
            if(Echo)
               USB<<endl;
            Command(buf);
         }
         else
         {
            // send GPIB
         }
      }
   }
}

// Receive a string from the USB serial port.  The string is stored
// in the buffer and this function will not exceed the buffer size.
// A carriage return or newline completes the string, and is not
// stored into the buffer.
// The return value is the number of characters received, or 255 if
// the virtual serial connection was closed while waiting.
//
uint8_t recv_str(char *buf, uint8_t size)
{
   int16_t r;
   uint8_t count=0;

   while(count < size)
   {
      // TODO: move to main
      r=GPIB.Poll();
      if(r)
         USB<<ROMS("++error ")<<r<<endl;

      r=GPIB.Read();
      if(r)
      {
         r&=0xff;
         if(r==10)
            USB<<ROMS("\\012\r\n");
         else if(r=='\\')
            USB<<ROMS("\\134");
         else
            USB.SendSafeChar(r);
      }

      r = usb_serial_getchar();
      if(r==-1)
      {
         if(!usb_configured() || !(usb_serial_get_control() & USB_SERIAL_DTR))
         {
            // user no longer connected
            return 255;
         }
         // just a normal timeout, keep waiting
      }
      else
      {
         if(r=='\r' || r=='\n')
            return count;
         if(r=='\b' && count>0)
         {  --buf;
            --count;
            if(Echo)
            {  usb_serial_putchar(r);
               usb_serial_putchar(' ');
               usb_serial_putchar(r);
            }
         }
         else if(r >= ' ' && r <= '~')
         {
            *buf++ = r;
            if(Echo)
               usb_serial_putchar(r);
            count++;
         }
      }
   }
   return count;
}

/* ----- EOF main.cc ----- */

/* #------------------------------------------------------------------------#
   |                                                                        |
   |   gpib.h                                                               |
   |                                                                        |
   |   USB-GPIB GPIB processing and state machine.                          |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#ifndef GPIB_H_
#define GPIB_H_

#include <avr/io.h>
#include "timer.h"
#include "formatting.h"

// Simple queue for commands and data read
template<unsigned SIZE> class Queue
{
   uint8_t In, Out;
   uint16_t Data[SIZE];

public:
   static const uint8_t Size=SIZE;

   __attribute__((noinline)) Queue()
   {  Clear();
   }
   __attribute__((noinline)) void Clear()
   {  In=Out=0;
   }

   __attribute__((noinline)) uint8_t Used()
   {
      if(In>=Out)
         return In-Out;
      else
         return In+Size-Out;
   }

   __attribute__((noinline)) uint16_t Peek()
   {
      uint8_t o=Out;
      if(In==o)
         return 0;
      return Data[o];
   }

   __attribute__((noinline)) uint16_t Get()
   {
      uint8_t o=Out;
      if(In==o)
         return 0;
      uint16_t value=Data[o];
      o++;
      if(SIZE<256 && o>=SIZE)
         o=0;
      Out=o;
      return value;
   }
   __attribute__((noinline)) bool Put(uint16_t value)
   {
      uint8_t i=In+1;
      if(SIZE<256 && i>=SIZE)
         i=0;
      if(i==Out)
         return false;
      Data[In]=value;
      In=i;
      return true;
   }
};

class GPIBDriver
{
public:
   // we never actively pull any signals high when this is true
   static const bool SAFER=true;

   static const unsigned PORTB_LED_MP_YELLOW=0;    // Yellow ~LED on Arduino (not used)

   static const unsigned PORTD_EOI=0,
                         PORTD_NRFD=1,
                         PORTD_NDAC=2,
                         PORTD_SRQ=3,
                         PORTD_TEENSY_N_MP=4,      // pulled low on Arduino
                         PORTD_LED_MP_GREEN=5,     // Green ~LED on Arduino, serial on Teensy
                         PORTD_LED_T=6,            // LED on Teensy
                         PORTD_DAV=7;

   // By happy coincidence the bits here are unused on port D
   static const unsigned PORTF_ATN=4,
                         PORTF_IFC=5,
                         PORTF_REN=6;

   // Data pins for MicroPro
   static const unsigned PORTC_DIO8=6;
   static const unsigned PORTE_DIO1=6;

   // Timer interrupt
   static ::Timer Timer;

   // command and read queues
   static const unsigned QUEUE_SIZE=256;
   static Queue<QUEUE_SIZE> CommandQueue;
   static Queue<QUEUE_SIZE> ReadQueue;

   // GPIB state machine
   enum State_t { RESET, IDLE,
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
   static State_t State;
   static uint8_t Byte;    // byte match or counter value

public:
   GPIBDriver();
   static void LED(bool);

   /* ..... public data ........................................................ */

   static uint16_t MyAddress;
   static uint16_t DeviceAddress;
   static uint16_t Timeout;
   static bool MicroPro;

   /* ..... high level commands ................................................ */

   static void Initialize();
   static void Float();
   static void Controller();

   /* ..... polled commands and reads .......................................... */

   // commands
   static const uint16_t CMD_SOURCE       =1<<12,     // source handshake
                            CMD_ATN       =1<<9,      //    assert ATN
                            CMD_EOI       =1<<8,      //    assert EOI
                         CMD_ACCEPT       =2<<12,     // accept handshake
                            CMD_AC_EOI    =0<<8,      //    until eoi
                            CMD_AC_TMO    =1<<8,      //    until timeout (i.e. forever)
                            CMD_AC_COUNT  =2<<8,      //    byte count in LSBs (only support count=1 for now)
                            CMD_AC_BYTE   =3<<8,      //    until byte seen
                            CMD_AC_MASK   =3<<8,      //    until byte seen
                            CMD_AC_QUICK  =1<<11,     //    short timeout/no error on first byte
                         CMD_CONTROLLER   =3<<12,     // take control of bus
                            CMD_CTRL_FORCE=1<<0,      //    without waiting for it to be idle
                         CMD_SIC          =4<<12,     // send interface clear
                         CMD_RELEASE      =5<<12,     // release control of the bus
                         CMD_PARPOLL      =6<<12,     // parallel poll
                         CMD_MASK         =15<<12;    // mask of commands

   // error returns
   static const int8_t OK=0,
                       EINTERN=-9,     // internal error
                       EACPROT=-10,    // acceptor protocol error
                       EACTMO=-11,     // acceptor timeout
                       ESRCPROT=-12,   // source protocl error
                       ESRCTMO=-13,    // source timeout
                       ECTRLTMO=-14,   // controller timeout
                       EFULL=-15;      // queue full

   static int8_t Poll();
   static bool Command(uint16_t);
   static uint16_t Read();

   /* ..... byte level commands ................................................ */

   static void Report(_ROMS msg=_ROMS(0));

   static uint8_t Lines();
   #define set_bit(port_,bit_)                           \
      do                                                 \
      {  static const uint8_t port=_SFR_IO_ADDR(port_);  \
         static const uint8_t bit=bit_;                  \
         __asm__ __volatile__ ("sbi %0,%1"::"I"(port),"I"(bit):"memory"); \
      } while(0)

   #define clear_bit(port_,bit_)                         \
      do                                                 \
      {  static const uint8_t port=_SFR_IO_ADDR(port_);  \
         static const uint8_t bit=bit_;                  \
         __asm__ __volatile__ ("cbi %0,%1"::"I"(port),"I"(bit):"memory"); \
      } while(0)

   #define test_bit_set(port_,bit_)                      \
      ({  static const uint8_t port=_SFR_IO_ADDR(port_); \
          static const uint8_t bit=bit_;                 \
          uint8_t value;                                 \
          __asm__ __volatile__ (                         \
          "  clr %0 \n"                                  \
          "  sbic %1,%2 \n"                              \
          "  inc %0 \n"                                  \
          :"=r"(value)                                   \
          :"I"(port),"I"(bit)                            \
          :"memory");                                    \
          0xff&value;                                    \
      })

   #define test_bit_clear(port_,bit_)                    \
      ({  static const uint8_t port=_SFR_IO_ADDR(port_); \
          static const uint8_t bit=bit_;                 \
          uint8_t value;                                 \
          __asm__ __volatile__ (                         \
          "  clr %0 \n"                                  \
          "  sbis %1,%2 \n"                              \
          "  inc %0 \n"                                  \
          :"=r"(value)                                   \
          :"I"(port),"I"(bit)                            \
          :"memory");                                    \
          0xff&value;                                    \
      })


   /* ..... bit level commands ................................................. */

   static void FloatData();
   static uint8_t Data();
   static void Data(uint8_t byte);

   // Who is allowed to drive the signals
   //
   // Control    Output
   // ---------  ---------
   //   NRFD     Listener
   //   NDAC     Listener
   //   DAV      Talker
   //
   // Control    Output
   // ---------  ---------
   //   ATN      Controller
   //   IFC      Controller
   //   REN      Controller
   //   SRQ      Any
   //   EOI      Talker and Controller

   inline static void ATN(bool value)
   {
      if(!value)
      {  if(SAFER) clear_bit(DDRF,PORTF_ATN);
         set_bit(PORTF,PORTF_ATN);
      }
      else
      {  clear_bit(PORTF,PORTF_ATN);
         if(SAFER) set_bit(DDRF,PORTF_ATN);
      }
   }
   inline static void IFC(bool value)
   {
      if(!value)
      {  if(SAFER) clear_bit(DDRF,PORTF_IFC);
         set_bit(PORTF,PORTF_IFC);
      }
      else
      {  clear_bit(PORTF,PORTF_IFC);
         if(SAFER) set_bit(DDRF,PORTF_IFC);
      }
   }
   inline static void REN(bool value)
   {
      if(!value)
      {  if(SAFER) clear_bit(DDRF,PORTF_REN);
         set_bit(PORTF,PORTF_REN);
      }
      else
      {  clear_bit(PORTF,PORTF_REN);
         if(SAFER) set_bit(DDRF,PORTF_REN);
      }
   }

   // shared pins, must not actively drive high
   inline static void EOI(bool value)
   {
      if(!value)
      {  clear_bit(DDRD,PORTD_EOI);
         set_bit(PORTD,PORTD_EOI);
      }
      else
      {  clear_bit(PORTD,PORTD_EOI);
         set_bit(DDRD,PORTD_EOI);
      }
   }
   inline static void NDAC(bool value)
   {
      if(!value)
      {  clear_bit(DDRD,PORTD_NDAC);
         set_bit(PORTD,PORTD_NDAC);
      }
      else
      {  clear_bit(PORTD,PORTD_NDAC);
         set_bit(DDRD,PORTD_NDAC);
      }
   }
   inline static void NRFD(bool value)
   {
      if(!value)
      {  clear_bit(DDRD,PORTD_NRFD);
         set_bit(PORTD,PORTD_NRFD);
      }
      else
      {  clear_bit(PORTD,PORTD_NRFD);
         set_bit(DDRD,PORTD_NRFD);
      }
   }
   inline static void SRQ(bool value)
   {
      if(!value)
      {  clear_bit(DDRD,PORTD_SRQ);
         set_bit(PORTD,PORTD_SRQ);
      }
      else
      {  clear_bit(PORTD,PORTD_SRQ);
         set_bit(DDRD,PORTD_SRQ);
      }
   }
   inline static void DAV(bool value)
   {
      if(!value)
      {  clear_bit(DDRD,PORTD_DAV);
         set_bit(PORTD,PORTD_DAV);
      }
      else
      {  clear_bit(PORTD,PORTD_DAV);
         set_bit(DDRD,PORTD_DAV);
      }
   }

   inline static uint8_t EOI()
   {
      return test_bit_clear(PIND,PORTD_EOI);
   }
   inline static uint8_t NDAC()
   {
      return test_bit_clear(PIND,PORTD_NDAC);
   }
   inline static uint8_t NRFD()
   {
      return test_bit_clear(PIND,PORTD_NRFD);
   }
   inline static uint8_t SRQ()
   {
      return test_bit_clear(PIND,PORTD_SRQ);
   }
   inline static uint8_t DAV()
   {
      return test_bit_clear(PIND,PORTD_DAV);
   }
   inline static int ATN()
   {
      return test_bit_clear(PINF,PORTF_ATN);
   }
   inline static uint8_t IFC()
   {
      return test_bit_clear(PINF,PORTF_IFC);
   }
   inline static uint8_t REN()
   {
      return test_bit_clear(PINF,PORTF_REN);
   }
};

extern const GPIBDriver GPIB;

#endif /* GPIB_H_ */

/* ----- EOF gpib.h ----- */

/* #------------------------------------------------------------------------#
   |                                                                        |
   |   gpib.cc                                                              |
   |                                                                        |
   |   USB-GPIB GPIB processing and state machine.                          |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <ctype.h>
#include <util/delay.h>
#include "usb_serial.h"
#include "gpib.h"

// GPIB Connector
//
// Pin  Name    Signal Description          Pin  Name    Signal Description
// ---  ------  -------------------------   ---  ------  -------------------------
// 1    DIO1    Data Input/Output Bit 1     13   DIO5    Data Input/Output Bit 5
// 2    DIO2    Data Input/Output Bit 2     14   DIO6    Data Input/Output Bit 6
// 3    DIO3    Data Input/Output Bit 3     15   DIO7    Data Input/Output Bit 7
// 4    DIO4    Data Input/Output Bit 4     16   DIO8    Data Input/Output Bit 8
// 5    EIO     End-Or-Identify             17   REN     Remote Enable
// 6    DAV     Data Valid                  18   GND     Ground DAV
// 7    NRFD    Not Ready For Data          19   GND     Ground NRFD
// 8    NDAC    Not Data Accepted           20   GND     Ground NDAC
// 9    IFC     Interface Clear             21   GND     Ground IFC
// 10   SRQ     Service Request             22   GND     Ground SRQ
// 11   ATN     Attention                   23   GND     Ground ATN
// 12   Shield  Chassis Ground              24   GND     Ground DIO, REN, EOI
//
// Handshake  Output
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
//
// Teensy 2.0 has two complete 8-bit ports PB and PD
// Arduino Leonardo and "Pro Micro" have 15 out of 16, good try!
// 20 mA/pin, 100 mA max for 8-bit port
//
// 32U4       Arduino  GPIB      32U4       Arduino  GPIB
// ---------- -----    ----      ---------- -----    ----
// PB0 (SS)   -      : DIO1      PD0 (INT0) "d3"   : EOI
// PB1 (SCLK) "sck"  : DIO2      PD1 (INT1) "d2"   : NRFD
// PB2 (MOSI) "mosi" : DIO3      PD2 (INT2) "rxi"  : NDAC
// PB3 (MISO) "miso" : DIO4      PD3 (INT3) "txo"  : SRQ
// PB4        "d8"   : DIO5      PD4        "d4"     (pull down for arduino)
// PB5        "d9"   : DIO6      PD5        (LED)    (debug serial)
// PB6        "d10"  : DIO7      PD6 (LED)  -        -
// PB7        -      : DIO8      PD7        "d6"   : DAV
//
// Other pins
//
// 32U4       Arduino  GPIB      32U4       Arduino  GPIB
// ---------- -----    ----      ---------- -----    ----
// PF0 (ADC0) -        -         PC6        "d5"     DIO8
// PF1 (ADC1) -        -         PC7        -
// PF4 (ADC4) "a3"   : ATN       PE6        "d7"     DIO1
// PF5 (ADC5) "a2"   : IFC
// PF6 (ADC6) "a1"   : REN
// PF7 (ADC7) "a0"   :
//
// Teensy has LED on PD6 to GND (shared)
// Arduino has LED on PD5 to VCC
//             and on PB0 to VCC (shared with DIO1)

#undef DEBUG

Queue<GPIBDriver::QUEUE_SIZE> GPIBDriver::CommandQueue;
Queue<GPIBDriver::QUEUE_SIZE> GPIBDriver::ReadQueue;

uint16_t GPIBDriver::MyAddress;
uint16_t GPIBDriver::DeviceAddress;
uint16_t GPIBDriver::Timeout;
uint8_t  GPIBDriver::Byte;
bool     GPIBDriver::MicroPro;
GPIBDriver::State_t GPIBDriver::State;
::Timer  GPIBDriver::Timer;

GPIBDriver::GPIBDriver()
{
   Initialize();
}

void GPIBDriver::Initialize()
{
   Float();
   MyAddress=0;
   DeviceAddress=1;
   Timeout=1000;     // one second
   MicroPro=test_bit_clear(PIND,PORTD_TEENSY_N_MP);
   FloatData();

   CommandQueue.Clear();
   ReadQueue.Clear();
   State=RESET;
}

// float data bus
void GPIBDriver::FloatData()
{
   DDRB=0;
   PORTB=0xff;
   if(MicroPro)
   {  clear_bit(DDRE,6);
      clear_bit(DDRC,6);
   }
}

// float all pins
void GPIBDriver::Float()
{
   // data lines all inputs
   FloatData();

   static const uint8_t used_c=(1<<PORTC_DIO8);
   DDRC&=0xff^used_c;
   PORTC=(PORTC&~used_c)|(0xff&used_c);

   static const uint8_t used_e=(1<<PORTE_DIO1);
   DDRE&=0xff^used_e;
   PORTE=(PORTE&~used_e)|(0xff&used_e);

   // grab all of port D, two outputs
   DDRD=(1<<PORTD_LED_MP_GREEN) |
        (1<<PORTD_LED_T);

   // pull ups, LED off
   PORTD=(1<<PORTD_EOI) |
         (1<<PORTD_NDAC) |
         (1<<PORTD_NRFD) |
         (1<<PORTD_SRQ) |
         (1<<PORTD_TEENSY_N_MP) |
         (1<<PORTD_LED_MP_GREEN) |
         (1<<PORTD_DAV);

   // controls, used indicates bits that are used by us
   static const uint8_t used_f=(1<<PORTF_ATN) | (1<<PORTF_IFC) | (1<<PORTF_REN);
   DDRF&=0xff^used_f;
   PORTF=(PORTF&~used_f)|(0xff&used_f);
}

// drive LED
void GPIBDriver::LED(bool on)
{
   if(MicroPro)
   {  if(on)
         clear_bit(PORTD, PORTD_LED_MP_GREEN);
      else
         set_bit(PORTD, PORTD_LED_MP_GREEN);
   }
   else
   {  if(on)
         set_bit(PORTD, PORTD_LED_T);
      else
         clear_bit(PORTD, PORTD_LED_T);
   }
}

// Become controller in charge of the bus
//
// ++cic
//    - float all pins
//    - REN=0

void GPIBDriver::Controller()
{
   Float();

   // start driving ATN IFC and REN
   PORTF |= (1<<PORTF_ATN) | (1<<PORTF_IFC) | (1<<PORTF_REN);
   DDRF  |= (1<<PORTF_ATN) | (1<<PORTF_IFC) | (1<<PORTF_REN);

   // pull /REN low to become actice controller
   REN(true);
}


uint8_t GPIBDriver::Lines()
{
   uint8_t pd=PIND & ((1<<PORTD_EOI) | (1<<PORTD_NRFD) | (1<<PORTD_NDAC) |
                      (1<<PORTD_SRQ) | (1<<PORTD_DAV));
   uint8_t pf=PINF & ((1<<PORTF_ATN) | (1<<PORTF_IFC) | (1<<PORTF_REN));

   return ~(pf | pd);
}

void GPIBDriver::Report(_ROMS msg)
{
   if(msg)
      USB<<msg<<':';
   else
      USB<<'<';
   if(DDRB==0)
      USB<<"i=";
   else
      USB<<"o=";
   USB.Hex(Data());

   uint8_t lines=Lines();
   if(lines & (1<<PORTD_EOI))  USB<<ROMS(" EOI");
   if(lines & (1<<PORTF_ATN))  USB<<ROMS(" ATN");
   if(lines & (1<<PORTD_SRQ))  USB<<ROMS(" SRQ");
   if(lines & (1<<PORTF_REN))  USB<<ROMS(" REN");
   if(lines & (1<<PORTF_IFC))  USB<<ROMS(" IFC");
   if(lines & (1<<PORTD_NRFD)) USB<<ROMS(" NRFD");
   if(lines & (1<<PORTD_NDAC)) USB<<ROMS(" NDAC");
   if(lines & (1<<PORTD_DAV))  USB<<ROMS(" DAV");

   if(msg)
      USB<<endl;
   else
      USB<<'>';
}

static const uint32_t TIMEOUT=0x1ff000ULL;  // 20 seconds

void GPIBDriver::Data(uint8_t byte)
{
   PORTB=~byte;
   DDRB=SAFER ? byte : 0xff;
   if(MicroPro)
   {
      // disable the LED
      clear_bit(DDRB,0);

      // pathetic board does not have an 8-bit port
      if(byte&1)
      {  clear_bit(PORTE,6);
         set_bit(DDRE,6);
      }
      else
      {  clear_bit(DDRE,6);
         set_bit(PORTE,6);
      }

      if(byte&0x80)
      {  clear_bit(PORTC,6);
         set_bit(DDRC,6);
      }
      else
      {  clear_bit(DDRC,6);
         set_bit(PORTC,6);
      }
   }
}

uint8_t GPIBDriver::Data()
{
   uint8_t byte=~PINB;
   if(MicroPro)
   {
      // pathetic board does not have an 8-bit port
      byte &= 0x7e;
      if(test_bit_clear(PINE,6))
         byte |= 1;
      if(test_bit_clear(PINC,6))
         byte |= 0x80;
   }
   return byte;
}

bool GPIBDriver::Command(uint16_t data)
{
#ifdef DEBUG
   USB<<ROMS("Command(");
   USB.Hex(data);

   uint16_t d=data&(CMD_SOURCE|CMD_ATN|0xff);
   if(d>=CMD_SOURCE+32 && d<CMD_SOURCE+127)
      USB<<'='<<'\''<<char(d)<<'\'';
   USB<<ROMS(")\r\n");
#endif
   if(data==0)
      return true;
   return CommandQueue.Put(data);
}

uint16_t GPIBDriver::Read()
{
   return ReadQueue.Get();
}

#ifdef DEBUG
GPIBDriver::State_t last=(GPIBDriver::State_t)~0;  // debug
uint16_t lasttimer=0;
#endif

int8_t GPIBDriver::Poll()
{
   // tune this so we go round the loop often enough to send/receive one byte
   for(uint8_t loop=0; loop<7; loop++)
   {
   #ifdef DEBUG
      if(Timer.Read()%128==1 && Timer.Read()!=lasttimer)
      {
         last=(GPIBDriver::State_t)~0;
         lasttimer=Timer.Read();
      }

      if(State!=last)
      {
         //Report();
         USB<<ROMS(" state=")<<State<<
                ROMS(" cq=")<<CommandQueue.Used()<<
                ROMS(" rq=")<<ReadQueue.Used()<<
                ROMS(" t=")<<Timer.Read();
         if(Timer.Timeout())
         {
            USB<<ROMS(" TO");
         }
         USB<<endl;
         last=State;
      }
   #endif

      switch(State)
      {
         case RESET:
            Float();
            Timer.Clear();
            CommandQueue.Clear();
            State=IDLE;
            return 0;

         case IDLE:
         {  uint16_t data=CommandQueue.Peek();

         #ifdef DEBUG
            if(data)
            {
               USB<<ROMS("data=");
               USB.Hex(data);
               USB<<endl;
            }
         #endif

            switch(data & CMD_MASK)
            {  case 0:
                  return 0;   // idle
               case CMD_SOURCE:
                  State=SH_START;
                  break;
               case CMD_ACCEPT:
                  State=AH_START;
                  break;
               case CMD_CONTROLLER:
                  State=CAC_START;
                  break;
               case CMD_SIC:
                  State=SIC_START;
                  break;
               case CMD_RELEASE:
                  Float();
                  State=IDLE;
                  break;
               default:
                  // drop unknown command
                  CommandQueue.Get();
                  return EINTERN;   // internal error
            }
         }  break;


         /* ..... source handshake ................................................... */

         case SH_START:
         {  uint16_t data=CommandQueue.Peek();
            ATN(data&CMD_ATN);
            DAV(false);

            // settling delay
            _delay_us(2);

            if(!NRFD() && !NDAC())
            {  State=RESET;
               return EACPROT;      // acceptor protocol error
            }
            State=SH_DATA;
         }  break;

         case SH_DATA:
         {  // remove command from the queue
            uint16_t data=CommandQueue.Get();

            // drive data
            EOI(data&CMD_EOI);
            Data(data);

            // settling delay
            _delay_us(2);

            // start timer default
            Timer.Single(Timeout);

            State=SH_WAIT_NRFD;
         }  break;

         case SH_WAIT_NRFD:
            if(!NRFD())
               State=SH_DAV;
            else if(Timer.Timeout())
            {  State=RESET;
               return EACTMO;      // acceptor timeout
            }
            break;

         case SH_DAV:
            DAV(true);

            // start timer default
            Timer.Single(Timeout);

            State=SH_WAIT_NDAC;
            break;

         case SH_WAIT_NDAC:
            if(!NDAC())
               State=SH_N_DAV;
            else if(Timer.Timeout())
            {  State=RESET;
               return EACTMO;      // acceptor timeout
            }
            break;

         case SH_N_DAV:
            DAV(false);
            EOI(false);
            FloatData();
            State=IDLE;
            break;


         /* ..... acceptor handshake ................................................. */

         case AH_START:
         {  uint16_t cmd=CommandQueue.Peek();
            Byte=cmd;

            NDAC(true);

            if(ATN())
            {  // settling delay
               _delay_us(1);
               State=AH_N_ATN;
            }
            else
               State=AH_READY;
         }  break;

         case AH_N_ATN:
            // negate ATN
            ATN(false);
            State=AH_READY;
            break;

         case AH_READY:
         {  uint16_t cmd=CommandQueue.Peek();

            // ready for data
            NRFD(false);

            // start timer
            Timer.Single(cmd&CMD_AC_QUICK ? 100 : Timeout);

            State=AH_WAIT_DAV;
         }  break;

         case AH_WAIT_DAV:
            if(DAV())
               State=AH_NRFD;
            else if(Timer.Timeout())
            {  uint16_t cmd=CommandQueue.Get();
               if(cmd&CMD_AC_QUICK)
               {  NDAC(false);
                  State=IDLE;
                  return 0;
               }
               else
               {  State=RESET;
                  return ESRCTMO;      // source timeout
               }
            }
            break;

         case AH_NRFD:
         {  NRFD(true);

            uint16_t cmd=CommandQueue.Peek();
            uint8_t eoi=EOI();

            uint16_t data=CMD_ACCEPT | Data();
            if(ATN())
               data|=CMD_ATN;
            if(eoi)
               data|=CMD_EOI;

         #ifdef DEBUG
            USB<<ROMS("data=");
            USB.Hex(data);
            if((data&0xff)>=32 && (data&0xff)<127)
            {  USB<<'='<<'\''<<char(data)<<'\'';
            }
            USB<<endl;
         #endif

            bool last;

            switch(cmd & CMD_AC_MASK)
            {
               case CMD_AC_EOI:
                  last=eoi;                     // EOI on byte
                  break;

               case CMD_AC_BYTE:
                  last=(Byte==(data & 0xff));   // byte match
                  break;

               case CMD_AC_COUNT:
                  last=--Byte==0;               // counter
                  break;

               default:
                  last=false;                   // forever/until timeout
                  break;
            }

            if(ReadQueue.Put(data))
               State=last ? AH_N_NDAC_LAST : AH_N_NDAC;
            else if(Timer.Timeout())
            {  State=last ? AH_WAIT_NDAV_LAST : AH_WAIT_NDAV;
               return EFULL;      // timeout on read queue
            }
            else
               return 0;
         } break;

         case AH_N_NDAC:
            NDAC(false);
            State=AH_WAIT_NDAV;
            break;

         case AH_WAIT_NDAV:
            if(!DAV())
               State=AH_NDAC;
            else if(Timer.Timeout())
            {  State=RESET;
               return ESRCTMO;      // source timeout
            }
            break;

         case AH_NDAC:
            NDAC(true);
            State=AH_READY;
            break;

         case AH_N_NDAC_LAST:
            NDAC(false);
            State=AH_WAIT_NDAV_LAST;
            break;

         case AH_WAIT_NDAV_LAST:
            if(!DAV())
            {  CommandQueue.Get();  // remove command from queue
               State=IDLE;
            }
            else if(Timer.Timeout())
            {  State=RESET;
               return ESRCTMO;      // source timeout
            }
            break;


         /* ..... take control ....................................................... */

         case CAC_START:
            ATN(false);
            EOI(false);
            NRFD(false);
            DAV(false);
            NDAC(false);

            if(CommandQueue.Get()&CMD_CTRL_FORCE)
               State=CAC_COMPLETE;
            else
            {
               Timer.Single(Timeout);
               State=CAC_WAIT;
            }
            break;

         case CAC_WAIT:
            if(!(GPIB.ATN() || GPIB.NRFD() || GPIB.NDAC() || GPIB.DAV()))
               State=CAC_COMPLETE;
            else if(Timer.Timeout())
            {  State=RESET;
               return ECTRLTMO;      // controller timeout
            }
            break;

         case CAC_COMPLETE:
            Controller();
            State=IDLE;
            break;


         /* ..... send interface clear ............................................... */

         case SIC_START:
            CommandQueue.Get();
            GPIB.IFC(true);
            Timer.Single(Timeout);
            State=SIC_WAIT;
            break;

         case SIC_WAIT:
            if(!(GPIB.DAV() || GPIB.NRFD() || GPIB.NDAC()))
               State=SIC_COMPLETE;
            else if(Timer.Timeout())
            {  State=SIC_COMPLETE;
               return ECTRLTMO;      // controller timeout
            }
            break;

         case SIC_COMPLETE:
            GPIB.IFC(false);
            State=IDLE;
            break;


         /* ..... parallel poll ...................................................... */

         case PP_START:
            GPIB.ATN(true);
            GPIB.EOI(true);
            // settling delay
            _delay_us(1);
            State=PP_COMPLETE;
            break;

         case PP_COMPLETE:
         {  uint16_t data;

            data=CMD_PARPOLL | Data();
            GPIB.ATN(false);
            GPIB.EOI(false);
            State=IDLE;

            if(!ReadQueue.Put(data))
               return EFULL;      // timeout on read queue

         }  break;

         /* .......................................................................... */

         default:
            State=RESET;
            return EINTERN;         // internal error
      }
   }
   return 0;
}

/* ========================================================================== */

const GPIBDriver __attribute__((__progmem__)) GPIB;

//

// GPIB System Concepts
// ====================
//                                    ______________________
// ATN_________________________ATN___/                      ATN
//
//    ______          _________   ________          ________
// DAV      \________/         DAV        \________/        DAV
//
//        _____           _____    __________           ____
// NRFD__/     \_________/     NRFD          \_________/    NRFD
//
//                _____                         _____
// NDAC__________/     \_______NDAC____________/     \______NDAC
//
//         ________________           ___________________
// DATA---<________________>---DATA--<___________________>--DATA
//
//
// TNT4882 Manual
// ==============
//        __________________________
// ATN___/                          ATN
//
//    ___________          _________
// DAV           \________/         DAV
//
//           ________            ___
// NRFD_____/        \__________/   NRFD
//
//                      _____
// NDAC________________/     \______NDAC
//
//              ___________________
// DATA--------<___________________>--DATA
//

/* ----- EOF gpib.cc ----- */

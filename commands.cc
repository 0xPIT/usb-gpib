/* #------------------------------------------------------------------------#
   |                                                                        |
   |   commands.cc                                                          |
   |                                                                        |
   |   USB-GPIB command processing                                          |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <ctype.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usb_serial.h"
#include "serial.h"
#include "gpib.h"

static const int16_t NONE=0x8000;
static const int16_t EINVCHAR=-1;
static const int16_t EBUFFER=-2;
static const int16_t ETOOMANY=-3;
static const int16_t ESYNTAX=-4;
static const int16_t ERANGE=-5;
static const int16_t EVALUE=-6;
static const int16_t EUNDERFLOW=-7;
static const int16_t ETIMEOUT=-8;
static const int16_t ECOMMAND=-100;

/* -------------------------------------------------------------------------- */

#define ROM_PTR /* PROGMEN as a modifier to a pointer */
#define STR2(a) #a
#define STR(a) STR2(a)
#define STRING(s) PROGMEM static const char String_##s[]=STR(s)

typedef int16_t (*Method)(char *);
struct Commands
{
   const char *Command;
   Method Function;
} __attribute__((__progmem__));
const ROM_PTR struct Commands *FindCommand(const ROM_PTR struct Commands *table,
                                           const char *command);

extern const struct Commands commandTable[];

char *skipws(char *ptr)
{
   while(isspace(*ptr))
      ptr++;
   return ptr;
}

int16_t strtoi(const char *ptr, const char **end)
{
   int16_t value=0;

   char c=*ptr;
   if(c=='0' && ptr[1]=='x')
   {  ptr+=2;
      while(isxdigit(c=*ptr))
      {
         ptr++;
         if(c<='9')
            c-='0';
         else
            c=tolower(c)-'a'+10;
         value=(value<<4) | c;
      }
   }
   else
   {  ptr++;
      value=c-'0';
      while(isdigit(c=*ptr))
      {
         ptr++;
         c-='0';
         value=value*10 + c;
      }
   }
   if(end)
      *end=ptr;
   return value;
}


/* ----- GPIB defined command bytes ----------------------------------------- */

int16_t Byte_MLA(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x20 | (GPIB.MyAddress&31);
}

int16_t Byte_MSA(char *)
{
   if((GPIB.MyAddress & 0xff00)!=0)
      return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x60 | ((GPIB.MyAddress>>8)&31);
   return 0;
}

int16_t Byte_MTA(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x40 | (GPIB.MyAddress&31);
}

int16_t Byte_DCL(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x14;
}

int16_t Byte_GET(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x08;
}

int16_t Byte_GTL(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x01;
}

int16_t Byte_LLO(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x11;
}

int16_t Byte_PPC(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x05;
}

int16_t Byte_PPU(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x15;
}

int16_t Byte_SDC(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x04;
}

int16_t Byte_SPD(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x19;
}

int16_t Byte_SPE(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x18;
}

int16_t Byte_TCT(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x09;
}

int16_t Byte_UNL(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x3f;
}

int16_t Byte_UNT(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x5f;
}

int16_t Byte_LAD(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x20 | (GPIB.DeviceAddress&31);
}

int16_t Byte_TAD(char *)
{  return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x40 | (GPIB.DeviceAddress&31);
}

int16_t Byte_SAD(char *)
{
   if((GPIB.DeviceAddress & 0xff00)!=0)
      return GPIB.CMD_SOURCE | GPIB.CMD_ATN | 0x60 | ((GPIB.DeviceAddress>>8)&31);
   return 0;
}


/* ----- command functions -------------------------------------------------- */

/* Return codes
 * --------------
 * ++eoi
 *    - message when EOI is seen when listening and not in ++rd or ++read eoi command
 * ++srq
 *    - message when SRQ is asserted
 * ++error <value>
 *    - message when an error occurs
 *
 *
 * Maybe later:
 * ++dev <pad> <sad> <tmo> <eot> <eos> -> <name>
 *     Assign settings to user-configurable device, return identifier
 * ++config/++ask various options
 * ++ln [<device>] -> result
 *     check for presence of a device on the bus
 *     (not clear how this is done?)
 * ++onl [<device>] <v>
 *     Set device online/offline
 * ++pad <device> <address>
 *     Change device primary address
 * ++rsc
 *     Request/release system control
 * ++sad <device> <address>
 *     Change device secondary address
 *
 */


/* ..........................................................................
 *
 * ++addr <addr> [<secondary>]
 * ++addr? -> <addr>
 *    - set or return default device address to <device> (default 1)
 *    - may include secondary address either as second word or MSBs in <device>
 */
int16_t Command_addr(char *tail)
{
   if(*tail==0 || *tail=='?')
      return GPIB.DeviceAddress;

   uint16_t addr=strtoi(tail,(const char **)&tail);

   if(*tail)
   {  if((addr & 0xffe0)!=0)
         return EVALUE;

      tail=skipws(tail);
      if(*tail)
      {
         uint16_t second=strtoi(tail,(const char **)&tail);
         if((second & 0xffe0)!=0)
            return EVALUE;
         addr|=(0x60 | second)<<8;
      }
   }
   GPIB.DeviceAddress=addr;

   return NONE;
}

/* ..........................................................................
 *
 * ++ATN <value>
 * ++ATN? -> <value>
 *
 *    set or return ~ATN
 */
int16_t Command_ATN(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return GPIB.ATN();
      case '0':
      case '1':
         GPIB.ATN(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++auto <state>
 *     if state==1: automatic ++read eoi after sending a string ending in "?"
 *     elif state==0: leave in LISTEN state
 */
int16_t Command_auto(char *tail)
{
   Serial<<ROMS("auto ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++cac <v>
 *    if v!=1: wait for handshake to complete
 *    Become active controller
 */
int16_t Command_cac(char *tail)
{
   GPIB.Command(GPIB.CMD_CONTROLLER | *tail);
   return NONE;
}


/* ..........................................................................
 *
 * ++clr [<device>]
 *    - selected/universal device clear
 *    - if device=="all":
 *         ++cmd MTA UNL DCL
 *      else:
 *         ++cmd MTA UNL LAD SAD SDC
 */
int16_t Command_clr(char *tail)
{
   GPIB.Command(Byte_MTA(0));
   GPIB.Command(Byte_UNL(0));
   if(!strcmp_P(tail, PSTR("all")))
      GPIB.Command(Byte_DCL(0));
   else
   {
      GPIB.Command(Byte_LAD(0));
      GPIB.Command(Byte_SAD(0));
      GPIB.Command(Byte_SDC(0));
   }
   return NONE;
}

/* ..........................................................................
 *
 * ++cmd <bytes or codes>
 *    - set ATN=0, send bytes, set ATN=1, set ATN=float
 *    - for each byte, if code is preceded by "-": EOI=0
 *     else: EOI=1
 */

#define BYTE(s) { String_##s, Byte_##s }

#undef SPE
STRING(DCL); STRING(GET); STRING(GTL); STRING(LAD); STRING(LLO);
STRING(MLA); STRING(MSA); STRING(MTA); STRING(PPC); STRING(PPU);
STRING(SAD); STRING(SDC); STRING(SPD); STRING(SPE); STRING(TAD);
STRING(TCT); STRING(UNL); STRING(UNT);

static const struct Commands byteTable[]=
{
   BYTE(DCL), BYTE(GET), BYTE(GTL), BYTE(LAD), BYTE(LLO),
   BYTE(MLA), BYTE(MSA), BYTE(MTA), BYTE(PPC), BYTE(PPU),
   BYTE(SAD), BYTE(SDC), BYTE(SPD), BYTE(SPE), BYTE(TAD),
   BYTE(TCT), BYTE(UNL), BYTE(UNT),
   { 0,0 },
};

#define Debug USB

int16_t Command_cmd_wrt(char *tail,bool atn)
{
   //GPIB.ATN(atn);
   uint8_t eoi=0;
   uint16_t cmd;
   while(char c=*tail)
   {  const char *word=tail++;

      if(c=='-')
      {  eoi=1;
         continue;
      }

      uint8_t byte=0;

      if(c=='"')
      {
         while((c=*tail)!='"' && c>=' ')
         {
            tail++;

            cmd=GPIB.CMD_SOURCE | (c&0xff);
            if(atn)
               cmd|=GPIB.CMD_ATN;
            if(eoi && *tail=='"')
               cmd|=GPIB.CMD_EOI;
            GPIB.Command(cmd);
         }

         if(c!='"')
            goto fail;

         tail++;
      }
      else
      {
         uint8_t skip=0;

         if(isdigit(c))
         {
            if(c=='0' && *tail=='x')
            {  tail++;
               while(isxdigit(c=*tail))
               {
                  byte<<=4;
                  if(c<='9')
                     byte+=c-'0';
                  else
                     byte+=tolower(c)-'a'+10;
                  tail++;
               }
            }
            else
            {
               byte=c-'0';
               while(isdigit(c=*tail))
               {
                  byte=byte*10+c-'0';
                  tail++;
               }
            }

            if(!(c==' ' || c==0))
               goto fail;
         }
         else
         {
            while(isalnum(*tail))
               tail++;

            c=*tail;
            *tail=0;

            const ROM_PTR struct Commands *cmd=FindCommand(byteTable,word);

            if(!cmd)
               goto fail;

            *tail=c;
            Method f=(Method)(pgm_read_word(&cmd->Function));

            byte=f(tail);

            // handle cases where there is nothing to send
            if(!byte)
               skip=1;
         }

         if(!skip)
         {
            //GPIB.EOI(eoi);
            //byte=GPIB.Out(byte);
            //GPIB.EOI(false);
            //if(byte)
            //   goto fail;

            cmd=GPIB.CMD_SOURCE | byte;
            if(atn)
               cmd|=GPIB.CMD_ATN;
            if(eoi)
               cmd|=GPIB.CMD_EOI;
            GPIB.Command(cmd);
         }
      }

      tail=skipws(tail);
      eoi=0;
   }

   return NONE;

fail:
   //GPIB.EOI(false);
   //GPIB.ATN(false);
   return EINVCHAR;
}

int16_t Command_cmd(char *tail)
{
   return Command_cmd_wrt(tail,true);
}

/* ..........................................................................
 *
 *  ++data? -> <byte>
 *     return ~DIO[8:1]
 */
int16_t Command_data(char *tail)
{
   return GPIB.Data();
}

/* ..........................................................................
 *
 * ++DAV <value>
 * ++DAV? -> <value>
 *
 *    set or return ~DAV
 */
int16_t Command_DAV(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return GPIB.DAV();
      case '0':
      case '1':
         GPIB.DAV(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++DEBUG <value>
 *     do whatever we think is interesting
 */
int16_t Command_DEBUG(char *tail)
{
   if(*tail)
   {
      int16_t addr=strtoi(tail,(const char **)&tail);

      if(!*tail)
      {
         USB<<addr<<endl;
         USB.Flush();

         _delay_ms(100);

         Method f=(Method)addr;
         GPIB.Float();
         cli();
         UDCON=1;
         USBCON=(1<<FRZCLK);  // disable USB
         UCSR1B=0;
         EIMSK=0; PCICR=0; SPCR=0; ACSR=0; EECR=0; ADCSRA=0;
         TIMSK0=0; TIMSK1=0; TIMSK3=0; TIMSK4=0; UCSR1B=0; TWCR=0;
         _delay_ms(10);

         f(tail);
         sei();
      }
   }

   USB<<ROMS("ddrb=");   USB.Hex(uint8_t(DDRB));
   USB<<ROMS(" portb="); USB.Hex(uint8_t(PORTB));
   USB<<ROMS(" pinb=");  USB.Hex(uint8_t(PINB));
   USB<<endl;

   USB<<ROMS("ddrc=");   USB.Hex(uint8_t(DDRC));
   USB<<ROMS(" portc="); USB.Hex(uint8_t(PORTC));
   USB<<ROMS(" pinc=");  USB.Hex(uint8_t(PINC));
   USB<<endl;

   USB<<ROMS("ddrd=");   USB.Hex(uint8_t(DDRD));
   USB<<ROMS(" portd="); USB.Hex(uint8_t(PORTD));
   USB<<ROMS(" pind=");  USB.Hex(uint8_t(PIND));
   USB<<endl;

   USB<<ROMS("ddre=");   USB.Hex(uint8_t(DDRE));
   USB<<ROMS(" porte="); USB.Hex(uint8_t(PORTE));
   USB<<ROMS(" pine=");  USB.Hex(uint8_t(PINE));
   USB<<endl;

   USB<<ROMS("ddrf=");   USB.Hex(uint8_t(DDRF));
   USB<<ROMS(" portf="); USB.Hex(uint8_t(PORTF));
   USB<<ROMS(" pinf=");  USB.Hex(uint8_t(PINF));
   USB<<endl;

   GPIB.Report();
   USB<<endl;

   return NONE;
}

/* ..........................................................................
 *
 * ++echo <mode>
 *     if mode==1: echo commands
 *     else: do not echo
 */
extern bool Echo;
int16_t Command_echo(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return Echo;
      case '0':
      case '1':
         Echo=(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++EOI <value>
 * ++EOI? -> <value>
 *
 *    set or return ~EOI
 */
int16_t Command_EOI(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return GPIB.EOI();
      case '0':
      case '1':
         GPIB.EOI(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++eoi <mode>
 *     if mode==0: no EOI asserted
 *     else: assert EOI on last byte of a command (default)
 */
int16_t Command_eoi(char *tail)
{
   Serial<<ROMS("eoi ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++eos <mode>
 *     if mode==0: append "\r\n" to commands
 *     elif mode==1: append "\r" to commands
 *     elif mode==2: append "\n" to commands
 *     else: do not append anything (default)
 * ++eos {4|8} <byte>
 *     if arg=0,1,2,3: prologix
 *     else:
 *     if first arg==4 or 12: terminate reads on <byte>
 *     if first arg==8 or 12: set EOI when writing <byte>
 */

int16_t Command_eos(char *tail)
{
   Serial<<ROMS("eos ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++eot_char <byte>
 *     set eot_char
 */
int16_t Command_eot_char(char *tail)
{
   Serial<<ROMS("eot_char ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++eot_enable <eot>
 *     control how EOI when receiving data is handled
 *     if eot==0: ignore
 *     elif eot==1: send <eot_char>
 *     else: send ++eoi (default)
 */
int16_t Command_eot_enable(char *tail)
{
   Serial<<ROMS("eot_enable ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 *
 *
 */
int16_t Command_error(char *tail)
{
   Serial<<ROMS("error ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++gts <v>
 *     if v:
 *        transfer data as acceptor, until END
 *     float all pins
 *     ++mode 0 (device)
 */
int16_t Command_gts(char *tail)
{
   Serial<<ROMS("gts ")<<tail<<endl;
   GPIB.Float();
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++ver
 *     Display version string
 */
void Version()
{
   static const char version[] PROGMEM =
                          " version"
                          #include "version.h"
                          "\r\n";
   USB<<ROMS("Kingswood USB-GPIB-32U4 ");
   if(GPIB.MicroPro)
      USB<<ROMS("MicroPro");
   else
      USB<<ROMS("Teensy");
   USB<<ROM(version);
}

/* ..........................................................................
 *
 * ++IFC <value>
 * ++IFC? -> <value>
 *    set or return ~IFC
 */
int16_t Command_IFC(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return GPIB.IFC();
      case '0':
      case '1':
         GPIB.IFC(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++lines? -> <byte>
 *    - return ~{EOI,ATN,SRQ,REN,IFC,NRFD,NDAC,DAV}
 */
int16_t Command_lines(char *tail)
{
   uint8_t byte=0;
   if(GPIB.EOI())  byte|=1<<7;
   if(GPIB.ATN())  byte|=1<<6;
   if(GPIB.SRQ())  byte|=1<<5;
   if(GPIB.REN())  byte|=1<<4;
   if(GPIB.IFC())  byte|=1<<3;
   if(GPIB.NRFD()) byte|=1<<2;
   if(GPIB.NDAC()) byte|=1<<1;
   if(GPIB.DAV())  byte|=1<<0;

   return byte;
}

/* ..........................................................................
 *
 * ++llo [<device>]
 *    equivalent to ++cmd LAD SAD LLO UNL
 */
int16_t Command_llo(char *tail)
{
   GPIB.Command(Byte_LAD(0));
   GPIB.Command(Byte_SAD(0));
   GPIB.Command(Byte_LLO(0));
   GPIB.Command(Byte_UNL(0));

   return NONE;
}

/* ..........................................................................
 *
 * ++loc [<device>]
 *    equivalent to ++cmd TAD SAD UNL LAD SAD GTL
 */
int16_t Command_loc(char *tail)
{
   GPIB.Command(Byte_TAD(0));
   GPIB.Command(Byte_SAD(0));
   GPIB.Command(Byte_UNL(0));
   GPIB.Command(Byte_LAD(0));
   GPIB.Command(Byte_SAD(0));
   GPIB.Command(Byte_GTL(0));

   return NONE;
}

/* ..........................................................................
 *
 * ++lon <value>
 *    enable listen only (promiscuous listen) mode
 */
int16_t Command_lon(char *tail)
{
   Serial<<ROMS("lon ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++mode <mode>
 *     if mode==0 or mode=='d': device mode
 *     elif mode==1 or mode=='c': controller in command mode
 *     elif mode=='+': prompting mode (default)
 */
int16_t Command_mode(char *tail)
{
   Serial<<ROMS("mode ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++NDAC <value>
 * ++NDAC? -> <value>
 *
 *    set or return ~NDAC
 */
int16_t Command_NDAC(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return GPIB.NDAC();
      case '0':
      case '1':
         GPIB.NDAC(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++NRFD <value>
 * ++NRFD? -> <value>
 *
 *    set or return ~NRFD
 */
int16_t Command_NRFD(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return GPIB.NRFD();
      case '0':
      case '1':
         GPIB.NRFD(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++pct <address>
 *    pass CIC to device
 *    equivalent to ++cmd UNL LAD SAD TAD SAD TCT
 */
int16_t Command_pct(char *tail)
{
   GPIB.Command(Byte_UNL(0));
   GPIB.Command(Byte_LAD(0));
   GPIB.Command(Byte_SAD(0));
   GPIB.Command(Byte_TAD(0));
   GPIB.Command(Byte_SAD(0));
   GPIB.Command(Byte_TCT(0));
   GPIB.Command(GPIB.CMD_RELEASE);

   return NONE;
}

/* ..........................................................................
 *
 * ++rd <count>
 *    read <count> bytes or until EOI or timeout
 */
int16_t Command_rd(char *tail)
{
   GPIB.Command(GPIB.CMD_ACCEPT);

   return NONE;
}

/* ..........................................................................
 *
 * ++read [<end>]
 *     ++cmd UNL UNT TAD SAD MLA
 *     if end=="eoi": read until eoi
 *     elif end=="tmo": read until timeout
 *     elif end: read until byte <end>
 *     else: read until timeout
 * ++read <device> <end>
 *     ++cmd UNL UNT <device> MLA
 *     if end=="eoi": read until eoi
 *     elif end=="tmo": read until timeout
 *     else: read until byte <end>
 */
int16_t Command_read(char *tail)
{
   GPIB.Command(Byte_UNL(0));
   GPIB.Command(Byte_MLA(0));
   GPIB.Command(Byte_TAD(0));
   GPIB.Command(Byte_SAD(0));

   uint16_t cmd=GPIB.CMD_ACCEPT;
   if(!strcmp_P(tail, PSTR("eoi")))
   { }
   else if(!strcmp_P(tail, PSTR("tmo")))
      cmd|=GPIB.CMD_AC_TMO;
   else if(*tail)
   {
      uint8_t n=strtoi(tail,(const char **)&tail);
      cmd|=GPIB.CMD_AC_COUNT | n;
   }

   GPIB.Command(cmd);

   return NONE;
}

/* ..........................................................................
 *
 * ++REN <value>
 * ++REN? -> <value>
 *
 *    set or return ~REN
 */
int16_t Command_REN(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return GPIB.REN();
      case '0':
      case '1':
         GPIB.REN(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++rpp -> <result>
 *     perform parallel poll by asserting ATN and EOI, reading response from data bus
 */
int16_t Command_rpp(char *tail)
{
   GPIB.Command(GPIB.CMD_PARPOLL);
   return NONE;
}

/* ..........................................................................
 *
 * ++rsp <device> -> <result>
 *     read serial poll data, equivalent to
 *     ++cmd UNL MLA SPE TAD SAD
 *     ++rd <1 byte>
 *     ++cmd SPD
 */
int16_t Command_rsp(char *tail)
{
   GPIB.Command(Byte_UNL(0));
   GPIB.Command(Byte_MLA(0));
   GPIB.Command(Byte_SPE(0));
   GPIB.Command(Byte_TAD(0));
   GPIB.Command(Byte_SAD(0));
   GPIB.Command(GPIB.CMD_ACCEPT | GPIB.CMD_AC_COUNT | 0x01);
   GPIB.Command(Byte_SPD(0));

   return NONE;
}

/* ..........................................................................
 *
 * ++rst
 *    - float all pins
 *    - set defaults
 */
int16_t Command_rst(char *tail)
{
   Echo=true;
   GPIB.Initialize();
   return NONE;
}

/* ..........................................................................
 *
 * ++savecfg <state>
 *     (no operation)
 *     if state=1: save <mode> <addr> <auto> <aoi> <eos> <eot> <timeout> whenver changed
 *     else: do not save
 */
int16_t Command_savecfg(char *tail)
{
   Serial<<ROMS("savecfg ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++self <address> [<secondary>]
 * ++self? -> <address>
 *    set or return address of USB-GPIB (default 0)
 */
int16_t Command_self(char *tail)
{
   if(*tail==0 || *tail=='?')
      return GPIB.MyAddress;

   uint16_t addr=strtoi(tail,(const char **)&tail);
   USB.Hex(addr);

   if(*tail)
   {  if((addr & 0xffe0)!=0)
         return EVALUE;

      tail=skipws(tail);
      if(*tail)
      {
         uint16_t second=strtoi(tail,(const char **)&tail);
         if((second & 0xffe0)!=0)
            return EVALUE;
         addr|=(0x60 | second)<<8;
      }
   }
   USB.Hex(addr);

   GPIB.MyAddress=addr;

   return NONE;
}

/* ..........................................................................
 *
 * ++sic [<value>]
 *    send interface clear
 */
int16_t Command_sic(char *tail)
{
   GPIB.Command(GPIB.CMD_SIC);
   return NONE;
}

/* ..........................................................................
 *
 * ++SRQ <value>
 * ++SRQ? -> <value>
 *
 *    set or return ~SRQ
 */
int16_t Command_SRQ(char *tail)
{
   switch(*tail)
   {  case 0:
      case '?':
         return GPIB.SRQ();
      case '0':
      case '1':
         GPIB.SRQ(*tail&1);
         return NONE;
   }

   return EVALUE;
}

/* ..........................................................................
 *
 * ++status <value>
 * ++status? -> <value>
 *     report device status
 * ++rsv <byte>
 *     set or return status byte to be used when we are a slave
 *     if byte&0x40: SRQ=0
 */
int16_t Command_status(char *tail)
{
   Serial<<ROMS("status ")<<tail<<endl;
   return ECOMMAND;
}

/* ..........................................................................
 *
 * ++tmo <timeout>
 *     set timeout in ms (up to 65 seconds)
 */
int16_t Command_tmo(char *tail)
{
   if(*tail==0 || *tail=='?')
      return GPIB.Timeout;

   uint16_t tmo=strtoi(tail,(const char **)&tail);
   if(*tail)
      return EVALUE;

   GPIB.Timeout=tmo;
   return NONE;
}

/* ..........................................................................
 *
 * ++trg <device> [<device>...]
 *     triggers one or more devices
 *     equivalent to ++cmd MTA UNL LAD SAD ... GET
 */
int16_t Command_trg(char *tail)
{
   GPIB.Command(Byte_MTA(0));
   GPIB.Command(Byte_UNL(0));
   GPIB.Command(Byte_LAD(0));
   GPIB.Command(Byte_SAD(0));
   GPIB.Command(Byte_GET(0));

   return NONE;
}

/* ..........................................................................
 *
 * ++ver
 *     show version
 */
int16_t Command_ver(char *tail)
{
   Version();
   return NONE;
}

/* ..........................................................................
 *
 * ++wrt <bytes>
 *    - set ATN=1, send bytes, set ATN=float
 *    - for each byte, if code is preceded by "-": EOI=0
 *     else: EOI=1
 */
int16_t Command_wrt(char *tail)
{
   return Command_cmd_wrt(tail,false);
}

/* ..........................................................................
 *
 * ++write <bytes>
 *    - set ATN=1, send MTA UNL LAD SAD, followed by bytes
 */
int16_t Command_write(char *tail)
{
   GPIB.Command(Byte_MTA(0));
   GPIB.Command(Byte_UNL(0));
   GPIB.Command(Byte_LAD(0));
   GPIB.Command(Byte_SAD(0));

   return Command_wrt(tail);
}


/* ----- command strings ---------------------------------------------------- */

STRING(addr);   STRING(ATN);    STRING(auto);   STRING(cac);
STRING(clr);    STRING(cmd);    STRING(data);   STRING(DAV);
STRING(DEBUG);  STRING(EOI);    STRING(eoi);    STRING(eos);
STRING(eot_char);               STRING(eot_enable);
STRING(error);  STRING(gts);    STRING(help);   STRING(IFC);
STRING(lines);  STRING(llo);
STRING(loc);    STRING(lon);    STRING(mode);   STRING(NDAC);
STRING(NRFD);   STRING(pct);    STRING(rd);     STRING(read);
STRING(REN);    STRING(rpp);
STRING(rsp);    STRING(rst);    STRING(rsv);    STRING(savecfg);
STRING(self);   STRING(sic);    STRING(spoll);  STRING(sre);
STRING(SRQ);    STRING(status); STRING(tmo);    STRING(trg);
STRING(ver);    STRING(write);  STRING(wrt);    STRING(echo);

#define COMMAND(s) { String_##s, Command_##s }

int16_t Command_help(char *tail);
const struct Commands commandTable[]=
{
   COMMAND(ATN),    COMMAND(DAV),    COMMAND(DEBUG),  COMMAND(EOI),
   COMMAND(IFC),    COMMAND(NDAC),   COMMAND(NRFD),   COMMAND(REN),
   COMMAND(SRQ),

   COMMAND(addr),   COMMAND(auto),   COMMAND(cac),
   COMMAND(clr),    COMMAND(cmd),    COMMAND(data),
   COMMAND(echo),   COMMAND(eoi),    COMMAND(eos),    COMMAND(eot_char),
   COMMAND(eot_enable),
   COMMAND(error),  COMMAND(gts),    COMMAND(help),
   COMMAND(lines),  COMMAND(llo),    COMMAND(loc),    COMMAND(lon),
   COMMAND(mode),   COMMAND(pct),
   COMMAND(rd),     COMMAND(read),
   COMMAND(rpp),    COMMAND(rsp),    COMMAND(rst),
   { String_rsv, Command_status },
   COMMAND(savecfg),
   COMMAND(self),   COMMAND(sic),
   { String_spoll, Command_rsp },
   { String_sre, Command_REN },
   COMMAND(status), COMMAND(tmo),    COMMAND(trg),
   COMMAND(ver),    COMMAND(write),  COMMAND(wrt),
   { 0,0 },
};

const ROM_PTR struct Commands *FindCommand(const ROM_PTR struct Commands *table,
                                           const char *command)
{
   uint8_t lim;
   for(lim=0; pgm_read_word(&table[lim].Command)!=0; lim++)
      { }

   for(; lim!=0; lim>>=1)
   {
      const ROM_PTR struct Commands *p = table + (lim >> 1);
      const ROM_PTR char *entry=(const ROM_PTR char *)pgm_read_word(&p->Command);
      int8_t cmp=strcmp_P(command, entry);

      if(cmp==0)
         return p;
      else if(cmp > 0)
      {  /* command > p: move right */
         table = p + 1;
         lim--;
      }
      else
      { /* move left */ }
   }

   return 0;
}


int16_t Command(char *command)
{  char c;
   char *tail=command;

   if(*command<' ')
      return 0;

   int r=ECOMMAND;
   if(*command=='?')
      r=Command_help(command+1);
   else
   {
      while(isalnum(*tail))
         tail++;

      c=*tail;
      *tail=0;

      const ROM_PTR struct Commands *cmd=FindCommand(commandTable,command);

      if(cmd)
      {
         *tail=c;
         tail=skipws(tail);

         Method f=(Method)(pgm_read_word(&cmd->Function));
         r=f(tail);
      }
   }

   if(r!=NONE)
      USB<<r<<endl;
   return r;
}


/* ..........................................................................
 *
 * ++help
 *    show available commands
 */
int16_t Command_help(char *tail)
{
   Version();
   USB<<ROMS("Available commands:");

   const char *p;
   for(uint8_t i=0; (p=(const char *)pgm_read_word(&commandTable[i].Command))!=0; i++)
   {
      char c=pgm_read_byte(p);

      if(*tail=='?' || c>='a' || c==*tail)
      {
         USB<<ROMS("\r\n   ++")<<ROM(p);
         if(p==String_cmd)
         {
            USB<<ROMS(" [-]<byte> ... or one of:\r\n        ");
            for(uint8_t i=0; (p=(const char *)pgm_read_word(&byteTable[i].Command))!=0; i++)
               USB<<' '<<ROM(p);
         }
      }
   }
   USB<<endl;
   return NONE;
}

/* ----- EOF commands.cc ----- */

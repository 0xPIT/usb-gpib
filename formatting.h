/* #------------------------------------------------------------------------#
   |                                                                        |
   |   formatting.h                                                         |
   |                                                                        |
   |   String formatting helper class, used as CRTP.                        |
   |                                                                        |
   |   Copyright 2014, Frank A. Kingswood, www.kingswood-consulting.co.uk   |
   |                                                                        |
   #------------------------------------------------------------------------# */

#ifndef FORMATTING_H_
#define FORMATTING_H_
#include <avr/pgmspace.h>

enum _ROMS { A=0, B=0xffff };
#define ROM(s) ((_ROMS)(intptr_t)(s))
#define ROMS(s) ROM(PSTR(s))

PROGMEM extern const char CRLF[];
template<class Output> class Formatting
{
public:

   static void Send(char c)
   {
      Output::Send_(c);
   }

   static void __attribute__((noinline)) SendSafeChar(char c)
   {
      // a bit of safety - output only visible chars and make the rest octal
      if((c>=32 && c<127) || c==13 || c==10 || c==9)
         Send(c);
      else
      {  Send('\\');
         Send(char('0'+(((c>>6))&3)));
         Send(char('0'+(((c>>3))&7)));
         Send(char('0'+(((c>>0))&7)));
      }
   }

   static void __attribute__((noinline)) Send(const char *message)
   {
      while(char c=*message)
      {
         SendSafeChar(c);
         message++;
      }
   }

   static void __attribute__((noinline)) Send(_ROMS message)
   {
      intptr_t m=message;
      while(char c=pgm_read_byte_near(m))
      {
         SendSafeChar(c);
         m++;
      }
   }

   static void __attribute__((noinline)) Send(uint16_t value)
   {
      static const unsigned powers[]={10000,1000,100,10,0};
      bool f=false;
      for(uint8_t i=0;;i++)
      {  unsigned n=powers[i];
         if(n==0)
            break;
         char d='0';
         while(value>=n)
         {  value-=n;
            d=d+1;
         }
         if(d!='0')
            f=true;
         if(f)
            Send(d);
      }
      Send(char('0'+value));
   }

   static void __attribute__((noinline)) Hex(uint16_t value)
   {
      Send(ROMS("0x"));
      SendHex8(value>>8);
      SendHex8(value);
   }

   static void __attribute__((noinline)) Hex(int16_t value)
   {  Hex(uint16_t(value));
   }

   static void __attribute__((noinline)) Hex(uint8_t value)
   {
      Send(ROMS("0x"));
      SendHex8(value);
   }

   static void __attribute__((noinline)) Hex(int8_t value)
   {  Hex(uint8_t(value));
   }

   static void __attribute__((noinline)) Send(int16_t value)
   {
      uint16_t v;
      if(value<0)
      {  Send('-');
         v=-value;
      }
      else
         v=value;
      Send(v);
   }

   static void __attribute__((noinline)) Send()
   {  Send(ROM(CRLF));
   }

private:
   static void __attribute__((noinline)) SendHex8(uint8_t value)
   {  SendHex4(value>>4);
      SendHex4(value);
   }
   static void __attribute__((noinline)) SendHex4(uint8_t value)
   {  value&=15;
      if(value>9)
         Send(char('a'-10+value));
      else
         Send(char('0'+value));
   }
};

class FormattingNL { };
extern const FormattingNL endl;

template<class Output> const Formatting<Output> &operator <<(const Formatting<Output> &o, int16_t value)
{  o.Send(value);
   return o;
}

template<class Output> const Formatting<Output> &operator <<(const Formatting<Output> &o, uint16_t value)
{  o.Send(value);
   return o;
}

template<class Output> const Formatting<Output> &operator <<(const Formatting<Output> &o, char value)
{  o.Send(value);
   return o;
}

template<class Output> const Formatting<Output> &operator <<(const Formatting<Output> &o, const char *value)
{  o.Send(value);
   return o;
}

template<class Output> const Formatting<Output> &operator <<(const Formatting<Output> &o, _ROMS value)
{  o.Send(value);
   return o;
}

template<class Output> const Formatting<Output> &operator <<(const Formatting<Output> &o, const FormattingNL &)
{  o.Send();
   return o;
}


#endif /* FORMATTING_H_ */

/* ----- EOF formatting.h ----- */

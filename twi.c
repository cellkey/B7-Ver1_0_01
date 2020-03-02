/***************************************************************************
 Two Wire Interface driver January, 2012
 (c) 2012 Eric Williams

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
****************************************************************************/

//#include <avr/io.h>
//#include <avr/pgmspace.h>
#include <string.h>
#include <stdint.h>
#include "twi.h"
#include "define.h"
#include "sfr_defs.h"
#include "uart.h"

//#include "util.h"

#define W 0
#define R 1

/* TWI status codes */
#define SR_START        0x08    /* START condition has been transmitted */
#define SR_RSTART       0x10    /* Repeated START has been transmitted */
#define SR_SLA_WA       0x18    /* SLA+W has been transmitted, ACK received */
#define SR_SLA_W        0x20    /* SLA+W has been transmitted, no ACK */
#define SR_DTA_A        0x28    /* Data byte has been transmitted, ACK received */
#define SR_DTA          0x30    /* Data byte has been transmitted, no ACK */
#define SR_BUS          0x38    /* Arbitration lost */
#define SR_SLA_RA       0x40    /* SLA+R has been transmitted, ACK received */
#define SR_SLA_R        0x48    /* SLA+R has been transmitted, no ACK */
#define SR_DTR_A        0x50    /* Data byte received, ACK returned */
#define SR_DTR          0x58    /* Data byte received, no ACK */

extern void SetAlarmTiming(unsigned int NewVal);

extern void rtc_set_time(unsigned char hour,unsigned char min,unsigned char sec);  
 extern void rtc_set_date(unsigned char date,unsigned char month,unsigned year);
extern  void RTCrePwr (void); 
extern void RTC_RESET(void);
extern char cuurent_interval;  
extern bit IsAlertNeeded;  
extern bit Illigal_Time_Val;
extern bit UpdateSession; 

bit TWI_err_alert_needed = FALSE;
bit TWIerrON = FALSE;  

char Handle_TWI_err(void); 

static inline void twiStop (void);
 


/*
** Wait for TWI hardware to finish
*/

#define twiWait() {loop_until_bit_is_set (TWCR, TWINT);} 

//radio modul2 interface
void SDA_ON(void)   {SETBIT(PORTF,2);};     //sensor 3 data
void SDA_OFF(void)  {CLRBIT(PORTF,2);}; 
void SDA_OUT(void)  {SETBIT(DDRF,2);};  //PC0 = output
void SDA_IN(void)   {CLRBIT(DDRF,2);}; //PC0 = input = high  

//-------------radio commands-------------------------------
//ON 1 : 0011 1111 0000 0011 0000 0000 //4129536
char RADIO_PIN_ON[] = {0x3F,0x03,0x00};
//ON 2 : 0000 1111 0000 0011 0000 0000 //983808
//ON 3 : 0011 0011 0000 0011 0000 0000 //3343104
//ON 4 : 0000 0011 0000 0011 0000 0000 //197376
//OFF 1 : 0011 1111 0000 0000 0000 0000 //4128768
char RADIO_PIN_OFF[] = {0x3F,0x00,0x00};
//OFF 2 : 0000 1111 0000 0000 0000 0000 //983040
//OFF 3 : 0011 0011 0000 0000 0000 0000 //3342336
//OFF 4 : 0000 0011 0000 0000 0000 0000 //196608
//with "0" is 240us on, 740us off and "1" is 740us on, 240us off
//------------------------------------------------------------
/*
** Generate error message that shows expected and received status
*/
//char twi_fmt[] PROGMEM = "twi %02x:%02x\r\n";
//char twi_fmt[] = "twi %02x:%02x\r\n";

//void twiError (uint8_t expected, uint8_t received)
//{    
//
//  char ok;
//  expected = 0;
// //  received= 0;
//
//    if( TWIerrON == 0)
//    {    
//         TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);  //disable twi  
//         SendDebugMsg("Handle TWI Error..\r\n\0");
//         TWIerrON = 1;
//         
//           SETBIT(DDRD, 0); //SCL output
//           CLRBIT(DDRD, 1);  //SDA input 
//           RTCrePwr(); 
//            
//           ok = Handle_TWI_err();  
//           if (ok)
//          {
//             rtc_set_time(0,0,0);
//             SetAlarmTiming(MEASURE_INTERVAL * cuurent_interval);
//             delay_ms(10); 
//          }
//    }
//    else  SendDebugMsg("TWI Error..\r\n\0");
//
//}  
//
//char Handle_TWI_err(void)
//{   
//    #define pSCL 0
//    #define pSDA 1 
//    #define TWI_PORT PORTD
//    char ok, i,j; 
//    
//    
////    TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);  //disable twi  
////    SETBIT(DDRD, 0); //SCL output
////    CLRBIT(DDRD, 1);  //SDA input 
//    
//  
//    for(i = 0; i< 20; i++)   //clock active
//    {                       
//      
//        SETBIT(TWI_PORT, pSCL);
//        delay_us(5); 
//        CLRBIT(TWI_PORT, pSCL);
//        delay_us(5);      
//    } 
//       
//    ok = TSTBIT(PIND,pSDA); //test if  pin SDA high
//    if (ok) 
//    {
//     //   SendDebugMsg("SDA high..!\r\n\0"); 
//        twi_master_init1(50);
//         twiStop(); 
//         return 1;
//    }   
//    else  SendDebugMsg("SDA low..!\r\n\0");
//    
//   return 0;
//}  

static inline void twiStop (void)
{
        TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

    
void twiError (uint8_t expected, uint8_t received)
{    

  char ok;
  expected = 0;
 //  received= 0;
   
      if(UpdateSession == FALSE)  // not while FOTA
      {
            if( TWIerrON == 0)
            {    
                 TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);  //disable twi  
                 SendDebugMsg("Handle TWI Error..\0"); 
                 PrintNum((long)received );
                 TWIerrON = 1;
                 
                   SETBIT(DDRD, 0);  //SCL output
                   CLRBIT(DDRD, 1);  //SDA input 
                   RTCrePwr(); 
                    
                   ok = Handle_TWI_err();  
                   if (ok)
                  {       
                           RTC_RESET()  ;
                          rtc_set_time(23,59,30); 
                          rtc_set_date(1,1,0);       
                         
                     //    SetAlarmTiming(MEASURE_INTERVAL * cuurent_interval);
                          delay_ms(10); 
                          TWI_err_alert_needed = TRUE; 
                          IsAlertNeeded = TRUE;   
                            Illigal_Time_Val= TRUE;      //dont send data blocks -no server error
                          InitVarsForConnecting();   //go to modem task
                  }
            }
            else  SendDebugMsg("TWI Error..\r\n\0");
       }
}  

char Handle_TWI_err(void)
{   
    #define pSCL 0
    #define pSDA 1 
    #define TWI_PORT PORTD
    char ok, i; 
         
    for(i = 0; i< 20; i++)   //clock active
    {                       
        CLRBIT(TWI_PORT, pSCL);
        delay_us(5);
        SETBIT(TWI_PORT, pSCL);
        delay_us(5);
       
    } 
       
    ok = TSTBIT(PIND,pSDA); //test if  pin SDA high
    if (ok) 
    {
      //  SendDebugMsg("SDA pin high.!\r\n\0"); 
        twi_master_init1(50);
         twiStop (); 
         return 1;
    }
    
   return 0;
}

/*
** Issue a START condition on the TWI bus
*/
static uint8_t twiStart (void)
{
        uint8_t stat;

        TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
        twiWait();
        stat = TWSR & 0xf8;
        if ((stat == SR_START) || (stat == SR_RSTART))
                return 0;
        twiError (SR_START, stat);
        return 1;
}

/*
** Issue STOP condition on TWI bus
*/

/*
** Write one byte of data
*/
static inline void twiWrite(const uint8_t data)
// void twiWrite(const uint8_t data)
{
        TWDR = data;
        #asm("cli")
        TWCR = _BV(TWINT) | _BV(TWEN);
      //  twiWait();
        while (!(TWCR & (1<<TWINT)));
         #asm("sei")
}

/*
** Read one byte of data
*/
static inline void twiRead(void)
{
        TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
        twiWait();
}

/*
** Read one byte without ACK
*/
static inline void twiReadNack(void)
{
        TWCR = _BV(TWINT) | _BV(TWEN);
        twiWait();
}

/*
** Write <data> into register <reg> of device at address <addr>
*/
int twiWriteReg(const uint8_t addr, const uint8_t reg, const uint8_t data)
{

    #asm("cli")
        if (twiStart() == 0) {
                twiWrite((addr & 0xfe) | W);
                if (TWSR != SR_SLA_WA)
                        twiError(SR_SLA_WA, TWSR);
                else {
                        twiWrite(reg);
                        if (TWSR != SR_DTA_A)
                                twiError(SR_DTA_A, TWSR);
                        else {
                                twiWrite(data);
                                if (TWSR != SR_DTA_A)
                                        twiError(SR_DTA_A, TWSR);
                                else {
                                        twiStop();
                                      #asm("sei")
                                       // return 0;
                                        return 1;
                                }
                        }
                }
        }



         #asm("sei")
        TWCR = 0;               /* Disable TWI */
        return -1;
}

/*
** Write N sequential bytes of data starting at register <reg>
*/
int twiWriteRegN(const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t *pdata)
//int twiWriteRegN(const uint8_t addr, const uint8_t reg, uint8_t n, void *pdata)
{
        if (twiStart() == 0) {
                twiWrite((addr & 0xfe) | W);
                if (TWSR != SR_SLA_WA)
                        twiError(SR_SLA_WA, TWSR);
                else {
                        twiWrite(reg);
                        if (TWSR != SR_DTA_A)
                                twiError(SR_DTA_A, TWSR);
                        else {
                                while (n > 0) {
                                   twiWrite(*(uint8_t *)pdata++);

                                        if (TWSR != SR_DTA_A) {
                                                twiError(SR_DTA_A, TWSR);
                                                TWCR = 0;
                                                return -1;
                                        }
                                        n--;
                                }
                                twiStop();
                              //  return 0;
                                return 1;
                        }
                }
        }
        TWCR = 0;               /* Disable TWI */
        return -1;
}

int twiWriteMemN(char block, unsigned int addr, unsigned char n, char *pdata)

{
       unsigned char AddH, AddL;
       char MemAddress = 0xA0;

      if(block == 1)
      MemAddress = 0xA8;    //upper section bit high

        AddH = (unsigned char)((addr >> 8) & 0xFF);     //address high
        AddL  = (unsigned char)(addr) ;

        if (twiStart() == 0) {
                twiWrite((MemAddress) | W);
                if (TWSR != SR_SLA_WA)
                        twiError(SR_SLA_WA, 0x30);
                     //  delay_us(8);
                else {
                        twiWrite(AddH);
                        if (TWSR != SR_DTA_A)
                               twiError(SR_DTA_A, 0x31);
                             //  delay_us(8);
                        else {
                               twiWrite(AddL);
                               if (TWSR != SR_DTA_A)
                                twiError(SR_DTA_A, 0x32);
                               // delay_us(8);
                            else {

                                    while (n > 0) {
                                       twiWrite(*(uint8_t *)pdata++);
                                     //  delay_us(8);
                                            if (TWSR != SR_DTA_A) {
                                                    twiError(SR_DTA_A, 0x33);
                                                    TWCR = 0;
                                                    return -1;
                                            }
                                            n--;
                                    }
                                    twiStop();

                                    return 1;
                                 }
                      }
                }
        }
        TWCR = 0;               /* Disable TWI */
        return -1;
}
//new for eeprom write. replace SendBuf() at eeprom.c
//*pdata include address of page
int twiWriteEEP(const char addr, char n, char *pdata)

{
        #asm("cli")
        if (twiStart() == 0)
         {
                twiWrite((addr & 0xfe) | W);
                if (TWSR != SR_SLA_WA)
                    twiError(SR_SLA_WA, TWSR);
                else
                {
                        while (n > 0)
                        {
                           twiWrite(*(uint8_t *)pdata++);

                               if (TWSR != SR_DTA_A)
                               {
                                        twiError(SR_DTA_A, TWSR);
                                         #asm("sei")
                                        TWCR = 0;
                                        return -1;
                                }
                                n--;
                          }
                          twiStop();
                           #asm("sei")
                          //  return 0;
                            return 1;
                }
            }
         #asm("sei")
        TWCR = 0;               /* Disable TWI */
        return -1;
}

/*
** Read register <reg> from device at address <addr>
*/
int twiReadReg(const uint8_t addr, const uint8_t reg)
 {
        uint8_t dr;

      #asm("cli")
        if (twiStart() == 0) {
                twiWrite((addr & 0xfe) | W);
                if (TWSR != SR_SLA_WA)
                        twiError(SR_SLA_WA, TWSR);
                else {
                        twiWrite(reg);
                        if (TWSR != SR_DTA_A)
                                twiError(SR_DTA_A, TWSR);
                        else if (twiStart() == 0) {
                                twiWrite((addr & 0xfe) | R);
                                if (TWSR != SR_SLA_RA)
                                        twiError(SR_SLA_RA, TWSR);
                                else {
                                        twiReadNack();
                                        if (TWSR != SR_DTR)
                                                twiError(SR_DTR, TWSR);
                                        else {
                                                dr = TWDR;
                                                 #asm("sei")
                                                twiStop();
                                                return (dr);
                                        }
                                }
                        }
                }
        }

//    #ifdef AGRICULTURE_TASK
//           RTCrePwr();
//    #endif


         #asm("sei")
        TWCR = 0;               /* Disable TWI */
        return -1;
 }


/*
** Read N sequential bytes of data starting at register <reg>
**
** Note pdata must point to a buffer of sufficient size.
*/
int twiReadRegN(const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t *pdata)
{
        if (twiStart() == 0) {
                twiWrite((addr & 0xfe) | W);
                if (TWSR != SR_SLA_WA)
                        twiError(SR_SLA_WA, TWSR);
                else {
                        twiWrite(reg);
                        if (TWSR != SR_DTA_A)
                                twiError(SR_DTA_A, TWSR);
                        else if (twiStart() == 0) {
                                twiWrite((addr & 0xfe) | R);
                                if (TWSR != SR_SLA_RA)
                                        twiError(SR_SLA_RA, TWSR);
                                else {
                                        while (n > 0) {
                                                if (n == 1) {
                                                        twiReadNack();  /* Last byte */
                                                        if (TWSR != SR_DTR) {
                                                                twiError(SR_DTR, TWSR);
                                                                TWCR = 0;               /* Disable TWI */
                                                                return -1;
                                                        }
                                                } else {
                                                        twiRead();
                                                        if (TWSR != SR_DTR_A) {
                                                                twiError(SR_DTR_A, TWSR);
                                                                TWCR = 0;               /* Disable TWI */
                                                                return -1;
                                                        }
                                                }
                                             *(uint8_t *)pdata++ = TWDR;
                                                n--;
                                        }
                                        twiStop();
                                     //   return 0;
                                        return 1;
                                }
                        }
                }
        }
        TWCR = 0;               /* Disable TWI */
        return -1;
}
//read n chars from external eeprom
int twiReadEEP(const uint16_t InternalAddr, uint8_t n, uint8_t *pdata)
{

    unsigned char EEPaddr = 0xA0;
    unsigned char   adressHigh, adressLow;

    adressHigh = (unsigned char)((InternalAddr >> 8) & 0xFF);
    adressLow  = (unsigned char)(InternalAddr) ;

    delay_ms(50);

      #asm("cli")
        if (twiStart() == 0)
        {
                 twiWrite((EEPaddr & 0xfe) | W);   //A0

                if (TWSR != SR_SLA_WA)
                        twiError(SR_SLA_WA, TWSR);
                else
                {
                        twiWrite(adressHigh);      //high address
                       if (TWSR != SR_DTA_A)
                            twiError(SR_DTA_A, TWSR);
                       else
                       {
                            twiWrite(adressLow);   //low address  -wait for ack
                            if (TWSR != SR_DTA_A )
                            twiError(SR_DTA_A, TWSR);

                            else if (twiStart() == 0)    //new start bit
                            {
                                  twiWrite((EEPaddr & 0xfe) | R);    //read command
                                  if (TWSR != SR_SLA_RA)
                                  twiError(SR_SLA_RA, TWSR);
                             else
                                {
                                        while (n > 0)
                                        {
                                                if (n == 1)
                                                {
                                                        twiReadNack();  /* Last byte */
                                                        if (TWSR != SR_DTR)
                                                        {
                                                                #asm("sei")
                                                                twiError(SR_DTR, TWSR);
                                                                TWCR = 0;               /* Disable TWI */
                                                                return -1;
                                                        }
                                                }
                                                 else
                                                 {

                                                        twiRead();
                                                        if (TWSR != SR_DTR_A)
                                                        {
                                                                twiError(SR_DTR_A, TWSR);
                                                                 #asm("sei")
                                                                TWCR = 0;               /* Disable TWI */
                                                                return -1;
                                                        }
                                                 }
                                               *(uint8_t *)pdata++ = TWDR;
                                                n--;
                                        }

                                        twiStop();
                                     //   return 0;
                                        return 1;
                                }
                        }
                }
           }
        }
         #asm("sei")
        TWCR = 0;               /* Disable TWI */
        return -1;
}

//read 1 char from external eeprom AND SET INTERNAL ADDRESS
char twiReadEEP1Byte(char block,  unsigned int InternalAddr )
{

    unsigned char EEPaddr = 0xA0;
    unsigned char   adressHigh, adressLow;

     if(block == 1)
      EEPaddr = (0xA8);   //    //upper section bit high

     adressHigh = (unsigned char)((InternalAddr >> 8) & 0xFF);
     adressLow  = (unsigned char)(InternalAddr) ;


        if (twiStart() == 0)
        {
                twiWrite((EEPaddr & 0xfe) | W);   //A0
               // delay_us(10);
                if (TWSR != SR_SLA_WA)
                        twiError(SR_SLA_WA, 0x30);
                else
                {
                        twiWrite(adressHigh);      //high address
                       if (TWSR != SR_DTA_A)
                            twiError(SR_DTA_A, 0x31);
                       else
                       {
                            twiWrite(adressLow);   //low address  -wait for ack
                            if (TWSR != SR_DTA_A )
                            twiError(SR_DTA_A, 0x32);

                            else if (twiStart() == 0)    //new start bit
                            {
                                  twiWrite((EEPaddr & 0xfe) | R);    //read command
                                  if (TWSR != SR_SLA_RA)
                                  twiError(SR_SLA_RA, 0x33);
                                  else
                                  {
                                            twiReadNack();  /* Last byte */
                                            if (TWSR != SR_DTR)
                                            {
                                                    twiError(SR_DTR, 0x34);
                                                    TWCR = 0;               /* Disable TWI */
                                                    return -1;
                                            }


                                        twiStop();
                                        return TWDR;
                                  }
                            }
                       }

                }

        }

        TWCR = 0;               /* Disable TWI */
        return -1;
}
char ReadEEPCurrentAddress(char EEPaddr)
{

        if (twiStart() == 0)
        {
              twiWrite (EEPaddr | 1);    //read command
                if (TWSR != SR_SLA_RA)
                 twiError(SR_SLA_RA, 0x39);
              else
              {
                    twiReadNack();      //read one byte
                    if (TWSR != SR_DTR)
                    {
                            twiError(SR_DTR, 0x34);
                            twiStop();
                            TWCR = 0;               /* Disable TWI */
                            return -1;
                    }
                    twiStop();
                    return TWDR;
              }
        }
         return -1;
}
//void twiInit (void)
//{
//        /* SCL freq = CPU clock/(16+2*TWBR*4^TWPS) */
//    // TWI initialization
//// Bit Rate: 20.035 kHz
//   TWBR=0x54;
//// Two Wire Bus Slave Address: 0x0
//// General Call Recognition: Off
//    TWAR=0x00;
//// Generate Acknowledge Pulse: Off
//// TWI Interrupt: Off
//    TWCR=0x04;
//    TWSR=0x00;
//}


//fropm test program
//void twiInit (void)
//{
//        /* SCL freq = CPU clock/(16+2*TWBR*4^TWPS) */
//      // TWI initialization
//// Bit Rate: 10.017 kHz
//TWBR=0xB0;
//// Two Wire Bus Slave Address: 0x0A
//// General Call Recognition: Off
//TWAR=0x14;
//// Generate Acknowledge Pulse: Off
//// TWI Interrupt: Off
//TWCR=0x04;
//TWSR=0x00;
//}

void twiInit (void)
{
        /* SCL freq = CPU clock/(16+2*TWBR*4^TWPS) */
      // TWI initialization
// Bit Rate: 10.017 kHz
TWBR=0x0A;
// Two Wire Bus Slave Address: 0x0A
// General Call Recognition: Off
TWAR=0x00;
// Generate Acknowledge Pulse: Off
// TWI Interrupt: Off
TWCR=0x44;   //04
//TWCR =0x00;
//TWSR=0x00;
}

void twi_master_init1(unsigned int bit_rate)
{
unsigned char d;

//twi_ready=true;
//twi_result=TWI_RES_UNKNOWN;
//twi_slave_rx_handler=NULL;
//twi_slave_tx_handler=NULL;

// 28042015_1
// ensure the pull-up resistors on the TWI bus pins are disabled
// so that 3.3V I2C devices can be used with 5V powered AVRs
// 24042012_1
SETBIT(_TWI_SDA_PORT_,_TWI_SDA_BIT_);     //WAS CLRBIT
SETBIT(_TWI_SCL_PORT_,_TWI_SCL_BIT_);
// 24012012_1
TWCR=0x00;
// set SCL bit rate
// SCL bit_rate = _MCU_CLOCK_FREQUENCY_/(16+2*TWBR))
//#ifdef TWPS0
// for processors with additional bitrate division
// SCL bit_rate = _MCU_CLOCK_FREQUENCY_/(16+2*TWBR*4^TWPS)
// TWPS0=0, TWPS1=0
TWSR &= ~(__BM(TWPS0) | __BM(TWPS1));
//#endif
d=(_MCU_CLOCK_FREQUENCY_/2000L)/bit_rate;
//d= (14745600L/2000L)/bit_rate;
if (d>=8) d-=8;
TWBR=d;
// enable TWI, TWI interrupt, slave address ACK, clear the TWINT flag
//TWCR=(TWCR & __BM(TWINT)) | __BM(TWEN) | __BM(TWIE) | __BM(TWEA);
TWCR=(TWCR & __BM(TWINT)) | __BM(TWEN) | __BM(TWEA);  

}
void Send_433_Byte(unsigned char sent_byte)
{

    unsigned char mask;

    // Set DATA for output
   
    for (mask = 0x80; mask; mask >>= 1)
    {    
        SDA_ON();
        if (sent_byte & mask)
        {          
          	delay_us(740);
            SDA_OFF();
            delay_us(240);            
        }   
        else 
        {   
           	delay_us(240);
            SDA_OFF();
            delay_us(740);                               
        }  
                       
     }
}

//send string for radio module
void  Send_433_Str(char length, unsigned char  *buffer)
{
    int current_byte;
    
    SDA_OUT();  //pin as output
    for (current_byte = 0; current_byte < length; current_byte++)
    {
        Send_433_Byte(buffer[current_byte]);
            
    } 
    SDA_IN();
}
/*
  CodeVisionAVR C Compiler

  (C) 1998-2001 Pavel Haiduc, HP InfoTech S.R.L.

  Philips PCF8583 I2C bus Real Time Clock functions

*/
//#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "util.h"
//#include <bcd.h>
#include <stdio.h>

//#include <avr/pgmspace.h> // to store the tables and strings in ROM
//#include <avr/eeprom.h>

#include "define.h"
#include "twi.h"
#include "rtc.h"
#include "util.h"
//#include "param.h"
//#include "ui.h"
#include "mytype.h"
#include "uart.h"




//const uint8_t AF = _BV(3); // RTC_CONTROL_2: alarm flag
//const uint8_t AIE = _BV(1); // RTC_CONTROL_1: alarm interrupt enabled
//const uint8_t RTC_ADDR = 0xD0;
//const uint8_t RTC_CLKOUT_DISABLED = (_BV(3) | _BV(4) | _BV(5));
//const uint8_t SF = _BV(4); // RTC_CONTROL_2: second interrupt flag
//const uint8_t SIE = _BV(2);// RTC_CONTROL_1: second interrupt enabled

extern bit Illigal_Time_Val;
extern bit ModemIsOn;
//                                      //
#define  dHOUR      12  //default values
#define  dMIN       0
#define  dDATE      1
#define  dMONTH     1
#define  dYEAR      14

char gMin;
  char gHour;
  char gDay;
  char gMonth;
  char gYear;
  char gWeekDay;
  char gSec;

extern eeprom unsigned char eConnectIntervalH; 
extern eeprom unsigned char eStartConnectionH;        //first connection hour
extern eeprom unsigned char eConnectionInDay;        //number on connectionsin a day
extern eeprom unsigned int eCurrentYear;
// eeprom unsigned int ePumpDuration ;

const  char sun[] = "Sun";
const  char mon[] = "Mon";
const  char tue[] = "Tue";
const  char wed[] = "Wed";
const  char thu[] = "Thu";
const  char fri[] = "Fri";
const  char sat[] = "Sat";
const  char* dayname[7] = {sun, mon, tue, wed, thu, fri, sat};

//const  char jan[] = "Jan";
//const  char feb[] = "Feb";
//const  char mar[] = "Mar";
//const  char apr[] = "Apr";
//const  char may[] = "May";
//const  char jun[] = "Jun";
//const  char jul[] = "Jul";
//const  char aug[] = "Aug";
//const  char sep[] = "Sep";
//const  char oct[] = "Oct";
//const  char nov[] = "Nov";
//const  char dec[] = "Dec";
//const  char* MonthList[12] = {jan, feb, mar, apr, may, jun, jul, aug, sep, oct, nov, dec};


//char clockBuf[7]; 		 //buffer for all clock operation need
unsigned int time_in_minutes;
char Str[30];

bit RTC_OK;
//extern bit Next_Wake_For_Server;

extern uint16_t UARTGetBin(void);
extern char cuurent_interval;

//extern unsigned int LastWakeInterval;
//extern uint8_t GetTimeFromUser(uint16_t *minutes, uint16_t *hours, uint8_t ampm_flg);


//RTC registers address
enum {
  RTC_CONTROL_1,       //0
  RTC_CONTROL_2,
  RTC_CONTROL_3,
  RTC_SECONDS,
  RTC_MINUTES,
  RTC_HOURS,
  RTC_DAYS,
  RTC_WEEKDAYS,
  RTC_MONTHS,
  RTC_YEARS,
  RTC_MINUTE_ALARM,     //10
  RTC_HOUR_ALARM,
  RTC_DAY_ALARM,
  RTC_WEEKDAY_ALARM,
  RTC_OFFSET,
  RTC_TMR_CLKOUT_CTRL,   //15
  RTC_TMR_A_FREQ_CTRL,
  RTC_TMR_A_REG,
  RTC_TMR_B_FREQ_CTRL,
  RTC_TMR_B_REG,
};

#define LENGTH 35



//PCF8523 rtc constants
//#define RTC_ADDRESS       0xD0        //PCF8523
#define RegControl_1      0x00        //address
#define RegControl_2      0x01        //address
#define RegClkOutCtrl     0x0F        //address
#define RegClkOutVal      0xBA        //set bit 7 to enable pulse int. set bit1 for count down mode
#define RegFreqCtrl       0x10        //address
#define RTC_1SEC_CLK      0x02       // 1 Hrz clock -1 sec res freq for timer clk
#define RTC_1MIN_CLK      0x03       // 1/60 Hrz clock -1 min res freq for timer clk
#define RegTmrVal         0x11       //address  M41T83
#define ClearRtcInt       0xBF       //control_2 & with reg read val.clear bit 6
#define EnTmrInt          0x02       //control_2 bit 1 =1
#define CountDownVal      0x01       //test
//#define DEFAULT_RTC_TIME      10    //10 min default val

//#define DEEP_SLEEP_FREQ    1



//unsigned RTCstatus;
//unsigned char DayOfWeek;
void ClearAF(void);
 void RTCrePwr (void);

extern const char* dayname[7];
char dow_tbl[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

// Calculate day of the week by Sakamoto's method, 0=Sunday
uint8_t dow(uint8_t y, uint8_t m, uint8_t d)
{
	uint16_t yy;

	yy = y + 2000 - (m < 3);
	//return (yy + yy/4 - yy/100 + yy/400 + pgm_read_byte(dow_tbl + m-1) + d) % 7;
    return (yy + yy/4 - yy/100 + yy/400 + dow_tbl[m-1] + d) % 7;
}


void RTC_Setup(void)
{
    unsigned char  data;
   //   OK = RTC_CDI_init();   //Test - set count down int every 5 sec-

     twiWriteReg(RTC_ADDRESS, RTC_TMR_CLKOUT_CTRL,0x38 ); //no clovkout pulses
 //    twiWriteReg(RTC_ADDRESS, RTC_CONTROL_2, 0);   //disable count down int

     twiWriteReg(RTC_ADDRESS,RTC_CONTROL_3, 0xA0);  // battery switch-over function is disabled
                                                    //- only one power supply (VDD);
                                                     //battery low detection function is disable
     twiWriteReg(RTC_ADDRESS,RTC_CONTROL_1, 0x082); //   //12.5pF load, Alarm int enable bit AIE
     data = twiReadReg(RTC_ADDRESS, RTC_CONTROL_1);
      if(data != 0x82)
      {
           SendDebugMsg("RTC init Failure.!\r\n\0");;
           return;
      }
    else
    {
       RTC_OK = TRUE;
     //  SendDebugMsg("RTC setup-alarm enabled..\r\n\0");
    }
 }


//Set RTC as WD
unsigned char Set_RTC_WD(void)    //count down int setting
{

     int data;

             twiWriteReg(RTC_ADDRESS, RTC_TMR_CLKOUT_CTRL,0x38 ); //no clovkout pulses
             twiWriteReg(RTC_ADDRESS, 0x10, 0x02);    //0x02 1 sec freq for timer clk

            delay_ms(5);
            twiWriteReg(RTC_ADDRESS, RegClkOutCtrl, 0xFC);   //add 0Fh -
            delay_ms(5);
            twiWriteReg(RTC_ADDRESS, RegTmrVal, 0);   //add 11h - 0-disable wd

            twiWriteReg(RTC_ADDRESS, RegControl_2, 0x04);    //en WD bit
            data = twiReadReg(RTC_ADDRESS, RegControl_2);             //0x02
              if(data != 0x04)
            {
                return 0;
            }
            delay_ms(10);
              SendDebugMsg("RTC WD set OK..!\r\n\0");
            return 1;
}


void RTC_RESET(void)
{
     int data;

       SendDebugMsg("Reset RTC..!\r\n\0");
     data =  twiWriteReg(RTC_ADDRESS,RTC_CONTROL_1, 0x58); //set ST bit
     if(data == -1)
     return;
   
      delay_ms(3);
      RTC_Setup();
        SendDebugMsg("RTC Setup..!\r\n\0");
}

 //test count down timer A int wake up
unsigned char RTC_CDI_init(unsigned int interval)    //count down int setting
{
  //   unsigned char temp;
     int data;

         //    temp = 0x02;          //0x02 1 sec freq for timer clk
             twiWriteReg(RTC_ADDRESS, 0x10, 0x02);
//              data = twiReadReg(RTC_ADDRESS, 0x10);             //0x02
//              if(data != 0x02)
//              return 0;

            delay_ms(10);
            twiWriteReg(RTC_ADDRESS, 0x0F, 0xFA);
            delay_ms(10);

            twiWriteReg(RTC_ADDRESS, 0x11, interval);   //interval in seonds -test 5 sec interval

            twiWriteReg(RTC_ADDRESS, RegControl_2, 0x02);    //EnTmrInt;
            data = twiReadReg(RTC_ADDRESS, RegControl_2);             //0x02
              if(data != 0x02)
            {
                return 0;
            }
            delay_ms(10);
              SendDebugMsg("RTC CD int-5 sec init end..!\r\n\0");
            return 1;
}


void Set_RTC_WD_Val(unsigned char WDtime)
{
       twiWriteReg(RTC_ADDRESS, RegTmrVal, WDtime);
       twiReadReg(RTC_ADDRESS, RegControl_2);
}

//set new value to count down timer
void SetRTCTiming (unsigned char interval, unsigned char clk_rate)
{
          unsigned char temp;

         if(clk_rate == 0)
            temp = RTC_1SEC_CLK;   ////0x02 1 sec freq for timer clk
         else
            temp = RTC_1MIN_CLK;               //0x03 - 1 min freq for timer clk
          twiWriteReg(RTC_ADDRESS, RTC_TMR_A_REG, temp);
         temp =  interval;
         twiWriteReg(RTC_ADDRESS, RTC_TMR_A_REG, temp);
}

unsigned char rtc_read_bcd(unsigned char addr)
{
        unsigned char data;
        #asm("cli")
        data = twiReadReg(RTC_ADDRESS, addr );
        #asm("sei")
        return bcd2bin(data);
}



unsigned char rtc_read_masked_bcd(unsigned char addr, unsigned char mask)
{
        unsigned char data;

        #asm("cli")
       data = twiReadReg(RTC_ADDRESS, addr);
        #asm("sei")
        return bcd2bin(data & mask);
}


void rtc_write_bcd(unsigned char addr,unsigned char data)
{
        unsigned char TempData;

        TempData =  bin2bcd(data);
        twiWriteReg(RTC_ADDRESS, addr, TempData);

}

// Phytech clock buf structure

//    clockBuf[0] = year
//	clockBuf[1] = month
//	clockBuf[2] = day
//	clockBuf[3] = day of week data
//	clockBuf[4] = hour
//	clockBuf[5] = minute
//	clockBuf[6] = second
void rtc_set_timeAll(char *clockBuf)
{

        rtc_write_bcd(RTC_MINUTES, clockBuf[5]);
        rtc_write_bcd(RTC_HOURS, clockBuf[4]);
        rtc_write_bcd(RTC_DAYS, clockBuf[2]);
        rtc_write_bcd(RTC_MONTHS, clockBuf[1]);
        rtc_write_bcd(RTC_YEARS, clockBuf[0]);


}

void rtc_set_time(unsigned char hour,unsigned char min,unsigned char sec)
{
    //    rtc_stop();
        rtc_write_bcd(RTC_SECONDS,sec);
        rtc_write_bcd(RTC_MINUTES,min);
        rtc_write_bcd(RTC_HOURS,hour);
   //     rtc_start();
}
 void rtc_set_date(unsigned char date,unsigned char month,unsigned year)
{

     // rtc_stop();
        rtc_write_bcd(RTC_DAYS,date);
        rtc_write_bcd(RTC_MONTHS,month);
        rtc_write_bcd(RTC_YEARS,year);
   //     rtc_start();
}

// Phytech clock buf structure

//    clockBuf[0] = year
//	clockBuf[1] = month
//	clockBuf[2] = day
//	clockBuf[3] = day of week data
//	clockBuf[4] = hour
//	clockBuf[5] = minute
//	clockBuf[6] = second
void rtc_get_timeAll(char *clockBuf)
{

        char str[40];

        //    SendDebugMsg("\r\nrtc_get_timeAll()\r\n\0");
             rtc_stop();

            clockBuf[0] = rtc_read_bcd(RTC_YEARS);//& 0x1F;      //BCD2BIN is built in
            clockBuf[1] = rtc_read_bcd(RTC_MONTHS);//& 0x1F;
            clockBuf[2] = rtc_read_bcd(RTC_DAYS);// & 0x3F;
            clockBuf[3] = rtc_read_masked_bcd(RTC_WEEKDAYS, 0x07);
            clockBuf[4] = rtc_read_masked_bcd(RTC_HOURS, 0x3F);
            clockBuf[5] = rtc_read_masked_bcd(RTC_MINUTES , 0x7F);
            rtc_start();


                sprintf(str,"\r\nRTC Time - %02d/%02d/%02d-%02d:%02d\n\r\0",clockBuf[0],\
                clockBuf[1],clockBuf[2],clockBuf[4],clockBuf[5]);
                UART_WriteMsg(str);
               if((clockBuf[0]< 17)||(clockBuf[0]> 20)||(clockBuf[1]>12)||(clockBuf[2]>31)||\ 
               
               (clockBuf[4]>23)||(clockBuf[5]>59))
               {
                  SendDebugMsg("\r\nIlligal Date/Time..!\r\n\0");                
                   Illigal_Time_Val = TRUE;                                  
               }
               else
               {
                   Illigal_Time_Val = FALSE;
                   time_in_minutes = ((int)clockBuf[4] * 60) + (clockBuf[5]);  
//                     sprintf(str,"time_in_minutes = %d\n\r\0", time_in_minutes);
//                    UART_WriteMsg(str);
               }

      rtc_start();

}

void rtc_get_time(unsigned char *hour,unsigned char *min, unsigned char *sec)
{
       unsigned char Sec, Min, Hour;
     //    unsigned char WD;
       rtc_stop();

      //  *sec  = rtc_read_bcd(RTC_SECONDS);;
        Sec  = rtc_read_masked_bcd (RTC_SECONDS, 0x7F);
        Min = rtc_read_masked_bcd(RTC_MINUTES , 0x7F);
        Hour = rtc_read_masked_bcd(RTC_HOURS, 0x3F);

        *sec  = Sec;
        *min = Min;
        *hour = Hour;
      //  WD = rtc_read_masked_bcd(RTC_WEEKDAYS, RTC_WEEKDAYS);

       rtc_start();
}

void rtc_get_date(unsigned char *day,unsigned char *month, unsigned char *year, unsigned char *weekday)
{
    rtc_stop();
	*day = rtc_read_bcd(RTC_DAYS);// & 0x3F;
	*month = rtc_read_bcd(RTC_MONTHS);//& 0x1F;
	*year = rtc_read_bcd(RTC_YEARS);//& 0x1F;
	*weekday = rtc_read_masked_bcd(RTC_WEEKDAYS, 0x07);
     rtc_start();
}

void rtc_get_dow(unsigned char *weekday){
	*weekday = rtc_read_masked_bcd(RTC_WEEKDAYS, RTC_WEEKDAYS);
}



//void RTCSetTime(void) //dddddddddddddeeeeeeeebbbbbbuuuuuuuggggg
//{
//	uint16_t minutes, hours, error;
//
//	error = GetTimeFromUser(&minutes, &hours, 0);
//	if(error==FALSE)
//		rtc_set_time( hours, minutes,0);
//
//
//}




void RTCSetDate(void)
{
     unsigned char day,month,error;
     unsigned int year,tmp, tmpY;
     char tmpYear[4];

      error = 1;
      day = month = 1;

      do{
	      SendDebugMsg("\n\rSet month: \0");
           tmp = UARTGetBin();
          if(tmp <= 12 &&  tmp > 0)
          {
              month = (unsigned char)tmp ;
		      error = FALSE;
          }
	      else
          {
		       SendDebugMsg("\r\n Illigal value.! should be in range 1-12.\r\n");
	      }
	   }while(error );

      do{
           SendDebugMsg("\n\rSet day of month: \0");
           tmp = UARTGetBin();
           if(tmp <= 31 && tmp > 0)
           {
               day = (unsigned char)tmp ; //fix <= 31....
               error = FALSE;
           }
           else
           {
               SendDebugMsg("\r\n Illigal value.!  should be in range 1-31.\r\n");
               error = TRUE;
           }
        }while(error );

      tmpY = eCurrentYear;
      itoa(tmpY,tmpYear);
      sprintf(Str, "\n\rSet year (%s): \0",tmpYear);
      UART_WriteMsg(Str);

      tmp = UARTGetBin();

       if(tmp == 0)
       {
            tmp = eCurrentYear - 2000;   //no input. use default
//            sprintf(Str, "\n\rDebug: def year-  %d \n\r\0",tmp);
//            UART_WriteMsg(Str);
       }
       else if (tmp >= 2013)
       {
           eCurrentYear  = tmp;
           tmp -= 2000;
       }
      else  error++;
      year = (unsigned char)tmp ;

//       sprintf(Str, "\n\rDebug: Set year to %d: \n\r\0",year);
//      UART_WriteMsg(Str);


       rtc_set_date(day,month, year);
     //set day of week
      rtc_write_bcd(RTC_WEEKDAYS, dow(year,month,day));
}

void RTCgetTime(void)
{
      unsigned char Sec, Min, Hour;
       char Str[LENGTH];

   //    SendDebugMsg("RTC get time\r\n\0");
//      SetMUX2PC();
      rtc_get_time(&Hour, &Min, &Sec);
      sprintf(Str, "Time: %02u:%02u:%02d \r\n",Hour,Min,Sec );
      UART_WriteMsg(Str);

      if((Hour > 23) || (Min > 59) || (Sec > 59))
      {
         SendDebugMsg("RTC time illigal...\r\n\0");
   //      RTCrePwr();
      }

     gMin = Min;
     gHour = Hour;
     gSec = Sec;
   //   gYear;
}

void RTCgetDate(void)
{
      unsigned char day, Mon, Year, WDay;

       char Str[LENGTH];
  //    SendDebugMsg("RTC get date\r\n\0");
//     SetMUX2PC();
      rtc_get_date(&day, &Mon, &Year,&WDay );
//      if(Year != 13){ // ????????????????????????????????????
//          SendDebugMsg("\nTime was not set!. Setting default time...\r\n\0");
//          rtc_set_time( dHOUR, dMIN, 0);        //default values
//          rtc_set_date( dDATE, dMONTH, dYEAR);
//          rtc_get_date(&day, &Mon, &Year,&WDay );
//      }

//      sprintf(Str, "Date: %s, %02u/%02u/%u\r\n",dayname[WDay],day, Mon, Year+2000 );
      sprintf(Str, "Date: %02u/%02u/%u\r\n",day, Mon, Year+2000 );
      UART_WriteMsg(Str);

       gDay =    day ;
       gMonth =  Mon;
       gYear =   Year ;
       gWeekDay = WDay;
}

// returns today's day (0-6): Sun=0, Wed=3, etc...
unsigned char RTCGetDOW(void)
{
    unsigned char dow;

//    SetMUX2PC();
    delay_ms(100);
	rtc_get_dow(&dow);
      sprintf(Str, "Today is: %s\r\n\0",dayname[dow] );
      UART_WriteMsg(Str);
	return dow;
}

// returns current time in minuts
unsigned int GetCurrentMinutsTime(void)
{
      unsigned char min, hour,sec;
      unsigned int tmpMin, tmpHour;
      char str[50];

    //  SetMUX2PC();
       rtc_get_time(&hour, &min, &sec);
       if((hour < 24) && (min <  60))   //no error reading time
      {
              tmpHour = (unsigned int)hour;
              tmpMin = (unsigned int)((tmpHour * 60) + min);
      }
      else
      {
          RTC_RESET();
          rtc_get_time(&hour, &min, &sec);
          if((hour < 24) && (min <  60))   //no error reading time
          {
                  tmpHour = (unsigned int)hour;
                  tmpMin = (unsigned int)((tmpHour * 60) + min);
          }
          else
          {
              tmpMin = 0; //??????????????????????????
              SendDebugMsg("\r\nRTC READ TIME FAILURE..!\r\n\0");
          }
      }
       sprintf(str, "RTC read: time [%02d:%02d]=%d min\r\n \0",hour, min,tmpMin);  //debug only
              UART_WriteMsg(str);
      return tmpMin;
}

void GetTimeFromMinutsTime(eeprom uint16_t *min_time, uint8_t *hour, uint8_t *min){
	//SendDebugMsg("GTFMT: *min_time= %d",*min_time);
	*min = *min_time%60;
	*hour = (*min_time - *min)/60;
	//SendDebugMsg("*min= %d, *hour=%d\n",*min,*hour);
}

void set_rtc_alarm(unsigned char day,unsigned char hour,unsigned char min, unsigned char DWeek )
{
     char temp;
//    int ok;
    //    rtc_stop();

        if(day == 0) day = 0x80;    //disable  day alarm
        rtc_write_bcd(RTC_DAY_ALARM, day );
        rtc_write_bcd(RTC_HOUR_ALARM, hour);
        rtc_write_bcd(RTC_MINUTE_ALARM, min);

        if(DWeek > 6) DWeek = 0x80;     //0-6 format
        rtc_write_bcd(RTC_WEEKDAY_ALARM, DWeek );

        temp = twiReadReg(RTC_ADDRESS, RTC_CONTROL_1);    //set AIE bit in CONTROL_1
        if(temp > -1)
        {
               temp |= 0x82;                                 //set AIE bit in CONTROL_1
               twiWriteReg(RTC_ADDRESS, RTC_CONTROL_1, temp);
          //  if(ok) SendDebugMsg("\r\nRTC-Alarm Set..\r\n\0");
        }
        else
        {
                 twiWriteReg(RTC_ADDRESS, RTC_CONTROL_1, 0x82);
                temp = twiReadReg(RTC_ADDRESS, RTC_CONTROL_1);
                if(temp == -1)
               SendDebugMsg("\r\nRTC-Alarm Setting failure..!\r\n\0");
        }  
        
        
}

void SetAlarmTiming2(unsigned int NewVal)
{
      unsigned int  MinCount;
      unsigned char tHour, tMin, tmp;
      char Str[15];

            RTCgetTime();               //set global vars of  hour minutes
            MinCount = (unsigned int)gHour * 60 + gMin; // calc offset from misnight by minutes

            if((NewVal == (MEASURE_INTERVAL * cuurent_interval)) || (NewVal == (MEASURE_INTERVAL * cuurent_interval * 2)))
            {
                  tmp = MinCount % NewVal;    //  get minutes value
                  MinCount = MinCount + ( NewVal - tmp);    //set new value - mincount+difference
            }
             else
             {
                   MinCount = MinCount + NewVal;
             }

             tHour =  MinCount / 60;          //calc hour & minutes new values
             tMin = MinCount % 60;

             if(tMin > 0)
             {
                  if((tMin - gMin) == 1)   //if Wake minute - current minute == 1
                  {
                      if(gSec > 50)       //current seconds: if must measure in 10 sec- delay it
                      tMin += NewVal ;       //if close to next minute add one interval to wake time

                      if (tMin == 60)
                      {
                          tMin = 0;
                          tHour++;
                      }
                  }
              }


             if( tHour == 24)
             tHour = 0;
        //  set_rtc_alarm (0x80, tHour, tMin, 0x80);        //set alarm-use hours and minutes only
           set_rtc_alarm (0x80,0x80, tMin, 0x80);        //set alarm-use hours and minutes only
 //--------->           LastWakeInterval = NewVal;           //set new value for next check


            if(tHour > 24)                   //pass midnight - update hour , day
            {
                tHour -= 24;
                rtc_set_time(tHour, tMin-1,0);   //in case time is out of range
            }

         if(ModemIsOn == FALSE)  //2 is WD value . dont show it
         {
              SetMUX2PC();
              sprintf(Str,"Next Wake-  [%02d:%02d]\n\r", tHour, tMin );
              UART_WriteMsg(Str);
          }
          ClearAF();         //reset int flag
}
//updated for MEASURE_INTERVAL wake up
void SetAlarmTiming(unsigned int NewVal)
{
      unsigned int  MinCount;
      unsigned char tHour, tMin, tmp1,i,nextConH; 
      unsigned int nVal;
      char Str[40];
            
            nVal = NewVal;
            RTCgetTime();               //set global vars of  hour minutes   
            if(gMin > 59) gMin = 0;
            MinCount = (unsigned int)gHour * 60 + gMin; // calc offset from misnight by minutes 
            tmp1 = MinCount;
            
//              sprintf(Str,"MinCount:NewVal = [%d-%d]\n\r\0", MinCount,nVal );
//                UART_WriteMsg(Str);   
//                
//                  sprintf(Str,"cuurent_interval= [%d]\n\r\0",  cuurent_interval );
//                UART_WriteMsg(Str);  
             
//            if(NewVal == (MEASURE_INTERVAL * cuurent_interval))//regular wake schedual
//            {
//                  tmp1 = MinCount % NewVal;    //  get minutes value to compare with NewVal  
//                  tmp2 = NewVal - tmp1;        //find how many minutes to next wake
//                  if(tmp2 > 1)                 //morew  1 minute interva = 0k
//                     MinCount = MinCount + tmp2;   
//                  else 
//                     MinCount = MinCount + tmp2 + NewVal; // if less than add i interval more
//            }
//             else
//             {
//                   MinCount = MinCount + NewVal; //uniq timming
//             } 

//              sprintf(Str,"nVal, cuurent_interval=  [%d, %d]\n\r\0",nVal, cuurent_interval );
//                UART_WriteMsg(Str); 
       
            if(nVal == (MEASURE_INTERVAL * cuurent_interval))
            {
                  tmp1 = MinCount % nVal;    //  get minutes value
                  MinCount = MinCount + ( nVal - tmp1);    //set new value - mincount+difference   
               //    SendDebugMsg("NewVal = (MEASURE_INTERVAL * cuurent_interval\n\r\0" );
                
            }
            else
            {      
                 //    SendDebugMsg("MinCount0 = \0");
                      PrintNum((long)MinCount); 
            
                   tmp1 = MinCount + nVal;
                   MinCount = tmp1;//MinCount + NewVal;  
                   
                  
            }  
       
             
//              sprintf(Str,"new MinCount = [%d]\n\r\0", MinCount );
//                UART_WriteMsg(Str);   
                
             tHour =  MinCount / 60;          //calc hour & minutes new values
             tMin = MinCount % 60;   
             
//              sprintf(Str,"Calculated hour, min=  [%02d:%02d]\n\r\0", tHour, tMin );
//                UART_WriteMsg(Str); 
             
//             if((tMin == 0) && (gMin > 58))   //dont wake in one min - like when clock updated backward  
//             {                
//                      tMin += NewVal ;                                       
//             } 
          

             if( tHour == 24)
             tHour = 0;   
             
             set_rtc_alarm (0x80, tHour, tMin, 0x80);        //set alarm-use hours and minutes only
         //  set_rtc_alarm (0x80,0x80, tMin, 0x80);        //set alarm-use hours and minutes only
 //--------->           LastWakeInterval = NewVal;           //set new value for next check


            if(tHour > 24)                   //wrong reading - update hour , day
            {
                tHour -= 24;
                rtc_set_time(tHour, tMin-1,0);   //in case time is out of range
            }

         if(ModemIsOn == FALSE)  //2 is WD value . dont show next comm
         {
              SetMUX2PC(); 
              
                 for (i = 0; i < eConnectionInDay; i++)
                {
                    //nextH = (int)startH + ((int)CycleInterval * i);
                     tmp1 = i * eConnectIntervalH;
                     nextConH = eStartConnectionH + tmp1;
                     if(nextConH  >= 24 )
                     nextConH -= 24; 
                    if( nextConH > gHour)
                    break;                     
                 }
 
              
                sprintf(Str,"Next Wake-  [%02d:%02d]\n\r", tHour, tMin );
                UART_WriteMsg(Str); 
                if( nextConH > gHour) 
                 sprintf(Str,"Next COMM - [%02d:00]\n\r\0", nextConH );                  
                UART_WriteMsg(Str);
          }
          ClearAF();         //reset int flag
}

//clear alarm int flag every occurance of int
void ClearAF(void)
{

       twiWriteReg(RTC_ADDRESS, RTC_CONTROL_2, 0x00);   //clear bit AF (3)
     // SendDebugMsg("Clear AF..\r\n\0");
}


void rtc_stop(void)
{
      unsigned char temp;


       temp = twiReadReg(RTC_ADDRESS, RTC_CONTROL_1);
        temp |= 0x20;             //set bit 5 to stop timer
        twiWriteReg(RTC_ADDRESS, RTC_CONTROL_1, temp);
}

void rtc_start(void)
{
        unsigned char temp;


        temp = twiReadReg(RTC_ADDRESS, RTC_CONTROL_1);
        temp &= 0xDF;
        twiWriteReg(RTC_ADDRESS, RTC_CONTROL_1, temp);
}



 //try to fix RTC locked -V6
 void RTCrePwr (void)
{

       #asm("wdr")
       V33_PWR_ON();
       delay_ms(100);
       DDRC.6 = 1;      //V6
       PORTC.6 = 1;

     delay_ms(3000);
       #asm("wdr");
     PORTC.6 = 0;

     SendDebugMsg("Reset RTC power..\r\n\0");
//     if(stat)
//     V33_PWR_OFF();

   //  #asm("jmp 0x0000")   //start program

}

 /*
//unsigned char rtc_get_status(void)
//{
//unsigned char status;
//RTCread(0,1, &status);
//RTCalarm = status & 0x02;  // check later if it's on
//return status;
//}



void rtc_start(void)
{
        unsigned char temp;

       RTCread(RegControl_1,1, &temp);
       temp &= 0xDF;
       RTCwrite(RegControl_1,1,&temp);
}



//void rtc_hold_off(void)
//{
//        RTCstatus = rtc_get_status();
//        RTCstatus&=0xbf;       //release for counting
//        RTCwrite(0,1,&RTCstatus);
//}

//void rtc_hold_on(void)
//{
//        RTCstatus = rtc_get_status();
//        RTCstatus|=0x40;            // set bit to latch data
//        }



//void rtc_alarm_off(void)
//{
//        RTCstatus = rtc_get_status();
//        RTCstatus&=0xfb;
//        RTCwrite(0,1,&RTCstatus);
//}



//void rtc_alarm_on(void)
//{
//        RTCstatus = rtc_get_status();
//        RTCstatus|=4;
//        RTCwrite(0,1,&RTCstatus);
//}





void rtc_write_word(unsigned char addr,unsigned data)
{
unsigned char TempData;
        TempData = (unsigned char) (data&0xff);
        RTCwrite(addr, 1, &TempData);
        TempData = (unsigned char)(data>>8);
        RTCwrite(++addr, 1, &TempData);
}






void rtc_get_time(unsigned char *hour,unsigned char *min,unsigned char *sec)
{
        rtc_hold_on();

        *sec = rtc_read_bcd(3);
        *min = rtc_read_bcd(4);
        *hour = rtc_read_bcd(5);
        rtc_hold_off();
}



void rtc_set_time(unsigned char hour,unsigned char min,unsigned char sec)
{
        rtc_stop();
   //     rtc_write_bcd(1,hsec);
       rtc_write_bcd(0x03,sec);
        rtc_write_bcd(0x04,min);
        rtc_write_bcd(0x05,hour);
        rtc_start();
}



void rtc_get_date(unsigned char *date,unsigned char *month,unsigned *year)
{
unsigned char dy,TempData ,TempData1, DOW;
unsigned int y1;
     //   rtc_hold_on();
        RTCread(5,1,&dy);
//dy=rtc_read(chip,5);
// *month=bcd2bin(rtc_read(chip,6)&0x1f);
        RTCread(6,1,&TempData);
        *month = bcd2bin(TempData & 0x1f); //convert MONTH to bin (5bits)
        DOW = TempData >> 5;              // take day (3 bits)
       // rtc_hold_off();
        *date = bcd2bin(dy&0x3f);        //ounert day to bin
        dy>>=6;                          //year section shifted (2 bits)
        RTCread(0x10,1,&TempData);       //read memory section for YEAR saved
        RTCread(0x11,1,&TempData1);
        y1 = (unsigned int)TempData | ((unsigned int)TempData1 << 8);
//y1=rtc_read(chip,0x10)|((unsigned) rtc_read(chip,0x11)<<8);
        if (((unsigned char) y1 & 3)!= dy) rtc_write_word(0x10,++y1);
        *year=y1;
        DayOfWeek = DOW;
}





// void rtc_set_alarm_time(unsigned char hour,
// unsigned char min,unsigned char sec)
//{
//
//        rtc_write_bcd(0x03,sec);
//        rtc_write_bcd(0x04,min);
//        rtc_write_bcd(0x05,hour);
//}

void rtc_get_alarm_time(unsigned char *hour,
unsigned char *min,unsigned char *sec,unsigned char *hsec)
{
*hsec=rtc_read_bcd(9);
*sec=rtc_read_bcd(0xa);
*min=rtc_read_bcd(0xb);
*hour=rtc_read_bcd(0xc);
}



void rtc_set_alarm_time(unsigned char hour,
unsigned char min,unsigned char sec,unsigned char hsec)
{
rtc_write_bcd(9,hsec);
rtc_write_bcd(0xa,sec);
rtc_write_bcd(0xb,min);
rtc_write_bcd(0xc,hour);
}


void rtc_get_alarm_date(unsigned char chip,unsigned char *date,
unsigned char *month)
{
*date=bcd2bin(rtc_read(chip,0xd)&0x3f);
*month=bcd2bin(rtc_read(chip,0xe)&0x1f);
}


void rtc_set_alarm_date(unsigned char chip,unsigned char date,
unsigned char month)
{
rtc_write_date(chip,0xd,date,0);
rtc_write_bcd(chip,0xe,month);
}


*/

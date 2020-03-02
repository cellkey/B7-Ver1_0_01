#include "define.h"
#include <string.h>

BYTE NoRecDataU0Cnt;
BYTE NoDataRecCnt;
BYTE rx1_buff_len;
char cntr;
char chksumBuf[4];
unsigned int Timer4Count;
//extern eeprom unsigned char eUnit_Type;

//BYTE cntr = 0;
BYTE bPORT = 0;
extern BYTE NextByteIndex;
extern char readClockBuf[];             //buffer for data reading from clock
//extern char tx_enable;        //enable tx new msg flag
extern char ComBuf[MAX_TX_BUF_LEN];
extern char RxUart0Buf[MAX_RX0_BUF_LEN];
extern char RxUart1Buf[MAX_RX_BUF_LEN];
//extern BYTE LedStatus[4];
//extern BYTE BlinkNum[4];
extern BYTE BytesToSend;
//extern BYTE triggerCnt;
extern unsigned int rx0_buff_len;
extern BYTE ModemResponse;
extern BYTE mainTask;
//extern BYTE useExtInt1; //water meter interrupt
//extern BYTE bQuickSleep;
extern BYTE nInt1Fired;
//extern BYTE lastByteIndex;
extern BYTE NotMonitored;
extern  char Timer0_Ticks;
extern  char LoopsCount;
extern BYTE modemCurTask;
extern BYTE modemCurSubTask;
extern int TimeLeftForWaiting;
extern bit bCheckRxBuf;
extern bit longAnswerExpected;
extern bit overFlow;
extern int heat_time;
extern int ServerResponseTimeOut;
extern bit bEndOfMeasureTask;
extern bit bWaitForModemAnswer;
extern bit bWaitForMonitorCmd;
extern bit bEndOfModemTask;
extern bit bNeedToWait4Answer;
extern bit bReset;
extern   bit UnitWaked;
extern bit ConnectedToServer;
extern bit Found200;  
extern  bit Found_200 ;      //if server com ended OK
//extern bit Found210;

extern bit  ServerComOn;
extern bit DataSent;
extern bit SeverResetCommand ;
extern bit ModemIsOn ;
extern bit Measure_Timer_Active;        //flag checked in TOF2
extern bit PumpActivated;
extern bit FirmwareUpdateTime ;
extern  bit CoverAlertSent;

extern bit BUF1_FULL;
extern bit BUF2_FULL;
extern bit Buf1_Read;
extern bit Buf2_Read;


extern unsigned int JasonLengh;
extern unsigned int nMaxWaitingTime;
extern unsigned int nextCompare;
extern  unsigned int wtrPulseCnt;
#ifdef BlueToothOption
extern int nMaxWaitBtResponse;
#endif BlueToothOption

//extern unsigned int wndSpdPulseCnt;
//extern unsigned int wtrPulseCnt;
extern unsigned int cpuWakeTimeCnt;
extern char rx_buffer1[];
extern unsigned char tx_wr_index1,tx_counter1;
extern unsigned char rx_wr_index1,rx_rd_index1,rx_counter1;

extern char WakeTest;
extern int timer0_count;       //count TOF2 int

bit MAGNET_SW_ON = FALSE;
bit MAGNET_SW2_ON = FALSE;
bit COVER_SW_ON = FALSE;
bit IsCoverAlertNeeded;
bit UpdateSession;
bit got_NO_CARRIER = FALSE;
bit Server_Error_msg = FALSE;
#ifdef NETIV_ALERT_UNIT
bit POWER_FAILURE = FALSE;
#endif


//#ifdef DebugMode

//#pragma used+
//
//void  _putchar1(char c)
//{
//    while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);
//    UDR1 = c;
//}
//#pragma used-
////#endif DebugMode

// Timer 0 overflow interrupt service routine
//mega664  = 72mS period
//mega2560 = 18mS period
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{

       TCNT0 = 0x70;      //// 10 mS for OVF mega2560- 14400 hrz
       if(Measure_Timer_Active == TRUE)
        {
           if(timer0_count > 0)
           timer0_count--;
           
        }
     //   else

       if (bWaitForModemAnswer)
        {
               if(rx0_buff_len > 3)  NoRecDataU0Cnt++;
                if (NoRecDataU0Cnt >= Timer0_Ticks )   //Timer0_Ticks
                {
                        bWaitForModemAnswer = FALSE;
                        bCheckRxBuf = TRUE;
                        longAnswerExpected = FALSE;
                        DISABLE_TIMER0();
                }
        }


}

 // Timer1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{

// Reinitialize Timer1 value
TCNT1H = 0xFA60 >> 8;
TCNT1L = 0xFA60 & 0xff;

  if(heat_time > 0)
    {
         heat_time--;
         cpuWakeTimeCnt++;
//         if (cpuWakeTimeCnt > 60000)
//         cpuWakeTimeCnt = 0;
    }

    if ((bWaitForModemAnswer) && (TimeLeftForWaiting > 0))
    {
        TimeLeftForWaiting--;
    }

     if ((ConnectedToServer == TRUE) && (ServerResponseTimeOut > 0))
    {
         ServerResponseTimeOut--;
    }
  //   TGLBIT(PORTH,7);
}


// Timer1 output compare A interrupt service routine - EVERY 100 ML SECOND
interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
   // BYTE i;
    //reset next interupt
    nextCompare += 0x5A0;       //1440 -clock freq 14400 mega2560
    OCR1AH = (unsigned char)(nextCompare >> 8  )& 0xFF;       // add high byte of 0x168 (0x1) to the high byte
    OCR1AL = (unsigned char) (nextCompare) & 0xFF;            // add low byte of 0x168 (0x68) to the low byte


    if(heat_time > 0)
    {
         heat_time--;
         cpuWakeTimeCnt++;
//         if (cpuWakeTimeCnt > 60000)
//         cpuWakeTimeCnt = 0;
    }

    if ((bWaitForModemAnswer) && (TimeLeftForWaiting > 0))
    {
        TimeLeftForWaiting--;
    }


     if ((ConnectedToServer == TRUE) && (ServerResponseTimeOut > 0))
    {
         ServerResponseTimeOut--;
    }

}


// Timer2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{

        NoDataRecCnt++;
        if (NoDataRecCnt > 12)       //mega2560
        {
              bWaitForMonitorCmd = FALSE;
              bCheckRxBuf = TRUE;
              DISABLE_TIMER2();
            //  LED1_ON;
        }
}
// Timer4 overflow interrupt service routine
interrupt [TIM4_OVF] void timer4_ovf_isr(void)
{
// Reinitialize Timer4 value
        TCNT4H=0xC666 >> 8;
        TCNT4L=0xC666 & 0xff;  
        
        if(Timer4Count)
        Timer4Count-- ;
// Place your code here

}


// USART0 Receiver interrupt service routine
interrupt [USART0_RXC] void usart0_rx_isr(void)
{
    BYTE LastRxByte;
    char status;   
 //   bit JasonOK = FALSE;
    status = UCSR0A;
    LastRxByte = UDR0;       //read uart data register

    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {
       
        if(UpdateSession == TRUE)    //start rceiving FOTA file
        {  
            RED_LED_ON;
               if( LastRxByte == ':')  //row start
               cntr++;         

              if(!BUF1_FULL)
              {
                   if( cntr == 2)  //complete recieved 1 row 
                   {
                           BUF1_FULL = TRUE;     //buffer holds one row
                           Buf1_Read = FALSE;   //buf1 not read yet
                           BUF2_FULL = FALSE;   //make buf2 available for next row                         
                           RxUart1Buf[rx1_buff_len++] =  LastRxByte;   //start second buffer
                            cntr = 1;   //counting ':'. we have one now   
                            
                            DISABLE_TIMER4();  
                           
                   }
                   else   RxUart0Buf[rx0_buff_len++] =  LastRxByte;
              }

              else if(!BUF2_FULL)  //500 ticks
             {

                   if( cntr == 2)
                   {
                           BUF2_FULL = TRUE;
                           Buf2_Read = FALSE;
                           BUF1_FULL = FALSE;
                           RxUart0Buf[rx0_buff_len++] =  LastRxByte;   //start first buffer
                           cntr = 1;  
                            
                               Timer4Count  = 1000; 
                             ENABLE_TIMER4(); 
                   }
                   else  RxUart1Buf[rx1_buff_len++] =  LastRxByte;
             }
        }
        else if(ServerComOn == TRUE)                 
        {       
        
              if((LastRxByte == '0') && ( RxUart0Buf[rx0_buff_len - 2] == ' ') && (RxUart0Buf[rx0_buff_len - 3] == '1' )) //look for got HTTP 1.1 200 or 400 ? 500?
              {
                 
                         if( RxUart0Buf[rx0_buff_len - 1]== '2')                         
                           Found200 = TRUE;
                          else 
                           Server_Error_msg = TRUE;
                  
              }

              else  if(LastRxByte == '{')   //look for 200{
             {
                    if (RxUart0Buf[rx0_buff_len - 3] == '2')
                    {
                               RxUart0Buf[0] = '2';
                              RxUart0Buf[1] = '0';
                              RxUart0Buf[2] = '0';
                              rx0_buff_len = 3;
                              longAnswerExpected = FALSE;
                              Found_200 = TRUE; 
                          //    JasonOK = TRUE;
                    }

             } 
                                                   
              else if(FirmwareUpdateTime == TRUE)   //start receiving file from server
              {                                     
                   if(LastRxByte == ']')   //look for patern like [2] as start of update file
                  {
                       if   (RxUart0Buf[rx0_buff_len - 2] == '[')   //pattern   <ABCD>[1]
                       {
                          
                                   UpdateSession = TRUE; 
                                
                                   RxUart0Buf[0] = '[';
                                   RxUart0Buf[1] = RxUart0Buf[rx0_buff_len-1];  //get file type: 1,2,3                                                             
                                
                                   chksumBuf[0] = RxUart0Buf[rx0_buff_len-7]; //char 1   save vertical chksum received in file header
                                   chksumBuf[1] = RxUart0Buf[rx0_buff_len-6]; //char 2
                                   chksumBuf[2] = RxUart0Buf[rx0_buff_len-5]; //char 3
                                   chksumBuf[3] = RxUart0Buf[rx0_buff_len-4]; //char 4
                                   UDR2 = LastRxByte;
                                   rx0_buff_len = 2;  //
                                   bCheckRxBuf = FALSE;  
                                   cntr = 0;
                              
                             
                       }
                  } 
              }
//              else if(LastRxByte == 'C')    //look for NO CARRIER msg
//              {
//                  if( RxUart0Buf[rx0_buff_len - 2] == 'O' )
//                  got_NO_CARRIER = TRUE;
//              }
               
              if(rx0_buff_len < MAX_RX0_BUF_LEN)  //dont allow over flow           
               RxUart0Buf[rx0_buff_len++] = LastRxByte; 
               else   rx0_buff_len = 0;
              
        }
        else RxUart0Buf[rx0_buff_len++] = LastRxByte;

//        if (longAnswerExpected)
//        {
//            if(rx0_buff_len >= MAX_RX0_BUF_LEN)
//            {
//                overFlow = 1;
//                rx0_buff_len = 0;
//                #asm("wdr")
//            }
//        }

         if(UpdateSession == FALSE)  //dont show when update file received
         {
            UDR2 = LastRxByte;
//            UDR3 = LastRxByte;
         }   

         ENABLE_TIMER0();
    }
    NoRecDataU0Cnt = 0;
    RED_LED_OFF;
}
 //mega2560
// USART1 Receiver interrupt service routine
interrupt [USART2_RXC] void usart2_rx_isr(void)
{
    char status, data;

    status = UCSR2A;  //mega2560
    data = UDR2;
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN)) == 0)
    {
        //set the msg into RxUart1Buf[] buffer
        if(rx1_buff_len >= MAX_RX_BUF_LEN)   //100 bytes???
            return; //exit the function
        RxUart1Buf[rx1_buff_len++] = data;
       rx_buffer1[rx_counter1++] = data; //buffer for GUI
//       if(rx_counter1 >= 20)
//       UCSR2B=0;     //stop rx
        UDR3 = data;     //show on J26
         ENABLE_TIMER2();
    }
    NoDataRecCnt = 0;      //inced at TMR2 OVF int

}

// USART1 Receiver interrupt service routine
//interrupt [USART1_RXC] void usart1_rx_isr(void)
//{
//    char status,data;
//
//
//    status = UCSR1A;
//    data = UDR2;
//        if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN)) == 0)
//        {
//              rx_buffer1[rx_wr_index1++] = data;
//               if (rx_wr_index1 == 64)
//               rx_wr_index1 = 0;
//               if (++rx_counter1 == 64)
//               {
//                  rx_counter1 = 0;
//              //    rx_buffer_overflow1 = 1;
//               }
//
//             PORTC.7 = 1;     //led
//             delay_ms(50);
//             PORTC.7 = 0;
//          }
//}

// USART3 Receiver interrupt service routine
interrupt [USART3_RXC] void usart3_rx_isr(void)
{
unsigned char status;
char data;
    status=UCSR3A;
    data=UDR3;



    status = UCSR3A;  //mega2560
    data = UDR3;
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN)) == 0)
    {
        //set the msg into RxUart1Buf[] buffer
        if(rx1_buff_len >= MAX_RX_BUF_LEN)   //100 bytes???
            return; //exit the function
        RxUart1Buf[rx1_buff_len++] = data;
       rx_buffer1[rx_counter1++] = data; //buffer for GUI
       if(rx_counter1 >= 20)
       UCSR2B=0;     //stop rx

         ENABLE_TIMER2();
    }
    NoDataRecCnt = 0;      //inced at TMR2 OVF int

}


  #ifdef NETIV_ALERT_UNIT 
// External Interrupt 3 service routine for wind speed
interrupt [EXT_INT3] void ext_int3_isr(void)
{
         bPORT = PIND;  
     //     DISABLE_CLOCK_INT(); 
       delay_ms(50);  
       DISABLE_EXT_INT3();
       if(!(TSTBIT(bPORT,3)))  ////Netiv Haasara- Power failure int
       {
          
           CLRBIT(EIMSK,3);
           POWER_FAILURE = TRUE;
           mainTask = TASK_WAKEUP;
           WakeTest = 8;
          
      } 
      else  //pin high-powe ok int
      {
        //  if(POWER_FAILURE == TRUE)  
        // if( CoverAlertSent == TRUE)
        //  mainTask = TASK_SLEEP;   //sleep and wake in 1 min  
          WakeTest = 11; 
//            LED1_ON; 
      }
        mainTask = TASK_POWER_FAILURE; // TASK_WAKEUP;
      
//           delay_ms(20);   
//           LED1_OFF; 
//           delay_ms(20);
//          LED1_ON;  
   
}
  #endif
  
// External Interrupt 1 service routine // water meter
interrupt [EXT_INT1] void ext_int1_isr(void)
{
  
}

// External Interrupt 2 service routine  // clock int
interrupt [EXT_INT2] void ext_int2_isr(void)
{
       #asm("cli");
     DISABLE_CLOCK_INT();
    if ( ModemIsOn == TRUE)   //modem WD active
    {
     //   ModemIsOn = FALSE;
        mainTask = TASK_MODEM;
        modemCurTask = TASK_MODEM_CLOSE ;
        modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;  //force closing session
        ModemResponse = TASK_COMPLETE;   //dont change
    }
    else
    {

       DISABLE_PB0_INT();   //disable this int until handled
       DISABLE_PB3_INT();  
   #ifndef WATER_METER_ACTIVE
       DISABLE_PJ2_INT();
   #endif
     //   DISABLE_EXT_INT1();
        mainTask = TASK_WAKEUP;
        WakeTest = 2;

 //   PRR_ALL_ON();
    }
      LED1_ON; 
      #asm("sei");
}



//Magnetic SW1 or 2 activated by user
//magnetic SW 1 PB3
//magnet SW N0. 2 int PB0

interrupt [PC_INT0] void pin_change_isr0(void)
{
      #asm("cli");
       bPORT = PINB;        //save port satus
       
       DISABLE_CLOCK_INT();
       DISABLE_PB0_INT();   //disable this int until handled
       DISABLE_PB3_INT(); 
       DISABLE_PB5_INT();  //Netiv Haasara- Power failure int
    #ifndef WATER_METER_ACTIVE
       DISABLE_PJ2_INT();
   #endif 
     #asm("sei");
       LED1_ON;
       delay_ms(20); 
        
//    #ifdef NETIV_ALERT_UNIT    
//       if(!(TSTBIT(bPORT,5)))  ////Netiv Haasara- Power failure int
//       {
//
//           POWER_FAILURE = TRUE;
//           mainTask = TASK_WAKEUP;
//           WakeTest = 3;
//       // LED1_ON;
//       } 
//        else 
//     #endif
         
      if(TSTBIT(bPORT,3))  //MS1 active
       {

           MAGNET_SW_ON = TRUE;
           mainTask = TASK_WAKEUP;
           WakeTest = 3;
       // LED1_ON;
       }
       else if(TSTBIT(bPORT,0))   //MS2
       {
             if(MAGNET_SW2_ON == TRUE)  //if already in measuring loop
             LoopsCount = 0 ;           //stop loop
             else
             MAGNET_SW2_ON = TRUE;      //start loop

               if(UnitWaked == FALSE)   //cae here from sleep
               {
                    mainTask = TASK_WAKEUP;
                    WakeTest = 6;
               }
       }
        else  mainTask = TASK_SLEEP;
//        delay_ms(20);
//        LED1_OFF;
}


// Pin change 8-15 interrupt service routine
//portJ.2 =Cover SW
interrupt [PC_INT1] void pin_change_isr1(void)
{
       #asm("cli"); 
        if(UnitWaked == TRUE)
        delay_ms(5);  
        
       bPORT = PINJ; 
             
       DISABLE_PB0_INT();   //magnet 2
       DISABLE_PB3_INT();      
       DISABLE_CLOCK_INT(); 
       DISABLE_PJ2_INT();    //disable current int
                          
    #ifdef WATER_METER_ACTIVE     
    
              if (!(TSTBIT(bPORT,2)))  //water meter version 
              {                       
                   if(UnitWaked == FALSE)   //cae here from sleep
                   {    
                         LED1_ON;  
                        mainTask = TASK_WAKEUP;
                          WakeTest = 9;
                   } 
                   else   //int while wake-inc counter here and continue
                   {
                         wtrPulseCnt++;   //inc counter 
                   }
             } 
     #else 
           if(UnitWaked == FALSE)   //cae here from sleep           
            if (TSTBIT(PINJ,2))  //regular                  
           {   
               COVER_SW_ON = TRUE;
             //  mainTask = TASK_WAKEUP;
                WakeTest = 9;  
                mainTask = TASK_WAKEUP;
           } 
      
   #endif          
      #asm("sei");
}




//void TransmitBuf(unsigned char* ComBuf, unsigned char BytesToSend, char iPortNum)
void TransmitBuf(char iPortNum)
{

    BYTE num = BytesToSend;     //global var

//    if (iPortNum == 0)
//	    while (bWaitForModemAnswer == TRUE); //wait until rx0 end
//    if (iPortNum == 1)
//	    while (bWaitForMonitorCmd == TRUE); //wait until rx1 end

	NextByteIndex = 0;	// reset for Tx
	if (iPortNum == 0)
    {

        while(BytesToSend-- )
        {
            // wait for UART's shift register to finish sending byte
		    while(( UCSR0A & DATA_REGISTER_EMPTY)==0);
            UDR0 = ComBuf[ NextByteIndex++ ];// send next byte..

        }
        rx0_buff_len = 0;
        PRR0_EN_TMR0();           // allow timer0 in PRR

        if(ConnectedToServer == TRUE)      //ver 54
        {
             ServerResponseTimeOut = 120;
             #asm ("wdr");
        }

    }
    if ((iPortNum == 1) || ((iPortNum == 0) && (!NotMonitored)))
    {

        BytesToSend = num;
        NextByteIndex = 0;	// reset for Tx

        while(BytesToSend-- )
        {
            while ((UCSR2A & DATA_REGISTER_EMPTY)==0);
            UDR2 = ComBuf[ NextByteIndex++];// send next byte..

        }
        rx1_buff_len = 0;
        rx_counter1 = 0;

    }

	NextByteIndex = 0;//prepare for Rx
  //  ModemResponse = NO_ANSWER;

    if (iPortNum == 0)
    {
	    TimeLeftForWaiting = nMaxWaitingTime;    //TimeLeftForWaiting decremented in comp int timer 1
        NoRecDataU0Cnt = 0;
        ModemResponse = NO_ANSWER;

        if (bNeedToWait4Answer == TRUE)
        {
              bWaitForModemAnswer = TRUE;
        }
        else
            bWaitForModemAnswer = FALSE;

    }
    else
    {
        if(mainTask == TASK_MONITOR)
        {
            TimeLeftForWaiting = MAX_WAIT_MNTR_SEC * 10;
            bWaitForMonitorCmd = TRUE;
            NoDataRecCnt = 0;
        }
    }
    bCheckRxBuf = FALSE;
 if ((iPortNum == 0) && (NotMonitored))   //added 241015- eliminate sending data blocks to PC when not needed
     NotMonitored = FALSE;
    // enable timer of receive
}


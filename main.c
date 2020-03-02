/*******************************************************
This program was created by the
CodeWizardAVR V2.60 Standard
Automatic Program Generator
© Copyright 1998-2012 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Crecell Logger
Version : 9.4.0.0
Date    : 27/01/2013
Author  : Dan G
Company : Creacell
Comments:


Chip type               : ATmega1280
Program type            : Application
AVR Core Clock frequency: 14.745600 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*******************************************************/


#include <stdio.h>
#include <i2c.h>
#include <iobits.h >

#include "define.h"
#include "vars.h"
#include "IOexpender.h"
#include "uart.h"
#include "rtc.h"
#include "ui.h"

// 1 Wire Bus interface functions
//#include <1wire.h>
// DS1820 Temperature Sensor functions
//#include <ds1820.h>

//I2C bits definition
//#asm
//   .equ __i2c_port=0x0B ;PORTD mega1280
//   .equ __sda_bit=1
//   .equ __scl_bit=0
//#endasm                                                      333
//extern eeprom char unique_id[]; //sensors id

//------------Program version FOR G3-------------------
 // flash  unsigned char RomVersion[2] = {223,0};   
   flash  char RomVersion[] = "1.0.02";
//------------------------------------------------

// eeprom struct PolutionVal
//               {
//                    char InPlay;
//                    int SensVals[2];
//               }  ValsArr[2] = {{0,1,2},{3,4,5}};
//
// eeprom struct ValsArr[2] @0x125
//int GlobalRamLocation @0x500;

extern char gYear;
//extern char ALertComCounter;


extern void MUX_SDI12_00(void);
extern int twiWriteReg(const uint8_t addr, const uint8_t reg, const uint8_t data);

extern void RTC_RESET(void);
extern int twiReadEEP(const uint16_t InternalAddr, uint8_t n, uint8_t *pdata);
extern void SendString(unsigned char *bufToSend, BYTE bufLen);
extern void rtc_set_time(unsigned char hour,unsigned char min,unsigned char sec);

extern void HandleCoverSW(void);
extern void PwrDwn(void);

extern void SendInfoBlock(void);
extern  void SaveErrorState(int ErrType);
extern void SendPostAckUpdate(char index);
extern void InitIOs(void);
extern void _putchar3(char c); 
extern unsigned int RxUpdateFile(void);
extern int ValidityCheck (unsigned int Saddress);
extern  char MemoryReadTest(unsigned int block, unsigned int start, unsigned int end);

extern unsigned char ds1820_devices;
extern  unsigned char ds1820_rom_codes[3][9];  //3 devices only 
extern char  WM_SilencCount;          

extern eeprom unsigned int logger_id;

//extern unsigned char GetStatusReg(unsigned char mask);
extern void SetStatusReg(unsigned char mask, unsigned char val);

//extern void LED2_OFF(void);
extern eeprom unsigned char ePUMP_HEAT_TIME;

extern  char LoopsCount;
extern unsigned int wtrPulseCnt;

extern  bit PumpActivationNeeded;
extern  bit bDemoMode;
extern bit IsAlertNeeded;
extern bit SeverResetCommand ;
extern bit MAGNET_SW_ON;
extern bit TWIerrON;
extern bit TestFlag;
extern bit CoverIsOpen ;
extern bit CoverAlertSent;
extern bit QuickSensorsCheckNeeded;
extern bit IsAlertActivated;
extern bit COVER_SW_ON;
extern bit IsCoverAlertNeeded;
extern bit ModemRepeatNeeded;
extern bit ConnectedToServer;
extern bit WLV_OVF ;
extern bit MAGNET_SW2_ON;
extern int ServerResponseTimeOut;
extern bit MODEM_GOT_IP;
extern bit Illigal_Time_Val;
extern bit TWI_err_alert_needed;
extern bit ModemIsOn;
extern bit GPS_data_needed;
extern bit ExtendedInfoBlockNeeded;
extern bit FirmwareUpdateTime;

extern unsigned int ErrorType;
extern char ModemRepeatCount;

#ifdef NETIV_ALERT_UNIT
extern  bit POWER_FAILURE;

#endif



extern  flash unsigned char MODEM_SHDN[];
extern eeprom int MAX_LIMIT[MAX_SEN_NUM];
extern eeprom int PUMP_MAX_LIMIT[MAX_SEN_NUM];
extern eeprom int MIN_LIMIT[MAX_SEN_NUM];
extern eeprom int PUMP_MIN_LIMIT[MAX_SEN_NUM];
extern eeprom  int Option_1[];
extern eeprom  char Option_2[];
extern eeprom char eFLAGS_STATUS;
extern eeprom unsigned char eUseCntrCode;
extern void SetModemPwrOff();
extern void SendATCmd(flash unsigned char *bufToSend);
extern void ModemReset(void);
extern  char  ReadSaveGPSSensor(void);

//int GlobalRamLocation @0x500;
char i, WakeTest = 0;
unsigned int AlarmTiming;
char WDactive = FALSE;
unsigned int Uid;
char PV;
unsigned int QuickMsrCount;
char  FlagsStatus;
char Str1[30];

 bit pBread_saved = FALSE;
bit PumpActivated;
bit LongSWActivation = FALSE;
bit new_flash_version = FALSE;
bit SuccessUpdate = FALSE;
bit Power_Failure_alert_needed = FALSE;
//char testStr[] = "\{\"61\":[0,144,0,150],\"62\":[750,875]\}";


//char params[] = "\{\"41\":10,\"62\":[428,442],\"60\":400,\"61\":[0,32768,0,32768]\}";

//extern eeprom int PUMP_ACTIVE_DURATION;
//char y,mo, d, h, M;

void main(void)
{
    char clockBuf[7],k; 
    char Str2[7];
    int check;
    // Global enable interrupts
 //   #asm("sei")

//	#asm ("wdr"); 		//reset the watchdog timer
//    WATCHDOG_ENABLE(); 	//set the watchdog
    //~~~~~~~~~~~check the startup reason~~~~~~~~~~~~~
    //the MCUSR should be set to 0x00 at the PowerDownSleep() function


    bExtReset = FALSE;
    bReset = FALSE;
    WDactive = FALSE;
    mainTask = TASK_NONE;
    TestFlag = FALSE;  
    
  
    // Reset Source checking  
    if (MCUSR & (1<<PORF))
   {
   // Power-on Reset
   MCUSR=0;
   // Place your code here
     k=1;
   }
else if (MCUSR & (1<<EXTRF))
   {
   // External Reset
   MCUSR=0;
   // Place your code here
     k=2;  
        // ModemRepeatNeeded = FALSE;
      //   ModemRepeatCount = 0;
         bExtReset = TRUE;
   }
else if (MCUSR & (1<<BORF))
   {
   // Brown-Out Reset
   MCUSR=0;
   // Place your code here
    k=3;
   }
else if (MCUSR & (1<<WDRF))
   {
   // Watchdog Reset
   MCUSR=0;
   // Place your code here
     k=4; 
     WDactive = TRUE;
   }
else if (MCUSR & (1<<JTRF))
   {
   // JTAG Reset
   MCUSR=0;
   // Place your code here
     k=5;
   }


      // init all IO's and vars
    SeverResetCommand = FALSE;

    InitProgram();
     PRR1_EN_UART2();           // allow uart2 in PRR1 - MONITOR
    LED1_ON;      //led
    delay_ms(10);
//   LED1_OFF;;
    V33_PWR_ON();  
    VOLTAGE_MULTIPLIER_ON();
    delay_ms(100);   
     ENABLE_UART2();  
     MUX_SDI12_00();
      delay_ms(300);
     //-------------------------------
//       PrintNum((long)MCUSR);
     	MCUSR = 0x00; //reset the reset source flags to 0
    //------------------------------
      SendDebugMsg("\r\n==========START===============\r\n\0");
 //    SendDebugMsg("\r\nWait, System Is In RESET Process...\r\n\0");
     SendDebugMsg("RESET source = \0");  
    if(k==1)  SendDebugMsg ("Power on reset\r\n\0");
    else if(k==2) SendDebugMsg ("Ext. reset\r\n\0"); 
   else if(k==3)  SendDebugMsg ("Bor reset\r\n\0");
   else if(k==4)  SendDebugMsg ("WD reset\r\n\0");
    else if(k==5)  SendDebugMsg ("JTAG Reset\r\n\0");



//---------------------------------------------------
   //    SendDebugMsg("\r\n\r\ System set clock...\r\n\0");
 //--------------------rtc---------------------------

   LED1_OFF;

     bDemoMode = FALSE;

      if(WDactive == TRUE)
      {
          SendDebugMsg("WD activated..!\r\n\0");
          WDactive = FALSE;
      }
  
         SendDebugMsg("\r\nCellular Data Logger by Creacell...\r\n\0");


   
//----------------------------------------------------
 //check external memory
    SendDebugMsg("Check Memory .\r\n\0");
 //  SendDebugMsg("Testing Ext.Memory..\r\n\0");
   check = twiReadEEP(0, 3, clockBuf);


    if(check == 1)
      SendDebugMsg("TEST-Read Memory ..OK\r\n\0");
      else
     SendDebugMsg("Read Memory FAILED - check..!\r\n\r\n\0");

     SendDebugMsg("\r\n\0");
    RTC_RESET();  
 
    if(TWIerrON == TRUE)
   {
        SendDebugMsg("\r\n\r\nRTC fault recovery...\r\n\0");
        goto MAIN_LOOP;
   }
//----------------------------------------------------


   //reset memory option
     CLRBIT( DDRE,2);        //mux a AS INPUT
    SETBIT( PORTE,2);       //input PU for level check
     delay_ms(10);

    check = TSTBIT(PINE,2);
    if (check == 0)   //if low reset memory
   {
        SendDebugMsg("Init Memory..\r\n\0");
        i = InitDataBlocks(1);     //reset memory - Data is LOST
        if(i) SendDebugMsg("InitDataBlocks OK\r\n\0");
       else   SendDebugMsg("InitDataBlocks failed\r\n\0");

        ResetPumpFlags();
        Option_2[0] = 0;          
        wtrPulseCnt = 0; 
        #ifdef DESHEN_WATER_METER     
        WM_SilencCount = 0;  
        #endif   
   }

     CLRBIT( PORTE,2);   //low
     SETBIT( DDRE,2);    //mux A output

    delay_ms(50);
    SendDebugMsg("\r\n\0");
  
     SendDebugMsg("\r\n-----------------------------\r\n\0");

    SendDebugMsg("Logger ID: \0");
    Uid = logger_id;
    PrintNum((long)Uid) ;
    //SendDebugMsg("\r\n\0");    
                 
   for(i = 0; i< 6; i++)
   Str2[i] = RomVersion[i];
   Str2[6] = '\0';
 
   sprintf(ComBuf,"Prog. Version: %s\n\r\0",Str2);
   UART_WriteMsg(ComBuf); 

//     sprintf(Str1,"Prog. Version:  [%03d.%02d]\n\r",RomVersion[0], RomVersion[1]);
//     UART_WriteMsg(Str1);
  
    SendDebugMsg("-----------------------------\r\n\0");
    SendDebugMsg("\r\n\0");
    QuickMsrCount = 0;
    
     sprintf(Str1,"COMM schedual- [%02d,%02d,%02d]\n\r\0",eStartConnectionH,eConnectionInDay,eConnectIntervalH );
     UART_WriteMsg(Str1);

 //-----------------------------------------------------------

    if(Option_2[0] == 1)      //240216- if failed to transmit data restor previous pointer to data
    {
       SendDebugMsg("\r\nReset - restoring pointers..\r\n\0");
            PV = ResetAllReadPointers();
             if(PV == TRUE)
             Option_2[0] = 0;
    }
      LoopsCount = 0 ;
      MAIN_LOOP:
      RTCgetDate();
      mainTask = TASK_SLEEP ;  
      
      #ifdef Evogen_Com_1Min  
             MODEM_GOT_IP = FALSE;
             Illigal_Time_Val= TRUE;      //dont send data blocks -no server error
            InitVarsForConnecting();
      #endif
      
        BatLevel_ON();       
       objToMsr = BATTERY;
       delay_ms(300);  
       AnalogSensRead();    
       BatLevel_OFF(); 
         
//   mainTask = TASK_SLEEP ;  
    
    FlagsStatus =  eFLAGS_STATUS; 
       
    if(FlagsStatus & MODEM_ON)    //reset while modem on 
    {     
         
           if(bExtReset == TRUE)  //reset while modem active
           {  
               rtc_get_timeAll (readClockBuf);     //get clock
               ErrorType = 15;   //
               SaveErrorState(ErrorType);      
               ModemRepeatNeeded = TRUE;  
               SendDebugMsg("Reset while Modem Is On..\r\n\0");
              
           }  
            SetStatusReg(MODEM_ON , 0);
           eFLAGS_STATUS =  FlagsStatus;
    }
  //  else SendDebugMsg("ModemIsOn == FALSE\r\n\0");

//modem again if reset while on
 //   ModemRepeatNeeded = FALSE;   //???????????????????????????????????????????????????????????????????????????????????? 
    
    if(ModemRepeatNeeded == TRUE)
    {
        if( ModemRepeatCount < 2)
        {
            ModemRepeatCount++; 
           //   SendDebugMsg("Modem Retry from main..\r\n\0");
            InitVarsForConnecting();
        }  
        else
        {
            ModemRepeatCount--;
        }
    }  

    //-------check if FOTA process ended-------------------------------------          
     if((eUseCntrCode== '@') || (eUseCntrCode == '?'))   //new firmware version detected
     {           
              if(eUseCntrCode == '@')    //update ok
              {
                  SendDebugMsg("\r\nSuccessful update. notifying server..\r\n\0");
                  new_flash_version = TRUE;
                  SuccessUpdate = TRUE; 
                   FirmwareUpdateTime = TRUE; 
               }
              else
              {
                    SendDebugMsg("\r\nUpdate Failed..notifying server..\r\n\0");
                     new_flash_version = TRUE;
                     SuccessUpdate = FALSE;
              }  
              eUseCntrCode = '#';          //reset flag of update
               i = InitDataBlocks(cuurent_interval);     //reset memory after used for fota  
            
//              if(i) SendDebugMsg("InitDataBlocks OK\r\n\0");
//               else   SendDebugMsg("InitDataBlocks failed\r\n\0");

              InitVarsForConnecting(); //prepare sever notification
     }  
  
   //------------------------------------------------------------------------- 
     
//       #ifdef GPS_INSTALED
//       check = ReadSaveGPSSensor(); 
//       if(check == 1)
//       {
//           GPS_data_needed = TRUE; 
//           InitVarsForConnecting(); 
//           SendDebugMsg("\r\nGPS read ended.\r\n\0");                               
//       }
//      
//       else
//       {
//          SendDebugMsg("\r\nGPS read failed.\r\n\0");
//             mainTask = TASK_SLEEP;
//       }
//      #endif   
    //    LED1_ON;                                                                                                  
     
//                                  mainTask = TASK_MEASURE;
//                                   msrCurTask = TASK_NONE;
//                                   objToMsr = SENSOR1;
//   rtc_set_date(13,10,19); 
//   rtc_set_time(10,14,35);
 //------------------------  main loop     ----------------------------------------------------
    while (1)
    {

        switch (mainTask)
        {
            case TASK_MEASURE:

                 PRR0_EN_TMR1();
              
                  ENABLE_PJ2_INT();         //enable Cover SW int again
              
             //    SendDebugMsg("\r\ntoMeasureMain()\r\n\0");
                 MeasureMain();
            //=================sleep after measure=====================
//                bEndOfMeasureTask = FALSE;     //sleep after measure
//                mainTask = TASK_SLEEP;    //testing SDI only-remove
             //===========================================
                if (bEndOfMeasureTask == TRUE)
                {
                    bEndOfMeasureTask = FALSE; 
                 //=============test meaure only remove===========================   
//                     mainTask = TASK_SLEEP;  
//                     break; 
                   //=============================================  
                     if (MAGNET_SW2_ON == TRUE )   //board version 5.0 - measure in loop for calibrations
                   {

                             if(LoopsCount > 0)
                             {
                                    delay_ms(50);
                                   mainTask = TASK_MEASURE;
                                   msrCurTask = TASK_NONE;
                                   objToMsr = SENSOR1;

                                   LoopsCount--;
                                   SendDebugMsg("\r\nMeasuring Loops left= \0");
                                   PrintNum((long)LoopsCount);
                             }
                             else
                             {
                                   MAGNET_SW2_ON = FALSE;
                                   mainTask = TASK_SLEEP;
                             }
                              InitDataBlocks(1);

                           break;
                    }

                    if (objToMsr == BATTERY)
                    {
                        if (bCheckBatr == 1)
                        {
                      //     SendDebugMsg("\r\nTo TASK_MODEM\r\n\0");

                            mainTask = TASK_MODEM;    // back to modem  Danny
                            bCheckBatr = 2;
                           break;
                            ///////////////test danny
                        }

                        else
                        if (bExtReset == TRUE)
                        {
                              PRR1_EN_UART2();           // allow uart1 in PRR
                                ENABLE_UART2();
                           //    mainTask = TASK_SLEEP;    // modem not active after reset - for debug only - remove!!!!!
                               mainTask = TASK_MONITOR;

                             //monitorCurTask = TASK_MONITOR_CONNECT;
                              bEndOfMonitorTask = TRUE;  //Daqnny 290115- end looking for monitor
                        }
                    }
                     else if((PumpActivationNeeded) && (PumpActivated == FALSE))
                     {
                        
                         SendDebugMsg("\r\nPump activation needed\r\n\0 ");
                      
                       //  PumpActivationNeeded = FALSE;
                         mainTask = TASK_MEASURE;    //activate  pump as sensor. Heat Time used
                         msrCurTask = TASK_NONE;
                         objToMsr = PUMP ;
                     }

                    else if (IsTimeToConnectGPRS())      //new 270515 - After measure check if time to server
                    {

                     //    mainTask = TASK_SLEEP;  //DEBBUGGGG only
                        InitVarsForConnecting();                     
                        SendDebugMsg("\r\nConnecting to Server..\r\n\0");                        
                    }
                    else  if(LoopsCount > 0)
                    {
                          LoopsCount = 0 ;
                    }

                    else
                        if (bExtReset == TRUE)
                        {

                            mainTask = TASK_MEASURE;
                            msrCurTask = TASK_NONE;
                            objToMsr = BATTERY ;
                         
                         //   SendDebugMsg("\r\nTASK_MEASURE in main()\0");
                  
                        }
                        else

                        {
                              #ifdef DEMO
                               if(bDemoMode)
                               {
                                     if (IsTimeToConnectGPRS())
                                    {
                                            InitVarsForConnecting();
                                          
                                            SendDebugMsg("\r\nTime to Server!\0");
                                          
                                    }
                                }
                                else
                                #endif
                                mainTask = TASK_SLEEP;

                                //bEndOfModemTask = FALSE;
                                if ((SendAlerts() == TRUE))
                                {
                                    //    InitVarsForConnecting();     //demo - no immidiate connection to sever (future?)
                                    if(PumpActivationNeeded)                      //demo if pump activation needed
                                    {

                                         if ( PumpActivated == FALSE)             //first time after last reset of flag
                                           {

//                                              mainTask = TASK_MEASURE;    //mooved to case TASK_SLEEP
//                                              msrCurTask = TASK_NONE;
//                                              objToMsr = PUMP ;
//                                              PUMP_HEAT_TIME = ePUMP_HEAT_TIME ;

                                               for (objToMsr = SENSOR1; objToMsr < MAX_SEN_NUM; objToMsr++)
                                               {
                                                    AlertStatus[objToMsr] = ALERT_WAIT;
                                                    OutOfLmtCnt[objToMsr] = 0;
                                                }

//                                                 if(PINB.0 == 0)                  //demo- no jumper on JP10
//                                                 PumpActivated = TRUE;            //demo dont fill again until reset flag
//                                                 ALertComCounter = 0;
                                           }

                                     }

                                }
                         }
                }
                break; //case TASK_MEASURE
 
     //-------------------------------------------------------------------------------------
            case TASK_MODEM:
       //--------------------------attention---row 112----------------------
            //    PRR1_EN_UART2();           // allow uart1 in PRR
            //     ENABLE_UART2();      //check

               #ifdef NO_SERVER
               SendPostData();         //Danny- test-build data string to be sent to monitor
                bEndOfModemTask = TRUE ;   //Danny   - No modem activity
               mainTask = TASK_SLEEP; // Danny check
               #else
              //      SendDebugMsg("main-T0 ModemMain()\r\n\0"); 
                    
                   ModemMain();
               #endif

               if (bEndOfModemTask == TRUE)
                {
//                      //-------------test antenna-------------------
//                    if(LoopsCount > 0)
//                    {
//                        LoopsCount--;
//                        mainTask = TASK_MEASURE;
//                        msrCurTask = TASK_NONE;
//                        objToMsr = SENSOR1;     //
//                        bExtReset = TRUE;
//                      //  delay_ms(1000);
//                      break;
//                    }
//                    else
//                    {
//                  //--------------------------------------

                        mainTask = TASK_SLEEP;
                        bEndOfModemTask = FALSE;
                        DISABLE_UART0();
//                    }
                }

                 if (bCheckBatr == 1)      //go to measure battery when modem is on
                {
// //                    SendDebugMsg("\r\nModem ON- Measure Battery\r\n\0 ");
                    mainTask = TASK_MEASURE;
                    msrCurTask = TASK_NONE;
                    objToMsr = BATTERY;
                }

               //if modem shut off failed get out with our WD
                if((ConnectedToServer == TRUE) && (ServerResponseTimeOut == 0))
                {
                   SendDebugMsg("\r\nFailure - Modem Shut off in Main..\r\n\0 ");
//                   SendString("+++", 3);
//                     delay_ms(300);
//                   i =  RTC_Update_by_Modem();

                  #ifdef HE910
                    ServerResponseTimeOut = 100;          //shorter than WD
                    TimeLeftForWaiting = 100;
                     SendATCmd(MODEM_SHDN);  //HE910
                       delay_ms(2000);
                    #asm("wdr")
                    delay_ms(2000);
                    #asm("wdr")
                    delay_ms(4000);
                 #endif             
                 
                 #ifdef Evogen_Com_1Min   
                          SendDebugMsg("\r\nReset Modem..\r\n\0");
                          ModemReset();   //if new modem start, reset first
                  #endif                     
                 
                    SetModemPwrOff();         //shut modem pwr
                    mainTask = TASK_SLEEP;
                    bEndOfModemTask = FALSE;
                    ConnectedToServer = FALSE;  
                    ExtendedInfoBlockNeeded = FALSE;
                    ModemRepeatCount++;
                    DISABLE_UART0();
                }

                break; //case TASK_MODEM  
                
                 
    #ifdef NETIV_ALERT_UNIT   
       
            case TASK_POWER_FAILURE: 
                          
                if( WakeTest == 11)  //power up int
                {   
                     LED1_ON;
                     WakeTest = 0;
                 //    SendDebugMsg("\r\nPower GOOD int detected..\r\n\0");   
                     QuickSensorsCheckNeeded = TRUE;
                     mainTask = TASK_SLEEP;   //wake in 1 min  
                    EICRA=(1<<ISC31) | (0<<ISC30) | (1<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00); //set as falling edge
                     delay_ms(1000); 
                }
        
               else  if((POWER_FAILURE == TRUE) && (WakeTest == 8))
               {  
                       WakeTest = 0;
                       delay_ms(2000);  
                      k = TSTBIT(PIND,3); 
                      if(!k)
                      {   
                            //  UnitWaked = TRUE;
                              PRR0 = 0x00;           // allow TWI in PRR
                              PRR1 = 0;
                              delay_ms(100);                             
                              InitIOs();     //set ios
                              V33_PWR_ON();    //power unit peripherals
                             //re enable TWI
                          //   twi_master_init1(50) ;                          
                          
                              ENABLE_UART2();
                              delay_ms(300);
                             
                             SendDebugMsg("\r\nPower Failure detected..\r\n\0");
                             DISABLE_UART2();
                    
                      //    Buzer_it(1, 1, 1);  
                            InitVarsForConnecting(); //prepair modem activity   
                                      
                            Power_Failure_alert_needed = TRUE;   //15 27 FFF1 00 00 00 
                            EICRA=(1<<ISC31) | (1<<ISC30) | (1<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00); //change int to "rising edge" type
                         
                      } 
                //      else
                //      {
                //          SendDebugMsg("\r\nPower OK detected..\r\n\0"); 
                //      //    POWER_FAILURE = FALSE;
                //          BackToNormalAlertNeeded = TRUE; 
                //          EICRA=(1<<ISC31) | (1<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00); //change int to "falling edge" type  
                ////          InitVarsForConnecting(); //prepair modem activity 
                //          return;
                //      }
                //      mainTask = TASK_SLEEP;
                //     
                //      return;
               } 
               else  mainTask = TASK_SLEEP;
  
  
 

            break; 
             #endif     
    //-------------------------------------------------------------------------------------
            case TASK_SLEEP:
                      delay_ms(100);
                      LED1_ON;;

                if(SeverResetCommand == TRUE)
                {
                     SeverResetCommand = FALSE;
                     SendDebugMsg("\r\nReseting unit by Server..\r\n\0 ");
                     MCUSR = (1<<EXTRF);       //?????????
                     #asm("jmp 0x000")
                }


                        if(QuickMsrCount == 2)
                        {
                                QuickSensorsCheckNeeded =  FALSE;
                        }
                      
                         #ifdef Evogen_Com_1Min
                               QuickSensorsCheckNeeded = TRUE;   //measuring every minute 
                         #endif 

                          
                        if  (QuickSensorsCheckNeeded == TRUE)
                       {
                                
                                  AlarmTiming = 1;  // alert. measure next minute again  
                                  
                          #ifndef Evogen_Com_1Min    //NOT-regular handle                                                  
                                  QuickMsrCount++;
                                  if(QuickMsrCount == 3)
                                  {
                                        QuickSensorsCheckNeeded =  FALSE;
        //                                QuickMsrCount = 0;
                                  } 
                         #endif      
                       }
                       else
                       {
                             AlarmTiming = ((int)MEASURE_INTERVAL * cuurent_interval);
                             if(QuickMsrCount == 2)
                             QuickMsrCount = 0;
                       }
//                       sprintf(Str1,"mainl-AlarmTiming= [%d]\n\r\0",AlarmTiming );
//                       UART_WriteMsg(Str1);
                       SetAlarmTiming(AlarmTiming);      //Danny added - test every minute

                       if(TWIerrON == TRUE)
                        {
                               LED1_ON;     //led
                               delay_ms(50);
                               LED1_OFF;
                         }
                          TWIerrON = FALSE;


                       PowerDownSleep();
//                        delay_ms(100);
//                        SendDebugMsg("\r\nMain - after PwrDwn()\r\n\0");



                break; //case TASK_SLEEP

            case TASK_WAKEUP:

               //   SendDebugMsg("\r\nMain -to wakeup..\r\n\0");
                WakeUpProcedure();
             //  SendDebugMsg("\r\nMain - After wakeup..\r\n\0");
             break;

             case TASK_DEBUG:
                  while (1)
                        {
                           LED1_ON;
                           delay_ms(100);
                            LED1_OFF;
                             delay_ms(100);
                        }
             break;

            default:
                mainTask = TASK_SLEEP;
        }

          if (bReset == TRUE)
        {

            SendDebugMsg("\r\nbReset on..\r\n\0 ");

            bReset = FALSE;
            bExtReset = TRUE;
            mainTask = TASK_WAKEUP;
            WakeTest = 4;


        }
        #asm ("wdr"); 		//reset the watchdog timer
    }
}

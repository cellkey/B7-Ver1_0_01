//#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "define.h"
#include "uart.h"
#include <iobits.h>

void  ReadAndSaveComboSensor(void);
int Read_485_Sensor(char SensAddress);
void  ReadAndSaveTEMP_HUMIDSensor(void);
  void  ReadAndSave_TEMP_1W_Sensors(void);
//void  Read_save_WS_data(void);
//char  ReadSaveGPSSensor(void);

int iLastMsr[MAX_SEN_NUM];
char SensorDataType[MAX_SEN_NUM] = {0, 0, 0, 0, 0, 0, 0, 0}; //holding data type: 0 = value as is, 1 = 4-20  mA, 2 = digital

char SDI_Combo_index;
char SensorAtAlertStatus;
int DataAtAlertStatus;
char SensorBackToNormalAlertNeeded;

bit SDI_EC_MEASURED = FALSE;
bit SDI_PH_MEASURED = FALSE;
bit SDI_TMP_MEASURED = FALSE;
bit BackToNormalAlertEnabled = FALSE;
bit BackToNormalAlertNeeded = FALSE;
bit QuickSensorsCheckNeeded = FALSE;
bit IsAlertNeeded=0;
bit IsAlertActivated ;
bit PumpActivationNeeded;
bit WLV_OVF = FALSE;
bit Glob_data_Saved ;
bit NotStable =FALSE;
//bit SENS4_20 = FALSE;


extern  void PCmessage( char *message );
//extern void LED2_OFF(void);
//extern void LED2_ON(void);
//extern void twiInit (void);
//extern unsigned char GetStatusReg(unsigned char mask);
extern void SetStatusReg(unsigned char mask, unsigned char val);
extern void ActivateTimer1(void);
//extern char Read_Weather_Station_485(void);
extern void SetUart2_4800(void);
extern void Set_UART2_RS485_9600(void);
extern  void MUX_SDI12_11(void);



extern  int timer0_count;
extern char readClockBuf[];
extern void rtc_get_timeAll(char *clockBuf);
extern void MUX_SDI12_00(void);

extern BYTE mainTask;
extern BYTE NotMonitored;
extern int SDI_TEMP_DATA;
extern unsigned int SDI_EC_DATA;
extern unsigned int SDI_PH_DATA;
extern int SDI_ORP_DATA;
extern int SDI_SM_DATA;
extern int COMBO_TMP_DATA;
extern int COMBO_HUMID_DATA;

#ifdef WEATHER_STATION_485
extern unsigned int WS_UV_DATA;
extern unsigned int WS_WSPEED_DATA;
//unsigned int WS_LIHGT_DATA;
extern unsigned int WS_RAIN_DATA;
//bit WS_LOW_BAT_FLAG;
#endif



 unsigned int wtrPulseCnt;  
 
#ifdef DESHEN_WATER_METER  
 unsigned int Last_wtrPulseCnt;
 char  WM_SilencCount = 0;
 #endif
 unsigned char ds1820_devices;
 unsigned char ds1820_rom_codes[3][9];  //2 devices only

#pragma keep+

eeprom BYTE SensorType[MAX_SEN_NUM ] @0x00;
eeprom BYTE SensorType[MAX_SEN_NUM ]= {0, 0, 0, 0, 0, 0, 0, 0};

eeprom BYTE PortIndex[MAX_SEN_NUM ]  @0x08;
eeprom BYTE PortIndex[MAX_SEN_NUM ]  = {1, 2, 3, 4, 0, 0, 0, 0};

eeprom signed char ADCMux[MAX_SEN_NUM ] @0x10;
eeprom signed char ADCMux[MAX_SEN_NUM ] = { -1, -1, -1, -1, -1, -1, -1, -1};

eeprom unsigned char NumSensors @0x18;
eeprom unsigned char NumSensors = 1;     

eeprom BYTE IOType[MAX_IO_TYPES] @0x19;
eeprom BYTE IOType[MAX_IO_TYPES]= {1, 2, 3, 5, 4, 6, 7};  //1-EC, 2-TEMP, 3-PH, 5-WL

eeprom char unique_id[ MAX_SEN_NUM ] @0x20 ;
eeprom char unique_id[ MAX_SEN_NUM ] = {1,2,3,4, 5,6,7,8}; //sensors id

eeprom unsigned int logger_id  @0x28;
eeprom unsigned int logger_id = 0;

eeprom int MIN_LIMIT[MAX_SEN_NUM] @0x30;
eeprom int MIN_LIMIT[MAX_SEN_NUM] = {MIN_INT, MIN_INT, MIN_INT, MIN_INT, MIN_INT, MIN_INT, MIN_INT, MIN_INT};
eeprom int MAX_LIMIT[MAX_SEN_NUM] @0x40;
eeprom int MAX_LIMIT[MAX_SEN_NUM] = {MAX_INT, MAX_INT,MAX_INT,MAX_INT, MAX_INT, MAX_INT, MAX_INT, MAX_INT};
eeprom int PUMP_MIN_LIMIT[MAX_SEN_NUM] @0x50;
eeprom int PUMP_MIN_LIMIT[MAX_SEN_NUM] = {MIN_INT, MIN_INT,MIN_INT, MIN_INT, MIN_INT, MIN_INT, MIN_INT, MIN_INT};
eeprom int PUMP_MAX_LIMIT[MAX_SEN_NUM] @0x60;
eeprom int PUMP_MAX_LIMIT[MAX_SEN_NUM] = {MAX_INT, MAX_INT, MAX_INT, MAX_INT, MAX_INT, MAX_INT, MAX_INT, MAX_INT};
//eeprom float C1 = 0.00000000297;
//eeprom float C2 = 0.00000737;
//eeprom float C3 = 0.00669;
//eeprom float C4 = 1.92;
eeprom int HS_offset_value @0x70; //use this value for offset calibration (in 10 mili_grams, 500 = 5Kg)
//eeprom int HS_offset_value = 0; //use this value for offset calibration (in 10 mili_grams, 500 = 5Kg)
eeprom int HS_cal_value @0x72; //use this value as a gain multiplier (100 = 1.00)

//eeprom int HS_cal_value = 100; //use this value as a gain multiplier (100 = 1.00)

eeprom unsigned char ePUMP_HEAT_TIME @0x2E;
eeprom unsigned char ePUMP_HEAT_TIME = 90;
//char PUMP_HEAT_TIME;
//eeprom int eOption3;
//eeprom int eOption4;
eeprom char eBackToNormalAlertEnabled @0x0111;
eeprom char eBackToNormalAlertEnabled = TRUE;

eeprom char eRelay1Enabled  @0x0112;
eeprom char eRelay1Enabled = TRUE;

eeprom char eRelay2Enabled  @0x0113;
eeprom char eRelay2Enabled = TRUE;

eeprom char eTimeZone  @0x0114;
eeprom char eTimeZone = 12;   //Summer

eeprom   unsigned int eTotalRainMeter @0x012E ;  
eeprom   unsigned int eTotalRainMeter = 0;

extern eeprom char eFLAGS_STATUS;
extern eeprom  unsigned int eECTresholdVal;
extern eeprom char cpue2_interval_1;


#pragma keep-
extern char FlagsStatus;


//BYTE pinB0 = 0;
extern bit bEndOfMeasureTask;
extern bit bExtReset;
extern bit Relay1Enabled;
extern bit Relay2Enabled;

extern  bit BackToNormalAlertActivated;
extern bit Measure_Timer_Active;
extern bit PumpActivated;
extern bit bDemoMode;
extern  bit CoverAlertSent;

#ifdef NETIV_ALERT_UNIT
extern  bit POWER_FAILURE;
#endif


//extern char  flagsStatus;
extern BYTE msrCurTask;
extern BYTE objToMsr;
extern BYTE msrAlertFlag;
extern BYTE OutOfLmtCnt[MAX_SEN_NUM];
extern BYTE OutOfLmtCntPump[MAX_SEN_NUM];
extern BYTE AlertStatus[MAX_SEN_NUM];
extern int heat_time;
extern int SensorResult;       //save the measuring result into variable
extern int measure_time;
extern int iVoltage;
extern unsigned int time_in_minutes;     //time from day start in ninutes
extern unsigned int nextCompare;
extern unsigned int wndSpdPulseCnt;

extern unsigned int cpuWakeTimeCnt;
extern unsigned int  QuickMsrCount;
extern BYTE rx1_buff_len;


#ifdef DebugMode
extern char ComBuf[MAX_TX_BUF_LEN];
#endif DebugMode





void SensorPowerOn()
{
//    if (SensorType[objToMsr] == EC)     //danny
//        return;

//    #ifdef DebugMode
//    SendDebugMsg("\r\nPowerOn port \0");
//    PrintNum(PortIndex[objToMsr]);
//    #endif DebugMode
    if(objToMsr == PUMP)
    return;  
     if (SensorType[objToMsr] == Weather_Sation_485)
     return;

    switch (PortIndex[objToMsr])    //PortIdex eeprom address 0x47
    {
        case 1:
           SETBIT( PORTC,1 );   //open voltage switch for sensor 1
        break;
        case 2:
            SETBIT( PORTC,2 );
        break;
        case 3:
            SETBIT( PORTC,3 );
        break;
        case 4:
            SETBIT( PORTC,4 );
        break;
//        case 5:
//             PORTA.7 = 1;
//            break;

        default:

             
                SendDebugMsg("\r\nCheck EEPROM -Port Index Not Valid \0");
                PrintNum(PortIndex[objToMsr]);
                 SendDebugMsg("\r\nobjTomsr = \0");
                 PrintNum((long)objToMsr);            
                  mainTask = TASK_SLEEP;

             break;
    }

}

void SensorPowerOff()
{

      CLRBIT( PORTC,1); 
       CLRBIT( PORTC,2); 
        CLRBIT( PORTC,3); 
        CLRBIT( PORTC,4);
//    switch (PortIndex[objToMsr])
//    {
//        case 1:
//           CLRBIT( PORTC,1);
//        break;
//        case 2:
//             CLRBIT( PORTC,2);
//        break;
//        case 3:
//            CLRBIT( PORTC,3);
//        break;
//        case 4:
//             CLRBIT( PORTC,4);
//        break;
//
////        case 5:
////             PORTA.7 = 0;
////            break;
//
//        default:
//        break;
//    }
     VOLTAGE_MULTIPLIER_OFF();
     //all ADC off
     DIDR0=(1<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D) | (1<<ADC2D) | (1<<ADC1D) | (1<<ADC0D);
}

void InitAdc(void)
{


    //  analog inputs init
    if( objToMsr < 6)      //no pump
    {
       PRR0_EN_ADC();           // allow adc in PRR
    //   ENABLE_ADC();
        switch (PortIndex[objToMsr])
        {
            case 1:
               CLRBIT( DDRF,0);
               CLRBIT( PORTF,0);
               DIDR0=(1<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D) | (1<<ADC2D) | (1<<ADC1D) | (0<<ADC0D);
            break;
            case 2:

                CLRBIT( DDRF,1);
               CLRBIT( PORTF,1);
               DIDR0=(1<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D) | (1<<ADC2D) | (0<<ADC1D) | (1<<ADC0D);
            break;
            case 3:
               CLRBIT( DDRF,2);
               CLRBIT( PORTF,2);
               DIDR0=(1<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D) | (0<<ADC2D) | (1<<ADC1D) | (1<<ADC0D);
            break;
            case 4:
                CLRBIT( DDRF,3);
               CLRBIT( PORTF,3);
               DIDR0=(1<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) | (0<<ADC3D) | (1<<ADC2D) | (1<<ADC1D) | (1<<ADC0D);
            break;

//            case 5:
//                DDRA.4 = 0;
//                PORTA.4 = 0;       //Danny
//                break;
            case 6:
    //            wndSpdPulseCnt = 0; //reset wind speed pulse counter
    //            cpuWakeTimeCnt = 0;  //resettime counter
           break;

            default:
        }


        switch (PortIndex[objToMsr])
        {
            case 1:
               SETBIT( DDRF,1);     // analog 1 power
               CLRBIT( PORTF,1);
            break;
            case 2:
               SETBIT( DDRF,2);     // analog 2 power
               CLRBIT( PORTF,2);
            break;
            case 3:
                  SETBIT( DDRF,3);     // analog 3 power
               CLRBIT( PORTF,3);
            break;
            case 4:
                 SETBIT( DDRF,4);     // analog 4 power
               CLRBIT( PORTF,4);
            break;

            case 6:
            case 7:

            break;

            default:
        }
  }
}
//values are in 1/10 sec
void SetSensorHeatTime()
{
    switch (SensorType[objToMsr])
    {
        case AT:
        case ST5: 
        case AT_B: 
        case COMBO_TEMP_1W:        
             heat_time = 25 ;
        break;

        case AH:
            heat_time = 100;
        break;

//        case OXGN:
//             heat_time = 20;
//        break;

        case PH:     //3      PH kando
               heat_time = 120;
        break;
        case WLV:     //5      Level kando
           heat_time = 80;    //15 sec for kando  090316
        break;
        case WLV_PULSTAR:
             heat_time = 30;  //1 sec
        break;
          case  SM_4_20:         //Elchanani
               heat_time = 30;
        break;

         case DOROT_PRESSURE:        //4   EC-5-C-2 kando
               heat_time = 3;
         break;

        case TNS:
        case LT:  
        case DENDROMETER:    
            heat_time = 30;
        break;

        case TIR:

             heat_time = 5;
        break;

        case HS10:

               heat_time = 20; //30    Danny for 4-20  sens check
        break;

        case GS1:
             heat_time = 30;
        break;

           case H2S:      //37
             heat_time = 250;  //1 sec
        break;  
        
      case TEMP_HUMIDITY:
            heat_time = 100;    //70
       break;

    
        case EC_SDI12:
        case PH_SDI12:
        case COMBO4_SDI12:
               heat_time = 40;
        break;

        case COMBO4_485:
       case  COMBO_PH_485:
       case  COMBO_EC_485:
               heat_time = 50;
        break;


        case COMBO_5TE:        //sdi sensor 5TE
              heat_time = 25;  //2 sec
         break;

        case SCALE_SHKILA_TCS60:
               heat_time = 30;
        break; 
        
//        case Weather_Sation_485:
//              heat_time = 0;
//        break; 
        
       #ifndef Evogen_Com_1Min  
         case TRH_300:      //evogen Humidity 4-20mA
         case TRE_150:       //evogen temp 4-20mA
         case HD2021T:       //evogen radiation 4-20mA                      
                heat_time = 50; //0;  //direct voltage 12 V               
         break;  
       #endif  
       
        default:
            heat_time = 0;
    }

}

int AnalyzeSensorRealValue(int AtoDSensResult)
{
 
    int sr;
    int airTmp;
    long percentValue;

//    #ifdef DebugMode
//    SendDebugMsg("\r\nRaw data= ");
//    PrintNum((long)AtoDSensResult);
//    #endif DebugMode

    switch (SensorType[objToMsr])
    {


        case AT:
        case ST5:
        case AT_B:
        case  COMBO_TEMP_1W:

       
//             if(SensorType[objToMsr] == AT_B)
//             AtoDSensResult/=10;     //Danny - for boaz's sensor
         
       
            if ((AtoDSensResult > 1250) || (AtoDSensResult < -600))
            {
                sr = 999;
            }
            else
                sr = AtoDSensResult;  
                
//                 SendDebugMsg("\r\ntemp 1wire Sensor val=  ");
//                   PrintNum(sr);
        break;

        // 4-20mA sensors
        case  DOROT_PRESSURE:    
        case PH:  //kando PH-  4-20mA
        case WLV:    //kando water level-  4-20mA

               sr =  AtoDSensResult;
                if( sr <= 400)
                sr = 400;       //fixed 030816

                 SensorDataType[objToMsr] = 1; //mark as 4-20mA

       case  SM_4_20:
       case TRH_300:      //evogen Humidity 4-20mA
       case TRE_150:       //evogen temp 4-20mA
       case HD2021T:       //evogen radiation 4-20mA
                 sr =  AtoDSensResult;
                 if( sr < 400)
                 sr = 400;
        break;

        case WLV_PULSTAR:
         case H2S:
                  sr =  AtoDSensResult;
                 if( sr <= 400)
                 sr = 400;       //fixed 030816
                 SensorDataType[objToMsr] = 1; //mark as 4-20mA
        break;

         // Kandu's SDI  sensors
         case EC_SDI12:
         case TMP_SDI12:
         case PH_SDI12:

                      sr = AtoDSensResult;  // temp mult by 10. server ve to divide by 10
//                    #ifdef DebugMode
//                    SendDebugMsg("\r\nAnalyze SDI Sensor val  ");
//                    PrintNum(sr);
//                      #endif DebugMode

         break;

//
 //---------------------------New by Danny--------------------
        case HS10:
        case TNS:
        case GS1:
        case DW_LE:  //kando water existance 319117
        case MATIC:     //Kando float sensor
        case DENDROMETER:
                 sr =  AtoDSensResult;
         break;
  //----------------------------------------------------------
         case AH:        // Air Humidity
            //calculate sensor real value
//            AtoDSensResult = (AtoDSensResult - 400) / 15;
//            sr = AtoDSensResult;  
//            
//            if (objToMsr > 0)
//                //check min/max value (temp > 50 or < 0) using the temp. sensor results
                if((iLastMsr[objToMsr - 1] > 0) && (iLastMsr[objToMsr-1] < 500))//if 1w read ok
                {
                    //calculate measuring according to out temp.
                    airTmp = iLastMsr[objToMsr-1] / 10;
                    //calculate the const value
                    percentValue = ((long)airTmp * 44) / 10;
                    percentValue = (10305 + percentValue) / 100;
                    //aprocsimated formula
                    sr =  (((long)AtoDSensResult * 100) / percentValue);
                }
//            //check min & max values
//            if (sr < 0)
//                sr = 0;
//            if (sr > 100)
//                sr = 100;    
                
               sr = AtoDSensResult ;//* 2;    //Danny 010116 Calc %H on server = (data-825)/3
       break;



      case TIR:  // radiation
            //if it is out of range, send to eeprom the low or high table value.
            if(AtoDSensResult < 0)
            {
                sr = 0;
                break;
            }
           else if(AtoDSensResult > 1260)
            {
                sr = 1260;
                break;
            }

            // measuring is in range
//            mVolt_differance = (((unsigned int)AtoDSensResult) * 10);
//            mVolt_differance = mVolt_differance / 33;
//            sr = (mVolt_differance * 10);

              sr = AtoDSensResult;      // calc on server
        break;

          case SCALE_SHKILA_TCS60:
                sr = AtoDSensResult;
          break;
         
         case WTRMTR:
                 sr = AtoDSensResult; 
//                   SendDebugMsg("\r\nAnalize WM = \0");
//                   PrintNum((long)sr);
          break;
       }


//    PrintNum(sr);
//    #endif DebugMode
    return sr;
}

BYTE GetSensorType()
{
   return SensorType[objToMsr];

}

BYTE GetSensorIOType()
{

     return IOType[objToMsr];       //changed 261015 - new eeprom and data block order
}




BYTE SendAlerts()
{
    int i, res = FALSE;


    for (i = SENSOR1; i < NumSensors; i++)
    {
//             if (AlertStatus[i] == ALERT_BACK_NORMAL)
//             res = TRUE;

            if (AlertStatus[i] == TRHRESHOLD_CROSSED)
  //          if (OutOfLmtCnt[i] >= 2)
            {
                res = TRUE;
             //   OutOfLmtCnt[i] = 0;
            }
            else
            {
                AlertStatus[i] = ALERT_WAIT;
//                #ifdef DebugMode
//                SendDebugMsg("\r\nAlert Wait.\r\n\0 ");      //demo
//                #endif DebugMode
             }
    }

    if(PumpActivationNeeded == TRUE)          //demo
    res = TRUE;

    return res;
}



void CheckMeasurmentsLimits()
{
    int i = 0;
    int count = 0;// counter to check if alert is over
    char str[70];    
    
 
    for (i = SENSOR1; i < NumSensors; i++)
    {
                              
        switch (AlertStatus[i])
        {
            case ALERT_WAIT:
            
           if((SensorDataType[i]!= 1) ||(( SensorDataType[i] == 1) && (iLastMsr[i] > 400)))  //WL 400 not exception
           {  
            
                 if (((iLastMsr[i] < MIN_LIMIT[i]) || (iLastMsr[i] > MAX_LIMIT[i])) && (iLastMsr[i] > 0))
                {

                     if(OutOfLmtCnt[i] < 3)
                     {
                           OutOfLmtCnt[i]++;
                           msrAlertFlag = 1;
                           QuickSensorsCheckNeeded = TRUE;

                     }

                 #ifdef DebugMode
                     if(iLastMsr[i] > MIN_LIMIT[i])
                     {
                           SendDebugMsg("\r\nMAX_LIMIT \0");
                            PrintNum(MAX_LIMIT[i]);
                     }
                     else
                     {
                            SendDebugMsg("\r\nMIN_LIMITT \0");
                            PrintNum(MIN_LIMIT[i]);

                     }
                             SendDebugMsg("\r\niLast data= \0");;
                             PrintNum(iLastMsr[i]);

                      SendDebugMsg("\r\nOver Treshold Count Sen \0");
                      PrintNum((long)i+1);
                      PrintNum(OutOfLmtCnt[i]);
                 #endif DebugMode


                   if ((iLastMsr[i] < PUMP_MIN_LIMIT[i]) || (iLastMsr[i] > PUMP_MAX_LIMIT[i]))
                   {

                        //    SendDebugMsg("Pump Exception measured ..\r\n\0 ");
                            OutOfLmtCntPump[i]++;

                            if(OutOfLmtCnt[i] > OutOfLmtCntPump[i])  //if second measure has pump vlue start counting again
                            OutOfLmtCnt[i] = OutOfLmtCntPump[i];

                           if(OutOfLmtCntPump[i] > pALERT_DELAY)
                           {

                                  QuickSensorsCheckNeeded = FALSE;

                                  if(((SensorType[i]) == WLV) || (SensorType[i] == WLV_PULSTAR))
                                  {
                                      if(WLV_OVF == FALSE)
                                       {
                                            WLV_OVF = TRUE;   //flag for alert it  WLV_OVF_FLAG
                                            FlagsStatus =  eFLAGS_STATUS;
                                            SetStatusReg( WLV_OVF_FLAG  , WLV_OVF_FLAG  );
                                            eFLAGS_STATUS = FlagsStatus;
                                       //      SendDebugMsg("WL over flow measured ..\r\n\0");
                                        }                                                                                                                   

                                  }
                                   else  if(Relay2Enabled == TRUE)
                                   {      
//                                          SendDebugMsg("Relay 2 enabled..\r\n\0 ");
//                                         if(PumpActivated == FALSE)
//                                         {    
////                                            SendDebugMsg("PumpActivated == FALSE.\r\n\0 ");
//                                             if ((eECTresholdVal == 0) || ((iLastMsr[0]> eECTresholdVal) && (eECTresholdVal > 0))) //EC value larger than Tresh
//                                             {     
////                                                    SendDebugMsg("EC trshold |OK\r\n\0 ");
//                                                    PumpActivationNeeded = TRUE;      //demo
//                                              //     FlagsStatus |= (1 << PUMP_ACTIVATION_NEEDED);
//                                                    FlagsStatus =  eFLAGS_STATUS;
//                                                   SetStatusReg( PUMP_ACTIVATION_NEEDED , PUMP_ACTIVATION_NEEDED );
//                                                  eFLAGS_STATUS = FlagsStatus;
//                                               
//                                              //    SendDebugMsg("Pump Activation Needed ..\r\n\0 ");
//                                               
//                                                   SensorAtAlertStatus = i;            // sensor with exception 010318
//                                                  DataAtAlertStatus = iLastMsr[i];       //data
//                                             }
//                                         }
                                     }
                           }

                   }

                     if(OutOfLmtCnt[i]== pALERT_DELAY +1)
                     {
                           OutOfLmtCntPump[i] = 0;    //rest pump counter if go to alert
                           QuickSensorsCheckNeeded = FALSE;

                           AlertStatus[i] = TRHRESHOLD_CROSSED;
                           if(IsAlertActivated == FALSE)
                           {
                               IsAlertNeeded = TRUE;  //check flag at  IsTimeToConnectGPRS() general.c

                               FlagsStatus =  eFLAGS_STATUS;
                               SetStatusReg( IS_ALERT_NEEDED ,IS_ALERT_NEEDED);
                               eFLAGS_STATUS = FlagsStatus;
                           }
                           SensorAtAlertStatus = i;            // sensor with exception 300715
                           DataAtAlertStatus = iLastMsr[i];       //data

                     }


                }
                //normal data - handle it
                else if(OutOfLmtCnt[i] > 0)
                {
                      OutOfLmtCnt[i]--;
                      sprintf(str, "Sensor %d Is B-T-N after 1 or 2 exceptions..\r\n\0", i + 1 );
                      UART_WriteMsg(str);


                      if (i == SensorAtAlertStatus)
                      SensorBackToNormalAlertNeeded = i;      //sensor that alerted before  ??????
                 }

           }
           break;

              case TRHRESHOLD_CROSSED:
                     
                       SendDebugMsg("\r\nPost Alert State..\r\n\0 ");
                   
              if((SensorDataType[i]!= 1) ||(( SensorDataType[i] == 1) && (iLastMsr[i] > 400  ))) //WL 400 not exception  //WL 400 not exception
              {
                //   SENS4_20 = FALSE ;
                  if ((iLastMsr[i] < MIN_LIMIT[i]) || (iLastMsr[i] > MAX_LIMIT[i])) //if over TH check Pump TH
                  {
                         if(OutOfLmtCnt[i] < 3)
                         OutOfLmtCnt[i]++;
                         QuickSensorsCheckNeeded = FALSE;    //post alert - no quick measure any more


                         if ((iLastMsr[i] < PUMP_MIN_LIMIT[i]) || (iLastMsr[i] > PUMP_MAX_LIMIT[i]))
                        {
                               if(OutOfLmtCntPump[i] < ( pALERT_DELAY + 1))//alert made but mot for bottol
                               {
                                     msrAlertFlag = 1;
                                     if(PumpActivated == FALSE)
                                     {
                                             QuickSensorsCheckNeeded = TRUE;
                                             QuickMsrCount = 0;
                                             OutOfLmtCntPump[i]++;
                                      //       SendDebugMsg("\r\nPump Treshold crossed- Sen ");
                                     //        PrintNum((long)i+1);
                                      //       PrintNum(OutOfLmtCntPump[i]);
                                      }

                               }
                               else  //3 measures done
                               {

                                      QuickSensorsCheckNeeded = FALSE;

                                      if(((SensorType[i]) == WLV) || (SensorType[i] == WLV_PULSTAR))
                                     {
                                           if(WLV_OVF == FALSE)
                                           {
                                                WLV_OVF = TRUE;   //flag for alert it  WLV_OVF_FLAG
                                                FlagsStatus =  eFLAGS_STATUS;
                                                 SetStatusReg( WLV_OVF_FLAG  , WLV_OVF_FLAG  );

                                                 IsAlertNeeded = TRUE;  //check flag at  IsTimeToConnectGPRS() general.c
                                                 SetStatusReg(IS_ALERT_NEEDED , IS_ALERT_NEEDED );
                                                 eFLAGS_STATUS = FlagsStatus;
                                             //     SendDebugMsg("WL over flow measured ..\r\n\0 ");
                                            }
                                       }

                                       else  if(Relay2Enabled == TRUE)   //pump allowed
                                       {
//                                             if ((eECTresholdVal == 0) || ((iLastMsr[0] > eECTresholdVal) && (eECTresholdVal > 0))) //EC value larger than Tresh
//                                             {
//                                                   if(PumpActivated == FALSE)
//                                                   {
//
//                                                          PumpActivationNeeded = TRUE;      //demo
//                                                          FlagsStatus =  eFLAGS_STATUS;
//                                                          SetStatusReg( PUMP_ACTIVATION_NEEDED , PUMP_ACTIVATION_NEEDED );
//                                                          eFLAGS_STATUS =  FlagsStatus;
//                                                       //    SendDebugMsg("\r\n Post Alert - Pump needed..!\r\n\0");
//                                                   }
//                                             }
                                       }
                                        else
                                        {
                                                  IsAlertNeeded = TRUE;  //check flag at  IsTimeToConnectGPRS() general.c
                                                   FlagsStatus =  eFLAGS_STATUS;
                                                   SetStatusReg(IS_ALERT_NEEDED , IS_ALERT_NEEDED );
                                                   eFLAGS_STATUS = FlagsStatus;
                                              //       SendDebugMsg("\r\n Post Alert - Pump  New alert needed..!\r\n\0");
                                        }
                                           SensorAtAlertStatus = i;            // sensor with exception 300715
                                           DataAtAlertStatus = iLastMsr[i];       //data

                                }
                           }

                 }
                 else   //no over treshold measured
                 {
                        if((OutOfLmtCnt[i] >= 3) || ( OutOfLmtCntPump[i] >= 3))
                        QuickMsrCount = 0;  //allow quick measure if data back to normal

                        OutOfLmtCnt[i]--;
                        OutOfLmtCntPump[i]--;
                                  //value not over TH now- start back to normal
                        if(IsAlertActivated == TRUE)
                       {
                               if(QuickMsrCount < 3)
                               QuickSensorsCheckNeeded = TRUE;   //after alert, if data is normal check every inute 3 times

                              SendDebugMsg("\r\nSystem is in  Back-To-Normal Process..\r\n\0 ");
//                               sprintf(Str, "Sensor %d Is in Back-To-Normal Process..\r\n\0", i + 1 );
//                               UART_WriteMsg(Str);

                       }

                  }

                  }
               break;   // case TRHRESHOLD_CROSSED:
            }
    }  //for i

     for (i = SENSOR1; i < NumSensors; i++)
     {
            if(OutOfLmtCnt[i] == 0)
            {
               count++;
                OutOfLmtCntPump[i] = 0;
               AlertStatus[i] =  ALERT_WAIT;
            }
            SensorDataType[i] = 0;  //reset data types
     }

    if((count == NumSensors) &&  (IsAlertActivated == TRUE))    //050517 --  &&  (IsAlertActivated == TRUE)
     {
             msrAlertFlag = 0;

             SendDebugMsg("\r\nNo more Over_Treshold Measured..\r\n\0 ");

             QuickSensorsCheckNeeded = FALSE;

              QuickMsrCount = 0;
             if(BackToNormalAlertEnabled == TRUE)      //300715
             {
                     if(IsAlertActivated == TRUE)    //if after alert session
                     {
                          BackToNormalAlertActivated = FALSE;
                          BackToNormalAlertNeeded = TRUE;    //all sensors OK. alert it

                          if( WLV_OVF == TRUE)
                          {
                               WLV_OVF = FALSE;
                               FlagsStatus =  eFLAGS_STATUS;
                               SetStatusReg(WLV_OVF_FLAG , 0);
                               eFLAGS_STATUS =  FlagsStatus;
                          }

                          SendDebugMsg("\r\nActivate Back_To_Normal Alert ..\r\n\r\n\0 ");
                     }
             }
             else SendDebugMsg("\r\nBack_To_Normal Alert is Disabled..!\r\n\r\n\0 ");
          //   IsAlertActivated = FALSE;   //M1

    }
    
      #ifdef NETIV_ALERT_UNIT  
     //       SendDebugMsg("After Measure - check power bit \r\n\r\n\0 "); 
           if(TSTBIT(PIND,3))  //power good - if alerted, send BTN msg
           {    
                SendDebugMsg("power bit high \r\n\r\n\0 "); 
               if(( CoverAlertSent == TRUE) && (POWER_FAILURE == TRUE))
               {     
                   BackToNormalAlertNeeded = TRUE;  
                   POWER_FAILURE = FALSE;
                   SendDebugMsg("\r\nPower Failure - Back_To_Normal Alert.. \r\n\r\n\0 ");  
               }
           }  
     #endif  

 }





void StartHeat()
{

    V33_PWR_ON();   //enable 3.3V to entire board
    if (objToMsr == BATTERY)
    { 
        BatLevel_ON();
    }
//    else if(objToMsr == PUMP)
//    {
//        ActivePump();            //put relay on
//        SendDebugMsg("\r\nPump ON..\r\n\0 ");
//    }
    else
    {

        if(SensorType[objToMsr] == COMBO4_SDI12) //set SDI interface befor sensor powerup
        {
               SET_SDI12_TX_DIRECTION();
              SETBIT(DDRH,1);       //TX1 pin output -prepare for BREAK
              SETBIT(PORTH,1);     //TX1 pin high
        }
        else if(SensorType[objToMsr] == COMBO_5TE)
        {
              SET_SDI12_RX_DIRECTION();

        } 
    
         delay_ms(2);
        VOLTAGE_MULTIPLIER_ON();  //Creacell - voltage mult switch on- POWERING SENSORs         
         delay_ms(500);
        SensorPowerOn();
     }

}

void InitAdcForBatt(void)
{
    // Analog Comparator initialization
    // PortA analog inputs init


      CLRBIT(DDRF,7);        //V6
      CLRBIT(PORTF,7);

}

void StartMeasure()
{
// unsigned int hTime;


    heat_time = 0;
    PRR0_EN_ADC();

    if (objToMsr == BATTERY)
    {
        heat_time = BATTERY_HEAT_TIME;    //1 sec of waiting
    }
//    else  if (objToMsr == PUMP)
//    {
//          hTime = ePUMP_HEAT_TIME ;
//          hTime *= 10;
//          heat_time =  hTime;    // sec to 1/10 sec
//
//          SendDebugMsg("Pump time: \r\n\0 ");
//          PrintNum(heat_time / 10);
//
//    }
   else  SetSensorHeatTime();

      InitAdc(); //init the cpu a2d port
 
     //set the measuring timing (from day start) into measure_time variable
      measure_time = time_in_minutes;      //   time_in_minutes = Hour * 60 + minutes  
      ActivateTimer1();
      StartHeat();
      delay_ms(10);
}

void ReadSensor()
{
    int curSensResult = 0;
   unsigned int tResult = 0;
    int tResult1 = 0;
    int ok,i,j; 
    char str[25];

//    SendDebugMsg("\r\nRead Sensor ");
//    PrintNum(objToMsr);


     if (objToMsr == PUMP)
     {
         SensorResult = 0;
         return;
     }
    if (objToMsr == BATTERY)
    {
        
         SendDebugMsg("\r\nMeasure Battery..\r\n\0");      
        SensorResult = AnalogSensRead();
        return;
    }
    switch (SensorType[objToMsr])
    {
//        case SM:
//        case TMP:   //old sensors combination NO_MSR: danny
//             curSensResult = 0;
//        break;

          case WTRMTR:
                  
                     curSensResult = wtrPulseCnt ;                     
                      
             #ifdef DESHEN_WATER_METER   //deshen 160120
             
                   if(wtrPulseCnt > Last_wtrPulseCnt)
                   Last_wtrPulseCnt = wtrPulseCnt; 
                   else
                   {   
                      
                       if(wtrPulseCnt > 0)
                       {
                           WM_SilencCount++; 
                           if( WM_SilencCount >= (60 / (5 * cpue2_interval_1)))  //at least 1 hour of no int
                           {
                                wtrPulseCnt = 0;
                                Last_wtrPulseCnt = 0; 
                                WM_SilencCount = 0; 
                                curSensResult = wtrPulseCnt ;
                           }
                       }
                   } 
                   
             #endif
                   
//                   SendDebugMsg("\r\ncurSensResult WM = \0");
//                   PrintNum((long)curSensResult);
          break; 
          
          case  COMBO_TEMP_1W: 
          
                         SendDebugMsg("\r\nRead 1wire TEMP combo..\r\n\0"); 
                         SendDebugMsg("Must be connected to input 4..!\r\n\0");  
          
                        // Determine the number of DS1820 devices
                        // connected to the 1 Wire bus
                        ds1820_devices = w1_search(0xf0,ds1820_rom_codes); 
                        SendDebugMsg("\r\nds1820_devices = \0 "); 
                        PrintNum((long)ds1820_devices); 
                        
                      
                          if (ds1820_devices)
                         {
                                for (i=0;i < ds1820_devices;)
                                {

                                          curSensResult = ds1820_temperature_10(&ds1820_rom_codes[i][0]);  
                                          tResult = curSensResult;
                                          if( ds1820_rom_codes[i][0] == 0x10)  //old chip (phytech)                                   
                                          tResult *= 5; 
                                       
                                          else   if( ds1820_rom_codes[i][0] == 0x28)  //old chip (phytech)
                                          {     
                                       
                                                   tResult =  (curSensResult / 16) * 10;     //convert 2'sComplement value to decimal  
                                                   tResult += ((curSensResult % 16) * 10) / 16;                                                                      
                                          }
                                           if(i == 0)
                                           COMBO_TMP_DATA = tResult;   //first sensor data
                                           else 
                                           COMBO_HUMID_DATA = tResult; //second data                                       
                                      
                                           sprintf(str,"t%-u=%-i.%-u\n\r",++i,tResult/10, abs(tResult%10));
                                           PCmessage(str);
                                };
                         }
                          else 
                          {
                               SendDebugMsg("\r\nNo 1wire TEMP detected..!\r\n\0");
                               COMBO_TMP_DATA = 0;
                          }     
                   break; 
                   
        case AT:
        case ST5:
        case AT_B:    
        
                   SendDebugMsg("\r\nRead 1wire Temp sensor..\r\n\0"); 
                    SendDebugMsg("Must be connected to input 4..!\r\n\0");
                      
                   w1_init();
                    ds1820_devices = w1_search(0xf0,ds1820_rom_codes); 
                        SendDebugMsg("\r\nds1820_devices = \0 "); 
                        PrintNum((long)ds1820_devices); 
                
                /* display the ROM codes for each detected device */
                
                 if (ds1820_devices)
                 {

                        for (i=0;i<ds1820_devices;i++)
                         {

                              sprintf(str,"DEVICE #%-u ROM CODE IS:", i+1); 
                              PCmessage(str);

                               for (j=0;j<8;j++)
                               {
                                  sprintf(str,"%-X ",ds1820_rom_codes[i][j]);  
                                   PCmessage(str);
                               }
                               sprintf(str,"\n\r\0"); 
                              PCmessage(str);

                        };

                   
                                               
                           curSensResult = ds1820_temperature_10(&ds1820_rom_codes[0][0]);                           
        //                 
        //                  sprintf(str,"%04X \0", curSensResult);
        //                 PCmessage(str);  
                         
                         if( ds1820_rom_codes[0][0] == 0x10)  //old chip (phytech) 
                           curSensResult *= 5;   
                           
                         else   if( ds1820_rom_codes[0][0] == 0x28)  //old chip (phytech) 
                         {                                                          
                                 tResult =  (curSensResult / 16) * 10;     //convert 2'sComplement value to decimal  
                                 tResult += ((curSensResult % 16) * 10) / 16;                                            
                                 curSensResult = tResult;                       
                         } 
                         else   SendDebugMsg("\r\nUnknown sensor type - check ROM CODE..\r\n\0 ");
                         
                         sprintf(str,"t%-u=%-i.%-u\n\r",1,curSensResult/10, abs(curSensResult%10));
                         PCmessage(str);
                 } 
                 else  SendDebugMsg("\r\nCould not read 1W sensor!!!..\r\n\0 ");
                             
        break;

        case AH:
        case TIR:
        case HS10:
        case TNS:
        case GS1:
         case DENDROMETER:
                 curSensResult = AnalogSensRead();   //Danny  
//                 sprintf(str,"curSensResult=%d\n\r",curSensResult);
//                 PCmessage(str);
        break;

        case DW_LE:    //read PA.6. if high water exist
                   curSensResult = 0;

                  {
                      delay_ms(100);
                     if (TSTBIT(PINJ,2))
                         curSensResult = 10;
                  }
                  if(curSensResult == 10)
                  SendDebugMsg("\r\nWater Exist Sensor: Water Flow..!\r\n\0 ");
                  else
                   SendDebugMsg("\r\nWater Exist Sensor: No Water..\r\n\0 ");

        break;

         case MATIC:     //Kando water exist floating sensor. if see 1 , sensor is floating
                   curSensResult = 0;

                    delay_ms(5);
                    if (TSTBIT(PINF,1))
                   {
                      delay_ms(500);
                     if (TSTBIT(PINF,1))
                         curSensResult = 10;
                   }
                  if(curSensResult == 10)
                  SendDebugMsg("\r\nWater Flows..!\r\n\0 ");
//                  else
//                   SendDebugMsg("\r\n No Water..\r\n\0 ");


         break;


           case  SM_4_20:
           case H2S:
            case TRH_300:      //evogen Humidity 4-20mA
           case TRE_150:       //evogen temp 4-20mA
           case HD2021T:       //evogen radiation 4-20mA

                  curSensResult = AnalogSensRead();   //Danny
                  if  (curSensResult > 2000)
                  {
                       delay_ms(500);
                       curSensResult = AnalogSensRead();   //try again - sensor unstable
                  }
                  if  (curSensResult < 400)
                       curSensResult = 400;       //protect from too low val

             //    SENS4_20 = TRUE;
                 SensorDataType[objToMsr] = 1; //mark as 4-20mA
              break;

     case WLV:
     case PH:        //PH
     case WLV_PULSTAR: 
    case DOROT_PRESSURE:   
                       i = 0;
                      timer0_count = 550;   //1100mS  max for next do loop
                      Measure_Timer_Active = TRUE;  //for timer2 operation
                      ENABLE_SDI_TIMER0();          //3600 Hrz clock
                     while  ((( tResult = AnalogSensRead()) < 390) && (timer0_count > 0)); //wait for sensor react

//                     SendDebugMsg("\r\ntimer= \0 ");
//                     PrintNum((long)timer0_count);

                  if (tResult >= 390)     //if ouput less than 300- abort
                  {
                          delay_ms(100);
                            curSensResult = AnalogSensRead();

                      do
                        {
                             delay_ms(100);
                            tResult = AnalogSensRead();
                            tResult1 =  curSensResult - tResult;
                            if ((tResult1 < 5) && (tResult1 > -5))  //+-5 accuracy
                            i++;
                            else i = 0;
                            curSensResult = tResult;
                        }
                       while ((i < 3) && (timer0_count > 0));


//                          SendDebugMsg("\r\ntimer= \0 ");
//                           PrintNum((long)timer0_count);
                  }
                  else
                  {
                     
                       SendDebugMsg("Measuring 4-20mA failed..\r\n\0");
                     
                  }

                    DISABLE_TIMER0();
                   Measure_Timer_Active = FALSE;

                  if  (curSensResult < 400)
                       curSensResult = 400;       //protect from too low val
                  SensorDataType[objToMsr] = 1; //mark as 4-20mA
          break;



//#ifdef RS485_ACTIVE

          case COMBO4_485:

//                  SDI_Combo_index = objToMsr;   //not implemented yet -keep base index of combo sensors
//                  Glob_data_Saved = FALSE;
//                 SDI_EC_MEASURED = FALSE ;
//                 SDI_PH_MEASURED = FALSE;
//                 i = 0;
//
//                  SendDebugMsg("Measuring Ponsel MODBUS sensors..\r\n\0");
//                 SendDebugMsg("\r\nMeasure EC + TEMP\r\n\0");
//
//                 do{
//
//                          ok =  Read_485_Sensor(S485_EC_ADDRESS);  //param 1: address;  param 2: 1=temp, 2=EC
//                          if(ok)
//                          {
//
//                                        SendDebugMsg("\r\nRead EC + TEMP OK\r\n\0 ");
//                                        NotStable = FALSE;
//
//                                        SDI_EC_MEASURED = TRUE;
//                                        SDI_TMP_MEASURED = TRUE;
//
//                          }
//                          else
//                          {
//                                //  SendDebugMsg("\r\nRead EC Failure\r\n\0 ");
////                                   SDI_EC_DATA = (int)0x8000;    //error  0xFF01
////                                   SDI_TEMP_DATA = (int)0x8000;    //error  0xFF01
//                                   SDI_EC_MEASURED = FALSE;
//                                   delay_ms(700);
//                                    i++;
//                          }
//
//                      }
//                      while ((SDI_EC_MEASURED == FALSE) && (i < 2)); //allow 2 loops
//
//                       if(i == 2)
//                         {
//
//                                 SDI_EC_DATA = (int)0x8000;         //error  0xFF01
//                                 SDI_TEMP_DATA = (int)0x8000;    //error  0xFF01
////                               SendDebugMsg("\r\nRead EC- i=2\r\n\0 ");
//                         }
//
//                      //PH also
//
//                       i = 0;
//                        SendDebugMsg("\r\nMeasure PH + ORP\r\n\0");
//                       do{
//
//                               delay_ms(500);
//                               ok =  Read_485_Sensor(S485_PH_ADDRESS);  //read PH and ORP
//                              if(ok)
//                              {
//
//                                  if ((int)SDI_PH_DATA < 0)      //semnsor sfault 9998
//                                  {
//                                       NotStable = TRUE;
//                                       SDI_PH_MEASURED = FALSE;    //try again
//                                       i++;
//                                       SendDebugMsg("\r\nPH data not stable..\r\n\0 ");
//                                       SDI_PH_MEASURED = FALSE;
//                                  }
//                                   else
//                                   {
//                                        SendDebugMsg("\r\nRead PH + ORP OK\r\n\0 ");
//                                        NotStable = FALSE;
//                                        SDI_PH_MEASURED = TRUE;
//                                    }
//                              }
//                              else
//                             {
////                                     SDI_PH_DATA = (int)0x8000;//-254;    //error  0xFF01
////                                     SDI_ORP_DATA =(int)0x8000;
//
//                                     delay_ms(700);
//                                     i++;
//                              }
//
//                         }
//                         while ((SDI_PH_MEASURED == FALSE)  && (i < 2)); //allow 2 loops
//
//                         if(i == 2)
//                         {
//
//                                //     SendDebugMsg("\r\nRead PH- i=2\r\n\0 ");
//                                     SDI_PH_DATA = (int)0x8000;//-254;    //error  0xFF01
//                                     SDI_ORP_DATA =(int)0x8000;
//
//                         }
//                         Measure_Timer_Active = FALSE;
//
//                         return;
          break;

          case COMBO_EC_485:

//                    i = 0;
//                   SendDebugMsg("\r\nMeasure EC + TEMP (MODBUS)\r\n\0");
//
//                 do{
//
//                          ok =  Read_485_Sensor(S485_EC_ADDRESS);  //param 1: address;  param 2: 1=temp, 2=EC
//                          if(ok)
//                          {
////                                   if(SDI_EC_DATA == 1249 )      //semnsor sfault 9998/8
////                                   {
////                                       NotStable = TRUE;
////                                       SDI_EC_MEASURED = FALSE;    //try again
////                                       i++;
////                                       SendDebugMsg("EC data not stable..\r\n\0 ");
////                                   }
////                                   else
//                                   {
//                                            SendDebugMsg("\r\nRead EC + TEMP OK\r\n\0 ");
//                                            NotStable = FALSE;
//
//                                            SDI_EC_MEASURED = TRUE;
//                                            SDI_TMP_MEASURED = TRUE;
//                                   }
//                          }
//                          else
//                          {
//                                   SDI_EC_MEASURED = FALSE;
//                                   delay_ms(700);
//                                    i++;
//                          }
//
//                      }
//                      while ((SDI_EC_MEASURED == FALSE) && (i < 2)); //allow 2 loops
//
//                       if(i == 2)
//                         {
//
//                                 SDI_EC_DATA = (int)0x8000;         //error  0xFF01
//                                 SDI_TEMP_DATA = (int)0x8000;    //error  0xFF01
////                               SendDebugMsg("\r\nRead EC- i=2\r\n\0 ");
//                         }
//                         Measure_Timer_Active = FALSE;
//                         return;
          break;

          case  COMBO_PH_485:
//                   i = 0;
//                        SendDebugMsg("\r\nMeasure PH + ORP  (MODBUS)..\r\n\0 ");
//                       do{
//
//                               delay_ms(500);
//                               ok =  Read_485_Sensor(S485_PH_ADDRESS);  //read PH and ORP
//                              if(ok)
//                              {
//
//                                  if ((int)SDI_PH_DATA < 0)      //semnsor sfault 9998
//                                  {
//                                       NotStable = TRUE;
//                                       SDI_PH_MEASURED = FALSE;    //try again
//                                       i++;
//                                       SendDebugMsg("\r\nPH data not stable..\r\n\0 ");
//                                  }
//                                   else
//                                   {
//                                        SendDebugMsg("\r\nRead PH + ORP OK\r\n\0 ");
//                                        NotStable = FALSE;
//                                        SDI_PH_MEASURED = TRUE;
//                                    }
//                              }
//                              else
//                             {
////                                     SDI_PH_DATA = (int)0x8000;//-254;    //error  0xFF01
////                                     SDI_ORP_DATA =(int)0x8000;
//
//                                     delay_ms(700);
//                                     i++;
//                              }
//
//                         }
//                         while ((SDI_PH_MEASURED == FALSE)  && (i < 2)); //allow 2 loops
//
//                         if(i == 2)
//                         {
//
//                                //     SendDebugMsg("\r\nRead PH- i=2\r\n\0 ");
//                                     SDI_PH_DATA = (int)0x8000;//-254;    //error  0xFF01
//                                     SDI_ORP_DATA =(int)0x8000;
//
//                         }
//                         Measure_Timer_Active = FALSE;
//
//                         return;
          break; 
          
            case TEMP_HUMIDITY:  //Gevim test
          
                      i = 0;                                                                                                 
                        SendDebugMsg("\r\nMeasure TEMP + HUMIDITY (MODBUS)..\r\n\0");
                       do{

                             //  delay_ms(500);
                               ok =  Read_485_Sensor(TEMP_HUMID_ADDRESS);  //read PH and ORP 
                               DISABLE_RX_INT_UART2();
                              if(ok)
                              {                                                                       
                                          NotStable = FALSE;                                                                               
                                          curSensResult = COMBO_TMP_DATA ;
                                         
                                           timer0_count = 1000;      //10 sec - 10mS clock
                                          Measure_Timer_Active = TRUE;  //for timer2 operation 
                                           ENABLE_SDI_TIMER0();   
                                        do
                                        {
                                             delay_ms(1000);  
                                             Read_485_Sensor(TEMP_HUMID_ADDRESS);   //address 1- default
                                             DISABLE_RX_INT_UART2();  
                                             
                                            tResult = COMBO_TMP_DATA ;
                                            tResult1 =  curSensResult - tResult;
                                            if ((tResult1 < 40) && (tResult1 > -40))  //+-50 accuracy
                                              i++;
                                            else i = 0;
                                            curSensResult = tResult;
                                             
                                          //  PrintNum((long)tResult1);
                                           
                                        }
                                       while ((i < 1) && ( timer0_count > 0)); 
                                        if(i == 1)  
                                        {    
                                               i = 0;
                                             SDI_TMP_MEASURED = TRUE;
                                             SendDebugMsg("\r\nRead Temp / Humidity OK\r\n\0 "); 
                                        }
                              }
                              else
                             {
                                     delay_ms(2000);
                                     i++;
                              }

                         }
                         while ((SDI_TMP_MEASURED == FALSE)  && (i < 2)); //allow 2 loops

                         if(i == 2)
                         {

                                //     SendDebugMsg("\r\nRead PH- i=2\r\n\0 ");
                                     COMBO_TMP_DATA = (int)0x8000;//-254;    //error  0xFF01
                                     COMBO_HUMID_DATA =(int)0x8000;
                                     SDI_TMP_MEASURED = FALSE;
                         }
                         Measure_Timer_Active = FALSE;

                         return;
          break; 


//#else  //RS485_ACTIVE


       //--------SDI12 -----------------



         case COMBO4_SDI12:     // 12  EC + TEMP +PH    (Ponsel..!)
//                 SDI_Combo_index = objToMsr;   //not implemented yet -keep base index of combo sensors
//                  Glob_data_Saved = FALSE;
//                 SDI_EC_MEASURED = FALSE ;
//                 SDI_PH_MEASURED = FALSE;
//                 i = 0;
//
//                         #ifdef DebugMode
//                         SendDebugMsg("Measure EC + TEMP.\r\n\0 ");
//                        #endif DebugMode
//                 do{
//
//                          ok = SD12_Sensors_Read(SDI_EC_ADDRESS);  //param 1: address;  param 2: 1=temp, 2=EC
//                          if(ok)
//                          {
//
//                                 if(SDI_EC_DATA == 1249 )      //semnsor sfault 9998/8
//                                 {
//                                       NotStable = TRUE;
//                                       SDI_EC_MEASURED = FALSE;    //try again
//                                       i++;
//                                       SendDebugMsg("EC data not stable..\r\n\0 ");
//                                  }
//                                  else
//                                  {
//                                           #ifdef DebugMode
//                                           SendDebugMsg("\r\nRead EC + TEMP READ OK\r\n\0 ");
//                                           #endif DebugMode
//                                            NotStable = FALSE;
//
//                                            SDI_EC_MEASURED = TRUE;
//                                            SDI_TMP_MEASURED = TRUE;
//
//                                  }
//
//                          }
//                          else
//                          {
//                                //  SendDebugMsg("\r\nRead EC Failure\r\n\0 ");
//                                   SDI_EC_DATA = (int)0x8000;    //error  0xFF01
//                                   SDI_TEMP_DATA = (int)0x8000;    //error  0xFF01
//
//                                   delay_ms(700);
//                                    i++;
//                          }
//
//                      }
//                      while ((SDI_EC_MEASURED == FALSE) && (i < 2)); //allow 2 loops
//
//                       if(i == 2)
//                         {
//
//                                 SDI_EC_DATA = (int)0x8000;         //error  0xFF01
//                                 SDI_TEMP_DATA = (int)0x8000;    //error  0xFF01
////                               SendDebugMsg("\r\nRead EC- i=2\r\n\0 ");
//                         }
//
//                      //PH also
//
//                       i = 0;
//                        SendDebugMsg("\r\nMeasure PH + ORP\r\n\0 ");
//                       do{
//
//                               delay_ms(500);
//                               ok = SD12_Sensors_Read(SDI_PH_ADDRESS);  //read PH and ORP
//                              if(ok)
//                              {
//
//                                  if ((int)SDI_PH_DATA < 0)      //semnsor sfault 9998
//                                  {
//                                       NotStable = TRUE;
//                                       SDI_PH_MEASURED = FALSE;    //try again
//                                       i++;
//                                       SendDebugMsg("\r\nPH data not stable..\r\n\0 ");
//                                  }
//                                   else
//                                   {
//                                        SendDebugMsg("\r\nRead PH + ORP OK\r\n\0 ");
//                                        NotStable = FALSE;
//                                       SDI_PH_MEASURED = TRUE;
//
//                                    }
//                              }
//                              else
//                              {
//                                     SDI_PH_DATA = (int)0x8000;//-254;    //error  0xFF01
//                                     SDI_ORP_DATA =(int)0x8000;
//
//                                     delay_ms(700);
//                                     i++;
//                              }
//
//                         }
//                         while ((SDI_PH_MEASURED == FALSE)  && (i < 2)); //allow 2 loops
//
//                         if(i == 2)
//                         {
//
//                                //     SendDebugMsg("\r\nRead PH- i=2\r\n\0 ");
//                                     SDI_PH_DATA = (int)0x8000;//-254;    //error  0xFF01
//                                     SDI_ORP_DATA =(int)0x8000;
//
//                         }
//                         Measure_Timer_Active = FALSE;
//
//                         return;
         break;

          case COMBO2_EC_SDI12:   //11 EC + temp only

//                 SDI_Combo_index = objToMsr;   //not implemented yet -keep base index of combo sensors
//                  Glob_data_Saved = FALSE;
//                 i = 0;
//                 do{
//
//                          #ifdef DebugMode
//                          SendDebugMsg("\r\nMeasure EC + TEMP (Ponsel)\r\n\0 ");
//                          #endif DebugMode
//                          ok = SD12_Sensors_Read(SDI_EC_ADDRESS);  //param 1: address;  param 2: 1=temp, 2=EC
//                          if(ok)
//                          {
//                                 SDI_EC_MEASURED = TRUE;
//                                 SDI_TMP_MEASURED = TRUE;
//                                 #ifdef DebugMode
//                                 SendDebugMsg("EC + TEMP OK\r\n\0 ");
//                                 #endif DebugMode
//                          }
//                          else
//                          {
//
//
//
//                                     SDI_EC_DATA = (int)0x8000;    //error  0xFF01
//                                     SDI_TEMP_DATA = (int)0x8000;    //error  0xFF01
//
//                               delay_ms(500);
//                               i++;
//                          }
//
//                      }while ((SDI_EC_MEASURED == FALSE) && (i < 2)); //allow 2 loops
//                       return;
             break;

           case  COMBO2_PH_SDI12:
//                  SDI_Combo_index = objToMsr;   //not implemented yet -keep base index of combo sensors
//                  Glob_data_Saved = FALSE;
//                 i = 0;
//                 do{
//                            if( SDI_PH_MEASURED == FALSE)
//                            {
//                               ok = SD12_Sensors_Read(SDI_PH_ADDRESS);  //param 1: address;  param 2: 1=temp, 2=EC
//                              if(ok)
//                              {
//
//                                   SDI_PH_MEASURED = TRUE;
//                                   SDI_TMP_MEASURED = TRUE;
//                                   #ifdef DebugMode
//                                   SendDebugMsg("\r\nPH + TEMP OK\r\n\0 ");
//                                   #endif DebugMode
//                              }
//                              else
//                              {
//
//
//                                      SDI_PH_DATA = (int)0x8000;   //-254;    //error  0xFF01
//                                      SDI_TEMP_DATA = (int)0x8000;
//                                       SDI_ORP_DATA = (int)0x8000;
//
//
//                                  delay_ms(500);
//                                  i++;
//                              }
//                            }
//                         }while ((SDI_PH_MEASURED == FALSE) && (i < 2)); //allow 2 loops
//                          Measure_Timer_Active = FALSE;
//                         return;

           break;

//  #endif

           case  COMBO_5TE:
                       i = 0;
                       Glob_data_Saved = FALSE;
                       
                       
                        SendDebugMsg("\r\nMeasure 5TE COMBO\r\n\r\n\0 ");
                      
                       do{

                          ok = SD12_Sensors_Read(SDI_5TE_ADDRESS);  //param 1: address;  param 2: 1=temp, 2=EC
                          if(ok)
                          {
                                 SDI_EC_MEASURED = TRUE;
                                 SDI_TMP_MEASURED = TRUE;
                              
                                 SendDebugMsg("\r\nMeasured EC + TEMP OK\r\n\0 ");
                                
                          }
                          else
                          {
                               SET_SDI12_RX_DIRECTION();
                               delay_ms(1000);
                               i++;
                          }

                      }while ((SDI_EC_MEASURED == FALSE) && (i < 2)); //allow 2 loops

                      if(SDI_EC_MEASURED == FALSE)
                      {
                              
                                 SendDebugMsg("\r\nNo Response from 5TE Sensor!\r\n\0 ");
                                
                               SDI_EC_DATA = (int)0x8000;    //error  0xFF01
                               SDI_TEMP_DATA = (int)0x8000;    //error  0xFF01
                      }
                      return;
        break ;   
 #ifdef WEATHER_STATION_485   
     
        case Weather_Sation_485:  
         
                 i = 0;
             //    VOLTAGE_MULTIPLIER_OFF(); 
                 ActivateTimer1();
             do{    
                 ok =  Read_Weather_Station_485();
                 if(ok)
                 {
                       SendDebugMsg("\r\nRead Weather station OK..\r\n\0 ");
                 } 
                 else i++;
              }while((ok == FALSE) && (i < 2));    
            //     ENABLE_RX_INT_UART2();
                  return;
        break; 
 #endif
        case SCALE_SHKILA_TCS60:
//
//             curSensResult = Read_Scale_TCS60();
//             if(curSensResult == -1)
//              SendDebugMsg("\r\nFailed to read scale data..!\r\n\0 ");

        break;


        default:
    }
    //calculate sensor real value

    SensorResult = AnalyzeSensorRealValue(curSensResult);
}

void EndHeat()
{
   // ADCSRA &= ~(1<<ADEN);

    if (objToMsr == BATTERY)
    {
       // BATT_CHECK_POWER_OFF(); //ToDo
//         #ifdef DebugMode
//        SendDebugMsg("\r\nShut Battery test..\r\n\0 ");
//        #endif DebugMode
        BatLevel_OFF();             //Danny
     }
//     else if (objToMsr == PUMP)
//     {
//          DeActivePump();        //shut off
//          PumpActivated = TRUE;
//          IsAlertNeeded = TRUE;
//          IsAlertActivated = FALSE;
//
//
//            SetStatusReg(IS_ALERT_NEEDED | PUMP_ACTIVAED | PUMP_ACTIVATION_NEEDED | IS_ALERT_ACTIVATED, IS_ALERT_NEEDED | PUMP_ACTIVAED );
//            eFLAGS_STATUS =  FlagsStatus;
//
//       
//          SendDebugMsg("\r\nPump OFF..\r\n\0 ");
//        
//      }
    else
    {
     //   V33_PWR_OFF();              //creacell V3-
        VOLTAGE_MULTIPLIER_OFF();
        SensorPowerOff();             //shut sensor sw
    }
    DISABLE_ADC();
//     ENABLE_UART2();    //re-enable uart1 - for monitor.
}

BYTE GetNextMsrTask()
{
    switch (msrCurTask)
    {
        case TASK_NONE:
            msrCurTask = TASK_MSR_START_HEAT;
            //  SendDebugMsg("\r\nGetNextMsrTask() -msrCurTask = TASK_MSR_START_HEAt\r\n\0");
        break;

        case  TASK_MSR_START_HEAT:
            if (heat_time == 0)
            {
                msrCurTask = TASK_MSR_READ;
            }
            else
            {
               //  SendDebugMsg("Heat time wait..\r\n\0");
                   return WAIT;
            }
        break;

        case TASK_MSR_READ:
            msrCurTask = TASK_MSR_SAVE; //TASK_MSR_END_HEAT;

        break;

        case TASK_MSR_SAVE:

            msrCurTask = TASK_NONE;  //

            if (objToMsr == BATTERY)
                bEndOfMeasureTask = TRUE;

            else if (objToMsr == PUMP)
               //  mainTask = TASK_SLEEP;    //return to sleep mode
                bEndOfMeasureTask = TRUE;

            else
            {
                // go to next sensor
                objToMsr++;

                // if finished last sensor - declare end of task
                if (objToMsr >= NumSensors)
                {
                    CheckMeasurmentsLimits();     //check limits and if true
                    bEndOfMeasureTask = TRUE;
                }
                else
                {
                        SendDebugMsg("\r\nMeasuring Sensor: \0 ");
                        PrintNum((long)objToMsr + 1);
                         msrCurTask = TASK_NONE;
                         delay_ms(100);      // if continue to msr next prob - wait a while
                }


            }
        break;
        default:
    }
    return CONTINUE;
}

void MeasureMain()
{
    long l;


    if (GetNextMsrTask() == WAIT)
    {
        return;
    }
//    MUX_SDI12_00();
    switch (msrCurTask)
    {
        case  TASK_MSR_START_HEAT:
             //     SendDebugMsg("to StartMeasure()..\r\n\0 ");
               StartMeasure();          //demo - including StartHeat() - put on pump

        break;

        case TASK_MSR_READ:   
        
                if (SensorType[objToMsr] == TEMP_HUMIDITY)      //COD sensor
                {       
                       ReadAndSaveTEMP_HUMIDSensor();
                
                } 
            

                else if (SensorType[objToMsr] == COMBO_5TE)                                           
                {     
                    //  SendDebugMsg("Read Combo..\r\n\0 ");
                      ReadAndSaveComboSensor();
                } 
//                else if(SensorType[objToMsr] == GPS_SHKILA)
//                {
//                      ReadSaveGPSSensor();
//                }
               #ifdef WEATHER_STATION_485    
                else if(SensorType[objToMsr] == Weather_Sation_485)
                {
                     Read_save_WS_data();
                } 
               #endif  
                             
                else if(SensorType[objToMsr] == COMBO_TEMP_1W)
                {     
                    //   SendDebugMsg("to ReadAndSave_TEMP_1W_Sensors..\r\n\0 ");
                      ReadAndSave_TEMP_1W_Sensors();
                }                
                
                else
                {
                     
                          ReadSensor(); 

                }

                 EndHeat();

        break;

        case TASK_MSR_SAVE:
            //disable interrupts

             if( Glob_data_Saved == TRUE)    //combo-saved alreay in measuring rutine
             {
                 Glob_data_Saved = FALSE;
                 break;
             }

             else if (objToMsr == PUMP)
             break;

            else if (objToMsr == BATTERY)
            {
                //iVoltage = SensorResult;
                l = (long)SensorResult * 47;  //was 47
                iVoltage = l / 10;
                iVoltage += l % 10;

                SendDebugMsg("\r\nBATTERY= ");
                PrintNum((long)iVoltage);  
              //  PrintNum((long)SensorResult);
            }
            else
            {
                iLastMsr[objToMsr] = SensorResult;
                SaveMeasurments();
                #asm ("sei")
            }  
            
            
             #ifndef DESHEN_WATER_METER 
             if ((readClockBuf[4] == 0) && (readClockBuf[5] == 0))  //at midnight reset counters 
             {       
               
                  //save daily total every midnight-enable calc daily rain -weather station
                wtrPulseCnt = 0;                      
                SendDebugMsg("reset wtrPulseCnt..\r\n\0 ");                  
             }   
             #endif
             break;   
             
             default:

    }
}



// read and save SDI COMBO3 combination of sensors EC, Temp, PH
void ReadAndSaveComboSensor(void)
{

    char   tmp_objToMsr;

             tmp_objToMsr = objToMsr;
             ReadSensor();   //read conbo- all 3 data available when done

            if((SensorType[tmp_objToMsr] == COMBO4_SDI12) || (SensorType[tmp_objToMsr] == COMBO2_EC_SDI12)\
            || (SensorType[tmp_objToMsr] == COMBO_5TE)|| (SensorType[tmp_objToMsr] == COMBO4_485)||(SensorType[tmp_objToMsr] == COMBO_EC_485) )
            {
                    iLastMsr[objToMsr] = SDI_EC_DATA;
                    SensorResult  = SDI_EC_DATA;
                    SaveMeasurments();
                    if(( SDI_EC_DATA == 0x8000)  || (SensorType[tmp_objToMsr] == COMBO_5TE))
                    SendDebugMsg("saved EC DATA \r\n\0 ");
                    else
                     SendDebugMsg("saved EC DATA (divided by 8!)\r\n\0 ");


                    objToMsr++;    //point to temp data
                    SensorResult  = SDI_TEMP_DATA;
                    iLastMsr[objToMsr] = SDI_TEMP_DATA;
                    SaveMeasurments();

                    SendDebugMsg("saved TEMP DATA\r\n\0 ");


                     if(SensorType[tmp_objToMsr] == COMBO_5TE)
                     {
                            objToMsr++;    //SM data
                            iLastMsr[objToMsr] = SDI_SM_DATA;
                            SensorResult  = SDI_SM_DATA;
                            SaveMeasurments();
                             SendDebugMsg("saved SM DATA\r\n\0 ");
                     }

                    if((SensorType[tmp_objToMsr] == COMBO4_SDI12)|| (SensorType[tmp_objToMsr] == COMBO4_485)) //indluding PH

                    {
                            objToMsr++;    //go to PH as well
                             iLastMsr[objToMsr] = SDI_PH_DATA;
                            SensorResult  = SDI_PH_DATA;
                            SaveMeasurments();
                           #ifdef DebugMode
                            SendDebugMsg("saved PH DATA\r\n\0 ");
                            #endif DebugMode

                             objToMsr++;       //point to orp
                             SensorResult  = SDI_ORP_DATA;
                             iLastMsr[objToMsr] = SDI_ORP_DATA;
                             SaveMeasurments();
                         
                             SendDebugMsg("saved ORP DATA\r\n\0 ");
                          
                          
                    } 
                    else
                    {
//                           if(SensorType[tmp_objToMsr] == COMBO_EC_485)
//                            objToMsr++;
                    }

             }
             else if((SensorType[tmp_objToMsr] == COMBO2_PH_SDI12)||(SensorType[tmp_objToMsr] == COMBO_PH_485)) //No EC, save Temp, PH , ORP only,
             {

                //   objToMsr++;
                   iLastMsr[objToMsr] = SDI_PH_DATA;
                   SensorResult  = SDI_PH_DATA;
                   SaveMeasurments();
                    SendDebugMsg("saved PH DATA\r\n\0 ");


                    objToMsr++;       //point to orp
                    SensorResult  = SDI_ORP_DATA;
                    iLastMsr[objToMsr] = SDI_ORP_DATA;
                    SaveMeasurments();
                     SendDebugMsg("saved ORP DATA\r\n\0 ");
//
             }
              Glob_data_Saved = TRUE;   //checked when in saving task
}

   void  ReadAndSaveTEMP_HUMIDSensor(void) 
   {
            ReadSensor();   //read combo COD- all 3 data available when done
           
            iLastMsr[objToMsr] = COMBO_TMP_DATA;
            SensorResult  = COMBO_TMP_DATA;
            SaveMeasurments();          
            SendDebugMsg("saved TEMP DATA\r\n\0");
                                
            objToMsr++;    //point to TSS
            SensorResult  = COMBO_HUMID_DATA;
            iLastMsr[objToMsr] = COMBO_HUMID_DATA;
            SaveMeasurments();

            SendDebugMsg("saved HUMIDITY DATA\r\n\0 ");
             Glob_data_Saved = TRUE;     //checked when in saving task
   
   } 
     
    void  ReadAndSave_TEMP_1W_Sensors(void) 
   {
            ReadSensor();   //read combo 1W sensors on bus
              SensorResult  = COMBO_HUMID_DATA;     //air temp for Deshen unit
            iLastMsr[objToMsr] = COMBO_HUMID_DATA;   //holds second sensor data
            SaveMeasurments();
              SendDebugMsg("saved 1W TEMP2 data\r\n\0");
             objToMsr++;  
            iLastMsr[objToMsr] = COMBO_TMP_DATA;   //soil temp for Deshen unit
            SensorResult  = COMBO_TMP_DATA;
            SaveMeasurments();          
            SendDebugMsg("saved 1W TEMP1 data\r\n\0");
                                                                 
            Glob_data_Saved = TRUE;     //checked when in saving task
   
   }    
   
   
 #ifdef WEATHER_STATION_485 
   void  Read_save_WS_data(void) 
   {
         ReadSensor(); 
         
         // saving code   
            iLastMsr[objToMsr] = COMBO_TMP_DATA;
            SensorResult  = COMBO_TMP_DATA;
            SaveMeasurments();          
            SendDebugMsg("saved TEMP data\r\n\0");    //server (%s-400)/10  type 2
                                
            objToMsr++;    //
            SensorResult  = COMBO_HUMID_DATA;
            iLastMsr[objToMsr] = COMBO_HUMID_DATA;
            SaveMeasurments(); 
            SendDebugMsg("saved HUMIDITY data\r\n\0 ");  //server %s     type 6   
             
            objToMsr++;            
            SensorResult  = WS_RAIN_DATA - eTotalRainMeter; 
                       
            if(SensorResult < 0)       //station reseted - counter has new value;
            {  
                SensorResult = WS_RAIN_DATA; 
                eTotalRainMeter = 0; 
            }   
            iLastMsr[objToMsr] = SensorResult;
            SaveMeasurments();          
            SendDebugMsg("saved Rain Meter data\r\n\0");  //server (%s*0.3) mm    type 40
              
            objToMsr++; 
            iLastMsr[objToMsr] = WS_UV_DATA;
            SensorResult  = WS_UV_DATA;
            SaveMeasurments();          
            SendDebugMsg("saved UV data\r\n\0");    //server  (uW/cm^2/100) = W/m^2    type 7
             
            objToMsr++; 
            iLastMsr[objToMsr] = WS_WSPEED_DATA;
            SensorResult  = WS_WSPEED_DATA;
            SaveMeasurments();          
            SendDebugMsg("saved Wind Speed data\r\n\0");  //server (%s/8*1.12 ) meter/Sec   type 45
            
        
          
            Glob_data_Saved = TRUE;  //exit without saving again later 
            if ((readClockBuf[4] == 0) && (readClockBuf[5] == 0))  //at midnight reset counters             
             {                      
                eTotalRainMeter = WS_RAIN_DATA;    //save daily total every midnight-enable calc daily rain -weather station
                SendDebugMsg("Daily RM data saved..\r\n\0");                                                                 
             }   
   }
  #endif 

void InitCombDef()
{
    BYTE i;

    NumSensors = 0;
    for (i = 0; i < MAX_SEN_NUM; i++)
    {
        ADCMux[i] = -1;
        PortIndex[i] = 0;
        SensorType[i] = 0;
        MIN_LIMIT[i] = MIN_INT;
        MAX_LIMIT[i] = MAX_INT;
    }

    for (i = 0; i < MAX_IO_TYPES; i++)
        IOType[i]= 0;

//    useExtInt1 = FALSE;
    // todo - is needed?
//    for (int i = 0; i < MAX_SEN_NUM * 4; i++)
//        unique_id[i] = 0;   //sensors id
}

//BYTE SetSensorCombination(BYTE *newCmb)
//{
//    BYTE i, tmp = 0;
//    //NumSensors = 0;
//    //init all vars refer to combination definition
//    InitCombDef();
//
//    for (i = 0; i < MAX_IO_TYPES; i++)
//    {
//        if (newCmb[i] != 0)
//        {
//            PortIndex[tmp] = i+1;
//            if (newCmb[i] == EC)
//            {
//                if (tmp > 6)
//                    return FALSE;
//                ADCMux[tmp] = -1;
//                ADCMux[(int)tmp+1] = -1;
//                ADCMux[(int)tmp+2] = -1;
//                SensorType[tmp] = EC;
//                SensorType[(int)tmp+1] = SM;
//                SensorType[(int)tmp+2] = TMP;
//                tmp += 3;
//            }
//            else
//            {
//
//                    if (i < 6)
//                    ADCMux[tmp] = i;
//                    SensorType[tmp] = newCmb[i];
//                tmp += 1;
//            }
//        }
//    }
//
//    // if type definition is ok -
//    NumSensors = tmp;            //add num of sensors added
//    for (i = 0; i < MAX_IO_TYPES; i++)
//        IOType[i] = newCmb[i];
//
//    return TRUE;
//}

 void ResetPumpFlags(void)
{
       char i;

                PumpActivated = FALSE;
                PumpActivationNeeded = FALSE;
                IsAlertNeeded = FALSE;

                FlagsStatus =  eFLAGS_STATUS;
                SetStatusReg( PUMP_ACTIVATION_NEEDED | IS_ALERT_NEEDED | PUMP_ACTIVAED | IS_ALERT_ACTIVATED | WLV_OVF_FLAG , 0);
                eFLAGS_STATUS =  FlagsStatus;

                msrAlertFlag = 0;
                for (i = SENSOR1; i < NumSensors; i++)
                {
                     OutOfLmtCnt[i] = 0;
                     OutOfLmtCntPump[i] = 0;
                     AlertStatus[i] = ALERT_WAIT;
                }
               
            //    SendDebugMsg("\r\nPump Reset ..\r\n\0");
            

}


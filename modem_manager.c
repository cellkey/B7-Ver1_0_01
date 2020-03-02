#ifndef MODEM_MANAGER_C
#define MODEM_MANAGER_C

#include "define.h"
#include "uart.h"
#include <mem.h>
#include "rtc.h"
#include <string.h>


 #define RX_FILE_OK 3
 #define GENERAL_ERR 4
 #define BAD_FORMAT 5
 #define NEW_VER_OK 6 
    
 extern void SendPostLength(int len);

char Pharse_params_struct(char *str, unsigned int index);
char RTC_Update_by_Server( char *str, int ptr);
void ModemForgetOldCop(void);
void SendStartDial();
char ModemTypeDetect(void);
void  server_ReDial(char index);
void ModemReset(void);
 void  Close_server_socket(void);
 void SaveErrorState(int ErrType);

//char Pharse_SDIparams_struct(char *str, char index);
extern flash  char RomVersion[];
extern bit bCheckRxBuf;
extern bit bExtReset;
extern bit bWaitForModemAnswer;
extern bit bEndOfModemTask;
extern bit bNeedToWait4Answer;
extern bit PumpActivated;
extern bit IsAlertNeeded;
extern bit PumpActivationNeeded;
extern bit CoverIsOpen ;
extern bit IsCoverAlertNeeded;
extern bit CoverAlertSent;
extern bit BackToNormalAlertEnabled;
extern bit MAGNET_SW_ON;
extern bit BackToNormalAlertNeeded;
extern bit QuickSensorsCheckNeeded;
extern bit bDemoMode;
extern bit IsAlertActivated;
extern bit pBread_saved ;             //flag to show if pBread saved
//extern bit bMoreDataToSend;
extern bit longAnswerExpected;
extern bit overFlow;
extern bit WLV_OVF ;
extern bit UpdateSession;
extern bit Illigal_Time_Val;
extern bit ModemIsOn;
extern bit IsScheduledCom;
extern bit new_flash_version;
extern bit Server_Error_msg; //400 or 500
extern bit SuccessUpdate;
extern bit TWI_err_alert_needed;
extern bit Power_Failure_alert_needed;
extern bit bNeedToWait4Answer;

#ifdef NETIV_ALERT_UNIT
extern bit POWER_FAILURE;
#endif

extern BYTE modemCurTask;
extern BYTE modemCurSubTask;
extern BYTE waitingTask;
extern BYTE dataSentOK;
extern BYTE prmSentOK;
extern BYTE toDoList;
extern BYTE BytesToSend;
extern BYTE ModemResponse;
extern BYTE objToMsr;
extern BYTE msrAlertFlag;
extern BYTE AlertStatus[MAX_SEN_NUM];
extern unsigned int timeCnt;
//extern BYTE lastByteIndex;
extern bit bReset;
extern int TimeLeftForWaiting;
extern unsigned int JasonLengh;
extern unsigned int rx0_buff_len;
extern BYTE rssi_val;
extern char bitErr_val;
//extern BYTE UpdatePrmArr[MAX_PRM_TASKS];
extern BYTE OutOfLmtCnt[MAX_SEN_NUM];
extern BYTE OutOfLmtCntPump[MAX_SEN_NUM];
extern BYTE bConnectOK;
extern char e2_writeFlag;
extern char readClockBuf[];	         //buffer for data reading from clock
extern char DataBlock[48];
//extern char clockBuf[7]; 		 //buffer for all clock operation need
extern char ComBuf[MAX_TX_BUF_LEN];
extern char RxUart0Buf[MAX_RX0_BUF_LEN];
extern char SensorAtAlertStatus;
extern unsigned int ErrorType;
extern char SavedDataLost;
extern char cuurent_interval;

extern int DataAtAlertStatus;
extern char TimeZone;
extern unsigned int timeCnt;
extern unsigned int nextCompare;
extern unsigned int nMaxWaitingTime;
extern unsigned int pBread;		//pointer to last read sensor data block in ext_e2
extern unsigned int pOriginalReadBlock;
extern int iVoltage;
extern int heat_time;
extern int iLastMsr[MAX_SEN_NUM];
extern int SensorResult;
extern unsigned int time_in_minutes;
// unsigned long GPSdataArr[4];
extern char GPS_strig[28];
extern unsigned int pBfirst;



extern  char Timer0_Ticks;
extern eeprom BYTE NumSensors;
extern eeprom char eBackToNormalAlertEnabled;
extern eeprom char eTimeZone;
extern eeprom int PUMP_MIN_LIMIT[MAX_SEN_NUM];
extern eeprom int PUMP_MAX_LIMIT[MAX_SEN_NUM];
extern eeprom unsigned char ePUMP_HEAT_TIME;
extern eeprom char eRelay1Enabled;
extern eeprom char eRelay2Enabled;
extern eeprom unsigned int logger_id;
extern eeprom BYTE SensorType[MAX_SEN_NUM];
extern eeprom BYTE IOType[MAX_IO_TYPES];
extern char gBUF[];
extern BYTE iFirstConnToday;
extern BYTE nUnreadBlocks;
extern BYTE bCheckBatr;
extern char CurrentHourOfFailure;
extern BYTE NotMonitored;

extern char FlagsStatus;

//extern unsigned char GetStatusReg(unsigned char mask);
extern void SetStatusReg(unsigned char mask, unsigned char val);
extern unsigned int RxUpdateFile(void);
extern int ValidityCheck (unsigned int Saddress);


unsigned char ICCID[] = "********************";
char COPS_value[15];
//char newURL[32];
char cEndMark = '\0';
char ComFailureCounter = 0;
//char ACTV_count = 0;
char ModemAgainCount;
int pStr2Bin (char *str, unsigned int p);
char updateErrorCount = 0;
char notification_index;
char MODEM_TYPE;
char sUUID[36];           
char TestPost;

int ServerResponseTimeOut;   //count down when get connected with server

bit bUpdateAddress;
bit bSendData;
bit ModemPwrSwapped;   //if TRUE other power switch needed when modem is powered
bit ComDelayNeeded = FALSE;
bit SeverResetCommand ;
bit  ServerComOn;
//char ALertComCounter;
//bit WaitNextHour;
 bit Relay1Enabled;
 bit Relay2Enabled;
 bit BackToNormalAlertActivated;
 bit DataSent = FALSE;
 bit ConnectedToServer = FALSE;
 bit Found200 =FALSE; 
  bit Found_200 = FALSE;
// bit Found210;
 bit DataPointersMoveNeeded;
// bit MemInitDone = FALSE;
 bit ClockUpdated = FALSE;


bit MODEM_GOT_IP = FALSE;
bit ModemRepeatNeeded = FALSE;         //for new trial of communication
bit ModemIsOn = FALSE;
bit ModemAgain = FALSE;
bit FirmwareUpdateTime = FALSE ;
bit UpdateFileOK = FALSE;

bit TestFlag = FALSE;
bit Backward_clock_update = FALSE; 
bit ExtendedInfoBlockNeeded = FALSE;
bit GPS_data_needed = FALSE;

//char moreData;  //sign if there is more data to send. 0x94 = more , 0x14 = nomore
BYTE nMaxFailuresNum;
BYTE prmUpdtIndex;
//BYTE readMode;
//BYTE bufIndexToUpd;
BYTE failCnt;

BYTE initCnt;
BYTE bPostAnswered;
BYTE bMakeReset;
BYTE bUseRoaming;
//BYTE bModemIsON;    //Danny - for demo
//BYTE PrmLen[MAX_PRM_TASKS] = {1,32,4,32,6,32,32,1,1,1,1,0,8,8};
BYTE nFailureCntr;
char Timer0_Ticks;
char ModemRepeatCount ;
#pragma keep+
extern eeprom char unique_id[]; //sensors id
extern eeprom unsigned char eStartConnectionH;        //first connection hour
extern eeprom unsigned char eConnectionInDay;        //number on connectionsin a day
extern eeprom unsigned char eConnectIntervalH;        //intervalbetween connections (hours)
//extern eeprom char eGMT;        //GMT offset param
extern eeprom char cpue2_interval_1;
extern eeprom int MIN_LIMIT[MAX_SEN_NUM];
extern eeprom int MAX_LIMIT[MAX_SEN_NUM];

eeprom unsigned char eRoamingDelay @0x74;      //eOperatorType
eeprom unsigned char eRoamingDelay = 15;      //eOperatorType
eeprom unsigned char eUseCntrCode @0x75;     //used now by BL to mark status
eeprom unsigned char eUseCntrCode = 0;
eeprom unsigned char eMobileNetCode[4]  @0x76;
eeprom unsigned char eMobileNetCode[4] = {'0','1','#','0'};
eeprom unsigned char eMobileCntryCode[4] @0x7B;
eeprom unsigned char eMobileCntryCode[4] = {'4','2','5','#'};

// if eMobileCntryCode & eMobileNetCode are integers
//eeprom unsigned int eMobileNetCode = 1;
//eeprom unsigned int eMobileCntryCode = 425;
eeprom  char eIPorURLval1[32] @0x80;
eeprom char eIPorURLval1[32] = {'c','r','e','a','-','c','e','l','l','.','c','o','m','#','0',\
                                        '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0'}; //32 bytes


//eeprom unsigned char eIPorURLval1[] =    "5.29.245.191#0000000000000000000"; //32 bytes  //hagai laptop
//eeprom unsigned char eIPorURLval1[] = "54.247.88.8#00000000000000000000"; //32 bytes  //amazon 1
//eeprom unsigned char eIPorURLval1[] = "62.90.59.100#0000000000000000000"; //32 bytes  //server 16
eeprom  char ePORTval1[4]   @0xA0;
eeprom  char ePORTval1[4]    = {'0','0','8','0'};
//eeprom  char ePORTval1[4]    = {'3','0','0','1'};      //kando
                          //4 bytes
eeprom  char eAPN[32]  @0xA4;
eeprom  char eAPN[32]         = {'i','n','t','e','r','n','e','t','m','2','m','.','a','i','r','.','c','o','m',\
                                        '#','0','0','0','0','0','0','0','0','0','0','0'};     //"internetm2m.air.com#000000000000";
//eeprom unsigned char eAPN[]         = "internetm2m.air.com#000000000000";     //"internetm2m.air.com#000000000000";
eeprom  char eCreacellPassWord[6]  @0xC7;
eeprom  char eCreacellPassWord[6]  = {'1','2','3','4','5','6'};               //6 bytes user PW for SMS
\
eeprom  unsigned char Option_1[15]    @0xCE;
eeprom  unsigned char Option_1[15]     = {'#','#','#','#','#','#','#','#','#','#','#','#','#','#','#'};   //,'0','0','0','0',\

eeprom  unsigned char eModemType  @0xDD; //same address as Option_1[15] -issue warning when compiled
eeprom  unsigned char eModemType = SIM800;

eeprom  unsigned char eReserved[14]  @0xDF;     //260117 for special EC treshold vlaue for PH pump activation limit
eeprom  unsigned char eReserved[14];

eeprom  unsigned int eECTresholdVal  @0xED;     //260117 for special EC treshold vlaue for PH pump activation limit
eeprom  unsigned int eECTresholdVal = 0;

eeprom  char Option_2[16]    @0xEF;
eeprom char Option_2[16]      = {'#','p','t','i','o','n','_','2','#','0','0','0','0','0','0','0'}; //16 bytes

eeprom  unsigned int eDataPtrSave[8]    @0xFF; //->10E                                      
eeprom unsigned int eDataPtrSave[8]= {0,0,0,0,0,0,0,0};   //pointers to data blocks temporary storage



eeprom char eFLAGS_STATUS @0x0110;
eeprom char eFLAGS_STATUS =  0;


//======================================
//see more eeprom vars at measure.c
//=======================================
eeprom char eErrIndex @0x0115;
eeprom char eErrIndex =  0;
eeprom unsigned int eErrArray[8]  @0x0116;
eeprom unsigned int eErrArray[8];

eeprom char  eICCID[5]  @0x0126 ;
eeprom char  eICCID[5]= {'0','0','0','0','0'} ;

eeprom unsigned int eCurrentYear @0x12B;
eeprom unsigned int eCurrentYear = 2019;


eeprom char eUUID[37] @0x0130;
eeprom char eUUID[37] = {'0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0',\
                         '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0'};

 #pragma keep-

//https://kando-staging.herokuapp.com:443/autosamplings/device_broadcast

//flash char END_MARK = '\n';
flash  unsigned char  AT_IsModemOK[] = "AT\r\n\0";             //5 bytes
flash  unsigned char  AT_IsModemReg[] = "AT+CREG?\r\n\0";     //10 bytes
 flash char AT_IsModemRegGPRS[] = "AT+CGREG?\r\n\0";
flash  unsigned char  AT_COPS_AUTO[] = "AT+COPS=0\r\n\0";      //11 bytes
flash  char  AT_COPS_MAN[] = "AT+COPS=4,2,@";       //12 bytes. must have # at the end

flash  unsigned char  AT_COPS_ASK[] = "AT+COPS?\r\n\0";
//flash unsigned char  AT_CELL_MONITOR[] = "AT#MONI\r\n\0";
flash unsigned char  AT_CSQ[] = "AT+CSQ\r\n\0";               //8 bytes
flash unsigned char AT_CCLK[] = "AT+CCLK?\r\n\0";
flash  char AT_CRSM[]= "AT+CRSM=214,28542,0,4,3,\"FFFFFF\"\r\n\0";   //reset sim card memory
 flash unsigned char  DE_ACTIVATE_ATCH[] =  "AT+CGATT=0\r\n\0";

//battery voltage check by modem
//flash unsigned char  AT_GPIO_ON[] =  "AT#GPIO=3,1,1\r\n\0 ";   // Switch ADC ON
//flash unsigned char  AT_ADC[] =      "AT#ADC=1,2\r\n\0 ";      // Value milivolt [Vm]  => Voltage input to sensor = Vm*4.7
//flash unsigned char  AT_GPIO_OFF[] = "AT#GPIO=3,0,1\r\n\0 ";   // Switch ADC OFF
//flash unsigned char AT_CRSM[] = "AT+CRSM=176,12258,0,0,10\r\n\0";



//flash unsigned char AT_CTZU[] = "AT+CTZU=1\r\n\0";
//flash unsigned char AT_NITZ[] = "AT#NITZ=1\r\n\0";

//GPRS connecting commands:
//flash unsigned char READ_PDP_CNTXT[] = "AT+CGDCONT?\r\0\n";                         //13

//SIM800
   flash  char CSTT[] = "AT+CSTT=@";    //39
   flash unsigned char CIICR[] = "AT+CIICR\r\n\0";                     //24
    flash unsigned char CIFSR[] = "AT+CIFSR\r\n\0";
    flash char CIPSTART[] = "AT+CIPSTART=\"TCP\",@";
    flash unsigned char CIPSHUT[] = "AT+CIPSHUT\r\n\0";
    flash unsigned char CIPCLOSE[] = "AT+CIPCLOSE=1\r\n\0";
    flash unsigned char AT_CIPMODE[] = "AT+CIPMODE=1\r\n\0";
    flash unsigned char AT_E0[] = "ATE0;&W\r\n\0";
     flash unsigned char AT_CCID1[] = "AT+CCID\r\n\0";   //SIIM800

//Telit


   flash unsigned char GPRS_ATTACH[] = "AT+CGATT=1\r\n\0";

   flash  char DEF_PDP_CNTXT[] = "AT+CGDCONT=1,\"IP\",@";    //39
   flash unsigned char DEF_QULT_MIN_PROF[] = "AT+CGQMIN=1,0,0,0,0,0\r\n\0";                     //24
   flash unsigned char DEF_QULT_REQ_PROF[] = "AT+CGQREQ=1,0,0,0,0,0\r\n\0";                     //24
   flash unsigned char DEF_SCKT_CNFG[] = "AT#SCFG=1,1,500,30,150,4\r\n\0";                      //24
   flash unsigned char ACTIVATE_CNTXT[] = "AT#SGACT=1,1\r\n\0";
//   flash  char  AT_IsIPactive[] = "AT#SGACT?\r\n\0";
   flash unsigned char DE_ACTIVATE_CNTXT[] = "AT#SGACT=1,0\r\n\0";                             //14
   flash char AT_TCP_OPN[] = "AT#SD=1,0,@";           //"AT#SD=1,0,1020,\"phytech1.dyndns.org\"\r\n\0 ";   //40
   flash unsigned char AT_TCP_CLS[] = "AT#SH=1\r\n\0";                               //9
   flash unsigned char MODEM_SHDN[] = "AT#SHDN\r\n\0";
   flash unsigned char MODEM_SYSHALT[] = "AT#SYSHALT\r\n\0";
   flash unsigned char AT_CCID[] = "AT#CCID\r\n\0";



// post commands
//flash unsigned char CGDCONT[] = "AT+CGDCONT= 1,"IP","internet","0.0.0.0",0,0\r\n\0";      //new 060316
//flash unsigned char AT_POST_TITLE_PRM[] = "POST /api/sensor/sensorparams HTTP/1.1\r\n\0";  //                 api/file/sensorparams
flash unsigned char AT_POST_TITLE_ALERT[] = "POST /m2m/update/sensorAlert HTTP/1.1\r\n\0";  //                 api/file/sensorparams
//flash unsigned char AT_POST_TITLE_DATA[] = "POST /api/sensor/sensordata HTTP/1.1\r\n#";   //36               api/file/sensordata



#ifdef KANDO_SERVER
//====================================== kando =====================================

flash unsigned char AT_POST_TITLE_DATA_LOGGER1[] = "POST /autosamplings/device_broadcast?unit_id=\0";   //kando
flash unsigned char AT_POST_ALERT[] = "POST /autosamplings/device_notification?unit_id=\0";
flash char HTTP1[] =" HTTP/1.1\r\n\0";
#endif

//===================================================================================
//flash unsigned char  POST_UPDATE_FIRMWARE[] = "POST /autosamplings/flash?unit_id=\0";     //update file
flash unsigned char  POST_UPDATE_FIRMWARE[] = "POST /m2m/update/flash_ HTTP/1.1\r\n\0";     //update file
flash unsigned char AT_POST_TITLE_DATA_LOGGER1[] = "POST /m2m/update/sensordata HTTP/1.1\r\n\0"; // 38 //add data string and / HTTP/1.1\r\n
flash unsigned char AT_POST_ALERT[] = "POST /m2m/update/general HTTP/1.1\r\n\0";;
flash unsigned char AT_POST_GPS_DATA[] = "POST crea-cell.com/m2m/update/gps HTTP/1.1\r\n\0";;    //gps data 201119
//=========================================================================================

flash unsigned char AT_POST_CONN[] = "Connection: keep-alive\r\n\0";       //24
flash unsigned char AT_POST_TYPE[] = "Content-Type: application/octet-stream\r\n\0";   //40
flash  char AT_POST_HOST[] = "Host: @#";         //Host: phytech1.dyndns.org:1011\r\n#";             //32

//flash unsigned char AT_POST_LENGTH1[] = "Content-Length: 226\r\n\r\n\0";                     //23
flash  char AT_POST_LENGTH4[] = "Content-Length: @#";                     //23
flash  unsigned char AT_POST_LENGTH3[] = "Content-Length: 130\r\n\r\n\0";                     //23
flash unsigned char AT_POST_FILE_HDR1[] = "Content-Disposition: form-data; name=\"file\"; \0";             //45
//flash unsigned char AT_POST_FILE_HDR_PRM[] = "filename=\"PARAMS.txt\"\r\n#";    //23
//flash unsigned char AT_POST_FILE_HDR_DATA[] = "filename=\"DATA.txt\"\r\n#";    //21
flash unsigned char AT_POST_FILE_HDR_GETDATA[] = "filename=\"GETPARAMPOST.txt\"\r\n#";    //29
flash unsigned char AT_POST_FILE_HDR2[] = "Content-Type: text/plain\r\n\r\n\0";          //28
flash unsigned char AT_POST_ACCEPT[] = "Accept: */*\r\n\0"; //13
flash unsigned char AT_POST_USER_AGENT[] = "User-Agent: Creacell/1.0\r\n\0"; //28
//flash unsigned char AT_POST_USER_AGENT[] = "User-Agent: Creacell-Kando/1.0\r\n\0"; //28
flash unsigned char UNITID[] = "Unit: \0";
flash unsigned char TIME_ZONE[] = "TIME_ZONE: \0";
flash unsigned char SIM_ID[] = "SIM_ID: \0";
flash char UUID[] = "FOTA_UID: \0";
flash char FW_VER[] = "FW_VERSION: \0";
flash char COPS_ID[] = "COPS_ID: \0";

//send flash init string to modem
void SendATCmd(flash unsigned char *bufToSend)
{
    BYTE i;

    i = 0;
    //copy flash string to buff
    do
    {
         ComBuf[i] = bufToSend[i];
         i++;
    }
    while (bufToSend[i] != cEndMark);

    BytesToSend = i ;

    //transmitt to local modem port
    TransmitBuf(0);


}
/*
BYTE GetContextStatus()
{
    BYTE i, b = 0;

    do            //if return "SGACT:"
    {
        if(RxUart0Buf[i++] == 0x2C)
            b = i;
    }
    while ((i < rx0_buff_len) && (b == 0));

    return RxUart0Buf[b];
}
*/


// translate rssi val from string to single byte
char ConvertRssiVal(void)
{
        BYTE i, b = 0;

        rssi_val = 100;
        for( i = 0; i < rx0_buff_len - 4 ; i++)
        {
            //if return "+CSQ:"
            if((RxUart0Buf[i] == 0x2B) &&
               (RxUart0Buf[(int)i+1] == 0x43) &&    //c
               (RxUart0Buf[(int)i+2] == 0x53) &&    //s
               (RxUart0Buf[(int)i+3] == 0x51) &&     //q
               (RxUart0Buf[(int)i+4] == 0x3A))
            {
                b = 1;
                //serch for ',':
                // if rssi val is only 1 digit - take it as is
                   if(RxUart0Buf[(int)i+7] == 0x2C)
                   {
                    //rssi val is 1 byte
                      rssi_val = RxUart0Buf[(int)i+6]-0x30;

                      if(RxUart0Buf[(int)i+9] == 0x0D)         //1 digit
                      bitErr_val =  RxUart0Buf[(int)i+8]-0x30;
                      else //2 digits
                      {
                             bitErr_val = RxUart0Buf[(int)i+8]-0x30;
                             bitErr_val *= 10;
                              bitErr_val  += RxUart0Buf[(int)i+9]-0x30;
                      }

                   }
                else
                    // if its 2 digits:
                     if(RxUart0Buf[(int)i+8] == 0x2C)
                     {
                        rssi_val = RxUart0Buf[(int)i+6]-0x30;
                        rssi_val *= 10;
                        rssi_val += RxUart0Buf[(int)i+7]-0x30;

                        if(RxUart0Buf[(int)i+10] == 0x0D)         //1 digit
                        bitErr_val =  RxUart0Buf[(int)i+9]-0x30;
                        else //2 digits
                          {
                              bitErr_val = RxUart0Buf[(int)i+9]-0x30;
                              bitErr_val *= 10;
                              bitErr_val  += RxUart0Buf[(int)i+10]-0x30;
                          }
                    }
                break;
            }
        }
        // no answer found (+CSQ:)
        if (b == 0)
            return FALSE;

        // if rssi is very low:             //Danny
//        if (rssi_val < 10)
//        {
//            // if should have done only data - send also params
//            if (toDoList == DO_DATA)
//                toDoList = DO_DATA_N_PRMS;
//        }

        return TRUE;
        //compare value with rssi min:
        //compare value with rssi min and rssi max (31 = 0x4E+0x31 = 0x7F-0x30 = 0x4F):
//        if((rssi_val >= MinRSSI) && (rssi_val <= 31))
//        {
//            //debug_byte = 1;
//             return 1;
//        }
//        else
//        {
//            //else: no muching bytes was found or rssi val too high:
//            //debug_byte = 2;
//            return 0;
//        }
}

//  addapted to Creacell
//void SetModemOn()
//{
//	//set power on
//    DDRA &= 0xBF;     //bit 6 input
//    PORTA |= 0x40;
//    if(PINA.6 == 0)        //ext power.
//    {
//           DDRD.4 = 1;
//          PORTD.4 = 1;          //Power Modem
//
//    }
//    else              //  3.6V bat at JP16
//    {
//           DDRB.3 = 1;
//           PORTB.3 = 1;          //Power Modem
//           ModemPwrSwapped = TRUE;   //power from BU battery.
//    }
//
//
//    delay_ms(1000);      // used to wait 100 milli sec. changed on 7/4/13
//    //start ignition
////     PORTC.3 = 1;
////     heat_time = 60;    // used to wait 3 sec. changed on 7/4/13
//////    PORTC.3 = 0;
////    initCnt++;
//    ModemResponse = TASK_COMPLETE;
//    bModemIsON = TRUE;
//
//}

void ModemIgnit(void)
{
      SendDebugMsg("Ignition pulse..\r\n\0");
             PORTB.7 = 1;        //HE910 modem ignition pulse
             delay_ms(4000);
             PORTB.7 = 0;
             ModemPwrSwapped = TRUE;   //power from BU battery.
             delay_ms(1000);
}

//V3 version
//HE910 201216
void SetModemPwrOn()
{
//	//set power on
     DDRD.7 = 1;
     PORTD.7 = 1;          //Power Modem BY MULT
     delay_ms(1);
     PORTD.7 = 0; 
      delay_ms(10);
     PORTD.7 = 1; 
      delay_ms(1);
      PORTD.7 = 0; 
     delay_ms(2);
     PORTD.7 = 1;
      delay_ms(3); 
     PORTD.7 = 0; 
     delay_ms(2);
     PORTD.7 = 1;  
     
     delay_ms(1000);      // used to wait 100 milli sec. changed on 7/4/13

    initCnt++;                //was marked!!. Danny removed mark 130515
    ModemResponse = TASK_COMPLETE;
//    bModemIsON = TRUE;

}


//  addapted to Creacell
//HE910 201216
void SetModemPwrOff()
{


	//set power off MEGA1280


        PORTD.7 = 0;      //in case of swapped modem voltage (BU battery or 3.6V bat as main)
        PORTC.0 = 0;      //if Ext Power
        delay_ms(10);
      //  bModemIsON = FALSE;
    ModemResponse = TASK_COMPLETE;
}

//send init string AT+WIPCREATE=2,1,"109.253.23.173",1007 to mode
void SendString(unsigned char *bufToSend, BYTE bufLen)
{
    BYTE i;

    for (i = 0; i < bufLen; i++)
        ComBuf[i] = bufToSend[i];
        //transmitt to local modem port
    BytesToSend = bufLen;
    TransmitBuf(0);
  //  ShowHexString(ComBuf);     //Danny- send to monitor
}

void SendPostHost()
{
    BYTE i, n;
    //build the host address from url+port
    n = CopyFlashToBuf(ComBuf, AT_POST_HOST);
    i = 0;
    while (eIPorURLval1[i] != '#')
    {
        ComBuf[(int)i+n] = eIPorURLval1[i];
        i++;
    }

      n += i;
      ComBuf[n++] = ':';
    cpu_e2_to_MemCopy(&ComBuf[n], ePORTval1, 4);
    n += 4;
    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
  //  ShowHexString(ComBuf);     //Danny- send to monitor
}

void PutString(unsigned char* dest, unsigned char* src, int len)
 {
    int i;
    for (i = 0; i <len; i++)
        dest[i] = src[i];
 }

// build the string of parameters to send to web service
void BuildParamsBuff()
{
    int index;
    BYTE cs, i,;
   unsigned char hBuf[2];

    index = 0;

    // Clock
    e2_writeFlag = 0; //enable reading the rtc
    rtc_get_timeAll (readClockBuf);     //new - Danny
    ComBuf[index++] = readClockBuf[0]; //year
    ComBuf[index++] = readClockBuf[1]; //month
    ComBuf[index++] = readClockBuf[2]; //day
    ComBuf[index++] = readClockBuf[4]; //hour
    ComBuf[index++] = readClockBuf[5]; //minute

     //Battery
    ComBuf[index++] = (unsigned char)((iVoltage >> 8) & 0xFF);     //address high
    ComBuf[index++] = (unsigned char)(iVoltage) ;                 //address low

       for (i = SENSOR1; i < NumSensors; i++)
    {
          if(AlertStatus[i] == TRHRESHOLD_CROSSED)
          {
               cpu_e2_to_MemCopy( &ComBuf[index], &unique_id[(int)i*4], 4);   //sensor id
                index += 4;
                ComBuf[index++] = SensorType[i];             // set  type
                ComBuf[index++] = 1;  //AlertType[i]        //code for alert
                int2bytes(iLastMsr[i], hBuf);               //convert int to 2 bytes (high, low)
                ComBuf[index++] = hBuf[0] ;                 //Data high
                ComBuf[index++] = hBuf[1] ;                 //Data low
                ComBuf[index++] = '-' ;                 //end of alert block for sensor


          }
    }

    //check sum
    cs = CheckSum(ComBuf, index, 1);
    ComBuf[index++] = cs;

    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';
    BytesToSend = index;
}


void SendInfoBlock(void)    //info block 64 bits
{
   
   
    int index,i;
  //  int post_type;
    BYTE cs;
    unsigned int unit_id, tData;
    char buf[2];
    index = 0;   
    
     //padding block
     for (i=0; i < 64; i += 2)
     {
          ComBuf[i] = 0xFF;
          ComBuf[i+1] = 0x7F;         
     }
    
    unit_id = logger_id;                 //read eeprom val
    int2bytes((unsigned int)unit_id , buf);        //make word shifted  as two bytes
    MemCopy( &ComBuf[index], buf, 2);                  //enter 2 bytes logger id
    index += 2;

//    ComBuf[index++] = 0x7E;   //block ID     
//    ComBuf[index++] = 0x05;   //Lengh   

     ComBuf[index++] = NumSensors+1;   //first sensor type as block name
   //   ComBuf[index++] = 0x17;   //block ID     
    ComBuf[index++] = 0x00;   //Lengh
      
	//status register
    // post_type = 0;       //attempts
                             
//    int2bytes(post_type, buf);        //make it swaped bytes  
//    ComBuf[index++] = buf[0]; 
//    ComBuf[index++] = buf[1];  

    ComBuf[index++] = IOType[0];
    ComBuf[index++] = 0;
     
    //pumping time   - not used in that version
    int2bytes(0, buf);        //make it swaped bytes  
    ComBuf[index++] = buf[0]; 
    ComBuf[index++] = buf[1];    //8 bytes
    
    for(i = 0; i<8; i++)                                //read error array into data block
    {     
//         SensorResult  = time_in_minutes;  
//      //   SensorResult += (unsigned int)(i<<8); 
//         tData = (int)(9 << 12); 
//          PrintNum((long)tData);
//        SensorResult |=  tData; 
      //  PrintNum((long)SensorResult);
         tData = eErrArray[i];        
         int2bytes((unsigned int) tData, buf);        //make word shifted  as two bytes
         MemCopy( &ComBuf[index], buf, 2);   
         index += 2;
    }
    //check sum
    cs = CheckSum(ComBuf, 61, 1);
      
    index = 61;
    ComBuf[index++] = cs;
    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';   
                                                                                                                                               
    bWaitForModemAnswer = FALSE;
    NotMonitored = FALSE;    //dont show on screen at  TransmitBuf() (old flag)
      BytesToSend = index;   
                     
//    ShowHexString(ComBuf, BytesToSend );  //Danny- send to monitor
	TransmitBuf(0);                //send info block
       
	SendDebugMsg("Errors info block has sent..\r\n\0 ");
//       BytesToSend = 64;
//      ShowHexString(ComBuf, BytesToSend );     //Danny- send to monitor debug only   
   

}


void BuildDataStr()
{
    #define POST_DATA  0x00
 //    #define POST_SPECIAL  0x01
    #define POST_ALERT 0x03        //bit 0 high for special post
    #define POST_PUMP  0x04        //bit 0 high for special post
    #define POST_BACK_NORMAL  0x09 //bit 0 high for special post
    #define WLV_OVF_STAT 0x10       //060916
    #define MAG_SW_ON  0x40
    #define DATA_LOST  0x80;

    int index;
    int post_type = POST_DATA;
    BYTE cs;
    unsigned int unit_id;
    char buf[2];



//     #ifdef DebugMode
//    SendDebugMsg("\r\nBuildDataStr()\r\n\0 ");
//    #endif DebugMode

    index = 0;
      // Logger ID + sensor ID
    unit_id = logger_id;                 //read eeprom val
    int2bytes((unsigned int)unit_id , buf);        //make word shifted  as two bytes
    MemCopy( &ComBuf[index], buf, 2);                  //enter 2 bytes logger id
    index += 2;



    cpu_e2_to_MemCopy( &ComBuf[index], &unique_id[objToMsr], 1);   //sensor ID - 1 byte + empty byte
    index ++;

    //reserved byte
    ComBuf[index++] = 0;       //reserved byte

     //sensor type
     ComBuf[index++] = GetSensorIOType();     // set server type

    // interval
    //cpu_e2_to_MemCopy( &ComBuf[index++], &cpue2_interval_1, 1);
    ComBuf[index++] = (char)DataBlock[1] * INTERVAL_PARAM; // interval of this block  5 min change

     //Battery  voltage
     ComBuf[index++] = (unsigned char)(iVoltage) ;                 //list
    ComBuf[index++] = (unsigned char)((iVoltage >> 8) & 0xFF);     //most


     //RSSI
     ComBuf[index] = (unsigned char)(rssi_val) ;
     index++;

     //Prog Version
      cpu_flash_to_MemCopy(&ComBuf[index], RomVersion, 1);   //Danny - prog ver data -2 BYTES-V6
      index++;

    if(IsAlertNeeded == TRUE)    //new - version 0x32 - send sensor index and over TH data
    {
        int2bytes((int)SensorAtAlertStatus +1, buf);
        MemCopy( &ComBuf[index], buf, 2);
        index += 2;

          int2bytes(DataAtAlertStatus, buf);
        MemCopy( &ComBuf[index], buf, 2);
        index += 2;
    }
    else
    {
        // min & max values
        int2bytes(MIN_LIMIT[objToMsr], buf);
        MemCopy( &ComBuf[index], buf, 2);
        index += 2;

        int2bytes(MAX_LIMIT[objToMsr], buf);
        MemCopy( &ComBuf[index], buf, 2);
        index += 2;
    }

    // Timestamp
    PutString(&ComBuf[index],&DataBlock[3], 5);         //DataBlock[10] - used to be cos offset of 7 from unknown reasen
    index += 5;

      // status code
    if(IsAlertNeeded)
        post_type |= POST_ALERT;

    if ( PumpActivated)  //Danny - send bit once until PumpActivated is reset
        post_type |= POST_PUMP;
 else if(WLV_OVF == TRUE )
    post_type |= WLV_OVF_STAT;           //060916 Water level over flow alert

   if ( MAGNET_SW_ON == TRUE)          //250117 mark dtata as initiated by use magnet
      post_type |= MAG_SW_ON;

    if(BackToNormalAlertNeeded == TRUE)
    post_type |= POST_BACK_NORMAL;           //notify sever by bit 3

    if(SavedDataLost == 1)
    {
       post_type |= DATA_LOST;          //if reset memory accured
       SavedDataLost = 0;
    }

    int2bytes(post_type, buf);
    MemCopy( &ComBuf[index], buf, 2);
    index += 2;

    // values:
    PutString(&ComBuf[index],&DataBlock[8], 40);      //DataBlock[15] - KANAL
    index += 40;


    //check sum
    cs = CheckSum(ComBuf, index, 1);
    ComBuf[index++] = cs;
    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';

 //    ComBuf[index++] = '/';        //Hagai asked to concel it


    BytesToSend = index;             //62 bytec

}

void SendPostParam()
{
   //  char endFileStr[10];

     // sign that for all next transmits - no need to wait for ana answer from modem
     bNeedToWait4Answer = FALSE;
     bSendData = 1;

     // preare end file mark string
//     endFileStr[0] = '-';                //Danny- no need for creacell
//     endFileStr[1] = '-';
//     cpu_flash_to_MemCopy(&endFileStr[2], PHYTECH_FILE_END_MARK, 8);

     //--- send post header---
  //   SendATCmd(AT_POST_TITLE_PRM);
//     SendATCmd(AT_POST_TITLE_ALERT );      //Danny for creacell alert stat
//     SendATCmd(AT_POST_CONN);
//     SendATCmd(AT_POST_TYPE);
//  //   SendATCmd(PHYTECH_FILE_END_MARK);
//     //GPRS_send_init_flash_string(AT_POST_HOST);
//     SendPostHost();
//     SendATCmd(AT_POST_LENGTH1);
//
//     // send file header:
//     SendString(endFileStr, 10);         //10
//     SendATCmd(AT_POST_FILE_HDR1);       //45
//     SendATCmd(AT_POST_FILE_HDR_PRM);    //23
//     SendATCmd(AT_POST_FILE_HDR2);       //28

 //-------------Danny- as in post data
     SendATCmd(AT_POST_TITLE_ALERT );           //Danny for creacell alert stat
      SendPostHost();
     //   SendATCmd(AT_POST_HOST);                //25      "Host: 5.29.245.191:8080\r\n\0"
        SendATCmd(AT_POST_TYPE);                //40      "Content-Type: application/octet-stream\r\n\0"
        SendATCmd(AT_POST_ACCEPT);              //13      "Accept: */*\r\n\0"
        SendATCmd(AT_POST_USER_AGENT);          //28      "User-Agent: Creacell/1.0\r\n\0"
        // build post body                                    //110
        BuildParamsBuff();
        SendPostLength(BytesToSend);                     //        12 bytes: ID-4, code-1, data-2, date/time-5, cs-1, CRLF-2

     // send post
        TransmitBuf(0);
    // ShowHexString(ComBuf);     //Danny- send to monitor

     bNeedToWait4Answer = TRUE;
//     SendString(endFileStr, 10);            //10

//     lastByteIndex = 0;
     longAnswerExpected = 1;
     overFlow = 0;
}

//#ifdef EXT_SERVICE
void SendPostLength(int len)
{
    BYTE i, n;
    char s[4];
    //build the host address from url+port
    n = CopyFlashToBuf(ComBuf, AT_POST_LENGTH4);
    i = 0;

    //make bin as ascii
    do
    {
        s[i++] = (char)(len % 10);
        len = len / 10;
    }
    while (len > 0);
    for (; i > 0; i--)
       ComBuf[n++] = s[i-1] + 48;


    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
   // ShowHexString(ComBuf);     //Danny- send to monitor
}


 //adjusted for creacell way
void SendPostData(/*BYTE readMode*/)
{
        char mBuf[11];
  
        int i,numOfBytes, tmpNblk;
        char tObj2Msr;
        char bytes2send;
         unsigned int unit_id;     
         char Tzone;
 //    static char count = 0;    
         
         tmpNblk = 0;
         tObj2Msr = objToMsr;
         DataSent = FALSE;
         #ifdef NO_SERVER
         objToMsr = 0;      //debug-remov
         #endif

  //count blocks of data to send
 if( Illigal_Time_Val == FALSE)
  {
  do
  {
       
         SendDebugMsg("\r\n Sensor index:   ");
         PrintNum((long)objToMsr+1);
       

      if (_GetMeasurments_(1) == FALSE)      //Danny - new function- Counting all unread blocks
      {

         SendDebugMsg("Failed Blocks Count. Reseting memory!\r\n\0 ");
         InitDataBlocks(cuurent_interval);
         ModemResponse = TASK_FAILED;
         return;
      }
      else
      {
          tmpNblk += nUnreadBlocks;             // collect  blocks  quant
          objToMsr++;                           //go to next sensor location in mem
      }
  }
   while (objToMsr  < NumSensors);

      pBread_saved = TRUE;
    #asm("cli")
       eRESTORE_DATA_POINTERS = NEED_TO_RESTORE;        //flag in eeprom
    #asm("sei")
 }
 else  tmpNblk = 0;

    objToMsr = tObj2Msr;                      //regain initial number
    unit_id = logger_id;


    Num2String_Modem((long)unit_id);      //fill gBUF with loggr id ascii number
       // sign that for all next transmits - no need to wait for an answer from modem
        bNeedToWait4Answer = FALSE;



        //calc the length
         numOfBytes =(int)  tmpNblk * 64; // 62;               //        Is it the right way????????????  
         
         if (ExtendedInfoBlockNeeded == TRUE)                        //send info blok 250318
          numOfBytes += 64;
        SendATCmd(AT_POST_TITLE_DATA_LOGGER1);  //38     "POST /m2m/update/sensordata HTTP/1.1\r\n\0"   
        SendPostHost(); 
        SendATCmd(AT_POST_TYPE);                //40      "Content-Type: application/octet-stream\r\n\0"
        SendATCmd(AT_POST_ACCEPT);              //13      "Accept: */*\r\n\0"
        SendATCmd(AT_POST_USER_AGENT);                 //28      "User-Agent: Creacell/1.0\r\n\0"

     //----------new url with unit id-----------------------  
     #ifdef KANDO_SERVER
       
         for(i = 0; i < 5; i++)
         ComBuf[i] = gBUF[i];
         BytesToSend = 5 ;
         TransmitBuf(0);     //send unit id digits only

         SendATCmd(http1); 
     #endif   
           
        SendATCmd(UNITID);        //send looger id to server
        for(i = 0; i < 5; i++)
        ComBuf[i] = gBUF[i];
        ComBuf[i] =  '\r';
        ComBuf[++i] =  '\n';
        BytesToSend = 7 ;
         TransmitBuf(0);
          //------------------Program Version---------------                
           for(i = 0; i< 6; i++)
           mBuf[i] = RomVersion[i]; 
           mBuf[i] = '\0';
           sprintf(ComBuf,"FW_VERSION: %s\r\n",mBuf);
     
           BytesToSend = 20; //12 + i;
           TransmitBuf(0); 
       //=================COPS===========================                  
         i=0; 
         while(COPS_value[i] != '\0')
         {
             mBuf[i] =  COPS_value[i];
             i++;
         } 
         mBuf[i] = '\0';
         sprintf(ComBuf,"COPS_ID: %s\r\n",mBuf); //set string to be sent);  
         BytesToSend = 11 + i;
         TransmitBuf(0);
        //---------------------CSQ---------------------------- 
         
         sprintf(ComBuf,"CSQ: %02d\r\n\0",rssi_val); //set string to be sent);  
         BytesToSend = 5 + 4;
         TransmitBuf(0);
   
//   #ifdef KANDO_SERVER

        Tzone = eTimeZone;

        SendATCmd(TIME_ZONE);
        if(Tzone > 0)
        {
           ComBuf[0] =  '+';
        }
        else
        {
            Tzone *= -1;
            ComBuf[0] =  '-';

        }
         ComBuf[1] =  (Tzone / 10) + 0x30;
         ComBuf[2] =  (Tzone % 10) + 0x30;

         ComBuf[3] =  '\r';
         ComBuf[4] =  '\n';
         BytesToSend = 5 ;
         TransmitBuf(0);
//  #endif

//        SendATCmd(SIM_ID);        //send sim id to server
//        for(i = 0; i < 20; i++)
//        ComBuf[i] = ICCID[i];
//        ComBuf[i++] =  '\r';
//        ComBuf[i++] =  '\n'; 
//        ComBuf[i++] =  '\r';
//        ComBuf[i++] =  '\n'; 
//        BytesToSend = i ;
//        TransmitBuf(0);

        SendATCmd(SIM_ID);        //send sim id to server 5 digits
        for(i = 0; i < 5; i++)
        ComBuf[i] = eICCID[i];
        ComBuf[i] =  '\r';
        ComBuf[++i] =  '\n';
        BytesToSend = 7 ;
         TransmitBuf(0);    
  //--------------------------------------------------       
//           for(i = 0; i< 6; i++)
//           mBuf[i] = fw_version[i];
//           mBuf[i] = '\0';
//           sprintf(ComBuf,"FW_VERSION: %s\r\n",mBuf);
//     
//           BytesToSend = 20; //12 + i;
//           TransmitBuf(0); 
 //--------------------------------------------------- 
                   //
        SendPostLength(numOfBytes);
 
  if( Illigal_Time_Val == FALSE)
  {     
    
    do{

          #ifdef NO_SERVER
         objToMsr = 0;      //debug-remov
         #endif    
         
         //get data block
         while((GetMeasurments(1) == FALSE) && (objToMsr < NumSensors))
       //  while((GetMeasurments(0) == FALSE) && (objToMsr < NumSensors))//get all blocks
          {
                 objToMsr++;
          }
          if(objToMsr ==  NumSensors)
          {
                  SendDebugMsg("Failed GetMeasurments(1)\r\n\0 ");
                   ModemResponse = TASK_FAILED;
                   return;
          }
      do
         {
                  // build post body
                 BuildDataStr();  //adding creacell part 
                 
                    //send the post
                 if(((objToMsr +1) == NumSensors) && (nUnreadBlocks == 1))      //last senor data transmit
                 bNeedToWait4Answer = TRUE;           //  Danny-prepare for answer check


                  bytes2send = BytesToSend ;    //save   BytesToSend
                  NotMonitored = TRUE;   //added 241015 - dont show data block on monitor
                 TransmitBuf(0);           //send data block                 
                 BytesToSend = bytes2send  ;   //restore

            //---------------------debug only------------------------
            //        #ifdef DebugMode
          //           ShowHexString(ComBuf, BytesToSend );  //Danny- send to monitor
            //        #endif DebugMode
            //------------------------------------------------------------
                    
                      nUnreadBlocks--;
                      if(  nUnreadBlocks)
                      GetMeasurments(2);    //move pBread
                 //   ServerResponseTimeOut = 70;
         }
         while (nUnreadBlocks);

             objToMsr++;
              #asm ("wdr");
   }
   while (objToMsr  < NumSensors);
 }   
 
       ServerResponseTimeOut = 70;          //shorter than WD
       TimeLeftForWaiting = 70;
       DataSent = TRUE;
       longAnswerExpected = TRUE;         //Danny - buffer is 255 bytes-no need
//       NeedToWait4Answer = TRUE;

      rx0_buff_len = 0;
      overFlow = 0;

}
//ask server for fota file
void SendPostAckUpdate(char index)
{


     //   char  i;
   //   unsigned int unit_id ;

 

       //   SendDebugMsg("SendPostAckUpdate()..\r\n\0 ");

        bNeedToWait4Answer = FALSE;
//       i = index; //dummy - not used
//        NotMonitored = FALSE;     
//        SendATCmd(POST_UPDATE_FIRMWARE);
////Kando way                 
//        unit_id = logger_id;
//        Num2String_Modem((long)unit_id);      //fill gBUF with loggr id ascii number
////        for(i = 0; i < 5; i++)
////        ComBuf[i] = gBUF[i];
////        BytesToSend = 5 ;
////        TransmitBuf(0);     //send unit id digits only
////        SendATCmd(HTTP1);
//
//        SendPostHost();
//        SendATCmd(AT_POST_TYPE);                //40      "Content-Type: application/octet-stream\r\n\0"          
//        SendATCmd(AT_POST_ACCEPT);         
//        SendATCmd(AT_POST_USER_AGENT);          //28      "User-Agent: Creacell/1.0\r\n\0"
//
//        SendATCmd(UNITID);        //send looger id to server
//        for(i = 0; i < 5; i++)
//        ComBuf[i] = gBUF[i];
//        ComBuf[i] =  '\r';
//        ComBuf[++i] =  '\n';
//        BytesToSend = 7 ;
//         TransmitBuf(0);
//
//
//         SendPostLength(0);

         if (index == 2)                 //ask FOTA update   
         {
                sprintf(ComBuf,"a\r\nSTART_FOTA\r\n");  //a\r\n START_FOTA\r\n 
                 BytesToSend = 15; 
                
         }  
         else   if (index == 3) 
         {
                sprintf(ComBuf,"7\r\nFOTA_OK\r\n");  //a\r\n START_FOTA\r\n 
                 BytesToSend = 11; 
         }  
         else if (index > 3) 
         {
                 sprintf(ComBuf,"a\r\nFOTA_ERROR\r\n");  //a\r\n START_FOTA\r\n 
                 BytesToSend = 15; 
         }   
               
      //   NotMonitored = TRUE;      //dont show payload on screen  at TransmitBuf()
          TransmitBuf(0);
      longAnswerExpected = TRUE;         //Danny - buffer is 255 bytes-no need
//      bNeedToWait4Answer = FALSE;
      overFlow = 0;

      #asm ("wdr");
       ServerResponseTimeOut = 70;          //shorter than WD
}

//adjusted for creacell way
//void SendPostInfo(char index)
//{
//
//        unsigned int tmpInt;
//        char buf[2];
//       unsigned int unit_id ;
//
//
////          SendDebugMsg("Send Alert Post..\r\n\0 ");
//
//        bNeedToWait4Answer = FALSE;
//
//        unit_id = logger_id;
//        Num2String_Modem((long)unit_id);      //fill gBUF with loggr id ascii number
//
//        SendATCmd(AT_POST_ALERT);  //   POST /m2m/update/general HTTP/1.1\r\n\0" 
//
//        SendPostHost();
//        SendATCmd(AT_POST_TYPE);                //40      "Content-Type: application/octet-stream\r\n\0"
//        SendATCmd(AT_POST_ACCEPT);              //13      "Accept: */*\r\n\0"
//        SendATCmd(AT_POST_USER_AGENT);          //28      "User-Agent: Creacell/1.0\r\n\0"
//
//// #ifdef KANDO_SERVER    //Kando veraion
//
//               SendPostLength(6);                             //
//              // build post body
//              if(index == 0)       //cover alert
//              {
//                  ComBuf[2] = 0xF1;    //code F1, F2,F3..  
//                  CoverAlertSent = TRUE;             //to be checked in CheckResult()
//              }
//              else  if(index == 1)    //update ACK to server
//              {
//                  ComBuf[2] = 0xF2;    //code F1, F2,F3..
//              }
//             else  if(index == 2)    //handle clock- twi error 
//             {
//                  ComBuf[2] = 0xF4;    //code F1, F2,F3..
//             }
//
//              tmpInt = logger_id;                 //read eeprom val
//              int2bytes((unsigned int)tmpInt , buf);        //make word shifted  as two bytes
//               MemCopy( &ComBuf[0], buf, 2);                  //enter 2 bytes logger id
//              ComBuf[3] = 0;
//              ComBuf[4] = 0;
//              ComBuf[5] = 0;
//
//              BytesToSend = 6 ;
//
//// #else    //creacell server
////
////               SendPostLength(7);                     //
////              // build post body
////              ComBuf[0] = 0x7E;    //key
////              ComBuf[1] = 0x01;    //num of bytes
////              if(index == 0)       //cover alert
////              {
////                  ComBuf[2] = 0xF1;    //code F1, F2,F3..
////              }
////              else  if(index == 1)    //update ACK to server
////              {
////                   ComBuf[2] = 0xF2;    //code F1, F2,F3..
////              }
////
////              tmpInt = logger_id;                 //read eeprom val
////              int2bytes((unsigned int)tmpInt , buf);        //make word shifted  as two bytes
////               MemCopy( &ComBuf[3], buf, 2);                  //enter 2 bytes logger id
////
////              cpu_e2_to_MemCopy( &ComBuf[5], &unique_id[0], 1);   //first sensor ID - 1 byte
////              ComBuf[6] = 0;
////               BytesToSend = 7;
//// #endif
//
//         bNeedToWait4Answer = TRUE;
//         NotMonitored = TRUE;      //dont show payload on screen  at TransmitBuf()
//        //send the post
////    #ifdef NO_SERVER
////   //    ShowHexString(ComBuf, BytesToSend );     //Danny- send to monitor
////    #else
//        TransmitBuf(0);
//
////---------------------------------------------------
////        #ifdef DebugMode
// //          ShowHexString(ComBuf, BytesToSend );  //Danny- send to monitor
////        #endif DebugMode
////------------------------------------------------------------
//
//  //  #endif
//
//      longAnswerExpected = TRUE;         //Danny - buffer is 255 bytes-no need
//      bNeedToWait4Answer = TRUE;
//      overFlow = 0;
//   //   CoverAlertSent = TRUE;             //to be checked in CheckResult()
//      #asm ("wdr");
//       ServerResponseTimeOut = 70;          //shorter than WD
//}

void SendPostInfo(char index)
{
     //  char eUUID[]="1ba90843-acc8-4201-bf20-53c797c7bdc5" ;
        unsigned int i, tmpInt;
        char buf[2]; 
        char mBuf[6];
       unsigned int unit_id ; 
       
       #define MH_OPEN          0xF1               //to be in byte 2
       #define PARMS_UPDATED    0xF2
	   #define TWI_ERROR        0xF4 
      //update fota codes
       #define UPDATE_FILE_OK   0xF5
       #define GENERAL_ERROR    0xF6
       #define POWER_DOWN    0xF7
       #define NEW_VERSION_OK   0xF8  



         SendDebugMsg("Send Post Alert..\r\n\0 ");
         bNeedToWait4Answer = FALSE;
         
         
         if(index == 8)
         SendATCmd(AT_POST_GPS_DATA); 
         else 
         SendATCmd(AT_POST_ALERT);   //"POST /m2m/update/general HTTP/1.1\r\n\0";;
                           
         SendPostHost();
         SendATCmd(AT_POST_TYPE);                //40      "Content-Type: application/octet-stream\r\n\0"          

         SendATCmd(AT_POST_ACCEPT);          
         SendATCmd(AT_POST_USER_AGENT);          //28      "User-Agent: Creacell/1.0\r\n\0" 
       
        unit_id = logger_id;
        Num2String_Modem((long)unit_id);      //fill gBUF with loggr id ascii number   
        SendATCmd(UNITID);        //send looger id to server
        for(i = 0; i < 5; i++)
        ComBuf[i] = gBUF[i];
        ComBuf[i] =  '\r';
        ComBuf[++i] =  '\n';
        BytesToSend = 7 ;
        TransmitBuf(0);   
              
          //------------------Program Version---------------                
           for(i = 0; i< 6; i++)
           mBuf[i] = RomVersion[i]; 
           mBuf[i] = '\0';
           sprintf(ComBuf,"FW_VERSION: %s\r\n",mBuf);
     
           BytesToSend = 20; //12 + i;
           TransmitBuf(0); 

//         i=0; 

         
//        if((index > 2) && (index < 7 ))   //FOTA process - UUID string neeed
//        {
//             SendATCmd(UUID);
//            if(index == 6)     //read uuid from eeprom
//            {
//                 for (i = 0; i < 36; i++)
//                 ComBuf[i] = eUUID[i];         //fill buffer with UUID  sent by server
//
//            }
//            else
//            {
//                  for (i = 0; i < 36; i++)
//                  ComBuf[i] = sUUID[i];         //fill buffer with UUID sent by server
//            }      
//            ComBuf[36] = '\r';
//            ComBuf[37] = '\n';
//            BytesToSend = 38;
//            TransmitBuf(0);
//       }
       
        if(index == 8)       //gps data
        SendPostLength(26);
        else
         SendPostLength(6);
         NotMonitored = TRUE;      //dont show payload on screen  at TransmitBuf()   
         
         
           // build post body 
            for(i = 0; i < 6; i++)
            ComBuf[i] = 0;    
                          
          tmpInt = logger_id;                          //read eeprom loggewr id val
          int2bytes((unsigned int)tmpInt , buf);        //make word shifted  as two bytes
           MemCopy( &ComBuf[0], buf, 2);                 //load buffer
                                   //
          //error codes loading
          if(index == 0)              //cover alert
          {
              ComBuf[2] |= MH_OPEN;    //cover sw on 
              ComBuf[2] = 0xF1;    //cover sw on 
              CoverAlertSent = TRUE;     //to be checked in CheckResult()
          }
          else  if(index == 1)
          {
              ComBuf[2] |= PARMS_UPDATED;     //update params ok
          }
		   else  if(index == 2)
		   {
			   ComBuf[2] |= TWI_ERROR;     //update params ok
		   }
           else  if(index == 3)
          {
              ComBuf[2] |= UPDATE_FILE_OK;     //Receiving file ok
          }
           else  if(index == 4)
          {
              ComBuf[2] |= GENERAL_ERROR;     //Receiving file failed

          }
           else  if(index == 5)
          {
           //   ComBuf[2] |= BAD_FILE_FORMAT;     //Read file ok, validiation failed

          }
           else  if(index == 6)
          {
              ComBuf[2] |= NEW_VERSION_OK;     //update version success
          } 
           else  if(index == 7)
          {
              ComBuf[2] = POWER_DOWN;     // 
                CoverAlertSent = TRUE;    //need that flag for later.
          }  
           else  if(index == 8)
          {   
              
               for(i = 0; i < 26; i++)
                ComBuf[i] = GPS_strig[i];        // Motor sensor not active
              IsAlertNeeded = FALSE; 
               BytesToSend = 26 ;
          }
//          else  if(index == 9)
//          {
//              ComBuf[4] |= NO_DRY_FAULT;     //  Motor sensor always active
//              IsAlertNeeded = FALSE;
//          }    
         if(index != 8)
         BytesToSend = 6 ;

         bNeedToWait4Answer = TRUE;
         NotMonitored = TRUE;      //dont show payload on screen  at TransmitBuf()   
       
         
         TransmitBuf(0);

//---------------------------------------------------
           if(index == 8)
            BytesToSend = 26 ;
            else
            BytesToSend = 6 ;
          ShowHexString(ComBuf, BytesToSend );  //Danny- send payload to monitor for debug


//------------------------------------------------------------

  //  #endif
       ServerResponseTimeOut = 70;          //shorter than WD
       TimeLeftForWaiting = 70;
       DataSent = TRUE;
       longAnswerExpected = TRUE;         //Danny - buffer is 255 bytes-no need
     
       bNeedToWait4Answer = TRUE;
       overFlow = 0;  
       Timer0_Ticks = 50;                    
    //   delay_ms(1000);         
      #asm ("wdr");  
           
}




BYTE IsPrmToUpdate()
{
//    BYTE i;
//    for (i = 0; i < MAX_PRM_TASKS; i++)   //Danny - Not in use now
//        if (UpdatePrmArr[i] == '1')
//        {
//            prmUpdtIndex = i;
//            return TRUE;
//        }
    return FALSE;
}

BYTE GetBufferIndex(int i)
{
    BYTE b;

    if (i >= MAX_RX0_BUF_LEN)
        b = (BYTE)i - MAX_RX0_BUF_LEN;
    else
        b = i;
    return b;
}

//+COPS: 0,0,"Orange"
char Get_COPS()
{
    int n, i = 0;  
   
    while ((RxUart0Buf[i] != '"') && (i < (rx0_buff_len - 5)))   //look for first "
        i++; 
         
   if(i < (rx0_buff_len - 5))
   {     
        i++;      //point to string
        n = 0;
        while (RxUart0Buf[i] != '"') 
        {
            COPS_value[n++] = RxUart0Buf[i++];
        }         
        COPS_value[n] = '\0';  //termination char  
        return 1;
   }
   else 
   {
         for(i = 0; i< 8; i++)
         COPS_value[i] = 'x';  
         COPS_value[i] = '\0';
         SendDebugMsg("No COPS value found..\r\n\0"); 
   } 
   return 0;   
}

BYTE IsOK()
{
//    BYTE index = 0; 
    char *ptr;

  //   index = 0;

 
    ptr = strstrf( RxUart0Buf, "OK");  
    if(ptr != NULL)
    return TRUE;
    return FALSE;
      
//    while (index < rx0_buff_len-1)
//    {
//
//     //  _putchar1(RxUart0Buf[index]);  //deeeeebbbbuuuuuggggggg
//        if ((RxUart0Buf[index] == 'O') && (RxUart0Buf[(int)index+1] == 'K'))
//        {
//            return TRUE;
//        }
//        else
//            index++;
//    }
//    return FALSE;
}

BYTE IsConnect()
{
    BYTE index = 0;
//  delay_ms(1000);
  if(rx0_buff_len > 5)
  {
        while (index < rx0_buff_len-3)
        {

            if ((RxUart0Buf[index] == 'C') && (RxUart0Buf[(int)index+1] == 'O') && (RxUart0Buf[(int)index+2] == 'N'))
            {

               if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
               {
                     index = 7;
                     while (index < rx0_buff_len-3)
                     {
                          if (RxUart0Buf[index] == 'F')  //CONNECT FAIL  msg
                           return FALSE;
                           else index++;

                     }
                } 
                #ifndef AlERT_SYSTEM    //silence if alert
                      Buzer_it(1,1, 1);
                #endif

              
                longAnswerExpected = TRUE;
                ServerComOn = TRUE;
                Timer0_Ticks = 12;

                return TRUE;
            }
            else
                index++;
        }

  }
   // else
   rx0_buff_len = 0;
    ErrorType = 1;
    SendDebugMsg("\r\nWait for CONNECT..\r\n\0");
    MODEM_GOT_IP = FALSE;
//     PrintNum((long)rx0_buff_len);  //deeeeebbbbuuuuuggggggg
   return FALSE;
}

 BYTE IsNO_CARRIER()
{
    BYTE index = 0;
    while (index < rx0_buff_len-4)
    {
        if ((RxUart0Buf[index] == 'N') && (RxUart0Buf[(int)index+1] == 'O') && (RxUart0Buf[(int)index+3] == 'C') && (RxUart0Buf[(int)index+3] == 'A'))
            return TRUE;
        else
            index++;
    }
    return FALSE;
}
BYTE Is_IP_REC()
{
    BYTE index = 0;

     index = 0;
 //   PrintNum((long)rx0_buff_len);  //deeeeebbbbuuuuuggggggg
    while (index < rx0_buff_len-1)
    {

     //  _putchar1(RxUart0Buf[index]);  //deeeeebbbbuuuuuggggggg
        if ((RxUart0Buf[index] == '.') && ((RxUart0Buf[(int)index+1] >='0') || (RxUart0Buf[(int)index+1] <='9')))
        {
            return TRUE;
        }
        else
            index++;
    }
    return FALSE;
}

BYTE    IsRegistOK()
{
    BYTE index = 0;
    bUseRoaming = FALSE;
    //find the , (comma \ psik)
    while (index < rx0_buff_len)
    {
        if (RxUart0Buf[index++] == ',')
            break;
    }
    // if , was found - lookon the digit after:
    if (index < rx0_buff_len)
    {
        if ((RxUart0Buf[index] == '1') || (RxUart0Buf[index] == '5'))
        {
            // if its after external reset - save the operator type (manual or automatic)
            if (RxUart0Buf[index] == '5')
                bUseRoaming = TRUE;
            return TRUE;
        }
        else
        {
              if (RxUart0Buf[index] == '0') //no connection with sim
             {
                   SendDebugMsg("\r\nOOPS, No SIM card or Antenna..?\r\n\0");
                 //  nMaxFailuresNum = 2 ;    //out immediatly

             }
        }
    }

    return FALSE;
}

BYTE Is_IP_OK()
{
    BYTE index = 0;

    //find the , (comma \ psik)
    while (index < rx0_buff_len)

    {
        if (RxUart0Buf[index++] == ',')
            break;
    }

    // if , was found - lookon the digitt after: and before
    if (index < rx0_buff_len)
        if ((RxUart0Buf[index] == '1') && (RxUart0Buf[index - 2] == '1'))
        {
            return TRUE;
        }

       return FALSE;
}


void GetICCID()
{
    int n, i = 0;
    if (!IsOK())
        return;
//   SendDebugMsg("ICCID valid..!\r\n\0");
    //find begining of SIM num
    while ((RxUart0Buf[i] < '0') || (RxUart0Buf[i] > '9'))
        i++;
     n = 0;
    do
    {
        ICCID[n++] = RxUart0Buf[i++];
    }
    while ((RxUart0Buf[i] >= '0') && (RxUart0Buf[i] <= '9') && (n < 20));

//    for (; n < 20; n++)
//        ICCID[n] = '*'; //fill buf until 20 chars


 //   SendDebugMsg("ICCID saved..!\r\n\0");
    for( i = 0; i < 5; i++)
    eICCID[i] = ICCID[n-5 + i];  //write to eeprom last 5 digits


}

//check numbers in between dot are leagal, i.e. not large than 256
BYTE IsLegalNum(char* ip, BYTE indexFirst, BYTE indexLast)
{
    BYTE numDigits, index;
    int num, multy;

    numDigits = indexLast - indexFirst + 1;
    if (numDigits > 3)
        return FALSE;
    if (numDigits == 3)
    {
        multy = 1;
        num = 0;
        index = indexLast;
        while (numDigits > 0)
        {
            num += (ip[index--] - 0x30) * multy;
            multy *= 10;
            numDigits--;
        }
        if (num > 255)
            return FALSE;
    }
    return TRUE;
}

//get new ip and check if its leagal, i.e: 4 numbers from 0 to 255 seperate by '.' (dots).
BYTE IsLegalIP(char* ip)
{
    BYTE dotCntr = 0, n = 0, lastDotIndex = -1;
    while ((ip[n] != '#') && (n < 32))
    {
        if ((ip[n] < '0') || (ip[n] > '9'))
        {
            if (ip[n] != '.')  //if ip is not a number or dot
                return FALSE;
            else
            {
                dotCntr++;               // count dots
                if (n == 0)             // if first IP char is dot
                    return FALSE;
                if ((dotCntr > 1) && (lastDotIndex + 1 == n))  //if its not first dot and the last dot is one index before-meanning 2 dots in a row
                    return FALSE;
                if (!IsLegalNum(ip, lastDotIndex + 1, n - 1))   //check if number in between dots is leagal
                    return FALSE;
                lastDotIndex = n;
            }
        }
        n++;
    }
    if (lastDotIndex + 1 == n) //if last IP char is dot
        return FALSE;
    if (dotCntr != 3)       // ip address can contain exactly 3 dots
        return FALSE;
    if (!IsLegalNum(ip, lastDotIndex + 1, n - 1))     //check if last number is leagal
        return FALSE;
    return TRUE;
}




// is task send as bytes
int ParseSnsrMinMax(char* sID, BYTE lmt)
{
    int i = 0, j = 0;
    long l = 0, lID = 0;
    char s[4];
    int min,max;
    BYTE  nSenNum, res = 0;;

    nSenNum = sID[0];
    if (nSenNum > MAX_SEN_NUM)
        return -1;
    #ifdef DebugMode
    #ifndef MEGA324
    SendDebugMsg("\r\nNum sensor to set limits: ");
    #endif

    PrintNum(nSenNum);
    #endif DebugMode
    for (j = 0; j < nSenNum; j++)
    {
        lID = Bytes2Long(&sID[j*8 + 1]);

        #ifdef DebugMode
        #ifndef MEGA324
        SendDebugMsg("\r\nSensor to change limits: ");
        #endif

        PrintNum(lID);
        #endif DebugMode
        for (i = SENSOR1; i < NumSensors; i++)
        {
            cpu_e2_to_MemCopy(s, &unique_id[i*4], 4);
            l = Bytes2Long(s);
//            #ifdef DebugMode
//            SendDebugMsg("\r\nSensor  ");
//            PrintNum(l);
//            #endif DebugMode
            if (l == lID)
            {
                #ifdef DebugMode
                SendDebugMsg("\r\nfound sensor: ");
                PrintNum(i);
                #endif DebugMode
                min = bytes2int(&sID[j*8 + 5]);  //
                max = bytes2int(&sID[j*8 + 7]);
                #ifdef DebugMode
                SendDebugMsg("\r\nMin value: ");
                PrintNum(min);
                SendDebugMsg("\r\nMax value: ");
                PrintNum(max);
                #endif DebugMode
                // if function was called to update limits
                if (lmt == 0)
                {
                    if (min < max)
                    {
                        MIN_LIMIT[i] = min;
                        MAX_LIMIT[i] = max;
                        res++;
                    }
                }
                else  //if function was called to update back to routine
                {
                    MIN_LIMIT[i] = min;
                    MAX_LIMIT[i] = max;
                    res++;
                }
            }
        }
    }
    if (res != nSenNum)   // if num of sensor was defined is different from num of sensor was sent - something wrong
        return -1;
    return 1;
}


void ParseModemResponse()
{
char i;
//   char str[30];

    switch (modemCurSubTask)
    {
        case SUB_TASK_INIT_MODEM_OK:
        case SUB_TASK_INIT_MODEM_COPS:
        case SUB_TASK_INIT_MODEM_GET_COPS:
        case SUB_TASK_INIT_MODEM_MONITOR:
        case SUB_TASK_INIT_MODEM_RSSI:
        case SUB_TASK_MODEM_CONNECT_ATCH:
        case SUB_TASK_MODEM_CONNECT_SETUP1:
        case SUB_TASK_MODEM_CONNECT_SETUP2:
        case SUB_TASK_MODEM_CONNECT_SETUP3:
        case SUB_TASK_MODEM_CONNECT_PDP_DEF:
//        case SUB_TASK_MODEM_CONNECT_IS_ACTV:
        //  case SUB_TASK_MODEM_CONNECT_ACTV:

       case SUB_TASK_MODEM_CLOSE_PPP:      //separated
//        case SUB_TASK_MODEM_CLOSE_TCP:
//        case SUB_TASK_MODEM_CLOSE_MDM:

          if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
         delay_ms(500);

            if (IsOK() == TRUE)
            {
                if( TestFlag == TRUE)
                 SendDebugMsg("-OK-\r\n\0");
                ModemResponse = TASK_COMPLETE;

                if (modemCurSubTask == SUB_TASK_INIT_MODEM_RSSI)
                {
                    ConvertRssiVal();              
                   if((rssi_val < 7) || (rssi_val == 99))              
                    {
                         ModemResponse = TASK_FAILED;
                       //  ModemRepeatNeeded = TRUE;         //for new trial of communication
//                         sprintf(str, "Bit Error Val = %02d\r\n\0", bitErr_val);  //debug only
//                         UART_WriteMsg(str);
                    }
                } 
                else if (modemCurSubTask == SUB_TASK_INIT_MODEM_GET_COPS)
                {
                      i = Get_COPS(); 
                      if(i)
                      ModemResponse = TASK_COMPLETE;
                      else
                      ModemResponse = TASK_FAILED; 
                } 
            }
            else
            {
                ModemResponse = TASK_FAILED;
                   
            }
        break;

            case SUB_TASK_MODEM_CONNECT_ACTV:
                if( Is_IP_REC() == TRUE)
                ModemResponse = TASK_COMPLETE;
                else
                 ModemResponse = TASK_FAILED;  //????????????????

          break;


        case SUB_TASK_MODEM_CLOSE_TCP:
        case SUB_TASK_MODEM_CLOSE_MDM:

            if (IsOK() == TRUE)
            {
                ModemResponse = TASK_COMPLETE;
            }
            else
                 ModemResponse = TASK_COMPLETE;     //test 9
             //   ModemResponse = TASK_FAILED;
        break;

        case SUB_TASK_INIT_MODEM_REG:
            if (IsRegistOK() == TRUE)
                ModemResponse = TASK_COMPLETE;
             else
            {
                ModemResponse = TASK_FAILED;
             //   ModemRepeatNeeded = TRUE;         //for new trial of communication
             //    ModemRepeatCount++;
            }
        break;

         case SUB_TASK_INIT_MODEM_REG_GPRS:
            if (IsRegistOK() == TRUE)
                ModemResponse = TASK_COMPLETE;
             else
            {
                ModemResponse = TASK_FAILED;
             //   ModemRepeatNeeded = TRUE;         //for new trial of communication
             //    ModemRepeatCount++;
            }
        break;

        case SUB_TASK_MODEM_CHK_ICCID:
            GetICCID();
            //ConvertIccID();
            ModemResponse = TASK_COMPLETE;
        break;

        case SUB_TASK_MODEM_CONNECT_START_DIAL:

               i = 0;
               ConnectedToServer = FALSE;
               delay_ms(500);

               do{
                

                      i++;
                     if (IsConnect() == TRUE)
                     {
                          ModemResponse = TASK_COMPLETE;
                          ConnectedToServer = TRUE;
                        
                     }
                     else
                     {
                            delay_ms(1000);
                            #asm("wdr")
                     }
                }
               while((ConnectedToServer == FALSE) && (i < 15));

               if(ConnectedToServer == FALSE)
               {
                     ModemResponse = TASK_FAILED;
                   //  ModemRepeatNeeded = TRUE;         //for new trial of communication
                 //    ModemRepeatCount++;
                      Timer0_Ticks = 12;              //restore regular value
                      ServerResponseTimeOut = 70;
               }
        break;

        case SUB_TASK_MODEM_POST_DATA: 
        case SUB_TASK_MODEM_SEND_NOTIFICATION:
        
           //     bCheckRxBuf = FALSE ;
               i = CheckResult();
              if (i == -2)   //server response is too short - SIM800? ignor it..
              {

                //  ModemResponse = NO_ANSWER;
                   #asm("wdr")
                  delay_ms(2000);
                  i = CheckResult();
              }

               if ((i > -1) || (Found200 == TRUE))
              {
                     ModemResponse = TASK_COMPLETE;
              }
              else
             {                                         
                         ModemResponse = TASK_FAILED;
                         SendDebugMsg("Waiting to proper server response failed,,\r\n\0 ");
             }


        break;
        }
    }


// check  if "phy111" is in buffer:
int CheckResult()
{
   // int i = 0, n;
    char *ptr;
    int index;
    BYTE ok; 
  //  static char comCount = 0;
 
   SendDebugMsg("\r\nCheckResult()..\r\n\0 "); 
    Timer0_Ticks = 12;  
   delay_ms(1000);   
   
   if(( rx0_buff_len < 10) && (Found200 == FALSE))
   {
      SendDebugMsg("\r\n** CheckResult() - Too short response\r\n\0 ");
          return -2;    //short  to be ok
   }


    if (Found200== FALSE)
    {
        SendDebugMsg("\r\nNo \"200\" found by RX int..\r\n\0 ");
    }  
    else if( new_flash_version == TRUE)
    {
             new_flash_version = FALSE;
             ptr = strstrf( RxUart0Buf, "ok"); //server response for alert post
             if(ptr != NULL)
             { 
                 Found200 = TRUE;
              } 
             
    }
    Timer0_Ticks = 12;
  
//     delay_ms(1000);

    index = -1;   
    if (Found_200 == FALSE)
    {    
          SendDebugMsg("\r\nNo Jason found !\r\n\0 ");
         ptr = strstrf( RxUart0Buf, "200{"); //expect APPROVED101
         if(ptr != NULL)
         { 
              Found200 = TRUE;  
               SendDebugMsg("Jason found..\r\n\0 "); 
               index = 1;
         }
    }
    else index = 1;  //

    // if hasnt find - return -1, means no data got back  
    
//      comCount++;      //test data ponters saving
//      if(comCount < 2) 
//      {
//         index = -1;   
//        Found200 = FALSE;
//      }
//      PrintNum((long)comCount);
    
    if ((index == -1) && (Found200 == FALSE))   //no 200 from server
    {

           SendDebugMsg("\r\n**Failure - No OK from Server**\r\n\0 ");   //Danny for debug
           ComFailureCounter++;
           ErrorType = 2;
         if (ComFailureCounter == 2)    //try twice and than delay it
         {
             if(bExtReset == FALSE)  //not if reset pressed by user
             ComDelayNeeded = TRUE;
             bConnectOK = FALSE ;     //Danny -old var
          //   ComFailureCounter = 0;
              CurrentHourOfFailure = readClockBuf[4];   //keep hour for next try
              SendDebugMsg("\r\n**Two succesive Modem failures-COM delayed ..**\r\n\0 ");   //Danny for debug
         }

            return -1;

    }
    else    //200 found - sccess
    {

         ClockUpdated = FALSE;
         ok =  RTC_Update_by_Server(RxUart0Buf, 0);
         if(ok)                     //found clock string
          ClockUpdated = TRUE;
          if( ok == 1)                //set by RTC_Update_by_Server() -  more update parms available
          {
              ok = Pharse_params_struct(RxUart0Buf, 20);
              if(ok)
              {

                  SendDebugMsg("\r\n**New Params Saved**\r\n\0 ");   //Danny for debug
                 #ifdef KANDO_SERVER
                  SendPostInfo(1);        //ack server successfull update
                 #endif
                 delay_ms(3000);
                 ServerResponseTimeOut = 70;          //shorter than WD
                 TimeLeftForWaiting = 70;
              }

           }

              CloseSuccessfulCom();
//                bCheckRxBuf = FALSE;    //moved here 041516 from   GetNextModemTask() line
               return 1;

    }
  //----------------------------------------------------------------------------------
       // sign that post answered - even if failed - its an answer. no need to open different socket. connection is OK.

    // if its parameters post - save the amswer
//    if (modemCurSubTask == SUB_TASK_MODEM_POST_PRM)
//    {
//        for (n = 0; n < 5; n++)
//        tmpClock[n] = RxUart0Buf[GetBufferIndex(i++)];
//
//        MemCopy( clockBuf, &tmpClock[0], 3 ); //copy year month day
//        MemCopy( &clockBuf[4], &tmpClock[3], 2 );  //copy hour minute
//        rtc_set_timeAll( clockBuf);               //Danny- new punc
//
////        if(SetRealTime() == FAILURE)           //Danny - old
////        {
////            //
////            #ifdef DebugMode
////            SendDebugMsg("\r\nSetRealTimeFailed\0");//set real time
////            #endif DebugMode
////        }
//        //i += 5;
//        for ( n = 0; n < MAX_PRM_TASKS; n++)
//            UpdatePrmArr[n] = RxUart0Buf[GetBufferIndex(i++)];
//
//        if ((UpdatePrmArr[1] == '1') && (UpdatePrmArr[2] == '1'))
//            bUpdateAddress = TRUE;
//        else
//            bUpdateAddress = FALSE;
//        bMakeReset = FALSE; // init flag of make reset
//    }

//    if (modemCurSubTask == SUB_TASK_MODEM_POST_UPD)     //Danny
//    {
//        bufIndexToUpd = index;
//        if (UpdateParam() != TRUE)
//            index = -1;
//    }
//=======================Clock update by server==needed?=======================Danny
    // check if there is ok after phy111
//    if (modemCurSubTask == SUB_TASK_MODEM_POST_DATA)
//    {
//                    i = index;
//            //        if (!((RxUart0Buf[GetBufferIndex(i)] == 0x4f) && (RxUart0Buf[GetBufferIndex(i+1)] == 0x4b)))         // OK
//            //        #ifdef EXT_SERVICE
//                    // first - save clock
//
//                    for (n = 0; n < 5; n++)
//                        tmpClock[n] = RxUart0Buf[GetBufferIndex(i++)];
//                    MemCopy( clockBuf, &tmpClock[0], 3 );       //update year month day
//                    MemCopy( &clockBuf[4], &tmpClock[3], 2 );   //update hour minute
//                    if ((toDoList == DO_DATA) && (objToMsr == SENSOR1)) // if its first sensor and no params this time:
//                     rtc_set_timeAll( clockBuf);                       //Danny - new
//===========================Clock update by server=========================
//
//        // check if there is PENDING
//        if ((RxUart0Buf[GetBufferIndex(i)] == 0x50) &&      //P
//        (RxUart0Buf[GetBufferIndex(i+1)] == 0x45) &&        //E
//        (RxUart0Buf[GetBufferIndex(i+2)] == 0x4e) &&        //N
//        (RxUart0Buf[GetBufferIndex(i+3)] == 0x44) &&        //D
//        (RxUart0Buf[GetBufferIndex(i+4)] == 0x49) &&        //I
//        (RxUart0Buf[GetBufferIndex(i+5)] == 0x4E) &&        //N
//        (RxUart0Buf[GetBufferIndex(i+6)] == 0x47) )         //G
//        {
//            toDoList = DO_DATA_N_PRMS;
//            i += 7;
//        }
//        #endif EXT_SERVICE
//        // now check for OK
//        if (!((RxUart0Buf[GetBufferIndex(i)] == 0x4f) && (RxUart0Buf[GetBufferIndex(i+1)] == 0x4b)))         // OK
//            return -1;
//    }
//       bCheckRxBuf = FALSE;    //moved here 041516 from   GetNextModemTask() line
    return index;
}

/*
// if eMobileCntryCode & eMobileNetCode are integers:
BYTE IntToStr(BYTE * to, int tmp)
{
    BYTE m[3];
    BYTE j, k, i = 0;

    // seperate each numbers into digits
    while (tmp > 0)
    {
        k = tmp % 10;
        m[i++] = k + 48;
        tmp = tmp / 10;
    }
    if (i == 1)
        m[i++] = 48;
    j = 0;
    //reverse the string
    do
    {
        i--;
        to[j++] = m[i];
    }
    while (i > 0);
    return j;
}
*/
//void SendOperatorSelection()
//{
////    if (eNotInUse == 0)
////        eNotInUse = 1;
//    if (bExtReset == TRUE) //|| (eUseCntrCode == 1) || ((eUseCntrCode == 0) && (eOperatorType != '5')))
//        SendATCmd(AT_COPS_AUTO);
//    else
////      if (eOperatorType == '5')
//        if (eUseCntrCode == 1)
//        {
//            BYTE i, n;
//            //build the cops command  from country+network codes
//            n = CopyFlashToBuf(ComBuf, AT_COPS_MAN);
//            ComBuf[n++] = '"';
//            i = 0;
//            while (eMobileCntryCode[i] != '#')
//            {
//                ComBuf[(int)i+n] = eMobileCntryCode[i];
//                i++;
//            }
//            n += i;
//            i = 0;
//            while (eMobileNetCode[i] != '#')
//            {
//                ComBuf[(int)i+n] = eMobileNetCode[i];
//                i++;
//            }
//            // if eMobileCntryCode & eMobileNetCode are integers
////            i = IntToStr(&ComBuf[n], eMobileCntryCode);
////            n += i;
////            i = IntToStr(&ComBuf[n], eMobileNetCode);
//            n += i;
//            ComBuf[n++] = '"';
//            ComBuf[n++] = '\r';
//            ComBuf[n++] = '\n';
//            BytesToSend = n;
//            TransmitBuf(0);
//        }
//}

void SendStartDial()
{
 BYTE i, n;
    //build the cops command  from country+network codes
      if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
      n = CopyFlashToBuf(ComBuf, AT_TCP_OPN);
      else     if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
      n = CopyFlashToBuf(ComBuf, CIPSTART); 
      
 if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
 {
 //   n++;
    ComBuf[n++] = '"';
    i = 0;
    while ((eIPorURLval1[i] != '#') && (i < 32))
    {
        ComBuf[(int)i+n] = eIPorURLval1[i];
        i++;
    }
    n += i;
    ComBuf[n++] = '"';
     ComBuf[n++] = ',';
     ComBuf[n++] = '"';  
     ComBuf[n++] = '0';  
     ComBuf[n++] = '0'; 
     ComBuf[n++] = '8'; 
     ComBuf[n++] = '0'; 
  //   cpu_e2_to_MemCopy(&ComBuf[n], ePORTval1, 4);
  //   n += 4;
      ComBuf[n++] = '"';
 }
 else  if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
 {
//    cpu_e2_to_MemCopy(&ComBuf[n], ePORTval1, 4);
//     n += 4; 
      ComBuf[n++] = '0';  
     ComBuf[n++] = '0'; 
     ComBuf[n++] = '8'; 
     ComBuf[n++] = '0'; 
    ComBuf[n++] = ',';
    ComBuf[n++] = '"';
    i = 0;
    while ((eIPorURLval1[i] != '#') && (i < 32))
    {
        ComBuf[(int)i+n] = eIPorURLval1[i];
        i++;
    }
    n += i;
    ComBuf[n++] = '"';
  }

    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);

   // ShowHexString(ComBuf);     //Danny- send to monitor
}


//1,0,0080,"crea-cell.com"


//void SendStartDial_test()
//{
//    BYTE i;
//   char str[] = "AT#SD=1,0,8180,\"54.149.4.152\"\r\n\0 ";   //32
//
//   for(i = 0; i < 31; i++)
//   {
//      ComBuf[15] = str[i];
//   }
////   ComBuf[15] = = "\"";
////    ComBuf[28] = ="\"";
////    ComBuf[29] = = '\r';
////    ComBuf[30] = = '\n';
//
//    BytesToSend = 31;
//    TransmitBuf(0);
//   // ShowHexString(ComBuf);     //Danny- send to monitor
//}

void SendPDPCntDef()
{
 BYTE i, n;
    //build the cops command  from country+network codes
     if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
    n = CopyFlashToBuf(ComBuf, DEF_PDP_CNTXT);
    else  if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
     n = CopyFlashToBuf(ComBuf, CSTT);
    ComBuf[n++] = '"';
    i = 0;
    while ((eAPN[i] != '#') && (i < 32))
    {
        ComBuf[(int)i+n] = eAPN[i];
        i++;
    }
    n += i;
    ComBuf[n++] = '"';
    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
   // ShowHexString(ComBuf);     //Danny- send to monitor
}

BYTE GetNextModemTask()
{
     char ok;
    // first task-
    if (modemCurTask == TASK_NONE)
    {
             if( MODEM_GOT_IP == TRUE)
             {
                      modemCurTask = TASK_MODEM_CONNECT;
                      modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;

             }
             else
             {
                     modemCurTask = TASK_MODEM_INIT;
                     modemCurSubTask = SUB_TASK_INIT_MODEM_ON; 
                     ModemRepeatNeeded = TRUE; 
                     FirmwareUpdateTime = FALSE ; 
                     TestPost = 0;
             }

                initCnt = 0;

        return  CONTINUE;
    }



    // if flag of end of rx received is on
     if (bCheckRxBuf == TRUE)                 //new msg in RX buf
    {
//        bCheckRxBuf = FALSE;       //no
          if(ConnectedToServer == TRUE)      //ver 54
          {

              if((  modemCurSubTask == SUB_TASK_MODEM_POST_DATA) ||  ( modemCurSubTask ==  SUB_TASK_MODEM_SEND_NOTIFICATION))
              {     
                 //   SendDebugMsg("checking post data response...\r\n\0 ");
                   if( rx0_buff_len < 6)
                   {
                     bCheckRxBuf = FALSE;
                     ENABLE_TIMER0();
                     ModemResponse = NO_ANSWER;
                      SendDebugMsg("\r\nShort response-Ignored...\r\n\0 ");
                   }
                   else   ParseModemResponse();
              }


//                        if( TestFlag == TRUE)
//                        {
//                         SendDebugMsg("-7-\r\n\0");
//                         TestFlag = FALSE;
//                         }

          }
         else //   delay_ms(100);
         ParseModemResponse();
    }

     //---------------------------------------------------------
     if (Found200 == TRUE)  //if(ConnectedToServer == TRUE)
      {
              if(  modemCurSubTask == SUB_TASK_MODEM_POST_DATA)//    200220?
              {         
                    //  SendDebugMsg("to ParseModemResponse ..\r\n\0"); 
                       ParseModemResponse();  
                       
              }
      }
   //-----------------------------------------------------------------


        //server timeout    new-270216 eliminate WD when waiting for server
    if (( ConnectedToServer == TRUE ) && (ServerResponseTimeOut == 0))
    {
        ModemResponse = TASK_FAILED;
        ErrorType = 3;
        #asm ("wdr");
        SendDebugMsg("\r\n Server respond time out..\r\n\0 ");
      //  ModemRepeatNeeded = TRUE;

    }

     if ((bWaitForModemAnswer == TRUE) && (TimeLeftForWaiting == 0))       //important check
    {
        bWaitForModemAnswer = FALSE;
        ModemResponse = TASK_FAILED;
     //   ModemRepeatNeeded = TRUE;

        SendDebugMsg("\r\n Modem answer - Wait time out..\r\n\0 ");
    }

    if (bCheckBatr == 2)   //AFTER BATTERY CHECK
    {
        bCheckBatr = 3;
        ModemResponse = TASK_COMPLETE;
        objToMsr = SENSOR1;

    }
    if (heat_time > 0)
        return WAIT;

    switch (ModemResponse)
    {
        case NO_ANSWER:
       //     SendDebugMsg("NO ANSWER..\r\n\0 ");
            return WAIT;


        case TASK_COMPLETE:
        {
            switch (modemCurTask)
            {
                case TASK_MODEM_INIT:
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_INIT_MODEM_ON:
                        // 4.4.13: add next 6 lines:
                        //check if iggnition time ends:
//                            if (heat_time > 0)          //remove on 17.9.13. ask  if (heat_time > 0)  before switch
//                                return WAIT;
                            modemCurSubTask = SUB_TASK_INIT_MODEM_IGN;
                        break;

                        case SUB_TASK_INIT_MODEM_IGN:
                            modemCurSubTask = SUB_TASK_INIT_MODEM_OK;
                        break;

                        case SUB_TASK_INIT_MODEM_OK:
                            if ((bExtReset == FALSE) && (eUseCntrCode == 0))
                            {
                                modemCurSubTask = SUB_TASK_INIT_MODEM_DELAY;
                                heat_time = 50; //was 100 Danny - delay before creg
                            }
                             else
                                modemCurSubTask = SUB_TASK_INIT_MODEM_COPS;    //Danny- no need
                               
                        break;

                        case  SUB_TASK_INIT_MODEM_COPS:
                           modemCurSubTask = SUB_TASK_INIT_MODEM_DELAY;

                           heat_time = 60; //delay before rssi
                        break;



                        case SUB_TASK_INIT_MODEM_DELAY:
                              //  modemCurSubTask = SUB_TASK_INIT_MODEM_REG;
                                 modemCurSubTask = SUB_TASK_INIT_MODEM_RSSI;

                                 MODEM_TYPE = eModemType;
                             //     MODEM_TYPE = SIM800;
                                 if(MODEM_TYPE  > 3) //not yet defined
                                 {
                                    ok = ModemTypeDetect();
                                    if(ok)
                                     SendDebugMsg("\r\n Modem Defined..\r\n\0 ");
                                 }
//                                 }
                                  #ifdef Evogen_Com_1Min   
                                //     SendDebugMsg("\r\nReset Modem..\r\n\0");
                                //     ModemReset();   //if new modem start, reset first
                                   #endif                     
                                    SendATCmd(AT_E0); //new

                              heat_time = 20; //delay before creg
                        break;

                        case SUB_TASK_INIT_MODEM_REG:
                            if (bUseRoaming)
                           //    delay_ms(eRoamingDelay-5 * 1000);

                                delay_ms(2000);
                            else
                                delay_ms(700);  //5000 Danny
                               modemCurSubTask = SUB_TASK_INIT_MODEM_GET_COPS;  // no need danny
                         //   modemCurSubTask = SUB_TASK_INIT_MODEM_REG_GPRS; 
                            
                        break;

                        case SUB_TASK_INIT_MODEM_GET_COPS:

                       //     modemCurSubTask = SUB_TASK_INIT_MODEM_MONITOR;  //sipmode=1
                             modemCurSubTask = SUB_TASK_MODEM_CONNECT_ATCH ;
                        break;



                         case SUB_TASK_MODEM_CONNECT_ATCH:
//
                                  modemCurSubTask = SUB_TASK_INIT_MODEM_REG_GPRS ;
                        break;

                         case SUB_TASK_INIT_MODEM_REG_GPRS:
                            if (bUseRoaming)
                                delay_ms(1000);
                            else
                                delay_ms(700);  //5000 Danny
                             modemCurSubTask = SUB_TASK_INIT_MODEM_MONITOR;    //cipmode=1
                        break;

                       case SUB_TASK_INIT_MODEM_MONITOR:
                       //    modemCurSubTask = SUB_TASK_INIT_MODEM_RSSI;
                            modemCurTask = TASK_MODEM_CONNECT;
                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_PDP_DEF;

                        break;

                        case SUB_TASK_INIT_MODEM_RSSI: //todo: check rssi and continue only if it over min

                                 modemCurSubTask = SUB_TASK_INIT_MODEM_REG;
                           //     modemCurSubTask = SUB_TASK_MODEM_CONNECT_PDP_DEF;
                                 //heat_time = 150; //delay before attach

//                 //-----------------------------if no server available-Remark when released--------------------------------
//                                     modemCurTask = TASK_MODEM_POST;            Danny-test no modem connected to sever
//                                     modemCurSubTask = SUB_TASK_MODEM_POST_DATA;
//                                     #ifdef DebugMode
//                                      SendDebugMsg("\r\n To POST-no modem\r\n\0 ");
//                                      #endif DebugMode
//                  //-------------------------------------------------------------------------
//
                        break;
                        case SUB_TASK_MODEM_CHK_ICCID:
                            modemCurTask = TASK_MODEM_CONNECT;
                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_DELAY;//SUB_TASK_MODEM_CONNECT_ATCH;
                            heat_time = 30; //delay before attach
                        break;
                    }
                break;

                case TASK_MODEM_CONNECT:
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_MODEM_CONNECT_DELAY:       //??????????????????????????????????????????
                           modemCurSubTask = SUB_TASK_MODEM_CONNECT_SETUP1 ;//SUB_TASK_MODEM_CONNECT_ATCH;
                            break;


                        case SUB_TASK_MODEM_CONNECT_SETUP1:

                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_SETUP2;
                        break;

                        case SUB_TASK_MODEM_CONNECT_SETUP2:
                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_SETUP3;
                        break;

                        case SUB_TASK_MODEM_CONNECT_SETUP3:
                            if ( MAGNET_SW_ON == TRUE)
                            {
                               //    MAGNET_SW_ON = FALSE;
                                   modemCurSubTask = SUB_TASK_MODEM_CHK_ICCID;
                             }
                            else
                                   modemCurSubTask = SUB_TASK_MODEM_CONNECT_ACTV;
                        break;

                        case SUB_TASK_MODEM_CHK_ICCID:
                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_ACTV;//SUB_TASK_MODEM_CONNECT_PDP_DEF;
                        break;

                        case SUB_TASK_MODEM_CONNECT_PDP_DEF:
                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_SETUP1;// SUB_TASK_MODEM_CONNECT_ACTV;   //   SUB_TASK_MODEM_CONNECT_IS_ACTV
                        break;

//                        case SUB_TASK_MODEM_CONNECT_IS_ACTV:
//                            if (GetContextStatus() == '1')
//                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;
//                            else
//                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_ACTV;
//                        break;

                        case SUB_TASK_MODEM_CONNECT_ACTV:

                             modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;

                        break;

                        case SUB_TASK_MODEM_CONNECT_START_DIAL:  
                        
                           if(new_flash_version == TRUE)  //when new version first alive-send notif
                           {
                               //   new_flash_version = FALSE;
                                  if(SuccessUpdate)
                                      notification_index = NEW_VER_OK;
                                  else                                  
                                      notification_index = GENERAL_ERR; 
                                      
                                                                                                                
                                   modemCurSubTask = SUB_TASK_MODEM_SEND_NOTIFICATION;
                           }
                           else if (bCheckBatr == 0)
                           {                                                             
                                    bCheckBatr = 1;  //stop for battery test when modem is  on
                                    return WAIT;     //evaluate bCheckBatr at main                                    
                           }
                           else    //continue with data transmission
                           {                              
                                   #ifdef Evogen_Com_1Min  
                                      MODEM_GOT_IP = TRUE ;   //flag IP exist
                                   #endif  
                                  
                                //    modemCurSubTask = waitingTask;    //set to SUB_TASK_MODEM_POST_DATA at InitVarsForConnecting()
                                     modemCurSubTask = SUB_TASK_MODEM_POST_DATA; 
                           }   
                           
                            modemCurTask = TASK_MODEM_POST;
                        break;
                    }
                break;
                case TASK_MODEM_POST:

                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_MODEM_POST_DATA:

                           //----------------update FW-----------------------
                              if(FirmwareUpdateTime == TRUE)  //firmware update needed
                              {
                                    notification_index = 0;     //ask for file
                                    modemCurTask = TASK_MODEM_POST;
                                    modemCurSubTask = SUB_TASK_MODEM_FLASH_UPDATE;    //new task 131217
                                    SendDebugMsg("FirmwareUpdateTime = TRUE ..\r\n\0"); 
                                    TestPost = 1;
                              }
                              //----------------------------------------------- 
                              
                              else
                              {
                                     Timer0_Ticks = 12;
                                      dataSentOK = TRUE;
                                      modemCurTask = TASK_MODEM_CLOSE;
                                      modemCurSubTask = SUB_TASK_MODEM_CLOSE_PPP; 
                                     
                               }
                        break;

                       case SUB_TASK_MODEM_FLASH_UPDATE:

                                    modemCurTask = TASK_MODEM_POST;
                                    modemCurSubTask = SUB_TASK_MODEM_SEND_NOTIFICATION;
                                   SendDebugMsg("FLASH_UPDATE ok- Send notification ..\r\n\0");

                                    delay_ms(1000);   //???????????
                        break;

                        case SUB_TASK_MODEM_SEND_NOTIFICATION:

                               SendDebugMsg("Complete NOTIF..\0");
//                               PrintNum((long)notification_index);

                               switch(notification_index)
                               {
                                   case 0:    //got here from post data - ask for update file
                                          modemCurTask = TASK_MODEM_POST;
                                          modemCurSubTask = SUB_TASK_MODEM_FLASH_UPDATE;    //go get file   
                                          TestPost = 2;
                                   break;

                                   case 1:

                                   break;

                                    case 3:    //after success notif
                                    case 6:
                                    case 4:
                                    case 5:
                                             delay_ms(500);
                                             modemCurTask = TASK_MODEM_CLOSE;
                                             modemCurSubTask = SUB_TASK_MODEM_CLOSE_PPP;
                                             dataSentOK = TRUE;
                                   break;

                                    default:
                                              modemCurTask = TASK_MODEM_CLOSE;
                                              modemCurSubTask = SUB_TASK_MODEM_CLOSE_PPP;
                                  
                               }
                        break;


                        default:
                    }
                break;

                case TASK_MODEM_CLOSE:
                    switch (modemCurSubTask)
                    {
                        case  SUB_TASK_MODEM_CLOSE_PPP:
                        //   delay_ms(500);                  //test 9
                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_TCP;

                        break;

                        case SUB_TASK_MODEM_CLOSE_TCP:

                             if( MODEM_GOT_IP == TRUE)
                             {
                                 SendDebugMsg(" Modem keep IP..\r\n\0");
                                 modemCurSubTask = SUB_TASK_MODEM_OFF;    //dont shut modem. keep IP
                             }
                             else
                             {
                              //  SendDebugMsg(" MODEM_GOT_IP = FALSE ..\r\n\0");
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;   // shut modem. no IP
                              }
                        break;
                          

                        case SUB_TASK_MODEM_CLOSE_MDM:
                            modemCurSubTask = SUB_TASK_MODEM_OFF;
                        break;

                        case SUB_TASK_MODEM_OFF:
                            modemCurTask = TASK_NONE;
                            modemCurSubTask = TASK_NONE;
                        break;
                    }
                break;
            }
            failCnt = 0;
            return CONTINUE;
        }
 //------------------------------Failed section------------------------------------
        case TASK_FAILED:
        {

        //  SendDebugMsg("TASK failure..\r\n\0");
            failCnt++;    // count num of failures

            // if failed more than 2 times - quit
            if (failCnt >= nMaxFailuresNum)
            {
                switch (modemCurTask)
                {
                    case TASK_MODEM_INIT:

                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_INIT_MODEM_OK:
                                // if try only once to jig the iggnition pulse - try again, else- switch off.

                                if(initCnt < 2)
                                 ModemIgnit();     //maybe HE910 - need ignition pulse
                                if (initCnt < 3)
                                    modemCurSubTask = SUB_TASK_INIT_MODEM_ON;
                                else
                                {
                                   
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_MODEM_OFF;
                                    ErrorType = 6;
                                }

                            break;

                            case  SUB_TASK_INIT_MODEM_COPS:
                                modemCurSubTask = SUB_TASK_INIT_MODEM_REG;
                            break;

                            case SUB_TASK_INIT_MODEM_REG:
                              
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_OFF;
                                 ModemAgain = TRUE;
                                ErrorType = 5;


                          break;

                           case SUB_TASK_MODEM_CONNECT_ATCH:

                                      SendDebugMsg("Attach GPRS failure .\r\n\0");
                                       modemCurTask = TASK_MODEM_CLOSE;              //changed 010716
                                       modemCurSubTask =  SUB_TASK_MODEM_OFF; // SUB_TASK_MODEM_CLOSE_MDM ; //test 270616
                                       nFailureCntr++;
                                  //     ModemRepeatNeeded = TRUE;
                                      ModemAgain = TRUE;
                                       ErrorType = 9;

                                 //        modemCurSubTask = SUB_TASK_INIT_MODEM_REG_GPRS ;

                            break;


                          case SUB_TASK_INIT_MODEM_REG_GPRS:
                           
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_OFF;

                            //     ModemRepeatNeeded = TRUE;
                                 ModemAgain = TRUE;

                                ErrorType = 7;

                          break;

                            case SUB_TASK_INIT_MODEM_GET_COPS:
                                modemCurSubTask = SUB_TASK_INIT_MODEM_MONITOR;
                             //    ModemRepeatNeeded = TRUE;
                                 ModemAgain = TRUE;
                            break;

                            case SUB_TASK_INIT_MODEM_MONITOR:  
                             //     ModemRepeatNeeded = TRUE;
                                 ModemAgain = TRUE;
                        
                                 modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_OFF;
                            break;

                            case SUB_TASK_INIT_MODEM_RSSI: //todo: check rssi and continue only if it over min

                                SendDebugMsg("RSSI level failure- Abort.\r\n\0");
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_OFF;
                           //     ModemRepeatNeeded = TRUE;
                                ModemAgain = TRUE;
                                ErrorType = 8;

                            break;

                        }
                    break;

                    case TASK_MODEM_CONNECT:
                        switch (modemCurSubTask)
                        {

                            case SUB_TASK_MODEM_CONNECT_SETUP1:   //210517

                                       SendDebugMsg("GPRS failure .Close.\r\n\0");
                                       modemCurTask = TASK_MODEM_CLOSE;              //changed 010716
                                       modemCurSubTask =  SUB_TASK_MODEM_OFF; // SUB_TASK_MODEM_CLOSE_MDM ; //test 270616
                                       nFailureCntr++;
                                       ModemAgain = TRUE;

                            break;

                            case SUB_TASK_MODEM_CONNECT_SETUP2:
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_SETUP3;
                            break;

                            case SUB_TASK_MODEM_CONNECT_SETUP3:

                                modemCurSubTask = SUB_TASK_MODEM_CHK_ICCID;
                            break;

                            case SUB_TASK_MODEM_CHK_ICCID:
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_PDP_DEF;
                            break;

                            case SUB_TASK_MODEM_CONNECT_PDP_DEF:
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_ACTV;
                            break;

                            case SUB_TASK_MODEM_CONNECT_ACTV:
                                modemCurTask = TASK_MODEM_CLOSE;
                                                modemCurSubTask = SUB_TASK_MODEM_OFF;   //SUB_TASK_MODEM_CLOSE_TCP
                                                 ErrorType = 4;
                        
                            break;

                            case SUB_TASK_MODEM_CONNECT_START_DIAL:      //xxx

                                  modemCurTask = TASK_MODEM_CLOSE;

                            //      modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM ; //test 270216
                                  modemCurSubTask = SUB_TASK_MODEM_OFF;
                                  nFailureCntr++;
                                  SendDebugMsg("Server CONNECT failure..\r\n\0");
                                  ModemAgain = TRUE;
                            break;
                        }
                    break;

                    case TASK_MODEM_POST:

                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_MODEM_POST_DATA:
                           //    SendDebugMsg("SUB_TASK_MODEM_POST_DATA -  failure process..\r\n\0");

                                 SendDebugMsg("\r\nFailed to get proper server response.!! \r\n\0");
                                  ResetAllReadPointers();// pointer set in GetMeasurments()  
                                  eRESTORE_DATA_POINTERS = RELEASE_POINTERS; 
                                  
                                 if (bPostAnswered == TRUE)
                                 {
                                    dataSentOK = TRUE;
                                    ComDelayNeeded = FALSE;   //added by Danny 260515
                                 }
                             //    SendDebugMsg("Post data failure..\r\n\0");
                                   bCheckRxBuf = FALSE;

                                   modemCurTask = TASK_MODEM_CLOSE;
//                                   if(ErrorType == 3 )       //modem didnt get respond from server- might be voltage problem
//                                   modemCurSubTask = SUB_TASK_MODEM_CLOSE_PPP;
//                                         // modemCurSubTask = SUB_TASK_MODEM_OFF ;//SUB_TASK_MODEM_CLOSE_MDM;
//                                   else
                                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_PPP;
                                  //      modemCurSubTask = SUB_TASK_MODEM_OFF; //SUB_TASK_MODEM_CLOSE_PPP;
                                     Timer0_Ticks = 12;
                            break;

                              case SUB_TASK_MODEM_FLASH_UPDATE:     //update file failure - new 131217

                                        if(updateErrorCount < 2)
                                        {
                                            //  server_ReDial(1);              //with close
                                           //   got_NO_CARRIER = FALSE;
                                              modemCurSubTask = SUB_TASK_MODEM_FLASH_UPDATE; //try again
                                              SendDebugMsg("update FW Failed - Retry..\r\n\0"); 
                                              TestPost = 4;
                                        }
                                        else
                                        {
                                              updateErrorCount = 0;
                                              modemCurSubTask = SUB_TASK_MODEM_SEND_NOTIFICATION;
                                              SendDebugMsg("Two updates Failures -notify..\r\n\0");
                                        }
                              break;

                               case SUB_TASK_MODEM_SEND_NOTIFICATION:

                                         bCheckRxBuf = FALSE;
                                         ModemRepeatNeeded = TRUE;
                                         modemCurTask = TASK_MODEM_CLOSE;
                                         modemCurSubTask = SUB_TASK_MODEM_CLOSE_PPP;
                                       //   eUUID[8] == '#';   //cut new version detected

                                break;                         
                          }
                                            
                 
                    break;  
                   
                    case TASK_MODEM_CLOSE:
                        switch (modemCurSubTask)
                        {
                            case  SUB_TASK_MODEM_CLOSE_PPP:
                             //   modemCurSubTask = SUB_TASK_MODEM_CLOSE_TCP;     //test jump to MDM
                              //   modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                                  modemCurSubTask = SUB_TASK_MODEM_OFF;
                                //   if( TestFlag == TRUE)
                                 //  SendDebugMsg("-4-\r\n\0");
                            break;

                            case SUB_TASK_MODEM_CLOSE_TCP:
                            //    modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                                  modemCurSubTask = SUB_TASK_MODEM_OFF;
                            break;

                            case SUB_TASK_MODEM_CLOSE_MDM:
                                modemCurSubTask = SUB_TASK_MODEM_OFF;

                            break;

                        }

                    break;
                 }
                failCnt = 0;

            }
            else   // if (failCnt >= nMaxFailuresNum)
            {
                delay_ms(1000);
                // if fail on send data - change the read mod to 3 (send again the buffer)

                if (modemCurSubTask == SUB_TASK_INIT_MODEM_RSSI)
                  delay_ms(3000);
                else if (modemCurSubTask == SUB_TASK_INIT_MODEM_REG)
                    delay_ms(3000);                    //was 4000 Danny

                  else if (modemCurSubTask == SUB_TASK_INIT_MODEM_REG_GPRS)
                    delay_ms(3000);                    //was 4000 Danny

                else if (modemCurSubTask == SUB_TASK_MODEM_CONNECT_START_DIAL)        //  (modemCurSubTask == SUB_TASK_MODEM_CONNECT_ACTV) ||
                {
                    delay_ms(3000);
                    nFailureCntr++;
                }

                else if (modemCurSubTask == SUB_TASK_MODEM_CONNECT_ACTV)
                    delay_ms(1000);

                 else if (modemCurSubTask ==  SUB_TASK_MODEM_POST_DATA)
                 {
                      bCheckRxBuf = FALSE;
                  //     SendDebugMsg("Failed SUB_TASK_MODEM_POST_DATA\r\n\0");
                      return WAIT;
                 }

//                 else if (modemCurSubTask == SUB_TASK_MODEM_CLOSE_PPP)
//                 {
//                     SendDebugMsg("Failed +++\r\n\0");
//                 }
                  else if (modemCurSubTask == SUB_TASK_MODEM_SEND_NOTIFICATION)
                  {
                      bCheckRxBuf = FALSE;
                       SendDebugMsg("Failed notification - Retry..\r\n\0");
                      delay_ms(2000);

                  } 

            }
        }
        return CONTINUE;
        break;
    }
}

void  ModemMain()
{
    BYTE res;
    bit ok = 0;
    unsigned int temp;
    char plusplus[] = "+++"; 
 //   char Str1[40]; 
   #define MaxErrs  8

         res = GetNextModemTask();
         if(res == WAIT)
         {
//            if( TestFlag == TRUE)
//             SendDebugMsg("-3-\0");
            return;
         }


    switch (modemCurTask)
    {
        case TASK_MODEM_INIT:
            switch (modemCurSubTask)
            {
                case SUB_TASK_INIT_MODEM_ON:
                   
                      UpdateSession = FALSE;
                     SendDebugMsg("Modem Power On..\r\n\0");
                     SetModemPwrOn();
                     ModemAgain =FALSE;
                     ModemIsOn = TRUE ; 
                     FlagsStatus =  eFLAGS_STATUS;
                     SetStatusReg(MODEM_ON ,MODEM_ON);
                     eFLAGS_STATUS =  FlagsStatus; 
                #ifdef WATER_METER_ACTIVE         
                      ENABLE_PJ2_INT();    //disable current int  
                #endif      
                break;

                //4.4.13: add next 4 lines:
                //set off iggnition pulse
                case SUB_TASK_INIT_MODEM_IGN:

                     MODEM_TYPE = eModemType;      //read type from eeprom
                   //  MODEM_TYPE = SIM800;
                      if((MODEM_TYPE < 1) || (MODEM_TYPE > 3))
                      SendDebugMsg("Modem Type yet unknown..!\r\n\0");

                     if(MODEM_TYPE == HE910)//  #ifdef HE910
                       ModemIgnit();


                    ModemIsOn = TRUE ;
//                    SetAlarmTiming(3);     //set alarm in 2 minutes as WD in case modem failure
//                    EIFR |= (1<<INTF2) ;
//                    ENABLE_CLOCK_INT(); // enable external WD on interrupt2 //Danny

                    ModemResponse = TASK_COMPLETE;
                    delay_ms(2000);

                break;

                case SUB_TASK_INIT_MODEM_OK:
                    cEndMark = '\0';
                    nMaxWaitingTime = 25;   // wait max 2.5 sec for answer
                    nMaxFailuresNum = 5;
                    bNeedToWait4Answer = TRUE;
                 
                    SendATCmd(AT_IsModemOK);
                 //   TimeLeftForWaiting = 70;
                    delay_ms(300);        //300116

                break;  
                
                 case SUB_TASK_INIT_MODEM_RSSI:
                    longAnswerExpected = FALSE;
                    nMaxWaitingTime = 30;
                    nMaxFailuresNum = 20;
                    bNeedToWait4Answer = TRUE;
                    //ask modem RSSI with network host
                    SendATCmd(AT_CSQ);
                  
                break;

                case SUB_TASK_INIT_MODEM_COPS:
                    //treat leds:
//                    if (!bExtReset)
//                        TurnOnLed(LED_1, SUCCESS);
                    //select the  operator
               //     SendOperatorSelection();     Danny - no need at Creacell
                break;

                case SUB_TASK_INIT_MODEM_REG:

                      MODEM_TYPE = eModemType;      //read type from eeprom
                      if((MODEM_TYPE < SIM800) || (MODEM_TYPE > 3))
                      SendDebugMsg("Modem Type yet unknown..!\r\n\0");

                     nMaxFailuresNum = 50;           //was 15 Danny -50* 3 = 150
                     SendATCmd(AT_IsModemReg);

                break;
                case  SUB_TASK_INIT_MODEM_DELAY:   
                    
                         heat_time = 30; //delay before creg
                break;
              case SUB_TASK_INIT_MODEM_REG_GPRS:
                    nMaxFailuresNum = 20;
                   bNeedToWait4Answer = TRUE;          //was 15 Danny -7 * 3 = 21 Sec max waiting time
                     SendATCmd(AT_IsModemRegGPRS);
                    ServerResponseTimeOut = 70;
                break;


                case SUB_TASK_INIT_MODEM_GET_COPS:
                    nMaxFailuresNum = 1;
                    SendATCmd(AT_COPS_ASK);
                break;

                 case SUB_TASK_MODEM_CONNECT_ATCH:
                    nMaxWaitingTime = 30;   // wait max 2.5 sec for answer
                    nMaxFailuresNum = 8;
                    bNeedToWait4Answer = TRUE;
                    SendATCmd(GPRS_ATTACH);    //AT+CGATT=1\r\n\0";
                     ServerResponseTimeOut = 90;

                break;

                case SUB_TASK_INIT_MODEM_MONITOR:
                    longAnswerExpected = FALSE;
                    nMaxWaitingTime = 30;
                    nMaxFailuresNum = 5;
                    bNeedToWait4Answer = TRUE;

                if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
                   {
                       SendATCmd(AT_CIPMODE);  //transparent momde
                       delay_ms(500);
                   }
                break;

               
            }
        break;
        case TASK_MODEM_CONNECT:

//        #ifdef DebugMode
//         SendDebugMsg("ModemMain-TASK_MODEM_CONNECT\r\n\0");
//        #endif DebugMode

            switch (modemCurSubTask)
            {


                case SUB_TASK_MODEM_CONNECT_SETUP1:


                    nMaxWaitingTime = 60;   // wait max 5 sec for answer
                    nMaxFailuresNum = 2;

                      if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))//
                      SendATCmd(DEF_QULT_MIN_PROF);   //AT+CIICR SIM800 --telit  "AT+CGQMIN=1,0,0,0,0,0\r\n\0"
                       if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
                      SendATCmd(CIICR);
                break;

               case SUB_TASK_MODEM_CONNECT_SETUP2:

                     if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
                    SendATCmd(DEF_QULT_REQ_PROF);   //AT+CGQREQ=1,0,0,0,0,0

                break;

               case SUB_TASK_MODEM_CONNECT_SETUP3:

                      if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
                      SendATCmd(DEF_SCKT_CNFG);     //"AT#SCFG=1,1,500,30,150,3

                break;

                case SUB_TASK_MODEM_CHK_ICCID:
                      if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
                      SendATCmd(AT_CCID);    //
                      else
                      SendATCmd(AT_CCID1);    //SiM800
                break;

                case SUB_TASK_MODEM_CONNECT_PDP_DEF:
                     nMaxWaitingTime = 30;
                     nMaxFailuresNum = 1;
                     SendPDPCntDef();
                     delay_ms(1500);
                     ServerResponseTimeOut = 70;

                break;


                case SUB_TASK_MODEM_CONNECT_ACTV:
                    longAnswerExpected = FALSE;      //150116 test
                    nMaxWaitingTime = 70;  //wait max 10 sec
                    nMaxFailuresNum = 7;    //was 10-Danny

               if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
               {
                    if(failCnt == 3)
                    {
                          SendATCmd(DE_ACTIVATE_CNTXT);   // AT#SGACT=1,0
                    }
                 }
                     delay_ms(1000);
                      if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
                           SendATCmd(ACTIVATE_CNTXT);
                     else   if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
                          SendATCmd(CIFSR);
                    delay_ms(2500);                //150116 test
                    ServerResponseTimeOut = 70;
                break;

                case SUB_TASK_MODEM_CONNECT_START_DIAL:
                      ServerComOn = FALSE;
                      nMaxWaitingTime = 120;  //wait max 10 (20)  sec    dannny
                     nMaxFailuresNum = 1;


                      Timer0_Ticks = 16;

                       BatLevel_ON();
                      SendStartDial();


                 //   SendStartDial_test();
                      delay_ms(100);      //150116 test
                       ServerResponseTimeOut = 70;   //???????????????????????

                break;
            }
        break;
        case TASK_MODEM_POST:


          //  cEndMark = '#';
              cEndMark = '\0';

            nMaxWaitingTime = 260;  // wait max 26 (30) sec fot response
            bPostAnswered = FALSE; 
            
            

            switch (modemCurSubTask)
            {

                case SUB_TASK_MODEM_POST_DATA:
                //     SendDebugMsg("Popst Data..\r\n\0");
                     Found200 = FALSE;
                     nMaxFailuresNum = 1;    //test
                     Timer0_Ticks = 25;    //extra time to wait for end of msg -1 tick=14mS

                    if(IsCoverAlertNeeded == TRUE)
                    {     
                         IsCoverAlertNeeded = FALSE;
                         SendPostInfo(0);  //0 for Cover Alert      140416                         
                    } 
                   else if(TWI_err_alert_needed == TRUE)
                    {
                           TWI_err_alert_needed = FALSE;
                           SendPostInfo(2);
                    } 
                    else if(Power_Failure_alert_needed == TRUE)
                    {
                           Power_Failure_alert_needed = FALSE;
                           SendPostInfo(7);
                    } 
                    else if(GPS_data_needed == TRUE)
                    {
                           GPS_data_needed = FALSE;
                           SendPostInfo(8);
                    } 
                     else if(GPS_data_needed == TRUE)
                    {
                           GPS_data_needed = FALSE;
                           SendPostInfo(8);
                    }   
                    
                    
                    else
                    {
                           SendPostData();
                             
                           SendDebugMsg("Data Blocks has sent..\r\n\0");
                                SendDebugMsg("Wait for server response 15 Sec. max..\r\n\0"); 
//                          heat_time = 50; 
//                           while(( Found_200 == FALSE) && (heat_time > 0));
                           delay_ms(2000); 
                          if( Server_Error_msg == TRUE)   //in rx0 int
                          {     
                               Server_Error_msg = FALSE;
                                modemCurTask = TASK_MODEM_CLOSE;
                               modemCurSubTask = SUB_TASK_MODEM_CLOSE_PPP;
                               break; 
                          }
                           if(ModemResponse == TASK_FAILED)
                           {
                                SendDebugMsg("Faild sending data. Memory initialized.. \r\n\0");
                                break;
                           }
                           else
                           {
//                               SendDebugMsg("Data Blocks has sent..\r\n\0");
//                                SendDebugMsg("Wait for server response 15 Sec. max..\r\n\0");
                                ServerResponseTimeOut = 150;    //wait for  server up to 150 sec.
                                TimeLeftForWaiting = 160;
                                ModemResponse = NO_ANSWER;
                            }
                    }  
                 break;
                    
                   case SUB_TASK_MODEM_FLASH_UPDATE:     //new 131217
//                        #define RX_FILE_OK 3
//                        #define GENERAL_ERR 4
//                        #define BAD_FORMAT 5
//                        #define NEW_VER_OK 6  
                        
//                          nMaxWaitingTime = 50;
                           nMaxFailuresNum = 1;

                          SetAlarmTiming(3);     //set alarm in 3 minutes as WD in case modem failure
                          EIFR |= (1<<INTF2) ;
                          ENABLE_CLOCK_INT(); // enable rtc as external WD on interrupt2 //Danny
                          delay_ms(200);  
                       
                        //  SendPostAckUpdate(2);        //post ask server for update file 
                        
                          temp =  RxUpdateFile();  //receive new firmware file and save it in ext memory
                     //     PrintNum((long)temp);                   

                           delay_ms(1000);
                          if(temp)       //file received
                          {
                                  UpdateSession = FALSE;  //show RX at int   again
                                 Close_server_socket();  //temporary close server connection
                                 ok = ValidityCheck(0);
                                 #asm("wdr");
                                 delay_ms(500);
                             
                                  server_ReDial(0);    //re connect to server
                      //          if(ok < 2)         //validiation success
                                if(ok == 0)         //validiation success
                                 {
                                      UpdateFileOK = TRUE;    
                                        
                                       notification_index = RX_FILE_OK;
                                       ModemResponse = TASK_COMPLETE;

                                 }
                                 else
                                 {
                                     SendDebugMsg("Validation error..\r\n\0"); 
                                      notification_index = BAD_FORMAT;
                                      updateErrorCount++;
                                      ModemResponse = TASK_FAILED;
                                 }
                          }
                          else   //file not received ok
                          {
                                  UCSR0B = 0x08;   //disable RX of rest of file file
                                  SendDebugMsg("File receiving error..\r\n\0");
                              //    server_ReDial(1);

                                  notification_index = GENERAL_ERR;
                                  updateErrorCount++;
                                   ModemResponse = TASK_FAILED;

                          }


                      break;

               case SUB_TASK_MODEM_SEND_NOTIFICATION:

                                nMaxFailuresNum = 2;
                                nMaxWaitingTime = 30;  // wait max 4 sec for response
                                cEndMark = '\0';                                
                                bCheckRxBuf = FALSE; 
                                 
                            switch(notification_index )
                            {
                                 case 0:
        //                                 SendDebugMsg("send post update..\r\n\0");
        //                                 SendPostAckUpdate(1); //ask for update file
        //                                 ModemResponse = TASK_COMPLETE;
                                 break;

                                  case 1:

                                 break;

                                  case RX_FILE_OK:       //3 
                                  case NEW_VER_OK:  //6 
                                          delay_ms(300);
                                       //  UpdateFileOK = TRUE;
                                         SendPostInfo(RX_FILE_OK);  //notify success
                                 break;

                                  case GENERAL_ERR:    //4
                                             SendDebugMsg("General error..\r\n\0");
                                             delay_ms(300);
                                             SendPostInfo(GENERAL_ERR);
                                 break;

                                  case BAD_FORMAT:   //5
                                           SendDebugMsg("Validation error..\r\n\0");
                                            delay_ms(300);
                                           SendPostAckUpdate(BAD_FORMAT);
                                 break;

//                                  case NEW_VER_OK:  //6 
//                                           delay_ms(300);
//                                           SendPostAckUpdate(NEW_VER_OK);
//                                 break;
                            }
                              #asm("wdr");
                            if(ModemResponse == TASK_FAILED)
                            {
                                SendDebugMsg("Faild Getting server response for alert.. \r\n\0");
                                break;
                            }
                               ServerResponseTimeOut = 70;    //wait for server up to 7 sec.
                               TimeLeftForWaiting = 70;
                           //    ModemResponse = NO_ANSWER;

              break;
            }
        break; //TASK_MODEM_POST

        case TASK_MODEM_CLOSE:

            nMaxFailuresNum = 2;
            nMaxWaitingTime = 30;  // wait max 4 sec for response
            cEndMark = '\0';
            Timer0_Ticks = 14;

            switch (modemCurSubTask)
            {
                case  SUB_TASK_MODEM_CLOSE_PPP:
                     SendDebugMsg("CLOSE_PPP\r\n\0");
                    bNeedToWait4Answer = TRUE;
                    nMaxWaitingTime = 20;
                    nMaxFailuresNum = 1;
                    SendString(plusplus, 3);
                    delay_ms(1500);            //????
                    TimeLeftForWaiting = 30;

                    TestFlag = TRUE;    //trest???????????????????
                    ModemResponse = TASK_COMPLETE;

                break;

                case SUB_TASK_MODEM_CLOSE_TCP:
            //        SendDebugMsg("at CLOSE_TCP\r\n\0");
                    nMaxWaitingTime = 30;
                    if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
                           SendATCmd(AT_TCP_CLS);
                     else  if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
                          SendATCmd(CIPCLOSE);

                    if ((toDoList == DO_DATA) && (dataSentOK == TRUE))
                        bConnectOK = TRUE;
                        delay_ms(1500);
                       ModemResponse = TASK_COMPLETE;
                break;

                case SUB_TASK_MODEM_CLOSE_MDM:
               //     SendDebugMsg("at CLOSE_MDM..\r\n\0");
                //   nMaxFailuresNum = 2;
                   nMaxFailuresNum = 1;
                   nMaxWaitingTime = 25;  // wait max 4 sec for response

//                   if(ClockUpdated == FALSE)
//                    RTC_Update_by_Modem();

                      if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
                          SendATCmd(MODEM_SHDN);
                       else  if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
                          SendATCmd(CIPSHUT);

                      ModemResponse = TASK_COMPLETE;
                       delay_ms(1500);

                break;

                case SUB_TASK_MODEM_OFF:

                   SendDebugMsg("MODEM_OFF..\r\n\0");


                if(ModemRepeatNeeded == TRUE)
                {
//                       if((ErrorType ==4) ||(ErrorType ==5) ||(ErrorType ==7) ||(ErrorType == 8)||(ErrorType == 9 ))
//                       {
//                           if(  ModemRepeatCount == 1)
//                           {
//                                SendDebugMsg("COM Failure - new COPS proc..\r\n\0");
//                                ModemForgetOldCop();
//                                ModemRepeatCount++;
//                                ModemAgainCount = 0;
//                                objToMsr = SENSOR1;
//                                break;
//                           }
//                      } 
                       ModemRepeatCount++;
                }
                //==============test================
//                     SendATCmd(AT_CRSM);
//                     delay_ms(1000);
                //=======================================
//                   if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
//                              SendATCmd(MODEM_SHDN);
//                     else  if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
                    SendATCmd(CIPSHUT);

                    delay_ms(2000);
//                    if (MODEM_TYPE == HE910)
//                    {
//                        ServerResponseTimeOut = 100;          //shorter than WD
//                        TimeLeftForWaiting = 100;
//                        #asm("wdr")
//                         delay_ms(2000);
//                         #asm("wdr")
//                          delay_ms(4000);
//                    }

               
                    SetModemPwrOff();
                    bEndOfModemTask = TRUE;

                                    
                      
                       if(DataPointersMoveNeeded == TRUE)      //true if data post closed OK
                       {
                             DataPointersMoveNeeded = FALSE;   //set in CheckResults 
                           
                             if (UpdateFileOK == FALSE)      //if true - memory has update file in it- dont touch
                             InitDataBlocks(cpue2_interval_1);   //new - init data blocsk - set the pBread pointers

//                             for(i = 0; i < NumSensors; i++)
//                             eDataPtrSave[i] = 0;       //no need to keep pointers any more  
                                  
                             eRESTORE_DATA_POINTERS = RELEASE_POINTERS; 
                             DataSent = FALSE;
                       }
                       else    //restore pointers of data in mem. send it later
                       {  
                            if(eRESTORE_DATA_POINTERS == NEED_TO_RESTORE) 
                            {
                                ResetAllReadPointers();// pointer set in GetMeasurments()  
                                  eRESTORE_DATA_POINTERS = RELEASE_POINTERS; 
                            }
                       }    
    
                  if(( ModemAgain == TRUE) &&(ModemAgainCount < 1))
                  {
                         ModemAgainCount++;
                         ModemAgain = FALSE;
                         ErrorType = 0;
                         InitVarsForConnecting();
                         SendDebugMsg("MODEM is trying AGAIN..\r\n\0");
                         delay_ms(3000);
                  }
                  
//--------------------------FOTA session------------------------------
                    if (UpdateFileOK == TRUE)
                    {
                          UpdateFileOK = FALSE;
                           WDT_off();

                           SendDebugMsg("Jump to bootloaer..\r\n\0");

                          #asm
                              jmp 0xF000;  //go to bootloader
                          #endasm
                    }

   //-------------------------------------------------------------------

   //-------------------------------------------------------------------  
                   if(ErrorType > 0 )
                   {                                                          
                          SaveErrorState(ErrorType);
                          ErrorType = 0; 

                   }
                 ConnectedToServer = FALSE;
                 ServerComOn = FALSE;
                 Server_Error_msg = FALSE;

                 TestFlag = FALSE;
                 ModemIsOn = FALSE ; 
                 FlagsStatus =  eFLAGS_STATUS;
                 SetStatusReg(MODEM_ON , 0);
                 eFLAGS_STATUS =  FlagsStatus;

                 break;
            }
        break;
        default:
    }
    timeCnt = 0;


}

//#define 0x01 ALERT_1 // kando buttle state: 0 - Empty, 1 - Full
//#define 0x20 RESET_BOTTLE_STATE // 0 - set bottle state as empty, 1 - full

//#define 0x40 RELAY1_STATE // PUMP is connected to relay1: 0 - turn pump off, 1 - turn pump on
//#define 0x41 RELAY2_STATE // 0 - relay2 off, 1 - relay2 on
//
////Sensor TH-levels (from device to server - on quary request, or from server to device update values)
//#define 0x60 TEMP_TH // values: <TH MAX>
//#define 0x61 PH_TH   // values: <PH MIN><PH_MAX><PH_BOTTLE_MIN><PH_BOTTLE_MAX>
//#define 0x62 EC_TH   // values: <EC_MIN><EC_MAX><EC_BOTTLE_MIN><EC_BOTTLE_MAX>
//#define 0x63 WATER_LEVEL // values: <WATER_LEVEL_HIGHT>, unit:cm

// *str = {"60":50,"61":[11,400,89,0]}
//210 {"60":400,"61":[0,150,0,150],"62":[1365,4369],"70":1,"41":90}

//210{"40":180,"60":400,"61":[30,100,50,110],"62":[1250,1875],"63":[-1,-1,-1,-1],"64":[-1,-1,-1,-1],"70":1}
//200{"90": "16/08/16, 12:12:53:+00"} 
//index point to begininh of str ({)
char Pharse_params_struct(char *str, unsigned int index)
{
      char i,chr, cnt, sSDI;
       int val = 0;;
      unsigned char WLcount, index1, index2, index3, index4;
      unsigned int p;                                                //\"66\":[10000,32767,6000,32767,32767,32767],

   sSDI = 0xFF;
   p = index;

  do
   {
        while((str[p] != '"') && (str[p] != '}'))
        p++;

        if(str[p] != '}')       //p point to '"'
        {
            p++;                    //skip " - point to a key val
            chr = str[p];           //keep the char
            sSDI = 255;

              index1 = index2 = 0xFF;
               WLcount = 0;

            switch (chr)            //check the char
            {
                case '0':
                break;

                case '2':          //bottle state
                       chr = str[(int)p + 1];  //check second char of key
                       switch (chr)            //check second char
                      {
                          case '0':          //key = 20

                               SendDebugMsg("Reset Bottle State\r\n\0");
                                         //key = 20

                              val = 0;
                              p += 4;       //points to value
                              val = pStr2Bin (str, p);

                              if(val == 1)
                              {
                                  ResetPumpFlags();
                                  SendDebugMsg("Reset Pump and bottle\r\n\0");
                              }

                          break;
                      }

                break;  // case '2':

                case '4':
                          chr = str[(int)p + 1];  //check second char
                          p += 4;       //points to value -
                          val = pStr2Bin (str, p);

                       switch (chr)            //check the char
                      {
                        case '0':
                         break;

                         case '1':               ////key = 41 -PUMP timing

                               SendDebugMsg("Relay 2 timing setting\r\n\0");

                             #asm("cli")
                               ePUMP_HEAT_TIME = val ;   //store in eeprom location. temp is first sensor
                             #asm("sei")
                          break;

                           case '2':       //key = 42
                                 SendDebugMsg("Relay 1 EN/DIS\r\n\0");
                                  #asm("cli")
                                  eRelay1Enabled = val ;   //store in eeprom location. temp is first sensor
                                  #asm("sei")
                                  Relay1Enabled = val ;
                             break;

                         case '3':       //key = 43
                                  SendDebugMsg("Relay 2 EN/DIS\r\n\0");
                                  #asm("cli")
                                  eRelay2Enabled = val ;   //store in eeprom location. temp is first sensor
                                  #asm("sei")
                                  Relay2Enabled = val ;
                         break;

                         case '4':    //270117 EC value  for pump PH limit
//                                  #asm("cli")
//                                  eECTresholdVal = val;    //store in eeprom location.
//                                  #asm("sei")
//                                  SendDebugMsg("EC Value for pump limit. val = \0");
//                                 PrintNum((long)val);
                         break;
                      }
                break;    //case 4

                case '6':               //first char of key is 6

                     chr = str[(int)p + 1];  //check second char
                     switch (chr)
                    {
                         case '0':    //KEY 60   //Evogen sensor added. Two params added 170419
                            //   SendDebugMsg("Temperature High Tresh setting\r\n\0");            
                               
                              for(i=0; i< NumSensors; i++)
                              {
                                 if(( SensorType[i] == AT_B) || (SensorType[i] == AT) || ( SensorType[i] == TRE_150)) 
                                 {
                                    
                                     break;
                                 }
                                 
                              }
                               cnt = 0;
                               p += 5;       //points to value - max temp
                                                     
                               if(i < NumSensors)   //PH sensor found
                             {
                                     do{
                                       val = pStr2Bin (str, p);   //temp high TH  

                                       switch (cnt)
                                      {

                                          case 0:
                                                #asm("cli")                                             
                                                   MIN_LIMIT[i] = val;  //low tmp value                                         
                                                #asm("sei")  
                                                      SendDebugMsg("Temp Low Treshold setting\r\n\0");
                                          break;

                                           case 1:
                                               #asm("cli")                                              
                                                   MAX_LIMIT[i] = val;  //                                             
                                                #asm("sei")
                                                  SendDebugMsg("Temp  High Treshold setting\r\n\0");
                                          break;
                                       }
                                        
                                        while((str[p] != ',') && (str[p] != ']'))  //look for next value
                                          p++;
                                          p++;       //point to first char of next value
                                          cnt ++;    //inc counter

                                   }while (cnt < 2);  
                               
                             }
                            break;   //case 60


                          case '1':      //key for  PH_TH
                           SendDebugMsg("PH Tresh setting\r\n\0");
                            for(i=0; i< NumSensors; i++)
                           {
                                 if(( SensorType[i] == COMBO2_PH_SDI12) ||( SensorType[i] == PH)||(SensorType[i] == COMBO_PH_485))
                                 {
                                     sSDI = 0;
                                     break;
                                 }
                                 else if (( SensorType[i] == COMBO4_SDI12) || ( SensorType[i] == COMBO4_485))
                                 {
                                      sSDI = 1;
                                      break;
                                 }
                           }
                           if(i < NumSensors)   //PH sensor found
                           {

                              p += 5;       //points to first value - min ph
                              cnt = 0;
                              val = 0;
                                do{
                                   val = pStr2Bin (str, p);

                                   switch (cnt)
                                  {

                                      case 0:
                                            #asm("cli")
                                           if(sSDI == 0)
                                               MIN_LIMIT[i] = val;  //PH value is first in comb
                                           else if(sSDI == 1)
                                                MIN_LIMIT[(int)i+2] = val;    //PH value is third in combo
                                              SendDebugMsg("PH  Low Tresh setting\r\n\0");
                                            #asm("sei")
                                      break;

                                       case 1:
                                           #asm("cli")
                                             if(sSDI == 0)
                                               MAX_LIMIT[i] = val;  //store in eeprom location.
                                           else if(sSDI == 1)
                                                 MAX_LIMIT[(int)i+2] = val;    //store in eeprom location. temp is first sensor
                                            #asm("sei")
                                              SendDebugMsg("PH  High Tresh setting\r\n\0");
                                      break;

                                       case 2:
                                            #asm("cli")
                                            if(sSDI == 0)
                                               PUMP_MIN_LIMIT[i] = val;  //store in eeprom location.
                                            else if(sSDI == 1)
                                                 PUMP_MIN_LIMIT[(int)i+2] = val;   //store in eeprom location. temp is first sensor
                                             #asm("sei")
                                              SendDebugMsg("PH Pump Low Tresh setting\r\n\0");
                                      break;
                                       case 3:
                                            #asm("cli")
                                             if(sSDI == 0)
                                               PUMP_MAX_LIMIT[i] = val;  //store in eeprom location.
                                           else if(sSDI == 1)
                                                PUMP_MAX_LIMIT[(int)i+2] = val;   //store in eeprom location. temp is first sensor
                                             #asm("sei")
                                             SendDebugMsg("PH Pump Hgh Tresh setting\r\n\0");
                                      break;


                                   }
                                    #asm("sei")
                                    while((str[p] != ',') && (str[p] != ']'))  //look for next value
                                      p++;
                                       p++;    //point to first char of next value
                                       cnt ++;

                               }while (cnt < 4);
                           }
                         break;   //case 1

                         case '2':     //EC_TH
                          SendDebugMsg("EC Tresh setting\r\n\0");
                           for(i=0; i< NumSensors; i++)
                           {
                                 if(( SensorType[i] == EC_K0)||(SensorType[i] == COMBO_EC_485))
                                 {
                                    sSDI = 0;
                                     break;
                                 }
                                  else if  (( SensorType[i] == COMBO4_SDI12) || ( SensorType[i] == COMBO_5TE) ||\
                                  ( SensorType[i] == COMBO2_EC_SDI12) ||  ( SensorType[i] == COMBO4_485))
                                  {
                                     sSDI = 1;
                                     break;
                                  }
                           }
                            //if i< NumSensors i has index of sensor in list of sensors
                           if(i < NumSensors)   //PH sensor found
                           {

                              p += 5;       //points to first value - max ec
                              cnt = 0;
                              val = 0;

                            do{

                               val = pStr2Bin (str, p);
//
                                  switch (cnt)       //parameter index
                                  {
                                      #asm("cli")
                                      case 0:
                                             if(sSDI == 0)
                                           {
                                                 MAX_LIMIT[i] = val;    //store in eeprom location.
                                                 MIN_LIMIT[i] = -32768;
                                              //   SendDebugMsg("EC High Tresh setting\r\n\0");

                                           }
                                           else if(sSDI == 1)
                                           {
                                                 MAX_LIMIT[0] = val;     //store in eeprom location. temp is first sensor
                                                 MIN_LIMIT[0] = -32768;;
                                              //    SendDebugMsg("EC High Tresh setting\r\n\0");

                                                //    PrintNum((long)val);
                                           }
                                            SendDebugMsg("EC High Tresh setting\r\n\0");

                                      break;

                                      case 1:
                                            if(sSDI == 0)
                                           {
                                                 PUMP_MAX_LIMIT[i] = val;    //store in eeprom location.
                                                 PUMP_MIN_LIMIT[i] = 0x8000;;    //store in eeprom location.
                                           }
                                           else if(sSDI == 1)
                                           {
                                                 PUMP_MAX_LIMIT[0] = val;      //store in eeprom location. temp is first sensor
                                                 PUMP_MIN_LIMIT[0] = 0x8000;;    //store in eeprom location.

                                              //    PrintNum((long)val);
                                           }
                                              SendDebugMsg("EC Pump High Tresh setting\r\n\0");
                                      break;

                                      default:
                                  }

                                #asm("sei")
                               val = 0;
                            while((str[p] != ',') && (str[p] != ']')) // look for next value
                            p++;
                            p++;    //point to first char of next value

                           cnt ++;
                           }while (cnt < 2);
                          }
                         break;   //case 2


   case '3':      //63 - WATER_LEVEL

                           for(i=0; i< NumSensors; i++)  //find sensors loc
                           {
                                  if(( SensorType[i] == WLV) ||( SensorType[i] == WLV_PULSTAR))
                                 {
                                     WLcount++;

                                     if(WLcount == 1)
                                     if(index1 == 0xFF)
                                     index1 = i;           //save index of sensor in list

                                      if(WLcount == 2)
                                      if(index2 == 0xFF)
                                      index2 = i;
                                 }
                           }
                           if(WLcount > 0)   //WL sensor found
                           {

                              p += 5;       //points to first value - min ph
                              cnt = 0;
                              val = 0;

                              do{
                                   val = pStr2Bin (str, p);
                                   switch (cnt)
                                  {
                                      case 0:

                                      //MIN value and MAX value are swaped due to meaurement mehtod in which water level
                                      //measured from top of SHUCHA
                                          //  if(val >= 0)
                                            {
                                                #asm("cli")
                                         //     MIN_LIMIT[index1] = val;  //
                                            MAX_LIMIT[index1] = val;  //store in eeprom location.
                                                #asm("sei")
                                                SendDebugMsg("WL Low vlaue saved\r\n\0");
                                             }
                                      break;

                                       case 1:

                                        //   if(val > -1)
                                           {
                                               #asm("cli")
                                           //       MAX_LIMIT[index1] = val;  //store in eeprom location.
                                                MIN_LIMIT[index1] = val;  //PH value is first in comb
                                                #asm("sei")
                                                SendDebugMsg("WL High value saved\r\n\0");

                                            }
                                      break;

                                        case 2:
                                        //    if(val > -1)
                                            {
                                                #asm("cli")
                                              //     MIN_LIMIT[index2] = val;  //PH value is first in comb
                                              //      MAX_LIMIT[index2] = val;  //PH value is first in comb

                                                     PUMP_MAX_LIMIT[index1] = val;    //06/09/16 overflow
                                                #asm("sei")
                                                 SendDebugMsg("WL OverFlow value saved\r\n\0");
                                             }
                                      break;

                                        case 3:
//                                           // if(val > -1)
//                                            {
//                                                #asm("cli")
//                                              //     MAX_LIMIT[index2] = val;  //PH value is first in comb
//                                                   MIN_LIMIT[index2] = val;  //PH value is first in comb
//                                                #asm("sei")
//                                                 SendDebugMsg("WL 2 High Tresh setting\r\n\0");
//                                             }
                                        break;

                                   }

                                    while((str[p] != ',') && (str[p] != ']'))  //look for next value
                                     p++;
                                     p++;    //point to first char of next value
                                     cnt ++;

                               }while (cnt < 4);
                           }

                         break;
        //{"41":44,"60":380,"61":[0,100,0,110],"62":[500,1500],"63":[-1,-1,-1,-1],"64":[-100,800,-100,800],"70":1}
                          case '4':      //64 key for ORP

                            for(i=0; i< NumSensors; i++)
                           {
                                 if(( SensorType[i] == COMBO2_PH_SDI12) ||( SensorType[i] == PH)||(SensorType[objToMsr] == COMBO_PH_485))
                                 {
                                     sSDI = 0;
                                     break;
                                 }
                                 else if  (( SensorType[i] == COMBO4_SDI12) ||  ( SensorType[i] == COMBO4_485))
                                 {
                                      sSDI = 1;
                                      break;
                                 }
                           }
                           if(i < NumSensors)   //PH sensor found
                           {

                              p += 5;       //points to first value - min ph
                              cnt = 0;
                              val = 0;

                                do{
                                   val = pStr2Bin (str, p);

                                   switch (cnt)
                                  {

                                      case 0:
                                         if(val != -1)
                                         {
                                                #asm("cli")
                                               if(sSDI == 0)
                                                   MIN_LIMIT[(int)i+1] = val;  //ORP value is second in comb
                                               else if(sSDI == 1)
                                                    MIN_LIMIT[(int)i+3] = val;    //ORP value is fourth in combo

                                                #asm("sei")
                                          }
                                      break;

                                       case 1:
                                        if(val != -1)
                                         {
                                               #asm("cli")
                                                 if(sSDI == 0)
                                                   MAX_LIMIT[(int)i+1] = val;  //store in eeprom location.
                                               else if(sSDI == 1)
                                                     MAX_LIMIT[(int)i+3] = val;    //store in eeprom location. temp is first sensor
                                                #asm("sei")
                                          }
                                       break;

                                        case 2:
                                         if(val != -1)
                                         {
                                                #asm("cli")
                                               if(sSDI == 0)
                                                   PUMP_MIN_LIMIT[(int)i+1] = val;  //ORP value is second in comb
                                               else if(sSDI == 1)
                                                    PUMP_MIN_LIMIT[(int)i+3] = val;    //ORP value is fourth in combo

                                                #asm("sei")
                                          }
                                          break;

                                       case 3:
                                         if(val != -1)
                                         {
                                                #asm("cli")
                                               if(sSDI == 0)
                                                   PUMP_MAX_LIMIT[(int)i+1] = val;  //ORP value is second in comb
                                               else if(sSDI == 1)
                                                    PUMP_MAX_LIMIT[(int)i+3] = val;    //ORP value is fourth in combo

                                                #asm("sei")
                                          }
                                         break;


                                   }
                                    #asm("sei")
                                    while((str[p] != ',') && (str[p] != ']'))  //look for next value
                                     p++;
                                     p++;    //point to first char of next value
                                     cnt ++;

                               }while (cnt < 4);

                             }

                      case '5':      //65 - Tensiomters
                           WLcount = 0;
                           for(i=0; i< NumSensors; i++)  //find sensors loc
                           {
                                 if(( SensorType[i] == TNS) ||( SensorType[i] == TNS_tronix))
                                 {
                                     WLcount++;       //just counter available for use.

                                     if(WLcount == 1)
                                     index1 = i;           //save index of sensor in list

                                      else if(WLcount == 2)
                                      index2 = i;

                                      else if(WLcount == 3)
                                      index3 = i;

                                       else if(WLcount == 4)
                                      index4 = i;
                                 }
                           }
                           if(WLcount > 0)   //TNS sensor found
                           {

                              p += 5;       //points to first value - min ph
                              cnt = 0;
                             // val = -1;

                              do{
                                   val = pStr2Bin (str, p);
                                   switch (cnt)
                                  {
                                      case 0:

                                            if(val > -1)
                                            {
                                                #asm("cli")
                                               if( SensorType[index1] == TNS_tronix)    //tens output high to low
                                                MIN_LIMIT[index1] = val;  //store in eeprom location.
                                                else
                                                MAX_LIMIT[index1] = val;  //store in eeprom location.
                                                #asm("sei")
                                                SendDebugMsg("Tens1 Alert Value updated\r\n\0");
                                             }
                                      break;

                                       case 1:
                                           if(val > -1)
                                           {
                                               #asm("cli") 
                                               if( SensorType[index2] == TNS_tronix)    //tens output high to low
                                                    MIN_LIMIT[index2] = val;  //store in eeprom location.
                                                else
                                                   MAX_LIMIT[index2] = val;  //store in eeprom location.

                                                #asm("sei")
                                                SendDebugMsg("Tens2 Alert Value updated\r\n\0");

                                            }
                                      break;

                                        case 2:
                                            if(val > -1)
                                            {
                                                #asm("cli")
                                               if( SensorType[index3] == TNS_tronix)    //tens output high to low
                                                   MIN_LIMIT[index3] = val;  //store in eeprom location.
                                                else
                                                   MAX_LIMIT[index3] = val;  //PH value is first in comb
                                                #asm("sei")
                                                 SendDebugMsg("Tens3 Alert Value updated\n\0");
                                             }
                                      break;

                                        case 3:
                                            if(val > -1)
                                            {
                                                #asm("cli")
                                                if( SensorType[index4] == TNS_tronix)    //tens output high to low
                                                    MIN_LIMIT[index4] = val;  //store in eeprom location.
                                                else
                                                   MAX_LIMIT[index4] = val;  //PH value is first in comb
                                                #asm("sei")
                                                SendDebugMsg("Tens4 Alert Value updated\n\0");
                                             }
                                   }

                                    while((str[p] != ',') && (str[p] != ']'))  //look for next value
                                     p++;
                                     p++;    //point to first char of next value
                                     cnt ++;

                               }while (cnt < WLcount);
                        }
                          break;
                          
                           case '6':                       //66 Evogen humidity  "66":[min,max]
                          
                            index1 = 0;
                            for(i=0; i< NumSensors; i++)  //find sensors loc
                           {
                                  if( SensorType[i] == TRH_300) 
                                 {                                  
                                     
                                      break;
                                 }
                           }
                           if(i < NumSensors)   //humidity  sensor found
                           {

                              p += 5;       //points to first value - min humid
                              cnt = 0;
                              val = 0;

                              do{
                                   val = pStr2Bin (str, p);
                                   switch (cnt)
                                  {
                                      case 0:

                                            if(val > -1)
                                            {
                                                #asm("cli")
                                                MIN_LIMIT[i] = val;  //                                           
                                                #asm("sei")
                                                SendDebugMsg("Humidity Low vlaue saved\r\n\0");
                                             }
                                      break;

                                       case 1:

                                           if(val > 0)
                                           {
                                                #asm("cli")
                                                 MAX_LIMIT[i] = val;  //store in eeprom location.                                               
                                                #asm("sei")
                                                SendDebugMsg("Humidity High value saved\r\n\0");
                                            }
                                       break;

                                   }

                                    while((str[p] != ',') && (str[p] != ']'))  //look for next value
                                     p++;
                                     p++;    //point to first char of next value
                                     cnt ++;

                               }while (cnt < 2);
                           }

                          break;  
                          
                          case '7':    //evogen radiatin
                                     
                                val = 0;
                                p += 4;       //points to value - max temp
                                val = pStr2Bin (str, p);   //temp high TH

                                 for(i=0; i< NumSensors; i++)
                                  {
                                     if( SensorType[i] == HD2021T)
                                     {
                                        break;
                                     }                                
                                  }

                                 if(i < NumSensors)      //i has index of the sensor in list
                                 {
                                     #asm("cli")                                   
                                         MAX_LIMIT[i] = val;   //store in eeprom location.
                                         MIN_LIMIT[i] = -32768;;    //negativ                                        
                                     #asm("sei")  
                                    SendDebugMsg("Radiation High Tresh setting\r\n\0");             
                                 }
                                                                                           
                          break;  
                                                   
                         
                           case '8':                       //68 DOROT_PRESSURE  "68":[min,max]
                          
                            for(i=0; i< NumSensors; i++)  //find sensors loc
                           {
                                  if( SensorType[i] == DOROT_PRESSURE) 
                                 {                                                                       
                                      break;
                                 }
                           }
                           if(i < NumSensors)   //humidity  sensor found
                           {

                              p += 5;       //points to first value - min humid
                              cnt = 0;
                             
                              do{
                                   val = pStr2Bin (str, p);
                                   switch (cnt)
                                  {
                                      case 0:

                                            if(val > -1)
                                            {
                                                #asm("cli")
                                                MIN_LIMIT[i] = val;  //                                           
                                                #asm("sei")
                                                SendDebugMsg("Set DOROT_PRESSURE Low treshold value. \r\n\0");
                                             }
                                      break;

                                       case 1:

                                           if(val > 0)
                                           {
                                                #asm("cli")
                                                 MAX_LIMIT[i] = val;  //store in eeprom location.                                               
                                                #asm("sei")
                                                SendDebugMsg("Set DOROT_PRESSURE High  treshold value.\r\n\0");
                                            }
                                       break;

                                   }

                                    while((str[p] != ',') && (str[p] != ']'))  //look for next value
                                     p++;
                                     p++;    //point to first char of next value
                                     cnt ++;

                               }while (cnt < 2);
                           }

                          break;  
                      }




                break;  //case 6

                case '7':

                       chr = str[(int)p + 1];  //check second char of key
                       switch (chr)            //check second char
                      {
                          case '0':          //key = 70

                              val = 0;
                              p += 4;       //points to value
                              val = pStr2Bin (str, p);
                              if(val == 1)
                              {
                                   eBackToNormalAlertEnabled = 1 ;     //stasus byte  ???
                                    BackToNormalAlertEnabled = TRUE ;
                                     SendDebugMsg("Back To Normal Alert ENABLED\r\n\0");
                              }
                               else
                              {
                                      eBackToNormalAlertEnabled = 0 ;     //stasus byte  ???
                                      BackToNormalAlertEnabled = FALSE;
                                      SendDebugMsg("Back To Normal Alert DISABLED\r\n\0");
                              }
                          break;

                            case '1':          //key = 71  interval multiplier

                              val = 0;
                              p += 4;       //points to value
                              val = pStr2Bin (str, p);
                              if(((val > 0) && (val < 5)) || (val == 6) || (val == 12) || (val == 24))
                              {
                                        cpue2_interval_1 =(char) val;
                                        SendDebugMsg("Interval set..\r\n\0");
                              }
                              else  SendDebugMsg("|Illigal Interval value..\r\n\0");

                          break;

                           case '2':     //COMMUNICATION params

                                SendDebugMsg("Communication params setting\r\n\0");


                                  p += 5;       //points to first value - max ec
                                  cnt = 0;
                                  val = 0;
                               do{

                                 val = pStr2Bin (str, p);
                                 switch (cnt)       //parameter index
                                {
                                      #asm("cli")
                                      case 0:
                                          eStartConnectionH = val;
                                      break;

                                      case 1:
                                          eConnectionInDay = val;
                                      break;

                                       case 2:
                                          eConnectIntervalH = val;
                                       break;
                                  }

                                #asm("sei")
                               val = 0;
                               while((str[p] != ',') && (str[p] != ']'))  //look for next value
                                p++;

                                p++;    //point to first char of next value
                               cnt ++;
                           }while (cnt < 3);
                         break;   //case 2  
                         
                           case '5':     //TEMP_HUMIDITY temp 2 tresholds
                            for(i=0; i< NumSensors; i++)
                           {
                                 if( SensorType[i] == TEMP_HUMIDITY)
                                 {
                                    sSDI = 0;
                                     break;
                                 }
                                
                           }
                            //if i< NumSensors i has index of sensor in list of sensors
                           if(i < NumSensors)   //PH sensor found
                           {
                              p += 5;       //points to first value - max ec
                              cnt = 0 ;                           

                             do
                             {

                               val = pStr2Bin (str, p);
//
                                  switch (cnt)       //parameter index
                                  {
                                      
                                      case 0:
                                             if(sSDI == 0)
                                           {    
                                                 #asm("cli")
                                                 MAX_LIMIT[i] = val;    //store in eeprom location.
                                                 MIN_LIMIT[i] = -32768;
                                              //   SendDebugMsg("EC High Tresh setting\r\n\0");
                                                 #asm("sei")
                                           }
                                         
                                            SendDebugMsg("Temperature High Tresh setting\r\n\0");

                                      break;

                                      case 1:
                                            if(sSDI == 0)
                                           {     
                                                 #asm("cli")
                                                 PUMP_MAX_LIMIT[i] = val;    //store in eeprom location.
                                                 PUMP_MIN_LIMIT[i] = 0x8000;;    //store in eeprom location.
                                                 #asm("sei")
                                           }
                                           else if(sSDI == 1)
                                           {     
                                                 #asm("cli")
                                                 PUMP_MAX_LIMIT[i] = val;      //store in eeprom location. temp is first sensor
                                                 PUMP_MIN_LIMIT[i] = 0x8000;;    //store in eeprom location.
                                                 #asm("sei")
                                              //    PrintNum((long)val);
                                           }
                                              SendDebugMsg("EC Pump High Tresh setting\r\n\0");
                                      break;

                                      default:
                                  }

                              
                                 val = 0;
                                 while((str[p] != ',') && (str[p] != ']')) // look for next value
                                 p++;
                                 p++;    //point to first char of next value

                             cnt ++;
                             }while (cnt < 2);
                          }
                          break; 
                       }
                break;

                case '8':        //reset unit
                       chr = str[(int)p + 1];  //check second char of key
                       switch (chr)            //check second char
                      {
                          case '0':          //key = 80

                              val = 0;
                              p += 4;       //points to value
                              val = pStr2Bin (str, p);
                              if(val == 1)
                              {

                                        eRESTORE_DATA_POINTERS = RELEASE_POINTERS;   //dont restore pointers from eeprom
                                        InitDataBlocks(cpue2_interval_1);
                                        ResetPumpFlags();  //at Measure.c
                                        IsAlertActivated = FALSE;      //updated 320815 for Tzaci request
                                        IsCoverAlertNeeded = FALSE;

                                         FlagsStatus =  eFLAGS_STATUS;
                                         SetStatusReg(IS_ALERT_NEEDED , 0);
                                         eFLAGS_STATUS = FlagsStatus;

                                        SeverResetCommand = TRUE;
                                      SendDebugMsg("RESETing unit by server..\r\n\0");
                              }

                          break;
   //-----------                     -----update FW----------------------- 
                           //200{"90":"18/04/24, 07:37:01:+12","81":"1ba90843-acc8-4201-bf20-53c797c7bdc5"}
                           
                            case '1':    //81  UUID string handling
                                    
                                   p += 5;       //points to string   66db8ceb-fcf7-4fba-a95e-4cb8dace32cf
//                                   for(i = 0; i < 36; i++)
//                                   {         //get uuid
//                                      sUUID[i] = str[p++];
//                                      eUUID[i] = sUUID[i];       //into eeprom as well for further use
//                                   }
                                   UpdateFileOK = FALSE;           //true if file received ok
                                   notification_index = 0;
                                   FirmwareUpdateTime = TRUE;                                  
                                   DISABLE_RX_INT_UART2();     //dont let it interfer
                                   Timer0_Ticks = 20;
                                   SendDebugMsg("FOTA request..\r\n\0");
                            break;
                            
                            case '4':        //84 list of sensors srver typs 
                            
                                  p += 5;       //points to first value -
                                  cnt = 0;                                
                                 do
                                 {
                                         val = pStr2Bin (str, p);                                                                              
                                         #asm("cli")                                                                               
                                             IOType[cnt] = val;    //store in  eeprom server type location.
                                             SensorType[cnt] = val;  //store in  eeprom sensor type location.                                                                                           
                                         #asm("sei")                                                                              
                                                                                           
                                         while((str[p] != ',') && (str[p] != ']')) // look for next value
                                         p++;   
                                         
                                         cnt ++;
                                         if (str[p] != ']')
                                         p++;    //point to first char of next value
                                        
                                 }while (str[p] != ']');  
                                 NumSensors = cnt;                        
                              //   Sensors_List_Updated = TRUE;   //flag it for further use
                            break; 
                       }
                break;

//                case '9':     //not here - special func
//                          chr = str[(int)p + 1];  //check second char of key
//                       switch (chr)
//                      {
//                          case '0':           //"90": [16/11/26,13:52:23:+03]
//                                p += 6;
//                                RTC_Update_by_Server( str, p);
//
//                          break;
//                       }
//                break;
              default:
            }
         }
   }   while((str[p] != '}') && ( p < MAX_RX0_BUF_LEN));     //while not end of string   MAX_RX0_BUF_LEN)

   if(FirmwareUpdateTime == TRUE)  //different treatment
   return 2;                    //handle firmware update
   return 1;
}

//200{"16/08/16,14:01:14+12"}         //27

//200{"90": "16/08/16, 12:12:53:+00"}     //kanso format
//200{"90": "16/09/12, 10:13:25:+12"}
  //update clock by server 100716
// 200{"90":"19/03/26, 10:14:10:+08","60":1680,"72":[6,18,1,"70":0} 
// 200{"90":"19/03/26, 11:17:16:+08"}
char RTC_Update_by_Server( char *str, int ptr)
 {
   char clkBuf[6];
 //char clkBuf_1[6];
   int tmp,i, p;

      //   SendDebugMsg("\r\n RTC_Update_by_Server().. \r\n\0");
       p = ptr;
       p = 0;
       i = 0;   
                   str[rx0_buff_len] = '\0';
                   UART_WriteMsg(str);
       
                  do{
                       if (str[p] == '"') //&& (str[p+1] == '9') //look for three "
                       i++;
                       p++;
                     }  while((i < 3) &&( p < 30));

                   if(p == 30)
                   {
                       SendDebugMsg("\r\nFailed: CLOCK string not valid.! \r\n\0");
                        return 0;
                   }

                   tmp = 0;
                   i = p ;
                   while ((str[i] != '"') && (tmp < 23))  //look for 4th "
                   {
                      i++;
                      tmp++;
                   }

                    if(tmp == 23)     //string not omplete
                    {
                             SendDebugMsg("\r\nCLOCK string not complete..ignored! \r\n\0");
                              return 0;
                    }

                        //collect date/time values
                         clkBuf[0]= ((str[p]-0x30) * 10) + (str[p+1]-0x30);    //year
                     
                        //PrintNum((long)clkBuf[0]);
                         p += 3;

                         clkBuf[1]= ((str[p]-0x30) * 10) + (str[p+1]-0x30);  //month
//                       PrintNum((long)clkBuf[1]);
                         p += 3;
                         clkBuf[2]= ((str[p]-0x30) * 10) + (str[p+1]-0x30);; //day
//                       PrintNum((long)clkBuf[2]);
                         p += 4;
                         clkBuf[4]= ((str[p]-0x30) * 10) + (str[p+1]-0x30);  //hour

//                       PrintNum((long)clkBuf[4]);
                         p += 3;
                         clkBuf[5]= ((str[p]-0x30) * 10) + (str[p+1]-0x30);  //minute
//                        PrintNum((long)clkBuf[5]);
                          p += 7;
                          TimeZone =  ((str[p]-0x30) * 10) + (str[p+1]-0x30);   //time zone
                          p--;
                          if(str[p] == '-')
                          TimeZone *= -1;
                          if(eTimeZone != TimeZone)
                          eTimeZone = TimeZone;
                              rtc_set_timeAll(clkBuf);

                             SendDebugMsg("\r\n*** Unit clock initialized by Server  ***\r\n\0");

//                             SetAlarmTiming(2);     //set alarm in 2 minutes as WD in case modem failure
//                             EIFR |= (1<<INTF2) ;
//                             ENABLE_CLOCK_INT(); // enable external WD on interrupt2 //Danny
                   //      }

                        for(i=0; i< 7; i++)
                        {
                            if(str[++p] == '}')     //end - no params update in jason string
                            break;
                        }
                        if(i < 7)
                        return 2;   //no update paams
                        return 1;  //need to read update params

  }


//{"60":400,"61":[60,92,50,98],"62":[1125,1250],"70":1}
//210 {"60":400,"61":[0,150,0,150],"62":[1365,4369],"70":1,"41":90}

//char Pharse_SDIparams_struct(char *str, char index)
//{
//      char i,p,chr, cnt;
//      unsigned int val = 0;;
//
//
//   p = index;
//  do
//   {
//        while((str[p] != '"') && (str[p] != '}'))
//        p++;
//
//        if(str[p] != '}')       //p point to '"'
//        {
//            p++;                    //skip " - point to a key val
//            chr = str[p];           //keep the char
//
//            switch (chr)            //check the char
//            {
//                case '0':
//                break;
//
//                case '2':
//                       chr = str[p + 1];  //check second char of key
//                       switch (chr)            //check second char
//                      {
//                          case '0':          //key = 20
//                           SendDebugMsg("Reset Bottle State\r\n\0");
//                              val = 0;
//                              p += 4;       //points to value
//                              val = pStr2Bin (str, p);
//
//                              if(val == 1)
//                               ResetPumpFlags();
//
//                          break;
//                      }
//
//                break;
//
//                case '4':
//                          chr = str[p + 1];  //check second char
//                           val = 0;
//                            p += 4;       //points to value - relay duration
//                           val = pStr2Bin (str, p);
//
//                       switch (chr)            //check the char
//                      {
//                         case '0':
//                         break;
//
//                         case '1':                //key=41
//                               SendDebugMsg("Relay 2 timing setting\r\n\0");
//
//                             #asm("cli")
//                             ePUMP_HEAT_TIME = val ;   //store in eeprom location. temp is first sensor
//                             #asm("sei")
//
//                         break;
//
//                         case 2:
//                                 SendDebugMsg("Relay 1 EN/DIS\r\n\0");
//                                  #asm("cli")
//                                  eRelay1Enabled = val ;   //store in eeprom location. temp is first sensor
//                                  #asm("sei")
//                                  Relay1Enabled = val;
//                         break;
//
//                         case 3:
//                                  SendDebugMsg("Relay 2 EN/DIS\r\n\0");
//                                  #asm("cli")
//                                  eRelay2Enabled = val ;   //store in eeprom location. temp is first sensor
//                                  #asm("sei")
//                                  Relay2Enabled = val;
//                         break;
//                       }
//                break;
//
//                case '6':               //first char of key is 6
//
//                     chr = str[p + 1];  //check second char
//                     switch (chr)
//                    {
//                         case '0':          //key for TEMP_TH
//                               SendDebugMsg("Temperature High Tresh setting\r\n\0");
////                              _putchar1 ('0');
////                               _putchar1 (' ');
//                              val = 0;
//                              p += 4;       //points to value - max temp
//                              val = pStr2Bin (str, p);
//
//                                 #asm("cli")           //val is holding binari value
//                                  MAX_LIMIT[1] = val;   //SDI combo. temp is second sensor
//                                  #asm("sei")
////                              }
//                          break;
//
//                         // {"60":500,"61":[0,150,0,150],"62":[750,875],"70":0,"41":90}
//
//                          case '1':   //   key for  PH_TH
//
//                              p += 5;       //points to first value - min ph
//                              cnt = 0;
//                              val = 0;
//                         do{
//
//                               val = pStr2Bin (str, p);
//                               switch (cnt)
//                              {
//                                  #asm("cli")
//                                  case 0:
//                                      MIN_LIMIT[2] = val;
////                                       SendDebugMsg("MIN_LIMIT[2]\r\n\0");
////                                       PrintNum(iLastMsr[val]);
//                                  break;
//                                   case 1:
//                                      MAX_LIMIT[2] = val;
////                                       SendDebugMsg("MAX_LIMIT[2]\r\n\0");
////                                        PrintNum(iLastMsr[val]);
//                                  break;
//                                   case 2:
//                                      PUMP_MIN_LIMIT[2] = val;
////                                       SendDebugMsg("PUMP_MIN_LIMIT[2]\r\n\0");
////                                        PrintNum(iLastMsr[val]);
//                                  break;
//                                   case 3:
//                                      PUMP_MAX_LIMIT[2] = val;
////                                       SendDebugMsg("PUMP_MAX_LIMIT[2]\r\n\0");
////                                        PrintNum(iLastMsr[val]);
//                                  break;
//
//                                  default:
//                              }
//                                #asm("sei")
//                                val = 0;
//                                while((str[p] != ',') && (str[p] != ']'))  //look for next value
//                                p++;
//                                p++;    //point to first char of next value
//                               cnt ++;
//
//                           }while (cnt < 4);
//                         break;
//
//                         case '2':     //EC_TH
//
//                          SendDebugMsg("EC High Tresh setting\r\n\0");
//                              p += 5;       //points to first value - max ec
//                              cnt = 0;
//                              val = 0;
//                         do{
//
//                                 val = pStr2Bin (str, p);
//                                 switch (cnt)       //parameter index
//                                {
//                                      #asm("cli")
//                                      case 0:
//                                          MAX_LIMIT[0] = val;
//
//                                      break;
//
//                                      case 1:
//                                          PUMP_MAX_LIMIT[0] = val;
//
//                                      break;
//
//                                      default:
//                                  }
////                              }
//                                #asm("sei")
//                               val = 0;
//                            while((str[p] != ',') && (str[p] != ']'))  //look for next value
//                            p++;
//                            p++;    //point to first char of next value
//
//                           cnt ++;
//                           }while (cnt < 2);
//                         break;   //case 2
//                   //???     break;
//
//                         case '3':      //WATER_LEVEL
//                         break;
//                    }
//
//                break;  //case 6
//
//                 case '7':
//
//                       chr = str[p + 1];  //check second char of key
//                       switch (chr)            //check second char
//                      {
//
//                          case '0':          //key = 70
//                           SendDebugMsg("Get BackToNormal Alert param\r\n\0");
//                              val = 0;
//                              p += 4;       //points to value
//                              val = pStr2Bin (str, p);
//                              if(val == 1)
//                              {
//                                   eBackToNormalAlertEnabled = 1 ;     //stasus byte  ???
//                                    BackToNormalAlertEnabled = TRUE ;
//                              }
//                              else
//                              {
//                                      eBackToNormalAlertEnabled = 0;     //stasus byte  ???
//                                      BackToNormalAlertEnabled = FALSE;
//                              }
//
//                            break;
//
//                            case '2':     //COMMUNICATION params
//
//                              SendDebugMsg("Communication params setting\r\n\0");
//                                  p += 5;       //points to first value - max ec
//                                  cnt = 0;
//                                  val = 0;
//                               do{
//
//                                 val = pStr2Bin (str, p);
//                                 switch (cnt)       //parameter index
//                                {
//                                      #asm("cli")
//                                      case 0:
//                                          eStartConnectionH = val;
//                                      break;
//
//                                      case 1:
//                                          eConnectionInDay = val;
//                                      break;
//
//                                       case 2:
//                                          eConnectIntervalH = val;
//                                       break;
//
//
//                                  }
//
//                                #asm("sei")
//                               val = 0;
//                               while((str[p] != ',') && (str[p] != ']'))  //look for next value
//                                p++;
//
//                                p++;    //point to first char of next value
//                               cnt ++;
//                           }while (cnt < 3);
//                         break;   //case 2
//
//                        }
//                 break;
//
//              default:
//            }
//         }
//   }   while(str[p] != '}');     //while not end of string
//
//   return 1;
//}
 //make params as binary
 int pStr2Bin (char *str, unsigned int p)
{
        unsigned int val = 0;
        char MINUS = 0;

          while((str[p] != ',') &&  (str[p] != ']') && (str[p] != '}')) // , or ] or } are end of value','
          {
                if(str[p] == '-')
                {
                   MINUS = 1;
                }
                else
                {
                    val *= 10;
                    val += (str[p] - 0x30);
                }
                  p++;
          }
          if(MINUS == 1)
          return (val * -1);
          return val;

}

//=======================================================================================

 void CloseSuccessfulCom(void)
 {        
 char i;    
 
  SendDebugMsg("\r\nclose successful coounication...\r\n\0 ");
             if( IsAlertNeeded == TRUE)       //aLERT COMMUNICATION ENDED
            {
                  IsAlertNeeded = FALSE;
                  IsAlertActivated = TRUE;

                  FlagsStatus =  eFLAGS_STATUS;
                   SetStatusReg( IS_ALERT_ACTIVATED | IS_ALERT_NEEDED , IS_ALERT_ACTIVATED);
                   eFLAGS_STATUS =  FlagsStatus;
              //     PrintNum((long)FlagsStatus);     //ddddddddeeeeeebbuuuuuuuuuugggggg
                  msrAlertFlag = 0;
                  QuickSensorsCheckNeeded = FALSE;
                  SendDebugMsg("\r\nAlert Activated by Unit....\r\n\0 ");   //Danny for debug
            }


            if(DataSent == TRUE)   //data call? if yes pointers can be set in new location
            {
                 DataPointersMoveNeeded = TRUE;   //- post data OK - move pointers
            }

            if(BackToNormalAlertNeeded == TRUE)
            {
                 BackToNormalAlertNeeded = FALSE;
                 BackToNormalAlertActivated = TRUE;
                 IsAlertActivated = FALSE;
                  WLV_OVF = FALSE;
                 QuickSensorsCheckNeeded = FALSE;
                 FlagsStatus =  eFLAGS_STATUS;
                   SetStatusReg( IS_ALERT_ACTIVATED |  WLV_OVF_FLAG , 0); 
                   
//               #ifdef NETIV_ALERT_UNIT      
//                  POWER_FAILURE = FALSE;                  
//               #endif    
            }

               ModemRepeatCount = 0;
               ModemAgainCount = 0;
               ModemRepeatNeeded = FALSE;

               ComDelayNeeded = FALSE;
               ComFailureCounter = 0;

              bPostAnswered = TRUE;
               bConnectOK = TRUE;

              if(Found200 == FALSE)
              SendDebugMsg("\r\nDebug-Found200 = FALSE.. \r\n\0");

               ConnectedToServer = FALSE;
               ErrorType = 0;
               Found200 = FALSE;
               Found_200 = FALSE;
               ModemAgain =  FALSE;

               eRESTORE_DATA_POINTERS = RELEASE_POINTERS;    //no need for  saved ponters
            
              if(FirmwareUpdateTime == FALSE)
                  modemCurSubTask = SUB_TASK_MODEM_POST_DATA;
               else  modemCurSubTask = SUB_TASK_MODEM_SEND_NOTIFICATION;
              modemCurTask = TASK_MODEM_POST;

              if(ExtendedInfoBlockNeeded == TRUE)  //bloch has sent-reset data
              {   
                  for(i = 0; i < MaxErrs; i++)
                  eErrArray[i] = 0; 
                  
                  eErrIndex = 0; 
                  ExtendedInfoBlockNeeded = FALSE;
              }
             eFLAGS_STATUS =  FlagsStatus; 
            
//              SendDebugMsg("\r\n String lengh: \0 ");
//              PrintNum((long)JasonLengh)
 } 
 
 void SaveErrorState(int ErrType)
{
       //    char Str1[50];
           
           if(eErrIndex < MaxErrs) 
           {   
               if(Illigal_Time_Val == FALSE)
               SensorResult  =  time_in_minutes; 
               else  SensorResult  = 0; 
               SensorResult |= (unsigned int)(ErrType << 12);
                                
            
               eErrArray[eErrIndex++] = SensorResult;        
               ExtendedInfoBlockNeeded = TRUE;  
               
                SendDebugMsg("Debug-Modem Error. save info \r\n\0");                
            
           } 
       //    else SendDebugMsg("Debug-eErrIndex = MaxErrs\r\n\0");   
           ErrorType = 0; 
 }

void ModemReset(void)
{
     SETBIT(PORTB,7);  //put transistor on - puul modem reset
     delay_ms(100);
     CLRBIT(PORTB,7); 
     MODEM_GOT_IP = FALSE;
     
}

void ModemForgetOldCop(void)
{
     SetAlarmTiming(3);
     SendATCmd(AT_CRSM);
     delay_ms(1000);
     SendATCmd(AT_COPS_AUTO);
      delay_ms(1000);

      modemCurTask = TASK_MODEM_INIT;
      modemCurSubTask = SUB_TASK_INIT_MODEM_DELAY;
      ModemResponse = TASK_COMPLETE;
      bCheckRxBuf = FALSE;
}
char ModemTypeDetect(void)
{
  char index, i,k, n;
  char SIM800_Check[] = "AT+CGMM\r\n\0";
  char Telit_Check [] = "AT#CGMM\r\n\0";
  char  nMaxBytesToCheck;


   for(i = 0; i < 9; i++)
   _putchar0(Telit_Check[i]);
   delay_ms(1000);

   k = 0;

do{
   nMaxBytesToCheck = rx0_buff_len;
   if( nMaxBytesToCheck < 5)
   return 0;

   index = -1;
    i = 0;
        do   //check if error
        {
             n = i;

            if((RxUart0Buf[GetBufferIndex(n)] == 'E') &&
            (RxUart0Buf[GetBufferIndex((int)n+1)] == 'R') &&
            (RxUart0Buf[GetBufferIndex((int)n+2)] == 'R'))

            index = GetBufferIndex((int)n +3 );      //phytech protocol..Danny
            i++;
        }
        while ((i < nMaxBytesToCheck - 3) && (index == -1));

        if(index > -1) //error - not telit
        {
              rx0_buff_len = 0;
              for(i = 0; i < 9; i++)
             _putchar0(SIM800_Check[i]);    //check if SIM800
               delay_ms(1000);
              k++;
        }

  }while ((index > -1) && ( k < 2));

      if( k == 2)
      return 0;

    if(k == 0)  //check telit reply
    {
        i = 0;
        do
        {
             n = i;

            if((RxUart0Buf[GetBufferIndex(n)] == 'G') &&  //GL865
            (RxUart0Buf[GetBufferIndex((int)n+1)] == 'L') &&
            (RxUart0Buf[GetBufferIndex((int)n+2)] == '8'))

            index = GetBufferIndex((int)n +2 );
            i++;
        }
        while ((i < nMaxBytesToCheck - 3) && (index == -1));
         if(index > -1) //865 found
         {
             MODEM_TYPE = 2;
             eModemType = MODEM_TYPE;
              SendDebugMsg("Modem Type GL865..\r\n\0");
             return 1;
         }
         else{
               i = 0;
                do
                {
                     n = i;

                    if((RxUart0Buf[GetBufferIndex(n)] == '9') &&
                    (RxUart0Buf[GetBufferIndex((int)n+1)] == '1') &&
                    (RxUart0Buf[GetBufferIndex((int)n+2)] == '0'))

                    index = GetBufferIndex((int)n +3 );      //phytech protocol..Danny
                    i++;
                }
                while ((i < nMaxBytesToCheck - 3) && (index == -1));

                 if(index > -1) //865 found
                 {
                     MODEM_TYPE = 3;
                     eModemType = MODEM_TYPE;
                      SendDebugMsg("Modem Type HE910..\r\n\0");
                     return 1;
                 }
         }
    }
    else  if(k == 1)
    {
          MODEM_TYPE = 1;
          eModemType = MODEM_TYPE;
          SendDebugMsg("Modem Type SIM800..\r\n\0");
          return 1;
    }

    return 0;
 }

void  Close_server_socket(void)
 {         
          char plusplus[] = "+++";  
                  
                  NotMonitored = FALSE;   //showon acreen
                  bNeedToWait4Answer = FALSE;                  
                 delay_ms(100);
                 SendString(plusplus, 3);
                 delay_ms(1700);
                 
                 if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))      
                 SendATCmd(AT_TCP_CLS);
                 else   if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
                 SendATCmd(CIPCLOSE);
                 delay_ms(2000);        
                 #asm ("wdr");
 } 
 
void  server_ReDial(char index)
 {
         
           delay_ms(500);
         
           if(index == 1)      //close socket needed
           { 
                 NotMonitored = FALSE;   //show on acreen   
                 bNeedToWait4Answer = FALSE;    
                 SendString("+++", 3);
                  delay_ms(500);
                 if((MODEM_TYPE == TELIT) || (MODEM_TYPE == HE910))
                 SendATCmd(AT_TCP_CLS);
                 else   if((MODEM_TYPE == SIM800)|| (MODEM_TYPE == DEFAULT_MODEM))
                 SendATCmd(CIPCLOSE);
                 delay_ms(2000);
           } 
           
              SendStartDial();    //re connect
           delay_ms(2000); 
             #asm ("wdr");
 }   
#endif MODEM_MANAGER_C




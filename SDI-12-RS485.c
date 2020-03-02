#include "define.h"
#include "uart.h"
#include <iobits.h >
#include <string.h >

extern bit bCheckRxBuf;
extern char RxUart1Buf[];
extern BYTE rx1_buff_len;
extern int heat_time;
extern  unsigned char tx_wr_index1,tx_counter1;
extern unsigned char rx_wr_index1,rx_rd_index1,rx_counter1;
extern BYTE objToMsr;
extern char rx_buffer1[];
extern bit GPS_data_needed;
extern bit  SDI_EC_MEASURED ;
extern bit  SDI_PH_MEASURED ;
extern bit  SDI_TMP_MEASURED ;
extern eeprom BYTE PortIndex[MAX_SEN_NUM];
extern eeprom unsigned int eTotalRainMeter;

int timer0_count;       //count TOF2 int
bit Measure_Timer_Active;        //flag checked in TOF2
//char gBUF5[5];
int SDI_TEMP_DATA;
int SDI_EC_DATA;
 int SDI_PH_DATA;
int SDI_ORP_DATA;
int SDI_SM_DATA ;
int COMBO_TMP_DATA;
int COMBO_HUMID_DATA;
unsigned long GPSdataArr[4];
char GPS_strig[28] ;

#ifdef WEATHER_STATION_485

unsigned int WS_UV_DATA;
unsigned int WS_WSPEED_DATA;
unsigned int WS_LIFGT_DATA;
unsigned int WS_RAIN_DATA;
bit WS_LOW_BAT_FLAG;

#endif

extern unsigned char bin2bcd (int bin);
extern char getchar2(void);
extern void ShowHexString(unsigned char *message, char Lengh);
extern void UART3_WriteMsg( char *InStr);
extern void _putchar3(char c);
extern void SendDebugMsg3(flash  char *bufToSend);
extern void PrintNum3(long val);

  void Set_UART2_RS485_9600(void);
  void MUX_SDI12_11(void);
  void Uart1MsgN( char *message, char Lengh ); 
  
  

  char PH_Measure[] =  {0x14, 0x06, 0x00 ,0x01, 0x00, 0x1f,0x9b,0x07,0x0d,0x0a,'\0'};     //measure
  char EC_Measure[] =  {0x1e, 0x06, 0x00 ,0x01, 0x00, 0x1f,0x9b,0xad,0x0d,0x0a,'\0'};     //measure
//  char ReadPH_Temp[] = {0x14, 0x03, 0x00 ,0x53, 0x00, 0x02,0x36,0xdf,0x0d,0x0a,'\0'};     //get temp
  char ReadPH_PH[] =   {0x14, 0x03, 0x00 ,0x55, 0x00, 0x02,0xd6,0xde,0x0d,0x0a,'\0'};
  char ReadPH_ORP[] =  {0x14, 0x03, 0x00 ,0x57, 0x00, 0x02,0x77,0x1e,0x0d,0x0a,'\0'};
  char ReadEC_EC[] =   {0x1e, 0x03, 0x00 ,0x55, 0x00, 0x02,0xd6,0x74,0x0d,0x0a,'\0'};
  char ReadEC_Temp[] = {0x1e, 0x03, 0x00 ,0x53, 0x00, 0x02,0x36,0x75,0x0d,0x0a,'\0'};
  char Temp_Humid_Read_Both1[] = {0x01,0x03,0x00,0x00,0x00,0x02,0xc4,0x0b,0x0d,0x0a,'\0'}; //address 1
  char Temp_Humid_Read_Both7[] = {0x07,0x03,0x00,0x00,0x00,0x02,0xc4,0x6d,0x0d,0x0a,'\0'}; //address 7   

#define RS485_DE()  (PORTD.3 = 1)
#define RS485_RE()  (PORTD.3 = 0)

//     Some  SDI12 Commands and responses
//--------------------------------------------------
//Acknowledge Active    a!      a<CR><LF>
//Send Identification   aI!     allccccccccmmmmmmvvvxxx...xx<CR><LF>
//Change Address        aAb!    b<CR><LF>
//Address Query         ?!      a<CR><LF>
//Start Measurement     aM!     atttn<CR><LF>
//Send Data             aD0!    a<values><CR><LF>  like 0+1234<CR\LF>

//CRYSTAL FREQ. 14.7456 MHz
#define BAUD1200()     (UBRR2L = 0xFF,UBRR0H=0x02)
#define BAUD19200()   (UBRR2H=0x00,UBRR2L = 0x2F)
#define BAUD9600()    (UBRR2H=0x00,UBRR2L = 0x5F)

//void SetComp1Delay(int delay);
 void _MakeBin (char *str, char p, long *val)
{
                         
          *val *= 10;           
          *val += (str[p] - 0x30);                        
          return ;          
}

void Set_UART2_485_9600(void)
{
       bCheckRxBuf = FALSE;
       rx1_buff_len = 0;
       rx_counter1 = 0;   
       
   //   Set_UART2_RS485_9600();   //set 9600  
      // USART2 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART2 Receiver: On
// USART2 Transmitter: Off
// USART2 Mode: Asynchronous
// USART2 Baud Rate: 9600
UCSR2A=(0<<RXC2) | (0<<TXC2) | (0<<UDRE2) | (0<<FE2) | (0<<DOR2) | (0<<UPE2) | (0<<U2X2) | (0<<MPCM2);
UCSR2B=(1<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);
UCSR2C=(0<<UMSEL21) | (0<<UMSEL20) | (0<<UPM21) | (0<<UPM20) | (0<<USBS2) | (1<<UCSZ21) | (1<<UCSZ20) | (0<<UCPOL2);
UBRR2H=0x00;
UBRR2L=0x5F;

       MUX_SDI12_11();
       RS485_DE();      //485 RE\     
       delay_ms(25);
       #asm("sei")
}
void Set_UART2_485_9600_WS(void)   //no tx- only rx
{
       bCheckRxBuf = FALSE;
       rx1_buff_len = 0; 
       rx_counter1 = 0;
               
   	UCSR2A=(0<<RXC2) | (0<<TXC2) | (0<<UDRE2) | (0<<FE2) | (0<<DOR2) | (0<<UPE2) | (0<<U2X2) | (0<<MPCM2);
    UCSR2B=(1<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (0<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);
    UCSR2C=(0<<UMSEL21) | (0<<UMSEL20) | (0<<UPM21) | (0<<UPM20) | (0<<USBS2) | (1<<UCSZ21) | (1<<UCSZ20) | (0<<UCPOL2);
    UBRR2H=0x00;
    UBRR2L=0x5F;
       
        MUX_SDI12_11();
   //    RS485_DE();      //485 RE\ 
        RS485_RE();
        delay_ms(15);
       #asm("sei")
}



void Set_UART2_SDI12(void)
{
      // USART1 initialization
// Communication Parameters: 7 Data, 1 Stop, Even Parity
// USART1 Receiver: On
// USART1 Transmitter: On
// USART1 Mode: Asynchronous
// USART1 Baud Rate: 1200

UCSR2A=(0<<RXC2) | (0<<TXC2) | (0<<UDRE2) | (0<<FE2) | (0<<DOR2) | (0<<UPE2) | (0<<U2X2) | (0<<MPCM2);
UCSR2B=(1<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);
UCSR2C=(0<<UMSEL21) | (0<<UMSEL20) | (1<<UPM21) | (0<<UPM20) | (0<<USBS2) | (1<<UCSZ21) | (0<<UCSZ20) | (0<<UCPOL2);  //!!!!!special setting for SDI -set back to 0x06 after SDI ends
UBRR2H=0x02;
UBRR2L=0xFF;  //1200

//    UCSR2C = 0x24;       //7 Data, 1 Stop, Even Parity
//    BAUD1200();       //1200
//    UCSR2B = 0x98;       //enable uart1 as configed

}

void Set_UART2_DEFAULT(void)  //19200
{
//  DISABLE_UART2();
  UCSR2A=(0<<RXC2) | (0<<TXC2) | (0<<UDRE2) | (0<<FE2) | (0<<DOR2) | (0<<UPE2) | (0<<U2X2) | (0<<MPCM2);
 UCSR2B=(1<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);
 UCSR2C=(0<<UMSEL21) | (0<<UMSEL20) | (0<<UPM21) | (0<<UPM20) | (0<<USBS2) | (1<<UCSZ21) | (1<<UCSZ20) | (0<<UCPOL2);
 UBRR2H=0x00;
 UBRR2L=0x2F;
 //     ENABLE_UART2();
     delay_ms(1);
}

void Set_UART2_9600(void)
{
   DISABLE_UART2();
  UCSR2A=(0<<RXC2) | (0<<TXC2) | (0<<UDRE2) | (0<<FE2) | (0<<DOR2) | (0<<UPE2) | (0<<U2X2) | (0<<MPCM2);
// UCSR2B=(1<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);
 UCSR2B=0x18;     //no rx int !!
 UCSR2C=(0<<UMSEL21) | (0<<UMSEL20) | (0<<UPM21) | (0<<UPM20) | (0<<USBS2) | (1<<UCSZ21) | (1<<UCSZ20) | (0<<UCPOL2);
 UBRR2H=0x00;
 UBRR2L=0x5F;

     delay_ms(1);
}
void Set_UART2_RS485_9600(void)
{
     DISABLE_UART2();
     UCSR2C = 0x06;
     BAUD9600();    //9600
//     PRR_EN_UART1();           // allow uart1 in PRR
      ENABLE_UART2();
     delay_ms(1);
}

void SetUart2_4800(void)
{
    // USART2 initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART2 Receiver: On
    // USART2 Transmitter: On
    // USART2 Mode: Asynchronous
    // USART2 Baud Rate: 4800
    UCSR2A=(0<<RXC2) | (0<<TXC2) | (0<<UDRE2) | (0<<FE2) | (0<<DOR2) | (0<<UPE2) | (0<<U2X2) | (0<<MPCM2);
    UCSR2B=(1<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);
    UCSR2C=(0<<UMSEL21) | (0<<UMSEL20) | (0<<UPM21) | (0<<UPM20) | (0<<USBS2) | (1<<UCSZ21) | (1<<UCSZ20) | (0<<UCPOL2);
    UBRR2H=0x00;
    UBRR2L=0xBF;
}

void MUX_SDI12_00(void)
{
//        SETBIT(DDRE,2);    //direction output
//       SETBIT(DDRE,3);    //direction output
//        delay_ms(1);
//      CLRBIT(PORTE,2);
//      CLRBIT(PORTE,3);
//       delay_ms(5);

         DDRE.2 = 1;
        DDRE.3 = 1;
        delay_ms(1);
        PORTE.2 = 0;      //MUX a
        PORTE.3 = 0 ;      //MUX b
         delay_ms(5);
}

 void MUX_SDI12_01(void)
{

       SETBIT(DDRE,2);    //direction output
       SETBIT(DDRE,3);    //direction output
        delay_ms(1);
       CLRBIT(PORTE,3);
       SETBIT(PORTE,2) ;
        delay_ms(1);
}
//2560 OK
//mux channel 2 - Set MUX control pins and set PA.1 as input 3st
void MUX_SDI12_10(void)
{
       CLRBIT(DDRF,1);    // in1 as input 3st
       CLRBIT(PORTF,1);

       SETBIT(DDRE,2);    //direction output
       SETBIT(DDRE,3);    //direction output
        delay_ms(1);
       CLRBIT(PORTE,2);
       SETBIT(PORTE,3) ;
        delay_ms(1);

}

 void MUX_SDI12_11(void)
{

       SETBIT(DDRE,2);    //direction output
       SETBIT(DDRE,3);    //direction output
        delay_ms(1);
       SETBIT(PORTE,2);
       SETBIT(PORTE,3) ;
        delay_ms(1);
}

//send BREAK command
void SDI12_BREAK(void)
{
         DISABLE_UART2();     //get control on pin TX1
         SETBIT(DDRC,7);          //3st buffer DIR pin as output
         SETBIT(DDRH,1);        //TX2 as output
         SET_SDI12_TX_DIRECTION();       //set pin- enable TX dir - sensor see high signal

         CLRBIT(PORTH,1);        //BREAK signal- low with inverter ahead -sensor see high
         delay_ms(15);       //BRAK duration
         SETBIT(PORTH,1);       //end of BREAK

         delay_ms(5);        //MARKING duration

}


void SDI12_ACK_Request(char add)
{
   //  char SDI12_ACK[] = "x!";

       _putchar1(add + 0x30);
       _putchar1('!');
      while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);
      delay_ms(9);
}

void SDI12_MSR(char add)
{

      char MSR[]= "xM!";
     char k;
    //    char MSR[]= "x?!";

       UCSR2B = 0x18;
        k = add + 0x30;
      _putchar1(k);
      _putchar1(MSR[1]);
      _putchar1(MSR[2]);
     while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);
     delay_ms(9);
      UCSR2B = 0x98;
}

void SDI12_NewAdd(char OldAdd, char NewAdd)
{

      char MSR[]= "xAx!";
     char k,j;


       UCSR2B = 0x18;
        k = OldAdd + 0x30;
        j = NewAdd + 0x30;
      _putchar1(k);
      _putchar1(MSR[1]);
      _putchar1(j);
      _putchar1(MSR[3]);
     while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);
     delay_ms(9);
      UCSR2B = 0x98;
}

void GET_SDI12_DATA(char add, char type )
{

   //  char DATA[]= "xDx!";
 //    char index;                //which data of multi data sensor - 1 for temp
//
//      _putchar1(add + 0x30);
//      _putchar1('D');
//      _putchar1(type + 0x30);
//      _putchar1('!');
//     while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);
//     delay_ms(10);
//
//

     char DATA[]= "xDx!\0";

      DATA[0] = add+0x30;
      DATA[2] = type+0x30;
      PCmessage(DATA);

     while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);
     delay_ms(10);

}


void GET_SDI12_ACK(char add)
{
      char SDI12_ACK[] = "x!";

      _putchar1(add + 0x30);
       _putchar1(SDI12_ACK[1]);
      while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);
      delay_ms(9);
}


void SDI12_ID_QUERY(char add)
{
       char SDI12_ID[]= "xI!\0";

        _putchar1(add + 0x30);
        _putchar1(SDI12_ID[1]);
        _putchar1(SDI12_ID[2]);
       while ((UCSR2A & DATA_REGISTER_EMPTY)==0);
       delay_ms(9);
}

void ADDRESS_QUERY(void)
{

   //   char Ask_4Add[]= "?!";

     _putchar1('?');
     _putchar1('!');
     while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);
     delay_ms(9);
}


//get the delay value from sensor response- convert ascii to bin
//format <add><nnn><d><CR><LF>...<add>  last add is optional
//unsigned int CalcMeasureDelay(char *buf, char add)
//{
//     unsigned int value;
//
//     if(buf[7] == add) return 0;    //string include "service request" from sensor. no delay
//     value = (unsigned int)((buf[1] - 0x30) * 100) + (unsigned int)((buf[2] - 0x30) * 10) + (buf[3] - 0x30) ;
//     return value;
//}

//data string example: 0+1044+0.42+23.9<CR><LF>
//index param point to the data that have to be converted to bin

//unsigned int SDI12_DATA_Pharsing(char * str, char index, char devide)
////int SDI12_DATA_Pharsing(char * str, char index)
//{
//      char i, j, k, dot, T;
//      char tData[6];
//    //  unsigned int binVal;
//     int binVal;
//
//      i = j = k = 0;
//    do{
//          while(((str[i] != '+')) && (k < 6))
//          {
//              i++;
//              k++;
//          }
//          if(k < 6)
//          {
//              i++;
//              j++;     //+ or - found. inc j as counte
//              k = 0;
//          }
//          else j = 127;      //not found. get out
//     }
//     while(j < index) ;
//
//     if(j == 127)
//     {
//          return 0x7FFF;   //error
//     }
//
//     j = 0;
//     dot = 0;
//     k = 0;
//     while(((str[i] != '+')) && (str[i] != 0x0D))     //i point to first char of value checked. collect all chars to tData
//     {
//            tData[j] = str[i];
//            if( tData[j] == '.')
//            dot = j ;              //keep pointer to dot in string
//            i++;
//            j++;
//            k++;                //count chars
//     }     //now tData holds the value in ascii
//
//             for (T = 0; T <= j ; T++)    //debuggggggggggggggggggggggggggg
//                 _putchar1 (tData[T]);
//
//     j = 0;
//     binVal = 0;
//     while((tData[j] != '.') && (k > 0))
//     {
//             binVal = (binVal * 10) ;
//             binVal += (tData[j] - 0x30);
//             k--;
//             j++;
//             if(tData[j] == '.')
//             {
//                 j++;
//                 k = 1;
//
//             }
//              if((k == 1) && (devide == 1))   //devide = 1  for particular sensors
//              binVal /= 2;   //eliminate int overflow
//     }
//
////        DebugMode
////    SendDebugMsg("\r\nSDI12 data= ");
////    PrintNum((long)binVal);
////    #endif DebugMode
//
//      #asm ("wdr");
//     return binVal;
//}

// void BIN2STR5(unsigned int index)
//  {
//
//  unsigned int tmp,tmp1 ;
//  unsigned char tmpBCD;
//
//   tmp = index / 10000;
//  gBUF5[0] =(unsigned char)tmp + 0x30; //most sig digit
//   tmp = index % 10000;
//
//  tmp1 = tmp /1000;
//  gBUF5[1] = (unsigned char)tmp1+ 0x30;
//  tmp1 = tmp % 1000;
//
//  tmp =  tmp1/100;
//  gBUF5[2] = (unsigned char)tmp + 0x30;
//  tmp =  tmp1 % 100;
//
//  tmpBCD = (unsigned char)bin2bcd(tmp);
//
//  gBUF5[3] = ((tmpBCD >> 4) & 0x0f) +0x30;
//  gBUF5[4] = (tmpBCD  & 0x0f) +0x30;
// }

//char SDI12_Measure(void /*char sens_address*/)
//{
//
//    char buf[]= "0M!\0";       //sensor address is 0 (default)
//    char  buf1[] = "0D!\0";
//    unsigned int delayVal;
//
////    buf[0] = sens_address;    //address the desired sensor
//    bCheckRxBuf = FALSE;      //reset flag for response detection
//     ENABLE_UART2();           //must be set to SDI12 mode
//    delay_ms(1);
//    PCmessage(buf);           //send command to sensor
//    SET_SDI12_RX_DIRECTION();    //wait for response- put 3st buffer as input
//    delay_ms(100);
//    if( bCheckRxBuf == TRUE)
//    {
//         bCheckRxBuf = FALSE;
//
//         //  Check_SDI12_Response();                //to be writen
//         delayVal = CalcMeasureDelay(RxUart1Buf);  //sensor response include measurement time in sec
//         delay_ms(1000 * delayVal);                // delay as equired
//         SET_SDI12_TX_DIRECTION();
//         PCmessage(buf1);                          //send D command to get the data - yet just show it on PC =test
//         SET_SDI12_RX_DIRECTION();
//         delay_ms(100);
//         Set_UART1_Regular();                     //prepare to send string to PC
//          if( bCheckRxBuf == TRUE)                //data string received
//         {
//             LED1_ON;                         //led  debugggggggg
//             RxUart1Buf[rx1_buff_len] = '\0';     //mark end of buffer
//             SendDebugMsg("data: \0");
//             PCmessage(RxUart1Buf);               //show response on screen for debuging
//             return 1;
//         }
//    }
//    else
//    {
//         SendDebugMsg("No Response for measure command\r\n\0");
//    }
//    return 0;
//}


//    raw text to be removed
//      MUX_SDI12_10();
//      SDI12_BREAK();
//      Set_UART2_SDI12();
//      SET_SDI12_TX_DIRECTION();
//      delay_ms(1);
//      SDI12_MSR(Sens_Add);      //kando sensor add
//      SET_SDI12_RX_DIRECTION();    //RX
//   //  delay_ms(300);
//
//    // if(rx_counter1)
//     while(!RX1_BUF_NOT_EMPTY);      //wait until sensor respose in buffer
//     {
//         RX1_BUF_NOT_EMPTY = FALSE;
//         PORTC.7 = 1 ;      //led
//         MUX_SDI12_00();
//         Set_UART2_DEFAULT();
//         delay_ms(1);
//         for (i = 0; i < rx_counter1 ; i++)
//         putchar1 (rx_buffer1[i]);
//         rx_counter1 = 0;
//         rx_wr_index1 = 0;
//         PORTC.7 = 0 ;      //led
//
//     //   delay_ms(1000);
//         MUX_SDI12_10();
//        Set_UART2_SDI12();
//   //    SDI12_BREAK();
//        PORTA.6 = 0;
//        delay_ms(1);
//        SDI12_DATA(3,0);
//        PORTA.6 = 1;       //RX
//        delay_ms(350);
//              //if(rx_counter1)
//               if(RX1_BUF_NOT_EMPTY)
//             {
//                 PORTC.7 = 1 ;      //led
//                 MUX_SDI12_00();
//                 Set_UART1_Regular();
//                 delay_ms(1);
//                 for (i = 0; i < rx_counter1 ; i++)
//                 putchar1 (rx_buffer1[i]);
//                 rx_counter1 = 0;
//                  rx_wr_index1 = 0;
//                  PORTC.7 = 0 ;      //led

void SDI_MakeBin (char *str, char p, long *val)
{

              *val *= 10;
              *val += (str[p] - 0x30);

         return ;
}

//PH and TEMP values are mult by 10 to have decimal point in it
//EC data with no decimal point and divided by 4

//EC data -   3+24.874+2684.9+1.4391+1552.4+0.0000
//PH data-   2+26.600+9.2293+40.908+0.0000+0.0000
//5TE data - 0+3789+0.64+27.0
//EC address = 3
//PH address = 2
//5TE address = 0
unsigned int SDI_Str2Bin (char *str,char index, char s_add)
{
        long val;
        unsigned char type, p, count;
        char PointFound = FALSE;
        char MINUS = FALSE;

        type = 0;
        p = 0;
        val = 0;
        count = 0;

//
//               while(p < MAX_RX_BUF_LEN - 15)
//               {
//                 if(( str[p] == s_add) && (( str[p + 1] == '+')|| ( str[p + 1] == '-')))
//                 break;
//                 else
//                  p++;
//               }
//            if (str[p] != s_add )
//            {
//               #ifdef DebugMode
//                      SendDebugMsg("\r\nBad data string.....\r\n\0 ");
//                      #endif DebugMode
//                return (int)-1;
//            }
//       SendDebugMsg("\r\nSDI_Str2Bin..\r\n\0 ");

          while((p < MAX_RX_BUF_LEN - 3) && (type < index))
          {
               if(( str[p] == '+') || ( str[p] == '-'))  //find  + of indexed data
               type++;
               p++;

          }

          if(type == index)   //p point to first char to be collected
          {
               if( str[p - 1] == '-')
               {
                   MINUS = TRUE;            //ORP mught be negativ

               }


              if(s_add == SDI_EC_ADDRESS)     //ponsel sensor
              {
                  if(index == 2)  //EC data. ignor decimal point
                  {
                        while((str[p] != '.') && (count < 6))  //colect up to decimal point
                        {
                              SDI_MakeBin(str, p, &val);
                              p++;
                              count++;
                         }
                         val /=8;      //adjust for memory size
//                          SendDebugMsg("\r\nSDI_EC data read..\r\n\0 ");
                  }
                  else if(index == 1)  //TMP data . Collect one digit beyond  decimal point
                  {
                       while(PointFound == FALSE) //
                       {
                             if((str[p] == '.') && (count < 5))
                             {
                                     p++;          //skip decimal point
                                      SDI_MakeBin(str, p, &val);
                                     PointFound = TRUE;   // decimal point found - flag it to get out of loop
                              }
                              else
                              {
                                      SDI_MakeBin(str, p, &val);
                                     p++;
                              }
                              count++;
                        }
                    }
                }

               else if(s_add == SDI_PH_ADDRESS)   //all data (PH,Temp,ORP) has one decimal point in values. treated similar
               {

                     while((PointFound == FALSE) && (count < 5))  //
                     {

                                  if(str[p] == '.')
                                 {
                                          p++;          //skip decimal point   
                                          if((str[p] < 0x30) || (str[p] > 0x39))
                                          str[p] = '0';
                                          SDI_MakeBin(str, p, &val);

                                         PointFound = TRUE;   // decimal point found - flag it to get out of loop
                                  }
                                  else
                                  {
                                          SDI_MakeBin(str, p, &val);
                                         p++;
                                  }
                           count++;
                     }

               }
                else if(s_add == SDI_5TE_ADDRESS)
               {

                   //   SendDebugMsg("\r\n5TE Values Measured...\r\n\0 ");

                     while((PointFound == FALSE) && (count < 3))//
                    {

                             if(str[p] == '.')
                             {
                                     p++;          //skip decimal point
                                 //    PrintNum((long)str[p]);
                                      SDI_MakeBin(str, p, &val);
                                    if(index == 2)     //EC - one mor digit
                                    {
                                         p++;
                                      //    PrintNum((long)str[p]);
                                         SDI_MakeBin(str, p, &val); //added 050616. two digits after dec  point

                                    }
                                     PointFound = TRUE;   // decimal point found - flag it to get out of loop
                              }
                              else
                              {
                                   //    PrintNum((long)str[p]);
                                      SDI_MakeBin(str, p, &val);
                                       p++;
                              }
                              count++;
                     }
                    if(index == 2)  // ec
                    {   // ec
                       //   val = (int)(val * 100);   //make it uS/cm
                         val = (int)(val * 10);   //make it uS/cm  //added 050616. two digits after dec  point-  if Ds/m, no need 090120
                          SendDebugMsg("\r\n5TE EC raw Value = \0");
                           PrintNum((long)val);
                    }
 //                         PrintNum((long)val);


                }
              if(MINUS == TRUE)
              {
                     val = (int)val * -1;     //make it negativ value
                     return (int)val;
                      #ifdef DebugMode
                      SendDebugMsg("\r\nNegative Value Measured...\r\n\0 ");
                      #endif DebugMode
              }
               return (int)val;
            }
                  else return (int)0x8000;


}
////PH and TEMP values are mult by 10 to have decimal point in it
////EC data with no decimal point and divided by 4
//
////3+24.874+2684.9+1.4391+1552.4+0.0000
////PH data-2+26.600+9.2293+40.908+0.0000+0.0000
////5TE data - 0+3789+0.64+27.0
////EC address = 3
////PH address = 2
////5TE address = 0
//unsigned int SDI_Str2Bin (char *str,char index, char s_add)
//{
//        long val;
//        unsigned char type, p;
//        char PointUsed = FALSE;
//
//        type = 0;
//        p = 0;
//        val = 0;
//
//          while((p < rx1_buff_len) && (type < index) && ( str[p] != 0x0D))
//          {
//               if( str[p] == '+')   //find  + of right data
//               type++;
//               p++;
//           //    _putchar1(type + 0x30);
//          }
//          if(type == index)   //p point to first char to be collected
//          {
//              while((str[p] != '+') && (PointUsed == FALSE) && (str[p] != 0x0D) && (p < rx1_buff_len)) //
//             {
//
//                 if(str[p] == '.')
//                 {
//                     if(!DivNeeded)   //not EC value checked. PH or temp need decimal point
//                    {
//                         p++;          //skip decimal point
//                         val *= 10;
//                         val += (str[p] - 0x30);
//
//                    }
//                   PointUsed = TRUE;   // decimal point found - flag it to get out of loop
//                  }
//                  else
//                  {
//                      val *= 10;
//                      val += (str[p] - 0x30);
//                      p++;
//                  }
////                  _putchar1('>');
//              }
//               if(s_add == 3)  //
//               val /=8;        //EC data might be to largs for int
//            }
//         return ( int)val;
//
//}
//data string expected:
//3+25.139+2277.4+1.2207+1316.8+0.0000
//3+25.240+2924.4+1.5675+1690.9+0.0000<><>    - 38 bytes
//EC address = 3
//PH address = 2
//5TE address = 0
 int SD12_Sensors_Read(char sens_add )
{
    char s_add, k ;
     unsigned int  tData, tECdata1, tECdata2;
  //   char test[] = "3+25.139+2277.4+1.2207+1316.8+0.0000\r\n\0 ";

 // SendDebugMsg("Read SDI-12 sensor.\r\n\0");

     #asm ("wdr")
     s_add =  sens_add;

     bCheckRxBuf = FALSE;   //RX flag
     k = 0;
//     j = 0;

        MUX_SDI12_10();
//       do{

            SET_SDI12_RX_DIRECTION();  //prepare data line for break
              delay_ms(400);

           SDI12_BREAK();         //send the BREAK  ignal

            MUX_SDI12_10();        //if sensor connected to input 2
             Set_UART2_SDI12();     //set uart to SDI12 configuration + enable uart
             delay_ms(1);
                                       //LOOP UNTIL SENSOR RESPOND- TESTED SENSOR WAS NOT ESPOND SOME TIMES -CHANGE!
              rx1_buff_len = 0;      //counter of data received by uart1
             rx_counter1 = 0;

               timer0_count = 35;   //?? 350 mS  max for next do loop
              Measure_Timer_Active = TRUE;  //for timer2 operation
            
//-----------------------------------------------------
            //  SDI12_NewAdd(0,3);
            //  ADDRESS_QUERY();
          //  GET_SDI12_DATA(s_add, 0);
//------------------------------------------------------
              SDI12_MSR(s_add);      //measure command   
   
              SET_SDI12_RX_DIRECTION();      //RX  
              ENABLE_SDI_TIMER0();          //3600 Hrz clock
           //    ENABLE_RX_INT_UART2();
          //  while((timer0_count > 0)) ;     //wait for sensor response  
                
            while((bCheckRxBuf == FALSE) && (timer0_count > 0)) ;     //wait for sensor response
               DISABLE_TIMER0();            //disable timer0
               Measure_Timer_Active = FALSE;  //for timer2 operation
               SET_SDI12_TX_DIRECTION();
               
//         }
//         while ((rx1_buff_len <5) && (k < 1));   //send measure command -wait for XnnnY<cr><lf>


        //-----------------------------------------------------
        
       if(rx1_buff_len <5)
        {
                MUX_SDI12_00();
                Set_UART2_DEFAULT();  
              _putchar1('?');
               return 0;           //500-
        } 
        
//
//          rx_buffer1[rx_counter1]  = '\0';
//         PCmessage(rx_buffer1);     //SEND DATa STRING TO MONITOR
   //------------------------debug----------------------------

         if(s_add == SDI_5TE_ADDRESS)  //for sensor 5TE  version 6.0x
         {
                SET_SDI12_RX_DIRECTION();  //prepare data line for break
                delay_ms(10);
               SDI12_BREAK();         //send the BREAK  ignal
                MUX_SDI12_10();        //if sensor connected to input 2
               Set_UART2_SDI12();     //set uart to SDI12 configuration + enable uart
                delay_ms(1);
         }
         //  delay_ms(5);        //1000

            bCheckRxBuf = FALSE;
            rx1_buff_len = 0;
            rx_counter1 = 0;
            timer0_count = 100;     //650 mS
             Measure_Timer_Active = TRUE;  //for timer2 operation
            SET_SDI12_TX_DIRECTION();
            delay_ms(1);
           
            GET_SDI12_DATA(s_add, 0);     //call for data  -show all  
           
            SET_SDI12_RX_DIRECTION();        //RX
             ENABLE_SDI_TIMER0();
         
           while((bCheckRxBuf == FALSE) && (timer0_count > 0)) ; //wait to get response or timeout 
        
            DISABLE_TIMER0();
            Measure_Timer_Active = FALSE;  //SDI not in use now   
                    
            MUX_SDI12_00();
            Set_UART2_DEFAULT();
            delay_ms(5);

            if(bCheckRxBuf == FALSE) 
//              if( rx1_buff_len < 4)
            {
                _putchar1('?');
                _putchar1('!');
                return 0;           //500-no answer from sensor
            }
         //   else   SendDebugMsg("Read SDI-12 data OK..\r\n\0");


   //------------------debug----------------------
    RxUart1Buf[rx_counter1]  = '\0';
    PCmessage(RxUart1Buf);     //SEND DATa STRING TO MONITOR  
    
     PrintNum((long)( rx1_buff_len));

    if( rx1_buff_len < 10)
    {
          SendDebugMsg("Read SDI-12 Failure..\r\n\0");
          
           return 0;           //500-no answer from sensor
    }
   //-----------------------------------------------

    //Pharsing the data string  - new func?
                 if(s_add == SDI_EC_ADDRESS)
                 {
                     if(rx1_buff_len > 10)     //data string long enough
                     {
                         SDI_TEMP_DATA = SDI_Str2Bin (RxUart1Buf,1, SDI_EC_ADDRESS);   //global var
                         SDI_EC_DATA = SDI_Str2Bin (RxUart1Buf,2, SDI_EC_ADDRESS);
                         SDI_EC_MEASURED = TRUE ;
                         SDI_TMP_MEASURED = TRUE ;

//                          PrintNum((long)SDI_TEMP_DATA);
//                            PrintNum((long)SDI_TEMP_DATA) ;

                     }
                      else   //error reading data
                      {
                          SDI_EC_DATA = (int)0x8000;    //error
                          SDI_TEMP_DATA = (int)0x8000;
                      }
                      if( SDI_EC_DATA  < 0)
                       return 0;

                 }
                  else  if(s_add == SDI_PH_ADDRESS)    //real data  0+34.85+1.89+25.9
                 {
                       if(rx1_buff_len > 10)     //data string long enough
                       {
                             SDI_PH_DATA = SDI_Str2Bin (RxUart1Buf,2, SDI_PH_ADDRESS);
                             SDI_ORP_DATA = SDI_Str2Bin (RxUart1Buf,3, SDI_PH_ADDRESS);

                             SDI_PH_MEASURED  = TRUE ;

                            if(SDI_TMP_MEASURED == FALSE)  //need temp as well
                            {
                                  SDI_TEMP_DATA = SDI_Str2Bin (RxUart1Buf,1, SDI_PH_ADDRESS);
                                   SDI_TMP_MEASURED  = TRUE ;
                            }

 //                          PrintNum((long)SDI_PH_DATA);
//                            PrintNum((long)SDI_TEMP_DATA) ;

                       }
                         else   //error reading data
                      {
                          SDI_PH_DATA = (int)0x8000;
                          SDI_ORP_DATA = (int)0x8000;
                            return 0;
                      }
                     
                     

                 }
                   else  if(s_add == SDI_5TE_ADDRESS)
                 {
                        k = 0;
                      while(( RxUart1Buf[k] != '0') && ( RxUart1Buf[(int)k + 1] != '+') && (k < (rx1_buff_len - 15)))
                      {
                        k++;
                      }
//                    if(RxUart1Buf[k] == '0')
//                   {
//                      _putchar1('\r');
//                      _putchar1('\n');
//                      for(i = k ; i < rx1_buff_len - k; i++)
//                      _putchar1(RxUart1Buf[i]);     //SEND DATa STRING TO MONITOR
//                   }
//                  else  _putchar1('#');

                       if(rx1_buff_len > 12)     //data string long enough
                       {
                            //real data  0+34.85+1.89+25.9
                             SDI_EC_DATA = SDI_Str2Bin (RxUart1Buf,2, SDI_5TE_ADDRESS);
                             SDI_TEMP_DATA = SDI_Str2Bin (RxUart1Buf,3, SDI_5TE_ADDRESS);   //global var
                             SDI_SM_DATA = SDI_Str2Bin (RxUart1Buf,1, SDI_5TE_ADDRESS);   //global var

                             SDI_EC_MEASURED = TRUE ;
                             SDI_TMP_MEASURED = TRUE ;

                             // equ: 80 * ECb / (VWC-4.1)
                       
                         //     tData = ((8 * SDI_EC_DATA ) / ((SDI_SM_DATA/10 + SDI_SM_DATA%10)  - 4.1));  //get value is dS/m mult by 100 
                        
                                tECdata1 = (8 * SDI_EC_DATA ); //ec = 0.27 *100 = 216  
                             //   PrintNum((long)tECdata1) ;
                             //  tECdata2 = (((SDI_SM_DATA/10) + (SDI_SM_DATA % 10)) - 4.1); //
                                tECdata2 = ((SDI_SM_DATA/10) - 4.1); //
                             //   PrintNum((long)tECdata2) ;
                                tData = ((tECdata1 / tECdata2) + (tECdata1 % tECdata2));  //SDI_SM_DATA already mult by 10 -  SDI_EC_DATA  by 100  
                             //   PrintNum((long)tData) ;                                                                                     
                            
//                              PrintNum((long)SDI_EC_DATA) ;
//                               PrintNum((long)SDI_SM_DATA) ;
                             
                               SDI_EC_DATA = (unsigned int)tData;   
                            //   PrintNum((long)SDI_EC_DATA) ;
                             

                         }
                         else   //error reading data
                      {

                                    SDI_EC_DATA = (int)0x8000;    //error
                                    SDI_TEMP_DATA = (int)0x8000;
                                    SDI_SM_DATA = (int)0x8000;

                           return 0;
                      }

                 }
//                 DISABLE_UART0();
//    if( rx1_buff_len > 10)
//    {
//              RxUart1Buf[rx1_buff_len] = '\0';  //debug
//              PCmessage(RxUart1Buf);     //SEND DATa STRING TO MONITOR
//    }
//    else
//    {
//          SendDebugMsg("Read SDI-12 Failure..\r\n\0");
//    }
                 bCheckRxBuf = FALSE;
                 ENABLE_RX_INT_UART2();
                  MUX_SDI12_00();
                  Set_UART2_DEFAULT();
     return 1;
}




//=========================================================================================
// #ifdef RS485_ACTIVE

 //Convert 4 bytes floating point value.
 // returns decimal value mult by 10.
int FP2DEC(unsigned char *FPval, char Saddress, char index )
{
  char MINUS=0;
  unsigned int i;
  unsigned int tmp1, tmp2, frac;
  unsigned int mask, m;
  int data = 0;
  int  count,exp_pwr;
  unsigned char fp[4];
  bit LongNum = 0;
  char str[30];

// char fp[] = {0x41, 0x66, 0xff, 0xc5};  //test 4476a995

  for(i = 0; i < 4; i++)  //copy to local
 {
      fp[i] = FPval[i];
//     _putchar1(fp[i]);
//      _putchar1(',');

 }

//      SendDebugMsg("\r\nMBUS raw data - \0");
//      ShowHexString(fp,4) ;  //debug


  if(fp[0] & 0x80)
  MINUS = 1;    //study the sign bit

    fp[0] *=2;         //shift integer part left one bit
    if(fp[1] & 0x80)  //check most sig bit at next byte
    fp[0] |= 1;       //add it as the least bit to integer
//    else
//    fp[0] &= 0xFE;

    exp_pwr = (int)fp[0] - 127; //set the power val
    count = exp_pwr;

    if(count < 0)    //if count < 0 data is fraction of 1. ignored..!
    {
       data = -1;
        putchar1('*');
       return data;
    }

    if(count > 14)   //data is not in int range. treat it differently
    {
       LongNum = 1; //flag it. use later
    }


     tmp1 =((int)fp[1] << 8) + fp[2];;
     tmp1 |= 0x8000;
     tmp2 = tmp1;

         if((Saddress == S485_EC_ADDRESS) && (index == 2))  //EC- devide value by 8
         {
               tmp2 = (tmp1/8) + (tmp1 % 8);
               LongNum = 1;
         }
       mask = 0x8000;

   do
   {
      if(( tmp2 & mask) == mask)          //build the integer data value in loop
      {
          m = 2;
         for(i=1; i<count;i++)
          m *= 2;

       //  PrintNum((long)m);
          data = data + (long)m;
      }

        mask /= 2;
        count--;

   } while (count > 0);

  //   PrintNum((long)data);
     if(( tmp2 & mask) == mask)
     data = data + 1;     //least bit

  //  putchar1('*');
  if(LongNum == 0)       //can mult value by 10 and add one decimal digit
   {

        frac = 0;
        count = 6 ;   //4 loops  to have fraction value
        mask /= 2;
        m = 1;

        for(i = 0; i < count; i++)  //now check 4 digits  beyond decimal point
        {
            m *=2 ;
            if( tmp2 & mask)          //build the data in loop
            {
                frac += (1000/m);        //fraction - value mult by 10000 but divided when rounding
            }
            mask /= 2;
        }

      //   PrintNum((long)frac);
      //      putchar1('&');

        while((frac / 10) > 0)    //rounding frac to 1 digit after decimal point
        {
                if((frac % 10) >= 5)
                   frac = frac/10 + 1;
                else
                   frac = frac/10;
               //   PrintNum((long)frac);
        }
        data *= 10;      //no decimal point allowed. we mult by 10 ( micro...)
        data += frac;     //add the decimal point value mult by 10


  }
 //  PrintNum((long)data);
       if(MINUS == 1)    //set sign bit
        data *= -1;
        delay_ms(200);
       SendDebugMsg("Converted to - \0");
       if(LongNum == 0)
       {
            sprintf(str,"%d.%d\r\n\0",data/10,data%10);
            UART_WriteMsg(str);
       }
       else
        PrintNum((long)data);
       return data;   //data is mult by 10 if not too large (EC)

}



int phars_485_data(char *str, char Saddress, char index)
{
      char p,count, dataStr[4];
   int val;
      int CleanData,i;  
      
        p = 0;
        count = 0;
        val = 0;

          while((str[p] != Saddress) && (p < 5))    //find address byte as beginng of string
          {
               p++;
          }
          if(p < 5)    //found string start
          {
//                  SendDebugMsg("\r\nfull string: \0");
//                  ShowHexString(&str[p],9) ;  //debug
                  p++;
                  p++;   //point to data lengh val (4)

                  count = str[p];     //set count as data lengh
                  p++;
                  for(i = 0; i < count; i++)   //collect the data bytes to array
                  {
                     dataStr[i] = str[p++];
                  }

                   if(index == 1)
                   {
                         if(Saddress == S485_EC_ADDRESS)
                         SendDebugMsg("\r\nTemp.\0");
                         else   if(Saddress == S485_PH_ADDRESS)
                         SendDebugMsg("\r\nPH\0");
                           else if(Saddress == TEMP_HUMID_ADDRESS)
                           SendDebugMsg("\r\nTEMP/HUMID raw data =  \0");   
                    }
                    else    if(index == 2)
                    {
                         if(Saddress == S485_EC_ADDRESS)
                         SendDebugMsg("\r\nEC\0");
                         else  if(Saddress == S485_PH_ADDRESS)
                         SendDebugMsg("\r\nORP\0");
                   //      else if(Saddress == TEMP_HUMID_ADDRESS)  //full string: 01 03 04 08 D6 15 19 D6 F1
                   //       SendDebugMsg("\r\nTEMP / HUMID\0");     

                    }
                 
                     if((Saddress == TEMP_HUMID_ADDRESS) && (index == 1))     //temp/humid sensors has 4 bytes- data 2 for each 
                    {       
                           
                           ShowHexString(dataStr,4) ;  //debug                                                                  
                           CleanData = (int)(dataStr[0] * 256);  //collect temp data                        
                           CleanData  += dataStr[1]; 
                        //   PrintNum((long)CleanData); 
                           
                           if(CleanData < 8000)   //80 dgree max
                             COMBO_TMP_DATA = CleanData/10; 
                           else  COMBO_TMP_DATA = 0x8000; 
                           
                            CleanData = (int)(dataStr[2] * 256);  //collect humidity data                      
                            CleanData  += dataStr[3]; 
                             CleanData /= 10;
                                                                                               
                           return CleanData;  //<=============                      
                     }  
                       
                     
                     else 
                     {                            
                         ShowHexString(dataStr,4) ;  //debug 
                         val = FP2DEC(dataStr, Saddress, index);      //make float a decimal value                 return val;                         
                     }    
                

                 val = FP2DEC(dataStr, Saddress, index);      //make float a decimal value                 return val;
                 return val;

            }
            else return 0x8000;   //failed
}

int Read_485_Sensor(char SensAddress)
{
  //MODBUS fixed strings for Ponsel's sensors
//  char PH_Measure[] =  {0x14, 0x06, 0x00 ,0x01, 0x00, 0x1f,0x9b,0x07,0x0d,0x0a,'\0'};     //measure
//  char EC_Measure[] =  {0x1e, 0x06, 0x00 ,0x01, 0x00, 0x1f,0x9b,0x9e,0x0d,0x0a,'\0'};     //measure
//  char ReadPH_Temp[] = {0x14, 0x03, 0x00 ,0x53, 0x00, 0x02,0x36,0xdf,0x0d,0x0a,'\0'};     //get temp
//  char ReadPH_PH[] =   {0x14, 0x03, 0x00 ,0x55, 0x00, 0x02,0xd6,0xde,0x0d,0x0a,'\0'};
//  char ReadPH_ORP[] =  {0x14, 0x03, 0x00 ,0x57, 0x00, 0x02,0x77,0x1e,0x0d,0x0a,'\0'};
//  char ReadEC_EC[] =   {0x1e, 0x03, 0x00 ,0x55, 0x00, 0x02,0xd6,0x47,0x0d,0x0a,'\0'};
//  char ReadEC_Temp[] = {0x1e, 0x03, 0x00 ,0x53, 0x00, 0x02,0x36,0x46,0x0d,0x0a,'\0'};
  char index;
  int data16;

   //     SendDebugMsg("Read RS485 Sensors..\r\n\0");  
    if (SensAddress == TEMP_HUMID_ADDRESS)   //skip ponsel section
    goto SKIP_PONSEL;
    
        LED1_OFF;
       Set_UART2_485_9600();
    //   delay_ms(500);    ??
       timer0_count = 45;   ////17.8 mS per tick
       Measure_Timer_Active = TRUE;  //for timer2 operation
       ENABLE_SDI_TIMER0();          //14400 Hrz clock

       if(SensAddress == S485_EC_ADDRESS)
          Uart1MsgN(EC_Measure, 10 );      //Measure command
       else   if(SensAddress == S485_PH_ADDRESS)
           Uart1MsgN(PH_Measure, 10 );

       RS485_RE();      //set control bit for response 485 RE\
       while((bCheckRxBuf == FALSE) && (timer0_count > 0)) ;     //wait for sensor response
       #asm("cli")

       DISABLE_TIMER0();            //disable timer0
       Measure_Timer_Active = FALSE;  //for timer2 operation

       MUX_SDI12_00();
       Set_UART2_DEFAULT();

//       if(bCheckRxBuf == FALSE)
//       {
//            SendDebugMsg("No RX int response..\r\n\0");
//            return 0;
//       }

        if(rx1_buff_len < 6)  //failure
        {

            #asm("sei")
            SendDebugMsg("\r\nNo valid response from sensor..\r\n\0");
//            PrintNum((long)rx1_buff_len);
            return 0;
        }
//----------------debug only-------------------------------
//       MUX_SDI12_00();
//        Set_UART2_DEFAULT();
//        tx_counter1 = 0;
//        delay_ms(50);
//         putchar1('#');
//      for( i = 0;  i < (rx1_buff_len) ; i++)
//      {
//           putchar1( rx_buffer1[i]);    //debug
//      }

 //----------------temp or PH reading-------------------------- 
 SKIP_PONSEL: 
 
      Set_UART2_485_9600();
      delay_ms(100);    //let sensor complete measurement

       timer0_count = 45;   //?? 35mS  max for next do loop
       Measure_Timer_Active = TRUE;  //for timer2 operation
       ENABLE_SDI_TIMER0();          //3600 Hrz clock

       index = 1;       //data type   
        if (SensAddress == TEMP_HUMID_ADDRESS)  //
       {     
            UCSR1C=6; //8bit, 1 stop bit, no parity 
         //  Uart1MsgN(Temp_Humid_Read_Both1, 10 );                               
            Uart1MsgN(Temp_Humid_Read_Both7, 10 );          
       }        
       else if(SensAddress == S485_EC_ADDRESS)
       {
            Uart1MsgN(ReadEC_Temp, 10 );      //get temp data
       }
       else   if(SensAddress == S485_PH_ADDRESS)
       {
           Uart1MsgN(ReadPH_PH, 10 );
       }
       
      RS485_RE();      //485 RE\

      while((bCheckRxBuf == FALSE) && (timer0_count > 0)) ;     //wait for sensor response
      #asm("cli")

      DISABLE_TIMER0();            //disable timer0
      Measure_Timer_Active = FALSE;  //for timer2 operation

       MUX_SDI12_00();
       Set_UART2_DEFAULT();

        if(rx1_buff_len < 8)  //failure
        {
            #asm("sei")
            SendDebugMsg("Read RS485 data Failure 1..\r\n\0");
   //          PrintNum((long)rx1_buff_len);
           return 0;
        }

//-----------show data string-----------------------------------
//        MUX_SDI12_00();
//        Set_UART2_DEFAULT();
//        tx_counter1 = 0;
//        delay_ms(50);
//
//      for( i = 0;  i < (rx1_buff_len) ; i++)
//      {
//           putchar1( rx_buffer1[i]);    //debug
//      }
 //----------------------------------------------------
    data16 =   phars_485_data(RxUart1Buf,SensAddress, index);

      if(data16 < 0x7FFF)
      {
            #asm("sei")
           SendDebugMsg("Pharsing 485 data failed..\r\n\0");
            return 0;
      }

  //   SendDebugMsg("Read RS485 check point -\r\n\0");

         if(SensAddress == S485_EC_ADDRESS)
         {
              SDI_TEMP_DATA = data16;
           //   SendDebugMsg("\r\nRead Temp - \0");
            //  PrintNum((long)data16);
         }
         else   if(SensAddress == S485_PH_ADDRESS)
         {
                SDI_PH_DATA = data16;
//                 SendDebugMsg("\r\nRead PH - \0");
//                 PrintNum((long)data16);
         }
     else if(SensAddress == TEMP_HUMID_ADDRESS)
     {       
          COMBO_HUMID_DATA = data16;  //temp data set already at the parsing rutine 
         goto SKIP_REST; 
     }  

 //----------PH / EC reading-------------------------------------

     Set_UART2_485_9600();
     delay_ms(100);

     timer0_count = 45;   //?? 35mS  max for next do loop
     Measure_Timer_Active = TRUE;  //for timer2 operation
     ENABLE_SDI_TIMER0();          //3600 Hrz clock

     index  = 2;

        if(SensAddress == S485_EC_ADDRESS)
          Uart1MsgN( ReadEC_EC, 10 );      //get temp data
       else   if(SensAddress == S485_PH_ADDRESS)
           Uart1MsgN(ReadPH_ORP, 10 );

      RS485_RE();      //485 RE\

       while((bCheckRxBuf == FALSE) && (timer0_count > 0)) ;     //wait for sensor response
      #asm("cli")

      DISABLE_TIMER0();            //disable timer0
      Measure_Timer_Active = FALSE;  //for timer2 operation

       MUX_SDI12_00();
       Set_UART2_DEFAULT();

        if(rx1_buff_len < 8)  //failure
        {
            #asm("sei")
            SendDebugMsg("Read RS485 second data Failure..\r\n\0");
           return 0;
        }

//-----------show data string-----------------------------------
//
//      delay_ms(100);
//      for( i = 0;  i < (rx1_buff_len) ; i++)
//      {
//           putchar1( rx_buffer1[i]);    //debug
//      }
 //----------------------------------------------------
    data16 =   phars_485_data(RxUart1Buf, SensAddress, index);

     if(data16 < 0x7FFF)
      {
           SendDebugMsg("Pharsing 485 data failed..\r\n\0");
            return 0;
      }

     if(SensAddress == S485_EC_ADDRESS)
     {
           SDI_EC_DATA = data16 ;// /80;   //adjust to current EC data size . data mult by 10 when arrived

         //  SendDebugMsg("\r\n**Read EC - \0");
         //  PrintNum((long)data16);
     }
     else   if(SensAddress == S485_PH_ADDRESS)
     {
        //  SDI_ORP_DATA = data16/10; 
           SDI_ORP_DATA = data16;
//          SendDebugMsg("\r\nRead ORP - \0");
//          PrintNum((long)data16);
     }
     SKIP_REST:
    return 1;
}


int Read_Scale_TCS60(void)
{
   char i,j,k;
   char dataBuf[6];
   int scaleData;


     k = 0;
     j = 0;
     i = 0;
         MUX_SDI12_00();
         Set_UART2_DEFAULT();
         delay_ms(10);
         SendDebugMsg("read scale..320 Kg MAX\r\n\0");

        if(PortIndex[objToMsr] == 1)  //if scale connected to first port
          MUX_SDI12_01();
         else  if(PortIndex[objToMsr] == 2)  //or second connector
         MUX_SDI12_10();
          ENABLE_SDI_TIMER0();
         Set_UART2_9600();  //no rx int !! getting dataa from scale directly
          timer0_count = 30;   //?? 35mS  max for next do loop
          Measure_Timer_Active = TRUE;  //for timer2 operation
          ENABLE_SDI_TIMER0();          //3600 Hrz clock

         while((( j = getchar2()) != 0xFF) && ( timer0_count > 0));  //wait for FF to begin
         Measure_Timer_Active = FALSE;
         DISABLE_TIMER0();

         if( timer0_count == 0)   //no data read
         {
              Set_UART2_DEFAULT();
              SendDebugMsg("\r\nNo scale data string..!\r\n\0");
              return -1;
         }

         dataBuf[i] = j;                    //ff - than collect 5 more bytes
         for(i = 1; i < 6; i++)
         dataBuf[i] = getchar2();

     //  LED1_ON;

          Set_UART2_DEFAULT();
          MUX_SDI12_00();
          delay_ms(10);

        //  SendDebugMsg("\r\nget response from scale..\r\n\0");
       i =  (dataBuf[2] & 0x80);    // bit 7 = OVF
       if(i == 0)                 //ok
       i =  (dataBuf[2] & 0x40);    //1 =  data stable
       if(i)
       {       i =  (dataBuf[4] & 0x0f) + (((dataBuf[4]>>4) & 0x0f) * 10);  //most
               j =  (dataBuf[3] & 0x0f) + (((dataBuf[3]>>4) & 0x0f) *10);
               k =  (dataBuf[2] & 0x0f)+(((dataBuf[2]>>4) & 0x0f) * 10); //least
    //       PrintNum((long)k);
    //        PrintNum((long)j);

            scaleData =  ((int)i * 10000) + ((int)j * 100) + k ;  //data = 320kg max
            SendDebugMsg("\r\nScale data = \0");
            delay_ms(10);
            PrintNum((long)scaleData);
            LED1_OFF;
             return scaleData;
       }
       SendDebugMsg("\r\nScale data not stable or OVF..\r\n\0");
       return -1;

}
// unsigned int WS_UV_DATA;
//unsigned int WS_WSPEED_DATA;
//unsigned int WS_LIFGT_DATA;
//unsigned int WS_RAIN_DATA;
//24 39 20 62 7E 3E 00 00 00 00 00 00 00 00 6E BD C6

 #ifdef WEATHER_STATION_485  
 
void Pharse_WS_data(unsigned char *Str)
{
    char indx; 
    char BatteryFlag;
    unsigned int temp16,tmp; 
    char msgStr[30]; 
    char str[18];
//  uint32_t temp32;
 
     for (indx = 0; indx < 17; indx++)
    {  
        str[indx] = Str[indx];  //save string  
    } 
    
     sprintf (msgStr,"\rWS DATA:\r\n\0");           //debug
    /* convert HEX to string */
    for (indx = 0; indx < 17; indx++)
    {       
       sprintf (msgStr,"%02X ", str[indx] ); 
       UART_WriteMsg(msgStr);
    }
    sprintf (msgStr,"\r\n\0");
    UART_WriteMsg(msgStr);
    
    // get temperature   
      tmp = (str[3] * 256);    
      temp16 = (tmp & 0x0700);     
      temp16 |= (str[4]);
            
    if( temp16 < 0x7FF)   //0x7FF error value
    {
         COMBO_TMP_DATA = (int)temp16; // - 400 at the server; 
          sprintf (msgStr,"temperature=%d\r\n\0", COMBO_TMP_DATA-400);
         UART_WriteMsg(msgStr); 
    }     
    else 
    {
       COMBO_TMP_DATA = 0x8000;    //error
        SendDebugMsg("Error reading temperature..!..\r\n\0");
    }   
  
     
    // get humidity  
     if( str[5] < 0xFF)  //0xFF error value
    COMBO_HUMID_DATA =  str[5]; 
    else
    COMBO_HUMID_DATA = 0x8000;
    sprintf (msgStr,"humidity=%d\r\n\0",COMBO_HUMID_DATA);
       UART_WriteMsg(msgStr);
       
    // get wind speed 
    temp16 = ((str[3] & 0x10) << 4);
    temp16 |= str[6];
    if( temp16 < 0x1FF)    //0x1FF error value
    WS_WSPEED_DATA = (int)temp16; // data/8*1.12  at the server;  
    else
    WS_WSPEED_DATA = 0x8000;   //error
    sprintf (msgStr,"raw wind Speed=%d\r\n\0", WS_WSPEED_DATA);   
      UART_WriteMsg(msgStr);
          
    // get rainfall
    temp16 = (str[8]* 256);
    temp16 |= str[9];
    WS_RAIN_DATA = temp16;    //real value= temp16 * 0.3 (server job)
    sprintf (msgStr,"raw rain meter-base=%d\r\n\0",WS_RAIN_DATA-eTotalRainMeter );
    UART_WriteMsg(msgStr);  
    sprintf (msgStr,"raw rain base = %d\r\n\0",eTotalRainMeter);
    UART_WriteMsg(msgStr);  
    
      
     // get UV value and convert to index later           
         
    temp16 = (str[10] * 256);
    temp16 |= str[11];
    WS_UV_DATA  = temp16 ; 
    sprintf (msgStr,"uv=%d\r\n\0", WS_UV_DATA);
    UART_WriteMsg(msgStr); 
  
           
    // get low battery flag  
    BatteryFlag =  ((str[3] & 0x08) >> 3);  
    if(BatteryFlag) 
    {
       WS_LOW_BAT_FLAG = TRUE; 
       SendDebugMsg("WS low battery detected..\r\n\0");
    } 
    else 
    {
       WS_LOW_BAT_FLAG = FALSE;  
       SendDebugMsg("WS battery OK..\r\n\0");
    }
  
       
    //    // get wind direction starts from byte 3
//    temp16 = ((str[3] & 0x80) << 1);
//    temp16 |= str[2];
//    WeatherRawData.windDirection = temp16; 
//    printf ("WeatherRawData.windDirection=%d\r", WeatherRawData.windDirection);
     //    // get ligh
//    temp32 = uart_buff[12];
//    temp32 = ((temp32 << 8 ) | str[13]);
//    temp32 = ((temp32 << 8 ) | str[14]);
//    temp32 = ((temp32 << 8 ) | str[15]);
//    WeatherRawData.light = temp32;
//    printf ("WeatherRawData.light=%ld\r", WeatherRawData.light);
//    // get gust speed
//    WeatherRawData.gustSpeed = str[7];
//    printf ("WeatherRawData.gustSpeed=%d\r", WeatherRawData.gustSpeed);   
}



 char Read_Weather_Station_485(void)
 {     
     #define WS_STRING_ID 0x24 
     #define WS_data_lengh 17 
     bit RX_OK;
     char count,count1; 
     int tmpCount;   
     
        #asm("wdr");
  //    SendDebugMsg("Wait for WS data..\r\n\0");
      count = 0; 
      count1 = 0;  
      DISABLE_UART2();  
      delay_ms(50);  
     Set_UART2_485_9600_WS();   //RX only!  
   //  Set_UART2_485_9600();   //RX only!                          
      Measure_Timer_Active = TRUE;  //for timer2 operation
                     
    do{    //loop twice if needed  
         
           rx1_buff_len = 0; 
           bCheckRxBuf = FALSE; 
           RX_OK = FALSE;
           RxUart1Buf[0] = 0;   //init byte       
           timer0_count = 700;   //17 sec       
           ENABLE_TIMER0();          //3600 Hrz clock  
           RS485_RE(); 
             
        //   while((bCheckRxBuf == FALSE) && (timer0_count > 0))     //wait for sensor response 
          while(( RxUart1Buf[0] != 0x24) && (timer0_count > 0));     //wait for sensor response
                 
          DISABLE_TIMER0(); 
                  
        //  if(bCheckRxBuf == TRUE) //rx flag on  
           if( timer0_count > 0) //rx flag on 
           {    
                   delay_ms(50);   //get all data
            //      if((bCheckRxBuf == TRUE) && (RxUart1Buf[0] == WS_STRING_ID) && (rx1_buff_len >= WS_data_lengh)) //is complete rx ok 
                  if((RxUart1Buf[0] == WS_STRING_ID) && (rx1_buff_len >= WS_data_lengh)) //is complete rx ok            
                  {  
                   //  DISABLE_RX_INT_UART2();   //keep data in buffer to be parsed
                     RX_OK = TRUE; 
                          
                  }               
                  else
                  {
                       count+=1;                     
                  }  
           }
           else
           {
                 count1++;                 
           } 
           #asm("wdr");
     }  while((RX_OK == FALSE) && (count < 3) && (count1 < 5)); //loop twice if needed
           
      
    //   DISABLE_TIMER0();            //disable timer0
       Measure_Timer_Active = FALSE;  //for timer2 operation  
  
       MUX_SDI12_00();      //set terminal way
       Set_UART2_DEFAULT(); //set uart 19200
       
       
        if((count == 3) || (count1 == 5))  //rx failure
        {
          //   ENABLE_RX_INT_UART2();
            SendDebugMsg("Read WS data Failure..\r\n\0");
 //        SendDebugMsg("\rcount= \0");     //debug
//        PrintNum((long)count);  
//         PrintNum((long)count1); 
//         PrintNum((long) rx1_buff_len); 
        
           return 0;
        } 
         SendDebugMsg("\rcount= \0");     //debug
        PrintNum((long)count);  
         PrintNum((long)count1); 
         PrintNum((long)tmpCount); 
//        PrintNum((long) rx1_buff_len);    
        
        Pharse_WS_data(RxUart1Buf);   //get the data out of it    
    

        return 1;
 }   
 #endif  
 //  GPS data format  3131.9599N, 03435.0938E   my place
//$GPRMC,174845,V,0000.0000,N,00000.0000,E,0.0,0.0,160316,0.0,W*7B
 //               000..000,,N,0000..000,,E#
char Pharse_GPS_data(char *str)
{
    
      char p;  
      char count, loops, index;  
      long val;  
     
   //   char Str[70]; 
    
      char *ptr;
   
     

//      for(p = 0; str[p] != '\n'; p++)
//      putchar1(str[p]);
      
       p = 0;    
       loops = 0;  
       count = 0;  
       index = 0;  
      
          _putchar3('&'); 
         _putchar3('&');   
           delay_ms(100);   
           
//         while(( str[p] != 'A') && (p < 70) )   //count 2 ','  look for A (valid GPS data)                          
//           p++;   
          
          ptr = strstrf( str, "A,"); 
          if((ptr != NULL) )   //found $GPRMC,084101,A, and full string
          {        
            _putchar3('*'); 
            _putchar3('*'); 
              delay_ms(100); 
                    p = ptr - str;                                                                    
                 //   Buzer_it(1,1, 1);       //just for unit 6                                
                    loops = 0; 
                    p++;             //skip 'A'    0000.0000,N,00000.0000,E
                    do{   
                           count = 0; 
                           val = 0;
                           p++;         //point to location of   data   
                           while((str[p] != '.') && (str[p] != ',') && (count < 6))   //cllt digits up to the dot
                           {       
                                   _putchar3( str[p]);
                                  _MakeBin(str, p, &val);     //build val as bin                                  
                                  
                                   GPS_strig[index++] = str[p];   //collect chars to beffer                             
                                   count++;                              
                                    p++;
                           } 
                          
                           GPSdataArr[loops] = (unsigned long) val;   //store bin data in array 
                           GPS_strig[index++] = str[p];   //p point to dot - add  point  or psik 
                             _putchar3( str[p]); 
                             
                           loops++;  
                           if(loops == 2) 
                           {
                                p += 2;     //skip ,N, 
                                GPS_strig[index++] = 'N';  //add the N
                                GPS_strig[index++] = ',';
                                
                            }                                      
                       }
                    while (loops < 4); 
                    
                     GPS_strig[index++] = 'E';
                     GPS_strig[index++] = '#';    //end of string
                     GPS_strig[index++] = '\0';    //end of string  
                    
                    
                
          } 
               
          else   
          {
                SendDebugMsg3("\r\nFailed pharsing GPS data ..\r\n ");    //failed  
                 return 0;
          }                                         
            
             return 1;            
           
                             
     
  }  
 
 char  ReadSaveGPSSensor(void)
{
    char i, k;
    char count; 
    char tBuf[100] ;
    char *ptr = NULL;
  //  char RxUart1Buf[] = "$GPRMC,084101,A,3131.9595,N,03435.0927,E,0.2,22.1,160316,0.0,W*55\r\n";     //68
      int index;           //             3130.84542,N,03435.69188,E
      bit   MeasureFailure = TRUE;                
                           
      
       SendDebugMsg("\r\nRead GPS data..\r\n ");
        V33_PWR_ON(); 
        delay_ms(500);
       count = 0; 
     
       index = -1 ;
         #asm("sei");  
        bCheckRxBuf = FALSE;    
               
             do{   //1    
          
                //  Set_UART2_485_9600();
                   Set_UART2_485_9600_WS();
                                              
                   timer0_count = 120;   ////10 mS per tick
                   Measure_Timer_Active = TRUE;  //for timer0 operation
                   ENABLE_TIMER0();          //14400 Hrz clock
                
                   while((bCheckRxBuf == FALSE) && (timer0_count > 0)) ;     //wait for sensor response            
                 
                   Measure_Timer_Active = FALSE;;
                   DISABLE_TIMER0();            //disable timer0

                 
                   if (bCheckRxBuf == TRUE)   
                   {    
                       bCheckRxBuf = FALSE; 
                        LED1_ON;
                        delay_ms(30);
                         LED1_OFF;  
                         
                        ptr = strstrf( RxUart1Buf, "$GPRMC");                                               
                        if(ptr != NULL)    //found "$GPRMC"
                        {                                                                                           
                                DISABLE_RX_INT_UART2();             
                         //-------------------------------------------------------------------- 
//                                _putchar3('%'); 
//                                _putchar3('%');     
                               k = ptr - RxUart1Buf;  
                                                 
                               for( i = 0; (RxUart1Buf[k] != '*') && (i < 72) ; i++)                      
                               {
                                      
                                    tBuf[i] =  RxUart1Buf[k];    //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                                   _putchar3(tBuf[i]);    //debug
                                    k++;                            
                               } 
                                _putchar3(RxUart1Buf[k]);                                                           
                            //--------------------------------------------------------------------- 
                                                         
                                ptr = strstrf( tBuf, ",A,"); 
                                if((ptr != NULL) )   //found $GPRMC,084101,A, and full string
                                {  
                                                                                                                                                         
                                    k =  Pharse_GPS_data(tBuf);    //and save  
                                    
                                     _putchar3('@'); 
                                     _putchar3('@');                                         
                                    if((k == 0) || (k == 2))   //failed read GPS                       
                                    { 
                                        GPSdataArr[0] = 0;     //send zero
                                        GPSdataArr[1] = 0;
                                        GPSdataArr[2] = 0;
                                        GPSdataArr[3] = 0;                                                                                                                     
                                   
                                    } 
                                    else //if(k == 1)
                                    {   
                                                                         
                                         MeasureFailure = FALSE ;  
                                         index = 1;      //we have data                                           
                                                                                                                                                                                                                                                  
                                     } 
                                     _putchar3('\r');
                                   PrintNum3((long)GPSdataArr[0]);
                                   PrintNum3((long)GPSdataArr[1]);
                                   PrintNum3((long)GPSdataArr[2]);
                                   PrintNum3((long)GPSdataArr[3]); 
                                   
                                    
                                  // sprintf(tBuf,"GPS  string:\r %s\n\r\0",GPS_strig);   //3130.84420,N,03435.67790,E  26 chats
                                   //UART3_WriteMsg(tBuf);        
                               } 
                                   
                        }
                    }                                                                            
                     else 
                   {  
                     count++;                   
                     delay_ms(1500);                                                       
                   }                          
                     #asm("wdr")
              } 
              while ((index == -1) && (count  < 60));   //do 1                                            
                                               
         Set_UART2_DEFAULT();
          MUX_SDI12_00();   //mux back to monitor
         SendDebugMsg("\r\nRead GPS data ended..\r\n ");  
  
       if( index == 1) 
       {
//                _putchar3('%'); 
//                _putchar3('%');  
                
            return 1;
       }
           
      
     
      return 0;         
}
 




//void SetComp1Delay(int delay)
//
//{
//
//    TCNT1H=0x00;
//    TCNT1L=0x00;
//    OCR1AH=0x01;
//    OCR1AL=0x68;
//    nextCompare = 0x168;
//     heat_time = delay;        //decriment in  timer1_compa_isr
//
//    ENABLE_TIMER1_COMPA();
//
//}

////////////////SDI12 functions///////////////////////












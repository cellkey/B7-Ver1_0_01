////////////////start of General.c file//////////////
#include "define.h"
#include "rtc.h"
#include "uart.h"
#include <iobits.h>

//void Radio_Pin1_ON(void);
//void Radio_Pin1_OFF(void);

extern flash unsigned char AT_CCLK[];
extern eeprom char cpue2_interval_1;
extern eeprom unsigned char eStartConnectionH;        //first connection hour
extern eeprom unsigned char eConnectionInDay;        //number on connectionsin a day
extern eeprom unsigned char eConnectIntervalH;        //intervalbetween connections (hours)
extern eeprom char unique_id[]; //sensors id
extern eeprom unsigned char NumSensors;
extern eeprom BYTE SensorType[];
extern eeprom char eBackToNormalAlertEnabled ;
extern eeprom char eRelay1Enabled;
extern eeprom char eRelay2Enabled;
extern eeprom char eFLAGS_STATUS;
extern eeprom char eTimeZone;

//----------------LOG---------------------------

//#define eLog_Msg_Len   6             // 5.3.12.33.4       05/03,12:33 4
//#define eLog_Size (eLog_Msg_Len * 50)
//#define eLog_Start 0x130
//#define eLog_End (eLog_Start + eLog_Size)
//
//eeprom char eLogSection[eLog_Size] @0x130;
//eeprom char eLogSection[eLog_Size];
//
//eeprom unsigned int epLog_Read @0x120;
//eeprom unsigned int epLog_Read = 0x130;
//
//eeprom unsigned int epLog_Write @0x122;
//eeprom unsigned int epLog_Write = 0x130;
//
//unsigned int pLog_Read;                //pointer to read location in the circular buffer
//unsigned int pLog_Write;              //pointer to write location
unsigned int ErrorType = 0;;
//-----------------------------------------------------------------------------
//extern eeprom char eCombination;
extern int_bytes union_b2i;
long_bytes lb;
extern unsigned int pSens_Ctrl_Params;	//pointer to current sensor control parameters in ext_e2
extern char e2_writeFlag;
extern char readClockBuf[];	         //buffer for data reading from clock
extern char cuurent_interval;  	//varible to hold the the current measuring interval
extern char ComFailureCounter;
extern BYTE NextByteIndex;
extern BYTE mainTask;
extern BYTE objToMsr;
//extern BYTE triggerCnt;
extern BYTE toDoList;
//extern BYTE bWatchDog;
extern BYTE monitorCurTask;
extern BYTE NotMonitored;
extern BYTE bTimeToMeasure;
extern BYTE bConnectOK;
extern BYTE msrCurTask;
extern BYTE modemCurTask;
extern BYTE dataSentOK;
extern BYTE prmSentOK;
//extern BYTE LedStatus[4];
//extern BYTE BlinkNum[4];
extern BYTE OutOfLmtCnt[MAX_SEN_NUM];
extern BYTE OutOfLmtCntPump[MAX_SEN_NUM];
extern BYTE BytesToSend;
extern BYTE waitingTask;
extern BYTE nFailureCntr;
extern BYTE iFirstConnToday;
extern BYTE bCheckBatr;
extern BYTE nInt1Fired;

extern BYTE msr4mntrFlag;
extern BYTE msrAlertFlag;
extern BYTE AlertStatus[MAX_SEN_NUM];
extern BYTE bPORT;
extern BYTE initCnt;
extern char ModemAgainCount;
extern char  WM_SilencCount;

extern int measure_time;
extern unsigned int time_in_minutes;
extern unsigned int wtrPulseCnt;


extern int timer0_count;       //count TOF2 int
extern bit Measure_Timer_Active;        //flag checked in TOF2
extern bit bCheckRxBuf;
extern bit bWaitForModemAnswer;
extern bit bWaitForMonitorCmd;
extern bit bExtReset;
extern bit bNeedToWait4Answer;
extern bit bEndOfMeasureTask;
extern bit bEndOfModemTask;
extern bit bEndOfMonitorTask;
extern bit bReset;
extern bit IsAlertNeeded;
extern bit IsAlertActivated;
extern bit ComDelayNeeded;
//extern bit WaitNextHour;
extern bit MAGNET_SW_ON;
extern bit COVER_SW_ON;
extern bit IsCoverAlertNeeded;
extern bit BackToNormalAlertEnabled;
extern bit BackToNormalAlertNeeded;
extern bit QuickSensorsCheckNeeded;
extern bit MAGNET_SW2_ON;
//extern bit POWER_FAILURE;

extern bit SDI_EC_MEASURED ;
extern bit SDI_PH_MEASURED ;
extern bit SDI_TMP_MEASURED ;
extern bit Relay1Enabled;
extern bit Relay2Enabled;
extern bit WLV_OVF ;
extern bit  PumpActivationNeeded;

extern bit PumpActivated;
extern bit LongSWActivation ;
extern bit ModemRepeatNeeded ;
extern bit DataSent;
extern bit ModemIsOn ;
extern bit Backward_clock_update;
extern bit GPS_data_needed;

extern char gHour;
extern char gMin;
extern char ComBuf[MAX_TX_BUF_LEN];
extern unsigned int nextCompare;

extern int iVoltage;
extern int heat_time;


extern char ModemRepeatCount;

extern char RADIO_PIN_ON[];
extern char RADIO_PIN_OFF[];

extern void SendATCmd(flash unsigned char *bufToSend);
extern char RxUart0Buf[MAX_RX0_BUF_LEN];
extern void SendATCmd(flash unsigned char *bufToSend);
extern int twiReadReg( unsigned char addr,  unsigned char reg);
//extern unsigned char GetStatusReg(unsigned char mask);
extern void SetStatusReg(unsigned char mask, unsigned char val);
extern void twiInit (void);
extern  void ClearAF(void);
extern void MUX_SDI12_00(void);
extern void twi_master_init1(unsigned int bit_rate);
extern void Send_433_Str(char length, unsigned char  *buffer);
extern  char  ReadSaveGPSSensor(void);
extern void UART3_WriteMsg( char *InStr);
extern void _putchar3(char c);
 

//extern char ALertComCounter;
extern char WakeTest;
extern  char Timer0_Ticks;
extern char FlagsStatus;

  void Buzer_it(char loops, unsigned int duration, char PwrOn);
   void HandleCoverSW(void) ;

  bit bDemoMode;
  bit CoverIsOpen = FALSE;
  bit CoverAlertSent = FALSE;
  bit UnitWaked;
  bit IsScheduledCom = FALSE;;

 // char CurrentDay = 0;
  char SavedDataLost = 0;
  char gBUF[7];
  char TimeZone;
  char LoopsCount;
  char CurrentHourOfFailure;


//  bit Relay1Enabled;
//  bit Relay2Enabled;
//extern int SensorResult;       //save the measuring result into variable
//extern BYTE useExtInt1; //water meter interrupt
//extern BYTE bQuickSleep;
//extern unsigned int wndSpdPulseCnt;
BYTE bTestRTC;

void ActivateTimer1(void)
{
    // Timer/Counter 1 initialization
// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 14.400 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.1 s
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
DISABLE_TIMER1();
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
//TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);

     TCNT1H=0xFA;
     TCNT1L=0x60;  
     TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (0<<OCIE1A) | (1<<TOIE1);  //allow OVF int
     ENABLE_TIMER1();      //int every 100 mili ssec  
}   

  
void InitVarsForConnecting()
{
    mainTask = TASK_MODEM;
    modemCurTask = TASK_NONE;      //modem to be activated
    bEndOfModemTask = FALSE;
    bConnectOK = FALSE;
    dataSentOK = FALSE;
    prmSentOK = FALSE;
    DataSent = FALSE;
  //  objToMsr = SENSOR1;

    if (bExtReset)
        toDoList = DO_DATA ; //DO_DATA_N_PRMS;         //Danny
     waitingTask = SUB_TASK_MODEM_POST_DATA;

    bCheckBatr = 0;   //battery check flag
    nFailureCntr = 0;  
  
    PRR0_EN_UART0();           // allow uart0 in PRR
    ENABLE_UART0();
    ActivateTimer1(); 
    Timer0_Ticks = 12;
   
}

 //modified to mega2560
void InitPeripherals()


{
  // USART0 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART0 Receiver: On
// USART0 Transmitter: On
// USART0 Mode: Asynchronous
// USART0 Baud Rate: 19200
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x2F;


  // USART1 initialization
// USART1 disabled
UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);

    // USART2 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART2 Receiver: On
// USART2 Transmitter: On
// USART2 Mode: Asynchronous
// USART2 Baud Rate: 19200
UCSR2A=(0<<RXC2) | (0<<TXC2) | (0<<UDRE2) | (0<<FE2) | (0<<DOR2) | (0<<UPE2) | (0<<U2X2) | (0<<MPCM2);
UCSR2B=(1<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82);
UCSR2C=(0<<UMSEL21) | (0<<UMSEL20) | (0<<UPM21) | (0<<UPM20) | (0<<USBS2) | (1<<UCSZ21) | (1<<UCSZ20) | (0<<UCPOL2);
UBRR2H = 0x00;

 UBRR2L=0x2F;   //19200

// USART3 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART3 Receiver: On
// USART3 Transmitter: On
// USART3 Mode: Asynchronous
// USART3 Baud Rate: 9600
UCSR3A=(0<<RXC3) | (0<<TXC3) | (0<<UDRE3) | (0<<FE3) | (0<<DOR3) | (0<<UPE3) | (0<<U2X3) | (0<<MPCM3);
UCSR3B=(0<<RXCIE3) | (0<<TXCIE3) | (0<<UDRIE3) | (1<<RXEN3) | (1<<TXEN3) | (0<<UCSZ32) | (0<<RXB83) | (0<<TXB83);
UCSR3C=(0<<UMSEL31) | (0<<UMSEL30) | (0<<UPM31) | (0<<UPM30) | (0<<USBS3) | (1<<UCSZ31) | (1<<UCSZ30) | (0<<UCPOL3);
UBRR3H=0x00;
UBRR3L=0x5F;

// Analog Comparator initialization




// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
// Digital input buffer on AIN0: On
// Digital input buffer on AIN1: On
DIDR1=(0<<AIN0D) | (0<<AIN1D);

// 2560 ADC initialization


//// ADC Clock frequency: 115.200 kHz
//// ADC Voltage Reference: AVCC pin
//// ADC Auto Trigger Source: ADC Stopped
//// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
//// ADC4: Off, ADC5: Off, ADC6: Off, ADC7: On
//DIDR0=(0<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
//// Digital input buffers on ADC8: Off, ADC9: Off, ADC10: Off, ADC11: Off
//// ADC12: Off, ADC13: Off, ADC14: Off, ADC15: Off
//DIDR2=(1<<ADC15D) | (1<<ADC14D) | (1<<ADC13D) | (1<<ADC12D) | (1<<ADC11D) | (1<<ADC10D) | (1<<ADC9D) | (1<<ADC8D);
//ADMUX=ADC_VREF_TYPE;
//ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
//ADCSRB=(0<<MUX5) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

// ADC initialization
// ADC Clock frequency: 57.600 kHz
// ADC Voltage Reference: 2.56V, cap. on AREF
// ADC Auto Trigger Source: ADC Stopped
// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
// ADC4: Off, ADC5: Off, ADC6: Off, ADC7: Off
DIDR0=(1<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (0<<ADC1D) | (0<<ADC0D);
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);
ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);






// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

    if( eBackToNormalAlertEnabled == 1)
    BackToNormalAlertEnabled = TRUE;   //read val from eeprom
    else
    BackToNormalAlertEnabled = FALSE;   //read val from eeprom

     if(eRelay1Enabled == 1)
    Relay1Enabled = TRUE;   //read val from eeprom
    else
    Relay1Enabled = FALSE;   //read val from eeprom

    if(eRelay2Enabled == 1)
    Relay2Enabled = TRUE;   //read val from eeprom
    else
    Relay2Enabled = FALSE;   //read val from eeprom

   // TWI initialization
// Mode: TWI Master
// Bit Rate: 100 kHz
twi_master_init1(50);

}

//SET PINS AFTER WAKEUP -CREACELL 2
void InitIOs(void)
{

            //--------------------------2560---------------------------------------------------
          // Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRB=(1<<DDB7) | (1<<DDB6) | (0<<DDB5) | (1<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
DDRC=(1<<DDC7) | (1<<DDC6) | (0<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=0 Bit6=0 Bit5=T Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (1<<PORTD0);

// Port E initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=Out Bit2=Out Bit1=Out Bit0=In
DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (1<<DDE3) | (1<<DDE2) | (1<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=0 Bit2=0 Bit1=0 Bit0=T
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);
#ifdef ONE_WIRE_F0
    PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (1<<PORTF0);  //pullup on bit 0
#endif


// Port G initialization
// Function: Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRG=(0<<DDG5) | (0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTG=(0<<PORTG5) | (0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Port H initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=In
DDRH=(1<<DDH7) | (1<<DDH6) | (1<<DDH5) | (1<<DDH4) | (1<<DDH3) | (0<<DDH2) | (1<<DDH1) | (0<<DDH0);  //bit 2 input
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=T
PORTH=(0<<PORTH7) | (0<<PORTH6) | (0<<PORTH5) | (0<<PORTH4) | (0<<PORTH3) | (0<<PORTH2) | (0<<PORTH1) | (0<<PORTH0);

// Port J initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=Out Bit1=Out Bit0=In
DDRJ=(0<<DDJ7) | (0<<DDJ6) | (0<<DDJ5) | (0<<DDJ4) | (0<<DDJ3) | (0<<DDJ2) | (1<<DDJ1) | (0<<DDJ0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=0 Bit1=0 Bit0=T
PORTJ=(0<<PORTJ7) | (0<<PORTJ6) | (0<<PORTJ5) | (0<<PORTJ4) | (0<<PORTJ3) | (0<<PORTJ2) | (0<<PORTJ1) | (0<<PORTJ0);

// Port K initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRK=(0<<DDK7) | (0<<DDK6) | (0<<DDK5) | (0<<DDK4) | (0<<DDK3) | (0<<DDK2) | (0<<DDK1) | (0<<DDK0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTK=(0<<PORTK7) | (0<<PORTK6) | (0<<PORTK5) | (0<<PORTK4) | (0<<PORTK3) | (0<<PORTK2) | (0<<PORTK1) | (0<<PORTK0);

// Port L initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTL=(0<<PORTL7) | (0<<PORTL6) | (0<<PORTL5) | (0<<PORTL4) | (0<<PORTL3) | (0<<PORTL2) | (0<<PORTL1) | (0<<PORTL0);
   //-----------------------------------------------------------------------------------



}

//void Radio_Pin1_ON(void)
//{
//        VOLTAGE_MULTIPLIER_ON(); 
//        delay_ms(10);
//        SETBIT( PORTC,3 );  //power sensor 3 -radio board
//        delay_ms(50);
//        Send_433_Str(3, RADIO_PIN_ON);
//        delay_ms(10);
//        Send_433_Str(3, RADIO_PIN_ON); 
//        VOLTAGE_MULTIPLIER_OFF();                             
//}
//void Radio_Pin1_OFF(void)
//{
//        VOLTAGE_MULTIPLIER_ON(); 
//        delay_ms(10);
//        SETBIT( PORTC,3 );  //power sensor 3 -radio board
//        delay_ms(50);
//        Send_433_Str(3, RADIO_PIN_OFF);
//        delay_ms(10);
//        Send_433_Str(3, RADIO_PIN_OFF); 
//        VOLTAGE_MULTIPLIER_OFF();                             
//}

void ResetExtWD(void)
{
     SETBIT(PORTB,4);   //external watchdog feeding
     delay_ms(50); 
     CLRBIT(PORTB,4);
}

void WakeUpProcedure(void)
{
    BYTE gPORT; 
   #ifdef WATER_METER_ACTIVE 
    char Str[40];
   #endif
     UnitWaked = TRUE;
     PRR0 = 0x00;           // allow TWI in PRR
     PRR1 = 0;
      delay_ms(100); 
    

      InitIOs();     //set ios
      V33_PWR_ON();    //power unit peripherals
     //re enable TWI
     twi_master_init1(50) ;
  
    Timer0_Ticks = 12;       //new- OVF int of timer0

      ENABLE_UART2();
      delay_ms(300);
     
      SendDebugMsg("\r\nWakeup cause = \0");
     PrintNum(WakeTest);  
         
  //   DISABLE_UART2();

     rtc_get_timeAll (readClockBuf);     //get clock
  //   measure_time = time_in_minutes;

     //external watchdog feeding
      ResetExtWD();    
      
    //  ENABLE_UART2(); 
      
      if( WakeTest == 9)    //int PJ.2
     // if(COVER_SW_ON == TRUE)  //cover is open-alert it to server
      {   
       //   SendDebugMsg("\r\nCover Is Open..checking..\r\n\0"); 
        #ifdef WATER_METER_ACTIVE 
         
             WakeTest = 0;
             wtrPulseCnt++;   //inc counter                
             Buzer_it(1, 1, 1);                      
             sprintf(Str,"WM Pulse. Counter = %d\n\r\0", wtrPulseCnt );
             UART_WriteMsg(Str); 
             delay_ms(500);          
             mainTask = TASK_SLEEP; 
          
        #else
        
           HandleCoverSW();                 
           V33_PWR_ON();   //enable 3.3V to entire board                 
          if((CoverIsOpen == TRUE) && ( CoverAlertSent == FALSE ))
          {
                COVER_SW_ON = TRUE ;
                SendDebugMsg("\r\nCover Is Open!- Alerting....\r\n\0");
     
                InitVarsForConnecting(); //prepair modem activity               
                IsCoverAlertNeeded = TRUE; 
                                   
          } 
          else
          {                 
                 if(CoverIsOpen == FALSE)
                 SendDebugMsg("Cover SW Reverse activation. sleep\r\n\0");        
                 mainTask = TASK_SLEEP;                        
          } 
        
        #endif  
        
            LED1_OFF;
          return;
      }
      
  #ifdef NETIV_ALERT_UNIT  
//    if( WakeTest == 11)
//    {
//         WakeTest = 0;
//         SendDebugMsg("\r\nPower GOOD int detected..\r\n\0"); 
//         mainTask = TASK_SLEEP;     
//         return;
//    }
        
  if((POWER_FAILURE == TRUE) && (WakeTest == 8))
  {  
       WakeTest = 0;
       delay_ms(2000);  
      gPORT = TSTBIT(PIND,3); 
      if(!gPORT)
      {   
          delay_ms(200); 
         SendDebugMsg("\r\nPower Failure detected..\r\n\0");
      //    Buzer_it(1, 1, 1);  
          InitVarsForConnecting(); //prepair modem activity               
          IsCoverAlertNeeded = TRUE;   //15 27 FFF1 00 00 00 
      //   EICRA=(1<<ISC31) | (1<<ISC30) | (1<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00); //change int to "rising edge" type
          return;
      } 

  }   
  
  
  #endif  
//-----------------------Check magnet sw --------------------------------
    if(MAGNET_SW_ON == TRUE)   //flag set at PCINT14 int rutine  by magnet
    {
       
            SendDebugMsg("\r\nMagnet SW Activated..\r\n\0");
           
            LED1_ON;                   //no pump- short blink
            delay_ms(20);
            LED1_OFF;

             ResetPumpFlags();              //at Measure.c
             IsAlertActivated = FALSE;      //updated 320815 for Tzaci request
             IsCoverAlertNeeded = FALSE;
             WLV_OVF = FALSE;      
             FlagsStatus =  eFLAGS_STATUS;
             SetStatusReg(IS_ALERT_NEEDED |  WLV_OVF_FLAG , 0);
             eFLAGS_STATUS = FlagsStatus;

            ErrorType = 0;  
              delay_ms(700);
             Buzer_it(1, 10, 1);
            delay_ms(1000);
            gPORT = TSTBIT(PINB,3);  
            
        //    if(gPORT & 0x04)           //if still on wait another 2 sec otherwise act as reset activated
             if(gPORT)
            {
                  #asm("wdr")
                  delay_ms(2000);           //wait 2 sec and check sw again
                   gPORT = TSTBIT(PINB,3);
                   if(gPORT)           //if still on wait another 2 sec otherwise act as reset activated
                   {
                           Buzer_it(3, 7, 1);           //
                           LongSWActivation = TRUE;    //go for 2 min. delay and than act as after reset
                           mainTask = TASK_SLEEP;
                           SendDebugMsg("\r\nLong SW detected..\r\n\0"); 
                           
                         #ifdef GPS_INSTALED
                           k = ReadSaveGPSSensor();                                                 
                           if(k == 1)
                           {
                               GPS_data_needed = TRUE; 
                               InitVarsForConnecting(); 
                               
                           }
                         #endif 
                            return;
                   }
            }
             else
             {
                      bExtReset = TRUE;  //  Magnetic SW act like ext. reset-asked by Tzachi-Kando 250615
                      Buzer_it(1, 10, 1);       //just for unit 6
             }
              msrAlertFlag = 0;   //unit 6 problem
      }

    if(MAGNET_SW2_ON == TRUE)   //flag set at PCINT3 int rutine  by magnet
    {

            SendDebugMsg("\r\nMagnet SW2 Activated..\r\n\0");

            LED1_ON;      //no pump- short blink
            delay_ms(20);
            LED1_OFF;

            ErrorType = 0;
            delay_ms(2000);
            gPORT = PINB;
            if(gPORT & 0x01)           //if still on wait another 2 sec otherwise act as reset activated
            {
                  #asm("wdr")   
                  Buzer_it(1, 10, 1);  
                  delay_ms(2000);           //wait 2 sec and check sw again
                   gPORT = PINB;
                   if(gPORT & 0x01)           //if still on wait another 2 sec otherwise act as reset activated
                   {
                           Buzer_it(3, 10, 1);  
                           wtrPulseCnt = 0;  
                          #ifdef DESHEN_WATER_METER     
                           WM_SilencCount = 0;  
                          #endif   
                           mainTask = TASK_SLEEP;         //    
                           
//                            LongSWActivation = TRUE;    //go for 2 min. delay and than act as after reset
//                            mainTask = TASK_MEASURE;
//                             msrCurTask = TASK_NONE;
//                            objToMsr = PUMP;

//                            SendDebugMsg("\r\nTesting PUMP..\r\n\0");
                            return;
                   }
              }
            else    //short sw activation
            {
                  bExtReset = TRUE;  //  Magnetic SW act like ext. reset-asked by Tzachi-Kando 250615
//                      Buzer_it(1, 10, 1);       //just for unit 6
                  LoopsCount = 10;
                  ENABLE_PB0_INT();    //allow int to stop loop
            }
            msrAlertFlag = 0;   //unit 6 problem
      }


  

     if (LongSWActivation == TRUE)     //waking with flag high added 160715
    {
         LongSWActivation = FALSE;
         bExtReset = TRUE;
    }
 //----------------------------------------------------------------
 //   SendDebugMsg("\r\nCheckpost 1.\r\n\0");
    bEndOfMeasureTask = FALSE;
    bEndOfModemTask = FALSE;
    bEndOfMonitorTask = FALSE;

 //   DISABLE_PB3_INT();   //PORTB.3 MISO
    DISABLE_PJ2_INT(); 
    
 
    
      //init led status
//    for (i = 1; i <= 3; i++)
//    {
//        LedStatus[i] = LED_OFF;
//        BlinkNum[i] = 0;
//    }

	//set condition for rtc communication
     SPCR=0x00; //reset spi control register

    if (bExtReset == FALSE)   //clock int is wake up cause - not user ext .int
    {
         mainTask = TASK_SLEEP;
         bTestRTC = FALSE;


            if (IsTimeToMeasure())
            {
                mainTask = TASK_MEASURE;
                msrCurTask = TASK_NONE;
                objToMsr = SENSOR1;
              
                SendDebugMsg("\r\nMeasuring Sensor: 1.\r\n\0");
              
                 SDI_EC_MEASURED = FALSE;
                 SDI_PH_MEASURED = FALSE;
                 SDI_TMP_MEASURED = FALSE;
            }
           else    
            Backward_clock_update = FALSE;  
         
    }
    else    //waked by user ext int
    {
      
        SendDebugMsg("\r\nExternal Activation!\r\n\0");
        if (NumSensors > 0)    //sensors defined?
        {

            mainTask = TASK_MEASURE;
            msrCurTask = TASK_NONE;
            objToMsr = SENSOR1;
             SendDebugMsg("\r\nMeasuring Sensor: 1.\r\n\0");
        }

    }
   
    bWaitForModemAnswer = FALSE;
    bWaitForMonitorCmd = FALSE;
    bCheckRxBuf = FALSE;
    bNeedToWait4Answer = TRUE;
    msr4mntrFlag = FALSE;


    if (mainTask != TASK_SLEEP)
    {
        #pragma optsize-
        #asm("wdr")
        WATCHDOG_ENABLE_STEP1();
        WATCHDOG_ENABLE_STEP2();
        #ifdef _OPTIMIZE_SIZE_
        #pragma optsize+
        #endif
    }

//      if (mainTask == TASK_MEASURE)
//     SendDebugMsg("\r\nwake to TASK_MEASURE\r\n\0");
        ClearAF();         //reset rtc int flag
        WakeTest = 0;
}

int IsExtInt1Used()
{
    //look for water/rain meter to sign whether application should activate external interrupt1 when sleeping
//    for (objToMsr = SENSOR1; objToMsr < NumSensors; objToMsr++)
//        if ((SensorType[objToMsr] == WM) || (SensorType[objToMsr] == RAINMTR))
//            return  TRUE;
    return 0;
}

void PwrDwn(void)
{
//int i;
//    EIFR=0x04;
//    EIMSK=0x04;


    SMCR = 4;
    SMCR |= 1;

    #asm
    sleep
    #endasm

//for(i = 0; i < 2000; i++)
    #asm ("nop"); 		//wakeup from sleep mode
    #asm ("nop");
}

void PowerDownSleep( void )
{
// char testbit = 0;
   //  SendDebugMsg("PowerDownSleep()\r\n\0"); 
  //---------------
    LED1_ON ;
    delay_ms(50);
    LED1_OFF;
  //---------------

    // Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
DDRC=(1<<DDC7) | (1<<DDC6) | (0<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=0 Bit6=0 Bit5=T Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (1<<PORTD4) | (0<<PORTD3) | (1<<PORTD2) | (1<<PORTD1) | (1<<PORTD0);

// Port E initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=Out Bit2=Out Bit1=Out Bit0=In
DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (0<<DDE3) | (1<<DDE2) | (1<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=0 Bit2=0 Bit1=0 Bit0=T
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRG=(0<<DDG5) | (0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTG=(0<<PORTG5) | (0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Port H initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=In
DDRH=(1<<DDH7) | (1<<DDH6) | (1<<DDH5) | (1<<DDH4) | (1<<DDH3) | (1<<DDH2) | (1<<DDH1) | (0<<DDH0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=T
PORTH=(0<<PORTH7) | (0<<PORTH6) | (0<<PORTH5) | (0<<PORTH4) | (0<<PORTH3) | (0<<PORTH2) | (0<<PORTH1) | (0<<PORTH0);

// Port J initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=Out Bit1=Out Bit0=In
DDRJ=(0<<DDJ7) | (0<<DDJ6) | (0<<DDJ5) | (0<<DDJ4) | (0<<DDJ3) | (0<<DDJ2) | (1<<DDJ1) | (0<<DDJ0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=0 Bit1=0 Bit0=T
PORTJ=(0<<PORTJ7) | (0<<PORTJ6) | (0<<PORTJ5) | (0<<PORTJ4) | (0<<PORTJ3) | (0<<PORTJ2) | (0<<PORTJ1) | (0<<PORTJ0);

// Port K initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRK=(0<<DDK7) | (0<<DDK6) | (0<<DDK5) | (0<<DDK4) | (0<<DDK3) | (0<<DDK2) | (0<<DDK1) | (0<<DDK0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTK=(0<<PORTK7) | (0<<PORTK6) | (0<<PORTK5) | (0<<PORTK4) | (0<<PORTK3) | (0<<PORTK2) | (0<<PORTK1) | (0<<PORTK0);

// Port L initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTL=(0<<PORTL7) | (0<<PORTL6) | (0<<PORTL5) | (0<<PORTL4) | (0<<PORTL3) | (0<<PORTL2) | (0<<PORTL1) | (0<<PORTL0);

 
 
       DISABLE_TIMER1();
   //    DISABLE_UART2();

      ENABLE_PB3_INT();     //       enable magnetic sw1 int
      ENABLE_PB0_INT();      //     enable MS2

      ENABLE_PJ2_INT();      //   enable Cover SW int again  //   enable new session  
      
       EIFR |= (1<<INTF2) ;
      SETBIT(EIMSK,2);           // ENABLE_CLOCK_INT(); // enable external interrupt2 //Danny   
            
  #ifdef NETIV_ALERT_UNIT 
       EIFR |= (1<<INTF3) ;   //ext int 3 active
       SETBIT(EIMSK,3);               
   #endif   
////---------------Ext SW check------------------------
    #ifndef WATER_METER_ACTIVE   //if NOT define!!
            
     #endif            
// //-------------------------------------------
     
      ModemIsOn = FALSE ;
      UnitWaked = FALSE;
      bExtReset = FALSE;
      MAGNET_SW_ON = FALSE;
      MAGNET_SW2_ON = FALSE;

      CLRBIT( TWCR,TWEN);   //disable twi
      EIFR |= (1<<INTF2) ;
      SETBIT(EIMSK,2);           // ENABLE_CLOCK_INT(); // enable external interrupt2 //Danny 
    
      #asm("sei")
      PRR0 = 0x7F;
      PRR1 = 0XFD;              //uart2 alive for test
//        ENABLE_PB3_INT();
      WDT_off();    //disable WD at sleep mode    
      
       SendDebugMsg("\r\nGo2Sleep\r\n\0");
       delay_ms(100);
       DISABLE_UART2(); 
       V33_PWR_OFF();
       PwrDwn();

}

//void PowerDownSleep( void )
//{
//  //  int t;
//    bExtReset = FALSE;
//
//// Port A initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);
//
//// Port B initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);
//
//// Port C initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRC=(1<<DDC7) | (0<<DDC6) | (0<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);
//
//// Port D initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (1<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);
//
//// Port E initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (0<<DDE3) | (0<<DDE2) | (0<<DDE1) | (0<<DDE0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);
//
//// Port F initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);
//
//// Port G initialization
//// Function: Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRG=(0<<DDG5) | (0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
//// State: Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTG=(0<<PORTG5) | (0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);
//
//// Port H initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRH=(1<<DDH7) | (0<<DDH6) | (0<<DDH5) | (0<<DDH4) | (0<<DDH3) | (0<<DDH2) | (0<<DDH1) | (0<<DDH0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTH=(0<<PORTH7) | (0<<PORTH6) | (0<<PORTH5) | (0<<PORTH4) | (0<<PORTH3) | (0<<PORTH2) | (0<<PORTH1) | (0<<PORTH0);
//
//// Port J initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRJ=(0<<DDJ7) | (0<<DDJ6) | (0<<DDJ5) | (0<<DDJ4) | (0<<DDJ3) | (0<<DDJ2) | (0<<DDJ1) | (0<<DDJ0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTJ=(0<<PORTJ7) | (0<<PORTJ6) | (0<<PORTJ5) | (0<<PORTJ4) | (0<<PORTJ3) | (0<<PORTJ2) | (0<<PORTJ1) | (0<<PORTJ0);
//
//// Port K initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRK=(0<<DDK7) | (0<<DDK6) | (0<<DDK5) | (0<<DDK4) | (0<<DDK3) | (0<<DDK2) | (0<<DDK1) | (0<<DDK0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTK=(0<<PORTK7) | (0<<PORTK6) | (0<<PORTK5) | (0<<PORTK4) | (0<<PORTK3) | (0<<PORTK2) | (0<<PORTK1) | (0<<PORTK0);
//
//// Port L initialization
//// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
//DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);
//// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
//PORTL=(0<<PORTL7) | (0<<PORTL6) | (0<<PORTL5) | (0<<PORTL4) | (0<<PORTL3) | (0<<PORTL2) | (0<<PORTL1) | (0<<PORTL0);
//
//
////  SendDebugMsg("\r\nGo2Sleep\r\n\0");
//
////   DISABLE_TIMER1_COMPA()
////   DISABLE_UART2();
//
//      WDT_off();
//ENABLE_PB3_INT();
//
//     PwrDwn();
//
//}



void TurnOnLed(BYTE led, BYTE type)
{
//    if (led ==  LED_1)
//        PORTA.5 = 1;
//    if (led ==  LED_2)
//        PORTA.6 = 1;
////    if (led ==  LED_3)
////        PORTA.7 = 1;
//    if (bExtReset)
//    {
//        if (type == SUCCESS)
//            LedStatus[led] = LED_ON;
//        else
//        {
//            LedStatus[led] = LED_BLINK;
//            BlinkNum[led] = 19;
//        }
//    }
//    else
//    {
//        LedStatus[led] = LED_BLINK;
//        if (type == SUCCESS)
//            BlinkNum[led] = 1;
//        else
//            BlinkNum[led] = 5;
//    }
}


void TurnOffLed()
{
//    PORTA.5 = 0;
//    PORTA.6 = 0;
//    PORTA.7 = 0;
}

//the function recieve pointer to buf
//buf[0]=hi_byte, buf[1]=lo_byte
//the function return int (address) combined from 2 bytes
int bytes2int(char* buf)
{
	//set 2 bytes into union
	union_b2i.bval[0] = buf[0];
	union_b2i.bval[1] = buf[1];

	return union_b2i.ival;
}

//the function recieve int (address) and pointer to buf
//the function set into buf[0] the hi_address_byte
//and into buf[1] the lo_address_byte
void int2bytes(int int_pointer, char* buf)
{
	union_b2i.ival = int_pointer;
	//set 2 bytes into buf
	buf[0] = union_b2i.bval[0];
	buf[1] = union_b2i.bval[1];
}

// #monitor
//the function recieve pointer to buf
//buf[0]=hi_byte, buf[1]=lo_byte
//the function return int (address) combined from 2 bytes
long Bytes2Long(char* buf)
{
	//set 4 bytes into union
	lb.bVal[0] = buf[0];
	lb.bVal[1] = buf[1];
	lb.bVal[2] = buf[2];
	lb.bVal[3] = buf[3];

	return lb.lVal;
}

//the function recieve int (address) and pointer to buf
//the function set into buf[0] the hi_address_byte
//and into buf[1] the lo_address_byte
void Long2Bytes(long l, char* buf)
{
	lb.lVal = l;
	//set 2 bytes into buf
	buf[0] = lb.bVal[0];
	buf[1] = lb.bVal[1];
	buf[2] = lb.bVal[2];
	buf[3] = lb.bVal[3];
}

//copy from cpu e2 into buf
void cpu_e2_to_MemCopy( BYTE* to, char eeprom* from, BYTE length)
{
	BYTE i;
	for(i = 0; i < length; i++)
		to[i] = from[i];
}

//copy from ram buf into cpu e2
void MemCopy_to_cpu_e2( char eeprom* to, BYTE* from, BYTE length)
{

	BYTE i;
	for(i = 0; i < length; i++)
		to[i] = from[i];
}


void MemCopy( BYTE* to, BYTE* from, BYTE length)
{
	BYTE i;
	for(i = 0; i < length; i++)
		to[i] = from[i];
}


//copy from cpu flash into buf
void cpu_flash_to_MemCopy( BYTE* to, char flash* from, BYTE length)
{
	BYTE i;
	for(i = 0; i < length; i++)
		to[i] = from[i];
}

//copy flash buf to buffer. return num of  copy bytes
BYTE CopyFlashToBuf( BYTE* to, char flash* from)
{
    BYTE index = 0;
    while (from[index] != '@')
    {
        to[index] = from[index];                      //"425#"
        index++;
    }
    return index;
}

//checksum with parameter
// 0 = check_sum ^
// 1 = check_sum +
BYTE CheckSum( BYTE *buff, BYTE length, BYTE param )
{
        BYTE check_sum;

        check_sum = 0;
        while (length--)
        {
                //filter the "246 (F6)" for gprs modem ??
                if(param)
                {
                    check_sum += *buff++;
                }

                else
        	        check_sum ^= *buff++;
        }
        return (check_sum);
}


//the function will set address into 'pSens_Ctrl_Params' and 'pSens_cpu_e2'
// (int global variables define in data manager module)
void SetCotrolParamAddress()
{
    //pSens_Ctrl_Params = sens1_control_param;
    // POINT TO 16 BYTES LOWER THAN END OF DATA BLOCK
    pSens_Ctrl_Params = (unsigned int)(unsigned int)(SENSOR_MEMORY_START + (unsigned int)(((int)objToMsr + 1) * SENSOR_MEMORY_SIZE) - SENSOR_CNTRL_PRM_SIZE);
}

void InitProgram(void)
{
    unsigned char  data_not_valid = 0;

    // Crystal Oscillator division factor: 1
    #pragma optsize-
    CLKPR=(1<<CLKPCE);
    CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
    #ifdef _OPTIMIZE_SIZE_
    #pragma optsize+
    #endif


    // Input/Output Ports initialization: 1=Out, 0=In
    // State: For Dir input and value=0 --> TriState

  // Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRB=(1<<DDB7) | (1<<DDB6) | (0<<DDB5) | (1<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
DDRC=(1<<DDC7) | (1<<DDC6) | (0<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=0 Bit6=0 Bit5=T Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (1<<DDD1) | (1<<DDD0);    //pd.3 input for Netiv
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (1<<PORTD0);

// Port E initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=Out Bit2=Out Bit1=Out Bit0=In
DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (1<<DDE3) | (1<<DDE2) | (1<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=0 Bit2=0 Bit1=0 Bit0=T
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRG=(0<<DDG5) | (0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTG=(0<<PORTG5) | (0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Port H initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=In
DDRH=(1<<DDH7) | (1<<DDH6) | (1<<DDH5) | (1<<DDH4) | (1<<DDH3) | (1<<DDH2) | (1<<DDH1) | (0<<DDH0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=T
PORTH=(0<<PORTH7) | (0<<PORTH6) | (0<<PORTH5) | (0<<PORTH4) | (0<<PORTH3) | (0<<PORTH2) | (0<<PORTH1) | (0<<PORTH0);

// Port J initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=Out Bit1=Out Bit0=In
DDRJ=(0<<DDJ7) | (0<<DDJ6) | (0<<DDJ5) | (0<<DDJ4) | (0<<DDJ3) | (0<<DDJ2) | (1<<DDJ1) | (0<<DDJ0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=0 Bit1=0 Bit0=T
PORTJ=(0<<PORTJ7) | (0<<PORTJ6) | (0<<PORTJ5) | (0<<PORTJ4) | (0<<PORTJ3) | (0<<PORTJ2) | (0<<PORTJ1) | (0<<PORTJ0);

// Port K initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRK=(0<<DDK7) | (0<<DDK6) | (0<<DDK5) | (0<<DDK4) | (0<<DDK3) | (0<<DDK2) | (0<<DDK1) | (0<<DDK0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTK=(0<<PORTK7) | (0<<PORTK6) | (0<<PORTK5) | (0<<PORTK4) | (0<<PORTK3) | (0<<PORTK2) | (0<<PORTK1) | (0<<PORTK0);

// Port L initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
DDRL=(0<<DDL7) | (0<<DDL6) | (0<<DDL5) | (0<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
PORTL=(0<<PORTL7) | (0<<PORTL6) | (0<<PORTL5) | (0<<PORTL4) | (0<<PORTL3) | (0<<PORTL2) | (0<<PORTL1) | (0<<PORTL0);


// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 14.400 kHz
// Mode: Normal top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
// Timer Period: 17.778 ms
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
//TCCR0B=(0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
DISABLE_TIMER0();
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;



//------------------------timer1 overflow int option--------------------------------
// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 14.400 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.1 s
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off

TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
//TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
TCNT1H=0xFA;
TCNT1L=0x60;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;



// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 14.400 kHz
// Mode: Normal top=0xFF
// OC2A output: Disconnected
// OC2B output: Disconnected
// Timer Period: 17.778 ms
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
//TCCR2B=(0<<WGM22) | (1<<CS22) | (1<<CS21) | (1<<CS20);
DISABLE_TIMER2();     //DISABLED
TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;

// Timer/Counter 3 initialization
// Clock source: System Clock
// Clock value: Timer3 Stopped
// Mode: Normal top=0xFFFF
// OC3A output: Disconnected
// OC3B output: Disconnected
// OC3C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer3 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR3A=(0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (0<<WGM32) | (0<<CS32) | (0<<CS31) | (0<<CS30);
TCNT3H=0x00;
TCNT3L=0x00;
ICR3H=0x00;
ICR3L=0x00;
OCR3AH=0x00;
OCR3AL=0x00;
OCR3BH=0x00;
OCR3BL=0x00;
OCR3CH=0x00;
OCR3CL=0x00;

// Timer/Counter 4 initialization
// Clock source: System Clock
// Clock value: 14745.600 kHz
// Mode: Normal top=0xFFFF
// OC4A output: Disconnected
// OC4B output: Disconnected
// OC4C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 1 ms
// Timer4 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR4A=(0<<COM4A1) | (0<<COM4A0) | (0<<COM4B1) | (0<<COM4B0) | (0<<COM4C1) | (0<<COM4C0) | (0<<WGM41) | (0<<WGM40);
//TCCR4B=(0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (0<<WGM42) | (0<<CS42) | (0<<CS41) | (1<<CS40);
TCCR4B=(0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (0<<WGM42) | (0<<CS42) | (0<<CS41) | (0<<CS40);
TCNT4H=0xC6;
TCNT4L=0x66;
ICR4H=0x00;
ICR4L=0x00;
OCR4AH=0x00;
OCR4AL=0x00;
OCR4BH=0x00;
OCR4BL=0x00;
OCR4CH=0x00;
OCR4CL=0x00;



// Timer/Counter 5 initialization
// Clock source: System Clock
// Clock value: Timer5 Stopped
// Mode: Normal top=0xFFFF
// OC5A output: Disconnected
// OC5B output: Disconnected
// OC5C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer5 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR5A=(0<<COM5A1) | (0<<COM5A0) | (0<<COM5B1) | (0<<COM5B0) | (0<<COM5C1) | (0<<COM5C0) | (0<<WGM51) | (0<<WGM50);
TCCR5B=(0<<ICNC5) | (0<<ICES5) | (0<<WGM53) | (0<<WGM52) | (0<<CS52) | (0<<CS51) | (0<<CS50);
TCNT5H=0x00;
TCNT5L=0x00;
ICR5H=0x00;
ICR5L=0x00;
OCR5AH=0x00;
OCR5AL=0x00;
OCR5BH=0x00;
OCR5BL=0x00;
OCR5CH=0x00;
OCR5CL=0x00;


// Timer/Counter 0 Interrupt(s) initialization-  overflow 0
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);

// Timer/Counter 1 Interrupt(s) initialization  - compare INT
//TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

// Timer/Counter 1 Interrupt(s) initialization - OVERFLOW INT2
TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (0<<OCIE1A) | (1<<TOIE1);


// Timer/Counter 2 Interrupt(s) initialization  -  overflow 2
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (1<<TOIE2);


// Timer/Counter 3 Interrupt(s) initialization
TIMSK3=(0<<ICIE3) | (0<<OCIE3C) | (0<<OCIE3B) | (0<<OCIE3A) | (0<<TOIE3);

// Timer/Counter 4 Interrupt(s) initialization
TIMSK4=(0<<ICIE4) | (0<<OCIE4C) | (0<<OCIE4B) | (0<<OCIE4A) | (1<<TOIE4);

// Timer/Counter 5 Interrupt(s) initialization
TIMSK5=(0<<ICIE5) | (0<<OCIE5C) | (0<<OCIE5B) | (0<<OCIE5A) | (0<<TOIE5);

//// External Interrupt(s) initialization
//// INT0: Off
//// INT1: Off
//// INT2: On
//// INT2 Mode: Falling Edge
//// INT3: Off
//// INT4: Off
//// INT5: Off
//// INT6: Off
//// INT7: Off
//EICRA=(0<<ISC31) | (0<<ISC30) | (1<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
//EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
//EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);
//EIFR=(0<<INTF7) | (0<<INTF6) | (0<<INTF5) | (0<<INTF4) | (0<<INTF3) | (1<<INTF2) | (0<<INTF1) | (0<<INTF0);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: On
// INT2 Mode: Falling Edge
// INT3: On
// INT3 Mode: Any change - for Netiv
// INT4: Off
// INT5: Off
// INT6: Off
// INT7: Off
//EICRA=(0<<ISC31) | (1<<ISC30) | (1<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRA=(1<<ISC31) | (0<<ISC30) | (1<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
//EICRA=(0<<ISC31) | (0<<ISC30) | (1<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);
EIFR=(0<<INTF7) | (0<<INTF6) | (0<<INTF5) | (0<<INTF4) | (1<<INTF3) | (1<<INTF2) | (0<<INTF1) | (0<<INTF0);

// PCINT0 interrupt: On
// PCINT3 interrupt: On
// PCINT11 interrupt: On
PCMSK0 = (0<<PCINT7) | (0<<PCINT6) | (0<<PCINT5) | (0<<PCINT4) | (0<<PCINT3) | (0<<PCINT2) | (0<<PCINT1) | (0<<PCINT0);
PCMSK1 = (0<<PCINT15) | (0<<PCINT14) | (0<<PCINT13) | (0<<PCINT12) | (0<<PCINT11) | (0<<PCINT10) | (0<<PCINT9) | (0<<PCINT8);
PCMSK2 = (0<<PCINT23) | (0<<PCINT22) | (0<<PCINT21) | (0<<PCINT20) | (0<<PCINT19) | (0<<PCINT18) | (0<<PCINT17) | (0<<PCINT16);
PCICR = (0<<PCIE2) | (1<<PCIE1) | (1<<PCIE0);

//
//   DISABLE_PJ2_INT();    //ext SW
//   DISABLE_PB0_INT();   //MS2 sw
//   DISABLE_PB3_INT();   //MS1 sw

  InitPeripherals();


    // Watchdog Timer initialization
    // Watchdog Timer Prescaler: OSC/1024k
    // Watchdog Timer interrupt: On
    MCUSR |= (1<<WDRF);
#pragma optsize-
    #asm("wdr")
//    WATCHDOG_ENABLE_STEP1(); //WDTCSR|=(1<<WDCE) | (1<<WDE);
//    WATCHDOG_ENABLE_STEP2(); //WDTCSR=(0<<WDIF) | (0<<WDIE) | (1<<WDP3) | (0<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif
//-------------WD init------------------------------------
//// Watchdog Timer initialization
//// Watchdog Timer Prescaler: OSC/1024k
//// Watchdog timeout action: Reset
//#pragma optsize-
//#asm("wdr")
//WDTCSR|=(1<<WDCE) | (1<<WDE);
//WDTCSR=(0<<WDIF) | (0<<WDIE) | (1<<WDP3) | (0<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
//#ifdef _OPTIMIZE_SIZE_
//#pragma optsize+
//#endif
//---------------------------------------------------
  	// if sensors data pointers validitation faild
	//pSens_Ctrl_Params = sens1_control_param;
    if( NumSensors > 0)          //this ondition is new-Danny 160415
    {
        for (objToMsr = SENSOR1; objToMsr < NumSensors; objToMsr++)
        {
            SetCotrolParamAddress();
            if (PointersValidate() == FALSE) /*|| (cpue2_interval_1 != cuurent_interval))*/    //danny 280115
            {
                data_not_valid = 1;
                 SendDebugMsg("Validiate memory pointers FAILED..! (InitDataBlocks() not allowed) \r\n\0");
               InitDataBlocks(cpue2_interval_1);   //test if help eliminaates holes in graphs

                break;
            }

        }
    }


//	if(data_not_valid)       //dann y 280115
//	{
//        //run the sensor initiation function
//        InitDataBlocks(cpue2_interval_1);
//	}


  	  cuurent_interval = cpue2_interval_1;
  //  mainTask = TASK_WAKEUP;   ?????????????????


    NotMonitored = FALSE;
    IsCoverAlertNeeded = FALSE;

    msrAlertFlag = 0;
     ComDelayNeeded = FALSE;
    PumpActivationNeeded = FALSE;
   // WaitNextHour = FALSE;          //if alert occured, dont repeat until next hour
    for (objToMsr = SENSOR1; objToMsr < MAX_SEN_NUM; objToMsr++)
    {
        AlertStatus[objToMsr] = ALERT_WAIT;
      //  OutOfLmtCnt[objToMsr] = 0;
    }

}

char SyncTiming(int e2_interval_val)
{
    char res = FALSE;
    //if it is 24:00
    if((readClockBuf[4] == 0)&&(readClockBuf[5] == 0))
        return TRUE;

    switch(e2_interval_val)
    {
         //if it is 5 minutes interval:
          case (5):
            if(readClockBuf[5]%5 == 0)
                res = 1;
            break;
        //if it is 10 minutes interval:
        case (10):
            if(readClockBuf[5]%10 == 0)
                res = 1;
            break;

        case (15):
            if(readClockBuf[5]%15 == 0)
                res = 1;
            break;
        //if it is 20 minutes interval:
        case (20):
            if(readClockBuf[5]%20 == 0)
                res = 1;
            break;
        //if it is 30 minutes interval:
        case (30):
            if(readClockBuf[5]%30 == 0)
                res = 1;
            break;
        //if it is round hour and 60 minutes interval:
        case (60):
            if(readClockBuf[5] == 0)
                res = 1;
            break;
        //if it is 120 minutes interval:
        case (120):
            if(readClockBuf[5] == 0)
            {
                if(readClockBuf[4] % 2 == 0)
                    res = 1;
            }
            break;
        //if it is 240 minutes interval:
        case (240):
            if(readClockBuf[5] == 0)
            {
                if(readClockBuf[4] % 4 == 0)
                    res = 1;
            }
            break;
    }

    //if it is not a measuring time
    return res;
}

//check_measuring_interval_time
BYTE IsTimeToMeasure(void)
{
	int e2_interval; //, round_minute;

//     SendDebugMsg("\r\nIsTimeToMeasure...\r\n\0 ");
    // if there are no defined sensors yet - do not measure
    if (NumSensors == 0)
    {
             return FALSE;
    }
  #ifdef Evogen_Com_1Min
        return TRUE;       //meaure now
   #endif


	//read the rtc
    e2_writeFlag = 0;   //enable reading the rtc
	//delay_ms(10);       //let stabilate the cpu oscilator


      bTimeToMeasure = FALSE;
	//check for mesuring timing:
    e2_interval = ((int)cpue2_interval_1 * INTERVAL_PARAM);

    if(SyncTiming(e2_interval) == 1)
    {              
         bTimeToMeasure = TRUE;        
    }
    else   if ((QuickSensorsCheckNeeded == TRUE) /*&&  (WaitNextHour == FALSE) &&  (PumpActivated == FALSE)*/)         //reset flag when done
    {
        SendDebugMsg("\r\nException Measured - Checking again\r\n\0 ");
        bTimeToMeasure = TRUE;   //meeasure again
    }

    //if minute is not "0" or minute modulo by "10" is not "0": disable measuring cycle
      else if (readClockBuf[5] != 0)
      if ((readClockBuf[5] % ( MEASURE_INTERVAL * cuurent_interval)) != 0)
      {
         bTimeToMeasure = FALSE;		//reset the msr flag
      } 
     
    #ifndef DESHEN_WATER_METER     //deshen option at 160120
 //       reset water pulse counter at midnight
    if ((readClockBuf[4] == 0) && (readClockBuf[5] == 0))  //midnight 
    {
        wtrPulseCnt = 0; 
         SendDebugMsg("reset wtrPulseCnt..\r\n\0 "); 
    } 
    #endif  
 //============================================================
     #ifdef DEMO
      bTimeToMeasure = TRUE;          //demo -every minute
     #endif
 //==========================================================
    return bTimeToMeasure;
}


///////////   -ORIGINAL, KEEP

//check_measuring_interval_time
//BYTE IsTimeToMeasure(void)
//{
//	int e2_interval; //, round_minute;
//    // if there are no defined sensors yet - do not measure
//    if (NumSensors == 0)
//        return FALSE;
//
//	//read the rtc
//    e2_writeFlag = 0;   //enable reading the rtc
//	delay_ms(10);       //let stabilate the cpu oscilator
//
//    rtc_get_timeAll (readClockBuf);     //new - Danny
////-->	GetRealTime();
//
//      bTimeToMeasure = FALSE;
//
//	//sens1_interval++;
//    e2_interval = ((int)cpue2_interval_1 * INTERVAL_PARAM);
//    // if u-normal value was found - measure every 10 minutes
//    if (msrAlertFlag > 0)
//        e2_interval = 10;
//
//	//check for mesuring timing:
//    if(SyncTiming(e2_interval) == 1)
//        bTimeToMeasure = TRUE;
//
//    //if minute is not "0" or minute modulo by "10" is not "0": disable measuring cycle
//    if (readClockBuf[5] != 0)
//        if (readClockBuf[5] % 10 != 0)
//            bTimeToMeasure = FALSE;		//reset the msr flag
//    //reset water pulse counter at midnight
//    if ((readClockBuf[4] == 0) && (readClockBuf[5] == 0))  //midnight
//        wtrPulseCnt = 0;
//
//    return bTimeToMeasure;
//}

BYTE IsTimeToConnectGPRS()
{
    BYTE i, t, nextConH, CyclesAday;
    iFirstConnToday = FALSE;

//   #ifdef DEMO
//    if(bDemoMode)
//   return TRUE;
//   #endif 
   
 #ifdef Evogen_Com_1Min   //connect with  server  any time passing here
        return TRUE;
   #endif
//   //===============================
//   return TRUE;      //call server every measurement
//   //=======================
  //   #ifdef DebugMode
     if( IsAlertNeeded == TRUE)
     SendDebugMsg("IsAlertNeeded = TRUE..\r\n\0 ");   //Danny for debug
     else SendDebugMsg("IsAlertNeeded = FALSE..\r\n\0 ");   //Danny for debug
       if( IsAlertActivated == FALSE)
     SendDebugMsg("IsAlertActivated = FALSE..\r\n\0 ");   //Danny for debug
     else SendDebugMsg("IsAlertActivated = TRUE..\r\n\0 ");   //Danny for debug
//       if( ComDelayNeeded == FALSE)
//     SendDebugMsg("Com Delay Needed = FALSE..\r\n\0 ");   //Danny for debug
//     else SendDebugMsg("Com Delay Needed = TRUE..\r\n\0 ");   //Danny for debug
//     if(PumpActivated == TRUE)
//     SendDebugMsg("Bottle is full..\r\n\0 ");
//     else  SendDebugMsg("Bottle is empty..\r\n\0 ");
 //    #endif DebugMode

  //    rtc_get_timeAll (readClockBuf);       //needed? clock is in buf already
//       SendDebugMsg("\r\nHour nowt = ");
//       PrintNum(readClockBuf[4]);

       #ifndef NO_SERVER
       if((IsAlertNeeded == TRUE) && (ComDelayNeeded == FALSE))   //first alert com
       {
            return TRUE;  //M0
       }
       #endif

        if (bExtReset)    //after ext reset -comm to server
        {
            toDoList = DO_DATA; 
            return TRUE;
        }

       if(( readClockBuf[4] == (CurrentHourOfFailure + 1)) && (ComDelayNeeded == TRUE))
       {
                toDoList = DO_DATA;
                return TRUE;
       }
      //  if((ModemRepeatNeeded == TRUE)  && (ModemRepeatCount == 1))
       //   if((ModemRepeatNeeded == TRUE)  && (ModemRepeatCount < 6))
        if((ModemRepeatCount > 0)   && (ModemRepeatCount < 4))
        {
                  toDoList = DO_DATA;
              //    ModemRepeatNeeded = FALSE;
                  SendDebugMsg("New Modem Com due to previous Failure....\r\n\0 ");
                  return TRUE;
        }
        else
        {
             SendDebugMsg("\r\nModemRepeatCount = ");
             PrintNum((long)ModemRepeatCount);
        } 
    
       // if last time connecting failed, and RETRY_CONNECTING_TIME passed since than - connect now
        if ((ComDelayNeeded == TRUE) && (readClockBuf[5] == RETRY_CONNECTING_TIME)) //Danny - rtry at 30 minute
        {

                SendDebugMsg("Retry Connecting due to previous failure..\r\n\0 ");   //Danny for debug
                ComDelayNeeded = FALSE;  //one retry only
                toDoList = DO_DATA;
          //             SendDebugMsg("\r\n-4-\r\n\0 ");
                return TRUE;
        }

       if(BackToNormalAlertNeeded == TRUE)   //
       return TRUE; 
                 
       CyclesAday = eConnectionInDay ; //read value from eeprom`

    //calculate the next hour for wakeup:
    for (i = 0; i < CyclesAday; i++)
    {
        //nextH = (int)startH + ((int)CycleInterval * i);
         t = i * eConnectIntervalH;
         nextConH = eStartConnectionH + t;
         if(nextConH  >= 24 )
         nextConH -= 24;

         if (readClockBuf[4] == nextConH)     //Curent HOUR value and after first measure of that hour
         {
               if (i == 0)//  if its first connection for today
                iFirstConnToday = TRUE;     //not used by Creacell

                if (readClockBuf[5] < 3 ) //== (CONNECTING_MINUTE ))  //max 3 minute offset from the hour
                {
                   
                    toDoList = DO_DATA;
                    ComFailureCounter = 0;
                    SendDebugMsg("\r\n** Scheduled Server Communication **\r\n\0");
                    ModemAgainCount = 0;
                    ModemRepeatCount = 0;
                 
                    IsScheduledCom = TRUE;
                    return TRUE;
                }
           }        
     }

    return FALSE;
}

/*
unsigned char GetNextMainTask()
{
    switch (mainTask)
    {
        case TASK_MEASURE:
            if (bEndOfMeasureTask == TRUE)
            {
                bEndOfMeasureTask = FALSE;
                if (objToMsr == BATTERY)
                {
                   // if (bCheckBatr == 1)
                    {
                        mainTask = TASK_MODEM;
                        //bCheckBatr = 2;
                        //modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;
                        break;
                    }
                }
                else
                {
                    if (bExtReset == TRUE)
                    {
                         ENABLE_UART2();
                        SendDebugMsg("\r\nCREACELL_!\0");
                        mainTask = TASK_MONITOR;
                        monitorCurTask = TASK_MONITOR_CONNECT;
                        #ifdef BlueToothOption
                        if (IsBlueToothConnect() == TRUE)
                        {
                            mainTask = TASK_BLUETOOTH;
                            NotMonitored = FALSE;
                            bEndOfBTTask = FALSE;
                            nMaxWaitBtResponse = 1200;
                             ENABLE_UART2();
                            //set power on
                            DDRD.4 = 1;
                            PORTD.4 = 1;//set PORTD.4 on
                        }
                        #endif BlueToothOption
                    }
                    else
                    {
                        mainTask = TASK_SLEEP;
                        bEndOfModemTask = FALSE;
                    }
                }
            }
            break;
        case TASK_MONITOR:
            if (bEndOfMonitorTask == TRUE)
            {
                DISABLE_UART1();
                bEndOfMonitorTask = FALSE;
                InitVarsForConnecting();
            }
        break;
        #ifdef BlueToothOption
        case TASK_BLUETOOTH:
            BlueToothMain();
            if (bEndOfBTTask == TRUE)
            {
                PORTD.4 = 0;
                bEndOfBTTask = FALSE;
                mainTask = TASK_SLEEP;
            }
            break;
        #endif BlueToothOption
        case TASK_MODEM:
            if (bEndOfModemTask == TRUE)
            {
                mainTask = TASK_SLEEP;
                bEndOfModemTask = FALSE;
                DISABLE_UART0();
            }
            if (bCheckBatr == 1)
            {
                mainTask = TASK_MEASURE;
                msrCurTask = TASK_NONE;
                objToMsr = BATTERY;
            }
            break;
        case TASK_SLEEP:
            break;
        case TASK_WAKEUP:
            break;
        default:
    }

}
*/
//#ifdef DebugMode      //Danny
//convert int less than 1000000 to string and send string to uart1
void PrintNum(long val)
{
    char s[6];
    BYTE i = 0;
    long tVAL;


    tVAL = val;
    if (tVAL < 0)
    {
         _putchar1('-');
        tVAL *= -1;
    }

    do
    {
        s[i++] = (char)(tVAL % 10);
        tVAL = tVAL / 10;
    }
    while (tVAL > 0);
    for (; i > 0; i--)
         _putchar1(s[i-1] + 48);
     _putchar1('\n');
     _putchar1('\r');
}
void PrintNum3(long val)
{
    char s[6];
    BYTE i = 0;
    long tVAL;


    tVAL = val;
    if (tVAL < 0)
    {
         _putchar1('-');
        tVAL *= -1;
    }

    do
    {
        s[i++] = (char)(tVAL % 10);
        tVAL = tVAL / 10;
    }
    while (tVAL > 0);
    for (; i > 0; i--)
         _putchar3(s[i-1] + 48);
     _putchar3('\n');
     _putchar3('\r');
}

 void Num2String_Modem(long val)
{
    char buf[6];
    BYTE i,k = 0;
    long tVAL;

    tVAL = val;
     do
    {
        buf[i++] = (char)(tVAL % 10) + 0x30;
        tVAL = tVAL / 10;
    }
    while (tVAL > 0);

    for (k = 0; i > 0; k++, i--)
     gBUF[k] = buf[i-1] ;

     gBUF[k++] = (0x0D);
     gBUF[k] = (0x0A);
   //  gBUF[k] = '\0' ;
}

void SendDebugMsg(flash  char *bufToSend)
{
    BYTE i;
  //   unsigned char message_length = 0;
    i = 0;
//
//    //copy flash string to buff
//    while (bufToSend[i] != '\0')
//    {
//         ComBuf[i] = bufToSend[i];
//         // _putchar1(ComBuf[i]);
//         i++;
//    }
//    BytesToSend = i ;
//
//   //transmitt to local modem port
//    TransmitBuf(1);
    

        while (bufToSend[i] != '\0')
        {
	          while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);  // wait for data to be transmitted
               UDR2 = bufToSend[i++];
        }
    //    RX1intEN;  // allow getting response from modem.

}

void SendDebugMsg3(flash  char *bufToSend)
{
  
      BYTE i = 0;  
      
        while (bufToSend[i] != '\0')
        {
	          while ((UCSR3A & DATA_REGISTER_EMPTY) == 0);  // wait for data to be transmitted
               UDR3 = bufToSend[i++];
        }
   
}
//#endif DebugMode            //Danny

void WDT_off(void)
{
    //__disable_interrupt();
	#asm ("wdr"); 		//reset the watchdog
    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1<<WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional time-out */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
    //__enable_interrupt();
}


//read clk to UART_RxBuf
 char RTC_Update_by_Modem(void)
 {
   char clkBuf[6];
   char clkBuf1[6];
   int tmp;
   char i, j;
 // bit ModemReadOK = FALSE;

       i = 0;
       j = 0;

            RxUart0Buf[2] = '\0';
            ENABLE_RX_INT_UART0();
             delay_ms(500);

            SendATCmd(AT_CCLK);       //get +CCLK: "05/11/21,01:47:44+00"  format from modem
            delay_ms(600);
            DISABLE_RX_INT_UART0();       //keep data
            while((RxUart0Buf[++i] != '+') && (i < 12)) ;
            if((i < 12) && (RxUart0Buf[(int)i + 1] == 'C'))    //we have \r\n+CCLK as response
          //   if((RxUart0Buf[2] == '+') && (RxUart0Buf[3] == 'C'))   //we have \r\n+CCLK as response
             {

                  SendDebugMsg("Read Modem clock.. \r\n\0");

                    clkBuf[0]= ((RxUart0Buf[(int)i+8]-0x30)*10)+(RxUart0Buf[(int)i+9] - 0x30);    //year
                     //SendDebugMsg("PrintNum result:\0");
                  //   PrintNum((long)clkBuf[0]);

                     clkBuf[1]= ((RxUart0Buf[(int)i+11]-0x30)*10)+(RxUart0Buf[(int)i+12] - 0x30);  //month
                  //   PrintNum((long)clkBuf[1]);
                      clkBuf[2]= ((RxUart0Buf[(int)i+14]-0x30)*10)+(RxUart0Buf[(int)i+15 ]- 0x30); //day
                  //    PrintNum((long)clkBuf[2]);
                       clkBuf[4]= ((RxUart0Buf[(int)i+17]-0x30)*10)+(RxUart0Buf[(int)i+18] - 0x30);  //hour
                   //    PrintNum((long)clkBuf[4]);
                        clkBuf[5]= ((RxUart0Buf[(int)i+20]-0x30)*10)+(RxUart0Buf[(int)i+21 ]- 0x30);  //minute
                   //     PrintNum((long)clkBuf[5]);

                          TimeZone =  ((RxUart0Buf[(int)i+26]-0x30)*10) + (RxUart0Buf[(int)i+27 ]- 0x30);

                          if(RxUart0Buf[(int)i+25] == '-')
                          TimeZone *= -1;
                          if(eTimeZone != TimeZone)
                          eTimeZone = TimeZone;

                      //    i = 0;
                          ENABLE_RX_INT_UART0();
                  if(( clkBuf[0]>=15) && (clkBuf[0]<20))   //year in range
                  {

                     //  +CCLK: "14/10/10,13:58:10+12"
                         rtc_get_timeAll(clkBuf1);

                       tmp =   clkBuf1[5]- clkBuf[5];   //if abs(tmp) <= 3 - OK
                       tmp *= tmp;          // make it positive

                       if((clkBuf1[0] == clkBuf[0]) &&(clkBuf1[1] == clkBuf[1])&&\
                        (clkBuf1[2] == clkBuf[2]) && (clkBuf1[4] == clkBuf[4])&&\
                        (tmp < 10))
                        {
                            SendDebugMsg("*** Unit time is OK ***\r\n\0");
                        }
                        else
                        {
                            rtc_set_timeAll(clkBuf);
                            SendDebugMsg("*** Unit time initialized with network clock ***\r\n\0");
                        }

                         j++;
                 //       CurrentDay = clkBuf[2];    //save day for inspection later
                   }
                   else  //if( clkBuf[0] <= 5)
                   {
                      SendDebugMsg("Fail: YEAR value not valid..\r\n\0");
                       #asm("wdr")

                   }

               }
               else
               {
                       SendDebugMsg("Check Clock: No answer..\r\n\0");
                       ENABLE_RX_INT_UART0();
                       return 0;
               }

             return 1;

  }




 void Buzer_it(char loops, unsigned int duration, char PwrOn)
 {
     char  j, n;
     unsigned int dur, k;

     dur = (unsigned int)(duration * 3);           //every loop - 300 micro sec. 330 loops=100 0mili sec

     DDRD.6 = 1;        //unit 6 only

     if(PwrOn == 1)
     {
         V33_PWR_ON();
         VOLTAGE_MULTIPLIER_ON();
         delay_ms(5);
     }

    for(j = 0; j < loops; j++)
    {
          //  #asm("wdr")

           for(k = 0; k< dur; k++)
           {

             for(n = 0; n< 100; n++)
             {

                 TOGGLE_BUZZER();
                 delay_us(200);                
             }
               delay_us(1000);
           }
             delay_ms(500);
     }

        DDRD.6 = 0;

       if(PwrOn == 1)
       {
            VOLTAGE_MULTIPLIER_OFF();
       }

 }

 void HandleCoverSW(void)
 {
// char gPORT;
           
      //    SendDebugMsg("\r\nCOVER_SW_ON == TRUE..\r\n\0");        
            COVER_SW_ON = FALSE;
   //         #asm("wdr");  
   
           delay_ms(2000);                  
              
           if (TSTBIT(PINJ,2))   //SW positive active
            {
                   CoverIsOpen = TRUE;                 
                   SendDebugMsg("Open Cover detected..\r\n\0");                  
            }
            else
            {       
                   CoverIsOpen = FALSE;           
                   SendDebugMsg("Cover sensor not active..\r\n\0");               
            } 
                               
 }

//extern unsigned char GetStatusReg(unsigned char mask);
extern void SetStatusReg(unsigned char mask, unsigned char val);


unsigned char GetSartatusReg(unsigned char mask){
	return (FlagsStatus & mask);
}

void SetStatusReg(unsigned char mask, unsigned char val){
	FlagsStatus &= ((~mask) | val);
	FlagsStatus |= (mask & val);
}



//
//eeprom unsigned int epLog_Read @0x120;
//eeprom unsigned int epLog_Read = 0x00;
//
//eeprom unsigned int epLog_Write @0x122;
//eeprom unsigned int epLog_Write = 0x00;
//
//unsigned int pLog_Read;                //pointer to read location in the circular buffer
//unsigned int pLog_Write;              //pointer to write location

// void WriteErrorLog(char errorType)
// {
//     char i;
//     char buf[6];
//
//      rtc_get_timeAll (readClockBuf);
//      pLog_Write = epLog_Write;        //write offset in log section
//
//      buf[0] = readClockBuf[1];     //month
//      buf[1] = readClockBuf[2];     //daqy
//      buf[2] = readClockBuf[4];     //hour
//      buf[3] = readClockBuf[5];     //min
//      buf[4] =  errorType;
//      buf[5] = '#';
//
//      for(i = 0 ; i< eLog_Msg_Len ; i++)
//      {
//         eLogSection[pLog_Write++] = buf[i];    //write to eeprom error log
//      }
//      if(pLog_Write >=  eLog_End)
//      pLog_Write =  0;
//      epLog_Write = pLog_Write;
//       SendDebugMsg("\r\nError Log updated...\r\n\0");
//
// }





// void Buzer_it(char loops)
// {
//     char i, j;
//
//     DDRA.2 = 1;
//
//    for(j = 0; j < loops; j++)
//    {
//
//                 PORTA.2 = 1;
//                 delay_ms(200);
//                 PORTA.2 = 0;
//
//
//             delay_ms(100);
//     }
//      DDRA.2 = 0;
// }



/*
//initiate the sensor
char SensorInit()
{
    char cpue2_interval;

	//initiate the clock
	InitRTC();
    //	prog_ver = PROG_VER; //set prog version into cpu_e2

	cpue2_interval = cpue2_interval_1;
    if(Sens1InitDataBlocks(cpue2_interval) == FALSE)
        return FALSE;
	return TRUE;
}
*/


/*
//move this code from "[EXT_INT0] clock_Int()" interrupt
void clock_int_procedure(void)
{
    //move this part into "clock_int_procedure()" function
	check_measuring_interval_time();

   	DisableClockIntr(); //disable clock int1


    #ifdef Sens_1_Valid
    if(interval_1 == 1)
            //bypass calling calculate_gprs_start_conditions()
            goto Bypass1;
    #endif Sens_1_Valid

    #ifdef Sens_2_Valid
    if(interval_2 == 1)
            //bypass calling calculate_gprs_start_conditions()
            goto Bypass1;
    #endif Sens_2_Valid

    #ifdef Sens_3_Valid
    if(interval_3 == 1)
            //bypass calling calculate_gprs_start_conditions()
            goto Bypass1;
    #endif Sens_3_Valid

    //no bypass calling was ditected:
    //-------------------
    //check the reply option
    if(ReplyDownload == 1)
            ReplyMinutesCount++;
    else
            ReplyMinutesCount = 0; //reset counter
    //check the reply counter: if it is 5, wakeup modem now
    if(ReplyMinutesCount >= 5)
    {
            flagWakeUp = 1;
            ReplyMinutesCount = 0; //reset counter
    }
    //--------------------
    //call this function once it wakeup.
    calculate_gprs_start_conditions();

    //bypass was ditected:
    //do not call the wakeup function
    Bypass1:
}
// int0 (external) interupt routine connected to the r.t.c int1 output
interrupt [EXT_INT0] void clock_Int(void)
{
    //decriese the interval counter each 1 minute
    //check if measuring interval is over
	#ifdef Int2_for_WM
	DISABLE_EXT_INT2();//disable int2
	#endif Int2_for_WM

    //set main loop flag and do all wakeup proccess in main
    main_loop_busy = 6;

    #ifdef DebugMode //--------
  	int0_flag = 1;
	DISABLE_EXT_INT();	//disable int0 and int1
    #endif //-----end of DebugMode

}


//-----------------------------------
// TIMER0 overflow interupt routine cycle - every 10 m/sec
interrupt [TIM0_OVF] void timer0_overflow(void)
{
    heatTime_count++;  	//64000x10ms max sensor heat time count
    cpuWakeTime_count++;    //6000ms cpu wake time from rx

        //------
	if(heatTime_count > 64000)
	        heatTime_count = 0;
	if(cpuWakeTime_count > 32000)
	        cpuWakeTime_count = 0;

//    #ifdef GPRSModem
    modem_waitlist_count++; //count for 10 to 180 sec before modem wakeup
	if(modem_waitlist_count > 32000)
	        modem_waitlist_count = 0;
//    #endif GPRSModem
	//------
	//check if "end of msg": time from last byte > 20 ms
	//------add for modem--------
//	#ifdef Modem_Comm
	if((main_loop_busy == 7) && (Send_SMS_Flag < 8))//if during modem test
	{
        if((postNodata_count >= 1)&&(endRx_msg == 0))
        {
            //"end of msg" condition
            postNodata_count = 0; //reset counter
            endRx_msg = 1; //set the end msg flag
            rx_enable = 0; //reset the rx enable flag for pre no data time
            check_rx_flag = 1; //check the rx msg now
        }
	}
	else
	{
        if((postNodata_count >= 2)&&(endRx_msg == 0))
        {
            //"end of msg" condition
            postNodata_count = 0; //reset counter
            endRx_msg = 1; //set the end msg flag
            rx_enable = 0; //reset the rx enable flag for pre no data time
            check_rx_flag = 1; //check the rx msg now
                //check if it is in modem process, if not continue as normal communication
                //if(Modem_On_Flag == 0)
            //	main_loop_busy = 1; //set the main loop flag to 1
        }

	}
//	#endif Modem_Comm

    if(endRx_msg == 0)//if not "end of msg" - block the pre no data counter
    {
            preNodata_count = 0;
            postNodata_count++;	//20 ms
    }
    //------
	//check if end pre noData time before new msg rx: time from last msg rx > 100 ms
	//------add for modem--------
//	#ifdef Modem_Comm
	if((main_loop_busy == 7) && (Send_SMS_Flag < 8))//if during modem test
	{
	        if((preNodata_count >= 1)&&(rx_enable == 0));
                {
                    //end pre noData time
                    preNodata_count = 0; //reset counter
                    rx_enable = 1; //set the rx enable flag
                }
	}
	else
	{
        	if((preNodata_count >= 10)&&(rx_enable == 0));
        	{
            	        //end pre noData time
            	        preNodata_count = 0; //reset counter
            	        rx_enable = 1; //set the rx enable flag
        	}
	}
//    #endif Modem_Comm

	if(rx_enable == 0)//if not end "pre noData time"
		preNodata_count++;	//100 ms
        //------
	//check if end timeout before new msg tx: time from last msg tx > 100 ms
	//------add for modem--------
//	#ifdef Modem_Comm
	if((main_loop_busy == 7) && (Send_SMS_Flag < 8))//if during modem test
	{
	        if((preTx_count >= 1)&&(tx_enable == 0)) //for modem test
	        {
                    preTx_count = 0; //reset counter
                    tx_enable = 1; //set the tx enable flag
            }
	}
	else
	{
	        if((preTx_count >= 10)&&(tx_enable == 0))

	        {
                    preTx_count = 0; //reset counter
                    tx_enable = 1; //set the tx enable flag
            }
	}
//        #endif Modem_Comm
    if(tx_enable == 0)//if not end timeout before tx
		preTx_count++; 	//100ms delay before next tx to transeiver
	//-------

        //use this part for the connection to net.
//        #ifdef Modem_Comm
//        #ifdef GPRSModem
	    Modem_10mSec_Count++;
        if(Modem_10mSec_Count > 32000) //for safty
                Modem_10mSec_Count = 0;

        Modem_Sec_Count++;
        if(Modem_Sec_Count > 32000) //for safty
                Modem_Sec_Count = 0;

        if(Send_SMS_Flag >= 8)
        {
            //incriment IdleTimeCount
            IdleTimeCount++;
            if(IdleTimeCount > 32000) //for safty
                    IdleTimeCount = 0;
        }
//        #endif GPRSModem
//        #endif Modem_Comm
        //-------

        // preset TIMER0 to: 3686400/1024/36 = 100 cycles/sec (10 m/sec)
        TCNT0 = 0xDB; //(255-36 = 219)
}
*/


/*
//#ifdef GPRSModem
//compare (and fix if corrapted) first sensor ram id with id number in eprom:
void CompareBackupID(void)
{
        BYTE i;

        //disable interrupts
        #asm ("cli")
        delay_ms(1);
        //compare the first 4 id bytes
        for (i=0; i<4; i++)
        {
                //if(backup_unique_id[i] != SensorAddress[i])
                if(unique_id[i] != SensorAddress[i])
                {
                      //SensorAddress[i] = backup_unique_id[i];
                      i = 5; //exit with with ram id corrapted
                }
        }
        //if ram id corrapted then copy again from eprom to ram
        if(i >= 5)
        {
            #ifdef Sens_3_Valid
            cpu_e2_to_MemCopy( SensorAddress, unique_id, 12);
            #else
            cpu_e2_to_MemCopy( SensorAddress, unique_id, 8);
            #endif Sens_3_Valid
        }
        //enable iterrupts
        #asm ("sei")
        delay_ms(1);
}

//#endif GPRSModem
//---------------------------------
*/


////////////////end of general.c////////////////
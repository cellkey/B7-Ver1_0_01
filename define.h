#ifndef DEFINE_H
#define DEFINE_H

#include <mega1280.h>
#include <1wire.h>
#include <ds1820.h>
#include <delay.h>
#include <stdio.h>
//#include <twi.h>
#include <iobits.h>
//extern int  sprintf(char *str, char flash *fmtstr,...);

//-------Board Definitions------------

#define DebugMode       //
//modem type definition
//#define SIM800         //if SIM800 assembled on board
//#define HE910           //Telit G3 modem
//-------------------------------
//#define KANDO_SERVER    //define Kando URL address 050516
//#define RS485_ACTIVE    //if MODBUS sensors attached
//#define SENSOR_MEM_2K    //extended sensor mmemory V6

//#define WATER_EXIST_ACTIVE   //water sensor connected at cover SW
//#define DEMO           //Kando
//#define NO_SERVER        //mark it before release

//#define BAT_MUX_V3      //RED BOARD - BATTERY mux control
//#define AGRICULTURE_TASK   //Tensiometers or other  -30 min. interval
//#define MODEM_EXT_PWR           //Modem pwered by ext power (more than 3.6V)
//#define Evogen_Com_1Min
// #define  GEVIM_ALERT_UNIT           //alert system with cover sw for Gevim
  #define WATER_METER_ACTIVE           //Netiv Haasara version   
  #define DESHEN_WATER_METER          //zero wm counter when long silence
//  #define  NETIV_ALERT_UNIT 
//  #define WEATHER_STATION_485  
//#define GPS_INSTALED
//#define ONE_WIRE_F0
//---------------------------------------------------------------------------
//modems types
#define DEFAULT_MODEM 0 
#define SIM800 1   // AT+CGMM   SIMCOM_SIM800L
#define TELIT 2    // #CGMM: GL865-QUAD
#define  HE910 3

#define BASE_INTERVAL   5

#ifdef AGRICULTURE_TASK
    #define INTERVAL_PARAM	 BASE_INTERVAL * 6        //30 min
     #define MEASURE_INTERVAL  BASE_INTERVAL * 6

#else
   #define INTERVAL_PARAM	 BASE_INTERVAL * 1        //10	//interval parameter: 10 = 10 min
   #define MEASURE_INTERVAL  BASE_INTERVAL * 1
#endif

//Option_2 bit defines
#define eRESTORE_DATA_POINTERS   Option_2[0] //address 0xEF
    #define RELEASE_POINTERS 0
    #define NEED_TO_RESTORE  1  
    


//---------memory size----------------
#ifdef SENSOR_MEM_2K
    //map memory: 2 KB memory per sensor  32b for ctrl prm- 16 used
     #define SENSOR_MEMORY_START     0x00   //0                                         +
     #define SENSOR_MEMORY_SIZE      0x800   //2048
     #define SENSOR_CNTRL_PRM_SIZE   0x20   //32   at the end of the 2048 bytes space
 #else
    //map memory: 1 KB memory per sensor
    #define SENSOR_MEMORY_START     0x00   //0                                         +
    #define SENSOR_MEMORY_SIZE      0x400   //1024
    #define SENSOR_CNTRL_PRM_SIZE   0x10   //16   at the end of the 1024
#endif
//---------------------------------------


//#define QUICK_HEAT
//#define TestMonitor
//#define EC_SM_TMP
//#define EXT_REF
//#define EC_BULK
//#define RTS19
//#define HIVE_SCALE_100

//#ifdef HIVE_SCALE_100
//    #define HIVE_MAX_VAL  11500
//    #define HIVE_MIN_VOLT  0
//    #define HIVE_TAN_ALFA  479
//#else
//    #define HIVE_MAX_VAL  20000
//    #define HIVE_MIN_VOLT  100
//    #define HIVE_TAN_ALFA  922
//#endif HIVE_SCALE_100

#define RTC_WD_VAL 20

#define _BV(bit)   (1<<(bit))

//typedef unsigned char BYTE;
typedef  char BYTE;

typedef union {
    int ival;
    unsigned char bval[2];
} int_bytes;

typedef union {
    long lVal;
    char bVal[4];
} long_bytes;

#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

#define MAX_INT         32767
#define MIN_INT         -32768

//#define MIN_BTR_4_ALERT 7000

//#define MAX_PRM_TASKS   14


// Voltage Reference: 2.56V, cap. on AREF
#define ADC_VREF_TYPE ((1<<REFS1) | (1<<REFS0) | (0<<ADLAR))

// Voltage Reference: AVCC pin
//#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

#define EXT_VREF 		2500	//reference voltage [mV] external
#define INT_VREF 		2560	//reference voltage [mV] internal
#define VCC_VREF        3300

 #define BATTERY_MUX     7  //Analog input PF7

#define TX_BUFFER_SIZE1 64


#define MAX_TX_BUF_LEN      32
#define MAX_RX_BUF_LEN      64 //was 64. change to 100 3/2014 for alerts
#define MAX_RX0_BUF_LEN     256 //was 64. change to 100 3/2014 for alerts
#define MAX_WAIT_MNTR_SEC   10
#ifdef BlueToothOption
#define MAX_WAIT_BLUETOOTH  120
#endif BlueToothOption

#define SUCCESS 1
#define FAILURE 0

#define CONTINUE    1
#define WAIT        2
#define NO_RSPNS    3

#define ALERT_WAIT          1
#define TRHRESHOLD_CROSSED  2
#define ALERT_SHOT          3
#define ALERT_BACK_NORMAL   4
#define SERVER_ALERT_NEEDED        5

//flag status
#define IS_ALERT_NEEDED          0x01
#define IS_ALERT_ACTIVATED       0x02
#define PUMP_ACTIVATION_NEEDED   0x04
#define PUMP_ACTIVAED            0x08
#define WLV_OVF_FLAG             0x10
#define MODEM_ON                 0x20


#define pALERT_DELAY      2   //delay btween measurements when TH crossed
//#define ERR_BUF_SIZE    16
//#define BEGIN_DATA      12		//byte number (start the data in ComBuf)
#define MAX_ICCID_LEN   20

#define TRUE 1
#define FALSE 0
#define ON  1
#define OFF 0

#define LED_1        1
#define LED_2        2
#define LED_3        3

#define LED_OFF     0
#define LED_ON      1
#define LED_BLINK   2

#define LED1_ON     SETBIT(PORTH,7)
#define LED1_OFF     CLRBIT(PORTH,7)
#define TOGGLE_BUZZER()  ((PIND.6 == 1) ? (PORTD.6 = 0) : (PORTD.6 = 1)) 

#define RED_LED_ON     SETBIT(PORTJ,4)
#define RED_LED_OFF    CLRBIT(PORTJ,4)
#define GRE_LED_ON     SETBIT(PORTJ,5)
#define GRE_LED_OFF    CLRBIT(PORTJ,5)
#define BLU_LED_ON     SETBIT(PORTJ,6)
#define BLU_LED_OFF    CLRBIT(PORTJ,6)

#define TOGGLE_BLU_LED()  ((TSTBIT(PINJ,6)) ?  (PORTJ &= ~(1 << 6)) : (PORTJ |= 1 << 6)) 




#define NO_ANSWER       0
#define TASK_COMPLETE   1
#define TASK_FAILED     2

#define TASK_NONE       0

#define TASK_MEASURE    1
#define TASK_MODEM      2
#define TASK_DIRECT     3
#define TASK_SLEEP      4
#define TASK_WAKEUP     5
#define TASK_MONITOR    6
#define TASK_POWER_FAILURE  7
#define TASK_DEBUG    8

#ifdef BlueToothOption
#define TASK_BLUETOOTH   9
#endif BlueToothOption


#define TASK_MODEM_INIT       1
//#define TASK_MODEM_CHK_VOLT   2
#define TASK_MODEM_CONNECT    3
#define TASK_MODEM_POST       4
#define TASK_MODEM_CLOSE      5
//#define TASK_MODEM_POST  5
//#define TASK_CONNECT    3
#define SUB_TASK_INIT_MODEM_ON   11
#define SUB_TASK_INIT_MODEM_OK   12
#define SUB_TASK_INIT_MODEM_REG  13
#define SUB_TASK_INIT_MODEM_REG_GPRS 55    //290517
#define SUB_TASK_INIT_MODEM_RSSI 14
#define SUB_TASK_INIT_MODEM_COPS 15
#define SUB_TASK_MODEM_CHK_ICCID        16
#define SUB_TASK_INIT_MODEM_IGN  17
#define SUB_TASK_INIT_MODEM_GET_COPS    18
#define SUB_TASK_INIT_MODEM_DELAY   19
#define SUB_TASK_INIT_MODEM_MONITOR    20

//#define SUB_TASK_MODEM_CHK_VOLT_IO_ON   21
//#define SUB_TASK_MODEM_CHK_VOLT_ADC     22
//#define SUB_TASK_MODEM_CHK_VOLT_IO_OFF  23

#define SUB_TASK_MODEM_CONNECT_ATCH     31


#define SUB_TASK_MODEM_CONNECT_SETUP1   32
#define SUB_TASK_MODEM_CONNECT_SETUP2   33
#define SUB_TASK_MODEM_CONNECT_SETUP3   34
#define SUB_TASK_MODEM_CONNECT_PDP_DEF  35
#define SUB_TASK_MODEM_CONNECT_ACTV     36
#define SUB_TASK_MODEM_CONNECT_START_DIAL    37
//#define SUB_TASK_MODEM_CONNECT_IS_ACTV  38
#define SUB_TASK_MODEM_CONNECT_DELAY    39

#define SUB_TASK_MODEM_POST_PRM   41
#define SUB_TASK_MODEM_POST_DATA  42
#define SUB_TASK_MODEM_POST_UPD   43
#define SUB_TASK_MODEM_POST_CNFRM   44
#define SUB_TASK_MODEM_FLASH_UPDATE 45
#define SUB_TASK_MODEM_SEND_NOTIFICATION 46
#define SUB_TASK_MODEM_CLOSE_PPP    51
#define SUB_TASK_MODEM_CLOSE_TCP    52
#define SUB_TASK_MODEM_CLOSE_MDM    53
#define SUB_TASK_MODEM_OFF          54

#define DO_DATA 1
#define DO_PARAMS   2
#define DO_DATA_N_PRMS  3

#define TASK_MSR_START_HEAT 1
#define TASK_MSR_READ       2
//#define TASK_MSR_END_HEAT   3
#define TASK_MSR_SAVE       4

#define TASK_MONITOR_CONNECT    1
#define TASK_MONITOR_WAIT   2
//#define TASK_MONITOR_GET_ID     2
//#define TASK_MONITOR_GET_PROP   3
//#define TASK_MONITOR_SET_PROP   4

#define GET_UPDATE      1
#define CONFIRM_UPDATE  0

//------------sensors types definition ----------------------
#define SENSOR1         0     // not more than 10 sensors!!!
#define SENSOR2         1
#define SENSOR3         2
#define SENSOR4         3
#define SENSOR5         4
#define SENSOR6         5
#define SENSOR7         6
#define SENSOR8         7

#define MAX_SEN_NUM     8       //was 8 120316

//sensors' addresses
#define SDI_EC_ADDRESS 3       //Ponsel sensors
#define S485_EC_ADDRESS 30
#define SDI_PH_ADDRESS 2       //Ponsel sensors
#define S485_PH_ADDRESS 20
#define SDI_5TE_ADDRESS 0       //
#define TEMP_HUMID_ADDRESS       7

#define NO_TYPE 0

#define AT  2
#define PH  3                 //PH kando     4-20mA
#define EC_K0  4             //EC kando   4-20mA
#define WLV 5                //Level kando  -4-20mA
#define AH  6
#define TIR 7               //total radiation
#define EC_SDI12        8   //Danny
#define TMP_SDI12       9
#define PH_SDI12        10
#define COMBO2_EC_SDI12  11    //EC+temp  or PH+TEMP
#define COMBO4_SDI12    12    //EC+TEMP+PH +ORP on same connector and power
#define COMBO_5TE       13      //5TE sensor
#define COMBO2_PH_SDI12 14   //PH+temp+ORP
#define WLV_U 15            //Level kando
#define CT    16
#define LT    17
#define DENDROMETER    18     //dendromer Deshen Gat
#define HS10  19
#define BATTERY         20
#define PUMP            21       //Danny 040315
#define ST5   22
#define TNS   23    //0-2 volt
#define AT_B  24  //tmp Boaz   1wire
#define DOROT_PRESSURE  25  //4-20mA   0-10 bar
#define TNS_tronix 26     //0x1A

//----shkila-----------
//#define SCAN_SHKILA  26       //shkila Scanner
//#define SCALE_SHKILA 27       //shkila sCale
#define GPS_SHKILA   28       //shkila GPS
//-------------------------

#define GS1   30
#define SM    31
#define EC    32
#define WLV_PULSTAR 33   //new water level 100117
#define TMP   34
#define OXGN  36
#define H2S   37        //290317 Kando
#define DW_LE  38 // use to be  37   //Kando-water existance - dry contact Spain

#define RAINMTR 39
#define WTRMTR  40   //water meter 28h
#define MATIC   41
#define SM_4_20 42     //elchanani
#define TSS     43
 //-------------Weather_Sation--------------
#define Weather_Sation_485 44
#define UV_Sensor    45     //#define TIR 7
#define Light_Sensor 46
#define Wind_Speed   47
//#define AT  2
//#define AH  6
//#define TIR 7
//#define RAINMTR 39
//------------------------------------------
#define COMBO4_485    48   //0x30 - EC+TEMP+PH +ORP on same connector and power
#define COMBO_EC_485    49  //0x32 EC+Temperature
#define COMBO_PH_485    50  //0x32  PH+ORP

//Evogen 4-20mA sensors
#define TRE_150 51       //evogen temp 4-20mA
#define TRH_300 52      //evogen Humidity 4-20mA
#define HD2021T 53       //evogen radiation 4-20mA


#define SCALE_SHKILA_TCS60 54  
#define COMBO_TEMP_1W    55     //two 1W sensors on bus
#define	  TEMP_HUMIDITY  	   56  //0x38 and 0x39 modbus sensor 

#define DIGIN1 60      //digital input - PCINT0
#define DIGIN2 61
#define DIGIN3 62
#define DIGIN4 63

//--------------End sensors types------------------------

#define MAX_IO_TYPES    7

//PRR0 bits

#define PRR_TWI    7
#define PRR_TMR2   6
#define PRR_TMR0   5

#define PRR_TMR1   3
#define PRR_SPI    2
#define PRR_UART0  1
#define PRR_ADC    0

 //PRR1 BITS
#define PRR_TMR5   5
#define PRR_TMR4   4
#define PRR_TMR3   3
#define PRR_UART3   2
#define PRR_UART2   1
#define PRR_UART1   0



#define PRR0_SHUTOFF()  (PRR0 = 0xFF);//(1<<PRR_TWI)|(1<<PRR_TMR2)|(1<<PRR_TMR0)|(1<<PRR_UART1)|(1<<PPRR_TMR1)|(1<<PRR_SPI)|(1<<PRR_UART0)|(1<<PRR_ADC));
#define PRR1_SHUTOFF()  (PRR1 = 0xFF);
#define PRR0_ALL_ON()  (PRR0 = 0x04);
#define PRR1_ALL_ON()  (PRR1 = 0x00);

#define PRR0_EN_TWI()  (PRR0 &= ~(1<<PRR_TWI));
#define PRR0_EN_TMR2()  (PRR0 &= ~(1<<PRR_TMR2));
#define PRR0_EN_TMR0()  (PRR0 &= ~(1<<PRR_TMR0));
#define PRR1_EN_UART1()  (PRR1 &= ~(1<<PRR_UART1));
#define PRR1_EN_UART2()  (PRR1 &= ~(1<<PRR_UART2));
#define PRR0_EN_TMR1()  (PRR0 &= ~(1<<PRR_TMR1));
#define PRR0_EN_SPI()  (PRR0 &= ~(1<<PRR_SPI));
#define PRR0_EN_UART0()  (PRR0 &= ~(1<<PRR_UART0));
#define PRR0_EN_ADC()  (PRR0 &= ~(1<<PRR_ADC));



#define ENABLE_CLOCK_INT()   (EIMSK |= (1<<INT2));    // enable  external interrupt (clock int)
#define DISABLE_CLOCK_INT()  (EIMSK &= ~(1<<INT2));   // disable  external interrupt (clock int)

#define ENABLE_EXT_INT3()    (EIMSK |= (1<<INT3));    // enable  external interrupt 3  (Netiv ver)
#define DISABLE_EXT_INT3()   (EIMSK &= ~(1<<INT3));   // disable  external interrupt 3

#define ENABLE_EXT_INT1()    (EIMSK |= (1<<INT1));    // enable  external interrupt (rain meter)
#define DISABLE_EXT_INT1()   (EIMSK &= ~(1<<INT1));   // disable  external interrupt (rain meter)

#define ENABLE_TIMER1_COMPA()   ( TIMSK1 |= (1<<OCIE1A));
#define DISABLE_TIMER1_COMPA()  ( TIMSK1 &= ~(1<<OCIE1A));   //danny

#define ENABLE_TIMER0()    (TCCR0B=(0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00));
#define DISABLE_TIMER0()   (TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00));
#define ENABLE_SDI_TIMER0()    (TCCR0B=(0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00));// (TCCR0B = 0x05); 17.8mS cycle period 14400 hrz 

#define ENABLE_TIMER1()      TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10);
#define DISABLE_TIMER1()   (TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10));

#define ENABLE_TIMER2()    (TCCR2B=(0<<WGM22) | (1<<CS22) | (1<<CS21) | (1<<CS20));  //(7) = 3.6 4khz danny
#define DISABLE_TIMER2()   (TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20));

 #define ENABLE_TIMER4()  (TCCR4B=(0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (0<<WGM42) | (0<<CS42) | (0<<CS41) | (1<<CS40));  //1ms
 #define DISABLE_TIMER4()  (TCCR4B=(0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (0<<WGM42) | (0<<CS42) | (0<<CS41) | (0<<CS40));  //1ms

//PIN CHANGE INTS CONTROL - MS1, MS2, COVER
#define ENABLE_PB0_INT()  SETBIT(PCMSK0,PCINT0);  //portb.0 MAGNETIC SW 2
#define DISABLE_PB0_INT() CLRBIT(PCMSK0,PCINT0);

#define ENABLE_PB3_INT()  SETBIT(PCMSK0,PCINT3); //(PCMSK0 |= (1<<PCINT3) );  //portb.3  mag sw 1
#define DISABLE_PB3_INT() CLRBIT(PCMSK0,PCINT3);

#define ENABLE_PB5_INT()  SETBIT(PCMSK0,PCINT5); //(PCMSK0 |= (1<<PCINT3) );  //portb.3  mag sw 1
#define DISABLE_PB5_INT() CLRBIT(PCMSK0,PCINT5);

#define ENABLE_PJ2_INT()  SETBIT(PCMSK1,PCINT11);  //portJ.2  Ext. SW
#define DISABLE_PJ2_INT()  CLRBIT(PCMSK1,PCINT11); //(PCMSK1 = 0); 




#define ENABLE_UART0()      (UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80));
#define DISABLE_UART0()     (UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80));

#define ENABLE_UART2()      (UCSR2B=(0<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82)); //no rx int !!
#define DISABLE_UART2()    ( UCSR2B=(0<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (0<<RXEN2) | (0<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82));

#define ENABLE_RX_INT_UART0()  (UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80));
#define DISABLE_RX_INT_UART0() (UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80));

#define ENABLE_RX_INT_UART2()  (UCSR2B=(1<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82));
#define DISABLE_RX_INT_UART2() (UCSR2B=(0<<RXCIE2) | (0<<TXCIE2) | (0<<UDRIE2) | (1<<RXEN2) | (1<<TXEN2) | (0<<UCSZ22) | (0<<RXB82) | (0<<TXB82));

#define ENABLE_RX_INT_UART3() (UCSR3B=(1<<RXCIE3) | (0<<TXCIE3) | (0<<UDRIE3) | (1<<RXEN3) | (1<<TXEN3) | (0<<UCSZ32) | (0<<RXB83) | (0<<TXB83));
#define DISABLE_RX_INT_UART3() (UCSR3B=(0<<RXCIE3) | (0<<TXCIE3) | (0<<UDRIE3) | (1<<RXEN3) | (1<<TXEN3) | (0<<UCSZ32) | (0<<RXB83) | (0<<TXB83));


 #define ENABLE_ADC()  (ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));
//#define ENABLE_ADC()  (ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0));
#define DISABLE_ADC() (ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0));

//MEGA1280 VERSION
#define WATCHDOG_ENABLE_STEP1()  WDTCSR|=(1<<WDCE) | (1<<WDE);
#define WATCHDOG_ENABLE_STEP2()  WDTCSR=(0<<WDIF) | (0<<WDIE) | (1<<WDP3) | (0<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);

//#define WATCHDOG_ENABLE() 	   	(WDTCSR = 0x0F) //WDE = 1 WDT = 2048 cycles
#define WATCHDOG_OFF_ENABLE()  	(MCUSR &= ~(1 << WDRF))
#define WATCHDOG_PRE_DISABLE()  (WDTCSR |= (1 << WDCE) | (1 << WDE))
#define WATCHDOG_DISABLE() 		(WDTCSR = 0x00)

#define TWI_DISABLE()    TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);
#define TWI_ENABLE()    TWCR=(0<<TWEA) | (0<<TWSTA) | (1<<TWSTO) | (0<<TWEN) | (0<<TWIE);

//Creacell board 3
#define VOLTAGE_MULTIPLIER_ON()    SETBIT(PORTD,5) //(PORTD.5 = 1) // CREACELL VOLTAGE MULTI
#define VOLTAGE_MULTIPLIER_OFF()  CLRBIT(PORTD,5)//(PORTD.5 = 0)
#define V33_PWR_ON()              SETBIT(PORTD,4)   // CREACELL 3.3V on
#define V33_PWR_OFF()             CLRBIT(PORTD,4)
#define MODEM_MULT_PWR_ON()        SETBIT(PORTD,7//)(PORTD.7 = 1)   // CREACELL Modem pwr mult from battery on
#define MODEM_MULT_PWR_OFF()      CLRBIT(PORTD,7)//(PORTD.7 = 0)
//#define PWR_SENS3_ON()              ( PORTD.4 = 1)
//#define PWR_SENS3_OFF()             ( PORTD.4 = 0)

#define MUX2PC()                  (PORTE.2 = 0; PORTE.3 = 0)
#define MUX_PIN_0(PIN)           (CLRBIT(PORTE,PIN))
#define MUX_PIN_1(PIN)           (SETBIT(PORTE,PIN))

#define SET_SDI12_TX_DIRECTION()  SETBIT(PORTC,7)   //3st buffer direction selection bit
#define SET_SDI12_RX_DIRECTION()  CLRBIT(PORTC,7)    //3st buffer direction selection bit

//==========================made above======================



//#define pRead_BU_ADDRESS        7184   //0x400 * 7 + 16     120316

#define WEIGHING_HEAT_TIME      30
#define BATTERY_HEAT_TIME       15
#define VISHAY_HEAT_TIME        30
//#define  PUMP_HEAT_TIME         450   //according to Kando

#define CONNECTING_MINUTE     1   // Changd by Danny - 310115 the 1st minutes is the c

#define RETRY_CONNECTING_TIME    30   //Danny - was 15

#define GET_REQUEST 0
#define SET_REQUEST 1

#define REQ_ID      0
#define REQ_MSR     10
#define REQ_INTRVL  20
#define REQ_TYPE    30
#define REQ_TIME    40
#define REQ_IP      41
#define REQ_PORT    42
#define REQ_APN     43
#define REQ_MCC     44
#define REQ_MNC     45
#define REQ_ROAMING 46
#define REQ_SCH     47  // START CONNECT HOUR
#define REQ_CPD     48  //CONNECTS PER DAY
#define REQ_CI      49  //CONNECTS INTERVAL
#define REQ_PSWD    50
#define REQ_OPTION1 51
#define REQ_OPTION2 52
#define REQ_OFFSET  53
#define REQ_GAIN    54
#define REQ_BTR     55
#define REQ_RSSI    56
//#define REQ_GMT     57
#define REQ_VER     58
#define REQ_NUM_SEN     59
#define REQ_COMBINATION 60
#define REQ_DISCNCT   61
//#define REQ_LIMITS      62
#define REQ_MSR_NOW   63


//Errors codes definition (note - err_buf real info started at position [1])
//the error code order is in roole: most importent error ->larger code num.
/*#define NO_ERROR        0x00
#define EXT_RESET       0x01
#define POWERDOWN_RESET 0x02
#define WATCHDOG_RESET  0x03
#define T_CIRC_FAIL     0x04
#define T_LVDT_FAIL     0x05
#define SENS_1_LO       0x06
#define SENS_1_HI       0x07
#define SENS_2_FAIL     0x08
#define SENS_1_FAIL     0x09
#define EE_FAIL         0x0A
#define CLOCK_FAIL      0x0B
#define RADIO_FAIL      0x0C
#define LO_BATT         0x0D
*/
//#ifdef DebugMode
void putchar1(char c);
void  _putchar1(char c);
void _putchar0(char c);
//#endif DebugMode
/////////////////////////////////////////////
// Data_manager functions
////////////////////////////////////////////
//declare functions prototypes
//the function will copy one data block in ext_e2 (pBread address)
//into 'sensDataBlock' buffer in ram
//return 1 (succsess) or 0 (failure)
//char CopyBlockIntoRam();

//open new data block
//char CreateNewDataBlock();

//arrange the saving of sensors measurments results in the external eeprom
//the function return (1 = success) or (0 = failue)
char SaveMeasurments();

char GetMeasurments(char read_mode);
//set next pBread position
//end of reading space is the first block (including first block)
//return 1 if there is new pBread address or 0 if not
//char NextpReadAddress();

char InitDataBlocks(char interval);

char SetNewInterval(char new_interval);

char PointersValidate();

//char ResetReadPointer();
char _GetMeasurments_(char read_mode);
/////////////////////////////////////////////
// i2c_bus functions
////////////////////////////////////////////
//void SendStartBit(void);

//void SendStopBit(void);

//void SendByte(unsigned char send_byte);

//unsigned char TestAck(void);

//void SendAck(void);

//unsigned char RecByte(void);

//unsigned char SendBuf(unsigned char adress, int length, unsigned char  *buffer);

//unsigned char GetBuf(unsigned char adress, int length, unsigned char *buffer) ;

//unsigned char GetBufClk(unsigned char adress, int length, unsigned char *buffer) ;

/////////////////////////////////////////////
// twi functions
////////////////////////////////////////////
/*
void twiInit (void);

int twiWriteData(unsigned char address, int dataLen,  const unsigned char *data);

void twiReadData(unsigned char address, int dataLen,   unsigned char *data);
*/
/////////////////////////////////////////////
// general functions
////////////////////////////////////////////

void WakeUpProcedure(void);

void PowerDownSleep ( void );

void WDT_off(void);

BYTE CheckSum( BYTE *buff, BYTE length, BYTE param );

void TransmitBuf(char iPortNum);

void InitProgram( void );

void InitVarsForConnecting();

int CheckResult();

//void CheckSen();

//void ReadSensor();

//void CheckBtrVolt();
//copy from cpu e2 into buf
void cpu_e2_to_MemCopy( BYTE* to, char eeprom* from, BYTE length);

void cpu_flash_to_MemCopy( BYTE* to, char flash* from, BYTE length);

BYTE CopyFlashToBuf( BYTE* to, char flash* from);

void MemCopy( BYTE* to, BYTE* from, BYTE length);

//the function will set address into 'pSens_Ctrl_Params' and 'pSens_cpu_e2'
void SetCotrolParamAddress();

int bytes2int( char* buf);

void int2bytes(int int_pointer,  char* buf);
long Bytes2Long(char* buf);

void Long2Bytes(long l, char* buf);

BYTE IsTimeToMeasure(void);

BYTE IsTimeToConnectGPRS();
char ReadCtrlParamsToRam();
char ResetAllReadPointers();
//char ResetAllReadPointerspBwrite();
//void ResetModem (void);

//copy from ram buf into cpu e2
void MemCopy_to_cpu_e2( char eeprom* to, BYTE* from, BYTE length);

void TurnOnLed(BYTE led, BYTE type);

void TurnOffLed();

/////////////////////////////////////////////
// globals functions
////////////////////////////////////////////

//arrange the clock buf before setiing time into rtc
//void SetClockBuf(void);

//power on initialization ruthin
//check if power flag is on, if yes you should preform reset to r.t.cclock
//unsigned char IsPowerFlagOn(void);

//unsigned char ResetCommand(void);
//void ResetCommand(void);

//unsigned char DisableClockIntr(void);
//void DisableClockIntr(void);

//unsigned char SetRealTime(void);

//read the clock data into buffer
//unsigned char ReadTime(void);

//initiate the rtc at program startup
//char InitRTC();
//void InitRTC();

//unsigned char ResetClockIntr(unsigned char);
//void ResetClockIntr();

//void GetRealTime(void);
void rtc_set_timeAll(char *clockBuf); //Danny
void rtc_get_timeAll(char *clockBuf);
void ActivePump(void);
void DeActivePump(void);
unsigned char RTC_CDI_init(unsigned int interval);

//void SetRtc24Hour();
//#ifdef DebugMode     //  Danny
void SendDebugMsg(flash  char *bufToSend);


void PrintNum(long val);
//void  _putchar1(char c);

//#endif DebugMode     //Danny
/////////////////////////////////////////////
// eeprom functions
////////////////////////////////////////////

char e2_readSeqBytes(unsigned int address, char read_length);

char e2_writePage(unsigned int address, char write_length, char* string_1);

/////////////////////////////////////////////
// modem_manager functions
////////////////////////////////////////////

void ModemMain(void);

//void ParseModemResponse();

void SetModemPwrOn();
void ModemIgnit(void);
//
//void SetModemOff();

//void SendATCmd(flash unsigned char *bufToSend);

/////////////////////////////////////////////
// measure functions
////////////////////////////////////////////
void MeasureMain(void);

unsigned int AnalogSensRead();

//BYTE SetSensorCombination(BYTE* );

BYTE SendAlerts();

//BYTE SetSensorType(BYTE IOindex, char newType);

/////////////////////////////////////////////
// Monitor_manager functions
////////////////////////////////////////////

//void MonitorMain();

#ifdef BlueToothOption
/////////////////////////////////////////////
// Bluetuth_manager functions
////////////////////////////////////////////

void BlueToothMain();

char IsBlueToothConnect();

#endif BlueToothOption
/////////////////////////////////////////////
// Each sensors combination general functions
////////////////////////////////////////////

void SetSensorHeatTime();

int AnalyzeSensorRealValue(int);

BYTE GetSensorType();
BYTE GetSensorIOType();

void SensorPowerOn();

void SensorPowerOff();

void InitAdc(void);
/////////////////////////////////////////////
// w1_ds1820 functions:
/////////////////////////////////////////////
int w1_SensRead(char index);
void BatLevel_ON(void);
void BatLevel_OFF(void);
void SendPostData(/*BYTE readMode*/);
char RTC_Update_by_Modem(void);
// int PHcalc(int value);
// int ECcalc(int value);
// int EC_mV_calc(int value);
// int PH_mV_calc(int value);
 unsigned int STR2BIN(unsigned char *str);
 void Set_RTC_WD_Val(unsigned char WDtime);
 unsigned char Set_RTC_WD(void);
 void SetMUX2PC(void);
void Num2String_Modem(long val);

 //IO expender
// unsigned char ReadIOExpReg(unsigned char reg);

 unsigned char ReadMode (void);

char Pharse_params_struct(char *str,unsigned int index);
//char Pharse_SDIparams_struct(char *str, char index);
void ResetPumpFlags(void);
 void Buzer_it(char loops, unsigned int duration, char PwrOn);
  void CloseSuccessfulCom(void);
  int Read_Scale_TCS60(void);  
// char  ReadSaveGPSSensor(void);
// char StoreOldReadPoiters(char num);
//#endif EC_SM_TMP

#endif DEFINE_H
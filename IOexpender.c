
#include <Delay.h>
#include "twi.h"
#include "define.h"
#include <iobits.h >


////new expender
//#define IOEXP_ADD       0x40       //expender address. add 0/1 for R/W
//#define IODIR           0x00       //reg address
//#define IPOL            0x01       //reg address
//#define GPINTEN         0x02       //reg address
//#define DEFVAL          0x03      //reg address
//#define INTCON          0x04      //reg address
//#define IOCON           0x05      //reg address
//#define GPPU            0x06     //reg address
//#define INTF            0x07      //reg address
//#define INTCAP          0x08      //reg address
//#define GPIO            0x09      //reg address
//#define OLAT            0x0A      //reg address

#define REL1_CLOSE_bit_input()  CLRBIT(DDRH,5)
#define REL1_CLOSE_bit_output() SETBIT(DDRH,5)
#define REL1_OPEN_bit_input()   CLRBIT(DDRH,4)
#define REL1_OPEN_bit_output()  SETBIT(DDRH,4)
#define REL2_CLOSE_bit_input()  CLRBIT(DDRH,3)
#define REL2_CLOSE_bit_output() SETBIT(DDRH,3)
#define REL2_OPEN_bit_input()   CLRBIT(DDRH,2)
#define REL2_OPEN_bit_output()  SETBIT(DDRH,2)

#define REL2_CLOSE_bit_high()  SETBIT(PORTH,3)     //ph5
#define REL2_CLOSE_bit_low()   CLRBIT(PORTH,3)     //ph5
#define REL2_OPEN_bit_high()    SETBIT(PORTH,2)        // ph4
#define REL2_OPEN_bit_low()     CLRBIT(PORTH,2)         // ph4

#define REL1_CLOSE_bit_high()     SETBIT(PORTH,5)        //ph2
#define REL1_CLOSE_bit_low()     CLRBIT(PORTH,5)        //ph2
#define REL1_OPEN_bit_high()     SETBIT(PORTH,4)     //ph3
#define  REL1_OPEN_bit_low()     CLRBIT(PORTH,4)       //ph3


#define EN_BAT_READ_bit_high() SETBIT(PORTH,6);
#define EN_BAT_READ_bit_low() CLRBIT(PORTH,6);


//#define TWSR (*(unsigned char *) 0xb9)

extern void twiError (uint8_t expected, uint8_t received);
extern char AlertStatus[];
void RTCrePwr (void);
void PUMP_FORWARD(void);

//config expender default setting
//adapted to new IOEXP 17/03/15
//void SetExpender(void)
//{
//     char data;
//     data =  twiWriteReg (IOEXP_ADD, IODIR, All_Inputs); config IOs as intputs
//     data =  twiWriteReg (IOEXP_ADD, IOCON, 0x20); config as no sequential read/write
//     if(data == -1) twiError(0x20, TWSR);                          //error - let me know about it
//}
//void  SetExpender(void)
//{
//      twiWriteReg (IOEXP_ADD, IODIR, All_Inputs); //config IOs as intputs
//      twiWriteReg (IOEXP_ADD, OLAT, All_IOs_Low);
//      twiWriteReg (IOEXP_ADD, IOCON, 0x20); //config as no sequential read/write
//   //  RTCrePwr();  //test only
//}


//adapted to new IOEXP 17/03/15
//void SetExpenderAllInputs(void)
//{
//
//      twiWriteReg (IOEXP_ADD, IODIR, All_Inputs); //config IOs as intputs
//   //  if(data == -1) twiError(0x20, TWSR);                          //error - let me know about it
//}

//void ResetModem (void)
//{
//
//
// }



//adapted to new IOEXP 17/03/15
void CloseRelay1 (void)
{


      V33_PWR_ON();
     VOLTAGE_MULTIPLIER_ON();
      delay_ms(100);
      REL1_CLOSE_bit_high();
      delay_ms(40);
      REL1_CLOSE_bit_low();

  //   VOLTAGE_MULTIPLIER_OFF();
}

void CloseRelay2 (void)
{

     V33_PWR_ON();
     VOLTAGE_MULTIPLIER_ON();
      delay_ms(100);

    REL2_CLOSE_bit_high();
    delay_ms(40);
    REL2_CLOSE_bit_low();

 //   VOLTAGE_MULTIPLIER_OFF();

}

//adapted to new IOEXP 17/03/15
void OpenRelay1 (void)
{
       DDRD.7 = 0 ;    //modem pwr 3st  - hw bug version 6.1
       PORTD.7 = 0;

     V33_PWR_ON();
     VOLTAGE_MULTIPLIER_ON();
     delay_ms(100);
     REL1_OPEN_bit_high();
      delay_ms(40);
     REL1_OPEN_bit_low();

  //   VOLTAGE_MULTIPLIER_OFF();

}


//adapted to new IOEXP 17/03/15
void OpenRelay2 (void)
{

     V33_PWR_ON();
    VOLTAGE_MULTIPLIER_ON();
    REL2_OPEN_bit_output(); 
    
      delay_ms(100);
     REL2_OPEN_bit_high();
    delay_ms(40);
    REL2_OPEN_bit_low();

     REL2_OPEN_bit_input();  //porth.2 in 3st at v6.1
  //  VOLTAGE_MULTIPLIER_OFF();

}
void PUMP_BACKWARD(void)
{

     V33_PWR_ON();
     VOLTAGE_MULTIPLIER_ON();

      delay_ms(100);
     REL2_OPEN_bit_high();
     delay_ms(40);
     REL2_OPEN_bit_low();

  //  VOLTAGE_MULTIPLIER_OFF();

}
void PUMP_FORWARD(void)
{

     V33_PWR_ON();
     VOLTAGE_MULTIPLIER_ON();
      delay_ms(100);

    REL2_CLOSE_bit_high();
    delay_ms(40);
    REL2_CLOSE_bit_low();

  //  VOLTAGE_MULTIPLIER_OFF();

}


//adapted to new IOEXP 17/03/15
//activation of switch of battery voltage measurement
void BatLevel_ON(void)
{

//     V33_PWR_ON();
         SETBIT(DDRH ,1);
         EN_BAT_READ_bit_high();

}

//adapted to new IOEXP 17/03/15
void BatLevel_OFF(void)
{

       EN_BAT_READ_bit_low();

}

//Fix rutine when possible. no delay
void ActivePump(void)
{
    PUMP_FORWARD();  // CloseRelay2();
     CloseRelay1();        //v7
}

void DeActivePump(void)
{

    OpenRelay1();      //v7 BOARD
    PUMP_BACKWARD();    //  OpenRelay2();
}




unsigned char ReadMode (void)
{
      unsigned char data;

       data =  PINB;  //pb5
       if ((data & 0x20) == 0x20)      //mode on bit 5
       return 1;
       return 0;
}


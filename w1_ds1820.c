// w1_ds1820.c file
//-----------------------
//DS1820 temp sensor measurments 07/01/2001


//#include <1wire.h>
//#include <ds1820.h>
 #include <stdio.h>
 #include <string.h>
 #include "define.h"
#include "uart.h"

//port and pin selected on menu  Project-->Configure-->C Compiler -->Libraries -->1 Wire.
//currently defined on PORTA.0
//selest port and pin for the 1w connection
//portA = 1B; portB = 18; portC = 15
//#asm
//   .equ __w1_port=0x02
//   .equ __w1_bit=1
//#endasm
//change from bit 1
//--------------------------
#pragma used+
unsigned char w1_init(void);
unsigned char w1_read(void);
unsigned char w1_write(unsigned char data);
unsigned char w1_search(unsigned char cmd,void *p);
unsigned char w1_dow_crc8(void *p,unsigned char n);
#pragma used-
//-----------------------------
//#define xtal	3686400	// quartz xtal frquency cycles/sec
//#define baud	9600
//-----------------------------
 //Declare global variables
  char temp_ubrr, temp_ucr; 
  
extern unsigned char ds1820_rom_codes[3][9];
extern  unsigned char ds1820_devices;
//extern void ShowHexString( char *message, char Lengh);

// extern struct __ds1820_scratch_pad_struct
// {
//       unsigned char temp_lsb,temp_msb;
//       signed char   temp_high,temp_low;
//       unsigned char res1,res2,
//                     cnt_rem,cnt_c,
//                     crc;
//} __ds1820_scratch_pad;          
//--------------------------------
// DS1820 Temperature Sensor functions
//--------------------------------

//int ds1820_temperature_10(unsigned char *addr)
//{
//     
//     
//       
//     if (w1_init()==0)  
//     {   
//       SendDebugMsg("w1_init() error..\r\n\0");
// 		return 9999;  
//      }   
//
//    if (ds1820_select(addr)==0 )
//    {
//         SendDebugMsg("ds1820_select error..\r\n\0");
//         return -9999;     
//    } 
// //   w1_write(0xcc);  //command for all devices
//    w1_write(0x44);
//    delay_ms(1000); 
//    
//    if (ds1820_read_spd(addr)==0)
//     return -9999;   
//        
//    w1_init();   
//    
////    return (((int)__ds1820_scratch_pad.temp_msb<<8)|
////           __ds1820_scratch_pad.temp_lsb)*5; 
//           
//           return (((int)__ds1820_scratch_pad.temp_msb<<8)|  //no mult by 5!
//           __ds1820_scratch_pad.temp_lsb);
//}
//
//// //only 1 sensor handled  here!
////int ds1820_temperature_10(unsigned char *addr)
////{
////	unsigned char i;
////	 char *p;
////    
////    if( ds1820_devices == 0)
////    return 9999;
////    
////	if (w1_init()==0)  
////    {   
//////       SendDebugMsg("w1_init()1 error..\r\n\0");
//// 		return 9999;  
////    }   
////	w1_write(0xcc);  //command for all devices
////	w1_write(0x44);  //start measure command
////	delay_ms(1000);
////
////   if (w1_init()==0)  
////    {   
//////       SendDebugMsg("w1_init()2 error..\r\n\0");
//// 		return 9999;  
////    }   
////	w1_write(0xcc);
////	w1_write(0xbe);   //read eeprom of device 
////    
////	i=0; 
////  
////	p=(char *) &__ds1820_scratch_pad;       
////	do
////     {
////  		*(p) = w1_read(); //fill __ds1820_scratch_pad 9 bytes
//////         delay_ms(2);
//////         ShowHexString( p,1);  //debug
////         p++;
////	}
////    while (++i < 9);
////
////	w1_init();  
////    
//////	return (((int)__ds1820_scratch_pad.temp_msb<<8)| __ds1820_scratch_pad.temp_lsb)*5;  
////    	return (((int)__ds1820_scratch_pad.temp_msb << 8)| __ds1820_scratch_pad.temp_lsb); //no mult by 5
////    
////}


//set the uart for the 1w operation
void w1_setup(void)
{
    //copy ubrr into temp (set temp into ubrr while not using the 1w)
  	temp_ubrr = UBRR0L; 
    
  	//initialize the UART's baud rate 9600
  	UBRR0L = 0x5F;

  	//copy ucr into temp (set temp into ucr while not using the 1w)
  	temp_ucr = UCSR0B;
  	//initialize the UART control register:
   	//TX enabled, no interrupts, 8 data bits
  	UCSR0B = 8;
}

int w1_SensRead(char index)
{
	int w1_res;
      #asm("cli");
 	w1_setup();
	delay_ms(1000);

//	w1_res = ds1820_temperature_10(&index);  //0   ??????????  ds1820_rom_codes  
 	w1_res = ds1820_temperature_10(&ds1820_rom_codes[index][0]); 
//  	w1_res = ds1820_temperature_10();  //0  
      #asm("sei");
	//return uart to original setup
 	UBRR0L = temp_ubrr;
  	UCSR0B = temp_ucr;

  	//set the power off for the selected input
	return w1_res;
}


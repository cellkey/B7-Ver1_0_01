//////////////////////////////////////////////////////
///////// start 64Kb eeprom implementation /////////
#include "define.h"
#include "twi.h"
#include "uart.h"

//declare local & global variables
unsigned char eepromWriteBuf[16]; 	//buffer for eeprom write operation
unsigned char e2cmdByte;			//buffer for the eeprom command byte
extern char e2_writeFlag;
extern unsigned char eepromReadBuf[16];	//buffer for eeprom read operation

//command buffer configuration: code=1010; blockNum=0 to 7; lsb=1
//char setReadCmd(char blockNum) {return 0xA1 | (blockNum<<1);}
//char setWriteCmd(char blockNum) {return 0xA0 | (blockNum<<1);}

//write address eeprom page
char e2_writePage(unsigned int address, char write_length, char* string_1)
{

    char i;
    e2_writeFlag = 1; //block use of i2c by clock

    eepromWriteBuf[0] = (unsigned char)((address >> 8) & 0xFF);     //address high
    eepromWriteBuf[1]  = (unsigned char)(address) ;                 //address low

    //set the eeprom block num to be writen to
//??    address >>= 8;
    //blockNum = (char)address;

    SPCR=0x00; //reset spi control register
    e2cmdByte = 0xA0;  // | (blockNum<<1);//set write command

    //copy data to be write onto eeprom into buffer
    for(i = 2; i < (write_length +2); i++)
        eepromWriteBuf[i] = string_1[i-2];

    //send buffer to be writen on eeprom
//    if(SendBuf(e2cmdByte , i, eepromWriteBuf) == FALSE)  //if SendBuf function faild

     if( twiWriteEEP(e2cmdByte , i, eepromWriteBuf) == -1)  //   TWI proc
     {
		return FAILURE;
     }
    delay_ms(10);
    e2_writeFlag = 0; //un block use of i2c by clock
    return SUCCESS;
}

//read sequential address eeprom bytes
char e2_readSeqBytes(unsigned int address, char read_length)
{

    e2_writeFlag = 1;   //block use of i2c by clock
    SPCR=0x00;          //reset spi control register

     if(twiReadEEP(address , read_length, eepromReadBuf) == -1)//send internal address (int) and work on it at twi
    {
        return FAILURE;
    }
    e2_writeFlag = 0;   //un block use of i2c by clock
	return SUCCESS;
}
///////// end of 24c16b eeprom implementation /////////

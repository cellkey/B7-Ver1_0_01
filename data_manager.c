//smart_data_manager.c file (update 03-01-01)
///////// start of data manager module /////////

#include <stdlib.h>
#include <string.h>
#include "define.h"

//define global variables
unsigned int pBwrite;		//pointer to current write sensor data block in ext_e2
unsigned int pBread;		//pointer to last read sensor data block in ext_e2
unsigned int pBfirst;		//pointer to first write sensor data block in ext_e2
unsigned int pWrite;		//pointer to last sensor data write in ext_e2
unsigned int pStart_space;	//pointer to start sensor data space in ext_e2
unsigned int pEnd_space;	//pointer to end sensor data space in ext_e2
bit Illigal_Time_Val = FALSE;;
bit  New_pBread_setting_needed = FALSE; //when com ended OK move pointer to next location


extern unsigned int pSens_Ctrl_Params;	//pointer to current sensor control parameters in ext_e2
unsigned int last_cycle_min;//last cycle data store time in minutes from day start
unsigned int pOriginalReadBlock;
extern eeprom char cpue2_interval_1;
extern eeprom BYTE NumSensors;
extern eeprom  unsigned int Option_1[];
extern eeprom  char eDataPtrSave[];
//extern bit bMoreDataToSend;
extern char cuurent_interval;  	//varible to hold the the current measuring interval
extern char e2_writeFlag;
//extern BYTE LengthOfData;
extern unsigned char eepromReadBuf[16];	//buffer for eeprom read operation
//extern char err_buf[ERR_BUF_SIZE];
extern char readClockBuf[7];	         //buffer for data reading from clock
//extern char clockBuf[7]; 		 //buffer for all clock operation need
extern BYTE objToMsr;
extern unsigned int time_in_minutes;     //time from day start in ninutes
extern char ComBuf[MAX_TX_BUF_LEN];
extern int measure_time;
extern int SensorResult;       //save the measuring result into variable
extern char DataBlock[48];
extern unsigned char nUnreadBlocks;
extern BYTE msrAlertFlag;
extern bit PumpActivated;
extern bit QuickSensorsCheckNeeded;
extern void ShowHexString(unsigned char *message, char Lengh);
extern void rtc_get_time(unsigned char *hour,unsigned char *min, unsigned char *sec);
extern void UART_WriteMsg( char *InStr);




//   eeprom struct PolutionWeigh
//               {
//
//                    int SensWeigh[15];
//               }  WeighArr[6];


//write 2 bytes data into ext_e2
int WriteIntoExte2(int intData, int intAddress, char sensorMeasur)
{
	 unsigned char helpBuf[2];
	int address;


	address = intAddress;
	//check if next writing point will cross e2 block (256 bytes) limit
	int2bytes(address, helpBuf); //address drvide into 2 bytes
	if(helpBuf[1] == 0xFF)//if lo byte is at the e2 block limit
	{
		helpBuf[0] += 1;	//add 1 to hi byte
		helpBuf[1] = 0x00;//set 0 to lo byte
		//set write pointer
		address = bytes2int(helpBuf);
	}
	else
	{
		if(sensorMeasur) //fla - if it is sensor measur data to be writen into e2
			//move pWrite to next write position
			address += 2;
    }
	//set data into 2 bytes
   	int2bytes(intData, helpBuf);

	//write data onto ext_e2
	if(!(e2_writePage(address, 2, helpBuf)))
		return 1;//if writing into ext_e2 faild exit faild
	//else
	return address;

}

//reset data block in ext_e2 (set FF7F into the block)
char ResetDataBlock(unsigned int block_address)
{
	char i;
	int tempAddress, resetData, pArr;

	pArr = block_address;
	//resetData = 0xff7f;

       resetData = 0x7FFF;       //kando's format

	for(i = 0; i < 24; i++)   //write into data block (2 bytes x 24 = 48 bytes)
	{
		//tempAddress = e2_writePage(pArr, 8, resetArr);
		tempAddress = WriteIntoExte2(resetData, pArr, 0);
		//if writing into ext_e2 faild exit faild
		if(!(tempAddress == 1))
			pArr = tempAddress;
		else
			return FALSE;
		pArr += 2; //move pointer 2 bytes
	}
	return TRUE;
}

//set next pBread position
//end of reading space is the first block (including first block)
//return 1 if there is new pBread address or 0 if not
char NextpReadAddress()
{
   //if read block = write block
    if(pBread == pBwrite)
		return FALSE; //exit the function: no more data

	//if read block + 48 > data end space
	if((pBread + 48) > pEnd_space)
 	{
		pBread = pStart_space;
		return TRUE;
	}

	//else
	pBread += 48;
	return TRUE;
}

//the function will copy one data block from ext_e2 (pBread address)
//into 'sensDataBlock' buffer in ram
//return 1 (succsess) or 0 (failure)
char CopyBlockIntoRam()
{
    int   i, j; //, more_data;
	unsigned int Block2Read;
	//char buf[2]; //add for debug only

	Block2Read = pBread;

	//disable other operation while run this function
    e2_writeFlag = 1;

	for(i = 0; i < 6; i++) 			//read data block (8 bytes x 6 = 48 bytes)
	{
		//read ext e2 (from data block into read buf)
		if(!(e2_readSeqBytes(Block2Read, 8)))
		{
			e2_writeFlag = 0; 	// inable other use
			return FALSE;		//if reading fail, exit the program
		}
		//copy data from read buf into sensDataBlock
		for(j = 0; j < 8; j++)
		{
		    DataBlock[j+((int)i*8)] = eepromReadBuf[j];
		}
		Block2Read += 8; 			//move pointer 8 bytes
	}
	//check if there is more data to send
//	temp_pBread = pBread; 			//save pBread into temp
	//bMoreDataToSend = NextpReadAddress(); 	//1 = more data; (old 0x94) 0 = no more data  (old 0x14)
//	pBread = temp_pBread;			//set the original address into pBread
	e2_writeFlag = 0; 				// enable other use
	return TRUE;

}

//calculate the time from last measuring cycle
//(in 10 minutes resulution)
unsigned int TimeFromLastCycle()
{
	unsigned int temp;

	//if minute now <= minute last measur sycle 
    
         
	if(measure_time <= last_cycle_min)
	{
		//add 24(houers)x60 = 1440 to minute now
		temp = measure_time + 1440;
		//calculate minutes from last cycle
		temp = temp - last_cycle_min;
	}
	else
		temp = measure_time - last_cycle_min;

	return temp;
}

//read control parameters from ext_e2 into ram
//arrange the pointers and parameters for manipulation ( saving)
char ReadCtrlParamsToRam()
{
	//read the sensor control parameters from ext_e2 into the ram read_buf
	if(!(e2_readSeqBytes(pSens_Ctrl_Params, 16)))    //pSens_ext_e2 points to last params - 16 bytes in block
		return FALSE;                           //reading to eepromReadBuf

	//set pointer to first data block
	pBfirst = bytes2int(eepromReadBuf);         //make two bytes as int pointer
	//set pointer to write data block
	pBwrite = bytes2int(eepromReadBuf+2);
	//set write pointer
	pWrite = bytes2int(eepromReadBuf+4);
	//set read block pointer
	pBread = bytes2int(eepromReadBuf+6);
	//set pointer to start data space
	pStart_space = bytes2int(eepromReadBuf+8);
	//set pointer to end data space
	pEnd_space = bytes2int(eepromReadBuf+10);
	//set last cycle time into variable
	last_cycle_min = bytes2int(eepromReadBuf+12);
	//set interval into variable
	cuurent_interval = eepromReadBuf[14];

	return TRUE;
}

//set time, interval, error flag into the block header
char SetBlockHeader(unsigned int block_address)
{
	char headerArr[8], i, arrByte;
	int tempAddress;
	int tempData;

	tempAddress = block_address;


	headerArr[0] = 'd'; 			//start of data block
	headerArr[1] = cuurent_interval;
	headerArr[2] = 0x00; 			//status (error) data
	headerArr[3] = readClockBuf[0]; //year
	headerArr[4] = readClockBuf[1]; //month
	headerArr[5] = readClockBuf[2]; //day
	headerArr[6] = readClockBuf[4]; //hour
	headerArr[7] = readClockBuf[5]; //minute

	arrByte=0;

	for(i = 0; i < 4; i++)
	{
		//set data 2 bytes into int data
		tempData = bytes2int(&headerArr[arrByte]);
                //write into ext_e2

		tempAddress = WriteIntoExte2(tempData, tempAddress, 0);

		if(!(tempAddress==1))   //ok
		{
            tempAddress += 2;
            arrByte += 2;
  		}
  		else
  			return FALSE;
	}

	return TRUE;
}

void pBread_vers_pBfirst()
{
	//if read block = first block, move it with the new first block
	if(pBread == pBfirst)
		pBread += 48;
	pBfirst += 48;
}

//open new data block
//set the write block pointer to the new place
char CreateNewDataBlock()
{
    char new_pWrite_flag = 0;
//    #ifdef DebugMode
//    SendDebugMsg("CreateNewDataBlock");
//    #endif DebugMode

	//if write block + 48 = first block
	if((pBwrite + 48) == pBfirst)
	{
		new_pWrite_flag = 1;
		pBwrite = pBfirst;
		//if first block + 48 = the data space end
		if((pBfirst + 48) > (pEnd_space))
		{
			//if read block = first block, move it with the new first block
			if(pBread == pBfirst)
		   		pBread = pStart_space;

		 	 pBfirst = pStart_space;
		}
		else //first block not in data space end
		{
		 	pBread_vers_pBfirst();
		}
	}

	//if write block + 48 = data space end
	if(((pBwrite + 48) > pEnd_space) && (new_pWrite_flag == 0))
	{
		new_pWrite_flag = 1;
		pBwrite = pBfirst;
		pBread_vers_pBfirst();
	}

	//if pBwrite not in one of the previos situation
    if(new_pWrite_flag == 0)
		pBwrite += 48;

	if(ResetDataBlock(pBwrite) == FALSE)
		return FALSE;
	//set time, interval, error flag into the block header
	if(SetBlockHeader(pBwrite) == FALSE)
		return FALSE;
	//set write pointer to block first data
	//pWrite points 2 step before the next writing point
	pWrite = pBwrite + 6;

    if( New_pBread_setting_needed == TRUE) //when data transmission done OK set new read pointer
    {
       pBread = pBwrite;
       New_pBread_setting_needed = FALSE;
    }

	return TRUE;
}

//save control parameters into sensor block in ext_e2
char SaveControlParam()
{
	 char controlArr_1[8];
	 char controlArr_2[8];

	//set parameters into array:
	//~~~~~~~~~~~~~~~~~~~~~~~~~~
	//controlArr[0] and [1] get pointer to first data block
	int2bytes(pBfirst, controlArr_1);
	//controlArr[2] and [3] get pointer to write data block
	int2bytes(pBwrite, &controlArr_1[2]);
	//controlArr[4] and [5] get pointer to write point
	int2bytes(pWrite, &controlArr_1[4]);
	//controlArr[6] and [7] get pointer to last read data block
	int2bytes(pBread, &controlArr_1[6]);

	//controlArr[8] and [9] get pointer to start data space
	int2bytes(pStart_space, controlArr_2);
	//controlArr[10] and [11] get pointer to end data space
	int2bytes(pEnd_space, &controlArr_2[2]);
	//controlArr[12] and [13] get last cycle time in minutes from day start
	int2bytes(measure_time, &controlArr_2[4]);
	controlArr_2[6] = cuurent_interval;
	controlArr_2[7] = 0xFF; //end of data sign

	//write control parameters onto ext_e2

	 if(!(e2_writePage(pSens_Ctrl_Params, 8, controlArr_1)))
    {
	  //if writing into ext_e2 faild exit faild
//        #asm ("sei")
		return FALSE;
    }
	//write control parameters onto ext_e2

	if(!(e2_writePage(pSens_Ctrl_Params + 8, 8, controlArr_2)))
   {
	    //if writing into ext_e2 faild exit faild
        #asm ("sei")
		return FALSE;
    }

    #asm ("sei")
	return TRUE;
}

//save pRead parameters into storing location in ext_e2  add=7184
//char StoreOldReadPoiters(char num)
//{
//	 char ParanmArr[14];
//	 char i, k;
//
//     k = 0;
//	//set parameters into array:
//	//~~~~~~~~~~~~~~~~~~~~~~~~~~
//    for (i = 0; i < num; i++)
//    {
//        int2bytes(tmp_pBread[i], &ParanmArr[k]);
//        k += 2;
//    }
//
//	//write  parameters onto ext_e2
//     #asm ("cli")
//	 if(!(e2_writePage(pRead_BU_ADDRESS, i, ParanmArr)))
//    {
//	  //if writing into ext_e2 faild exit faild
//        #asm ("sei")
//		return FALSE;
//    }
//
//    #asm ("sei")
//	return TRUE;
//}

//save sensors measurments results in the external eeprom
//the function return (1 = success) or (0 = failue)
char SaveMeasurments()
{
	int temp_interval;
	int calc_interval;
	int temp_address;
    int tmp_pBwrite;
  //   char Str[100], i, k;
       char i, k;
  //  char hour, min, sec;

//     #ifdef DebugMode
//        SendDebugMsg("in SaveMeasurments()\r\n\0");
//    #endif DebugMode


     i = k = 0;

	//set address for selected sensor into 'pSens_Ctrl_Params'
	SetCotrolParamAddress();          //set  pSens_Ctrl_Params as pointer to control params of block

	//read control parameters from cpu_e2 into ram
	//arrange the pointers and parameters for manipulation
	if(ReadCtrlParamsToRam() == FALSE)
		return FALSE;

    //check if it is the first data in block - if(pWrite == pBwrite + 6) yes
    //init new block
    if(pWrite == pBwrite + 6)
    {     
      //  SendDebugMsg("\r\ndebug - First data in block..\r\n\0");
        if(ResetDataBlock(pBwrite) == FALSE)
         return FALSE;
        //set 8 first bytes-time, interval, error flag into the block header
        if(SetBlockHeader(pBwrite) == FALSE)
          return FALSE;
    }


		//check if time pass from last measur cycle != interval
		//or if write block is full
		//create a new data block
		temp_interval = TimeFromLastCycle();
		//if temp =, or +1, or -1, from current interval it is o.k.
		calc_interval = ((int)cuurent_interval * INTERVAL_PARAM);   //minutes

//         PrintNum((long)cuurent_interval);  //1
//          PrintNum((long)calc_interval);   //5
//           PrintNum((long)temp_interval);  //5

		if((temp_interval == calc_interval)||\
					((temp_interval+1) == calc_interval)||\
			 		((temp_interval-1) == calc_interval))
			temp_interval = 0;
		else
		   	temp_interval = 1;

        // add on 15/10/13 - in case interval of block is different from the global one - update new interval and start new block
		if (cuurent_interval !=  cpue2_interval_1)
		{
            cuurent_interval = cpue2_interval_1;
            temp_interval = 1;
		}
//        SendDebugMsg("\r\ndebug - First data in block..\r\n\0");
//         PrintNum((long)temp_interval);  //5
//-----------------------------------------------------
     //    temp_interval = 0;               //Danny demo - no new block every minute
//-----------------------------------------------------


            //check the write pointer position in respect to write block pointer and if measuring in alret status every 1 min.
        //	if((temp_interval) || ((pWrite + 2) >= (pBwrite + 48))|| ((msrAlertFlag == 1) && (PumpActivated == FALSE)))

         if((temp_interval) || ((pWrite + 2) >= (pBwrite + 48))|| (QuickSensorsCheckNeeded == TRUE)) //new block needed
         {
//               if(Illigal_Time_Val == FALSE)  //if lligal time stamp -  dont create block
//              {

                  tmp_pBwrite  =  pBwrite;    //save pointer if needed later
                  do{
                       // SendDebugMsg("\r\nNew data block needed..\r\n\0");
                            if(CreateNewDataBlock() == FALSE)   //open new block for data
                            {
                                      SendDebugMsg("\r\nNew Block creation FAILED...\r\n\0");
                                     return FALSE;
                            }

                          else
                          {
                               SendDebugMsg("\r\nNew data block created..\r\n\0");

                               calc_interval = pBread ; //save pBread
                               pBread = pBwrite;   //use it
                               CopyBlockIntoRam();    //read new block for debug  into DataBlock[]
                               pBread = calc_interval; //restore pBread

    //                          sprintf(Str,"Block Time check - %02d/%02d/%02d-%02d:%02d\n\r\0",DataBlock[3],DataBlock[4],DataBlock[5],DataBlock[6],DataBlock[7]);
    //                          UART_WriteMsg(Str);
    //                         ShowHexString(DataBlock, 8 );

                               if((readClockBuf[0] != DataBlock[3]) || (readClockBuf[1] != DataBlock[4]) ||\
                               (readClockBuf[2] != DataBlock[5]) || (readClockBuf[4] != DataBlock[6]) || (readClockBuf[5] != DataBlock[7]))
                               {
                                     k++;
                                     pBwrite =  tmp_pBwrite;   //restore pBwrite for the new block
                               }
                               else i++;
                          }   
                       
                          
                  }while ((i == 0) && (k < 2));  
                  
                  if(k == 2)
                  {
                        SendDebugMsg("\r\nWrite new Block time-stamp failure..!\r\n\0");
                        return FALSE;
                  }  
                  
//               }
//              else
//              {
//                   SendDebugMsg("\r\n Illigal time/date - New data block conceled..!\r\n\0");
//                   return FALSE;  //ignor new data due to illgal time stamp
//              }

       }

        //save current data   
//         SendDebugMsg("\r\nnow Saving..\r\n\0");

         temp_address = WriteIntoExte2(SensorResult, pWrite, 1); //actual saving proc
                
         if(!(temp_address == 1))    //write ok
        {
         //-------------new 010817- //vrification that data saved ok  -read back-------- 
         
              if((e2_readSeqBytes(temp_address, 2)))
              temp_interval = bytes2int(eepromReadBuf);  //make it int again

             if(temp_interval == SensorResult)     //similar to original? if yes go on
             {
                    pWrite = temp_address;      //update write pointer
                    SendDebugMsg("\r\nData Saved- ");
                    PrintNum(SensorResult);
              }
              else
              return FALSE;

        }
        else
        {
           SendDebugMsg("\r\nSaving data failure..! \r\n\0");
           return FALSE;
        }


        SaveControlParam();
	    return TRUE;

}

//char ResetReadPointer()
//{
//    if (objToMsr >= NumSensors)
//        return FALSE;
//
//	//set address for selected sensor into 'pSens_Ctrl_Params'
//	SetCotrolParamAddress();
//
//	//read control parameters from cpu_e2 into ram
//	//arrange the pointers and parameters for manipulation
//	if(ReadCtrlParamsToRam() == FALSE)
//		return FALSE;
//    pBread = pOriginalReadBlock;
//
//    if (SaveControlParam() == FALSE)
//        return FALSE;
//
//	return TRUE;
//}
 //Danny - 25/02/2016
 //new way 2903
char ResetAllReadPointers()
{
   char i, k=0;
   char tmp_objToMsr;

   tmp_objToMsr = objToMsr;      //save val

   for( i = 0; i < NumSensors; i++)
   {
        if(Option_1[i] != 0)    //pointer is ready
        {
            //set address for selected sensor into 'pSens_Ctrl_Params'
            objToMsr = i;
            SetCotrolParamAddress();     //Set pointer to params of currwnt sensor- need objToMsr for location

            //read control parameters from cpu_e2 into ram
            if(ReadCtrlParamsToRam() == FALSE)
            {
                objToMsr = tmp_objToMsr;
                return FALSE;
            }

         //   pBread = Option_1[i];   //read from eeprom array  
              pBread =  eDataPtrSave[i];   //read from eeprom array
          //   eDataPtrSave[i] = 0;      ??????????????
           if (SaveControlParam() == FALSE)
           {
               objToMsr = tmp_objToMsr;
               return FALSE;
           }
           k++;
        }
    }
      objToMsr = tmp_objToMsr;
      if(k == NumSensors) SendDebugMsg("\r\nCOMM Failure - data pointers restored  ..\r\n\0");

	return TRUE;
}


 //new way 2903
char ResetAllReadPointerspBwrite()
{
   char i;
   char tmp_objToMsr;


   tmp_objToMsr = objToMsr;      //save val

   for( i = 0; i < NumSensors; i++)
   {
	    //set address for selected sensor into 'pSens_Ctrl_Params'
        objToMsr = i;
        SetCotrolParamAddress();     //need objToMsr for location

        //read control parameters from cpu_e2 into ram
        //arrange the pointers and parameters for manipulation
        if(ReadCtrlParamsToRam() == FALSE)
        {
            objToMsr = tmp_objToMsr;
            return FALSE;
        }
      //--------------old-020716--------------
    //   pBread = pBwrite;   //pBread move to pBwrite in case data transmitted, keep last block to be filled
      //--------------new-------------------------------

    ///////  *  if((temp_interval) || ((pWrite + 2) >= (pBwrite + 48))|| (QuickSensorsCheckNeeded == TRUE))*

           pWrite = pBwrite + 46 ; //force new data block creation in next data save.
             New_pBread_setting_needed = TRUE;
      //-----------------------------------------------------------

             objToMsr = tmp_objToMsr;

             if (SaveControlParam() == FALSE)
             return FALSE;
    }

    SendDebugMsg("\r\nPost data Success - Set pointers ..\r\n\0");

	return TRUE;
}

char _GetMeasurments_(char read_mode)
{
    int n;

    if (objToMsr >= NumSensors)
        return FALSE;
	//set address for selected sensor pointers into 'pSens_Ctrl_Params'
	SetCotrolParamAddress();

	//read control parameters from cpu_e2 ('pSens_Ctrl_Params' address) into ram
	//arrange the pointers and parameters for manipulation
	if(ReadCtrlParamsToRam() == FALSE)
    {
         SendDebugMsg("GM-1\r\n\0");
		return FALSE;
    }
	//check if there is any data block in eeprom
	if(pWrite == pBwrite + 6) //there is no data records yet
    {
         SendDebugMsg("No BLocks..\r\n\0");
		return FALSE;
    }


//        SendDebugMsg("\r\n _GetMeasurments- pBread = ");
//        PrintNum((long)pBread);

      //save ponter to be restored if com failure 
       pBread = pStart_space;
      #asm("cli")  
           eDataPtrSave[objToMsr] =  pBread;   //280220  save current pWrite pointer (point to current write location)
      #asm("sei")

    // calculate num of blocks to send   
    if (read_mode == 1)
    {
        if (pBwrite >= pBread)
        {
            n = pBwrite - pBread;  //pointers diff - 48 bytes per block

        }
        else
        {
            n = pEnd_space - pBread + 1;
            n += pBwrite - pStart_space;
        }
        n = n / 48;
        nUnreadBlocks = n + 1;

     //    PrintNum(nUnreadBlocks);

     //-------------------------------------------------------
     #ifdef SENSOR_MEM_2K
        if( nUnreadBlocks > 42)  //max 21 blocks for sensor 080517
     #else
        if( nUnreadBlocks > 21)  //max 21 blocks for sensor 080517
     #endif
        {
     //     SendDebugMsg("GM-3\r\n\0");
	    	return FALSE;
        }
     //=============================================================

       
         SendDebugMsg("Data Blocks count = ");
        PrintNum(nUnreadBlocks);
   

      return TRUE;
    }
}
//get sensoer measurments results from the external eeprom
//read_mode = 0 - send all data (from 'first' block)
//read_mode = 1 - send un read data (from 'read' block)
//read_mode = 2 - next data block
//read_mode = 3 - last data block again
//read_mode = 4 - send end data msg
//arrange in 'sensDataBlock': more data|records; status; time; data;
//return 1 (succsess) or 0 (failure)

char GetMeasurments(char read_mode)
{
    int n;

//    SendDebugMsg("\r\nGet data Records");


    if (objToMsr >= NumSensors)
        return FALSE;    
        
	//set address for selected sensor into 'pSens_Ctrl_Params'
	SetCotrolParamAddress();

	//read control parameters from cpu_e2 ('pSens_Ctrl_Params' address) into ram
	//arrange the pointers and parameters for manipulation
	if(ReadCtrlParamsToRam() == FALSE)
    return FALSE;

	//check if there is any data block in eeprom
	if(pWrite == pBwrite + 6) //pWrite didnt move - there is no data records yet
	{
		return FALSE;   //change 26.05.2013: false instead of true
	}

//    // calculate num of blocks to send
    if (read_mode == 1)
  //   if (read_mode == 0)    //changed 290220 -  pBread = pBfirst;     
    {    
          pBread =  pStart_space;;    //changed 290220 -  pBread = pBfirst; 
    
        if (pBwrite >= pBread)
            n = pBwrite - pBread;
        else    //write pointer is folded at end space
        {
            n = pEnd_space - pBread + 1;
            n += pBwrite - pStart_space;
        }
        n = n / 48;

        nUnreadBlocks = n + 1;
        // save for further use - in case
        pOriginalReadBlock = pBread;
    }

	//set the proper address for data block to be read
	switch(read_mode)
	{
		case 0:
            pBread = pBfirst;    //get all
        break; 
            
		case 1:   
                  //try read from start of memory
               //   pBread = pStart_space;    //new approach-no dont chang pBread, start read from last block sent
        break; 
            
		case 2:
                NextpReadAddress();   //handle pBread location while reading data            
        break; 
            
		case 3:
        break;                 //dont chang pBread 
            
//		case 4:
//            end_of_data_msg();
//            return TRUE;              //return just for ack exit function o.k.

		default:
            return FALSE;          //exit the function (failure)
	}

	if(CopyBlockIntoRam() == FALSE) //copy data block at the pBread address
    {
        SendDebugMsg("\r\nfailed CopyBlockIntoRam()\r\n\0");
		return FALSE;
    }

//     //restor bBread here in case of problem. Move it to pBwrite if comm success
//     if((nUnreadBlocks == 1) && (read_mode == 2 ) )   //last read of curren sensor blocks
//     {
//             if(Option_1[objToMsr] != 0)
//             pBread = (Option_1[objToMsr]);   //point to original block in case server fail. dont loos data
//     }

	//save control parameters into ext mem
	if (SaveControlParam() == FALSE)
    {
            SendDebugMsg("\r\n failed SaveControlParam()\r\n\0");
		    return FALSE;
    }
	//else
	return TRUE;
}


//set new interval
char SetNewInterval(char new_interval)
{
    //if interval in cpu_e2 == new interval do nothing
    if((cpue2_interval_1 == new_interval))
        return TRUE;
    if ((new_interval != 1) &&
        (new_interval != 2) &&
        (new_interval != 3) &&
        (new_interval != 6) &&
        (new_interval != 12)&&
        (new_interval != 24))
        return FALSE;
    //if it is a new interval
    //initiate the data blocks in ext_e2 for sensor 1   //15/10/13 - remove next line
//    if(InitDataBlocks(new_interval) == FALSE)
//        return FALSE;
    //update interval in cpu_e2
    cpue2_interval_1 = new_interval;
	//else if all o.k.
	return TRUE;
}

//check validity of sensors data pointers and interval
char PointersValidate()
{
  char check = 0;
//    #ifdef DebugMode
//    SendDebugMsg("PointersValidate\r\n\0");
//    #endif DebugMode
	//check sensor pointers validate:
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//read control parameters from cpu_e2 into ram
	//arrange the pointers and parameters for manipulation
	if(ReadCtrlParamsToRam() == FALSE)
    {
        check = 1;
		return FALSE;
    }

	//if pointer to write data block not in range exit failure
	if(!((pBwrite >= pStart_space) && (pBwrite < pEnd_space)))
   {
        check = 2;
		return FALSE;
    }
	//if pointer to firts data block not in range exit failure
	if(!((pBfirst >= pStart_space) && (pBfirst < pEnd_space)))
   {
        check = 3;
		return FALSE;
    }
    if(check > 0) PrintNum((long)check);
	//if all test is ok
	return TRUE;
}

//initiate the data blocks in ext_e2 for all sensors
char InitDataBlocks(char interval)
{
    BYTE senIndex;
    BYTE SensorsQuant;

    SensorsQuant = NumSensors; //read from eeprom

   // for (senIndex = SENSOR1; senIndex < (MAX_SEN_NUM ); senIndex++)  //set blocks
      for (senIndex = SENSOR1; senIndex < SensorsQuant ; senIndex++)  //set blocks
    {
        //set pointer to first data block
        pBfirst = SENSOR_MEMORY_START + (senIndex * SENSOR_MEMORY_SIZE);    // sens1_data_start;  SENSOR_MEMORY_SIZE=0x400
        //set pointer of write data block
        pBwrite = pBfirst;    //sens1_data_start;
        //set write pointer
        pWrite = pBfirst; //sens1_data_start;
        pWrite += 6;
        //set read block pointer
        pBread = pBfirst; //sens1_data_start;
        //set pointer to start data space
        pStart_space = pBfirst; //sens1_data_start;
        //set pointer to end data space
        pEnd_space = pBfirst + SENSOR_MEMORY_SIZE - SENSOR_CNTRL_PRM_SIZE - 1;//SENSOR_MEMORY_START + (senIndex * SENSOR_MEMORY_SIZE) - SENSOR_CNTRL_PRM_SIZE - 1; //sens1_data_ends;
        //set parameters for sensor
        pSens_Ctrl_Params = pEnd_space + 1; //SENSOR_MEMORY_START + (senIndex * SENSOR_MEMORY_SIZE) - SENSOR_CNTRL_PRM_SIZE; //sens1_control_param;
        //set last cycle time into variable
        last_cycle_min = time_in_minutes;
        //set interval into variable
        cuurent_interval = interval;
        //save control parameters into ext_e2
        if (SaveControlParam() == FALSE)
            return FALSE;


    }
      #ifdef DebugMode
        SendDebugMsg("\r\nInit Memory OK..!\r\n\0 ");
     //   PrintNum(nUnreadBlocks);
        #endif DebugMode
	//else if ok
	return TRUE;
}

//set new parameters (receive parameters by radio)
//parameter = 7 - clock; parameter  = 8 - interval
//add for sensor+modem type: parameter  = 9 - H1, H2, PCmodem number
/*
void SetSensorParameter(char param_type, char* new_data_buf)
{
    char fail_flag;
    //fail_flag = 0;
    fail_flag = 1; //(31-01-02)

	//set address for selected sensor into 'pSens_Ctrl_Params'
	SetCotrolParamAddress(1);
	//check the parameter type
	if(param_type == 6) //6 = do measuring cycle now
	{
    }
	if(param_type == 7) //7 = set clock parameter
	{
		MemCopy( clockBuf, new_data_buf, 3 );
		MemCopy( clockBuf+4, new_data_buf+3, 3 );
		if(SetRealTime())	//set real time
			fail_flag = 0;
   	}

	if(param_type == 8) //8 = set interval parameter
	{
		//check if new interval > 0
		if(new_data_buf[0] > 0)
		{
			//check if new interval < 10 (or) mode 6 result 0 (disable receiving randum values)
			if((new_data_buf[0] < 10) || ((new_data_buf[0] >= 10) && ((new_data_buf[0] % 6) == 0)))
			{
//                if(RxSensorNumber+1 == 1)
                {
                    //disable interrupts
                    #asm ("cli")
                    //set interval sensor 1
                    if(SetNewInterval(new_data_buf[0]))
                        fail_flag = 0;
                    //enable iterrupts
                    #asm ("sei")
                }
			}
		}
	}
	if(param_type == 10) //10 = set modem parameters
	{
//	        //set hour1
//	        Start_H1 = new_data_buf[0];
//	        //set hour2
//	        Start_H2 = new_data_buf[1];
//	        //set MinRSSI
//	        MinRSSI = new_data_buf[27];
//
//	        //set phone number
//	        MemCopy_to_cpu_e2(&Dest_Modem_Num[0], new_data_buf+2, 12);
//	        //set internal sim number
//	        MemCopy_to_cpu_e2(&Int_Modem_Num[0], new_data_buf+14, 12);
//	        //set ip number
//	        //MemCopy_to_DestIP(&Dest_Server_IP[0], new_data_buf+2, 12);
//	        //set port number
//	        //MemCopy_to_cpu_e2(&Dest_Server_Port[0], new_data_buf+14, 4);
//	        fail_flag = 0;
	}

	//preper acknolage for rf
//	ComBuf[BEGIN_DATA] = 'p';
//	if(fail_flag == 1)
//		ComBuf[BEGIN_DATA+1] = NCK_CODE;
//	else
//	 	ComBuf[BEGIN_DATA+1] = ACK_CODE;
//	LengthOfData = 2; //set the data size
//
//    //length inclouding rssi and batt.
//    LengthOfData = LengthOfData + 3;
}
 */

/*
#ifdef Sens_1_Valid
char sensor1_parameters(char data_code)
{
	char data_len;

	switch(data_code)
	{
	case 3: //set sensor1_last_value into 2 bytes
			int2bytes(sensor1_last_value, &ComBuf[BEGIN_DATA+3]);
			data_len = 2;
	   		break;

	case 4: //return the sensor type
			cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_type_1, 10);
			data_len = 10;
            break;

	case 5: //return the sensor name
			cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_name_1, 32);
			data_len = 32;
            break;

	case 6: //return measurment unit
			cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], measur_unit_1, 20);
			data_len = 20;
            break;

	case 8:	//return the interval
			ComBuf[BEGIN_DATA+3] = cpue2_interval_1;
	        data_len = 1;
            break;

	case 9:	//return the data format
			ComBuf[BEGIN_DATA+3] = data_format_1;
	        data_len = 1;
            break;

    default: return 0;

	}

    return data_len;
}
#endif Sens_1_Valid
*/
/*
#ifdef Sens_2_Valid
char sensor2_parameters(char data_code)
{
	char data_len;

	switch(data_code)
	{
	case 3: //set sensor2_last_value into 2 bytes
			int2bytes(sensor2_last_value, &ComBuf[BEGIN_DATA+3]);
			data_len = 2;
	   		break;

	case 4: //return the sensor type
			cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_type_2, 10);
			data_len = 10;
                        break;

	case 5: //return the sensor name
			cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_name_2, 32);
			data_len = 32;
                        break;

	case 6: //return measurment unit
			cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], measur_unit_2, 20);
			data_len = 20;
                        break;

	case 8:	//return the interval
			ComBuf[BEGIN_DATA+3] = cpue2_interval_2;
	                data_len = 1;
                        break;

	case 9:	//return the data format
			ComBuf[BEGIN_DATA+3] = data_format_2;
	                data_len = 1;
                        break;

    default: return 0;
	}

    return data_len;
}
#endif Sens_2_Valid
*/

/*
void end_of_data_msg(void)
{
//	ComBuf[0+BEGIN_DATA] = 0x64; //command code 'd'
//	ComBuf[1+BEGIN_DATA] = 0x00; //more data (no)
// 	LengthOfData = 2; //set the data size
}
*/
/*
#ifdef PREV_DATA_BLOCK
//set pBread position one block back to disable missing of data
//end of reading space is the first block (including first block)
//return 1 if there is new pBread address or 0 if not

char Prev_pRead_address()
{

	//if read block = data start space
	if(pBread == pStart_space)
 	{
	 	//check if it is not the only one data block
	 	if( (pBwrite > pStart_space) &&
			(pEnd_space > pBwrite + 48) &&
			(pBfirst > pBwrite))
	 	{
	 		//move the read block to the end of data space
			pBread = pEnd_space - 47;
			return 1;
		}
		else
		{
			//dont move read block pointer
			return 0;
		}
	}
	//else
	//if write block is one block back from read block
	if(pBread - 48 == pBwrite)
			//dont move read block pointer
			return 0;
	//else
	pBread -= 48;
	return 1;
}

//send the ex_e2 pointers by rf
void CopyPointersListToBuff()
{
	//set parameters into array ComBuf[]:
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	ComBuf[0+BEGIN_DATA] = 0x74; 	//command code 't'
	//ComBuf[1] and [2] get pointer to first data block
	int2bytes(pBfirst, &ComBuf[1+BEGIN_DATA]);
	//ComBuf[3] and [4] get pointer to write data block
	int2bytes(pBwrite, &ComBuf[3+BEGIN_DATA]);
	//ComBuf[5] and [6] get pointer to write point
	int2bytes(pWrite, &ComBuf[5+BEGIN_DATA]);
	//ComBuf[7] and [8] get pointer to last read data block
	int2bytes(pBread, &ComBuf[7+BEGIN_DATA]);

	//ComBuf[9] and [10] get pointer to start data space
	int2bytes(pStart_space, &ComBuf[9+BEGIN_DATA]);
	//ComBuf[11] and [12] get pointer to end data space
	int2bytes(pEnd_space, &ComBuf[11+BEGIN_DATA]);
	//ComBuf[13] and [14] get last cycle time in minutes from day start
	int2bytes(measure_time, &ComBuf[13+BEGIN_DATA]);
	ComBuf[15+BEGIN_DATA] = cuurent_interval;
	//ComBuf[16] = 0xFF; //end of data sign

 	LengthOfData = 16; //set the data size
}
#endif //PREV_DATA_BLOCK
*/
//set data into sens_data_block
//command = V; data_length; data code; data value
//set total buffer length into global parameter
/*
char GetSensorParameter(char sensor_num, char data_code)
{
	char data_len;
	switch(data_code)
	{
	    case 1: //set w1_Result into 2 bytes
			int2bytes(w1_Result, &ComBuf[BEGIN_DATA+3]);
			data_len = 2;
			break;

	    case 2: //set circuit_temp into 2 bytes
            #ifdef Modem_Comm
            #ifdef Sens_2_bhs1
            //show the scale mV as program version
            circuit_temp = scale_mv;
            #endif Sens_2_bhs1
            #endif Modem_Comm

			int2bytes(circuit_temp, &ComBuf[BEGIN_DATA+3]);
			//show the 12v_batt value as the sensor name
			#ifdef Sens_1_ph
            #ifdef Sens_2_lws
            if(sensor_num == 1)
                //show the 12v battery value
                int2bytes(batt_12v_value, &ComBuf[BEGIN_DATA+3]);
            if(sensor_num == 2)
                //show the drainage counter value
                int2bytes(drainage_pulse_done, &ComBuf[BEGIN_DATA+3]);
            #endif Sens_1_ph
            #endif Sens_2_lws

			data_len = 2;
			break;

	    case 3: //set sensor1/2_last_value into 2 bytes
            #ifdef Sens_1_Valid
            if(sensor_num == 1)
                int2bytes(sensor1_last_value, &ComBuf[BEGIN_DATA+3]);
                //int2bytes(ram_op_setpoint, &ComBuf[BEGIN_DATA+3]);
            #endif Sens_1_Valid
            #ifdef Sens_2_Valid
            if(sensor_num == 2)
                int2bytes(sensor2_last_value, &ComBuf[BEGIN_DATA+3]);
            #endif Sens_2_Valid
            #ifdef Sens_3_Valid
            if(sensor_num == 3)
                int2bytes(sensor3_last_value, &ComBuf[BEGIN_DATA+3]);
            #endif Sens_3_Valid
            data_len = 2;
            break;

	    case 4: //return the sensor type
            #ifdef Sens_1_Valid
            if(sensor_num == 1)
                cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_type_1, 10);
            #endif Sens_1_Valid
            #ifdef Sens_2_Valid
            if(sensor_num == 2)
                cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_type_2, 10);
            #endif Sens_2_Valid
            #ifdef Sens_3_Valid
            if(sensor_num == 3)
                cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_type_3, 10);
            #endif Sens_3_Valid
            data_len = 10;
            break;

	    case 5: //return the sensor name
            #ifdef Sens_1_Valid
            if(sensor_num == 1)
                cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_name_1, 32);
            #endif Sens_1_Valid
            #ifdef Sens_2_Valid
            if(sensor_num == 2)
                #ifdef Sens_1_ech2o_te
                //show the ech2o sensor output string as the sensor name
                MemCopy( &ComBuf[BEGIN_DATA+3], byte_com_result, 32);
                #else
                cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_name_2, 32);
                #endif Sens_1_ech2o_te
            #endif Sens_2_Valid
            #ifdef Sens_3_Valid
            if(sensor_num == 3)
                #ifdef Sens_2_ech2o_te
                //show the ech2o sensor output string as the sensor name
                MemCopy( &ComBuf[BEGIN_DATA+3], byte_com_result, 32);
                #else
                cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], sensor_name_3, 32);
                #endif Sens_2_ech2o_te
            #endif Sens_3_Valid
            data_len = 32;
            break;

	    case 6: //return measurment unit
     			#ifdef Sens_1_Valid
     			if(sensor_num == 1)
			        cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], measur_unit_1, 20);
     			#endif Sens_1_Valid
                #ifdef Sens_2_Valid
                if(sensor_num == 2)
			        cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], measur_unit_2, 20);
     			#endif Sens_2_Valid
                #ifdef Sens_3_Valid
                if(sensor_num == 3)
			        cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+3], measur_unit_3, 20);
     			#endif Sens_3_Valid
			data_len = 20;
                        break;

	    case 7: GetRealTime(); //read the rtc
			MemCopy( &ComBuf[BEGIN_DATA+3], readClockBuf, 3 );
			MemCopy( &ComBuf[BEGIN_DATA+6], readClockBuf+4, 3 );
			data_len = 6;
                        break;

	    case 8:	//return the interval
			#ifdef Sens_1_Valid
     			if(sensor_num == 1)
			        ComBuf[BEGIN_DATA+3] = cpue2_interval_1;
			#endif Sens_1_Valid
			#ifdef Sens_2_Valid
	                if(sensor_num == 2)
			        ComBuf[BEGIN_DATA+3] = cpue2_interval_2;
			#endif Sens_2_Valid
			#ifdef Sens_3_Valid
	                if(sensor_num == 3)
			        ComBuf[BEGIN_DATA+3] = cpue2_interval_3;
			#endif Sens_3_Valid
	                data_len = 1;
                        break;

	    case 9:	//return the data format
			//ComBuf[BEGIN_DATA+3] = data_format_1;
     			#ifdef Sens_1_Valid
     			if(sensor_num == 1)
			        ComBuf[BEGIN_DATA+3] = data_format_1;
			        //ComBuf[BEGIN_DATA+3] = Interrupting_RTU_is_Active;
     			#endif Sens_1_Valid
	                #ifdef Sens_2_Valid
	                if(sensor_num == 2)
			        ComBuf[BEGIN_DATA+3] = data_format_2;
			        //ComBuf[BEGIN_DATA+3] = OP_Mode;
     			#endif Sens_2_Valid
	                #ifdef Sens_3_Valid
	                if(sensor_num == 3)
			        ComBuf[BEGIN_DATA+3] = data_format_3;
     			#endif Sens_3_Valid
	                data_len = 1;
                        break;
        #ifdef Modem_Comm
        case 10: //return modem parameters
            //get hour1
            ComBuf[BEGIN_DATA+3] = Start_H1;
            //get hour2
            ComBuf[BEGIN_DATA+4] = Start_H2;
            //get dest. phone number
            cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+5], Dest_Modem_Num, 12);
            //get internal phone number
            cpu_e2_to_MemCopy( &ComBuf[BEGIN_DATA+17], Int_Modem_Num, 12);
            //get measuring interval
 			#ifdef Sens_1_Valid
 		        ComBuf[BEGIN_DATA+29] = cpue2_interval_1;
			#endif Sens_1_Valid

			#ifndef Sens_1_Valid
			#ifdef Sens_2_Valid
		        ComBuf[BEGIN_DATA+29] = cpue2_interval_2;
			#endif Sens_2_Valid
			#endif Sens_1_Valid

            //get rssi_val
            ComBuf[BEGIN_DATA+30] = MinRSSI;

            data_len = 28;//27;
            break;

        case 'e': //return sensor id gor GPRS socket in pc
                //disable interrupts
                #asm ("cli")
                delay_ms(1);
                //copy sensor #1 ID to buff
                //MemCopy( &ComBuf[BEGIN_DATA+3],&SensorAddress[0],4 );
                //chane from MemCopy, trying to ignor corraption of sensor id
                ComBuf[BEGIN_DATA+3] = SensorAddress[0];
                ComBuf[BEGIN_DATA+4] = SensorAddress[1];
                ComBuf[BEGIN_DATA+5] = SensorAddress[2];
                ComBuf[BEGIN_DATA+6] = SensorAddress[3];
                data_len = 4;
                #ifdef Sens_2_Valid
                //add sensor #2 id
                MemCopy( &ComBuf[BEGIN_DATA+7],&SensorAddress[4],4 );
                data_len = 8;
                #endif Sens_2_Valid
                #ifdef Sens_3_Valid
                //add sensor #3 id
                MemCopy( &ComBuf[BEGIN_DATA+11],&SensorAddress[8],4 );
                data_len = 12;
                #endif Sens_3_Valid
                //enable iterrupts
                #asm ("sei")
                delay_ms(1);

                break;

        #endif Modem_Comm

        default:        break;

    }
	//preper tx data for rf
	ComBuf[BEGIN_DATA] = 'v';
	ComBuf[BEGIN_DATA+1] = data_len+1; //data length
	ComBuf[BEGIN_DATA+2] = data_code;
	LengthOfData = data_len+3; //default data size

	return 1;
}
*/

/*
//save the sensor measuring data into ext_e2
char save_measure_data()
{
	char meas_data_buf[2];
	//check if next writing point will cross e2 block (256 bytes) limit
	int2bytes(pWrite, meas_data_buf); //pwrite drvide into 2 bytes
	if(meas_data_buf[1] == 0xFF)//if lo byte is at the e2 block limit
	{
		meas_data_buf[0] += 1;	//add 1 to hi byte
		meas_data_buf[1] = 0x00;//set 0 to lo byte
		//set write pointer
		pWrite = bytes2int(meas_data_buf);
	}
	else
		//move pWrite to next write position
		pWrite += 2;

	//set SensorResult into 2 bytes
   	int2bytes(SensorResult, meas_data_buf);

	//write current sensor measurment onto ext_e2
	if(!(e2_writePage(pWrite, 2, meas_data_buf)))
		return 0;//if writing into ext_e2 faild exit faild
	//else
	return 1;

}
*/
///////// end of data manager module ///////////
////////////////////////////////////////////////


 #include "define.h"
 #include "twi.h"    
 #include <iobits.h>
char MemoryReadTest(unsigned int block, unsigned int start, unsigned int end);

char ptr;
bit BUF1_FULL;
bit BUF2_FULL;
bit Buf1_Read;
bit Buf2_Read;
//bit Folded = FALSE;
//bit LowMemFull = FALSE;
bit tmpBuf_full = FALSE;
//bit BufNotEmpty = FALSE;
bit E_O_F = 0;
bit HighMemFlag = 0;


#define PageSize 128
#define RX_BUFFER_SIZE1 PageSize
char tmp_buf [150];    //larger than page size=128

//#define highBit 2      //ATMLH030 memory chip
#define highBit 8      //24L1025i Creacell board 7

unsigned int GlobChksum;

//char rx_buffer1[50];
//char rx_buffer2[50];

unsigned char gIndx ;
unsigned char BytesCount;
unsigned int  RowsCount;
long MemIndex = 0;
extern bit  ServerComOn;
extern char Timer0_Ticks;
extern unsigned int Timer4Count;
extern bit bCheckRxBuf;
extern bit UpdateSession;
//extern int Timer0_10msCount;       //count TOF2 int
extern bit Measure_Timer_Active;        //flag checked in TOF2
extern bit bWaitForModemAnswer;

extern bit FirmwareUpdateTime;
extern int heat_time;

extern char RxUart0Buf[];    //100
extern char RxUart1Buf[];     //100
extern char chksumBuf[];
extern unsigned int rx0_buff_len;
extern unsigned char rx1_buff_len;
extern int ServerResponseTimeOut;
extern eeprom unsigned char eUseCntrCode;

extern int twiWriteMemN(char block, unsigned int addr, unsigned char n, char *pdata);
extern char ReadEEPCurrentAddress(char EEPaddr);
extern char twiReadEEP1Byte(char block,  unsigned int InternalAddr );
extern void ShowHexString(unsigned char *message, char Lengh);
extern void ShowHexByte( char byte);
extern void MUX_SDI12_00(void);
extern void SendPostAckUpdate(char index);
extern  void PCmessage( char *message );
//extern void PrintNum3(long val);
//extern void putchar3(char c);

void Send_Char(unsigned char ch) //send one character via the serial port
{

	while (!(UCSR1A & 0x20))
		;    //wait xmit ready
	UDR2 = ch;                 //send the character
}
 ////now replace get byte
unsigned char Get_Char(void)  //get one character from the serial data source
{
  unsigned char rch, EEPchip;
 // unsigned char  filtered_char;


     if( HighMemFlag == 0)
      EEPchip = 0xA0;
      else
      EEPchip = (0xA0 | highBit);   //

    if(MemIndex == 0x10000)
    {
           rch =  twiReadEEP1Byte(1 , 0); //set address at upper section
            HighMemFlag = 1;
    }
    else
    rch =  ReadEEPCurrentAddress(EEPchip);  //read memory current address - self incrimented

        MemIndex++;
  //     ShowHexByte(rch);   //gebugggggggggggggggggggggggg
       return (rch );

}

unsigned int Get_Word (void)   //read word of data from data source
{
    unsigned int ch_word;

//    ch_word = Get_Byte() * 256;   //msb byte
//    ch_word = ch_word + Get_Byte(); //lsb byte

    ch_word = (unsigned int)( Get_Char() * 256);   //msb byte
    ch_word = (unsigned int)(ch_word + Get_Char()); //lsb byte

    return ch_word;
}

unsigned char Get_Byte (void) //read byte of data from data source
{
    unsigned char ch_byte;
    //TxMsg("8");
    ch_byte = Get_Char() * 16;  //msb digit

    ch_byte = ch_byte + Get_Char();  //lsb digit

    return ch_byte;
}

//the next two functions would need to be changed to use a data source other than
//the serial port


char CharsFilter(char rch)
{
      char filtered_char;

   //filter character into hex digits or special characters
   if ((rch >= 0x30) && (rch<0x3A))
      filtered_char = rch-0x30;  //numeric digit
   else if (rch > 0x40)
      filtered_char = (rch & 0xdf) - 55;    //alpha digit
   else if (rch < 0x20)
      filtered_char = 0xff;        //dump the line feeds, etc.
   else
      filtered_char = rch;   //unconverted digit

   return (filtered_char);            //return character
}

char Ascii2Bin(char *str)
{
  char ptr, j, i, count;

         
           if(gIndx == 0)
           {
                 if(str[0]== '[')  //look for start
               {
                    tmp_buf[gIndx++] = str[0];   //[
                    tmp_buf[gIndx++] = str[1] - 0x30; //1                      
                    tmp_buf[gIndx++] = str[2]; 
                                      
                     _putchar1('S');
                }
           }

            j = 0;       //j = 0 - do not change
            while((str[j] != ':') && (j < 7)) //look for first ':'  
            {               
             //  _putchar1(str[j]);
                 j++;  
            }
            if(j == 7)
            {
                 _putchar1('!'); 
                PrintNum((long) RowsCount);
                  return 0;
            }
            else   //found ':'
            {
                RowsCount++;
                ptr = j;
                tmp_buf[gIndx++] = str[ptr++];    //get :
                //get num of data bytes in the current ro              
           
                 count = (CharsFilter(str[(int)ptr++]) * 16) + CharsFilter(str[(int)ptr++]) ;

                tmp_buf[gIndx++] = count ;        //num of bytes

                if(count > 0)  //regular  row
                {
                    i = 0;
//                    ptr++;         //skip num of bytes
//                    ptr++;
                  do
                    {
                       tmp_buf[gIndx++] = (CharsFilter(str[ptr++]) * 16) + CharsFilter(str[ptr++]) ; //get 2 bytes and bin it to 1
                       i++;
                    }
                    while (i < (count + 4));    //plus address and checksum)

                    if( gIndx >= PageSize)  //buf is full. go to write it
                    {
                       tmpBuf_full = TRUE;
                    }
                }
                else      //count = 0- end of file
                {
                    tmp_buf[gIndx++] = 0;
                    tmp_buf[gIndx++] = 0;
                    tmp_buf[gIndx++] = 1;
                    tmp_buf[gIndx++] = 0xFF;

                    tmpBuf_full = TRUE;
                    E_O_F = 1;
                    _putchar1('%');
                }
                
                j = gIndx;
            }


            return j;   //buffer size

}

#pragma used-
//READ ALL FILE FROM MEM AND CHECK ALL CS VALUES. MAKE SURE WRITING TO MEME OK
int ValidityCheck (unsigned int Saddress)
 {
	unsigned char num_bytes;       //line buffer number of bytes

	unsigned char Var;                   //general varaiable
	 int Errs = 0;                //error counter
	unsigned char CS;
    unsigned int Addr_start;
    unsigned char k;
    char EOF_code = 0;
    unsigned int RowsCounter; 
    unsigned int VerticalChksum;  
    unsigned char tVal; 
    char str[40];
    
     SendDebugMsg("File Validity check..\r\n\0") ; 
      SendDebugMsg("Rows expected = \0") ;
     PrintNum((long)RowsCount);

 	Var = 0;
  	Errs = 0;
    k=0;
    HighMemFlag = 0;
    MemIndex = 1;  
    VerticalChksum = 0;
    
     CS = 5;  
//    ShowHexString(&chksumBuf[0], 1);
//     ShowHexString(&chksumBuf[1], 1);
//      ShowHexString(&chksumBuf[2], 1);
 //      ShowHexString(chksumBuf, 4); 
     
     //convert vertical checksum to bin 
     tVal = ((unsigned char)(CharsFilter(chksumBuf[0]) * 16) + CharsFilter(chksumBuf[1]));  //calc most sig byte 
                   
     GlobChksum = (unsigned int)(tVal *256); 
           
     tVal = ((unsigned char)(CharsFilter(chksumBuf[2]) * 16) + CharsFilter(chksumBuf[3]));  //add leat sig byte     
     GlobChksum += tVal;   
                                                                                     //to be checked later
    

     Var =  twiReadEEP1Byte(0 , Saddress);  //read mem address 0
     do 	//wait for colon - start of line od data
      {
           //    ShowHexString(&Var,1);

             Var = Get_Char();      //read mem current address

             if (Var == 2)  //if eeprom only
             {
                  SendDebugMsg("Check EEP..\r\n\0") ;
             }

            else if (Var == 3)  // if this symbol is received, eeprom file first, then flash
            {
                     SendDebugMsg("Both..\r\n\0") ;
            }
          // else  if (Var == 1)  SendDebugMsg("Flash..\r\n\0") ;
           CS--;

      }  while ((Var != ']') && (CS > 0));

       if(CS == 0)  //we have found : -row start
       {
             Errs = 255;  //out
             SendDebugMsg("No file mark found..\r\n\0") ; 
             MemoryReadTest(0,0,8) ;
       }
       else
       {
           RowsCounter = 0;
          do  
            {
                     if((RowsCounter % 50) == 0)  
                     {
                         LED1_ON;
                          Send_Char('~');  
                     } 
                   
                    Var = Get_Char();    //get : of next row of hex
                     CS = Get_Char();
                    num_bytes = CS;              //get number of bytes  in row -hex file

                        Addr_start = Get_Word();     //get starting address of current line
                        CS += Addr_start & 0xFF;     //build checksum on CS
                        CS += ((Addr_start >> 8) & 0xFF);

                         EOF_code = Get_Char(); // get data type code                           
                        
                         if(EOF_code == 1)
                         {
                             RowsCounter++;
                             SendDebugMsg("found EOF..\r\n\0") ;
                             goto END;
                             //break;
                         } 
//                         else
//                         {
//                              if(EOF_code == 2)
//                              SendDebugMsg("EOF_code == 2\r\n\0") ;
//                         }
                         
                          CS += EOF_code;

                        //now get data line into buffer if not eof
                        for (k = 0;k < num_bytes;k++)
                        {
                             Var = Get_Char();
                              CS += Var;
                      
                        }

                        Var = Get_Char();                 //get checksum  
                        tVal = ((CS-1) ^ 0xFF);
                        if (tVal != Var )      //compar checksums
                        {                        
                            Errs++;  
                            sprintf(str,"\rCS Error row: %d - Calc: %02x - expected: %02x\r\n\0",RowsCounter,tVal,Var);
                             PCmessage(str);
                         //   Send_Char('R');             //debug only
                         //   PrintNum((long)RowsCounter);  //debug only
                        } 
                        else
                        {
                             VerticalChksum += Var;
                        }

//                          Send_Char('~');        //debug only
                           RowsCounter++;
                              LED1_OFF;
//                   }
//                   else  //end of file mark? //  01FF
//                   {
//                          Var = Get_Char();
//                          Var = Get_Char();
//                          Var = Get_Char();
//                          if(Var != 1)
//                         Errs++;
//                        Send_Char('?');  //debug only
//                   }
                  END:
                   #asm("wdr");
            }  while((EOF_code != 1) && (RowsCounter < RowsCount));
         
            if(VerticalChksum == GlobChksum)
            {     
                   SendDebugMsg("Vertical Checksum OK..\n\r\0") ;              
            }
            else
            {                   
                   SendDebugMsg("\r\nVertical Checksum Error..!\n\r\0") ;
                    sprintf(str,"\rVCS Error: Calc: %04x - expected: %04x\r\n\0",VerticalChksum,GlobChksum);
                             PCmessage(str); 
                    Errs = 99;
            } 
             
            SendDebugMsg("\r\nValidity check ended..Errors= \0") ;
            PrintNum((long)Errs);
             PrintNum((long)MemIndex);   //debug 
            PrintNum((long)RowsCounter);   //debug

        }
      
          return Errs;
}

unsigned int RxUpdateFile(void)
{

        char i,k,m;
    //   unsigned int Waddress; 
      long Waddress; 
       
        bit  MemSec = 0;

       tmpBuf_full = FALSE;     
       BUF1_FULL = FALSE;
       BUF2_FULL = FALSE;
       Buf1_Read = TRUE;
       Buf2_Read = TRUE; 
       bCheckRxBuf = FALSE;
       rx0_buff_len = 0;
       rx1_buff_len= 0;        
        MemSec = 0;
       Waddress = 0; 
       gIndx  = 0;
       RowsCount = 0;
       E_O_F = 0;
       k = 0;
       m=0;
       Measure_Timer_Active = FALSE;
       LED1_ON;            //debug
  //-----------------debug -RX via cable- uart 2------------------------------
     
    
        FirmwareUpdateTime = TRUE;   /////////test uart 2/////////////////// 
       
//         MUX_SDI12_00();
//          ENABLE_UART2();
 //--------------------------------------------------------
       SendDebugMsg("\r\nRX update file..\r\n\0");  
       ServerComOn = TRUE;
       SendPostAckUpdate(2);        //post ask server for update file   
       //----------------------------- 
        UBRR0L=0x17;    //38400 
   //     UBRR0L=0x0F;    //57600  
   //      UBRR0L=0x03;    //115200     
       
       //------------------------ 
       
        heat_time  = 40;   //?? 15 S  max waiting         
        ENABLE_TIMER1();             
                                        
         while(( UpdateSession == FALSE)  && (heat_time > 0))  //wait for start of file token to set flag
         #asm("wdr");  
    //---------------------------     
         DISABLE_TIMER4();
         Timer4Count  = 1000; 
    //----------------------------      
       if(heat_time == 0)     //file start not detected - abort
       {          
//             UpdateSession = FALSE;   not here because RX int beaviour
              ENABLE_UART2();
             SendDebugMsg("\r\nRX FAILED- file time out..!\r\n\0");
             delay_ms(100);
             return 0;
       }  
              
     //   ENABLE_TIMER4();         
        heat_time  = 55;
        DISABLE_TIMER2();
        bCheckRxBuf = FALSE;
        Timer0_Ticks = 15;
        bWaitForModemAnswer = TRUE;  
         eUseCntrCode = '?';       //eeprom flag - fota process start
   

   while(E_O_F == 0)
 //    while(1)
    {
              #asm("wdr")
                 if(( BUF1_FULL == TRUE)&& (Buf1_Read == FALSE)&& (tmpBuf_full == FALSE))   //set in RX int when data byte recieved
                 {
                         BytesCount = Ascii2Bin(RxUart0Buf);  //14 ticks
                         rx0_buff_len = 0;
                         Buf1_Read = TRUE;                     
                         heat_time  = 35;      //set timer for count down

                 }

                if(( BUF2_FULL == TRUE)&& (Buf2_Read == FALSE)&& (tmpBuf_full == FALSE))
                {
                        BytesCount = Ascii2Bin(RxUart1Buf);
                        rx1_buff_len = 0;
                        Buf2_Read = TRUE;
                        heat_time  = 35;
                 }

                 if(tmpBuf_full == TRUE)  //buffer is ready to write into mem
                 {
                      tmpBuf_full = FALSE;
                       LED1_ON; 
                       
                       
                       GRE_LED_ON;                         
                    //  TOGGLE_BLU_LED();
                       
                     //  ShowHexString(tmp_buf, BytesCount);    //debug ??????????????? 
                                           
                            
                       if(BytesCount < PageSize)
                                k = twiWriteMemN(MemSec ,(unsigned int) Waddress, BytesCount ,tmp_buf);
                       else
                       {      
//                              if(m == 0)
//                               Timer4Count  = 1000;
                           
                              k = twiWriteMemN(MemSec, (unsigned int)Waddress, PageSize ,tmp_buf);    //  22 mS!
                            //  DISABLE_TIMER4(); 
                       }
                      if(k)     //success
                      {
                            m++;
                           LED1_OFF;
                            GRE_LED_OFF; 
                            
                           //set new address
                           if(BytesCount < PageSize)  //last page?                           
                                Waddress += BytesCount;                                                
                           else 
                               Waddress += PageSize;
                                                                                                                                                                     
                            if(Waddress >= 0x10000)  //last low section page was written- reached end of low mem +1 =10000h
                            {     
                                
                                   MemSec = 1;   //now in upper mem
                                   Waddress = 0;  //address 0 again
                                   putchar1('$');
                                 
                            }                               
                          
                              if(BytesCount > PageSize)      //extra bytes in buffer taken to next session
                              {
                                  for(i = 0; i < (BytesCount - PageSize); i++)
                                  tmp_buf[i] = tmp_buf[i + PageSize];  //save extra bytes to at start of buf
                                  gIndx = i;
                              }
                              else  gIndx = 0;    //new buffer index

                              if(E_O_F == 1)     //end of file
                              {       
                                  //    UBRR0L=0x2F;  //19200
                                        _putchar1('E');                   
                                        break;  //end of file recieved
                              }

                      }
                     else    //k = 0, failed
                     {
                       //   UBRR0L=0x2F; 
                          _putchar1('?'); _putchar1('?');
                           heat_time = 0;  //error
                         break;
                     }
                 }

                if(bCheckRxBuf == TRUE)
                {
                                       
                       if(rx0_buff_len > 0)   //handle partial buffer. last received bytes
                       {
                          BUF1_FULL = TRUE ;
                          Buf1_Read = FALSE;
                          rx0_buff_len = 0;
                       }

                       else if(rx1_buff_len > 0)
                       {
                             BUF2_FULL = TRUE ;
                            Buf2_Read = FALSE;
                            rx1_buff_len = 0;
                       }                                               
                }
                if( heat_time  == 0)
                {
                      Waddress = 0;
                      _putchar1('?');
                      break;
                }
      }

       
      DISABLE_TIMER4(); 
       ENABLE_UART2(); 
       UBRR0L=0x2F;    //19200
      LED1_OFF;
       ServerResponseTimeOut = 70;
      Measure_Timer_Active = FALSE;
 //   UpdateSession = FALSE;    not here
  
       PrintNum((long)RowsCount);  
        if(MemSec == 1)      
          PrintNum((long)Waddress+0xFFFF);
        else
          PrintNum((long)Waddress); 
          
          PrintNum((long)Timer4Count);
          
       bCheckRxBuf = FALSE;
       rx0_buff_len = 0; 
       rx1_buff_len = 0;
         Timer0_Ticks = 12;   
       
      return Waddress;

   }  
   
   char MemoryReadTest(unsigned int block, unsigned int start, unsigned int end)
   {
         unsigned char k;
       //  unsigned int i;  
     //   char MemAdd =  0xA0; 
        char data[] = {0x39};  
        
      //     if(block == 1) MemAdd = 0xA8; 
           SendDebugMsg("Mwmory test..!\r\n\0");  
          
    
          k =  twiWriteMemN( block, start, end, data);  
          if(k > -1 ) 
          {       
            delay_ms(100);
           k =  twiReadEEP1Byte(block , start);  //block 0
           if(k > -1 ) 
           {      
                 ShowHexString(&k, 1);
                SendDebugMsg("read first byte ok..!\r\n\0");
////               _putchar1(k);         
//               for (i = 0; i < end; i++)
//               {
//                     k =  ReadEEPCurrentAddress(MemAdd); 
//                     ShowHexString(&k, 1);
//                   //    _putchar1(k);
//                    //   delay_ms(1);
//               }  

           } 
           } 
           return k;
         
   }
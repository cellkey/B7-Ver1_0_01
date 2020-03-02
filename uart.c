#include <string.h>
#include "define.h"
#include <stdlib.h>
#include <stdio.h>
#include <bcd.h>

#define TBUF_MAX_LENGTH 40


#ifndef RXB8
#define RXB8 1
#endif

#ifndef TXB8
#define TXB8 0
#endif

#ifndef UPE
#define UPE 2
#endif

#ifndef DOR
#define DOR 3
#endif

#ifndef FE
#define FE 4
#endif

#ifndef UDRE
#define UDRE 5
#endif

#ifndef RXC
#define RXC 7
#endif

//#define FRAMING_ERROR           (1<<FE)
//#define PARITY_ERROR            (1<<UPE)
//#define DATA_OVERRUN            (1<<DOR)
//#define DATA_REGISTER_EMPTY     (1<<UDRE)
//#define RX_COMPLETE             (1<<RXC)

//extern void putchar(char c);
extern BYTE rx1_buff_len;
extern char RxUart1Buf[MAX_RX_BUF_LEN];

// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 64
//char rx_buffer0[RX_BUFFER_SIZE0];

#define RX_BUFFER_SIZE1 64
char rx_buffer1[RX_BUFFER_SIZE1];


char tx_buffer1[TX_BUFFER_SIZE1];

 unsigned char rx_rd_index1,rx_counter1;
 unsigned char tx_wr_index1,tx_counter1;
// unsigned char tx_rd_index1,rx_wr_index1;

// This flag is set on USART0 Receiver buffer overflow
//bit rx_buffer_overflow0;
#define uint16_t unsigned


// USART Receiver interrupt service routine
// interrupt [USART0_RXC] void usart0_rx_isr(void)
//
//{
//    char status,data;
//
//    StatLED_ON();
//    status = UCSR0A;
//    data = UDR0;
//    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
//    {
//               rx_buffer0[rx_wr_index0++] = data;
//               if (rx_wr_index0 == RX_BUFFER_SIZE0)
//               rx_wr_index0=0;
//               if (++rx_counter0 == RX_BUFFER_SIZE0)
//               {
//                  rx_counter0=0;
//                  rx_buffer_overflow0=1;
//               }
//       }
//}

// This flag is set on USART1 Receiver buffer overflow
//bit rx_buffer_overflow1;

// USART1 Receiver interrupt service routine
//interrupt [USART1_RXC] void usart1_rx_isr(void)
//{
//    char status,data;
//
//    TURN_LED2_ON();
//    status = UCSR2A;
//    data = UDR2;
//        if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN)) == 0)
//        {
//              rx_buffer1[rx_wr_index1++]=data;
//               if (rx_wr_index1 == RX_BUFFER_SIZE1)
//               rx_wr_index1 = 0;
//               if (++rx_counter1 == RX_BUFFER_SIZE1)
//               {
//                  rx_counter1 = 0;
//                  rx_buffer_overflow1 = 1;
//               }
//           }
//}

// Not busy wait
// Return 0 if buffer empty
//9/8/14 change for uart1
char UART_Getc(void)
{
    int data;

    if(rx_counter1 == 0)
     return 0;

    data = rx_buffer1[rx_rd_index1++];

    if (rx_rd_index1 == RX_BUFFER_SIZE1)
    rx_rd_index1 = 0;

    #asm("cli")
    --rx_counter1;
    #asm("sei")


    return data;
}


#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART0 Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void)
{
    char data;

    while (rx_counter1 == 0);
  data = rx_buffer1[rx_rd_index1++];
   //  data = RxUart1Buf[rx_rd_index1++];
   if (rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1=0;

    #asm("cli")
    --rx_counter1;
    #asm("sei")

      return data;
}
#pragma used-
#endif

// USART0 Transmitter buffer
#define TX_BUFFER_SIZE0 120
char tx_buffer0[TX_BUFFER_SIZE0];

unsigned char tx_wr_index0,tx_counter0;

//// USART0 Transmitter interrupt service routine
//interrupt [USART0_TXC] void usart0_tx_isr(void)
//{
//        if (tx_counter0)
//        {
//           --tx_counter0;
//           UDR0 = tx_buffer0[tx_rd_index0++];
//           if (tx_rd_index0 == TX_BUFFER_SIZE0) tx_rd_index0 = 0;
//
//        }
//}
//

#ifndef _DEBUG_TERMINAL_IO_
// Write a character to the USART0 Transmitter buffer
#define _ALTERNATE_PUTCHAR_
#pragma used+
void putchar0(char c)
{
        while (tx_counter0 == TX_BUFFER_SIZE0);
   //    #asm("cli")
        if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY)==0))
       {
           tx_buffer0[tx_wr_index0++] = c;
           if (tx_wr_index0 == TX_BUFFER_SIZE0)
           tx_wr_index0 = 0;
           ++tx_counter0;
       }
        else
        UDR0 = c;
   //     #asm("sei")
}
#pragma used-

//#define _ALTERNATE_PUTCHAR_
#pragma used+
void putchar(char c)
{
        while (tx_counter0 == TX_BUFFER_SIZE0);
   //    #asm("cli")
        if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY)==0))
       {
           tx_buffer0[tx_wr_index0++] = c;
           if (tx_wr_index0 == TX_BUFFER_SIZE0)
           tx_wr_index0 = 0;
           ++tx_counter0;
       }
        else
        UDR0 = c;
   //     #asm("sei")
}
#pragma used-

// Get a character from the USART1 Receiver buffer
#pragma used+
char getchar1(void)
{
    char data;
    while (rx_counter1 == 0);
    data = rx_buffer1[rx_rd_index1++];

    if (rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1 = 0;

    #asm("cli")
    --rx_counter1;
    #asm("sei")
    return data;
}
#pragma used-

// Get a character from the USART2 Receiver
#pragma used+
char getchar2(void)
{
unsigned char status;
char data;
while (1)
      {
      while (((status=UCSR2A) & RX_COMPLETE)==0);
      data=UDR2;
      if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
         return data;
      }
}
#pragma used-
// USART1 Transmitter buffer
//#define TX_BUFFER_SIZE1 64
char tx_buffer1[TX_BUFFER_SIZE1];

//#if TX_BUFFER_SIZE1 <= 256
//unsigned char tx_wr_index1,tx_rd_index1,tx_counter1;
//#else
//unsigned int tx_wr_index1,tx_rd_index1,tx_counter1;
//#endif

// USART1 Transmitter interrupt service routine
//interrupt [USART1_TXC] void usart1_tx_isr(void)
//{
//if (tx_counter1)
//   {
//   --tx_counter1;
//    UDR2 = tx_buffer1[tx_rd_index1++];
//
//   if (tx_rd_index1 == TX_BUFFER_SIZE1) tx_rd_index1 = 0;
//
//   }
//}

// Write a character to the USART1 Transmitter buffer
#pragma used+
void putchar1(char c)
{
        while (tx_counter1 == TX_BUFFER_SIZE1);
     //   #asm("cli")
        if (tx_counter1 || ((UCSR2A & DATA_REGISTER_EMPTY) == 0))
           {
               tx_buffer1[tx_wr_index1++]  =c;

               if (tx_wr_index1 == TX_BUFFER_SIZE1)
               tx_wr_index1 = 0;
               ++tx_counter1;
           }
           else
           UDR2 = c;
   //     #asm("sei")

}
void _putchar1(char c)
{
    while ((UCSR2A & DATA_REGISTER_EMPTY)==0);
    UDR2 = c;  
    UDR3 = c; 
   // while ((UCSR2A & DATA_REGISTER_EMPTY)==0);
}

void _putchar0(char c)
{
    while ((UCSR0A & DATA_REGISTER_EMPTY)==0);
    UDR0 = c;
   // while ((UCSR2A & DATA_REGISTER_EMPTY)==0);
}

void _putchar3(char c)
{
    while ((UCSR3A & DATA_REGISTER_EMPTY)==0);
  
    UDR3 = c; 
   // while ((UCSR2A & DATA_REGISTER_EMPTY)==0);
}
#pragma used-
#endif

// =====================================================
// SysMsg -sends string to monitor
// =====================================================
//void SendDebugMsg(flash char *InStr)
//{
//    int length = 0;
//    int i = 0;
//
//    if(InStr == NULL)
//        return;
//
// //     length = strlen(InStr);
//      while((InStr[++length] !='\0') && length < MAX_LENGTH); //check lengh
//      if(length < MAX_LENGTH)
//      {
//          StatLED_ON();
//          for(i= 0 ; i <length ; i++)
//          {
//
//                delay_us(800);
//                putchar0(*(InStr+i));
//
//           }
//           StatLED_OFF();
//    }
//}
// =====================================================
// UART_WriteStr
// =====================================================
//9/8/14 uart1 for creacell test
void UART_WriteMsg( char *InStr)
{
    int length = 0;
    int i = 0;

    if(InStr == NULL)
        return;

    length = strlen(InStr);
//   while((InStr[++length] !='\0') && length < TBUF_MAX_LENGTH);
    if(length < TBUF_MAX_LENGTH)
    {
//        StatLED_ON();
        for(i= 0 ; i <length ; i++)
        {
            delay_us(800);
             _putchar1(*(InStr+i));
           //  _putchar3(*(InStr+i));
        }

    }
}

void UART3_WriteMsg( char *InStr)
{
    int length = 0;
    int i = 0;

    if(InStr == NULL)
        return;

    length = strlen(InStr);
//   while((InStr[++length] !='\0') && length < TBUF_MAX_LENGTH);
    if(length < TBUF_MAX_LENGTH)
    {
//        StatLED_ON();
        for(i= 0 ; i <length ; i++)
        {
              while ((UCSR3A & DATA_REGISTER_EMPTY) == 0);  // wait for data to be transmitted
             _putchar3(*(InStr+i));

        }

    }
}
//
//#ifdef RS485_ACTIVE
 void Uart1MsgN( char *message, char Lengh )
{

       	unsigned char i = 0;
	    unsigned char message_length = 0;


        message_length = Lengh;
      // 	message_length = (unsigned char)strlen(message);
        for(i = 0; i < message_length; i++)
        {
	           while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);  // wait for data to be transmitted
               UDR2 = message[i];
             // while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);  // wait for data to be transmitted
        }

}

//#endif


//  void TxByte(char data )
//{
//
//    #asm("push R30")   //  ;save pointers
//    #asm("push R31")
//
//       while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);  // wait for data to be transmitted
//        UDR2 = data;
//   //     while ((UCSR0A & DATA_REGISTER_EMPTY)==0);
//
//   #asm("pop R31")   //  ;and re use it
//   #asm("pop R30")
//
//
//}

// =====================================================
// UART_GetNum - get num as string from user/ convert to bnin
// =====================================================
//unsigned int UART_GetNum(void)
//{
//    char c,i,k,m;
//    unsigned int num = 0;
//   unsigned char buf[5];
//   // char Str[15];
//
//
//     for(i = 0; i < 5; i++)
//     buf[i] = '0';
//
//     i = 0;
//    do
//     {
//        //  while((c = UART_Getc()) == 0);
//           while((c = getchar()) == 0);
////          if((c == '\r') ||(i == 5)) break;
////          else
////          {
//
//                if(c == 8)                      //back space
//                {
//                        if(i == 0) break;
//                        i--;
//                        TxByte('\b');
//                        TxByte(' ');
//                        TxByte('\b');
//                 }
//                 else if((c != 0) && (c > 47) && (c < 58) && (i <5))
//                 {
//                        buf[i] = c;
//                        TxByte(buf[i]);       //show on screen
//                        i++;
//
//                 }
//          // }
//      }  while(c != '\r' );
//
//     if(i == 0)       //no entered digit
//     return 0;
//
//     if(i < 5)
//     {
//          m = i ;
//          k = 4;
//        do
//          {
//               m--;
//               buf[k] = buf[m];    //shift num to right. list sig
//               buf[m] = '0';
//               k--;
//          }  while ( m >0);
//
////  for(m = 0; m<5; m++)
////  {
////       TxByte(buf[m]);
////  }
//
////          k++;
////         do{
////             k--;
////            buf[k] = '0';
////           }while(k>0);
//    }
//       num = STR2BIN(buf);
//
//    return num;
//}

////used to send nessages to screen
//void TxMsg( char flash *str)
//{
//
//  //  ToPC ;  // select PC
//  PORTC.2 = 0;
//  PORTC.3 = 0;
//  delay_ms(20);
//#asm
//
//      ld     r30,y       ; Z regist
//      ldd    r31,y+1
// __loop0:
//      lpm                ; load [Z] to r0
//      tst    r0          ; test  zero
//      breq __end         ; out if 0
//      adiw r30,1         ;increment pointer
//      st   -y,r0          ; store value held by r0 as parameter
//
//     rcall _TxByte  ;transmit byte
//      rjmp __loop0
//__end:
//
//#endasm
//
//   //  delay_ms(100);
//
//}

////9/8/14 uart1 for creacell test
unsigned int UARTGetBin(void)
{

	#define LENGTH 20
	int i=0;
	char c, input[LENGTH];

	while(( c = getchar1()) != '\n' && c != '\r' && i < LENGTH)
    {
		if(c == '\b')
        {
			if(i>0)
            {
                 _putchar1(c);
                 _putchar1(' ');
                 _putchar1(c);
                i--;
			}
		}
		else
        {
			input[i++] = c;
			 _putchar1(c);
		}
	}

    if(i == 0)      //no input from user
    {
        return 0;
    }
    else

    {
         input[i] = '\0';
      	 return (uint16_t)atoi(input);
    }

}


void BIN2STR(unsigned int BinData, char *str)
  {

  unsigned int tmp,tmp1 ;
  unsigned char tmpBCD;

     tmp = BinData / 1000;
  str[0] =(unsigned char)tmp + 0x30; //most sig digit
   tmp = BinData % 1000;

  tmp1 = tmp /100;
  str[1] = (unsigned char)tmp1+ 0x30;
  tmp1 = tmp % 100;

  tmpBCD = (unsigned char)bin2bcd(tmp1);

  str[2] = ((tmpBCD >> 4) & 0x0f) +0x30;
  str[3] = (tmpBCD  & 0x0f) +0x30;
 }

   void PCmessage( char *message )
{

       	unsigned char i = 0;
	    unsigned char message_length = 0;

      //  while (message[i] != '\0') message_length++;
       	message_length = (unsigned char)strlen(message);
     //   UDR2 = message[0];
        for(i = 0; i < message_length; i++)
        {
	           while ((UCSR2A & DATA_REGISTER_EMPTY) == 0);  // wait for data to be transmitted
               UDR2 = message[i];
        }
    //    RX1intEN;  // allow getting response from modem.

}
//used to show data string as decimal nums
void ShowHexString( char *message, char Lengh)
{
        char str[5];
      // char CRLF[] = "\r\n\0";
       unsigned char i;
       unsigned char message_length = 0;
     //   unsigned char  tmpBub[2];
    //    int dataWord;

//       SetMUX2PC();
       message_length = Lengh; //(unsigned char)strlen(message);
       for(i = 0; i < message_length; i++)
       {
               sprintf(str,"%02X \0", message[i]);
               PCmessage(str);
       }
//      _putchar1('\r');
//       _putchar1('\n');
//      delay_ms(50);
        
}

void ShowHexByte( char byte)
{
       char str[5];    
       sprintf(str,"%02X \0", byte);
       PCmessage(str);  
//        _putchar1(byte); 
//          _putchar1(' ');
}

//handle analog mux - monitor dierction
void SetMUX2PC(void)
{
     CLRBIT(PORTE,2); ;
     CLRBIT(PORTE,3);
}
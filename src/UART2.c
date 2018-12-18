// UART2.c
// Runs on LM3S1968
// Use UART0 to implement bidirectional data transfer to and from a
// computer running HyperTerminal.  This time, interrupts and FIFOs
// are used.
// Daniel Valvano
// October 9, 2011
// Modified by EE345L students Charlie Gough && Matt Hawk
// Modified by EE345M students Agustinus Darmawan && Mingjie Qiu

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to the Arm Cortex M3",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2011

   Program 5.11 Section 5.6, Program 3.10

 Copyright 2011 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

#include "FIFO.h"
#include "UART2.h"
#include "lm3s1968.h"

#define NVIC_EN0_INT5           0x00000020  // Interrupt 5 enable
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI1_R             (*((volatile unsigned long *)0xE000E404))  // IRQ 4 to 7 Priority Register
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define UART0_DR_R              (*((volatile unsigned long *)0x4000C000))
#define UART0_FR_R              (*((volatile unsigned long *)0x4000C018))
#define UART0_IBRD_R            (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R            (*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH_R            (*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL_R             (*((volatile unsigned long *)0x4000C030))
#define UART0_IFLS_R            (*((volatile unsigned long *)0x4000C034))
#define UART0_IM_R              (*((volatile unsigned long *)0x4000C038))
#define UART0_RIS_R             (*((volatile unsigned long *)0x4000C03C))
#define UART0_ICR_R             (*((volatile unsigned long *)0x4000C044))
#define UART_FR_RXFF            0x00000040  // UART Receive FIFO Full
#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8        0x00000060  // 8 bit word length
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_CTL_UARTEN         0x00000001  // UART Enable
#define UART_IFLS_RX1_8         0x00000000  // RX FIFO >= 1/8 full
#define UART_IFLS_TX1_8         0x00000000  // TX FIFO <= 1/8 full
#define UART_IM_RTIM            0x00000040  // UART Receive Time-Out Interrupt
                                            // Mask
#define UART_IM_TXIM            0x00000020  // UART Transmit Interrupt Mask
#define UART_IM_RXIM            0x00000010  // UART Receive Interrupt Mask
#define UART_RIS_RTRIS          0x00000040  // UART Receive Time-Out Raw
                                            // Interrupt Status
#define UART_RIS_TXRIS          0x00000020  // UART Transmit Raw Interrupt
                                            // Status
#define UART_RIS_RXRIS          0x00000010  // UART Receive Raw Interrupt
                                            // Status
#define UART_ICR_RTIC           0x00000040  // Receive Time-Out Interrupt Clear
#define UART_ICR_TXIC           0x00000020  // Transmit Interrupt Clear
#define UART_ICR_RXIC           0x00000010  // Receive Interrupt Clear
#define SYSCTL_RCGC1_R          (*((volatile unsigned long *)0x400FE104))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC1_UART0      0x00000001  // UART0 Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
#define FIFOSIZE   16         // size of the FIFOs (must be power of 2)
#define FIFOSUCCESS 1         // return value on success
#define FIFOFAIL    0         // return value on failure
                              // create index implementation FIFO (see FIFO.h)
AddIndexFifo(Rx, FIFOSIZE, char, FIFOSUCCESS, FIFOFAIL)
AddIndexFifo(Tx, FIFOSIZE, char, FIFOSUCCESS, FIFOFAIL)

/////////////////////////////////////////////////////////
//---------------------OUTCRLF_UART0---------------------
// Output a CR,LF to UART0 to go to a new line
// Input: none
// Output: none
void OutCRLF_UART0(void)
{
  UART0_OutChar(CR);
  UART0_OutChar(LF);
}
/////////////////////////////////////////////////////////

// Initialize UART0
// Baud rate is 115200 bits/sec - I changed this to 9600 bits/sec
void UART0_Init(void)
{
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  RxFifo_Init();                        // initialize empty FIFOs
  TxFifo_Init();
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 325;///27;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
  UART0_FBRD_R =  33;///8;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_IFLS_R &= ~0x3F;                // clear TX and RX interrupt FIFO level fields
                                        // configure interrupt for TX FIFO <= 1/8 full
                                        // configure interrupt for RX FIFO >= 1/8 full
  UART0_IFLS_R += (UART_IFLS_TX1_8|UART_IFLS_RX1_8);
                                        // enable TX and RX FIFO interrupts and RX time-out interrupt
  UART0_IM_R |= (UART_IM_RXIM|UART_IM_TXIM|UART_IM_RTIM);
  UART0_CTL_R |= UART_CTL_UARTEN;       // enable UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1-0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1-0
                                        // UART0=priority 2
  NVIC_PRI1_R = (NVIC_PRI1_R&0xFFFF00FF)|0x00004000; // bits 13-15
  NVIC_EN0_R |= NVIC_EN0_INT5;          // enable interrupt 5 in NVIC
}
// copy from hardware RX FIFO to software RX FIFO
// stop when hardware RX FIFO is empty or software RX FIFO is full
void static copyHardwareToSoftware_UART0(void)
{
  char letter;
  while(((UART0_FR_R&UART_FR_RXFE) == 0) && (RxFifo_Size() < (FIFOSIZE - 1)))
	{
    letter = UART0_DR_R;
    RxFifo_Put(letter);
  }
}
// copy from software TX FIFO to hardware TX FIFO
// stop when software TX FIFO is empty or hardware TX FIFO is full
void static copySoftwareToHardware_UART0(void)
{
  char letter;
  while(((UART0_FR_R&UART_FR_TXFF) == 0) && (TxFifo_Size() > 0))
	{
    TxFifo_Get(&letter);
    UART0_DR_R = letter;
  }
}
// input ASCII character from UART
// spin if RxFifo is empty
unsigned char UART0_InChar(void)
{
  char letter;
  while(RxFifo_Get(&letter) == FIFOFAIL){};
  return(letter);
}
// output ASCII character to UART
// spin if TxFifo is full
void UART0_OutChar(unsigned char data)
{
  while(TxFifo_Put(data) == FIFOFAIL){};
  UART0_IM_R &= ~UART_IM_TXIM;          // disable TX FIFO interrupt
  copySoftwareToHardware_UART0();
  UART0_IM_R |= UART_IM_TXIM;           // enable TX FIFO interrupt
}
// at least one of three things has happened:
// hardware TX FIFO goes from 3 to 2 or less items
// hardware RX FIFO goes from 1 to 2 or more items
// UART receiver has timed out
void UART0_Handler(void)
{
  if(UART0_RIS_R&UART_RIS_TXRIS)
	{       // hardware TX FIFO <= 2 items
    UART0_ICR_R = UART_ICR_TXIC;        // acknowledge TX FIFO
    // copy from software TX FIFO to hardware TX FIFO
    copySoftwareToHardware_UART0();
    if(TxFifo_Size() == 0)
		{             // software TX FIFO is empty
      UART0_IM_R &= ~UART_IM_TXIM;      // disable TX FIFO interrupt
    }
  }
  if(UART0_RIS_R&UART_RIS_RXRIS)
	{       // hardware RX FIFO >= 2 items
    UART0_ICR_R = UART_ICR_RXIC;        // acknowledge RX FIFO
    // copy from hardware RX FIFO to software RX FIFO
    copyHardwareToSoftware_UART0();
  }
  if(UART0_RIS_R&UART_RIS_RTRIS)
	{       // receiver timed out
    UART0_ICR_R = UART_ICR_RTIC;        // acknowledge receiver time out
    // copy from hardware RX FIFO to software RX FIFO
    copyHardwareToSoftware_UART0();
  }
}


//------------UART0_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART0_OutString(char *pt)
{
  while(*pt)
	{
    UART0_OutChar(*pt);
    pt++;
  }
}

//------------UART0_InUDec------------
// InUDec accepts ASCII input in unsigned decimal format
//     and converts to a 32-bit unsigned number
//     valid range is 0 to 4294967295 (2^32-1)
// Input: none
// Output: 32-bit unsigned number
// If you enter a number above 4294967295, it will return an incorrect value
// Backspace will remove last digit typed
unsigned long UART0_InUDec(void)
{
	unsigned long number=0, length=0;
	char character;
  character = UART0_InChar();
  while(character != CR)
	{ // accepts until <enter> is typed
// The next line checks that the input is a digit, 0-9.
// If the character is not 0-9, it is ignored and not echoed
    if((character>='0') && (character<='9')) 
		{
      number = 10*number+(character-'0');   // this line overflows if above 4294967295
      length++;
      UART0_OutChar(character);
    }
// If the input is a backspace, then the return number is
// changed and a backspace is outputted to the screen
    else if((character==BS) && length)
		{
      number /= 10;
      length--;
      UART0_OutChar(character);
    }
    character = UART0_InChar();
  }
  return number;
}

//-----------------------UART0_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART0_OutUDec(unsigned long n)
{
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10)
	{
    UART0_OutUDec(n/10);
    n = n%10;
  }
  UART0_OutChar(n+'0'); /* n is between 0 and 9 */
}

//---------------------UART0_InUHex----------------------------------------
// Accepts ASCII input in unsigned hexadecimal (base 16) format
// Input: none
// Output: 32-bit unsigned number
// No '$' or '0x' need be entered, just the 1 to 8 hex digits
// It will convert lower case a-f to uppercase A-F
//     and converts to a 16 bit unsigned number
//     value range is 0 to FFFFFFFF
// If you enter a number above FFFFFFFF, it will return an incorrect value
// Backspace will remove last digit typed
unsigned long UART0_InUHex(void)
{
	unsigned long number=0, digit, length=0;
	char character;
  character = UART0_InChar();
  while(character != CR)
	{
    digit = 0x10; // assume bad
    if((character>='0') && (character<='9'))
		{
      digit = character-'0';
    }
    else if((character>='A') && (character<='F'))
		{
      digit = (character-'A')+0xA;
    }
    else if((character>='a') && (character<='f'))
		{
      digit = (character-'a')+0xA;
    }
// If the character is not 0-9 or A-F, it is ignored and not echoed
    if(digit <= 0xF)
		{
      number = number*0x10+digit;
      length++;
      UART0_OutChar(character);
    }
// Backspace outputted and return value changed if a backspace is inputted
    else if((character==BS) && length)
		{
      number /= 0x10;
      length--;
      UART0_OutChar(character);
    }
    character = UART0_InChar();
  }
  return number;
}

//--------------------------UART0_OutUHex----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1 to 8 digits with no space before or after
void UART0_OutUHex(unsigned long number)
{
// This function uses recursion to convert the number of
//   unspecified length as an ASCII string
  if(number >= 0x10)
	{
    UART0_OutUHex(number/0x10);
    UART0_OutUHex(number%0x10);
  }
  else
	{
    if(number < 0xA)
		{
      UART0_OutChar(number+'0');
    }
    else
		{
      UART0_OutChar((number-0x0A)+'A');
    }
  }
}

//------------UART0_InString------------
// Accepts ASCII characters from the serial port
//    and adds them to a string until <enter> is typed
//    or until max length of the string is reached.
// It echoes each character as it is inputted.
// If a backspace is inputted, the string is modified
//    and the backspace is echoed
// terminates the string with a null character
// uses busy-waiting synchronization on RDRF
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
// -- Modified by Agustinus Darmawan + Mingjie Qiu --
void UART0_InString(char *bufPt, unsigned short max) 
{
	int length=0;
	char character;
  character = UART0_InChar();
  while(character != CR)
	{
    if(character == BS)
		{
      if(length)
			{
        bufPt--;
        length--;
        UART0_OutChar(BS);
      }
    }
    else if(length < max)
		{
      *bufPt = character;
      bufPt++;
      length++;
      UART0_OutChar(character);
    }
    character = UART0_InChar();
  }
  *bufPt = 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Make UART1 for going to the XBEE///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////
AddIndexFifo(XBeeRx, FIFOSIZE, char, FIFOSUCCESS, FIFOFAIL)
AddIndexFifo(XBeeTx, FIFOSIZE, char, FIFOSUCCESS, FIFOFAIL)

//---------------------OUTCRLF_UART1---------------------
// Output a CR,LF to UART1 to go to a new line
// Input: none
// Output: none
void OutCRLF_UART1(void)
{
  UART1_OutChar(CR);
  UART1_OutChar(LF);
}
////////////////
void UART1_Init(void)
{
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1; // activate UART1
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD; // activate port D

  XBeeRxFifo_Init();                    // initialize empty FIFOs
  XBeeTxFifo_Init();
	
  UART1_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
	//what i think it is
	UART1_IBRD_R = 325; // IBRD = int(50,000,000 / (16 * 9600)) = int(325.52)
	UART1_FBRD_R = 33;  // FBRD = int(0.5208 * 64 + 0.5) = 33.83333
	
 // need to see how to change this to be 9600 bits/s, currently 115,200 bits/s
//	UART1_IBRD_R = 27;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
//  UART1_FBRD_R = 8;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART1_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART1_IFLS_R &= ~0x3F;                // clear TX and RX interrupt FIFO level fields
                                        // configure interrupt for TX FIFO <= 1/8 full
                                        // configure interrupt for RX FIFO >= 1/8 full
  UART1_IFLS_R += (UART_IFLS_TX1_8|UART_IFLS_RX1_8);
                                        // enable TX and RX FIFO interrupts and RX time-out interrupt
  UART1_IM_R |= (UART_IM_RXIM|UART_IM_TXIM|UART_IM_RTIM);
  UART1_CTL_R |= UART_CTL_UARTEN;       // enable UART
	
	
	GPIO_PORTD_AFSEL_R |= 0x0C; // enable alt funct on PD3-2         
  GPIO_PORTD_DEN_R |= 0x0C;             // enable digital I/O on PD3-2
                                        // UART1=priority 2
											
	// this should set UART1 to interrupt	in the NVIC																		
  NVIC_PRI1_R = (NVIC_PRI1_R&0xFF1FFFFF)|0x00004000; // bits 21-23
  NVIC_EN0_R |= NVIC_EN0_INT6;          // enable interrupt 5 in NVIC
}


// copy from hardware RX FIFO to software RX FIFO
// stop when hardware RX FIFO is empty or software RX FIFO is full
void static copyHardwareToSoftware_UART1(void)
{
  char letter;
  while(((UART1_FR_R&UART_FR_RXFE) == 0) && (XBeeRxFifo_Size() < (FIFOSIZE - 1)))
	{
    letter = UART1_DR_R;
    XBeeRxFifo_Put(letter);
  }
}
// copy from software TX FIFO to hardware TX FIFO
// stop when software TX FIFO is empty or hardware TX FIFO is full
void static copySoftwareToHardware_UART1(void)
{
  char letter;
  while(((UART1_FR_R&UART_FR_TXFF) == 0) && (XBeeTxFifo_Size() > 0))
	{
    XBeeTxFifo_Get(&letter);
    UART1_DR_R = letter;
  }
}
// input ASCII character from UART1
// spin if RxFifo is empty
unsigned char UART1_InChar(void)
{
  char letter;
  while(XBeeRxFifo_Get(&letter) == FIFOFAIL){};
  return(letter);
}
// output ASCII character to UART
// spin if TxFifo is full
void UART1_OutChar(unsigned char data)
{
  while(XBeeTxFifo_Put(data) == FIFOFAIL){};
  UART1_IM_R &= ~UART_IM_TXIM;          // disable TX FIFO interrupt
  copySoftwareToHardware_UART1();
  UART1_IM_R |= UART_IM_TXIM;           // enable TX FIFO interrupt
}
// at least one of three things has happened:
// hardware TX FIFO goes from 3 to 2 or less items
// hardware RX FIFO goes from 1 to 2 or more items
// UART receiver has timed out
void UART1_Handler(void)
{
  if(UART1_RIS_R&UART_RIS_TXRIS)
	{       // hardware TX FIFO <= 2 items
    UART1_ICR_R = UART_ICR_TXIC;        // acknowledge TX FIFO
    // copy from software TX FIFO to hardware TX FIFO
    copySoftwareToHardware_UART1();
    if(XBeeTxFifo_Size() == 0)
		{             // software TX FIFO is empty
      UART1_IM_R &= ~UART_IM_TXIM;      // disable TX FIFO interrupt
    }
  }
  if(UART1_RIS_R&UART_RIS_RXRIS)
	{       // hardware RX FIFO >= 2 items
    UART1_ICR_R = UART_ICR_RXIC;        // acknowledge RX FIFO
    // copy from hardware RX FIFO to software RX FIFO
    copyHardwareToSoftware_UART1();
  }
  if(UART1_RIS_R&UART_RIS_RTRIS)
	{       // receiver timed out
    UART1_ICR_R = UART_ICR_RTIC;        // acknowledge receiver time out
    // copy from hardware RX FIFO to software RX FIFO
    copyHardwareToSoftware_UART1();
  }
}


//------------UART1_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART1_OutString(char *pt)
{
  while(*pt)
	{
    UART1_OutChar(*pt);
    pt++;
  }
}

//------------UART1_InUDec------------
// InUDec accepts ASCII input in unsigned decimal format
//     and converts to a 32-bit unsigned number
//     valid range is 0 to 4294967295 (2^32-1)
// Input: none
// Output: 32-bit unsigned number
// If you enter a number above 4294967295, it will return an incorrect value
// Backspace will remove last digit typed
unsigned long UART1_InUDec(void)
{
	unsigned long number=0, length=0;
	char character;
  character = UART1_InChar();
  while(character != CR)
	{ // accepts until <enter> is typed
// The next line checks that the input is a digit, 0-9.
// If the character is not 0-9, it is ignored and not echoed
    if((character>='0') && (character<='9')) 
		{
      number = 10*number+(character-'0');   // this line overflows if above 4294967295
      length++;
      UART1_OutChar(character);
    }
// If the input is a backspace, then the return number is
// changed and a backspace is outputted to the screen
    else if((character==BS) && length)
		{
      number /= 10;
      length--;
      UART1_OutChar(character);
    }
    character = UART1_InChar();
  }
  return number;
}

//-----------------------UART1_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART1_OutUDec(unsigned long n)
{
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10)
	{
    UART1_OutUDec(n/10);
    n = n%10;
  }
  UART1_OutChar(n+'0'); /* n is between 0 and 9 */
}

//---------------------UART1_InUHex----------------------------------------
// Accepts ASCII input in unsigned hexadecimal (base 16) format
// Input: none
// Output: 32-bit unsigned number
// No '$' or '0x' need be entered, just the 1 to 8 hex digits
// It will convert lower case a-f to uppercase A-F
//     and converts to a 16 bit unsigned number
//     value range is 0 to FFFFFFFF
// If you enter a number above FFFFFFFF, it will return an incorrect value
// Backspace will remove last digit typed
unsigned long UART1_InUHex(void)
{
	unsigned long number=0, digit, length=0;
	char character;
  character = UART1_InChar();
  while(character != CR)
	{
    digit = 0x10; // assume bad
    if((character>='0') && (character<='9'))
		{
      digit = character-'0';
    }
    else if((character>='A') && (character<='F'))
		{
      digit = (character-'A')+0xA;
    }
    else if((character>='a') && (character<='f'))
		{
      digit = (character-'a')+0xA;
    }
// If the character is not 0-9 or A-F, it is ignored and not echoed
    if(digit <= 0xF)
		{
      number = number*0x10+digit;
      length++;
      UART1_OutChar(character);
    }
// Backspace outputted and return value changed if a backspace is inputted
    else if((character==BS) && length)
		{
      number /= 0x10;
      length--;
      UART1_OutChar(character);
    }
    character = UART1_InChar();
  }
  return number;
}

//--------------------------UART1_OutUHex----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1 to 8 digits with no space before or after
void UART1_OutUHex(unsigned long number)
{
// This function uses recursion to convert the number of
//   unspecified length as an ASCII string
  if(number >= 0x10)
	{
    UART1_OutUHex(number/0x10);
    UART1_OutUHex(number%0x10);
  }
  else
	{
    if(number < 0xA)
		{
      UART1_OutChar(number+'0');
    }
    else
		{
      UART1_OutChar((number-0x0A)+'A');
    }
  }
}

//------------UART1_InString------------
// Accepts ASCII characters from the serial port
//    and adds them to a string until <enter> is typed
//    or until max length of the string is reached.
// It echoes each character as it is inputted.
// If a backspace is inputted, the string is modified
//    and the backspace is echoed
// terminates the string with a null character
// uses busy-waiting synchronization on RDRF
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
// -- Modified by Agustinus Darmawan + Mingjie Qiu --
void UART1_InString(char *bufPt, unsigned short max) 
{
	int length=0;
	char character;
  character = UART1_InChar();
  while(character != CR)
	{
    if(character == BS)
		{
      if(length)
			{
        bufPt--;
        length--;
        UART1_OutChar(BS);
      }
    }
    else if(length < max)
		{
      *bufPt = character;
      bufPt++;
      length++;
      UART1_OutChar(character);
    }
    character = UART1_InChar();
  }
  *bufPt = 0;
}

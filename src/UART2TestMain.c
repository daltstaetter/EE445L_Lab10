// UART2TestMain.c
// Runs on LM3S1968
// Tests the UART0 to implement bidirectional data transfer to and from a
// computer running HyperTerminal.  This time, interrupts and FIFOs
// are used.
// Daniel Valvano
// October 9, 2011

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to the Arm Cortex M3",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2011

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

#include "UART2.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "systick.h"
#include "Xbee.h"

//debug code
#define UART0

int main(void)
{

  unsigned char i;
  char string[20];  // global to assist in debugging
  unsigned long n;
	
  // Set the clocking to run at 50MHz from the PLL.
  SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_8MHZ);
	SysTick_Init();
	
#ifdef UART0
  UART0_Init();              // initialize UART0
	//UART1_Init();              // initialize UART1
	EnableInterrupts();
  OutCRLF_UART0();
	//OutCRLF_UART1();
	
  for(i='A'; i<='Z'; i=i+1)
	{// print the uppercase alphabet
    UART0_OutChar(i);
  }
  OutCRLF_UART0();
  UART0_OutChar(' ');
  for(i='a'; i<='z'; i=i+1)
	{// print the lowercase alphabet
    UART0_OutChar(i);
  }
  OutCRLF_UART0();
  UART0_OutChar('-');
  UART0_OutChar('-');
  UART0_OutChar('>');
  while(1)
	{
    UART0_OutString("InString0: ");
    UART0_InString(string,19);
    UART0_OutString(" OutString0="); UART0_OutString(string); OutCRLF_UART0();

    UART0_OutString("InUDec0: ");  n=UART0_InUDec();
    UART0_OutString(" OutUDec0="); UART0_OutUDec(n); OutCRLF_UART0();

    UART0_OutString("InUHex0: ");  n=UART0_InUHex();
    UART0_OutString(" OutUHex0="); UART0_OutUHex(n); OutCRLF_UART0();
  }
#else
	//UART1_Init();              // initialize UART1
	EnableInterrupts();
	OutCRLF_UART1();
	
  for(i='A'; i<='Z'; i=i+1)
	{// print the uppercase alphabet
    UART1_OutChar(i);
  }
  OutCRLF_UART1();
  UART1_OutChar(' ');
  for(i='a'; i<='z'; i=i+1)
	{// print the lowercase alphabet
    UART1_OutChar(i);
  }
  OutCRLF_UART1();
  UART1_OutChar('-');
  UART1_OutChar('-');
  UART1_OutChar('>');
  while(1)
	{
//    UART1_OutString("InString: ");
//    UART1_InString(string,19);
//    UART1_OutString(" OutString="); UART1_OutString(string); OutCRLF_UART1();

//    UART1_OutString("InUDec: ");  n=UART1_InUDec();
//    UART1_OutString(" OutUDec="); UART1_OutUDec(n); OutCRLF_UART1();

//    UART1_OutString("InUHex: ");  n=UART1_InUHex();
//    UART1_OutString(" OutUHex="); UART1_OutUHex(n); OutCRLF_UART1();
		XBee_SendTxFrame();		
  }
#endif
}

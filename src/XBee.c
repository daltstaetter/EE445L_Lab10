// XBee.c
#include "Xbee.h"
#include "systick.h"
#include "UART2.h"

#define NULL 0
static void sendATCommand(char* input);

unsigned char destination[2] = {0x00,0x4F};
unsigned char startDelimiter = 0x7E;
unsigned char opt = 0x00;

void XBee_Init(void)
{
	UART1_OutChar('X');     // send to XBee
	UART0_OutChar('X');     // echo to user
	SysTick_Wait10ms(110);  // guard time delay
	UART1_OutString("+++"); // send to XBee for AT cmd mode
	UART0_OutString("+++"); // echo to user
	SysTick_Wait10ms(110);  // guard time delay
	
	// not sure how to retrieve CR response
	UART0_OutChar(UART1_InChar()); // think this get the OK<CR> response
	sendATCommand("ATDL4F");  // sets destination address to 79 	
	sendATCommand("ATDH0"); OutCRLF_UART0();  // sets destination high address to 0
//	UART0_OutChar(UART1_InChar()); // think this get the OK<CR> response
	sendATCommand("ATMY4E"); OutCRLF_UART0(); // sets my address to 78
//	UART0_OutChar(UART1_InChar()); // think this get the OK<CR> response
	sendATCommand("ATAP1"); OutCRLF_UART0();  // set for API mode 1
//	UART0_OutChar(UART1_InChar()); // think this get the OK<CR> response
	sendATCommand("ATCN"); OutCRLF_UART0();   // ends the AT Command mode
//	UART0_OutChar(UART1_InChar()); // think this get the OK<CR> response
	// also check ATBD == 3 to make sure the baud rate is set at 9600 bits/sec
}


//sendATCommand – sends an AT command repeatedly until it receives a reply that it was correctly received
//This routine receives the various parameters associated with an AT command as input then transmits the formatted
//command to the XBee module. After a blind-cycle delay, the routine checks if the command has been successfully
//received by determining if the module has returned the ‘OK’ character string.
static void sendATCommand(char* input)
{
	UART1_OutString(input);	
	OutCRLF_UART0();
	UART0_OutChar(UART1_InChar()); // think this get the OK<CR> response
}



//XBee_TxStatus – determine transmit status
//When the XBee module transmits an API transmit data frame it will receive an acknowledgement from the
//destination module if the frame was received without errors. The status of the transmission will be sent to the
//LM3S1968 via an API transmit status frame. This routine returns a ‘1’ if the transmission was successful and a ‘0’
//otherwise. The following figure shows a response the XBee returns after the transmitter sends a TxFrame that was
//properly received by the other computer, measured on XBee pin 2 Dout.
int XBee_TxStatus(void)
{



	return 0; // if error
	//return 1; // if successful
}
//-------------------------------------------------------------------------------------------------
void XBee_SendTxFrame(void)
{
	char* XbeeString;
	char string[25] = {0};
	
	
	UART0_OutString("InString0: ");
  UART0_InString(&string[0],19);
	XbeeString = XBee_CreateTxFrame(&string[0]);
	UART1_OutString(XbeeString); OutCRLF_UART1();
	
	if(!XBee_TxStatus)
	{
		UART0_OutString("Error, acknolwdge not received"); OutCRLF_UART0();
	}
	
	
}
//-------------------------------------------------------------------------------------------------
char* XBee_CreateTxFrame(char* string)
{
	static unsigned char ID = 1;
	int i;
	unsigned short length;
	static unsigned char message[25];
	unsigned short numBytes;
	unsigned char checksum;
	unsigned char sum;
	
	// zero message buffer
	for(i=0;i<25;i++)
	{
		message[i] = 0;
	}
	
	numBytes = 0;
	// should I look for <CR> instead of the null?
	// I am not sure how it stores the values from the UART
	while(*string != CR)
	{
		numBytes++;
	}
	length = numBytes+5; // 5 counts for the API, ID, Destination, & OPT bytes
	
	//for(numBytes = 0; *string != NULL; numBytes++){}	
	
	
	message[0] = startDelimiter;
	message[1] = (unsigned char)((length & 0xFF00)>>8); // get top byte of length
	message[2] = (unsigned char)(length & 0x00FF); // get lower byte of length
	message[3] = 0x01; // API mode 1
	message[4] = ID;
	message[5] = destination[0];
	message[6] = destination[1];
	message[7] = opt;
	
	for(i=0; i<numBytes; i++)
	{
		// fill the message array
		message[8+i] = string[i];		
	}
	
	ID = (ID+1)%256; // keep in range of an unsigned char
	if(ID == 0)
	{
		ID = 1; // make sure the ID never equals zero
	}
	
	sum = message[3]+message[4]+message[5]+message[6]+message[7];
	
	for(i=0; i < numBytes; i++)
	{
		sum += message[i+8];
	}
	checksum = 0xFF-sum;
	message[numBytes+9] = checksum;
	return &message[0];
}
//-------------------------------------------------------------------------------------------------

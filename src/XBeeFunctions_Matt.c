//Matt's XBee Functions
char reponse[3];


int XBee_CheckOK() {
	char check[2];

	//not sure if checking for <CR>, CR, or \r
	while(response[0] != 'O' && response[1] != 'K' && response[2] != '\r') {
		UART1_InString(response);
		//what to do if failure?
	}
}

//intialize XBee
void XBeeInit(void) {
	//enter command mode
	UART1_OutChar('X');
	Delay(18333334); //~1.1s at 50MHz
	UART1_OutString("+++");
	Delay(18333334); //~1.1s at 50MHz
	XBee_CheckOK(); //check for OK response

	//not sure if I need to tack <CR> onto the end of these
	sendATCommand("ATDL 4F"); //Sets destination address to 79
	Delay(333333); //~20ms at 50MHZ
	XBee_CheckOK(); //check for OK response

	sendATCommand("ATDH 0<CR>"); //sets destination high address to 0
	Delay(333333); //~20ms at 50MHZ
	XBee_CheckOK(); //check for OK response

	sendATCommand("ATMY 4E<CR>"); //sets my address to 78
	Delay(333333); //~20ms at 50MHZ
	XBee_CheckOK(); //check for OK response

	sendATCommand("ATAP 1<CR>"); //API mode 1 (sends/recieve packets)
	Delay(333333); //~20ms at 50MHZ
	XBee_CheckOK(); //check for OK response

	//ATCH parameter<CR> changes the channel range between 0x0B and 0x1A (deafult 0x0C)
	//ATID parameter<CR> changes the personal area network ID range between 0x0000 and 0xFFFF (default 0x3332) 

	sendATCommand("ATCN<CR>"); //ends command mode
	Delay(333333); //~20ms at 50MHZ
	XBee_CheckOK(); //check for OK response
}

//returns 1 if transmission was succesful, 0 otherwise
int XBee_TxStatus(void) {
	//length + 4
	char buffer[50];
	int x, y, length, returnedChecksum;

	UART1_InString(buffer);

	//calculate length of message
	x = buffer[1];
	y = buffer[2];
	length = (x << 8) + y; 

	//find returned checksum value
	returnedChecksum = buffer[length + 3];

	//check if the check sums match
	if(returnedChecksum == checksum) {
		return 1;
	}
	return 0;
}


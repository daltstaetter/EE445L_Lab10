// XBee.h

//matt
void XBee_Init(void);
int XBee_TxStatus(void);

//mine
void XBee_SendTxFrame(void);
char* XBee_CreateTxFrame(char* string);


#define INFINEER_COMMAND 1421
#include <stdlib.h>
#include <stdint.h>

/*****************************************************************
/
/ Copyright (C) 2001  Infineer Inc
/ File   :   ifdhandler.c
/ Author :   Hariharan Swaminathan  hswami@infineer.com
/ Base file is taken from pcsclite sample code written by	
/					David Corcoran 	<corcoran@linuxnet.com>
/ 
/This program is free software; you can redistribute it and/or
/modify it under the terms of the GNU General Public License
/as published by the Free Software Foundation; either version 2
/of the License, or any later version.
/ 
/This program is distributed in the hope that it will be useful,
/but WITHOUT ANY WARRANTY; without even the implied warranty of
/MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/GNU General Public License for more details.
/
******************************************************************/
/* This is a pcsc-lite library for Infineer smartcard reader */

#define DEBUG
#ifdef DEBUG
#define PRINTF if ( SmartCard[Lun].temp == 1 ) printf
#else
#define PRINTF //
#endif
#include <pcscdefines.h>
#include <ifdhandler.h>
#include <infineer.h>
#include <sys/fcntl.h>
#include <sys/types.h>

#define FUNCTION_LOG(format, ...) printf("%s:"format,  __FUNCTION__, ## __VA_ARGS__)

enum {
	MAX_PACKET_DATA = 506,
	READ_BUFFER_SIZE = MAX_PACKET_DATA + 5,
};

enum {
	BAUDRATE_115200         = 0x06,
};

enum Commands {
	CARD_COMMAND            = 0x00,
	CARD_PRESENCE           = 0x09,
	SEND_NUM_MASK           = 0x0a,
	SET_SERIAL_BAUD_RATE    = 0x0d,
	POWER_OFF               = 0x4d,
	POWER_UP_ISO            = 0x69,
};

enum {
	ALPAR_ACK               = 0x60,
	ALPAR_NAK               = 0xe0,
};

enum {
	ERR_PROTOCOL            = (1<<24),
};

static uint8_t alparLrc(uint32_t length, const uint8_t *buffer) {
	uint8_t lrc = buffer[0];

	unsigned i;
	for(i = 1; i < length; i++)
		lrc = lrc ^ buffer[i];

	return lrc;
}

static uint8_t *alparBufferPayload(uint8_t *data)
{
	return (data + 4);
}

static void alparBufferPrepare(uint8_t command, uint8_t *buffer, uint16_t length)
{
	buffer[0] = ALPAR_ACK;
	buffer[1] = (length & 0xFF00) >> 8 ;
	buffer[2] = length & 0x00FF;
	buffer[3] = command;

	buffer[length + 4] = alparLrc(buffer, length + 4);
}

static const char* alparErrorStr(uint8_t status)
{
	int _errno = status;

	switch(status)
	{
		case 0x08: return "Length of the data buffer too short";
		case 0x0a: return "3 consecutive errors from the card in T=1 protocol";

		case 0x20: return "Wrong APDU";
		case 0x21: return "Too short APDU";
		case 0x22: return "Card mute now (during T=1 exchange)";
		case 0x24: return "Bad NAD";
		case 0x25: return "Bad LRC";
		case 0x26: return "Resynchronized";
		case 0x27: return "Chain aborted";
		case 0x28: return "Bad PCB";
		case 0x29: return "Overflow from card";

		case 0x30: return "Non negotiable mode (TA2 present)";
		case 0x31: return "Protocol is neither T=0 nor T=1 (negotiate command)";
		case 0x32: return "T=1 is not accepted (negotiate command)";
		case 0x33: return "PPS answer is different from PPS request";
		case 0x34: return "Error on PCK (negotiate command)";
		case 0x35: return "Bad parameter in command";
		case 0x38: return "TB3 absent";
		case 0x39: return "PPS not accepted (no answer from card)";
		case 0x3b: return "Early answer of the card during the activation";

		case 0x55: return "Unknown command";

		case 0x80: return "Card mute (after power on)";
		case 0x81: return "Time out (waiting time exceeded)";
		case 0x83: return "5 parity errors in reception";
		case 0x84: return "5 parity errors in transmission";
		case 0x86: return "Bad FiDi";
		case 0x88: return "ATR duration greater than 19200 etus (E.M.V.)";
		case 0x89: return "CWI not supported (E.M.V.)";
		case 0x8a: return "BWI not supported (E.M.V.)";
		case 0x8b: return "WI (Work waiting time) not supported (E.M.V.)";
		case 0x8c: return "TC3 not accepted (E.M.V.)";
		case 0x8d: return "Parity error during ATR";

		case 0x90: return "3 consecutive parity errors in T=1 protocol";
		case 0x91: return "SW1 different from 6X or 9X";
		case 0x92: return "Specific mode byte TA2 with b5 byte=1";
		case 0x93: return "TB1 absent during a cold reset (E.M.V.)";
		case 0x94: return "TB1 different from 00 during a cold reset (E.M.V.)";
		case 0x95: return "IFSC<10H or IFSC=FFH";
		case 0x96: return "Wrong TDi";
		case 0x97: return "TB2 is present in the ATR (E.M.V.)";
		case 0x98: return "TC1 is not compatible with CWT";
		case 0x9b: return "Not T=1 card";

		case 0xa0: return "Procedure byte error";
		case 0xa1: return "Card deactivated due to a hardware problem";

		case 0xc0: return "Card absent";
		case 0xc3: return "Checksum error";
		case 0xc4: return "TS is neither 3B nor 3F";
		case 0xc6: return "ATR not supported";
		case 0xc7: return "VPP is not supported";

		case 0xe1: return "Card clock frequency not accepted (after a set_clock_card command)";
		case 0xe2: return "UART overflow";
		case 0xe3: return "Supply voltage drop-off";
		case 0xe4: return "Temperature alarm";
		case 0xe9: return "Framing error";

		case 0xf0: return "Serial LRC error";
		case 0xff: return "Serial time out";

		default:   return "Unknown error";
	}
}

static int alparBufferParse(const uint8_t *buffer)
{
	if((buffer[0] != ALPAR_ACK) && (buffer[0] != ALPAR_NAK) )
	{
		FUNCTION_LOG("Unknown packet signature %x\r\n", buffer[0]);
		return 0;
	}

	int command = buffer[4];
	int length = ( buffer[1] << 8 ) | (buffer[2] );

	if (length > MAX_PACKET_DATA)
	{
		FUNCTION_LOG("Data is too long %x\r\n", length);
		return 0;
	}

	uint8_t lrc = alparLrc(buffer, length + 5);


	if (alparLrc)
	{
		FUNCTION_LOG("Bad LRC %x, should be zero\r\n", alparLrc);
		return 0;
	}

	if (buffer[0] != ALPAR_ACK)
	{
		FUNCTION_LOG("error %s", alparErrorStr(buffer[5]));
		return 0;
	}

	return 1;
}

static int tda8029Receive(uint8_t *buffer, uint16_t *length)
{
	tda8029UartRead(buffer, 4);

	*length = ( buffer[1] << 8 ) | ( buffer[2] );

	tda8029UartRead(buffer + 4, length + 1);

	// check error
	int res = alparBufferParse(buffer);

	return res;
}

static void tda8029Transmit(uint8_t command, uint8_t *buffer, uint16_t length)
{
	flush_uart();
	alparBufferPrepare(command, buffer, length);
	tda8029UartWrite(buffer, length);
}

static int tda8029CheckPresenceCard()
{
	tda8029Transmit(CARD_PRESENCE, NULL, 0);
	uint16_t length;
	uint8_t buffer[READ_BUFFER_SIZE];
	int res = tda8029Receive(buffer, &length);

	uint8_t data = alparBufferPayload(buffer)[0];

	res = res && (length == 1) && (data == 1);

	return res;
}

static int tda8029PowerUpISO()
{
	tda8029Transmit(POWER_UP_ISO, NULL, 0);
	uint16_t length;
	uint8_t buffer[READ_BUFFER_SIZE];
	int res = tda8029Receive(buffer, &length);

	uint8_t data = alparBufferPayload(buffer)[0];
	if(data == 0xc0) {
		FUNCTION_LOG("Card is absent\r\n");
		return 0;
	}

	FUNCTION_LOG("Status %d, %x\r\n", length, data);

	return 1;
}

static int tda8029PowerOff()
{
	tda8029Transmit(POWER_OFF, NULL, 0);
	uint16_t length;
	uint8_t buffer[READ_BUFFER_SIZE];
	int res = tda8029Receive(buffer, &length);

	return res;
}


static int tda8029CardCommand(const void *cmd, unsigned cmd_len, void *resp, unsigned *resp_len)
{
	tda8029Transmit(CARD_COMMAND, cmd, cmd_len);
	int res = tda8029Receive(resp, resp_len);

	return res;
}

static int tda8029SendNumMask(char *firmware_str)
{
	tda8029Transmit(SEND_NUM_MASK, NULL, 0);
	uint16_t length;
	uint8_t buffer[READ_BUFFER_SIZE];
	int res = tda8029Receive(buffer, &length);
	if (res)
	{
		uint8_t *data = alparBufferPayload(buffer);
		memcpy(firmware_str, data, length);
		firmware_str[length]=0;
	}

	return res;
}


DEVICE_CAPABILITIES device_data = {  "Infineer Inc.","Infineer",1,"DT3000/DT3500/LT4000",0,SCARD_PROTOCOL_T0|SCARD_PROTOCOL_T1,4000,4000,10752,10752 ,
		MAX_IFSD,0,0,0,0,0};
struct smartport_t SmartCard[16];

PROTOCOL_OPTIONS protocol;

static int ConstructResponseAPDU(int Lun,PUCHAR SPRespPacket,PUCHAR RespAPDU,PDWORD length);
RESPONSECODE IFDHTransmitToICC_T0 ( int Lun,PUCHAR TxBuffer, DWORD TxLength,
                 PUCHAR RxBuffer, PDWORD RxLength);
RESPONSECODE IFDHTransmitToICC_T1 ( int Lun,PUCHAR TxBuffer, DWORD TxLength,
                 PUCHAR RxBuffer, PDWORD RxLength);
static UCHAR calcChksum(PUCHAR data,DWORD length );
// UCHAR Cmd_MemPowerUp[8]= 0xC1,5,0x15,0x00,0x14,0x00,0x01,0x00 };
static int validStatus( UCHAR status );

ICC_STATE ICC;
#define check_status(sts_byte) if( ! validStatus(sts_byte) ) { \
				PRINTF(" error in status \n"); 	}
				
#ifdef _SERIAL_
static int IOCTL(int fd,int command,unsigned char buffer[] ) {
	int len;
	int i;
	unsigned char status;
	unsigned char l;
	unsigned char chksum;
	unsigned char rbuffer[300];
	int retval;
	scdrain(fd);
	len = buffer[1];
	if( buffer[0] & CONTROL_LENGTHBIT ) len+=256;
	for( i=0;i<len+3;i++) {
		//printf("data = %0x \n",buffer[i]);
		while(write(fd,&buffer[i],1)==0);
	}
	while( (retval = scgetc(fd,&status,10)) ==3 || status == 0xAF );
	len=0;
	while((retval = scgetc(fd,&len,10))==3);
	if( status &  STATUS_LENGTHBIT ) len+=256;
	while((retval = scgetc(fd,&chksum,10))==3);
	buffer[0]=status;
	buffer[1]=len;
	buffer[2]=chksum;

	for(i=3;i<len+3;i++) {
		while((retval = scgetc(fd,&buffer[i],10))==3);
		//printf("resp = %0x \n", buffer[i]);
	}
	//printf("..................\n");
//	PRINTF("In ioctl ... before return \n");
	return 0;
}
#else 
#ifdef _USB_
static int IOCTL(int fd,int command,unsigned char buffer[] ) {
	int len;
	int i;
	unsigned char status;
    unsigned char l;
    unsigned char chksum;
    unsigned char rbuffer[300];
    int retval;
	len = buffer[1];
    if( buffer[0] & CONTROL_LENGTHBIT ) len+=256;
  	if(write(fd,buffer,len+3) != len+3 ) {
		return -1;
	}
	read(fd,&status,1);	
	read(fd,&l,1);
	len=l;
	if( status &  STATUS_LENGTHBIT ) len+=256;
	read(fd,&chksum,1);	
	buffer[0]=status;
    buffer[1]=len;
    buffer[2]=chksum;
	for(i=3;i<len+3;i++) {
		read(fd,&buffer[i],1);
//		printf("resp = %0x \n", buffer[i]);
	}
	return 0;
}
#endif

#ifdef _PCMCIA_
#define IOCTL ioctl
#endif
#endif

/**
 * WHat this function does? Something chip related?
 */
static void CardTracking(int Lun) {
#if 0
    unsigned char buff[300];
    unsigned char Resp[300];
    int rlen;
    int i;
    buff[0]=0xC0;
    buff[1]=0x05;
    buff[2] = 0x18^0x0A;
    buff[3]=0x00;
    buff[4]=0x18;
    buff[5]=0x00;
    buff[6]=0x0A;
    buff[7]=0x00;
	if( IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,buff) < 0) 
		PRINTF("error in ioctl \n");
    if( ConstructResponseAPDU(Lun,buff,Resp,&rlen) < 0) {
        }
    for(i=0;i<rlen;i++) {
        PRINTF("%0x ",Resp[i]);
    }
#endif
}			

static int CalculatePCB(int Lun,PUCHAR RxBuffer) {
	UCHAR rpcb  = RxBuffer[1];
	PRINTF("In calc pcb \n");
	if( rpcb >= 0xC0 ) {
		PRINTF("S Block \n");
		if ( rpcb == 0xE0 ) {
			PRINTF("Resynch response \n");
			return 1;
		}
		else if( rpcb == 0xC1 ) {
			PRINTF("IFS Request \n");
			SmartCard[Lun].oldpcb=SmartCard[Lun].pcb;
			SmartCard[Lun].pcb = rpcb ;
			SmartCard[Lun].pcb |= SBLOCK_RESP_BIT;
			 //oldifs=ifs;
			 SmartCard[Lun].ifs=RxBuffer[3];
			
			PRINTF("IFS = %0x\n",SmartCard[Lun].ifs);
			return 1;
		}
		else if( rpcb == 0xE1 ) {
			PRINTF("IFS Response \n");
			 //oldifs=ifs;
			SmartCard[Lun].ifs=RxBuffer[3];
			return 0;
		}
		else if( rpcb == 0xC2 ) {
			PRINTF("Abort Request \n");
			SmartCard[Lun].oldpcb=SmartCard[Lun].pcb;
			SmartCard[Lun].pcb = rpcb |SBLOCK_RESP_BIT;
			SmartCard[Lun].abrt=RxBuffer[3];
			return 1;
		}
		else if(rpcb == 0xE2 ) {
			PRINTF("Abort response \n");
			SmartCard[Lun].abrt=RxBuffer[3];
			return 0;
		}
		else if( rpcb == 0xC3 ) {
			PRINTF("WTX Request \n");
			SmartCard[Lun].oldpcb=SmartCard[Lun].pcb;
			SmartCard[Lun].pcb = rpcb | SBLOCK_RESP_BIT;
			SmartCard[Lun].wtx=RxBuffer[3];
			return 1;
		}
		else if( rpcb == 0xE3 ) {
			PRINTF("WTX Response \n");
			SmartCard[Lun].wtx=RxBuffer[3];
			return 0;
		}
			
	}
	else if( rpcb >= 0x80) {
		PRINTF("R Block \n");
		if( ( rpcb == 0x80 ) || (rpcb == 0x90 ) )
		{
				PRINTF("No error \n");
	/*			oldpcb=pcb;
				if( rpcb == 0x80)
					pcb=0x20;
				else 
					pcb=0x60; */
				return 0;
		}
		else 
			if( rpcb == 0x91 || rpcb == 0x81 ) {
				PRINTF("Checksum error \n");
				return -1;
			}
		else {
				PRINTF("some other error \n");
				SmartCard[Lun].oldpcb=SmartCard[Lun].pcb;
				PRINTF("pcb = %0x rpcb = %0x \n",SmartCard[Lun].pcb,rpcb);
				SmartCard[Lun].pcb=0xC0;
				PRINTF("pcb = %0x rpcb = %0x rchain = %d \n",SmartCard[Lun].pcb,rpcb,SmartCard[Lun].rchain);
				return -1;
			}
	}
	else {
		PRINTF("I Block \n");
		if( rpcb == 0x20  )   {
			//pcb=0x40;
			SmartCard[Lun].rchain=1;
			if(SmartCard[Lun].pcb==0x90) {
				SmartCard[Lun].pcb=0xC0;
			}
			else { 
					SmartCard[Lun].pcb=0x90;
			}
			PRINTF("rchain = %d\n",SmartCard[Lun].rchain);
			return 1;
		}
		else if( rpcb == 0x60) {
							//pcb=0x00;
							if(SmartCard[Lun].pcb==0x80) {
							SmartCard[Lun].pcb=0xC0;
							}
							else { 
								SmartCard[Lun].pcb=0x80;
							}
							SmartCard[Lun].rchain=1;
							PRINTF("rchain = %d\n",SmartCard[Lun].rchain);
							return 1;
					}
		SmartCard[Lun].rchain=0;
		PRINTF("before return \n");
		return 0;
	}

	
	return 1;
}
static void ConstructSPCmdPacket(int Lun,PUCHAR cmd , PUCHAR SPCmdPacket, DWORD len ,UCHAR cntrl )
{
int i;
unsigned char control = cntrl;
	SPCmdPacket[2]=calcChksum(cmd,len);
	if( len > 255) control |= CONTROL_LENGTHBIT;
SPCmdPacket[0]=control;
SPCmdPacket[1]=(UCHAR) len;
for(i=0;i<len;i++)
    {
        SPCmdPacket[i+3]=cmd[i];
    }      
}


static int ConstructResponseAPDU(int Lun,PUCHAR SPRespPacket,PUCHAR RespAPDU,PDWORD length) {
	int i;
	UCHAR chksum;
	SmartCard[Lun].sts_byte = SPRespPacket[0];
	//printf("status = %0x \n", SmartCard[Lun].sts_byte );
	check_status(SmartCard[Lun].sts_byte);
	*length=SPRespPacket[1];
	if(SmartCard[Lun].sts_byte & 0x80) *length += 256;
	//printf("length = %d \n", *length);
	chksum = SPRespPacket[2];	
	//printf("chksum = %d \n", chksum);
	for(i=0;i<*length;i++) {
                RespAPDU[i]=SPRespPacket[i+3];
                chksum^=SPRespPacket[i+3];
				//printf("data = %0x \n", RespAPDU[i]);
                }
   if( chksum != 0 ) {
			printf("Chksum error \n");
	}
	return 0;
}
#ifdef _SERIAL_
static int SetBaudRate(int Lun,int b ,PUCHAR RxBuffer,PDWORD RxLength ) {
	PUCHAR CmdAPDU,SPCmdPacket;
	UCHAR control;
	int length;
	int baud;
	int brate=b;
	int i;
	switch(b) {
		case 1: 
				baud=2400;
				break;
		case 2: 
				baud=4800;
				break;	
		case 3:
				baud=9600;
				break;	
		case 4: 
				baud=9600;
				brate=3;
				break;	
		case 5:
				baud=19200;
				break;	
		case 6:
				baud=19200;
				brate=5;
				break;
		case 7: 
				baud=19200;
				brate=5;
				break;
		default: 
				printf("Baud rate not supported \n");	
				return -1;
		}
	CmdAPDU = ( PUCHAR ) alloca(INFINEER_MAX_BUFFER);
	SPCmdPacket = ( PUCHAR ) alloca(INFINEER_MAX_BUFFER);
	//printf( "baud = %d , brate = %d \n",baud,brate);
	control= 0x80;
	CmdAPDU[0]=0x00;
	CmdAPDU[1]=0x04;
	CmdAPDU[2]=brate;
	CmdAPDU[3]=0x00;
	CmdAPDU[4]=0x00;
	length=5;
	ConstructSPCmdPacket(Lun,CmdAPDU,SPCmdPacket,length,control);
    if (IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,SPCmdPacket) < 0 )  {
        perror("SetBaudRate");
        return IFD_COMMUNICATION_ERROR ;
    }
    if( ConstructResponseAPDU(Lun,SPCmdPacket,RxBuffer,RxLength) < 0) {
        return IFD_COMMUNICATION_ERROR ;
    }                                 	
//	for(i=0;i<*RxLength;i++) {
//		printf("Resp = %0x \n", RxBuffer[i]);
//	}
	SmartCard[Lun].fd = scfdopen(SmartCard[Lun].fd,0,0,baud);
	return 0;
}	
#endif


RESPONSECODE IFDHCreateChannel ( DWORD Lun, DWORD Channel ) {

  /* Lun - Logical Unit Number, use this for multiple card slots 
     or multiple readers. 0xXXXXYYYY -  XXXX multiple readers,
     YYYY multiple slots. The resource manager will set these 
     automatically.  By default the resource manager loads a new
     instance of the driver so if your reader does not have more than
     one smartcard slot then ignore the Lun in all the functions.
     Future versions of PC/SC might support loading multiple readers
     through one instance of the driver in which XXXX would be important
     to implement if you want this.
  */
  
  /* Channel - Channel ID.  This is denoted by the following:
     0x000001 - /dev/pcsc/1
     0x000002 - /dev/pcsc/2
     0x000003 - /dev/pcsc/3
     
     USB readers may choose to ignore this parameter and query 
     the bus for the particular reader.
  */

  /* This function is required to open a communications channel to the 
     port listed by Channel.  For example, the first serial reader on COM1 would
     link to /dev/pcsc/1 which would be a sym link to /dev/ttyS0 on some machines
     This is used to help with intermachine independance.
     
     Once the channel is opened the reader must be in a state in which it is possible
     to query IFDHICCPresence() for card status.
 
     returns:

     IFD_SUCCESS
     IFD_COMMUNICATION_ERROR
  */
	int i;
	char devname[50];
	Lun=Lun&0xFF;
	SmartCard[Lun].temp=0;
	SmartCard[Lun].pcb= 0;
	SmartCard[Lun].oldpcb= 0;
	SmartCard[Lun].rchain= 0;
	SmartCard[Lun].ifs= 0x20;
	SmartCard[Lun].wtx= 1;
	SmartCard[Lun].abrt= 1;
	SmartCard[Lun].abrt= 1;
	SmartCard[Lun].device = device_data;
	SmartCard[Lun].sts_byte=0;
	SmartCard[Lun].memory_card=0;
		
for(i=0;i<NUM_SLOTS;i++)
{
	SmartCard[i].fd = scopen(i,0,0);
}
for(i=0;i<NUM_SLOTS;i++) 
{
   if( SmartCard[i].fd > 0)  
		break;
}
if(i== NUM_SLOTS) {
		perror("Unable to open ");
		return IFD_COMMUNICATION_ERROR ;
}
	PRINTF("In IFD handler open \n");   
for(i=0;i<NUM_SLOTS;i++)
   CardTracking(i);
   return IFD_SUCCESS;
}

RESPONSECODE IFDHCloseChannel ( DWORD Lun ) {
  
  /* This function should close the reader communication channel
     for the particular reader.  Prior to closing the communication channel
     the reader should make sure the card is powered down and the terminal
     is also powered down.

     returns:

     IFD_SUCCESS
     IFD_COMMUNICATION_ERROR     
  */
	Lun=Lun&0xff;
	PRINTF("In IFD handler close \n");   
 	if( close(SmartCard[Lun].fd) < 0) 
		return IFD_COMMUNICATION_ERROR;
	return IFD_SUCCESS; 

}

RESPONSECODE IFDHGetCapabilities ( DWORD Lun, DWORD Tag, 
				   PDWORD Length, PUCHAR Value ) {
  
  /* This function should get the slot/card capabilities for a particular
     slot/card specified by Lun.  Again, if you have only 1 card slot and don't mind
     loading a new driver for each reader then ignore Lun.

     Tag - the tag for the information requested
         example: TAG_IFD_ATR - return the Atr and it's size (required).
         these tags are defined in ifdhandler.h
         TAG_IFD_ATR			0x0303
         TAG_IFD_SLOTNUM                 0x0180
         TAG_IFD_SLOTS_NUMBER            0x0FAE

     Length - the length of the returned data
     Value  - the value of the data

     returns:
     
     IFD_SUCCESS
     IFD_ERROR_TAG
  */

/* Have to populate the structure Capabilities ( defined in ifdhandler.h ) & return */
RESPONSECODE lRetVal;
  DWORD HighNibble;
  DWORD LowNibble;
  int i;
	Lun=Lun&0xff;
  if( Value == 0 ) {
		PRINTF("Null pointer exception\n");
		return IFD_ERROR_TAG;
  }
  HighNibble = Tag >> 8;
  LowNibble  = Tag - ( HighNibble << 8 );   /* Shift left and subtract. */
  lRetVal = IFD_SUCCESS; 
  if ( HighNibble == 0x0F) {
	switch(LowNibble) {
	  case 0xAE:
		*Length = 1;
		*(char *) Value = NUM_SLOTS;
	}
  } else 
  if ( HighNibble == 0x02 ) {
 
    switch( LowNibble ) {
      case 0x01:
		*Length = sizeof(DWORD);
		*(DWORD *) Value = SmartCard[Lun].protocol.Protocol_Type;
        break;
	  default:
			PRINTF("default 1 \n");
		break;
 
    } 
 } else if ( HighNibble == 0x03 ) {      /* This is the ICC_STATE */
 
    switch( LowNibble ) {
 
      case 0x00:
			IFDHICCPresence(Lun);
			*Length = sizeof(char);
			*Value = SmartCard[Lun].ICC.ICC_Presence;
    		break;
      case 0x01:
			IFDHICCPresence(Lun);
			*Length = sizeof(DWORD); 
			if(SmartCard[Lun].ICC.ICC_Presence == SCARD_PRESENT ) {
				if( validStatus(SmartCard[Lun].sts_byte) == 1) 
					*(DWORD *) Value = SCARD_POWERED;
				else 
					*(DWORD *) Value = SCARD_SWALLOWED;
			}
			else 
				*(DWORD *) Value = SCARD_ABSENT ;
    		break;
      case 0x03:
		if( ICC.ICC_Presence ==  SCARD_PRESENT )
		{
			*Length = SmartCard[Lun].ATRLen;
	        memcpy(Value, SmartCard[Lun].ICC.ATR, SmartCard[Lun].ATRLen);
		}
		else 
			*Length =0;
    break;
      case 0x04:
			PRINTF("default 2 \n");
		break;
/*		
	forget about the foll. code. should return valid flags instead .
		{
		char tempStr[50];
		memcpy(tempStr,"",1);
		IFDHICCPresence(Lun);
		if(sts_byte & 0x10 ) 
			strcat(tempStr,"asynchronous smart card");
		else 
			strcat(tempStr,"synchronous memory card");
		if( sts_byte & 0x08)
			strcat(tempStr," T=1 protocol");
		else
			strcat(tempStr," T=0 protocol");
		*Length = strlen(tempStr);
		strcpy(Value,tempStr);	
	}	
    break;
*/
      default:
			PRINTF("default 3 \n");
    break;
 
    }
  }
  else if( HighNibble == 0x01 ) { /* Specific to the Reader */ 

		switch(LowNibble) {
		case 0x00:
			*Length = strlen(SmartCard[Lun].device.Vendor_Name);	
			strcpy(Value,SmartCard[Lun].device.Vendor_Name);
			break;
		case 0x01:
			*Length = strlen(SmartCard[Lun].device.IFD_Type);
			strcpy(Value,SmartCard[Lun].device.IFD_Type);
			break;
		case 0x02:
			*Length = sizeof(SmartCard[Lun].device.IFD_Version);
			*(DWORD *) Value = SmartCard[Lun].device.IFD_Version; 
			break;
		case 0x03:
			*Length = strlen(SmartCard[Lun].device.IFD_Serial);
			strcpy(Value,SmartCard[Lun].device.IFD_Serial);
			break;
		case 0x21:
			*Length = sizeof(SmartCard[Lun].device.Default_Clock);
			*(DWORD *) Value = SmartCard[Lun].device.Default_Clock;
			break;
			
		case 0x22:
			*Length = sizeof(SmartCard[Lun].device.Max_Clock);
			*(DWORD *) Value = SmartCard[Lun].device.Max_Clock;
			break;
		case 0x23:
			*Length = sizeof(SmartCard[Lun].device.Default_Data_Rate);
			*(DWORD *) Value = SmartCard[Lun].device.Default_Data_Rate ;
			break;
		case 0x24:
			*Length = sizeof(SmartCard[Lun].device.Max_Data_Rate);
			*(DWORD *)Value = SmartCard[Lun].device.Max_Data_Rate ;
			break;
		case 0x0125:
			*Length = sizeof(SmartCard[Lun].device.Max_IFSD);
			*(DWORD *) Value = SmartCard[Lun].device.Max_IFSD ;
			break;
		case 0x80:
			*Length=sizeof(char);
			*(char *)Value = NUM_SLOTS;
		default:
			PRINTF("default 4 \n");
	}
  }
 
  return lRetVal;
}                     
/*	PRINTF("In IFD handler GetCapabilities \n");   
	PRINTF("Tag = %0x \n",Tag);
	switch ( Tag ) {
		case TAG_IFD_ATR :
			*Length = ATRLen;
			memcpy(Value,ICC.ATR,ATRLen);
			break;
	}
	return IFD_SUCCESS;
	
} */

RESPONSECODE IFDHSetCapabilities ( DWORD Lun, DWORD Tag, 
			       DWORD Length, PUCHAR Value ) {

  /* This function should set the slot/card capabilities for a particular
     slot/card specified by Lun.  Again, if you have only 1 card slot and don't mind
     loading a new driver for each reader then ignore Lun.

     Tag - the tag for the information needing set

     Length - the length of the returned data
     Value  - the value of the data

     returns:
     
     IFD_SUCCESS
     IFD_ERROR_TAG
     IFD_ERROR_SET_FAILURE
     IFD_ERROR_VALUE_READ_ONLY
  */
	int i;
	Lun=Lun&0xff;
	PRINTF("In IFD handler Set Capabilities \n");   
	PRINTF("Tag = %0x Length = %0x\n",Tag,Length);
	for(i=0;i<Length;i++) 
		PRINTF("value[%d] = %0x \n",i,Value[i]); 
	
	return IFD_SUCCESS;
  
}

RESPONSECODE IFDHSetProtocolParameters ( DWORD Lun, DWORD Protocol, 
				   UCHAR SelectionFlags, UCHAR PTS1,
				   UCHAR PTS2, UCHAR PTS3) {

  /* This function should set the PTS of a particular card/slot using
     the three PTS parameters sent

     Protocol  - 0 .... 14  T=0 .... T=14
     Flags     - Logical OR of possible values:
     IFD_NEGOTIATE_PTS1 IFD_NEGOTIATE_PTS2 IFD_NEGOTIATE_PTS3
     to determine which PTS values to negotiate.
     PTS1,PTS2,PTS3 - PTS Values.

     returns:

     IFD_SUCCESS
     IFD_ERROR_PTS_FAILURE
     IFD_COMMUNICATION_ERROR
     IFD_PROTOCOL_NOT_SUPPORTED
  */
/*	switch(Protocol ) {
	case SCARD_PROTOCOL_T0:
	case SCARD_PROTOCOL_T1:
	}
	PRINTF("In IFD handler setProtocol Parameters\n");   
	return IFD_SUCCESS;
*/
    UCHAR control;
	PUCHAR TxBuffer;
	PUCHAR RxBuffer;
	int currentProtocol;
	DWORD TxLength;
	DWORD RxLength;
/*
    SPCmdPacket=alloca(INFINEER_MAX_BUFFER);
    TxBuffer =alloca(INFINEER_MAX_BUFFER);
    RxBuffer =alloca(INFINEER_MAX_BUFFER);
 
	control = 0x80;
	TxLength = 5;
	TxBuffer[0]= 00;
	TxBuffer[1]= 0x16;
	TxBuffer[2]= 00;
	if( Protocol == SCARD_PROTOCOL_T0) 	
		TxBuffer[3]= sts_byte & 0xF7;
	else 
		TxBuffer[3]=sts_byte |0x08;
	TxBuffer[4]= 00;
	
    ConstructSPCmdPacket(Lun,TxBuffer,SPCmdPacket,TxLength,control);
     if (IOCTL(fd,INFINEER_COMMAND,SPCmdPacket) < 0 )  {
                perror("IFDSetProtocolParameters  ");
                return IFD_COMMUNICATION_ERROR ;
        }
        if( ConstructResponseAPDU(Lun,SPCmdPacket,RxBuffer,&RxLength) < 0) {
                return IFD_COMMUNICATION_ERROR ;
        }
	if( ! validStatus(sts_byte) )  {
		Protocol = currentProtocol;
		return IFD_PROTOCOL_NOT_SUPPORTED;
	}
*/		
  RESPONSECODE rv;
  DWORD        PTSCount;
  UCHAR	       pucPTSRequest[6], pucPTSReply[6],SPCmdPacket[300];
  unsigned long        NewProtocol, BufferLength;
  int i;
	return IFD_SUCCESS;
	PRINTF("In IFD handler setProtocol Parameters\n");   
	if( SmartCard[Lun].ICC.ICC_Presence !=SCARD_PRESENT) 
		return IFD_COMMUNICATION_ERROR;
	if( SmartCard[Lun].ICC.ICC_Interface_Status != SCARD_POWERED ) 
		return IFD_COMMUNICATION_ERROR;
	currentProtocol = SmartCard[Lun].protocol.Protocol_Type;
//	if(Protocol == currentProtocol ) return IFD_SUCCESS;
  PRINTF("IFD Sublayer reader to set protocol %d\n", Protocol);

  PTSCount        = 0;

  /* Default PTSS             */
  pucPTSRequest[0] = 0xFF;
  
  /* set the format character */
   PRINTF("Protocol = %d\n",Protocol);
  if ( Protocol == 2 ) {
    pucPTSRequest[1] = (SelectionFlags*0x10) + 0x01;
  } else {
    pucPTSRequest[1] = (SelectionFlags*0x10) + 0x00;
  }
  
  PTSCount = 2;

  if ( SelectionFlags & 0x01 ) {
    pucPTSRequest[2] = PTS1;
    PTSCount += 1;
  }
  if ( SelectionFlags & 0x02 ) {
    pucPTSRequest[3] = PTS2;
    PTSCount += 1;
  }
  if ( SelectionFlags & 0x04 ) {
    pucPTSRequest[4] = PTS3;
    PTSCount += 1;
  }

  /*	check character  */

  pucPTSRequest[PTSCount] = 0x00;

  for ( i=0; i < PTSCount; i++ ) {
    pucPTSRequest[PTSCount] ^= pucPTSRequest[i];
  }  

      PRINTF("PTS Request: ");
      for ( i=0; i < PTSCount+1; i++ ) {
	PRINTF("%x ", pucPTSRequest[i]);
      } PRINTF("\n");
  /*	write PTSRequest */
	ConstructSPCmdPacket(Lun,pucPTSRequest,SPCmdPacket,PTSCount+1,0);
	for(i=0;i<PTSCount+4;i++)
		PRINTF("%0x ",SPCmdPacket[i]);
	PRINTF("\n");
	IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,SPCmdPacket);
	for(i=0;i<PTSCount+4;i++)
		PRINTF("%0x ",SPCmdPacket[i]);
	PRINTF("\n");
	ConstructResponseAPDU(Lun,SPCmdPacket,pucPTSReply,&PTSCount);
      PRINTF("PTS Response: ");
      for ( i=0; i < PTSCount; i++ ) {
	PRINTF("%x ", pucPTSReply[i]);
      } PRINTF("\n");

  if ( memcmp(pucPTSReply, pucPTSRequest, PTSCount) != 0 ) {
    rv = IFD_ERROR_PTS_FAILURE;
  } else {
    rv = IFD_SUCCESS;
  }
  return rv;
}


RESPONSECODE IFDHPowerICC ( DWORD Lun, DWORD Action, 
			    PUCHAR Atr, PDWORD AtrLength ) {

  /* This function controls the power and reset signals of the smartcard reader
     at the particular reader/slot specified by Lun.

     Action - Action to be taken on the card.

     IFD_POWER_UP - Power and reset the card if not done so 
     (store the ATR and return it and it's length).
 
     IFD_POWER_DOWN - Power down the card if not done already 
     (Atr/AtrLength should
     be zero'd)
 
    IFD_RESET - Perform a quick reset on the card.  If the card is not powered
     power up the card.  (Store and return the Atr/Length)

     Atr - Answer to Reset of the card.  The driver is responsible for caching
     this value in case IFDHGetCapabilities is called requesting the ATR and it's
     length.  This should not exceed MAX_ATR_SIZE.

     AtrLength - Length of the Atr.  This should not exceed MAX_ATR_SIZE.

     Notes:

     Memory cards without an ATR should return IFD_SUCCESS on reset
     but the Atr should be zero'd and the length should be zero

     Reset errors should return zero for the AtrLength and return 
     IFD_ERROR_POWER_ACTION.

     returns:

     IFD_SUCCESS
     IFD_ERROR_POWER_ACTION
     IFD_COMMUNICATION_ERROR
     IFD_NOT_SUPPORTED
  */
	PUCHAR SPCmdPacket,CmdAPDU;
	DWORD length;
	UCHAR control;
	int i;
	Lun=Lun&0xff;
	PRINTF("In IFD handler Power ICC \n");   
	if(Atr == 0) {
		PRINTF("Null pointer Exception \n");
		return IFD_SUCCESS;
	}
/*
*/
	SmartCard[Lun].pcb=0;
	SPCmdPacket = alloca(INFINEER_MAX_BUFFER);
	CmdAPDU = alloca(INFINEER_MAX_BUFFER);
	PRINTF("Action = %d \n" , Action);
	if ( Action ==  IFD_POWER_UP )  {
		CmdAPDU[0]=0x00;
        	CmdAPDU[1]=0x14;
        	CmdAPDU[2]=0x00;
		CmdAPDU[3]=1;
		CmdAPDU[4]=0x20;
		control = 0xC0;
		length = 5;
	}
	else if ( Action == IFD_POWER_DOWN)  {
                CmdAPDU[0]=0x00;
                CmdAPDU[1]=0x14;
                CmdAPDU[2]=0x00;
                CmdAPDU[3]=0x00;
                CmdAPDU[4]=0x20;
                control = 0xC0;
                length = 5;      

	}
	else if ( Action == IFD_RESET ) {
		CmdAPDU[0]=0x00;
        	CmdAPDU[1]=0x02;
        	CmdAPDU[2]=0x00;
          	CmdAPDU[3]=0x00;
        	CmdAPDU[4]=0x00;
       		length = 5;
        	control = 0xC0;
	}
	else {
		PRINTF("Unknown action\n");
		return IFD_NOT_SUPPORTED ;
	}
	ConstructSPCmdPacket(Lun,CmdAPDU,SPCmdPacket,length,control);	
	if (IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,SPCmdPacket) < 0 )  {
		perror("IFDPowerICC ");
		return IFD_COMMUNICATION_ERROR ;
	}
	if( ConstructResponseAPDU(Lun,SPCmdPacket,Atr,AtrLength) < 0) {
		return IFD_ERROR_POWER_ACTION;
	}
	if(Action == IFD_POWER_DOWN) {
		*AtrLength=0;
		Atr[0]=0;
		SmartCard[Lun].ICC.ATR[0]=0;
		SmartCard[Lun].ATRLen=0;
		SmartCard[Lun].ICC.ICC_Interface_Status = SCARD_SWALLOWED;
	}
	else {
		if( ! ( SmartCard[Lun].sts_byte & 0x10) ) {
			UCHAR Tx[300];
			UCHAR Rx[300];
			int Txlen,Rxlen;
			printf("No ATR ! . Memory card  / Card not inserted properly \n");
			SmartCard[Lun].memory_card=1;
			Tx[0]=0x00;
			Tx[1]=0x00;
			Tx[2]=0xA0;
			Tx[3]=0x00;
			Tx[4]=0x01;
			Txlen=5;
			IFDHControl(Lun,Tx,Txlen,Rx,&Rxlen);
			if( ( Rxlen ==3 ) && ( Rx[1] == 0x90)) {
				SmartCard[Lun].memory_card=1;
				return IFD_SUCCESS;
			}	
			else {
				UCHAR Tx[300];
            	UCHAR Rx[300];
            	int Txlen,Rxlen; 
#ifndef _USB_
				Tx[0]=3;
				Tx[1]=0x73;
				Tx[2]=0xBF;
				Tx[3]=0x00;
				Tx[4]=0x01;	
				Txlen=5;
				SmartCard[Lun].memory_card=1;
				IFDHControl(Lun,Tx,Txlen,Rx,&Rxlen);
#else
				UCHAR Cmd_MemReadT3[11] = {0x60,8,0x00,0x00,0x00,0x01,0x73,0xBF,0xBF^0x73^0x01 };
				memcpy(Tx,Cmd_MemReadT3,11);
				if( IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,Tx) < 0) 
					perror("Memory card read ");
				ConstructResponseAPDU(Lun,Tx,Rx,&Rxlen);
			printf("Rxlen = %d \n", Rxlen);
			for(i=0;i<Rxlen;i++)
				printf("%0x ",Rx[i]);
			printf("\n");
#endif
				if( ( Rxlen ==3 )  && ( Rx[0] != 0xFF ) ){
					printf("Type 3 memory card \n");
					SmartCard[Lun].memory_card=1;
	                return IFD_SUCCESS;
				}
				else {
				*AtrLength=0;
				SmartCard[Lun].memory_card=0;
				SmartCard[Lun].ICC.ICC_Interface_Status = SCARD_SWALLOWED;
				return IFD_COMMUNICATION_ERROR;
				}
			}
		}
		SmartCard[Lun].ATRLen=*AtrLength-2;
		*AtrLength-=2;
		memcpy(SmartCard[Lun].ICC.ATR,Atr,SmartCard[Lun].ATRLen);
		for ( i=0;i<*AtrLength;i++)
			PRINTF("%0x \n",*(Atr+i));
		if(*AtrLength == 0) 
			{
			SmartCard[Lun].ICC.ICC_Interface_Status = SCARD_SWALLOWED;
			return IFD_ERROR_POWER_ACTION;
		}
		SmartCard[Lun].ICC.ICC_Interface_Status = SCARD_POWERED;
		if( SmartCard[Lun].sts_byte & 0x08 )  {
			SmartCard[Lun].protocol.Protocol_Type =  1;
			PRINTF("before ATR Calc 5 \n");
			IFSRequest(Lun,0x20);
		// DO_WTX_REQUEST;
		}
		else
			SmartCard[Lun].protocol.Protocol_Type =  0;
	}
#ifdef _SERIAL_
	if ( Action ==  IFD_POWER_UP )  { 
		 int templen;
		 UCHAR temparr[10]	;
		 SetBaudRate(Lun,3,temparr,&templen);
	}
#endif

	return IFD_SUCCESS;	

}

RESPONSECODE IFDHTransmitToICC ( DWORD Lun, SCARD_IO_HEADER SendPci, 
				 PUCHAR TxBuffer, DWORD TxLength, 
				 PUCHAR RxBuffer, PDWORD RxLength, 
				 PSCARD_IO_HEADER RecvPci ) {
	RESPONSECODE rc;
	int tries;
	Lun=Lun&0xFF;
	if ( SendPci.Protocol == SCARD_PROTOCOL_T0 ) {
		rc = IFDHTransmitToICC_T0(Lun,TxBuffer,TxLength,RxBuffer,RxLength );
		memcpy(RecvPci,&SendPci,sizeof(RecvPci));
	}
	else if ( SendPci.Protocol == SCARD_PROTOCOL_T1 ) {
			tries=0;
		do {
			rc = IFDHTransmitToICC_T1(Lun,TxBuffer,TxLength,RxBuffer,RxLength );
			tries++;
		} while( rc == IFD_RESYNC && tries < 5);
		PRINTF("tries = %0x\n",tries);
		if( tries >=5 ) rc=IFD_SUCCESS;
		memcpy(RecvPci,&SendPci,sizeof(RecvPci));
	}
	else {
/*		rc = TransmitToICC_RAW(Lun,TxBuffer,TxLength,RxBuffer,RxLength ); */
		printf("Unsupported  protocol \n");
		memcpy(RecvPci,&SendPci,sizeof(RecvPci));
	}	
		return rc;	
}
RESPONSECODE IFDHTransmitToICC_T0 ( int Lun,PUCHAR TxBuffer, DWORD TxLength,
                 PUCHAR RxBuffer, PDWORD RxLength
                 ) { 
  
  /* This function performs an APDU exchange with the card/slot specified by
     Lun.  The driver is responsible for performing any protocol specific exchanges
     such as T=0/1 ... differences.  Calling this function will abstract all protocol
     differences.

     SendPci
     Protocol - 0, 1, .... 14
     Length   - Not used.

     TxBuffer - Transmit APDU example (0x00 0xA4 0x00 0x00 0x02 0x3F 0x00)
     TxLength - Length of this buffer.
     RxBuffer - Receive APDU example (0x61 0x14)
     RxLength - Length of the received APDU.  This function will be passed
     the size of the buffer of RxBuffer and this function is responsible for
     setting this to the length of the received APDU.  This should be ZERO
     on all errors.  The resource manager will take responsibility of zeroing
     out any temporary APDU buffers for security reasons.
  
     RecvPci
     Protocol - 0, 1, .... 14
     Length   - Not used.

     Notes:
     The driver is responsible for knowing what type of card it has.  If the current
     slot/card contains a memory card then this command should ignore the Protocol
     and use the MCT style commands for support for these style cards and transmit 
     them appropriately.  If your reader does not support memory cards or you don't
     want to then ignore this.

     RxLength should be set to zero on error.

     returns:
     
     IFD_SUCCESS
     IFD_COMMUNICATION_ERROR
     IFD_RESPONSE_TIMEOUT
     IFD_ICC_NOT_PRESENT
     IFD_PROTOCOL_NOT_SUPPORTED
  */
	PUCHAR SPCmdPacket;
        UCHAR control; 
	//printf("In IFD handler Transmit to ICC length = %d \n",TxLength);   
	if(RxBuffer == 0 ) {
		printf("RxBuffer null\n");
		return IFD_SUCCESS;
	}
	SPCmdPacket=alloca(INFINEER_MAX_BUFFER);
	if(TxLength < 4 ) {
		RxBuffer[0]=67;
		RxBuffer[1]=0;
		*RxLength=2;	
		printf("APDU Too Small \n");
		return IFD_COMMUNICATION_ERROR;
	}
	if( TxLength > 5 ) 
		if(!((TxLength == (TxBuffer[4]+5)) || (TxLength == (TxBuffer[4]+6))))
		{
			printf("LC(%d) field doesn't match with # of bytes to send(%d)\n",TxBuffer[5],TxLength-4);
			RxBuffer[0]=67;
			RxBuffer[1]=0;
			*RxLength=2;	
			return IFD_COMMUNICATION_ERROR;
		}
		
	if(TxLength == 5 ) 
		control=SC_READ;
	else 
		control=SC_WRITE;
	PRINTF("Control = %0x \n",control);
	// if(SendPci.Protocol == SCARD_PROTOCOL_T1 ) control |= SC_PROT_HANDLE_BIT;
	ConstructSPCmdPacket(Lun,TxBuffer,SPCmdPacket,TxLength,control);
	 if (IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,SPCmdPacket) < 0 )  {
                perror("IFDTransmit  ");
                return IFD_COMMUNICATION_ERROR ;
        }
        if( ConstructResponseAPDU(Lun,SPCmdPacket,RxBuffer,RxLength) < 0) {
                return IFD_COMMUNICATION_ERROR ;
        }              
	return IFD_SUCCESS;
  
}
void IFSRequest(int Lun , int val) {
	PUCHAR rbuff = alloca(INFINEER_MAX_BUFFER);
	PUCHAR buff  = alloca(INFINEER_MAX_BUFFER);
	UCHAR control=0x10;
	int length;
	int i;	
	buff[0]=0;
	buff[1] = 0xC1;
	buff[2]=01;
	buff[3]=val;
	buff[4]=0xC1^0x01^val;
	ConstructSPCmdPacket(Lun,buff,rbuff,5,control);
	PRINTF("before ioctl \n");
	if(IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,rbuff) < 0){
		perror("IFSRequest \n");
	}
	PRINTF("after ioctl \n");
	if( ConstructResponseAPDU(Lun,rbuff,buff,&length) < 0) 
		PRINTF("error in response \n");
	PRINTF("after resp \n");
	/*for(i=0;i<length;i++)
		PRINTF("result = %0x \n", buff[i]); */
	if( CalculatePCB(Lun,buff)== -1 ) {
		PRINTF("Card doesn't support IFS request\n");
		SmartCard[Lun].ifs=0x20;
	}
		SmartCard[Lun].pcb=0;
		SmartCard[Lun].oldpcb=0;
	}	
RESPONSECODE IFDHTransmitToICC_T1 ( int Lun,PUCHAR TxBuffer, DWORD TxLength,
                 PUCHAR RxBuffer, PDWORD RxLength
                 ) { 
  
  /* This function performs an APDU exchange with the card/slot specified by
     Lun.  The driver is responsible for performing any protocol specific exchanges
     such as T=0/1 ... differences.  Calling this function will abstract all protocol
     differences.

     SendPci
     Protocol - 0, 1, .... 14
     Length   - Not used.

     TxBuffer - Transmit APDU example (0x00 0xA4 0x00 0x00 0x02 0x3F 0x00)
     TxLength - Length of this buffer.
     RxBuffer - Receive APDU example (0x61 0x14)
     RxLength - Length of the received APDU.  This function will be passed
     the size of the buffer of RxBuffer and this function is responsible for
     setting this to the length of the received APDU.  This should be ZERO
     on all errors.  The resource manager will take responsibility of zeroing
     out any temporary APDU buffers for security reasons.
  
     RecvPci
     Protocol - 0, 1, .... 14
     Length   - Not used.

     Notes:
     The driver is responsible for knowing what type of card it has.  If the current
     slot/card contains a memory card then this command should ignore the Protocol
     and use the MCT style commands for support for these style cards and transmit 
     them appropriately.  If your reader does not support memory cards or you don't
     want to then ignore this.

     RxLength should be set to zero on error.

     returns:
     
     IFD_SUCCESS
     IFD_COMMUNICATION_ERROR
     IFD_RESPONSE_TIMEOUT
     IFD_ICC_NOT_PRESENT
     IFD_PROTOCOL_NOT_SUPPORTED
  */
	PUCHAR SPCmdPacket;
        UCHAR control; 
 	UCHAR nad=0;
	int OrgLength;
	int i;
	int chain=0;
	int length;
	UCHAR rbuffer[300];
	UCHAR buffer[300];
	int buff_start=0;
	PUCHAR tempbuffer;
	PUCHAR tempresult;
	DWORD templength;
	// PRINTF("In IFD handler Transmit to ICC length = %d Protocol = %d \n",TxLength,SendPci.Protocol );   
	if(RxBuffer == 0 ) {
		PRINTF("RxBuffer null\n");
		return IFD_SUCCESS;
	}
	SPCmdPacket=alloca(INFINEER_MAX_BUFFER);
	tempbuffer = alloca(INFINEER_MAX_BUFFER);
	tempresult = alloca(INFINEER_MAX_BUFFER);
	*RxLength=0;
	if(TxLength == 5 ) 
		control=SC_READ;
	else 
		control=SC_WRITE;
	PRINTF("Control = %0x here here \n",control);
	PRINTF("Rxlength = %0x \n", RxLength);
	*RxLength=0;
	PRINTF("Rxlength = %0x \n", RxLength);
	control |= SC_PROT_HANDLE_BIT;
	do {	
		printf("TxBuffer = ");
		for(i=buff_start;i<TxLength;i++)
			printf("%0x ",TxBuffer[i]);
		printf("\n");
		printf("pcb = %0x\n",SmartCard[Lun].pcb);
		if((SmartCard[Lun].pcb == 0xE0)|| (SmartCard[Lun].pcb == 0xC0 )) {
			memcpy(RxBuffer,tempresult,templength);
			*RxLength=templength;
			DO_RESPONSE(SmartCard[Lun].pcb,0);
			SmartCard[Lun].pcb=0;
		    SmartCard[Lun].oldpcb=0;
			chain=0;
			return IFD_RESYNC;
		}
		else if(SmartCard[Lun].pcb == 0xE1) {
			DO_RESPONSE(SmartCard[Lun].pcb,SmartCard[Lun].ifs);
			SmartCard[Lun].pcb=SmartCard[Lun].oldpcb;
			if( chain == -1 ) {
				chain=1;	
			}
		}
		else if( SmartCard[Lun].pcb == 0xE2 ) {
			DO_RESPONSE(SmartCard[Lun].pcb,SmartCard[Lun].abrt);
			PRINTF("Abort requested.... Try again \n");
				SmartCard[Lun].pcb=SmartCard[Lun].oldpcb;
			return IFD_COMMUNICATION_ERROR;
		}
		else if( SmartCard[Lun].pcb == 0xE3 ) {
			DO_RESPONSE(SmartCard[Lun].pcb,SmartCard[Lun].wtx);
			SmartCard[Lun].pcb=SmartCard[Lun].oldpcb;
		}
		else {
			tempbuffer[0]= nad; 
			tempbuffer[1] = SmartCard[Lun].pcb ; 
		if( TxLength > SmartCard[Lun].ifs ) {
//			OrgLength = TxLength-SmartCard[Lun].ifs+SmartCard[Lun].oldifs;
			OrgLength = TxLength-SmartCard[Lun].ifs;
			//IFSRequest(Lun,SmartCard[Lun].ifs);
			printf("Chaining \n");
			TxLength = SmartCard[Lun].ifs;
			chain=1;
			tempbuffer[1] |= PCB_MORE_BIT;
			if(SmartCard[Lun].pcb == 0x40)  
					SmartCard[Lun].pcb=0x00;
			else if(SmartCard[Lun].pcb == 0x00)
					SmartCard[Lun].pcb=0x40;
			SmartCard[Lun].oldpcb=SmartCard[Lun].pcb;
		}
		else {
			chain =-1;
			OrgLength=0;
	//		TxLength+=SmartCard[Lun].oldifs;
		}
	//	buff_start -=SmartCard[Lun].oldifs;
	//	SmartCard[Lun].oldifs=0;
		tempbuffer[2] = TxLength;
			memcpy(tempbuffer+3,TxBuffer+buff_start,TxLength);
			tempbuffer[3+TxLength] = calcChksum(TxBuffer+buff_start,TxLength)^tempbuffer[0]^tempbuffer[1]^tempbuffer[2];
			if( chain > 0 ) buff_start+=SmartCard[Lun].ifs;
		ConstructSPCmdPacket(Lun,tempbuffer,SPCmdPacket,TxLength+4,control);
		for(i=0;i<TxLength+7;i++)
			PRINTF("SPCmdPacket = %0x\n",SPCmdPacket[i]);
			
		 if (IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,SPCmdPacket) < 0 )  {
                perror("IFDTransmit  ");
                return IFD_COMMUNICATION_ERROR ;
        }
/*		for(i=0;i<SPCmdPacket[1]+3;i++)
			PRINTF("SPCmdResponse = %0x\n",SPCmdPacket[i]);
		
*/
        if( ConstructResponseAPDU(Lun,SPCmdPacket,tempresult,&templength) < 0) {
                return IFD_COMMUNICATION_ERROR ;
        }              
		TxLength=OrgLength;	
		if(TxLength<0) TxLength=0;
		PRINTF("TxLength = %0x\n",TxLength);
	}
		for(i=0;i<templength;i++)
			printf("tempbuffer = %0x\n",tempresult[i]);
			printf("templen = %d \n",templength);
		if(templength < 4 ) {
			if(tempresult[0]== 0x67 && tempresult[1]==0x09) {
				PRINTF("time out \n");
				SmartCard[Lun].pcb=0xC0;
				memcpy(RxBuffer,tempresult,templength);
				*RxLength=templength;
				DO_RESPONSE(SmartCard[Lun].pcb,0);
				SmartCard[Lun].pcb=0;
				SmartCard[Lun].oldpcb=0;
				return IFD_RESYNC;
			}
			memcpy(RxBuffer,tempresult,templength);
			*RxLength=templength;
			return IFD_SUCCESS;
		} 
		else if( templength > 5 ) {
		PRINTF("RxLength = %0x\n",*RxLength);
		memcpy(RxBuffer+(*RxLength),tempresult+3,templength-4);
			*RxLength +=(templength-4);
		}
	} while( (CalculatePCB(Lun,tempresult) >= 1) || chain>0 || SmartCard[Lun].rchain ) ;
/*	if( (SmartCard[Lun].pcb != 0x40 ) && (  SmartCard[Lun].pcb!= 0x00) && (  SmartCard[Lun].pcb!= 0x20) )*/
		SmartCard[Lun].pcb=SmartCard[Lun].oldpcb;
		printf("PCB == %0x \n", SmartCard[Lun].pcb );
	if( ( SmartCard[Lun].pcb == 0x00  )  || (SmartCard[Lun].pcb == 0x20 ) )
		SmartCard[Lun].pcb = 0x40;
	else 
		SmartCard[Lun].pcb=0x00;
	SmartCard[Lun].oldpcb=SmartCard[Lun].pcb;
	
	
	for(i=0;i<*RxLength;i++) 
		printf("Result = %0x \n",RxBuffer[i]);
//	if( *RxLength < 4) return IFD_COMMUNICATION_ERROR;
//	for(i=0;i< *RxLength-4;i++) {
//		RxBuffer[i]=RxBuffer[i+3];
//	}
//		if( *RxLength > 4 ) *RxLength -=4;
	return IFD_SUCCESS;
  
}
RESPONSECODE IFDHControl ( DWORD Lun, PUCHAR TxBuffer, 
			 DWORD TxLength, PUCHAR RxBuffer, 
			 PDWORD RxLength ) {

  /* This function performs a data exchange with the reader (not the card)
     specified by Lun.  Here XXXX will only be used.
     It is responsible for abstracting functionality such as PIN pads,
     biometrics, LCD panels, etc.  You should follow the MCT, CTBCS 
     specifications for a list of accepted commands to implement.

     TxBuffer - Transmit data
     TxLength - Length of this buffer.
     RxBuffer - Receive data
     RxLength - Length of the received data.  This function will be passed
     the length of the buffer RxBuffer and it must set this to the length
     of the received data.

     Notes:
     RxLength should be zero on error.
  */
	PUCHAR SPCmdPacket;
        UCHAR control;
	int memReadOrWrite=1;
	int i; 
	Lun=Lun&0xff;
	PRINTF("In IFD handler Control \n");   
	if(RxBuffer == 0 ) {
		PRINTF("RxBuffer null\n");
		return IFD_SUCCESS;
	}
	if( TxLength == 5 ) 
		control = 0x40;
	else 
		control = 0x00;
	if( SmartCard[Lun].memory_card ) {
		if( TxLength > 5 )
			memReadOrWrite=1;
		else
		if( TxBuffer[0] != 0x00 )
			memReadOrWrite=1;
		else 
		if ( TxBuffer[1] != 0)
			memReadOrWrite=0;
		else
			if(TxBuffer[4] == 0)
			memReadOrWrite=0;
	}
	else memReadOrWrite = 0;
printf("memReadOrWrite %d \n", memReadOrWrite );
	if ( ! memReadOrWrite ) {	
		switch( TxBuffer[1] ) {
			case 0: 
					control=0xC0;
					break;
			case 2:
					control =0xC0;
					break;
			case 4: 
#ifdef _SERIAL_
					SetBaudRate(Lun,TxBuffer[2],RxBuffer,RxLength);
#endif
					return IFD_SUCCESS;
			case 0x0E: 
					control = 0x80;
					break;
			case 0x10:
					control = 0x80;
					break;
		
			case 0x14:
					control = 0xC0;
					break;
			case 0x16:
					if( TxBuffer[3] == 0 )
						control = 0xC0;
					else 
						control = 0x80;
		}
	}
	printf("control = %d \n", control);			
    SPCmdPacket=alloca(INFINEER_MAX_BUFFER);
	for(i=0;i<TxLength;i++) {
		printf("%0x ", TxBuffer[i]);
	}
	printf("\n");
    ConstructSPCmdPacket(Lun,TxBuffer,SPCmdPacket,TxLength,control);
	for(i=0;i<TxLength+3;i++) {
		printf("%0x ", SPCmdPacket[i]);
	}
	printf("\n");
	
	if (IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,SPCmdPacket) < 0 )  {
                perror("IFDControl ");
                return IFD_COMMUNICATION_ERROR ;
    }
    if( ConstructResponseAPDU(Lun,SPCmdPacket,RxBuffer,RxLength) < 0) {
               return IFD_COMMUNICATION_ERROR ;
    }                          
	for(i=0;i<*RxLength;i++) {
		printf("%0x ", RxBuffer[i]);
	}
	printf("\n");
    return IFD_SUCCESS;                
}

RESPONSECODE IFDHICCPresence( DWORD Lun ) {

  /* This function returns the status of the card inserted in the 
     reader/slot specified by Lun.  It will return either:

     returns:
     IFD_ICC_PRESENT
     IFD_ICC_NOT_PRESENT
     IFD_COMMUNICATION_ERROR
  */
	PUCHAR CmdAPDU,SPCmdPacket,RespAPDU;
	DWORD length;
	UCHAR control;
	Lun=Lun&0xff;
	PRINTF("In IFD handler ICCPresence \n");   
	CmdAPDU = alloca(INFINEER_MAX_BUFFER);
	SPCmdPacket= alloca(INFINEER_MAX_BUFFER);
	RespAPDU = alloca(INFINEER_MAX_BUFFER);
	CmdAPDU[0]=0;
	CmdAPDU[1]=0x16;
	CmdAPDU[2]=0;
	CmdAPDU[3]=0;
	CmdAPDU[4]=0;
	length = 5;
	control = 0xC0;
	ConstructSPCmdPacket(Lun,CmdAPDU,SPCmdPacket,length,control);
	if (IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,SPCmdPacket) < 0 )  {
                perror("IFDPresence  ");
                return IFD_COMMUNICATION_ERROR ;
        }
        if( ConstructResponseAPDU(Lun,SPCmdPacket,RespAPDU,&length) < 0) {
                return IFD_COMMUNICATION_ERROR ;
        }                          
	if ( SmartCard[Lun].sts_byte & 0x40 )  {
		SmartCard[Lun].ICC.ICC_Presence = SCARD_PRESENT;
		return IFD_ICC_PRESENT;
	}
	else 
		{
		SmartCard[Lun].ICC.ICC_Presence = SCARD_ABSENT;
		return IFD_ICC_NOT_PRESENT;
	}
}

static UCHAR calcChksum(PUCHAR data,DWORD length )
{
       DWORD i;
       unsigned char chksum=0;
       for(i=0;i<length;i++) {
               chksum^=data[i];
       }
       // PRINTF("chksum = %0x \n",chksum);
       return chksum;
}

static int validStatus( UCHAR status ) {
       if( status & READER_ERROR )
               return 0;
       if ( !(status & POWERED) )
               return 0;
       return 1;
}

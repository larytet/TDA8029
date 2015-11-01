#define READER_ERROR 0x02
#define POWERED 0x04
#define CONTROL_LENGTHBIT 0x20
#define STATUS_LENGTHBIT 0x80
#define INFINEER_MAX_BUFFER 275
#define SC_READ 0x40
#define SC_WRITE 0x00
#define SCARD_PROTOCOL_T0 0
#define SCARD_PROTOCOL_T1 1
#define SCARD_PROTOCOL_UNDFINED -1
#define SC_PROT_HANDLE_BIT 0x10
#define MAX_IFSD 0x00FE
#define SBLOCK_RESP_BIT 0x20
#define PCB_MORE_BIT 0x20
#ifdef _PCMCIA_
#define NUM_SLOTS 2
#else
#define NUM_SLOTS 1
#endif
#define DO_RESPONSE(PCB,PARAM) \
	buffer[0]= 0x00; \
	buffer[1]= PCB; \
	if( PARAM == 0 ) { \
	buffer[2]=0; \
	buffer[3]=PCB; \
	length=4; \
	} \
	else { \
	buffer[2]=0x01; \
	buffer[3]=PARAM;\
	buffer[4]=PCB^0x01^PARAM; \
	length = 5; \
	} \
	ConstructSPCmdPacket(Lun,buffer,rbuffer,length,0x10); \
	for(i=0;i<length+3;i++)  \
		printf("%0x ",rbuffer[i]); \
	printf("\n"); \
	printf("before ioctl in do response PARAM = %0x \n",PARAM); \
	if( IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,rbuffer) < 0 ) { \
		printf("error in WTX_RESPONSE \n"); \
	} \
	printf("after ioctl in do response \n"); \
	ConstructResponseAPDU(Lun,rbuffer,tempresult,&templength); \
	printf("Length = %d\n",templength); \
	for(i=0;i<templength;i++) \
        printf("result = %0x \n", tempresult[i]);

#define DO_WTX_REQUEST \
	buffer[0]=0x10; \
    buffer[1]=0x05; \
    buffer[2]=0x00; \
    buffer[3]=0x00; \
    buffer[4]=0xC3; \
    buffer[5]=0x01; \
    buffer[6]=0x10; \
    buffer[7]=0xC3;\
    if( IOCTL(SmartCard[Lun].fd,INFINEER_COMMAND,buffer) < 0) { \
        printf("error in WTX_RESPONSE \n"); \
    } \
	ConstructResponseAPDU(Lun,buffer,rbuffer,&rlen); \
	for(i=0;i<rlen;i++) \
		printf("rbuff[%d] = %0x \n",i,rbuffer[i]);


struct smartport_t {
	int fd; /* This is the file descriptor to be used for the device */
	int temp;
	int rlen;
	UCHAR sts_byte;
	UCHAR pcb;
	int rchain;
	UCHAR oldpcb;
	int ATRLen;
	int ifs,wtx,abrt;
	int memory_card;
	DEVICE_CAPABILITIES device ;
 
	PROTOCOL_OPTIONS protocol;
 
	ICC_STATE ICC;	
	} ;	




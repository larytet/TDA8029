# Muscle Smartcard Development
# Makefile
# David Corcoran
PCSCHOME=/usr/local/include
CC       = cc
CFLAGS   = -g -fpic -w 
LD       = ld -g 
LEX      = flex
INCLUDE  = -I. -I${PCSCHOME}/include

MAKEXE   = make
SERIAL   = libDT3000.so
PCMCIA   = libLT4000.so
USB      = libDT3500.so
OBJ_SERIAL = ifdhandler_serial.o
OBJ_PCMCIA = ifdhandler_pcmcia.o
OBJ_USB    = ifdhandler_usb.o
PREFIX   = /usr/local/pcsc

#DEFS     = -DDEBUG

all: $(SERIAL)


$(USB)    : $(OBJ_USB)
	$(LD) -shared ifdhandler_usb.o -o $(USB)

$(PCMCIA) : $(OBJ_PCMCIA)
	$(LD) -shared ifdhandler_pcmcia.o -o $(PCMCIA)
	
$(SERIAL) : $(OBJ_SERIAL)
	$(LD) -shared ifdhandler_serial.o serial.o -o $(SERIAL)

$(OBJ_PCMCIA) : ifdhandler.c ifdhandler.h pcscdefines.h infineer.h
	$(CC) -D_PCMCIA_ $(CFLAGS) -c $< $(INCLUDE) 
	mv ifdhandler.o ifdhandler_pcmcia.o

$(OBJ_USB) : ifdhandler.c ifdhandler.h infineer.h pcscdefines.h
	$(CC) -D_USB_ $(CFLAGS) -c $< $(INCLUDE)
	mv ifdhandler.o ifdhandler_usb.o

$(OBJ_SERIAL) : ifdhandler.c serial.c ifdhandler.h  infineer.h pcscdefines.h serial.h
	$(CC) -D_SERIAL_ $(CFLAGS) -c ifdhandler.c $(INCLUDE)
	$(CC) -D_SERIAL_ $(CFLAGS) -c serial.c $(INCLUDE)
	mv ifdhandler.o ifdhandler_serial.o
clean:
	rm -f *.o $(USB) $(PCMCIA) $(SERIAL) core

osx: $(OBJ)
	echo $OBJ
	$(CC) -dynamiclib $(OBJ) -o $(LIBNAME)

unix: $(LIBNAME)

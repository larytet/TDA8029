#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/types.h>
#include <sys/time.h>


#include "serial.h"

/* modified by Gregor A. Panstruga */ 

static char ttynametmpl[] = "/dev/ttyS%01d";

/* Right now we only save termios state for the last tty opened, which is fine
   as long as only one tty is open at a time. */

struct termios sct;


int
scopen(int ttyn, int flags, int *ep)
{
    char ttyname[32];
    int fd, oflags;
	unsigned char data;

    sprintf(ttyname, ttynametmpl, ttyn);
    oflags = O_RDWR;
    if (!(flags & SCODCD))
	oflags |= O_NONBLOCK;
    if ((fd = open(ttyname, oflags, 0)) < 0) {
	if (ep)
	    *ep = SCENOTTY;
	return -1;
    }
    if ((fd = scfdopen(fd, flags, ep,1200)) < 0) {
	close(fd);
	return -1;
    }

    if (flags & SCODSR)
	while (!scdsr(fd))
    scsleep(100);
	scgetc(fd,&data,10);
	if( data == '(' ) {
	while ( data != ')' ) {
        scgetc(fd,&data,10);
    }             
	}
    if (flags & SCODSR)
	while (!scdsr(fd))
    scsleep(100); 
	if ((fd = scfdopen(fd, flags, ep,9600)) < 0) {
    close(fd);
    return -1;
    }    
    if (flags & SCODSR)
	while (!scdsr(fd))
    scsleep(100); 
    return fd;
}

int
scfdopen(int fd, int flags, int *ep,int baud)
{
    struct termios t;

    /* Get and save the tty state */
if( baud != 1200 )  {
    if (tcgetattr(fd, &sct) < 0) {
	if (ep)
	    *ep = SCENOTTY;
	return -1;
    }
}
 t = sct;

    /* Now put the tty in a happy ISO state */

	if ( baud == 1200) {
    	cfsetispeed(&t, B1200);
		cfsetospeed(&t, B1200);
		/* 7/N/1 */
		t.c_cflag &= ~(CSIZE | PARODD);
		t.c_cflag &= ~PARENB;     
		t.c_cflag |= CS7;
	}
	else if ( baud == 9600 ) {
		cfsetispeed(&t, B9600);
		cfsetospeed(&t, B9600);
			/* 8/N/1 */
		t.c_cflag &= ~(CSIZE | PARODD);
		t.c_cflag |= CS8;
		t.c_cflag &= ~PARENB;     
	}
	else if ( baud == 2400 ) {
		cfsetispeed(&t,B2400);
		cfsetospeed(&t, B2400);
		t.c_cflag &= ~(CSIZE | PARODD);
		t.c_cflag |= CS8;
		t.c_cflag &= ~PARENB;     
	}
	else if ( baud == 4800 ) {
		cfsetispeed(&t,B4800);
		cfsetospeed(&t, B4800);
		t.c_cflag &= ~(CSIZE | PARODD);
		t.c_cflag |= CS8;
		t.c_cflag &= ~PARENB;     
	}
	else if ( baud == 19200 ) {
		cfsetispeed(&t,B19200);
		cfsetospeed(&t, B19200);
		t.c_cflag &= ~(CSIZE | PARODD);
		t.c_cflag |= CS8;
		t.c_cflag &= ~PARENB;     
	}
    /* raw */
    t.c_iflag &= ~(ISTRIP|INLCR|IGNCR|ICRNL|IXON);
    t.c_iflag |= (IGNBRK|IGNPAR);
    t.c_oflag &= ~OPOST;
    t.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
    t.c_cc[VMIN] = 1;
    t.c_cc[VTIME] = 0;
    t.c_cflag |= CLOCAL;
#ifdef CHWFLOW
    t.c_cflag &= ~CHWFLOW;
#endif
    if (tcsetattr(fd, TCSANOW, &t) < 0) {
	if (ep)
	    *ep = SCENOTTY;
	return -1;
    }

    /* The open may or may not have reset the card.  Wait a while then flush
       anything that came in on the port. */
    scsleep(10);
	scdrain(fd);
    return fd;
}

/* query dsr on the port */

int 
scdsr(int fd)
{
    int i;

    if (fd < 0 || ioctl(fd, TIOCMGET, &i) < 0)
        return 0;
    
    return ((i & TIOCM_DSR) ? 1 :0);
}

int
scclose(int fd)
{
    tcsetattr(fd, TCSANOW, &sct);
    close(fd);
    return 0;
}

/*
 * get one byte from the terminal.
 * wait at most ms msec.  0 for poll, -1 for infinite.
 * return byte in *cp.
 * return 0 or error.
 */

int
scgetc(int fd, char *cp, int ms)
{
    fd_set fdset;
    struct timeval tv, *tvp;

    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);

    if (ms == -1)
	tvp = NULL;
    else {
	tv.tv_sec = (ms) / 1000;
	tv.tv_usec = (ms % 1000) * 1000;
	tvp = &tv;
    }

    if (select(fd + 1, &fdset, NULL, NULL, tvp) != 1)
	return SCTIMEO;

    if (read(fd, cp, 1) != 1)
	return SCTIMEO;
    return SCEOK; /* 0 */
}

/* write one byte to the terminal */

int
scputc(int fd, char c)
{
    int retval;
      
    retval = write(fd, &c, 1);
    if (retval == 1)
       return 0;
    
    return -1;
}

void
scsleep(int ms)
{
    struct timeval tv;

    if (!ms)
	return;
    tv.tv_sec = (ms + 1) / 1000;
    tv.tv_usec = (ms % 1000) * 1000;

    select(0, NULL, NULL, NULL, &tv);
}

void
scdrain(int fd)
{
    tcflush(fd, TCIFLUSH);
}



/*
copyright 1997
the regents of the university of michigan
all rights reserved
 
permission is granted to use, copy, create derivative works
and redistribute this software and such derivative works
for any purpose, so long as the name of the university of
michigan is not used in any advertising or publicity
pertaining to the use or distribution of this software
without specific, written prior authorization.  if the
above copyright notice or any other identification of the
university of michigan is included in any copy of any
portion of this software, then the disclaimer below must
also be included.
 
this software is provided as is, without representation
from the university of michigan as to its fitness for any
purpose, and without warranty by the university of
michigan of any kind, either express or implied, including
without limitation the implied warranties of
merchantability and fitness for a particular purpose. the
regents of the university of michigan shall not be liable
for any damages, including special, indirect, incidental, or
consequential damages, with respect to any claim arising
out of or in connection with the use of the software, even
if it has been or is hereafter advised of the possibility of
such damages.
*/                                                                             

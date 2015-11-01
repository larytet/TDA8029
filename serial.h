/* open flags */
#define SCODSR		1	/* wait for dsr */
#define SCODCD		2	/* wait for dcd */

/* error codes */
#define SCEOK		0
#define SCENOTTY	1	/* no such tty */
#define SCENOMEM	2	/* malloc (or similar) failed */
#define SCTIMEO		3	/* time out */
#define SCESLAG		4	/* slag (no atr) */
#define SCENOSUPP	5	/* card type not supported */
#define SCENOCARD	6	/* no card in reader */

extern char *scerrtab[];

int scopen(int ttyn, int flags, int *ep);
int scfdopen(int fd, int flags, int *ep,int baud);
int scclose(int fd);
int scgetc(int fd, char *cp, int ms);
int scputc(int fd, char cp);
void scsleep(int ms);
void scdrain(int fd);

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

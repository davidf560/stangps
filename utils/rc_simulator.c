#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
        
#define BAUDRATE B38400
#define MODEMDEVICE "/dev/com1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
        
volatile int STOP=FALSE; 
       
main()
{
  int fd,c, res;
  struct termios options;
  unsigned char buf[255];
        
  fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY ); 
  if (fd <0) {perror(MODEMDEVICE); exit(-1); }
        
  tcgetattr(fd,&options); /* save current port settings */
        
//  cfsetispeed(&options, BAUDRATE);
//  cfsetospeed(&options, BAUDRATE);

  bzero(&options, sizeof(options));
  options.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;

  // Set input mode
  options.c_lflag = 0;
         
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 5;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);
        
  while (STOP==FALSE) {
    res = read(fd, buf, 1);
    printf("Recvd: %d char: %02x\n", res, buf[0]);
    if(buf[0] == 'X') {
      STOP = TRUE;
    }
  }

  STOP = FALSE;
  read(fd, buf, 4);
        
  while (STOP==FALSE) {       /* loop for input */
    res = read(fd,buf,5);   /* returns after 5 chars have been input */
    buf[res]=0;               /* so we can printf... */
    printf("%c: %d.%02d ", buf[0], (buf[2] * 256) + buf[3], 
        (buf[4] * 100) / 256);
    if(buf[0] == 'X') {
      printf("\n");
    }
    if (buf[0]=='z') STOP=TRUE;
  }
}
    

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define  ATTN_CHAR 0x06

int main(int argc, char **argv)
{
  unsigned char buf[128];
  int fd, num;
  struct termios opts;
  int rc = 1;
  int cc = 1;

  if(argc > 1) {
    if(argv[1][0] == 'r') {
      cc = 0;
    }
    else if(argv[1][0] == 'c') {
      rc = 0;
    }
  }

  if((fd = open("/dev/com1", O_RDWR | O_NOCTTY)) == -1) {
    printf("Unable to open port.\n");
    exit(1);
  }

  tcgetattr(fd, &opts);
  opts.c_lflag &= ~(ICANON | ECHO | ISIG);
  opts.c_cflag |= (CLOCAL | CREAD);
  cfsetispeed(&opts, B9600);
  cfsetospeed(&opts, B9600);
  opts.c_cflag &= ~CRTSCTS;
  opts.c_iflag |= (INPCK | ISTRIP);
  opts.c_iflag &= ~(IXON | IXOFF | IXANY);
  opts.c_cflag &= ~CSIZE;
  opts.c_cflag |= CS8;
  opts.c_oflag &= ~OPOST;
  opts.c_cc[VMIN] = 1;
  opts.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &opts);

  while(1) {
    write(fd, buf, 1);    

    num = read(fd, buf, 1);
    fflush(stdout);
    if(buf[0] == ATTN_CHAR)
    {
      num = read(fd, buf, 3);
      if(cc) {
        printf("\nX: %d ", buf[0]);
        printf("Y: %d ", buf[1]);
        printf("Theta: %d\t", buf[2]);
      }
    }
    else
    {
      if(rc) {
        printf("%c", buf[0]);
      }
    }
  }

  close(fd);
}

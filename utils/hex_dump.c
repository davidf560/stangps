#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

int main(int argc, char **argv)
{
  unsigned char buf[128];
  int fd, num, i, status;
  struct termios opts;

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

  ioctl(fd, TIOCMGET, &status);
  status &= ~TIOCM_DTR;
  ioctl(fd, TIOCMSET, &status);

  for(i = 0; 1; i++) {
    if(((i % 4) == 0) && (i != 0)) {
      printf("  ");
    }
    if(i == 16) {
      printf("\n");
      i = 0;
    }
    num = read(fd, buf, 1);

    printf("%02X ", buf[0]);

    fflush(stdout);
  }

  close(fd);
}

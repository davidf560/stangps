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
  int fd, num;
  struct termios opts;
  unsigned char attn_byte = 185;

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
  opts.c_cc[VTIME] = 5;
  tcsetattr(fd, TCSANOW, &opts);

  write(fd, &attn_byte, 1);

  while((num = read(fd, buf, 1)) < 1);
  printf("RSR: 0x%02x\n", buf[0]);

  if(buf[0] & 0x80) {
    printf("  POWER ON\n");
  }
  if(buf[0] & 0x40) {
    printf("  RESET PIN\n");
  }
  if(buf[0] & 0x20) {
    printf("  WATCHDOG\n");
  }
  if(buf[0] & 0x10) {
    printf("  ILLEGAL OPCODE\n");
  }
  if(buf[0] & 0x08) {
    printf("  ILLEGAL ADDRESS\n");
  }
  if(buf[0] & 0x04) {
    printf("  MONITOR MODE\n");
  }
  if(buf[0] & 0x02) {
    printf("  LOW VOLTAGE INHIBIT\n");
  }

  close(fd);
}

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
  unsigned char attn_byte = 184;

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

  while(1) {
    write(fd, &attn_byte, 1);

    while((num = read(fd, buf, 4)) < 1);
    printf("X: %d.%02d ", buf[0] * 256 + buf[1], (buf[2] * 100) / 256);

    while((num = read(fd, buf, 4)) < 1);
    printf("Y: %d.%02d ", buf[0] * 256 + buf[1], (buf[2] * 100) / 256);

    while((num = read(fd, buf, 4)) < 1);
    printf("Theta: %d ", buf[0]);
    
    while((num = read(fd, buf, 1)) < 1);
    printf("Gyro: %d ", buf[0]);
    
    while((num = read(fd, buf, 1)) < 1);
    printf("PotBrads: %d ", buf[0]);
    
    while((num = read(fd, buf, 1)) < 1);
    printf("PotValue: %d ", buf[0]);
    
    while((num = read(fd, buf, 1)) < 1);
    printf("RSR: 0x%02x\n", buf[0]);

    usleep(26000);
  }

  close(fd);
}

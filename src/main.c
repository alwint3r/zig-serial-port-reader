#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int configure_serial_port(int fd, speed_t baud_rate) {
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    fprintf(stderr, "Error getting terminal attributes: %s\n", strerror(errno));
    return -1;
  }

  cfsetispeed(&tty, baud_rate);
  cfsetospeed(&tty, baud_rate);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
#if defined(CRTSCTS)
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | INLCR | IGNCR | ICRNL);

  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    fprintf(stderr, "Error setting terminal attributes: %s\n", strerror(errno));
    return -1;
  }

  return 0;
}

int main(int argc, char** argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s <serial_device> <baud>\n", argv[0]);
    fprintf(stderr, "Example: %s /dev/ttyUSB0 115200\n", argv[0]);
    return 1;
  }

  int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    fprintf(stderr, "Error opening serial port: %s\n", strerror(errno));
    return 1;
  }

  speed_t baud_rate = (speed_t)atoi(argv[2]);
  if (configure_serial_port(fd, baud_rate) != 0) {
    close(fd);
    return 1;
  }

  printf("Serial port %s configured successfully.\n", argv[1]);

  char buf[1024] = {0};
  while (1) {
    ssize_t read_len = read(fd, buf, sizeof(buf) - 1);
    if (read_len > 0) {
      buf[read_len] = '\0';
      fwrite(buf, 1, read_len, stdout);
      fflush(stdout);
    } else if (read_len < 0) {
      fprintf(stderr, "Error reading from serial port: %s\n", strerror(errno));
      break;
    }
  }

  close(fd);

  return 0;
}

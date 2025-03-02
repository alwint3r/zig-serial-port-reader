#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
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

int reset_device_using_dtr(int fd) {
  int status;
  if (ioctl(fd, TIOCMGET, &status) == -1) {
    fprintf(stderr, "Error getting modem status: %s\n", strerror(errno));
    return -1;
  }

  status &= ~TIOCM_DTR;
  if (ioctl(fd, TIOCMSET, &status) == -1) {
    fprintf(stderr, "Error setting modem status: %s\n", strerror(errno));
    return -1;
  }
  usleep(100000);

  status |= TIOCM_DTR;
  if (ioctl(fd, TIOCMSET, &status) == -1) {
    fprintf(stderr, "Error setting modem status: %s\n", strerror(errno));
    return -1;
  }

  return 0;
}

typedef enum {
  OPM_RESET,
  OPM_READ,
  OPM_UNKNOWN = -1,
} OperatingMode;

OperatingMode parse_opmode(const char* mode) {
  if (strcmp(mode, "reset") == 0) {
    return OPM_RESET;
  } else if (strcmp(mode, "read") == 0) {
    return OPM_READ;
  } else {
    return OPM_UNKNOWN;
  }
}

int main(int argc, char** argv) {
  if (argc != 4) {
    fprintf(stderr, "Usage: %s <mode> <serial_device> <baud>\n", argv[0]);
    fprintf(stderr, "Example: %s reset /dev/ttyUSB0 115200\n", argv[0]);
    return 1;
  }

  const char* s_opmode = argv[1];
  const char* s_serial_port = argv[2];
  const char* s_baud = argv[3];

  OperatingMode opmode = parse_opmode(s_opmode);
  if (opmode == OPM_UNKNOWN) {
    fprintf(stderr, "Unknown operating mode: %s\n", s_opmode);
    return 1;
  }

  int fd = open(s_serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    fprintf(stderr, "Error opening serial port: %s\n", strerror(errno));
    return 1;
  }

  speed_t baud_rate = (speed_t)atoi(s_baud);
  if (configure_serial_port(fd, baud_rate) != 0) {
    close(fd);
    return 1;
  }

  printf("Serial port %s configured to %s baud.\n", s_serial_port, s_baud);
  printf("Operating mode: %s (%d)\n", s_opmode, (int)opmode);

  if (opmode == OPM_RESET) {
    if (reset_device_using_dtr(fd) != 0) {
      printf("Failed to reset device.\n");
    } else {
      printf("Device reset successfully.\n");
    }
    close(fd);
    return 0;
  }

  printf("Serial port %s configured successfully.\n", s_serial_port);

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

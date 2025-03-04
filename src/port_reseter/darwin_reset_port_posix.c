#include "darwin_reset_port_posix.h"

int darwin_reset_port_dtr(int fd) {
  int status;
  if (ioctl(fd, TIOCMGET, &status) == -1) {
    return -1;
  }

  status &= ~TIOCM_DTR;
  if (ioctl(fd, TIOCMSET, &status) == -1) {
    return -1;
  }

  usleep(100000);

  status |= TIOCM_DTR;
  if (ioctl(fd, TIOCMSET, &status) == -1) {
    return -1;
  }

  return 0;
}

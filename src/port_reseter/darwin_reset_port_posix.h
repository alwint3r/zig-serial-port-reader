#pragma once

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

int darwin_reset_port_dtr(int fd);

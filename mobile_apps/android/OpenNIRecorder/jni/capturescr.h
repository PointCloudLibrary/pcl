#ifndef __CAPTURESCR_H__
#define __CAPTURESCR_H__

#include <stdlib.h>
#include <unistd.h>

#include <fcntl.h>
#include <stdio.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <time.h>

#include <linux/fb.h>
#include <linux/kd.h>

#include "pixelflinger.h"

void get_buffer(unsigned char *buffer);

#endif

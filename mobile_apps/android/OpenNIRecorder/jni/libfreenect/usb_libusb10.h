/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#ifndef USB_LIBUSB10
#define USB_LIBUSB10

#include "libusb.h"

#if defined(__APPLE__)
/*
  From Github Issue 22 by Roefer -
  https://github.com/OpenKinect/libfreenect/issues/#issue/22

  The current implementation still does not reach 30 Hz on MacOS. This
  is due to bad scheduling of USB transfers in libusb (Ed Note: libusb
  1.0.8). A fix can be found at
  http://www.informatik.uni-bremen.de/~roefer/libusb/libusb-osx-kinect.diff

  (Ed Note: patch applies to libusb repo at 7da756e09fd)

  In camera.c, I use PKTS_PER_XFER = 128, NUM_XFERS = 4. There are a
  few rules: PKTS_PER_XFER * NUM_XFERS <= 1000, PKTS_PER_XFER % 8 == 0.
*/
#define PKTS_PER_XFER 128
#define NUM_XFERS 4
#define DEPTH_PKTBUF 2048
#define VIDEO_PKTBUF 2048
#else
#ifdef _WIN32
  #define PKTS_PER_XFER 32
  #define NUM_XFERS 8
#else
  #define PKTS_PER_XFER 16
  #define NUM_XFERS 16
#endif
#define DEPTH_PKTBUF 1920
#define VIDEO_PKTBUF 1920
#endif

typedef struct {
	libusb_context *ctx;
	int should_free_ctx;
} fnusb_ctx;

typedef struct {
	freenect_device *parent; //so we can go up from the libusb userdata
	libusb_device_handle *dev;
} fnusb_dev;

typedef struct {
	fnusb_dev *parent; //so we can go up from the libusb userdata
	struct libusb_transfer **xfers;
	uint8_t *buffer;
	fnusb_iso_cb cb;
	int num_xfers;
	int pkts;
	int len;
	int dead;
	int dead_xfers;
} fnusb_isoc_stream;

int fnusb_num_devices(fnusb_ctx *ctx);

int fnusb_init(fnusb_ctx *ctx, freenect_usb_context *usb_ctx);
int fnusb_shutdown(fnusb_ctx *ctx);
int fnusb_process_events(fnusb_ctx *ctx);

int fnusb_open_subdevices(freenect_device *dev, int index);
int fnusb_close_subdevices(freenect_device *dev);

int fnusb_start_iso(fnusb_dev *dev, fnusb_isoc_stream *strm, fnusb_iso_cb cb, int ep, int xfers, int pkts, int len);
int fnusb_stop_iso(fnusb_dev *dev, fnusb_isoc_stream *strm);

int fnusb_control(fnusb_dev *dev, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data, uint16_t wLength);
#ifdef BUILD_AUDIO
int fnusb_bulk(fnusb_dev *dev, uint8_t endpoint, uint8_t *data, int len, int *transferred);
int fnusb_num_interfaces(fnusb_dev *dev);
#endif

#endif

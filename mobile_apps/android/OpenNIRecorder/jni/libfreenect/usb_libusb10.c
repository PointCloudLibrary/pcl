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

#include <android/log.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "libusb.h"
#include "freenect_internal.h"

int fnusb_num_devices(fnusb_ctx *ctx)
{
	libusb_device **devs; 
	//pointer to pointer of device, used to retrieve a list of devices	
	ssize_t cnt = libusb_get_device_list (ctx->ctx, &devs); 
	//get the list of devices	
	if (cnt < 0)
		return (-1);
	int nr = 0, i = 0;
	struct libusb_device_descriptor desc;
	for (i = 0; i < cnt; ++i)
	{
		int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;
		if (desc.idVendor == VID_MICROSOFT && desc.idProduct == PID_NUI_CAMERA){
			nr++;
		}
		/**
		 * <6>[102559.590977] usb 2-1: new high speed USB device using tegra-ehci and address 58
		<6>[102559.634467] usb 2-1: New USB device found, idVendor=1d27, idProduct=0600
		<6>[102559.641327] usb 2-1: New USB device strings: Mfr=2, Product=1, SerialNumber=0
		<6>[102559.648563] usb 2-1: Product: PrimeSense Device
		<6>[102559.653360] usb 2-1: Manufacturer: PrimeSense
		 * HACK FOR PRIMESENSE?
		 */
		else if(desc.idVendor==VID_ASUS && desc.idProduct == VID_ASUS_XTION_CAM){
			nr++;
		}
	}
	libusb_free_device_list (devs, 1);
	// free the list, unref the devices in it
	return nr;
}

int fnusb_init(fnusb_ctx *ctx, freenect_usb_context *usb_ctx)
{
	int res;
	if (!usb_ctx) {
		res = libusb_init(&ctx->ctx);
		if (res >= 0) {
			ctx->should_free_ctx = 1;
			return 0;
		} else {
			ctx->should_free_ctx = 0;
			ctx->ctx = NULL;
			return res;
		}
	} else {
    // explicit cast required: in WIN32, freenect_usb_context* maps to void*
    ctx->ctx = (libusb_context*)usb_ctx;
		ctx->should_free_ctx = 0;
		return 0;
	}
}

int fnusb_shutdown(fnusb_ctx *ctx)
{
	//int res;
	if (ctx->should_free_ctx) {
		libusb_exit(ctx->ctx);
		ctx->ctx = NULL;
	}
	return 0;
}

int fnusb_process_events(fnusb_ctx *ctx)
{
	return libusb_handle_events(ctx->ctx);
}

int fnusb_open_subdevices(freenect_device *dev, int index)
{
	freenect_context *ctx = dev->parent;

	dev->usb_cam.parent = dev;
	dev->usb_cam.dev = NULL;
	dev->usb_motor.parent = dev;
	dev->usb_motor.dev = NULL;
#ifdef BUILD_AUDIO
	dev->usb_audio.parent = dev;
	dev->usb_audio.dev = NULL;
#endif

	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt = libusb_get_device_list (dev->parent->usb.ctx, &devs); //get the list of devices
	if (cnt < 0){
		__android_log_write(ANDROID_LOG_INFO, "Kinect","libusb_get_device_list is not working\n");
		return -1;
	}
	int i = 0, nr_cam = 0, nr_mot = 0;
#ifdef BUILD_AUDIO
	int nr_audio = 0;
#endif
	int res;
	struct libusb_device_descriptor desc;

	for (i = 0; i < cnt; i++) {
		int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;
		char buf[512];
		sprintf(buf, "device descriptor %d vs %d", VID_MICROSOFT, desc.idVendor);
		__android_log_write(ANDROID_LOG_INFO, "Kinect", buf);

		//hack for PRIMESENSE
		if (desc.idVendor != VID_MICROSOFT && desc.idVendor != VID_ASUS)
			continue;

		sprintf(buf, "camera device descriptor %d vs %d | %d vs %d", VID_MICROSOFT, desc.idVendor, desc.idProduct, PID_NUI_CAMERA);
		__android_log_write(ANDROID_LOG_INFO, "Kinect", buf);
		// Search for the camera
		if ((ctx->enabled_subdevices & FREENECT_DEVICE_CAMERA) && !dev->usb_cam.dev && desc.idProduct == PID_NUI_CAMERA) {
			// If the index given by the user matches our camera index
			if (nr_cam == index) {
				res = libusb_open (devs[i], &dev->usb_cam.dev);
				if (res < 0 || !dev->usb_cam.dev) {
					FN_ERROR("Could not open camera: %d\n", res);
					__android_log_write(ANDROID_LOG_INFO, "Kinect","cannot open camera\n");
					dev->usb_cam.dev = NULL;
					break;
				}
#ifndef _WIN32
				// Detach an existing kernel driver for the device
				res = libusb_kernel_driver_active(dev->usb_cam.dev, 0);
				if (res == 1) {
					res = libusb_detach_kernel_driver(dev->usb_cam.dev, 0);
					if (res < 0) {
						FN_ERROR("Could not detach kernel driver for camera: %d\n", res);
						__android_log_write(ANDROID_LOG_INFO, "Kinect","Could not detach kernel driver for camera\n");
						libusb_close(dev->usb_cam.dev);
						dev->usb_cam.dev = NULL;
						break;
					}
				}
#endif
				res = libusb_claim_interface (dev->usb_cam.dev, 0);
				if (res < 0) {
					FN_ERROR("Could not claim interface on camera: %d\n", res);
					__android_log_write(ANDROID_LOG_INFO, "Kinect","Could not claim interface on camera\n");

					libusb_close(dev->usb_cam.dev);
					dev->usb_cam.dev = NULL;
					break;
				}
			} else {
				nr_cam++;
			}
		}
		// Search for the XTION camera
		if ((ctx->enabled_subdevices & FREENECT_DEVICE_CAMERA) && !dev->usb_cam.dev && desc.idProduct == VID_ASUS_XTION_CAM) {
			// If the index given by the user matches our camera index
			if (nr_cam == index) {
				res = libusb_open (devs[i], &dev->usb_cam.dev);
				if (res < 0 || !dev->usb_cam.dev) {
					FN_ERROR("Could not open camera: %d\n", res);
					__android_log_write(ANDROID_LOG_INFO, "Kinect","cannot open camera\n");
					dev->usb_cam.dev = NULL;
					break;
				}
				res = libusb_claim_interface (dev->usb_cam.dev, 0);
				if (res < 0) {
					FN_ERROR("Could not claim interface on camera: %d\n", res);
					__android_log_write(ANDROID_LOG_INFO, "Kinect","Could not claim interface on camera\n");

					libusb_close(dev->usb_cam.dev);
					dev->usb_cam.dev = NULL;
					break;
				}
			} else {
				nr_cam++;
			}
		}
		// Search for the motor
		if ((ctx->enabled_subdevices & FREENECT_DEVICE_MOTOR) && !dev->usb_motor.dev && desc.idProduct == PID_NUI_MOTOR) {
			// If the index given by the user matches our camera index
			if (nr_mot == index) {
				res = libusb_open (devs[i], &dev->usb_motor.dev);
				if (res < 0 || !dev->usb_motor.dev) {
					FN_ERROR("Could not open motor: %d\n", res);
					__android_log_write(ANDROID_LOG_INFO, "Kinect","Could not open motor\n");
					dev->usb_motor.dev = NULL;
					break;
				}
				res = libusb_claim_interface (dev->usb_motor.dev, 0);
				if (res < 0) {
					FN_ERROR("Could not claim interface on motor: %d\n", res);
					__android_log_write(ANDROID_LOG_INFO, "Kinect","Could not claim interface on motor\n");

					libusb_close(dev->usb_motor.dev);
					dev->usb_motor.dev = NULL;
					break;
				}
			} else {
				nr_mot++;
			}
		}

#ifdef BUILD_AUDIO
		// TODO: check that the firmware has already been loaded; if not, upload firmware.
		// Search for the audio
		if ((ctx->enabled_subdevices & FREENECT_DEVICE_AUDIO) && !dev->usb_audio.dev && desc.idProduct == PID_NUI_AUDIO) {
			// If the index given by the user matches our audio index
			if (nr_audio == index) {
				res = libusb_open (devs[i], &dev->usb_audio.dev);
				if (res < 0 || !dev->usb_audio.dev) {
					FN_ERROR("Could not open audio: %d\n", res);
					__android_log_write(ANDROID_LOG_INFO, "Kinect","Could not open audio\n");
					dev->usb_audio.dev = NULL;
					break;
				}
				res = libusb_claim_interface (dev->usb_audio.dev, 0);
				if (res < 0) {
					FN_ERROR("Could not claim interface on audio: %d\n", res);
					__android_log_write(ANDROID_LOG_INFO, "Kinect","Could not claim interface on audio\n");

					libusb_close(dev->usb_audio.dev);
					dev->usb_audio.dev = NULL;
					break;
				}
			} else {
				nr_audio++;
			}
		}
#endif

	}

	libusb_free_device_list (devs, 1);  // free the list, unref the devices in it

	// Check that each subdevice is either opened or not enabled.
	if ( (dev->usb_cam.dev || !(ctx->enabled_subdevices & FREENECT_DEVICE_CAMERA))
		&& (dev->usb_motor.dev || !(ctx->enabled_subdevices & FREENECT_DEVICE_MOTOR))
#ifdef BUILD_AUDIO
		&& (dev->usb_audio.dev || !(ctx->enabled_subdevices & FREENECT_DEVICE_AUDIO))
#endif
		) {
		return 0;
	} else {
		if (dev->usb_cam.dev) {
			libusb_release_interface(dev->usb_cam.dev, 0);
			libusb_close(dev->usb_cam.dev);
		}
		if (dev->usb_motor.dev) {
			libusb_release_interface(dev->usb_motor.dev, 0);
			libusb_close(dev->usb_motor.dev);
		}
#ifdef BUILD_AUDIO
		if (dev->usb_audio.dev) {
			libusb_release_interface(dev->usb_audio.dev, 0);
			libusb_close(dev->usb_audio.dev);
		}
#endif
		return -1;
	}
}

int fnusb_close_subdevices(freenect_device *dev)
{
	if (dev->usb_cam.dev) {
		libusb_release_interface(dev->usb_cam.dev, 0);
#ifndef _WIN32
		libusb_attach_kernel_driver(dev->usb_cam.dev, 0);
#endif
		libusb_close(dev->usb_cam.dev);
		dev->usb_cam.dev = NULL;
	}
	if (dev->usb_motor.dev) {
		libusb_release_interface(dev->usb_motor.dev, 0);
		libusb_close(dev->usb_motor.dev);
		dev->usb_motor.dev = NULL;
	}
#ifdef BUILD_AUDIO
	if (dev->usb_audio.dev) {
		libusb_release_interface(dev->usb_audio.dev, 0);
		libusb_close(dev->usb_audio.dev);
		dev->usb_audio.dev = NULL;
	}
#endif
	return 0;
}

static void iso_callback(struct libusb_transfer *xfer)
{
	int i;
	fnusb_isoc_stream *strm = (fnusb_isoc_stream*)xfer->user_data;

	if (strm->dead) {
		freenect_context *ctx = strm->parent->parent->parent;
		strm->dead_xfers++;
		FN_SPEW("EP %02x transfer complete, %d left\n", xfer->endpoint, strm->num_xfers - strm->dead_xfers);
		return;
	}

	switch(xfer->status) {
		case LIBUSB_TRANSFER_COMPLETED: // Normal operation.
		{
			//char sbuf[512];
			//sprintf(sbuf,"Transfer Completed %d\n", xfer->iso_packet_desc[0].actual_length);
			//__android_log_write(ANDROID_LOG_INFO, "Xtion", sbuf);
			uint8_t *buf = (uint8_t*)xfer->buffer;
			for (i=0; i<strm->pkts; i++) {
				strm->cb(strm->parent->parent, buf, xfer->iso_packet_desc[i].actual_length);
				buf += strm->len;
			}
			libusb_submit_transfer(xfer);
			break;
		}
		case LIBUSB_TRANSFER_NO_DEVICE:
		{
			// We lost the device we were talking to.  This is a large problem,
			// and one that we should eventually come up with a way to
			// properly propagate up to the caller.
			freenect_context *ctx = strm->parent->parent->parent;
			FN_ERROR("USB device disappeared, cancelling stream :(\n");
			strm->dead_xfers++;
			fnusb_stop_iso(strm->parent, strm);
			break;
		}
		case LIBUSB_TRANSFER_CANCELLED:
		{
			freenect_context *ctx = strm->parent->parent->parent;
			FN_SPEW("EP %02x transfer cancelled\n", xfer->endpoint);
			strm->dead_xfers++;
			break;
		}
		default:
		{
			// On other errors, resubmit the transfer - in particular, libusb
			// on OSX tends to hit random errors a lot.  If we don't resubmit
			// the transfers, eventually all of them die and then we don't get
			// any more data from the Kinect.
			freenect_context *ctx = strm->parent->parent->parent;
			FN_WARNING("Isochronous transfer error: %d\n", xfer->status);
			libusb_submit_transfer(xfer);
			break;
		}
	}
}

int fnusb_start_iso(fnusb_dev *dev, fnusb_isoc_stream *strm, fnusb_iso_cb cb, int ep, int xfers, int pkts, int len)
{

	char buf[512];
	sprintf(buf,"Starting ISO %d %d", ep, len);
	__android_log_write(ANDROID_LOG_INFO, "Xtion", buf);

	freenect_context *ctx = dev->parent->parent;
	int ret, i;

	strm->parent = dev;
	strm->cb = cb;
	strm->num_xfers = xfers;
	strm->pkts = pkts;
	strm->len = len;
	strm->buffer = (uint8_t*)malloc(xfers * pkts * len);
	strm->xfers = (struct libusb_transfer**)malloc(sizeof(struct libusb_transfer*) * xfers);
	strm->dead = 0;
	strm->dead_xfers = 0;

	uint8_t *bufp = strm->buffer;

	for (i=0; i<xfers; i++) {
		char buf[512];
		sprintf(buf,"Creating EP %02x transfer #%d\n", ep, i);
		__android_log_write(ANDROID_LOG_INFO, "Xtion", buf);
		FN_SPEW("Creating EP %02x transfer #%d\n", ep, i);
		strm->xfers[i] = libusb_alloc_transfer(pkts);

		libusb_fill_iso_transfer(strm->xfers[i], dev->dev, ep, bufp, pkts * len, pkts, iso_callback, strm, 0);

		libusb_set_iso_packet_lengths(strm->xfers[i], len);

		ret = libusb_submit_transfer(strm->xfers[i]);
		if (ret < 0) {
			FN_WARNING("Failed to submit isochronous transfer %d: %d\n", i, ret);
			sprintf(buf,"Failed to submit isochronous transfer %d: %d\n", i, ret);
			__android_log_write(ANDROID_LOG_INFO, "Xtion", buf);
			strm->dead_xfers++;
		}

		bufp += pkts*len;
	}

	return 0;

}

int fnusb_stop_iso(fnusb_dev *dev, fnusb_isoc_stream *strm)
{
	freenect_context *ctx = dev->parent->parent;
	int i;

	strm->dead = 1;

	for (i=0; i<strm->num_xfers; i++)
		libusb_cancel_transfer(strm->xfers[i]);

	while (strm->dead_xfers < strm->num_xfers) {
		libusb_handle_events(ctx->usb.ctx);
	}

	for (i=0; i<strm->num_xfers; i++)
		libusb_free_transfer(strm->xfers[i]);

	free(strm->buffer);
	free(strm->xfers);

	memset(strm, 0, sizeof(*strm));
	return 0;
}

int fnusb_control(fnusb_dev *dev, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data, uint16_t wLength)
{
	return libusb_control_transfer(dev->dev, bmRequestType, bRequest, wValue, wIndex, data, wLength, 0);
}

#ifdef BUILD_AUDIO
int fnusb_bulk(fnusb_dev *dev, uint8_t endpoint, uint8_t *data, int len, int *transferred) {
	return libusb_bulk_transfer(dev->dev, endpoint, data, len, transferred, 0);
}

int fnusb_num_interfaces(fnusb_dev *dev) {
	int retval = 0;
	int res;
	libusb_device* d = libusb_get_device(dev->dev);
	struct libusb_config_descriptor* config;
	res = libusb_get_active_config_descriptor(d, &config);
	if (res < 0) // Something went wrong
		return res;
	retval = config->bNumInterfaces;
	libusb_free_config_descriptor(config);
	return retval;
}
#endif

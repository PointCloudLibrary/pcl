/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
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

#ifndef __LOADER_H__
#define __LOADER_H__

#include <stdint.h>
#include "usb_libusb10.h"

typedef struct {
	uint32_t magic;
	uint32_t tag;
	uint32_t bytes;
	uint32_t cmd;
	uint32_t addr;
	uint32_t unk;
} bootloader_command;

typedef struct {
	uint32_t magic;
	uint32_t tag;
	uint32_t arg1; // initial command: 0.  Firmware blocks: byte count.
	uint32_t cmd;
	uint32_t arg2; // initial command: byte count.  Firmware blocks: target address.
	uint32_t zeros[8];
} cemdloader_command;

typedef struct {
	uint32_t magic;
	uint32_t tag;
	uint32_t status;
} bootloader_status_code;

int upload_firmware(fnusb_dev* dev);
int upload_cemd_data(fnusb_dev* dev);

#endif //__LOADER_H__

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

#include "libfreenect.h"
#include "freenect_internal.h"
#include "loader.h"
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

static void dump_bl_cmd(freenect_context* ctx, bootloader_command cmd) {
	int i;
	for(i = 0; i < 24; i++)
		FN_INFO("%02X ", ((unsigned char*)(&cmd))[i]);
	FN_INFO("\n");
}

static void dump_cemd_cmd(freenect_context* ctx, cemdloader_command cmd) {
	int i;
	for(i = 0; i < 24; i++)
		FN_INFO("%02X ", ((unsigned char*)(&cmd))[i]);
	FN_INFO("(%d more zeros)\n", (int)(sizeof(cmd)-24));
}

static int get_reply(fnusb_dev* dev) {
	freenect_context* ctx = dev->parent->parent;
	unsigned char dump[512];
	bootloader_status_code buffer = ((bootloader_status_code*)dump)[0];
	int res;
	int transferred;
	res = fnusb_bulk(dev, 0x81, (unsigned char*)&buffer, 512, &transferred);
	if(res != 0 || transferred != sizeof(bootloader_status_code)) {
		FN_ERROR("Error reading reply: %d\ttransferred: %d (expected %d)\n", res, transferred, (int)(sizeof(bootloader_status_code)));
		return res;
	}
	if(fn_le32(buffer.magic) != 0x0a6fe000) {
		FN_ERROR("Error reading reply: invalid magic %08X\n",buffer.magic);
		return -1;
	}
	if(fn_le32(buffer.tag) != dev->parent->audio_tag) {
		FN_ERROR("Error reading reply: non-matching tag number %08X (expected %08X)\n", buffer.tag, dev->parent->audio_tag);
		return -1;
	}
	if(fn_le32(buffer.status) != 0) {
		FN_ERROR("Notice reading reply: last uint32_t was nonzero: %d\n", buffer.status);
	}
	FN_INFO("Reading reply: ");
	int i;
	for(i = 0; i < transferred; ++i) {
		FN_INFO("%02X ", ((unsigned char*)(&buffer))[i]);
	}
	FN_INFO("\n");
	return res;
}

static int check_version_string(fnusb_dev* dev) {
	freenect_context* ctx = dev->parent->parent;
	bootloader_command bootcmd;
	memset(&bootcmd, 0, sizeof(bootcmd));
	bootcmd.magic = fn_le32(0x06022009);
	bootcmd.tag   = fn_le32(dev->parent->audio_tag);
	bootcmd.bytes = fn_le32(0x60);
	bootcmd.cmd   = fn_le32(0);
	bootcmd.addr  = fn_le32(0x15);
	unsigned char buffer[512];
	int res;
	int transferred;

	FN_INFO("check_version_string(): About to send: ");
	dump_bl_cmd(ctx, bootcmd);

	// Send "get version string" command
	res = fnusb_bulk(dev, 1, (unsigned char*)&bootcmd, sizeof(bootcmd), &transferred);
	if(res != 0 || transferred != sizeof(bootcmd)) {
		FN_ERROR("Error: res: %d\ttransferred: %d (expected %d)\n",res, transferred, sizeof(bootcmd));
		return -1;
	}

	// Read version string reply
	res = fnusb_bulk(dev, 0x81, buffer, 512, &transferred);
	if(res != 0 ) {
		FN_ERROR("Error reading version string: %d\ttransferred: %d (expected %d)\n", res, transferred, 0x60);
		return res;
	}
	FN_INFO("Read version string: ");
	int i;
	for(i = 0; i < transferred; ++i) {
		FN_INFO("%02X ", buffer[i]);
	}
	FN_INFO("\n");

	// Read status code reply
	res = get_reply(dev);
	dev->parent->audio_tag++;
	return res;
}

int upload_firmware(fnusb_dev* dev) {
	freenect_context* ctx = dev->parent->parent;
	bootloader_command bootcmd;
	memset(&bootcmd, 0, sizeof(bootcmd));
	bootcmd.magic = fn_le32(0x06022009);

	int res;
	int transferred;

	/* Search for firmware file (audios.bin) in the following places:
	 * $LIBFREENECT_FIRMWARE_PATH
	 * .
	 * ${HOME}/.libfreenect
	 * /usr/local/share/libfreenect
	 * /usr/share/libfreenect
	 */
	const char* fw_filename = "/audios.bin";
	int filenamelen = strlen(fw_filename);
	int i;
	int searchpathcount;
	FILE* fw = NULL;
	for(i = 0, searchpathcount = 5; !fw && i < searchpathcount; i++) {
		char* fwfile;
		int needs_free = 0;
		switch(i) {
			case 0: {
				char* envpath = getenv("LIBFREENECT_FIRMWARE_PATH");
				if (!envpath)
					continue;
				int pathlen = strlen(envpath);
				fwfile = malloc(pathlen + filenamelen + 1);
				strcpy(fwfile, envpath);
				strcat(fwfile, fw_filename);
				needs_free = 1;
				}
				break;
			case 1:
				fwfile = "./audios.bin";
				break;
			case 2: {
				// Construct $HOME/.libfreenect/
				char* home = getenv("HOME");
				if (!home)
					continue;
				int homelen = strlen(home);
				char* dotfolder = "/.libfreenect";
				int locallen = strlen(dotfolder);
				fwfile = (char*)malloc(homelen + locallen + filenamelen + 1);
				strcpy(fwfile, home);
				strcat(fwfile, dotfolder);
				strcat(fwfile, fw_filename);
				needs_free = 1;
				}
				break;
			case 3:
				fwfile = "/usr/local/share/libfreenect/audios.bin";
				break;
			case 4:
				fwfile = "/usr/share/libfreenect/audios.bin";
				break;
			default: break;
		}
		FN_INFO("Trying to open %s as firmware...\n", fwfile);
		fw = fopen(fwfile, "rb");
		if (needs_free) {
			free(fwfile);
		}
	}
	if (!fw) {
		FN_ERROR("upload_firmware: failed to find firmware file.\n");
		return -errno;
	}
	// Now we have an open firmware file handle.
	uint32_t addr = 0x00080000;
	int read;
	unsigned char page[0x4000];
	do {
		read = fread(page, 1, 0x4000, fw);
		if(read <= 0) {
			break;
		}
		bootcmd.tag = fn_le32(dev->parent->audio_tag);
		bootcmd.bytes = fn_le32(read);
		bootcmd.cmd = fn_le32(0x03);
		bootcmd.addr = fn_le32(addr);
		FN_INFO("About to send: ");
		dump_bl_cmd(ctx, bootcmd);
		// Send it off!
		res = fnusb_bulk(dev, 1, (unsigned char*)&bootcmd, sizeof(bootcmd), &transferred);
		if(res != 0 || transferred != sizeof(bootcmd)) {
			FN_ERROR("upload_firmware(): Error: res: %d\ttransferred: %d (expected %d)\n",res, transferred, (int)(sizeof(bootcmd)));
			fclose(fw);
			return -1;
		}
		int bytes_sent = 0;
		while(bytes_sent < read) {
			int to_send = (read - bytes_sent > 512 ? 512 : read - bytes_sent);
			res = fnusb_bulk(dev, 1, &page[bytes_sent], to_send, &transferred);
			if(res != 0 || transferred != to_send) {
				FN_ERROR("upload_firmware(): Error: res: %d\ttransferred: %d (expected %d)\n",res, transferred, to_send);
				fclose(fw);
				return -1;
			}
			bytes_sent += to_send;
		}
		res = get_reply(dev);
		addr += (uint32_t)read;
		dev->parent->audio_tag++;
	} while (read > 0);
	fclose(fw);
	fw = NULL;

	bootcmd.tag   = fn_le32(dev->parent->audio_tag);
	bootcmd.bytes = fn_le32(0);
	bootcmd.cmd   = fn_le32(0x04);
	bootcmd.addr  = fn_le32(0x00080030);
	dump_bl_cmd(ctx, bootcmd);
	res = fnusb_bulk(dev, 1, (unsigned char*)&bootcmd, sizeof(bootcmd), &transferred);
	if(res != 0 || transferred != sizeof(bootcmd)) {
		FN_ERROR("upload_firmware(): Error: res: %d\ttransferred: %d (expected %d)\n", res, transferred, (int)sizeof(bootcmd));
		return -1;
	}
	res = get_reply(dev);
	dev->parent->audio_tag++;
	FN_INFO("Firmware successfully uploaded and launched.  Device will disconnect and reenumerate.\n");
	return 0;
}

int upload_cemd_data(fnusb_dev* dev) {
	// Now we upload the CEMD data.
	freenect_context* ctx = dev->parent->parent;
	cemdloader_command cemdcmd;
	memset(&cemdcmd, 0, sizeof(cemdcmd));
	cemdcmd.magic = fn_le32(0x06022009);
	cemdcmd.tag   = fn_le32(dev->parent->audio_tag);
	cemdcmd.arg1  = fn_le32(0);
	cemdcmd.cmd   = fn_le32(0x00000133);
	cemdcmd.arg2  = fn_le32(0x00064014); // This is the length of the CEMD data.
	FN_INFO("Starting CEMD data upload:\n");
	int res;
	int transferred;
	res = fnusb_bulk(dev, 1, (unsigned char*)&cemdcmd, sizeof(cemdcmd), &transferred);
	if(res != 0 || transferred != sizeof(cemdcmd)) {
		FN_ERROR("Error: res: %d\ttransferred: %d (expected %d)\n", res, transferred, (int)sizeof(cemdcmd));
		return -1;
	}
	res = get_reply(dev);
	dev->parent->audio_tag++;

	const char* cemd_filename = "cemd_data.bin";
	FILE* cf = fopen(cemd_filename, "r");
	if(cf == NULL) {
		FN_ERROR("upload_cemd_data: Failed to open %s: error %d", cemd_filename, errno);
		return errno;
	}
	uint32_t addr = 0x00000000;
	int read = 0;
	unsigned char page[0x4000];
	do {
		read = fread(page, 1, 0x4000, cf);
		if(read <= 0) {
			break;
		}
		//LOG("");
		cemdcmd.tag  = fn_le32(dev->parent->audio_tag);
		cemdcmd.arg1 = fn_le32(read);
		cemdcmd.cmd  = fn_le32(0x134);
		cemdcmd.arg2 = fn_le32(addr);
		FN_INFO("About to send: ");
		dump_cemd_cmd(ctx, cemdcmd);
		// Send it off!
		res = fnusb_bulk(dev, 1, (unsigned char*)&cemdcmd, sizeof(cemdcmd), &transferred);
		if(res != 0 || transferred != sizeof(cemdcmd)) {
			FN_ERROR("Error: res: %d\ttransferred: %d (expected %d)\n",res, transferred, (int)sizeof(cemdcmd));
			return -1;
		}
		int bytes_sent = 0;
		while(bytes_sent < read) {
			int to_send = (read - bytes_sent > 512 ? 512 : read - bytes_sent);
			res = fnusb_bulk(dev, 1, &page[bytes_sent], to_send, &transferred);
			if(res != 0 || transferred != to_send) {
				FN_ERROR("Error: res: %d\ttransferred: %d (expected %d)\n",res, transferred, to_send);
				return -1;
			}
			bytes_sent += to_send;
		}
		res = get_reply(dev);
		addr += (uint32_t)read;
		dev->parent->audio_tag++;
	} while (read > 0);
	fclose(cf);
	cf = NULL;

	cemdcmd.tag  = fn_le32(dev->parent->audio_tag);
	cemdcmd.arg1 = fn_le32(0); // bytes = 0
	cemdcmd.cmd  = fn_le32(0x135);
	cemdcmd.arg2 = fn_le32(0x00064000); // mimicing the USB logs.  This is the # of bytes of actual CEMD data after the 20-byte CEMD header.
	FN_INFO("Finishing CEMD data upload...\n");
	res = fnusb_bulk(dev, 1, (unsigned char*)&cemdcmd, sizeof(cemdcmd), &transferred);
	if(res != 0 || transferred != sizeof(cemdcmd)) {
		FN_ERROR("upload_cemd_data(): Error: res: %d\ttransferred: %d (expected %d)\n", res, transferred, (int)sizeof(cemdcmd));
		return -1;
	}
	res = get_reply(dev);
	dev->parent->audio_tag++;
	FN_INFO("CEMD data uploaded successfully.\n");
	return 0;
}

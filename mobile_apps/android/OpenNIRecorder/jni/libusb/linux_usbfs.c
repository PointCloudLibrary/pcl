/*
 * Linux usbfs backend for libusb
 * Copyright (C) 2007-2008 Daniel Drake <dsd@gentoo.org>
 * Copyright (c) 2001 Johannes Erdfelt <johannes@erdfelt.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <android/log.h>
#include <config.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "libusb.h"
#include "libusbi.h"
#include "linux_usbfs.h"

/* sysfs vs usbfs:
 * opening a usbfs node causes the device to be resumed, so we attempt to
 * avoid this during enumeration.
 *
 * sysfs allows us to read the kernel's in-memory copies of device descriptors
 * and so forth, avoiding the need to open the device:
 *  - The binary "descriptors" file was added in 2.6.23.
 *  - The "busnum" file was added in 2.6.22
 *  - The "devnum" file has been present since pre-2.6.18
 *  - the "bConfigurationValue" file has been present since pre-2.6.18
 *
 * If we have bConfigurationValue, busnum, and devnum, then we can determine
 * the active configuration without having to open the usbfs node in RDWR mode.
 * We assume this is the case if we see the busnum file (indicates 2.6.22+).
 * The busnum file is important as that is the only way we can relate sysfs
 * devices to usbfs nodes.
 *
 * If we also have descriptors, we can obtain the device descriptor and active 
 * configuration without touching usbfs at all.
 *
 * The descriptors file originally only contained the active configuration
 * descriptor alongside the device descriptor, but all configurations are
 * included as of Linux 2.6.26.
 */

static const char *usbfs_path = NULL;

/* do we have a busnum to relate devices? this also implies that we can read
 * the active configuration through bConfigurationValue */
static int sysfs_can_relate_devices = -1;

/* do we have a descriptors file? */
static int sysfs_has_descriptors = -1;

struct linux_device_priv {
	char *sysfs_dir;
	unsigned char *dev_descriptor;
	unsigned char *config_descriptor;
};

struct linux_device_handle_priv {
	int fd;
};

enum reap_action {
	NORMAL = 0,
	/* submission failed after the first URB, so await cancellation/completion
	 * of all the others */
	SUBMIT_FAILED,

	/* cancelled by user or timeout */
	CANCELLED,

	/* completed multi-URB transfer in non-final URB */
	COMPLETED_EARLY,
};

struct linux_transfer_priv {
	union {
		struct usbfs_urb *urbs;
		struct usbfs_urb **iso_urbs;
	};

	enum reap_action reap_action;
	int num_urbs;
	unsigned int awaiting_reap;
	unsigned int awaiting_discard;

	/* next iso packet in user-supplied transfer to be populated */
	int iso_packet_offset;
};

static void __get_usbfs_path(struct libusb_device *dev, char *path)
{
	snprintf(path, PATH_MAX, "%s/%03d/%03d", usbfs_path, dev->bus_number,
		dev->device_address);
}

static struct linux_device_priv *__device_priv(struct libusb_device *dev)
{
	return (struct linux_device_priv *) dev->os_priv;
}

static struct linux_device_handle_priv *__device_handle_priv(
	struct libusb_device_handle *handle)
{
	return (struct linux_device_handle_priv *) handle->os_priv;
}

static int check_usb_vfs(const char *dirname)
{
	DIR *dir;
	struct dirent *entry;
	int found = 0;

	dir = opendir(dirname);
	if (!dir)
		return 0;

	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_name[0] == '.')
			continue;

		/* We assume if we find any files that it must be the right place */
		found = 1;
		break;
	}

	closedir(dir);
	return found;
}

static const char *find_usbfs_path(void)
{
	const char *path = "/dev/bus/usb";
	const char *ret = NULL;

	if (check_usb_vfs(path)) {
		ret = path;
	} else {
		path = "/proc/bus/usb";
		if (check_usb_vfs(path))
			ret = path;
	}

	usbi_dbg("found usbfs at %s", ret);
	return ret;
}

static int op_init(struct libusb_context *ctx)
{
	struct stat statbuf;
	int r;

	usbfs_path = find_usbfs_path();
	if (!usbfs_path) {
		usbi_err(ctx, "could not find usbfs");
		return LIBUSB_ERROR_OTHER;
	}

	r = stat(SYSFS_DEVICE_PATH, &statbuf);
	if (r == 0 && S_ISDIR(statbuf.st_mode)) {
		usbi_dbg("found usb devices in sysfs");
	} else {
		usbi_dbg("sysfs usb info not available");
		sysfs_has_descriptors = 0;
		sysfs_can_relate_devices = 0;
	}

	return 0;
}

static int usbfs_get_device_descriptor(struct libusb_device *dev,
	unsigned char *buffer)
{
	struct linux_device_priv *priv = __device_priv(dev);

	/* return cached copy */
	memcpy(buffer, priv->dev_descriptor, DEVICE_DESC_LENGTH);
	return 0;
}

static int __open_sysfs_attr(struct libusb_device *dev, const char *attr)
{
	struct linux_device_priv *priv = __device_priv(dev);
	char filename[PATH_MAX];
	int fd;

	snprintf(filename, PATH_MAX, "%s/%s/%s",
		SYSFS_DEVICE_PATH, priv->sysfs_dir, attr);
	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		usbi_err(DEVICE_CTX(dev),
			"open %s failed ret=%d errno=%d", filename, fd, errno);
		return LIBUSB_ERROR_IO;
	}

	return fd;
}

static int sysfs_get_device_descriptor(struct libusb_device *dev,
	unsigned char *buffer)
{
	int fd;
	ssize_t r;

	/* sysfs provides access to an in-memory copy of the device descriptor,
	 * so we use that rather than keeping our own copy */

	fd = __open_sysfs_attr(dev, "descriptors");
	if (fd < 0)
		return fd;

	r = read(fd, buffer, DEVICE_DESC_LENGTH);;
	close(fd);
	if (r < 0) {
		usbi_err(DEVICE_CTX(dev), "read failed, ret=%d errno=%d", fd, errno);
		return LIBUSB_ERROR_IO;
	} else if (r < DEVICE_DESC_LENGTH) {
		usbi_err(DEVICE_CTX(dev), "short read %d/%d", r, DEVICE_DESC_LENGTH);
		return LIBUSB_ERROR_IO;
	}

	return 0;
}

static int op_get_device_descriptor(struct libusb_device *dev,
	unsigned char *buffer, int *host_endian)
{
	if (sysfs_has_descriptors) {
		return sysfs_get_device_descriptor(dev, buffer);
	} else {
		*host_endian = 1;
		return usbfs_get_device_descriptor(dev, buffer);
	}
}

static int usbfs_get_active_config_descriptor(struct libusb_device *dev,
	unsigned char *buffer, size_t len)
{
	struct linux_device_priv *priv = __device_priv(dev);
	if (!priv->config_descriptor)
		return LIBUSB_ERROR_NOT_FOUND; /* device is unconfigured */

	/* retrieve cached copy */
	memcpy(buffer, priv->config_descriptor, len);
	return 0;
}

/* read the bConfigurationValue for a device */
static int sysfs_get_active_config(struct libusb_device *dev, int *config)
{
	char *endptr;
	char tmp[4] = {0, 0, 0, 0};
	long num;
	int fd;
	size_t r;

	fd = __open_sysfs_attr(dev, "bConfigurationValue");
	if (fd < 0)
		return fd;

	r = read(fd, tmp, sizeof(tmp));
	close(fd);
	if (r < 0) {
		usbi_err(DEVICE_CTX(dev), 
			"read bConfigurationValue failed ret=%d errno=%d", r, errno);
		return LIBUSB_ERROR_IO;
	} else if (r == 0) {
		usbi_err(DEVICE_CTX(dev), "device unconfigured");
		*config = -1;
		return 0;
	}

	if (tmp[sizeof(tmp) - 1] != 0) {
		usbi_err(DEVICE_CTX(dev), "not null-terminated?");
		return LIBUSB_ERROR_IO;
	} else if (tmp[0] == 0) {
		usbi_err(DEVICE_CTX(dev), "no configuration value?");
		return LIBUSB_ERROR_IO;
	}

	num = strtol(tmp, &endptr, 10);
	if (endptr == tmp) {
		usbi_err(DEVICE_CTX(dev), "error converting '%s' to integer", tmp);
		return LIBUSB_ERROR_IO;
	}

	*config = (int) num;
	return 0;
}

/* takes a usbfs/descriptors fd seeked to the start of a configuration, and
 * seeks to the next one. */
static int seek_to_next_config(struct libusb_context *ctx, int fd)
{
	struct libusb_config_descriptor config;
	unsigned char tmp[6];
	off_t off;
	int r;

	/* read first 6 bytes of descriptor */
	r = read(fd, tmp, sizeof(tmp));
	if (r < 0) {
		usbi_err(ctx, "read failed ret=%d errno=%d", r, errno);
		return LIBUSB_ERROR_IO;
	} else if (r < sizeof(tmp)) {
		usbi_err(ctx, "short descriptor read %d/%d", r, sizeof(tmp));
		return LIBUSB_ERROR_IO;
	}

	/* seek forward to end of config */
	usbi_parse_descriptor(tmp, "bbwbb", &config, 1);
	off = lseek(fd, config.wTotalLength - sizeof(tmp), SEEK_CUR);
	if (off < 0) {
		usbi_err(ctx, "seek failed ret=%d errno=%d", off, errno);
		return LIBUSB_ERROR_IO;
	}

	return 0;
}

static int sysfs_get_active_config_descriptor(struct libusb_device *dev,
	unsigned char *buffer, size_t len)
{
	int fd;
	ssize_t r;
	off_t off;
	int to_copy;
	int config;
	unsigned char tmp[6];

	r = sysfs_get_active_config(dev, &config);
	if (r < 0)
		return r;
	if (config == -1)
		return LIBUSB_ERROR_NOT_FOUND;

	usbi_dbg("active configuration %d", config);

	/* sysfs provides access to an in-memory copy of the device descriptor,
	 * so we use that rather than keeping our own copy */

	fd = __open_sysfs_attr(dev, "descriptors");
	if (fd < 0)
		return fd;

	/* device might have been unconfigured since we read bConfigurationValue,
	 * so first check that there is any config descriptor data at all... */
	off = lseek(fd, 0, SEEK_END);
	if (off < 1) {
		usbi_err(DEVICE_CTX(dev), "end seek failed, ret=%d errno=%d",
			off, errno);
		close(fd);
		return LIBUSB_ERROR_IO;
	} else if (off == DEVICE_DESC_LENGTH) {
		close(fd);
		return LIBUSB_ERROR_NOT_FOUND;
	}

	off = lseek(fd, DEVICE_DESC_LENGTH, SEEK_SET);
	if (off < 0) {
		usbi_err(DEVICE_CTX(dev), "seek failed, ret=%d errno=%d", off, errno);
		close(fd);
		return LIBUSB_ERROR_IO;
	}

	/* unbounded loop: we expect the descriptor to be present under all
	 * circumstances */
	while (1) {
		r = read(fd, tmp, sizeof(tmp));
		if (r < 0) {
			usbi_err(DEVICE_CTX(dev), "read failed, ret=%d errno=%d",
				fd, errno);
			return LIBUSB_ERROR_IO;
		} else if (r < sizeof(tmp)) {
			usbi_err(DEVICE_CTX(dev), "short read %d/%d", r, sizeof(tmp));
			return LIBUSB_ERROR_IO;
		}

		/* check bConfigurationValue */
		if (tmp[5] == config)
			break;

		/* try the next descriptor */
		off = lseek(fd, 0 - sizeof(tmp), SEEK_CUR);
		if (off < 0)
			return LIBUSB_ERROR_IO;

		r = seek_to_next_config(DEVICE_CTX(dev), fd);
		if (r < 0)
			return r;
	}

	to_copy = (len < sizeof(tmp)) ? len : sizeof(tmp);
	memcpy(buffer, tmp, to_copy);
	if (len > sizeof(tmp)) {
		r = read(fd, buffer + sizeof(tmp), len - sizeof(tmp));
		if (r < 0) {
			usbi_err(DEVICE_CTX(dev), "read failed, ret=%d errno=%d",
				fd, errno);
			r = LIBUSB_ERROR_IO;
		} else if (r == 0) {
			usbi_dbg("device is unconfigured");
			r = LIBUSB_ERROR_NOT_FOUND;
		} else if (r < len - sizeof(tmp)) {
			usbi_err(DEVICE_CTX(dev), "short read %d/%d", r, len);
			r = LIBUSB_ERROR_IO;
		}
	} else {
		r = 0;
	}

	close(fd);
	return r;
}

static int op_get_active_config_descriptor(struct libusb_device *dev,
	unsigned char *buffer, size_t len, int *host_endian)
{
	if (sysfs_has_descriptors) {
		return sysfs_get_active_config_descriptor(dev, buffer, len);
	} else {
		*host_endian = 1;
		return usbfs_get_active_config_descriptor(dev, buffer, len);
	}
}

/* takes a usbfs fd, attempts to find the requested config and copy a certain
 * amount of it into an output buffer. */
static int get_config_descriptor(struct libusb_context *ctx, int fd,
	uint8_t config_index, unsigned char *buffer, size_t len)
{
	off_t off;
	ssize_t r;

	off = lseek(fd, DEVICE_DESC_LENGTH, SEEK_SET);
	if (off < 0) {
		usbi_err(ctx, "seek failed ret=%d errno=%d", off, errno);
		return LIBUSB_ERROR_IO;
	}

	/* might need to skip some configuration descriptors to reach the
	 * requested configuration */
	while (config_index > 0) {
		r = seek_to_next_config(ctx, fd);
		if (r < 0)
			return r;
		config_index--;
	}

	/* read the rest of the descriptor */
	r = read(fd, buffer, len);
	if (r < 0) {
		usbi_err(ctx, "read failed ret=%d errno=%d", r, errno);
		return LIBUSB_ERROR_IO;
	} else if (r < len) {
		usbi_err(ctx, "short output read %d/%d", r, len);
		return LIBUSB_ERROR_IO;
	}

	return 0;
}

static int op_get_config_descriptor(struct libusb_device *dev,
	uint8_t config_index, unsigned char *buffer, size_t len, int *host_endian)
{
	char filename[PATH_MAX];
	int fd;
	int r;

	/* always read from usbfs: sysfs only has the active descriptor
	 * this will involve waking the device up, but oh well! */

	/* FIXME: the above is no longer true, new kernels have all descriptors
	 * in the descriptors file. but its kinda hard to detect if the kernel
	 * is sufficiently new. */

	__get_usbfs_path(dev, filename);
	fd = open(filename, O_RDONLY);
	if (fd < 0) {
		usbi_err(DEVICE_CTX(dev),
			"open '%s' failed, ret=%d errno=%d", filename, fd, errno);
		return LIBUSB_ERROR_IO;
	}

	r = get_config_descriptor(DEVICE_CTX(dev), fd, config_index, buffer, len);
	close(fd);
	*host_endian = 1;
	return r;
}

/* cache the active config descriptor in memory. a value of -1 means that
 * we aren't sure which one is active, so just assume the first one. 
 * only for usbfs. */
static int cache_active_config(struct libusb_device *dev, int fd,
	int active_config)
{
	struct linux_device_priv *priv = __device_priv(dev);
	struct libusb_config_descriptor config;
	unsigned char tmp[8];
	unsigned char *buf;
	int idx;
	int r;

	if (active_config == -1) {
		idx = 0;
	} else {
		r = usbi_get_config_index_by_value(dev, active_config, &idx);
		if (r < 0)
			return r;
		if (idx == -1)
			return LIBUSB_ERROR_NOT_FOUND;
	}

	r = get_config_descriptor(DEVICE_CTX(dev), fd, idx, tmp, sizeof(tmp));
	if (r < 0) {
		usbi_err(DEVICE_CTX(dev), "first read error %d", r);
		return r;
	}

	usbi_parse_descriptor(tmp, "bbw", &config, 1);
	buf = malloc(config.wTotalLength);
	if (!buf)
		return LIBUSB_ERROR_NO_MEM;

	r = get_config_descriptor(DEVICE_CTX(dev), fd, idx, buf,
		config.wTotalLength);
	if (r < 0) {
		free(buf);
		return r;
	}

	if (priv->config_descriptor)
		free(priv->config_descriptor);
	priv->config_descriptor = buf;
	return 0;
}

/* send a control message to retrieve active configuration */
static int usbfs_get_active_config(struct libusb_device *dev, int fd)
{
	unsigned char active_config = 0;
	int r;

	struct usbfs_ctrltransfer ctrl = {
		.bmRequestType = LIBUSB_ENDPOINT_IN,
		.bRequest = LIBUSB_REQUEST_GET_CONFIGURATION,
		.wValue = 0,
		.wIndex = 0,
		.wLength = 1,
		.timeout = 1000,
		.data = &active_config
	};

	r = ioctl(fd, IOCTL_USBFS_CONTROL, &ctrl);
	if (r < 0) {
		if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(DEVICE_CTX(dev),
			"get_configuration failed ret=%d errno=%d", r, errno);
		return LIBUSB_ERROR_IO;
	}

	return active_config;
}

static int initialize_device(struct libusb_device *dev, uint8_t busnum,
	uint8_t devaddr, const char *sysfs_dir)
{
	struct linux_device_priv *priv = __device_priv(dev);
	unsigned char *dev_buf;
	char path[PATH_MAX];
	int fd;
	int active_config = 0;
	int device_configured = 1;
	ssize_t r;

	dev->bus_number = busnum;
	dev->device_address = devaddr;

	if (sysfs_dir) {
		priv->sysfs_dir = malloc(strlen(sysfs_dir) + 1);
		if (!priv->sysfs_dir)
			return LIBUSB_ERROR_NO_MEM;
		strcpy(priv->sysfs_dir, sysfs_dir);
	}

	if (sysfs_has_descriptors)
		return 0;

	/* cache device descriptor in memory so that we can retrieve it later
	 * without waking the device up (op_get_device_descriptor) */

	priv->dev_descriptor = NULL;
	priv->config_descriptor = NULL;

	if (sysfs_can_relate_devices) {
		int tmp = sysfs_get_active_config(dev, &active_config);
		if (tmp < 0)
			return tmp;
		if (active_config == -1)
			device_configured = 0;
	}

	__get_usbfs_path(dev, path);
	fd = open(path, O_RDWR);
	if (fd < 0 && errno == EACCES) {
		fd = open(path, O_RDONLY);
		/* if we only have read-only access to the device, we cannot
		 * send a control message to determine the active config. just
		 * assume the first one is active. */
		active_config = -1;
	}

	if (fd < 0) {
		usbi_err(DEVICE_CTX(dev), "open failed, ret=%d errno=%d", fd, errno);
		return LIBUSB_ERROR_IO;
	}

	if (!sysfs_can_relate_devices) {
		if (active_config == -1) {
			/* if we only have read-only access to the device, we cannot
			 * send a control message to determine the active config. just
			 * assume the first one is active. */
			usbi_warn(DEVICE_CTX(dev), "access to %s is read-only; cannot "
				"determine active configuration descriptor", path);
		} else {
			active_config = usbfs_get_active_config(dev, fd);
			if (active_config < 0) {
				close(fd);
				return active_config;
			} else if (active_config == 0) {
				/* some buggy devices have a configuration 0, but we're
				 * reaching into the corner of a corner case here, so let's
				 * not support buggy devices in these circumstances.
				 * stick to the specs: a configuration value of 0 means
				 * unconfigured. */
				usbi_dbg("assuming unconfigured device");
				device_configured = 0;
			}
		}
	}

	dev_buf = malloc(DEVICE_DESC_LENGTH);
	if (!dev_buf) {
		close(fd);
		return LIBUSB_ERROR_NO_MEM;
	}

	r = read(fd, dev_buf, DEVICE_DESC_LENGTH);
	if (r < 0) {
		usbi_err(DEVICE_CTX(dev),
			"read descriptor failed ret=%d errno=%d", fd, errno);
		free(dev_buf);
		close(fd);
		return LIBUSB_ERROR_IO;
	} else if (r < DEVICE_DESC_LENGTH) {
		usbi_err(DEVICE_CTX(dev), "short descriptor read (%d)", r);
		free(dev_buf);
		close(fd);
		return LIBUSB_ERROR_IO;
	}

	/* bit of a hack: set num_configurations now because cache_active_config()
	 * calls usbi_get_config_index_by_value() which uses it */
	dev->num_configurations = dev_buf[DEVICE_DESC_LENGTH - 1];

	if (device_configured) {
		r = cache_active_config(dev, fd, active_config);
		if (r < 0) {
			close(fd);
			free(dev_buf);
			return r;
		}
	}

	close(fd);
	priv->dev_descriptor = dev_buf;
	return 0;
}

static int enumerate_device(struct libusb_context *ctx,
	struct discovered_devs **_discdevs, uint8_t busnum, uint8_t devaddr,
	const char *sysfs_dir)
{
	struct discovered_devs *discdevs;
	unsigned long session_id;
	int need_unref = 0;
	struct libusb_device *dev;
	int r = 0;

	/* FIXME: session ID is not guaranteed unique as addresses can wrap and
	 * will be reused. instead we should add a simple sysfs attribute with
	 * a session ID. */
	session_id = busnum << 8 | devaddr;
	usbi_dbg("busnum %d devaddr %d session_id %ld", busnum, devaddr,
		session_id);

	dev = usbi_get_device_by_session_id(ctx, session_id);
	if (dev) {
		usbi_dbg("using existing device for %d/%d (session %ld)",
			busnum, devaddr, session_id);
	} else {
		usbi_dbg("allocating new device for %d/%d (session %ld)",
			busnum, devaddr, session_id);
		dev = usbi_alloc_device(ctx, session_id);
		if (!dev)
			return LIBUSB_ERROR_NO_MEM;
		need_unref = 1;
		r = initialize_device(dev, busnum, devaddr, sysfs_dir);
		if (r < 0)
			goto out;
		r = usbi_sanitize_device(dev);
		if (r < 0)
			goto out;
	}

	discdevs = discovered_devs_append(*_discdevs, dev);
	if (!discdevs)
		r = LIBUSB_ERROR_NO_MEM;
	else
		*_discdevs = discdevs;

out:
	if (need_unref)
		libusb_unref_device(dev);
	return r;
}

/* open a bus directory and adds all discovered devices to discdevs. on
 * failure (non-zero return) the pre-existing discdevs should be destroyed
 * (and devices freed). on success, the new discdevs pointer should be used
 * as it may have been moved. */
static int usbfs_scan_busdir(struct libusb_context *ctx,
	struct discovered_devs **_discdevs, uint8_t busnum)
{
	DIR *dir;
	char dirpath[PATH_MAX];
	struct dirent *entry;
	struct discovered_devs *discdevs = *_discdevs;
	int r = 0;

	snprintf(dirpath, PATH_MAX, "%s/%03d", usbfs_path, busnum);
	usbi_dbg("%s", dirpath);
	dir = opendir(dirpath);
	if (!dir) {
		usbi_err(ctx, "opendir '%s' failed, errno=%d", dirpath, errno);
		/* FIXME: should handle valid race conditions like hub unplugged
		 * during directory iteration - this is not an error */
		return LIBUSB_ERROR_IO;
	}

	while ((entry = readdir(dir))) {
		int devaddr;

		if (entry->d_name[0] == '.')
			continue;

		devaddr = atoi(entry->d_name);
		if (devaddr == 0) {
			usbi_dbg("unknown dir entry %s", entry->d_name);
			continue;
		}

		r = enumerate_device(ctx, &discdevs, busnum, (uint8_t) devaddr, NULL);
		if (r < 0)
			goto out;
	}

	*_discdevs = discdevs;
out:
	closedir(dir);
	return r;
}

static int usbfs_get_device_list(struct libusb_context *ctx,
	struct discovered_devs **_discdevs)
{
	struct dirent *entry;
	DIR *buses = opendir(usbfs_path);
	struct discovered_devs *discdevs = *_discdevs;
	int r = 0;

	if (!buses) {
		usbi_err(ctx, "opendir buses failed errno=%d", errno);
		return LIBUSB_ERROR_IO;
	}

	while ((entry = readdir(buses))) {
		struct discovered_devs *discdevs_new = discdevs;
		int busnum;

		if (entry->d_name[0] == '.')
			continue;

		busnum = atoi(entry->d_name);
		if (busnum == 0) {
			usbi_dbg("unknown dir entry %s", entry->d_name);
			continue;
		}

		r = usbfs_scan_busdir(ctx, &discdevs_new, busnum);
		if (r < 0)
			goto out;
		discdevs = discdevs_new;
	}

out:
	closedir(buses);
	*_discdevs = discdevs;
	return r;

}

static int sysfs_scan_device(struct libusb_context *ctx,
	struct discovered_devs **_discdevs, const char *devname,
	int *usbfs_fallback)
{
	int r;
	FILE *fd;
	char filename[PATH_MAX];
	int busnum;
	int devaddr;

	usbi_dbg("scan %s", devname);

	/* determine descriptors presence ahead of time, we need to know this
	 * when we reach initialize_device */
	if (sysfs_has_descriptors == -1) {
		struct stat statbuf;

		snprintf(filename, PATH_MAX, "%s/%s/descriptors", SYSFS_DEVICE_PATH,
			devname);
		r = stat(filename, &statbuf);
		if (r == 0 && S_ISREG(statbuf.st_mode)) {
			usbi_dbg("sysfs descriptors available");
			sysfs_has_descriptors = 1;
		} else {
			usbi_dbg("sysfs descriptors not available");
			sysfs_has_descriptors = 0;
		}
	}

	snprintf(filename, PATH_MAX, "%s/%s/busnum", SYSFS_DEVICE_PATH, devname);
	fd = fopen(filename, "r");
	if (!fd) {
		if (errno == ENOENT) {
			usbi_dbg("busnum not found, cannot relate sysfs to usbfs, "
				"falling back on pure usbfs");
			sysfs_can_relate_devices = 0;
			*usbfs_fallback = 1;
			return LIBUSB_ERROR_OTHER;
		}
		usbi_err(ctx, "open busnum failed, errno=%d", errno);
		return LIBUSB_ERROR_IO;
	}

	sysfs_can_relate_devices = 1;

	r = fscanf(fd, "%d", &busnum);
	fclose(fd);
	if (r != 1) {
		usbi_err(ctx, "fscanf busnum returned %d, errno=%d", r, errno);
		return LIBUSB_ERROR_IO;
	}

	snprintf(filename, PATH_MAX, "%s/%s/devnum", SYSFS_DEVICE_PATH, devname);
	fd = fopen(filename, "r");
	if (!fd) {
		usbi_err(ctx, "open devnum failed, errno=%d", errno);
		return LIBUSB_ERROR_IO;
	}

	r = fscanf(fd, "%d", &devaddr);
	fclose(fd);
	if (r != 1) {
		usbi_err(ctx, "fscanf devnum returned %d, errno=%d", r, errno);
		return LIBUSB_ERROR_IO;
	}

	usbi_dbg("bus=%d dev=%d", busnum, devaddr);
	if (busnum > 255 || devaddr > 255)
		return LIBUSB_ERROR_INVALID_PARAM;

	return enumerate_device(ctx, _discdevs, busnum & 0xff, devaddr & 0xff,
		devname);
}

static int sysfs_get_device_list(struct libusb_context *ctx,
	struct discovered_devs **_discdevs, int *usbfs_fallback)
{
	struct discovered_devs *discdevs = *_discdevs;
	DIR *devices = opendir(SYSFS_DEVICE_PATH);
	struct dirent *entry;
	int r = 0;

	if (!devices) {
		usbi_err(ctx, "opendir devices failed errno=%d", errno);
		return LIBUSB_ERROR_IO;
	}

	while ((entry = readdir(devices))) {
		struct discovered_devs *discdevs_new = discdevs;

		if ((!isdigit(entry->d_name[0]) && strncmp(entry->d_name, "usb", 3))
				|| strchr(entry->d_name, ':'))
			continue;

		r = sysfs_scan_device(ctx, &discdevs_new, entry->d_name,
			usbfs_fallback);
		if (r < 0)
			goto out;
		discdevs = discdevs_new;
	}	

out:
	closedir(devices);
	*_discdevs = discdevs;
	return r;
}

static int op_get_device_list(struct libusb_context *ctx,
	struct discovered_devs **_discdevs)
{
	/* we can retrieve device list and descriptors from sysfs or usbfs.
	 * sysfs is preferable, because if we use usbfs we end up resuming
	 * any autosuspended USB devices. however, sysfs is not available
	 * everywhere, so we need a usbfs fallback too.
	 *
	 * as described in the "sysfs vs usbfs" comment, sometimes we have
	 * sysfs but not enough information to relate sysfs devices to usbfs
	 * nodes. the usbfs_fallback variable is used to indicate that we should
	 * fall back on usbfs.
	 */
	if (sysfs_can_relate_devices != 0) {
		int usbfs_fallback = 0;
		int r = sysfs_get_device_list(ctx, _discdevs, &usbfs_fallback);
		if (!usbfs_fallback)
			return r;
	}

	return usbfs_get_device_list(ctx, _discdevs);
}

static int op_open(struct libusb_device_handle *handle)
{
	struct linux_device_handle_priv *hpriv = __device_handle_priv(handle);
	char filename[PATH_MAX];

	__get_usbfs_path(handle->dev, filename);
	hpriv->fd = open(filename, O_RDWR);
	if (hpriv->fd < 0) {
		if (errno == EACCES) {

			fprintf(stderr, "libusb couldn't open USB device %s: "
				"Permission denied.\n"
				"libusb requires write access to USB device nodes.\n",
				filename);
			char buf[512];

			sprintf(buf, "libusb couldn't open USB device %s: "
				"Permission denied.\n"
				"libusb requires write access to USB device nodes.\n",
				filename);
			__android_log_write(ANDROID_LOG_INFO, "Kinect", buf);

			return LIBUSB_ERROR_ACCESS;
		} else if (errno == ENOENT) {
			__android_log_write(ANDROID_LOG_INFO, "Kinect", "ENOENT");
			return LIBUSB_ERROR_NO_DEVICE;
		} else {
			usbi_err(HANDLE_CTX(handle),
				"open failed, code %d errno %d", hpriv->fd, errno);
			char buf[512];
			sprintf(buf, "open failed, code %d errno %d", hpriv->fd, errno);
			__android_log_write(ANDROID_LOG_INFO, "Kinect", buf);
			return LIBUSB_ERROR_IO;
		}
	}

	return usbi_add_pollfd(HANDLE_CTX(handle), hpriv->fd, POLLOUT);
}

static void op_close(struct libusb_device_handle *dev_handle)
{
	int fd = __device_handle_priv(dev_handle)->fd;
	usbi_remove_pollfd(HANDLE_CTX(dev_handle), fd);
	close(fd);
}

static int op_get_configuration(struct libusb_device_handle *handle,
	int *config)
{
	int r;
	if (sysfs_can_relate_devices != 1)
		return LIBUSB_ERROR_NOT_SUPPORTED;

	r = sysfs_get_active_config(handle->dev, config);
	if (*config == -1)
		*config = 0;

	return 0;
}

static int op_set_configuration(struct libusb_device_handle *handle, int config)
{
	struct linux_device_priv *priv = __device_priv(handle->dev);
	int fd = __device_handle_priv(handle)->fd;
	int r = ioctl(fd, IOCTL_USBFS_SETCONFIG, &config);
	if (r) {
		if (errno == EINVAL)
			return LIBUSB_ERROR_NOT_FOUND;
		else if (errno == EBUSY)
			return LIBUSB_ERROR_BUSY;
		else if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(HANDLE_CTX(handle), "failed, error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	}

	if (!sysfs_has_descriptors) {
		/* update our cached active config descriptor */
		if (config == -1) {
			if (priv->config_descriptor) {
				free(priv->config_descriptor);
				priv->config_descriptor = NULL;
			}
		} else {
			r = cache_active_config(handle->dev, fd, config);
			if (r < 0)
				usbi_warn(HANDLE_CTX(handle),
					"failed to update cached config descriptor, error %d", r);
		}
	}

	return 0;
}

static int op_claim_interface(struct libusb_device_handle *handle, int iface)
{
	int fd = __device_handle_priv(handle)->fd;
	int r = ioctl(fd, IOCTL_USBFS_CLAIMINTF, &iface);
	if (r) {
		if (errno == ENOENT)
			return LIBUSB_ERROR_NOT_FOUND;
		else if (errno == EBUSY)
			return LIBUSB_ERROR_BUSY;
		else if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(HANDLE_CTX(handle),
			"claim interface failed, error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	}
	return 0;
}

static int op_release_interface(struct libusb_device_handle *handle, int iface)
{
	int fd = __device_handle_priv(handle)->fd;
	int r = ioctl(fd, IOCTL_USBFS_RELEASEINTF, &iface);
	if (r) {
		if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(HANDLE_CTX(handle),
			"release interface failed, error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	}
	return 0;
}

static int op_set_interface(struct libusb_device_handle *handle, int iface,
	int altsetting)
{
	int fd = __device_handle_priv(handle)->fd;
	struct usbfs_setinterface setintf;
	int r;

	setintf.interface = iface;
	setintf.altsetting = altsetting;
	r = ioctl(fd, IOCTL_USBFS_SETINTF, &setintf);
	if (r) {
		if (errno == EINVAL)
			return LIBUSB_ERROR_NOT_FOUND;
		else if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(HANDLE_CTX(handle),
			"setintf failed error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	}

	return 0;
}

static int op_clear_halt(struct libusb_device_handle *handle,
	unsigned char endpoint)
{
	int fd = __device_handle_priv(handle)->fd;
	unsigned int _endpoint = endpoint;
	int r = ioctl(fd, IOCTL_USBFS_CLEAR_HALT, &_endpoint);
	if (r) {
		if (errno == ENOENT)
			return LIBUSB_ERROR_NOT_FOUND;
		else if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(HANDLE_CTX(handle),
			"clear_halt failed error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	}

	return 0;
}

static int op_reset_device(struct libusb_device_handle *handle)
{
	int fd = __device_handle_priv(handle)->fd;
	int r = ioctl(fd, IOCTL_USBFS_RESET, NULL);
	if (r) {
		if (errno == ENODEV)
			return LIBUSB_ERROR_NOT_FOUND;

		usbi_err(HANDLE_CTX(handle),
			"reset failed error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	}

	return 0;
}

static int op_kernel_driver_active(struct libusb_device_handle *handle,
	int interface)
{
	int fd = __device_handle_priv(handle)->fd;
	struct usbfs_getdriver getdrv;
	int r;

	getdrv.interface = interface;
	r = ioctl(fd, IOCTL_USBFS_GETDRIVER, &getdrv);
	if (r) {
		if (errno == ENODATA)
			return 0;
		else if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(HANDLE_CTX(handle),
			"get driver failed error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	}

	return 1;
}

static int op_detach_kernel_driver(struct libusb_device_handle *handle,
	int interface)
{
	int fd = __device_handle_priv(handle)->fd;
	struct usbfs_ioctl command;
	int r;

	command.ifno = interface;
	command.ioctl_code = IOCTL_USBFS_DISCONNECT;
	command.data = NULL;

	r = ioctl(fd, IOCTL_USBFS_IOCTL, &command);
	if (r) {
		if (errno == ENODATA)
			return LIBUSB_ERROR_NOT_FOUND;
		else if (errno == EINVAL)
			return LIBUSB_ERROR_INVALID_PARAM;
		else if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(HANDLE_CTX(handle),
			"detach failed error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	}

	return 0;
}

static int op_attach_kernel_driver(struct libusb_device_handle *handle,
	int interface)
{
	int fd = __device_handle_priv(handle)->fd;
	struct usbfs_ioctl command;
	int r;

	command.ifno = interface;
	command.ioctl_code = IOCTL_USBFS_CONNECT;
	command.data = NULL;

	r = ioctl(fd, IOCTL_USBFS_IOCTL, &command);
	if (r < 0) {
		if (errno == ENODATA)
			return LIBUSB_ERROR_NOT_FOUND;
		else if (errno == EINVAL)
			return LIBUSB_ERROR_INVALID_PARAM;
		else if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;
		else if (errno == EBUSY)
			return LIBUSB_ERROR_BUSY;

		usbi_err(HANDLE_CTX(handle),
			"attach failed error %d errno %d", r, errno);
		return LIBUSB_ERROR_OTHER;
	} else if (r == 0) {
		return LIBUSB_ERROR_NOT_FOUND;
	}

	return 0;
}

static void op_destroy_device(struct libusb_device *dev)
{
	struct linux_device_priv *priv = __device_priv(dev);
	if (!sysfs_has_descriptors) {
		if (priv->dev_descriptor)
			free(priv->dev_descriptor);
		if (priv->config_descriptor)
			free(priv->config_descriptor);
	}
	if (priv->sysfs_dir)
		free(priv->sysfs_dir);
}

static void free_iso_urbs(struct linux_transfer_priv *tpriv)
{
	int i;
	for (i = 0; i < tpriv->num_urbs; i++) {
		struct usbfs_urb *urb = tpriv->iso_urbs[i];
		if (!urb)
			break;
		free(urb);
	}

	free(tpriv->iso_urbs);
}

static int submit_bulk_transfer(struct usbi_transfer *itransfer,
	unsigned char urb_type)
{
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	struct linux_device_handle_priv *dpriv =
		__device_handle_priv(transfer->dev_handle);
	struct usbfs_urb *urbs;
	int r;
	int i;
	size_t alloc_size;

	/* usbfs places a 16kb limit on bulk URBs. we divide up larger requests
	 * into smaller units to meet such restriction, then fire off all the
	 * units at once. it would be simpler if we just fired one unit at a time,
	 * but there is a big performance gain through doing it this way. */
	int num_urbs = transfer->length / MAX_BULK_BUFFER_LENGTH;
	int last_urb_partial = 0;

	if ((transfer->length % MAX_BULK_BUFFER_LENGTH) > 0) {
		last_urb_partial = 1;
		num_urbs++;
	}
	usbi_dbg("need %d urbs for new transfer with length %d", num_urbs,
		transfer->length);
	alloc_size = num_urbs * sizeof(struct usbfs_urb);
	urbs = malloc(alloc_size);
	if (!urbs)
		return LIBUSB_ERROR_NO_MEM;
	memset(urbs, 0, alloc_size);
	tpriv->urbs = urbs;
	tpriv->num_urbs = num_urbs;
	tpriv->awaiting_discard = 0;
	tpriv->awaiting_reap = 0;
	tpriv->reap_action = NORMAL;

	for (i = 0; i < num_urbs; i++) {
		struct usbfs_urb *urb = &urbs[i];
		urb->usercontext = itransfer;
		urb->type = urb_type;
		urb->endpoint = transfer->endpoint;
		urb->buffer = transfer->buffer + (i * MAX_BULK_BUFFER_LENGTH);
		if (i == num_urbs - 1 && last_urb_partial)
			urb->buffer_length = transfer->length % MAX_BULK_BUFFER_LENGTH;
		else
			urb->buffer_length = MAX_BULK_BUFFER_LENGTH;

		r = ioctl(dpriv->fd, IOCTL_USBFS_SUBMITURB, urb);
		if (r < 0) {
			int j;

			if (errno == ENODEV) {
				r = LIBUSB_ERROR_NO_DEVICE;
			} else {
				usbi_err(TRANSFER_CTX(transfer),
					"submiturb failed error %d errno=%d", r, errno);
				r = LIBUSB_ERROR_IO;
			}
	
			/* if the first URB submission fails, we can simply free up and
			 * return failure immediately. */
			if (i == 0) {
				usbi_dbg("first URB failed, easy peasy");
				free(urbs);
				return r;
			}

			/* if it's not the first URB that failed, the situation is a bit
			 * tricky. we must discard all previous URBs. there are
			 * complications:
			 *  - discarding is asynchronous - discarded urbs will be reaped
			 *    later. the user must not have freed the transfer when the
			 *    discarded URBs are reaped, otherwise libusb will be using
			 *    freed memory.
			 *  - the earlier URBs may have completed successfully and we do
			 *    not want to throw away any data.
			 * so, in this case we discard all the previous URBs BUT we report
			 * that the transfer was submitted successfully. then later when
			 * the final discard completes we can report error to the user.
			 */
			tpriv->reap_action = SUBMIT_FAILED;
			for (j = 0; j < i; j++) {
				int tmp = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, &urbs[j]);
				if (tmp == 0)
					tpriv->awaiting_discard++;
				else if (errno == EINVAL)
					tpriv->awaiting_reap++;
				else
					usbi_warn(TRANSFER_CTX(transfer),
						"unrecognised discard return %d", tmp);
			}

			usbi_dbg("reporting successful submission but waiting for %d "
				"discards and %d reaps before reporting error",
				tpriv->awaiting_discard, tpriv->awaiting_reap);
			return 0;
		}
	}

	return 0;
}

static int submit_iso_transfer(struct usbi_transfer *itransfer)
{
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	struct linux_device_handle_priv *dpriv =
		__device_handle_priv(transfer->dev_handle);
	struct usbfs_urb **urbs;
	size_t alloc_size;
	int num_packets = transfer->num_iso_packets;
	int i;
	int this_urb_len = 0;
	int num_urbs = 1;
	int packet_offset = 0;
	unsigned int packet_len;
	unsigned char *urb_buffer = transfer->buffer;

	/* usbfs places a 32kb limit on iso URBs. we divide up larger requests
	 * into smaller units to meet such restriction, then fire off all the
	 * units at once. it would be simpler if we just fired one unit at a time,
	 * but there is a big performance gain through doing it this way. */

	/* calculate how many URBs we need */
	for (i = 0; i < num_packets; i++) {
		int space_remaining = MAX_ISO_BUFFER_LENGTH - this_urb_len;
		packet_len = transfer->iso_packet_desc[i].length;

		if (packet_len > space_remaining) {
			num_urbs++;
			this_urb_len = packet_len;
		} else {
			this_urb_len += packet_len;
		}
	}
	usbi_dbg("need %d 32k URBs for transfer", num_urbs);

	alloc_size = num_urbs * sizeof(*urbs);
	urbs = malloc(alloc_size);
	if (!urbs)
		return LIBUSB_ERROR_NO_MEM;
	memset(urbs, 0, alloc_size);

	tpriv->iso_urbs = urbs;
	tpriv->num_urbs = num_urbs;
	tpriv->awaiting_discard = 0;
	tpriv->awaiting_reap = 0;
	tpriv->reap_action = NORMAL;
	tpriv->iso_packet_offset = 0;

	/* allocate + initialize each URB with the correct number of packets */
	for (i = 0; i < num_urbs; i++) {
		struct usbfs_urb *urb;
		int space_remaining_in_urb = MAX_ISO_BUFFER_LENGTH;
		int urb_packet_offset = 0;
		unsigned char *urb_buffer_orig = urb_buffer;
		int j;
		int k;

		/* swallow up all the packets we can fit into this URB */
		while (packet_offset < transfer->num_iso_packets) {
			packet_len = transfer->iso_packet_desc[packet_offset].length;
			if (packet_len <= space_remaining_in_urb) {
				/* throw it in */
				urb_packet_offset++;
				packet_offset++;
				space_remaining_in_urb -= packet_len;
				urb_buffer += packet_len;
			} else {
				/* it can't fit, save it for the next URB */
				break;
			}
		}

		alloc_size = sizeof(*urb)
			+ (urb_packet_offset * sizeof(struct usbfs_iso_packet_desc));
		urb = malloc(alloc_size);
		if (!urb) {
			free_iso_urbs(tpriv);
			return LIBUSB_ERROR_NO_MEM;
		}
		memset(urb, 0, alloc_size);
		urbs[i] = urb;

		/* populate packet lengths */
		for (j = 0, k = packet_offset - urb_packet_offset;
				k < packet_offset; k++, j++) {
			packet_len = transfer->iso_packet_desc[k].length;
			urb->iso_frame_desc[j].length = packet_len;
		}

		urb->usercontext = itransfer;
		urb->type = USBFS_URB_TYPE_ISO;
		/* FIXME: interface for non-ASAP data? */
		urb->flags = USBFS_URB_ISO_ASAP;
		urb->endpoint = transfer->endpoint;
		urb->number_of_packets = urb_packet_offset;
		urb->buffer = urb_buffer_orig;
	}

	/* submit URBs */
	for (i = 0; i < num_urbs; i++) {
		int r = ioctl(dpriv->fd, IOCTL_USBFS_SUBMITURB, urbs[i]);
		if (r < 0) {
			int j;

			if (errno == ENODEV) {
				r = LIBUSB_ERROR_NO_DEVICE;
			} else {
				usbi_err(TRANSFER_CTX(transfer),
					"submiturb failed error %d errno=%d", r, errno);
				r = LIBUSB_ERROR_IO;
			}

			/* if the first URB submission fails, we can simply free up and
			 * return failure immediately. */
			if (i == 0) {
				usbi_dbg("first URB failed, easy peasy");
				free_iso_urbs(tpriv);
				return r;
			}

			/* if it's not the first URB that failed, the situation is a bit
			 * tricky. we must discard all previous URBs. there are
			 * complications:
			 *  - discarding is asynchronous - discarded urbs will be reaped
			 *    later. the user must not have freed the transfer when the
			 *    discarded URBs are reaped, otherwise libusb will be using
			 *    freed memory.
			 *  - the earlier URBs may have completed successfully and we do
			 *    not want to throw away any data.
			 * so, in this case we discard all the previous URBs BUT we report
			 * that the transfer was submitted successfully. then later when
			 * the final discard completes we can report error to the user.
			 */
			tpriv->reap_action = SUBMIT_FAILED;
			for (j = 0; j < i; j++) {
				int tmp = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, urbs[j]);
				if (tmp == 0)
					tpriv->awaiting_discard++;
				else if (errno == EINVAL)
					tpriv->awaiting_reap++;
				else
					usbi_warn(TRANSFER_CTX(transfer),
						"unrecognised discard return %d", tmp);
			}

			usbi_dbg("reporting successful submission but waiting for %d "
				"discards and %d reaps before reporting error",
				tpriv->awaiting_discard, tpriv->awaiting_reap);
			return 0;
		}
	}

	return 0;
}

static int submit_control_transfer(struct usbi_transfer *itransfer)
{
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct linux_device_handle_priv *dpriv =
		__device_handle_priv(transfer->dev_handle);
	struct usbfs_urb *urb;
	int r;

	if (transfer->length - LIBUSB_CONTROL_SETUP_SIZE > MAX_CTRL_BUFFER_LENGTH)
		return LIBUSB_ERROR_INVALID_PARAM;

	urb = malloc(sizeof(struct usbfs_urb));
	if (!urb)
		return LIBUSB_ERROR_NO_MEM;
	memset(urb, 0, sizeof(struct usbfs_urb));
	tpriv->urbs = urb;
	tpriv->reap_action = NORMAL;

	urb->usercontext = itransfer;
	urb->type = USBFS_URB_TYPE_CONTROL;
	urb->endpoint = transfer->endpoint;
	urb->buffer = transfer->buffer;
	urb->buffer_length = transfer->length;

	r = ioctl(dpriv->fd, IOCTL_USBFS_SUBMITURB, urb);
	if (r < 0) {
		free(urb);
		if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(TRANSFER_CTX(transfer),
			"submiturb failed error %d errno=%d", r, errno);
		return LIBUSB_ERROR_IO;
	}
	return 0;
}

static int op_submit_transfer(struct usbi_transfer *itransfer)
{
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

	switch (transfer->type) {
	case LIBUSB_TRANSFER_TYPE_CONTROL:
		return submit_control_transfer(itransfer);
	case LIBUSB_TRANSFER_TYPE_BULK:
		return submit_bulk_transfer(itransfer, USBFS_URB_TYPE_BULK);
	case LIBUSB_TRANSFER_TYPE_INTERRUPT:
		return submit_bulk_transfer(itransfer, USBFS_URB_TYPE_INTERRUPT);
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
		return submit_iso_transfer(itransfer);
	default:
		usbi_err(TRANSFER_CTX(transfer),
			"unknown endpoint type %d", transfer->type);
		return LIBUSB_ERROR_INVALID_PARAM;
	}
}

static int cancel_control_transfer(struct usbi_transfer *itransfer)
{
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct linux_device_handle_priv *dpriv =
		__device_handle_priv(transfer->dev_handle);
	int r;

	tpriv->reap_action = CANCELLED;
	r = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, tpriv->urbs);
	if(r) {
		if (errno == EINVAL) {
			usbi_dbg("URB not found --> assuming ready to be reaped");
			return 0;
		} else {
			usbi_err(TRANSFER_CTX(transfer),
				"unrecognised DISCARD code %d", errno);
			return LIBUSB_ERROR_OTHER;
		}
	}

	return 0;
}

static void cancel_bulk_transfer(struct usbi_transfer *itransfer)
{
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct linux_device_handle_priv *dpriv =
		__device_handle_priv(transfer->dev_handle);
	int i;

	tpriv->reap_action = CANCELLED;
	tpriv->awaiting_reap = 0;
	tpriv->awaiting_discard = 0;
	for (i = 0; i < tpriv->num_urbs; i++) {
		int tmp = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, &tpriv->urbs[i]);
		if (tmp == 0)
			tpriv->awaiting_discard++;
		else if (errno == EINVAL)
			tpriv->awaiting_reap++;
		else
			usbi_warn(TRANSFER_CTX(transfer),
				"unrecognised discard return %d", errno);
	}
}

static void cancel_iso_transfer(struct usbi_transfer *itransfer)
{
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct linux_device_handle_priv *dpriv =
		__device_handle_priv(transfer->dev_handle);
	int i;

	tpriv->reap_action = CANCELLED;
	tpriv->awaiting_reap = 0;
	tpriv->awaiting_discard = 0;
	for (i = 0; i < tpriv->num_urbs; i++) {
		int tmp = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, tpriv->iso_urbs[i]);
		if (tmp == 0)
			tpriv->awaiting_discard++;
		else if (errno == EINVAL)
			tpriv->awaiting_reap++;
		else
			usbi_warn(TRANSFER_CTX(transfer),
				"unrecognised discard return %d", errno);
	}
}

static int op_cancel_transfer(struct usbi_transfer *itransfer)
{
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

	switch (transfer->type) {
	case LIBUSB_TRANSFER_TYPE_CONTROL:
		return cancel_control_transfer(itransfer);
	case LIBUSB_TRANSFER_TYPE_BULK:
	case LIBUSB_TRANSFER_TYPE_INTERRUPT:
		cancel_bulk_transfer(itransfer);
		return 0;
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
		cancel_iso_transfer(itransfer);
		return 0;
	default:
		usbi_err(TRANSFER_CTX(transfer),
			"unknown endpoint type %d", transfer->type);
		return LIBUSB_ERROR_INVALID_PARAM;
	}
}

static void op_clear_transfer_priv(struct usbi_transfer *itransfer)
{
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);

	switch (transfer->type) {
	case LIBUSB_TRANSFER_TYPE_CONTROL:
	case LIBUSB_TRANSFER_TYPE_BULK:
	case LIBUSB_TRANSFER_TYPE_INTERRUPT:
		free(tpriv->urbs);
		break;
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
		free_iso_urbs(tpriv);
		break;
	default:
		usbi_err(TRANSFER_CTX(transfer),
			"unknown endpoint type %d", transfer->type);
	}
}

static int handle_bulk_completion(struct usbi_transfer *itransfer,
	struct usbfs_urb *urb)
{
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	int num_urbs = tpriv->num_urbs;
	int urb_idx = urb - tpriv->urbs;
	enum libusb_transfer_status status = LIBUSB_TRANSFER_COMPLETED;

	usbi_dbg("handling completion status %d of bulk urb %d/%d", urb->status,
		urb_idx + 1, num_urbs);

	if (urb->status == 0 ||
			(urb->status == -EOVERFLOW && urb->actual_length > 0))
		itransfer->transferred += urb->actual_length;

	if (tpriv->reap_action != NORMAL) {
		/* cancelled, submit_fail, or completed early */
		if (urb->status == -ENOENT) {
			usbi_dbg("CANCEL: detected a cancelled URB");
			if (tpriv->awaiting_discard == 0)
				usbi_err(ITRANSFER_CTX(itransfer),
					"CANCEL: cancelled URB but not awaiting discards?");
			else
				tpriv->awaiting_discard--;
		} else if (urb->status == 0) {
			usbi_dbg("CANCEL: detected a completed URB");

			/* FIXME we could solve this extreme corner case with a memmove
			 * or something */
			if (tpriv->reap_action == COMPLETED_EARLY)
				usbi_warn(ITRANSFER_CTX(itransfer), "SOME DATA LOST! "
					"(completed early but remaining urb completed)");

			if (tpriv->awaiting_reap == 0)
				usbi_err(ITRANSFER_CTX(itransfer),
					"CANCEL: completed URB not awaiting reap?");
			else
				tpriv->awaiting_reap--;
		} else if (urb->status == -EPIPE || urb->status == -EOVERFLOW) {
			if (tpriv->awaiting_reap == 0)
				usbi_err(ITRANSFER_CTX(itransfer),
					"CANCEL: completed URB not awaiting reap?");
			else
				tpriv->awaiting_reap--;
		} else {
			usbi_warn(ITRANSFER_CTX(itransfer),
				"unhandled CANCEL urb status %d", urb->status);
		}

		if (tpriv->awaiting_reap == 0 && tpriv->awaiting_discard == 0) {
			usbi_dbg("CANCEL: last URB handled, reporting");
			if (tpriv->reap_action == CANCELLED) {
				free(tpriv->urbs);
				usbi_handle_transfer_cancellation(itransfer);
				return 0;
			} else if (tpriv->reap_action == COMPLETED_EARLY) {
				goto out;
			} else {
				status = LIBUSB_TRANSFER_ERROR;
				goto out;
			}
		}
		return 0;
	}

	switch (urb->status) {
	case 0:
		break;
	case -EPIPE:
		usbi_dbg("detected endpoint stall");
		status = LIBUSB_TRANSFER_STALL;
		goto out;
	case -EOVERFLOW:
		/* overflow can only ever occur in the last urb */
		usbi_dbg("overflow, actual_length=%d", urb->actual_length);
		status = LIBUSB_TRANSFER_OVERFLOW;
		goto out;
	case -ETIME:
	case -EPROTO:
	case -EILSEQ:
		usbi_dbg("low level error %d", urb->status);
		status = LIBUSB_TRANSFER_ERROR;
		goto out;
	default:
		usbi_warn(ITRANSFER_CTX(itransfer),
			"unrecognised urb status %d", urb->status);
		status = LIBUSB_TRANSFER_ERROR;
		goto out;
	}

	/* if we're the last urb or we got less data than requested then we're
	 * done */
	if (urb_idx == num_urbs - 1) {
		usbi_dbg("last URB in transfer --> complete!");
	} else if (urb->actual_length < urb->buffer_length) {
		struct libusb_transfer *transfer =
			__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
		struct linux_device_handle_priv *dpriv =
			__device_handle_priv(transfer->dev_handle);
		int i;

		usbi_dbg("short transfer %d/%d --> complete!", urb->actual_length,
			urb->buffer_length);

		/* we have to cancel the remaining urbs and wait for their completion
		 * before reporting results */
		tpriv->reap_action = COMPLETED_EARLY;
		for (i = urb_idx + 1; i < tpriv->num_urbs; i++) {
			int r = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, &tpriv->urbs[i]);
			if (r == 0)
				tpriv->awaiting_discard++;
			else if (errno == EINVAL)
				tpriv->awaiting_reap++;
			else
				usbi_warn(ITRANSFER_CTX(itransfer),
					"unrecognised discard return %d", errno);
		}
		return 0;
	} else {
		return 0;
	}

out:
	free(tpriv->urbs);
	usbi_handle_transfer_completion(itransfer, status);
	return 0;
}

static int handle_iso_completion(struct usbi_transfer *itransfer,
	struct usbfs_urb *urb)
{
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	int num_urbs = tpriv->num_urbs;
	int urb_idx = 0;
	int i;

	for (i = 0; i < num_urbs; i++) {
		if (urb == tpriv->iso_urbs[i]) {
			urb_idx = i + 1;
			break;
		}
	}
	if (urb_idx == 0) {
		usbi_err(TRANSFER_CTX(transfer), "could not locate urb!");
		return LIBUSB_ERROR_NOT_FOUND;
	}

	usbi_dbg("handling completion status %d of iso urb %d/%d", urb->status,
		urb_idx, num_urbs);

	if (urb->status == 0) {
		/* copy isochronous results back in */

		for (i = 0; i < urb->number_of_packets; i++) {
			struct usbfs_iso_packet_desc *urb_desc = &urb->iso_frame_desc[i];
			struct libusb_iso_packet_descriptor *lib_desc =
				&transfer->iso_packet_desc[tpriv->iso_packet_offset++];
			lib_desc->status = urb_desc->status;
			lib_desc->actual_length = urb_desc->actual_length;
		}
	}

	if (tpriv->reap_action != NORMAL) { /* cancelled or submit_fail */
		if (urb->status == -ENOENT) {
			usbi_dbg("CANCEL: detected a cancelled URB");
			if (tpriv->awaiting_discard == 0)
				usbi_err(TRANSFER_CTX(transfer),
					"CANCEL: cancelled URB but not awaiting discards?");
			else
				tpriv->awaiting_discard--;
		} else if (urb->status == 0) {
			usbi_dbg("CANCEL: detected a completed URB");
			if (tpriv->awaiting_reap == 0)
				usbi_err(TRANSFER_CTX(transfer),
					"CANCEL: completed URB not awaiting reap?");
			else
				tpriv->awaiting_reap--;
		} else {
			usbi_warn(TRANSFER_CTX(transfer),
				"unhandled CANCEL urb status %d", urb->status);
		}

		if (tpriv->awaiting_reap == 0 && tpriv->awaiting_discard == 0) {
			usbi_dbg("CANCEL: last URB handled, reporting");
			free_iso_urbs(tpriv);
			if (tpriv->reap_action == CANCELLED)
				usbi_handle_transfer_cancellation(itransfer);
			else
				usbi_handle_transfer_completion(itransfer,
					LIBUSB_TRANSFER_ERROR);
		}
		return 0;
	}

	switch (urb->status) {
	case 0:
		break;
	case -ETIME:
	case -EPROTO:
	case -EILSEQ:
		usbi_dbg("low-level USB error %d", urb->status);
		break;
	default:
		usbi_warn(TRANSFER_CTX(transfer),
			"unrecognised urb status %d", urb->status);
		break;
	}

	/* if we're the last urb or we got less data than requested then we're
	 * done */
	if (urb_idx == num_urbs) {
		usbi_dbg("last URB in transfer --> complete!");
		free_iso_urbs(tpriv);
		usbi_handle_transfer_completion(itransfer, LIBUSB_TRANSFER_COMPLETED);
	}

	return 0;
}

static int handle_control_completion(struct usbi_transfer *itransfer,
	struct usbfs_urb *urb)
{
	struct linux_transfer_priv *tpriv = usbi_transfer_get_os_priv(itransfer);
	int status;

	usbi_dbg("handling completion status %d", urb->status);

	if (urb->status == 0)
		itransfer->transferred += urb->actual_length;

	if (tpriv->reap_action == CANCELLED) {
		if (urb->status != 0 && urb->status != -ENOENT)
			usbi_warn(ITRANSFER_CTX(itransfer),
				"cancel: unrecognised urb status %d", urb->status);
		free(tpriv->urbs);
		usbi_handle_transfer_cancellation(itransfer);
		return 0;
	}

	switch (urb->status) {
	case 0:
		itransfer->transferred = urb->actual_length;
		status = LIBUSB_TRANSFER_COMPLETED;
		break;
	case -EPIPE:
		usbi_dbg("unsupported control request");
		status = LIBUSB_TRANSFER_STALL;
		break;
	case -ETIME:
	case -EPROTO:
	case -EILSEQ:
		usbi_dbg("low-level bus error occurred");
		status = LIBUSB_TRANSFER_ERROR;
		break;
	default:
		usbi_warn(ITRANSFER_CTX(itransfer),
			"unrecognised urb status %d", urb->status);
		status = LIBUSB_TRANSFER_ERROR;
		break;
	}

	free(tpriv->urbs);
	usbi_handle_transfer_completion(itransfer, status);
	return 0;
}

static int reap_for_handle(struct libusb_device_handle *handle)
{
	struct linux_device_handle_priv *hpriv = __device_handle_priv(handle);
	int r;
	struct usbfs_urb *urb;
	struct usbi_transfer *itransfer;
	struct libusb_transfer *transfer;

	r = ioctl(hpriv->fd, IOCTL_USBFS_REAPURBNDELAY, &urb);
	if (r == -1 && errno == EAGAIN)
		return 1;
	if (r < 0) {
		if (errno == ENODEV)
			return LIBUSB_ERROR_NO_DEVICE;

		usbi_err(HANDLE_CTX(handle), "reap failed error %d errno=%d",
			r, errno);
		return LIBUSB_ERROR_IO;
	}

	itransfer = urb->usercontext;
	transfer = __USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

	usbi_dbg("urb type=%d status=%d transferred=%d", urb->type, urb->status,
		urb->actual_length);

	switch (transfer->type) {
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
		return handle_iso_completion(itransfer, urb);
	case LIBUSB_TRANSFER_TYPE_BULK:
	case LIBUSB_TRANSFER_TYPE_INTERRUPT:
		return handle_bulk_completion(itransfer, urb);
	case LIBUSB_TRANSFER_TYPE_CONTROL:
		return handle_control_completion(itransfer, urb);
	default:
		usbi_err(HANDLE_CTX(handle), "unrecognised endpoint type %x",
			transfer->type);
		return LIBUSB_ERROR_OTHER;
	}
}

static int op_handle_events(struct libusb_context *ctx,
	struct pollfd *fds, nfds_t nfds, int num_ready)
{
	int r;
	int i = 0;

	pthread_mutex_lock(&ctx->open_devs_lock);
	for (i = 0; i < nfds && num_ready > 0; i++) {
		struct pollfd *pollfd = &fds[i];
		struct libusb_device_handle *handle;
		struct linux_device_handle_priv *hpriv = NULL;

		if (!pollfd->revents)
			continue;

		num_ready--;
		list_for_each_entry(handle, &ctx->open_devs, list) {
			hpriv =  __device_handle_priv(handle);
			if (hpriv->fd == pollfd->fd)
				break;
		}

		if (pollfd->revents & POLLERR) {
			usbi_remove_pollfd(HANDLE_CTX(handle), hpriv->fd);
			usbi_handle_disconnect(handle);
			continue;
		}

		r = reap_for_handle(handle);
		if (r == 1 || r == LIBUSB_ERROR_NO_DEVICE)
			continue;
		else if (r < 0)
			goto out;
	}

	r = 0;
out:
	pthread_mutex_unlock(&ctx->open_devs_lock);
	return r;
}

const struct usbi_os_backend linux_usbfs_backend = {
	.name = "Linux usbfs",
	.init = op_init,
	.exit = NULL,
	.get_device_list = op_get_device_list,
	.get_device_descriptor = op_get_device_descriptor,
	.get_active_config_descriptor = op_get_active_config_descriptor,
	.get_config_descriptor = op_get_config_descriptor,

	.open = op_open,
	.close = op_close,
	.get_configuration = op_get_configuration,
	.set_configuration = op_set_configuration,
	.claim_interface = op_claim_interface,
	.release_interface = op_release_interface,

	.set_interface_altsetting = op_set_interface,
	.clear_halt = op_clear_halt,
	.reset_device = op_reset_device,

	.kernel_driver_active = op_kernel_driver_active,
	.detach_kernel_driver = op_detach_kernel_driver,
	.attach_kernel_driver = op_attach_kernel_driver,

	.destroy_device = op_destroy_device,

	.submit_transfer = op_submit_transfer,
	.cancel_transfer = op_cancel_transfer,
	.clear_transfer_priv = op_clear_transfer_priv,

	.handle_events = op_handle_events,

	.device_priv_size = sizeof(struct linux_device_priv),
	.device_handle_priv_size = sizeof(struct linux_device_handle_priv),
	.transfer_priv_size = sizeof(struct linux_transfer_priv),
	.add_iso_packet_size = 0,
};


/*
 * Internal header for libusb
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

#ifndef __LIBUSBI_H__
#define __LIBUSBI_H__

#include <config.h>

#include <poll.h>
#include <pthread.h>
#include <stddef.h>
#include <time.h>

#include <libusb.h>

#define DEVICE_DESC_LENGTH		18

#define USB_MAXENDPOINTS	32
#define USB_MAXINTERFACES	32
#define USB_MAXCONFIG		8

struct list_head {
	struct list_head *prev, *next;
};

/* Get an entry from the list 
 * 	ptr - the address of this list_head element in "type" 
 * 	type - the data type that contains "member"
 * 	member - the list_head element in "type" 
 */
#define list_entry(ptr, type, member) \
	((type *)((char *)(ptr) - (unsigned long)(&((type *)0L)->member)))

/* Get each entry from a list
 *	pos - A structure pointer has a "member" element
 *	head - list head
 *	member - the list_head element in "pos"
 */
#define list_for_each_entry(pos, head, member)				\
	for (pos = list_entry((head)->next, typeof(*pos), member);	\
	     &pos->member != (head);					\
	     pos = list_entry(pos->member.next, typeof(*pos), member))

#define list_for_each_entry_safe(pos, n, head, member)			\
        for (pos = list_entry((head)->next, typeof(*pos), member),	\
		n = list_entry(pos->member.next, typeof(*pos), member);	\
	     &pos->member != (head);					\
	     pos = n, n = list_entry(n->member.next, typeof(*n), member))

#define list_empty(entry) ((entry)->next == (entry))

static inline void list_init(struct list_head *entry)
{
	entry->prev = entry->next = entry;
}

static inline void list_add(struct list_head *entry, struct list_head *head)
{
	entry->next = head->next;
	entry->prev = head;

	head->next->prev = entry;
	head->next = entry;
}

static inline void list_add_tail(struct list_head *entry,
	struct list_head *head)
{
	entry->next = head;
	entry->prev = head->prev;

	head->prev->next = entry;
	head->prev = entry;
}

static inline void list_del(struct list_head *entry)
{
	entry->next->prev = entry->prev;
	entry->prev->next = entry->next;
}

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})

#define MIN(a, b)	((a) < (b) ? (a) : (b))
#define MAX(a, b)	((a) > (b) ? (a) : (b))

#define TIMESPEC_IS_SET(ts) ((ts)->tv_sec != 0 || (ts)->tv_nsec != 0)

enum usbi_log_level {
	LOG_LEVEL_DEBUG,
	LOG_LEVEL_INFO,
	LOG_LEVEL_WARNING,
	LOG_LEVEL_ERROR,
};

void usbi_log(struct libusb_context *ctx, enum usbi_log_level,
	const char *function, const char *format, ...);

#ifdef ENABLE_LOGGING
#define _usbi_log(ctx, level, fmt...) usbi_log(ctx, level, __FUNCTION__, fmt)
#else
#define _usbi_log(ctx, level, fmt...)
#endif

#ifdef ENABLE_DEBUG_LOGGING
#define usbi_dbg(fmt...) _usbi_log(NULL, LOG_LEVEL_DEBUG, fmt)
#else
#define usbi_dbg(fmt...)
#endif

#define usbi_info(ctx, fmt...) _usbi_log(ctx, LOG_LEVEL_INFO, fmt)
#define usbi_warn(ctx, fmt...) _usbi_log(ctx, LOG_LEVEL_WARNING, fmt)
#define usbi_err(ctx, fmt...) _usbi_log(ctx, LOG_LEVEL_ERROR, fmt)

#define USBI_GET_CONTEXT(ctx) if (!(ctx)) (ctx) = usbi_default_context
#define DEVICE_CTX(dev) ((dev)->ctx)
#define HANDLE_CTX(handle) (DEVICE_CTX((handle)->dev))
#define TRANSFER_CTX(transfer) (HANDLE_CTX((transfer)->dev_handle))
#define ITRANSFER_CTX(transfer) \
	(TRANSFER_CTX(__USBI_TRANSFER_TO_LIBUSB_TRANSFER(transfer)))

extern struct libusb_context *usbi_default_context;

struct libusb_context {
	int debug;
	int debug_fixed;

	/* internal control pipe, used for interrupting event handling when
	 * something needs to modify poll fds. */
	int ctrl_pipe[2];

	struct list_head usb_devs;
	pthread_mutex_t usb_devs_lock;

	/* A list of open handles. Backends are free to traverse this if required.
	 */
	struct list_head open_devs;
	pthread_mutex_t open_devs_lock;

	/* this is a list of in-flight transfer handles, sorted by timeout 
	 * expiration. URBs to timeout the soonest are placed at the beginning of
	 * the list, URBs that will time out later are placed after, and urbs with
	 * infinite timeout are always placed at the very end. */
	struct list_head flying_transfers;
	pthread_mutex_t flying_transfers_lock;

	/* list of poll fds */
	struct list_head pollfds;
	pthread_mutex_t pollfds_lock;

	/* a counter that is set when we want to interrupt event handling, in order
	 * to modify the poll fd set. and a lock to protect it. */
	unsigned int pollfd_modify;
	pthread_mutex_t pollfd_modify_lock;

	/* user callbacks for pollfd changes */
	libusb_pollfd_added_cb fd_added_cb;
	libusb_pollfd_removed_cb fd_removed_cb;
	void *fd_cb_user_data;

	/* ensures that only one thread is handling events at any one time */
	pthread_mutex_t events_lock;

	/* used to see if there is an active thread doing event handling */
	int event_handler_active;

	/* used to wait for event completion in threads other than the one that is
	 * event handling */
	pthread_mutex_t event_waiters_lock;
	pthread_cond_t event_waiters_cond;
};

struct libusb_device {
	/* lock protects refcnt, everything else is finalized at initialization
	 * time */
	pthread_mutex_t lock;
	int refcnt;

	struct libusb_context *ctx;

	uint8_t bus_number;
	uint8_t device_address;
	uint8_t num_configurations;

	struct list_head list;
	unsigned long session_data;
	unsigned char os_priv[0];
};

struct libusb_device_handle {
	/* lock protects claimed_interfaces */
	pthread_mutex_t lock;
	unsigned long claimed_interfaces;

	struct list_head list;
	struct libusb_device *dev;
	unsigned char os_priv[0];
};

#define USBI_TRANSFER_TIMED_OUT	 			(1<<0)

/* in-memory transfer layout:
 *
 * 1. struct usbi_transfer
 * 2. struct libusb_transfer (which includes iso packets) [variable size]
 * 3. os private data [variable size]
 *
 * from a libusb_transfer, you can get the usbi_transfer by rewinding the
 * appropriate number of bytes.
 * the usbi_transfer includes the number of allocated packets, so you can
 * determine the size of the transfer and hence the start and length of the
 * OS-private data.
 */

struct usbi_transfer {
	int num_iso_packets;
	struct list_head list;
	struct timeval timeout;
	int transferred;
	uint8_t flags;
};

#define __USBI_TRANSFER_TO_LIBUSB_TRANSFER(transfer) \
	((struct libusb_transfer *)(((void *)(transfer)) \
		+ sizeof(struct usbi_transfer)))
#define __LIBUSB_TRANSFER_TO_USBI_TRANSFER(transfer) \
	((struct usbi_transfer *)(((void *)(transfer)) \
		- sizeof(struct usbi_transfer)))

static inline void *usbi_transfer_get_os_priv(struct usbi_transfer *transfer)
{
	return ((void *)transfer) + sizeof(struct usbi_transfer)
		+ sizeof(struct libusb_transfer)
		+ (transfer->num_iso_packets
			* sizeof(struct libusb_iso_packet_descriptor));
}

/* bus structures */

/* All standard descriptors have these 2 fields in common */
struct usb_descriptor_header {
	uint8_t  bLength;
	uint8_t  bDescriptorType;
};

/* shared data and functions */

int usbi_io_init(struct libusb_context *ctx);
void usbi_io_exit(struct libusb_context *ctx);

struct libusb_device *usbi_alloc_device(struct libusb_context *ctx,
	unsigned long session_id);
struct libusb_device *usbi_get_device_by_session_id(struct libusb_context *ctx,
	unsigned long session_id);
int usbi_sanitize_device(struct libusb_device *dev);
void usbi_handle_disconnect(struct libusb_device_handle *handle);

void usbi_handle_transfer_completion(struct usbi_transfer *itransfer,
	enum libusb_transfer_status status);
void usbi_handle_transfer_cancellation(struct usbi_transfer *transfer);

int usbi_parse_descriptor(unsigned char *source, char *descriptor, void *dest,
	int host_endian);
int usbi_get_config_index_by_value(struct libusb_device *dev,
	uint8_t bConfigurationValue, int *idx);

/* polling */

struct usbi_pollfd {
	/* must come first */
	struct libusb_pollfd pollfd;

	struct list_head list;
};

int usbi_add_pollfd(struct libusb_context *ctx, int fd, short events);
void usbi_remove_pollfd(struct libusb_context *ctx, int fd);

/* device discovery */

/* we traverse usbfs without knowing how many devices we are going to find.
 * so we create this discovered_devs model which is similar to a linked-list
 * which grows when required. it can be freed once discovery has completed,
 * eliminating the need for a list node in the libusb_device structure
 * itself. */
struct discovered_devs {
	size_t len;
	size_t capacity;
	struct libusb_device *devices[0];
};

struct discovered_devs *discovered_devs_append(
	struct discovered_devs *discdevs, struct libusb_device *dev);

/* OS abstraction */

/* This is the interface that OS backends need to implement.
 * All fields are mandatory, except ones explicitly noted as optional. */
struct usbi_os_backend {
	/* A human-readable name for your backend, e.g. "Linux usbfs" */
	const char *name;

	/* Perform initialization of your backend. You might use this function
	 * to determine specific capabilities of the system, allocate required
	 * data structures for later, etc.
	 *
	 * This function is called when a libusb user initializes the library
	 * prior to use.
	 *
	 * Return 0 on success, or a LIBUSB_ERROR code on failure.
	 */
	int (*init)(struct libusb_context *ctx);

	/* Deinitialization. Optional. This function should destroy anything
	 * that was set up by init.
	 *
	 * This function is called when the user deinitializes the library.
	 */
	void (*exit)(void);

	/* Enumerate all the USB devices on the system, returning them in a list
	 * of discovered devices.
	 *
	 * Your implementation should enumerate all devices on the system,
	 * regardless of whether they have been seen before or not.
	 *
	 * When you have found a device, compute a session ID for it. The session
	 * ID should uniquely represent that particular device for that particular
	 * connection session since boot (i.e. if you disconnect and reconnect a
	 * device immediately after, it should be assigned a different session ID).
	 * If your OS cannot provide a unique session ID as described above,
	 * presenting a session ID of (bus_number << 8 | device_address) should
	 * be sufficient. Bus numbers and device addresses wrap and get reused,
	 * but that is an unlikely case.
	 *
	 * After computing a session ID for a device, call
	 * usbi_get_device_by_session_id(). This function checks if libusb already
	 * knows about the device, and if so, it provides you with a libusb_device
	 * structure for it.
	 *
	 * If usbi_get_device_by_session_id() returns NULL, it is time to allocate
	 * a new device structure for the device. Call usbi_alloc_device() to
	 * obtain a new libusb_device structure with reference count 1. Populate
	 * the bus_number and device_address attributes of the new device, and
	 * perform any other internal backend initialization you need to do. At
	 * this point, you should be ready to provide device descriptors and so
	 * on through the get_*_descriptor functions. Finally, call
	 * usbi_sanitize_device() to perform some final sanity checks on the
	 * device. Assuming all of the above succeeded, we can now continue.
	 * If any of the above failed, remember to unreference the device that
	 * was returned by usbi_alloc_device().
	 *
	 * At this stage we have a populated libusb_device structure (either one
	 * that was found earlier, or one that we have just allocated and
	 * populated). This can now be added to the discovered devices list
	 * using discovered_devs_append(). Note that discovered_devs_append()
	 * may reallocate the list, returning a new location for it, and also
	 * note that reallocation can fail. Your backend should handle these
	 * error conditions appropriately.
	 *
	 * This function should not generate any bus I/O and should not block.
	 * If I/O is required (e.g. reading the active configuration value), it is
	 * OK to ignore these suggestions :)
	 *
	 * This function is executed when the user wishes to retrieve a list
	 * of USB devices connected to the system.
	 *
	 * Return 0 on success, or a LIBUSB_ERROR code on failure.
	 */
	int (*get_device_list)(struct libusb_context *ctx,
		struct discovered_devs **discdevs);

	/* Open a device for I/O and other USB operations. The device handle
	 * is preallocated for you, you can retrieve the device in question
	 * through handle->dev.
	 *
	 * Your backend should allocate any internal resources required for I/O
	 * and other operations so that those operations can happen (hopefully)
	 * without hiccup. This is also a good place to inform libusb that it
	 * should monitor certain file descriptors related to this device -
	 * see the usbi_add_pollfd() function.
	 *
	 * This function should not generate any bus I/O and should not block.
	 *
	 * This function is called when the user attempts to obtain a device
	 * handle for a device.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_ACCESS if the user has insufficient permissions
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since
	 *   discovery
	 * - another LIBUSB_ERROR code on other failure
	 *
	 * Do not worry about freeing the handle on failed open, the upper layers
	 * do this for you.
	 */
	int (*open)(struct libusb_device_handle *handle);

	/* Close a device such that the handle cannot be used again. Your backend
	 * should destroy any resources that were allocated in the open path.
	 * This may also be a good place to call usbi_remove_pollfd() to inform
	 * libusb of any file descriptors associated with this device that should
	 * no longer be monitored.
	 *
	 * This function is called when the user closes a device handle.
	 */
	void (*close)(struct libusb_device_handle *handle);

	/* Retrieve the device descriptor from a device.
	 *
	 * The descriptor should be retrieved from memory, NOT via bus I/O to the
	 * device. This means that you may have to cache it in a private structure
	 * during get_device_list enumeration. Alternatively, you may be able
	 * to retrieve it from a kernel interface (some Linux setups can do this)
	 * still without generating bus I/O.
	 *
	 * This function is expected to write DEVICE_DESC_LENGTH (18) bytes into
	 * buffer, which is guaranteed to be big enough.
	 *
	 * This function is called when sanity-checking a device before adding
	 * it to the list of discovered devices, and also when the user requests
	 * to read the device descriptor.
	 *
	 * This function is expected to return the descriptor in bus-endian format
	 * (LE). If it returns the multi-byte values in host-endian format,
	 * set the host_endian output parameter to "1".
	 *
	 * Return 0 on success or a LIBUSB_ERROR code on failure.
	 */
	int (*get_device_descriptor)(struct libusb_device *device,
		unsigned char *buffer, int *host_endian);

	/* Get the ACTIVE configuration descriptor for a device.
	 *
	 * The descriptor should be retrieved from memory, NOT via bus I/O to the
	 * device. This means that you may have to cache it in a private structure
	 * during get_device_list enumeration. You may also have to keep track
	 * of which configuration is active when the user changes it.
	 *
	 * This function is expected to write len bytes of data into buffer, which
	 * is guaranteed to be big enough. If you can only do a partial write,
	 * return an error code.
	 *
	 * This function is expected to return the descriptor in bus-endian format
	 * (LE). If it returns the multi-byte values in host-endian format,
	 * set the host_endian output parameter to "1".
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NOT_FOUND if the device is in unconfigured state
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*get_active_config_descriptor)(struct libusb_device *device,
		unsigned char *buffer, size_t len, int *host_endian);

	/* Get a specific configuration descriptor for a device.
	 *
	 * The descriptor should be retrieved from memory, NOT via bus I/O to the
	 * device. This means that you may have to cache it in a private structure
	 * during get_device_list enumeration.
	 *
	 * The requested descriptor is expressed as a zero-based index (i.e. 0
	 * indicates that we are requesting the first descriptor). The index does
	 * not (necessarily) equal the bConfigurationValue of the configuration
	 * being requested.
	 *
	 * This function is expected to write len bytes of data into buffer, which
	 * is guaranteed to be big enough. If you can only do a partial write,
	 * return an error code.
	 *
	 * This function is expected to return the descriptor in bus-endian format
	 * (LE). If it returns the multi-byte values in host-endian format,
	 * set the host_endian output parameter to "1".
	 *
	 * Return 0 on success or a LIBUSB_ERROR code on failure.
	 */
	int (*get_config_descriptor)(struct libusb_device *device,
		uint8_t config_index, unsigned char *buffer, size_t len,
		int *host_endian);

	/* Get the bConfigurationValue for the active configuration for a device.
	 * Optional. This should only be implemented if you can retrieve it from
	 * cache (don't generate I/O).
	 *
	 * If you cannot retrieve this from cache, either do not implement this
	 * function, or return LIBUSB_ERROR_NOT_SUPPORTED. This will cause
	 * libusb to retrieve the information through a standard control transfer.
	 *
	 * This function must be non-blocking.
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - LIBUSB_ERROR_NOT_SUPPORTED if the value cannot be retrieved without
	 *   blocking
	 * - another LIBUSB_ERROR code on other failure.
	 */
	int (*get_configuration)(struct libusb_device_handle *handle, int *config);

	/* Set the active configuration for a device.
	 *
	 * A configuration value of -1 should put the device in unconfigured state.
	 *
	 * This function can block.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NOT_FOUND if the configuration does not exist
	 * - LIBUSB_ERROR_BUSY if interfaces are currently claimed (and hence
	 *   configuration cannot be changed)
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - another LIBUSB_ERROR code on other failure.
	 */
	int (*set_configuration)(struct libusb_device_handle *handle, int config);

	/* Claim an interface. When claimed, the application can then perform
	 * I/O to an interface's endpoints.
	 *
	 * This function should not generate any bus I/O and should not block.
	 * Interface claiming is a logical operation that simply ensures that
	 * no other drivers/applications are using the interface, and after
	 * claiming, no other drivers/applicatiosn can use the interface because
	 * we now "own" it.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NOT_FOUND if the interface does not exist
	 * - LIBUSB_ERROR_BUSY if the interface is in use by another driver/app
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*claim_interface)(struct libusb_device_handle *handle, int iface);

	/* Release a previously claimed interface.
	 *
	 * This function should also generate a SET_INTERFACE control request,
	 * resetting the alternate setting of that interface to 0. It's OK for
	 * this function to block as a result.
	 *
	 * You will only ever be asked to release an interface which was
	 * successfully claimed earlier.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*release_interface)(struct libusb_device_handle *handle, int iface);

	/* Set the alternate setting for an interface.
	 *
	 * You will only ever be asked to set the alternate setting for an
	 * interface which was successfully claimed earlier.
	 *
	 * It's OK for this function to block.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NOT_FOUND if the alternate setting does not exist
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*set_interface_altsetting)(struct libusb_device_handle *handle,
		int iface, int altsetting);

	/* Clear a halt/stall condition on an endpoint.
	 *
	 * It's OK for this function to block.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NOT_FOUND if the endpoint does not exist
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*clear_halt)(struct libusb_device_handle *handle,
		unsigned char endpoint);

	/* Perform a USB port reset to reinitialize a device.
	 *
	 * If possible, the handle should still be usable after the reset
	 * completes, assuming that the device descriptors did not change during
	 * reset and all previous interface state can be restored.
	 *
	 * If something changes, or you cannot easily locate/verify the resetted
	 * device, return LIBUSB_ERROR_NOT_FOUND. This prompts the application
	 * to close the old handle and re-enumerate the device.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NOT_FOUND if re-enumeration is required, or if the device
	 *   has been disconnected since it was opened
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*reset_device)(struct libusb_device_handle *handle);

	/* Determine if a kernel driver is active on an interface. Optional.
	 *
	 * The presence of a kernel driver on an interface indicates that any
	 * calls to claim_interface would fail with the LIBUSB_ERROR_BUSY code.
	 *
	 * Return:
	 * - 0 if no driver is active
	 * - 1 if a driver is active
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*kernel_driver_active)(struct libusb_device_handle *handle,
		int interface);
	
	/* Detach a kernel driver from an interface. Optional.
	 *
	 * After detaching a kernel driver, the interface should be available
	 * for claim.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NOT_FOUND if no kernel driver was active
	 * - LIBUSB_ERROR_INVALID_PARAM if the interface does not exist
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*detach_kernel_driver)(struct libusb_device_handle *handle,
		int interface);

	/* Attach a kernel driver to an interface. Optional.
	 *
	 * Reattach a kernel driver to the device.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NOT_FOUND if no kernel driver was active
	 * - LIBUSB_ERROR_INVALID_PARAM if the interface does not exist
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected since it
	 *   was opened
	 * - LIBUSB_ERROR_BUSY if a program or driver has claimed the interface,
	 *   preventing reattachment
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*attach_kernel_driver)(struct libusb_device_handle *handle,
		int interface);

	/* Destroy a device. Optional.
	 *
	 * This function is called when the last reference to a device is
	 * destroyed. It should free any resources allocated in the get_device_list
	 * path.
	 */
	void (*destroy_device)(struct libusb_device *dev);

	/* Submit a transfer. Your implementation should take the transfer,
	 * morph it into whatever form your platform requires, and submit it
	 * asynchronously.
	 *
	 * This function must not block.
	 *
	 * Return:
	 * - 0 on success
	 * - LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
	 * - another LIBUSB_ERROR code on other failure
	 */
	int (*submit_transfer)(struct usbi_transfer *itransfer);

	/* Cancel a previously submitted transfer.
	 *
	 * This function must not block. The transfer cancellation must complete
	 * later, resulting in a call to usbi_handle_transfer_cancellation()
	 * from the context of handle_events.
	 */
	int (*cancel_transfer)(struct usbi_transfer *itransfer);

	/* Clear a transfer as if it has completed or cancelled, but do not
	 * report any completion/cancellation to the library. You should free
	 * all private data from the transfer as if you were just about to report
	 * completion or cancellation.
	 *
	 * This function might seem a bit out of place. It is used when libusb
	 * detects a disconnected device - it calls this function for all pending
	 * transfers before reporting completion (with the disconnect code) to
	 * the user. Maybe we can improve upon this internal interface in future.
	 */
	void (*clear_transfer_priv)(struct usbi_transfer *itransfer);

	/* Handle any pending events. This involves monitoring any active
	 * transfers and processing their completion or cancellation.
	 *
	 * The function is passed an array of pollfd structures (size nfds)
	 * as a result of the poll() system call. The num_ready parameter
	 * indicates the number of file descriptors that have reported events
	 * (i.e. the poll() return value). This should be enough information
	 * for you to determine which actions need to be taken on the currently
	 * active transfers.
	 *
	 * For any cancelled transfers, call usbi_handle_transfer_cancellation().
	 * For completed transfers, call usbi_handle_transfer_completion().
	 * For control/bulk/interrupt transfers, populate the "transferred"
	 * element of the appropriate usbi_transfer structure before calling the
	 * above functions. For isochronous transfers, populate the status and
	 * transferred fields of the iso packet descriptors of the transfer.
	 *
	 * This function should also be able to detect disconnection of the
	 * device, reporting that situation with usbi_handle_disconnect().
	 *
	 * Return 0 on success, or a LIBUSB_ERROR code on failure.
	 */
	int (*handle_events)(struct libusb_context *ctx,
		struct pollfd *fds, nfds_t nfds, int num_ready);

	/* Number of bytes to reserve for per-device private backend data.
	 * This private data area is accessible through the "os_priv" field of
	 * struct libusb_device. */
	size_t device_priv_size;

	/* Number of bytes to reserve for per-handle private backend data.
	 * This private data area is accessible through the "os_priv" field of
	 * struct libusb_device. */
	size_t device_handle_priv_size;

	/* Number of bytes to reserve for per-transfer private backend data.
	 * This private data area is accessible by calling
	 * usbi_transfer_get_os_priv() on the appropriate usbi_transfer instance.
	 */
	size_t transfer_priv_size;

	/* Mumber of additional bytes for os_priv for each iso packet.
	 * Can your backend use this? */
	/* FIXME: linux can't use this any more. if other OS's cannot either,
	 * then remove this */
	size_t add_iso_packet_size;
};

extern const struct usbi_os_backend * const usbi_backend;

extern const struct usbi_os_backend linux_usbfs_backend;

#endif


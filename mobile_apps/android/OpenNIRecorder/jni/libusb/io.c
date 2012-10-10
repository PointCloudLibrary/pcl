/*
 * I/O functions for libusb
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

#include <config.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "libusbi.h"

/**
 * \page io Synchronous and asynchronous device I/O
 *
 * \section intro Introduction
 *
 * If you're using libusb in your application, you're probably wanting to
 * perform I/O with devices - you want to perform USB data transfers.
 *
 * libusb offers two separate interfaces for device I/O. This page aims to
 * introduce the two in order to help you decide which one is more suitable
 * for your application. You can also choose to use both interfaces in your
 * application by considering each transfer on a case-by-case basis.
 *
 * Once you have read through the following discussion, you should consult the
 * detailed API documentation pages for the details:
 * - \ref syncio
 * - \ref asyncio
 *
 * \section theory Transfers at a logical level
 *
 * At a logical level, USB transfers typically happen in two parts. For
 * example, when reading data from a endpoint:
 * -# A request for data is sent to the device
 * -# Some time later, the incoming data is received by the host
 *
 * or when writing data to an endpoint:
 *
 * -# The data is sent to the device
 * -# Some time later, the host receives acknowledgement from the device that
 *    the data has been transferred.
 *
 * There may be an indefinite delay between the two steps. Consider a
 * fictional USB input device with a button that the user can press. In order
 * to determine when the button is pressed, you would likely submit a request
 * to read data on a bulk or interrupt endpoint and wait for data to arrive.
 * Data will arrive when the button is pressed by the user, which is
 * potentially hours later.
 *
 * libusb offers both a synchronous and an asynchronous interface to performing
 * USB transfers. The main difference is that the synchronous interface
 * combines both steps indicated above into a single function call, whereas
 * the asynchronous interface separates them.
 *
 * \section sync The synchronous interface
 *
 * The synchronous I/O interface allows you to perform a USB transfer with
 * a single function call. When the function call returns, the transfer has
 * completed and you can parse the results.
 *
 * If you have used the libusb-0.1 before, this I/O style will seem familar to
 * you. libusb-0.1 only offered a synchronous interface.
 *
 * In our input device example, to read button presses you might write code
 * in the following style:
\code
unsigned char data[4];
int actual_length,
int r = libusb_bulk_transfer(handle, EP_IN, data, sizeof(data), &actual_length, 0);
if (r == 0 && actual_length == sizeof(data)) {
	// results of the transaction can now be found in the data buffer
	// parse them here and report button press
} else {
	error();
}
\endcode
 *
 * The main advantage of this model is simplicity: you did everything with
 * a single simple function call.
 *
 * However, this interface has its limitations. Your application will sleep
 * inside libusb_bulk_transfer() until the transaction has completed. If it
 * takes the user 3 hours to press the button, your application will be
 * sleeping for that long. Execution will be tied up inside the library -
 * the entire thread will be useless for that duration.
 *
 * Another issue is that by tieing up the thread with that single transaction
 * there is no possibility of performing I/O with multiple endpoints and/or
 * multiple devices simultaneously, unless you resort to creating one thread
 * per transaction.
 *
 * Additionally, there is no opportunity to cancel the transfer after the
 * request has been submitted.
 *
 * For details on how to use the synchronous API, see the
 * \ref syncio "synchronous I/O API documentation" pages.
 * 
 * \section async The asynchronous interface
 *
 * Asynchronous I/O is the most significant new feature in libusb-1.0.
 * Although it is a more complex interface, it solves all the issues detailed
 * above.
 *
 * Instead of providing which functions that block until the I/O has complete,
 * libusb's asynchronous interface presents non-blocking functions which
 * begin a transfer and then return immediately. Your application passes a
 * callback function pointer to this non-blocking function, which libusb will
 * call with the results of the transaction when it has completed.
 *
 * Transfers which have been submitted through the non-blocking functions
 * can be cancelled with a separate function call.
 *
 * The non-blocking nature of this interface allows you to be simultaneously
 * performing I/O to multiple endpoints on multiple devices, without having
 * to use threads.
 *
 * This added flexibility does come with some complications though:
 * - In the interest of being a lightweight library, libusb does not create
 * threads and can only operate when your application is calling into it. Your
 * application must call into libusb from it's main loop when events are ready
 * to be handled, or you must use some other scheme to allow libusb to
 * undertake whatever work needs to be done.
 * - libusb also needs to be called into at certain fixed points in time in
 * order to accurately handle transfer timeouts.
 * - Memory handling becomes more complex. You cannot use stack memory unless
 * the function with that stack is guaranteed not to return until the transfer
 * callback has finished executing.
 * - You generally lose some linearity from your code flow because submitting
 * the transfer request is done in a separate function from where the transfer
 * results are handled. This becomes particularly obvious when you want to
 * submit a second transfer based on the results of an earlier transfer.
 *
 * Internally, libusb's synchronous interface is expressed in terms of function
 * calls to the asynchronous interface.
 *
 * For details on how to use the asynchronous API, see the
 * \ref asyncio "asynchronous I/O API" documentation pages.
 */


/**
 * \page packetoverflow Packets and overflows
 *
 * \section packets Packet abstraction
 *
 * The USB specifications describe how data is transmitted in packets, with
 * constraints on packet size defined by endpoint descriptors. The host must
 * not send data payloads larger than the endpoint's maximum packet size.
 *
 * libusb and the underlying OS abstract out the packet concept, allowing you
 * to request transfers of any size. Internally, the request will be divided
 * up into correctly-sized packets. You do not have to be concerned with
 * packet sizes, but there is one exception when considering overflows.
 *
 * \section overflow Bulk/interrupt transfer overflows
 *
 * When requesting data on a bulk endpoint, libusb requires you to supply a
 * buffer and the maximum number of bytes of data that libusb can put in that
 * buffer. However, the size of the buffer is not communicated to the device -
 * the device is just asked to send any amount of data.
 *
 * There is no problem if the device sends an amount of data that is less than
 * or equal to the buffer size. libusb reports this condition to you through
 * the \ref libusb_transfer::actual_length "libusb_transfer.actual_length"
 * field.
 *
 * Problems may occur if the device attempts to send more data than can fit in
 * the buffer. libusb reports LIBUSB_TRANSFER_OVERFLOW for this condition but
 * other behaviour is largely undefined: actual_length may or may not be
 * accurate, the chunk of data that can fit in the buffer (before overflow)
 * may or may not have been transferred.
 *
 * Overflows are nasty, but can be avoided. Even though you were told to
 * ignore packets above, think about the lower level details: each transfer is
 * split into packets (typically small, with a maximum size of 512 bytes).
 * Overflows can only happen if the final packet in an incoming data transfer
 * is smaller than the actual packet that the device wants to transfer.
 * Therefore, you will never see an overflow if your transfer buffer size is a
 * multiple of the endpoint's packet size: the final packet will either
 * fill up completely or will be only partially filled.
 */

/**
 * @defgroup asyncio Asynchronous device I/O
 *
 * This page details libusb's asynchronous (non-blocking) API for USB device
 * I/O. This interface is very powerful but is also quite complex - you will
 * need to read this page carefully to understand the necessary considerations
 * and issues surrounding use of this interface. Simplistic applications
 * may wish to consider the \ref syncio "synchronous I/O API" instead.
 *
 * The asynchronous interface is built around the idea of separating transfer
 * submission and handling of transfer completion (the synchronous model
 * combines both of these into one). There may be a long delay between
 * submission and completion, however the asynchronous submission function
 * is non-blocking so will return control to your application during that
 * potentially long delay.
 *
 * \section asyncabstraction Transfer abstraction
 *
 * For the asynchronous I/O, libusb implements the concept of a generic
 * transfer entity for all types of I/O (control, bulk, interrupt,
 * isochronous). The generic transfer object must be treated slightly
 * differently depending on which type of I/O you are performing with it.
 *
 * This is represented by the public libusb_transfer structure type.
 *
 * \section asynctrf Asynchronous transfers
 *
 * We can view asynchronous I/O as a 5 step process:
 * -# <b>Allocation</b>: allocate a libusb_transfer
 * -# <b>Filling</b>: populate the libusb_transfer instance with information
 *    about the transfer you wish to perform
 * -# <b>Submission</b>: ask libusb to submit the transfer
 * -# <b>Completion handling</b>: examine transfer results in the
 *    libusb_transfer structure
 * -# <b>Deallocation</b>: clean up resources
 * 
 *
 * \subsection asyncalloc Allocation
 *
 * This step involves allocating memory for a USB transfer. This is the
 * generic transfer object mentioned above. At this stage, the transfer
 * is "blank" with no details about what type of I/O it will be used for.
 *
 * Allocation is done with the libusb_alloc_transfer() function. You must use
 * this function rather than allocating your own transfers.
 *
 * \subsection asyncfill Filling
 *
 * This step is where you take a previously allocated transfer and fill it
 * with information to determine the message type and direction, data buffer,
 * callback function, etc.
 *
 * You can either fill the required fields yourself or you can use the
 * helper functions: libusb_fill_control_transfer(), libusb_fill_bulk_transfer()
 * and libusb_fill_interrupt_transfer().
 *
 * \subsection asyncsubmit Submission
 *
 * When you have allocated a transfer and filled it, you can submit it using
 * libusb_submit_transfer(). This function returns immediately but can be
 * regarded as firing off the I/O request in the background.
 *
 * \subsection asynccomplete Completion handling
 *
 * After a transfer has been submitted, one of four things can happen to it:
 *
 * - The transfer completes (i.e. some data was transferred)
 * - The transfer has a timeout and the timeout expires before all data is
 * transferred
 * - The transfer fails due to an error
 * - The transfer is cancelled
 *
 * Each of these will cause the user-specified transfer callback function to
 * be invoked. It is up to the callback function to determine which of the
 * above actually happened and to act accordingly.
 *
 * The user-specified callback is passed a pointer to the libusb_transfer
 * structure which was used to setup and submit the transfer. At completion
 * time, libusb has populated this structure with results of the transfer:
 * success or failure reason, number of bytes of data transferred, etc. See
 * the libusb_transfer structure documentation for more information.
 *
 * \subsection Deallocation
 *
 * When a transfer has completed (i.e. the callback function has been invoked),
 * you are advised to free the transfer (unless you wish to resubmit it, see
 * below). Transfers are deallocated with libusb_free_transfer().
 *
 * It is undefined behaviour to free a transfer which has not completed.
 *
 * \section asyncresubmit Resubmission
 *
 * You may be wondering why allocation, filling, and submission are all
 * separated above where they could reasonably be combined into a single
 * operation.
 *
 * The reason for separation is to allow you to resubmit transfers without
 * having to allocate new ones every time. This is especially useful for
 * common situations dealing with interrupt endpoints - you allocate one
 * transfer, fill and submit it, and when it returns with results you just
 * resubmit it for the next interrupt.
 *
 * \section asynccancel Cancellation
 *
 * Another advantage of using the asynchronous interface is that you have
 * the ability to cancel transfers which have not yet completed. This is
 * done by calling the libusb_cancel_transfer() function.
 *
 * libusb_cancel_transfer() is asynchronous/non-blocking in itself. When the
 * cancellation actually completes, the transfer's callback function will
 * be invoked, and the callback function should check the transfer status to
 * determine that it was cancelled.
 *
 * Freeing the transfer after it has been cancelled but before cancellation
 * has completed will result in undefined behaviour.
 *
 * \section bulk_overflows Overflows on device-to-host bulk/interrupt endpoints
 *
 * If your device does not have predictable transfer sizes (or it misbehaves),
 * your application may submit a request for data on an IN endpoint which is
 * smaller than the data that the device wishes to send. In some circumstances
 * this will cause an overflow, which is a nasty condition to deal with. See
 * the \ref packetoverflow page for discussion.
 *
 * \section asyncctrl Considerations for control transfers
 *
 * The <tt>libusb_transfer</tt> structure is generic and hence does not
 * include specific fields for the control-specific setup packet structure.
 *
 * In order to perform a control transfer, you must place the 8-byte setup
 * packet at the start of the data buffer. To simplify this, you could
 * cast the buffer pointer to type struct libusb_control_setup, or you can
 * use the helper function libusb_fill_control_setup().
 *
 * The wLength field placed in the setup packet must be the length you would
 * expect to be sent in the setup packet: the length of the payload that
 * follows (or the expected maximum number of bytes to receive). However,
 * the length field of the libusb_transfer object must be the length of
 * the data buffer - i.e. it should be wLength <em>plus</em> the size of
 * the setup packet (LIBUSB_CONTROL_SETUP_SIZE).
 *
 * If you use the helper functions, this is simplified for you:
 * -# Allocate a buffer of size LIBUSB_CONTROL_SETUP_SIZE plus the size of the
 * data you are sending/requesting.
 * -# Call libusb_fill_control_setup() on the data buffer, using the transfer
 * request size as the wLength value (i.e. do not include the extra space you
 * allocated for the control setup).
 * -# If this is a host-to-device transfer, place the data to be transferred
 * in the data buffer, starting at offset LIBUSB_CONTROL_SETUP_SIZE.
 * -# Call libusb_fill_control_transfer() to associate the data buffer with
 * the transfer (and to set the remaining details such as callback and timeout).
 *   - Note that there is no parameter to set the length field of the transfer.
 *     The length is automatically inferred from the wLength field of the setup
 *     packet.
 * -# Submit the transfer.
 *
 * The multi-byte control setup fields (wValue, wIndex and wLength) must
 * be given in little-endian byte order (the endianness of the USB bus).
 * Endianness conversion is transparently handled by
 * libusb_fill_control_setup() which is documented to accept host-endian
 * values.
 *
 * Further considerations are needed when handling transfer completion in
 * your callback function:
 * - As you might expect, the setup packet will still be sitting at the start
 * of the data buffer.
 * - If this was a device-to-host transfer, the received data will be sitting
 * at offset LIBUSB_CONTROL_SETUP_SIZE into the buffer.
 * - The actual_length field of the transfer structure is relative to the
 * wLength of the setup packet, rather than the size of the data buffer. So,
 * if your wLength was 4, your transfer's <tt>length</tt> was 12, then you
 * should expect an <tt>actual_length</tt> of 4 to indicate that the data was
 * transferred in entirity.
 *
 * To simplify parsing of setup packets and obtaining the data from the
 * correct offset, you may wish to use the libusb_control_transfer_get_data()
 * and libusb_control_transfer_get_setup() functions within your transfer
 * callback.
 *
 * Even though control endpoints do not halt, a completed control transfer
 * may have a LIBUSB_TRANSFER_STALL status code. This indicates the control
 * request was not supported.
 *
 * \section asyncintr Considerations for interrupt transfers
 * 
 * All interrupt transfers are performed using the polling interval presented
 * by the bInterval value of the endpoint descriptor.
 *
 * \section asynciso Considerations for isochronous transfers
 *
 * Isochronous transfers are more complicated than transfers to
 * non-isochronous endpoints.
 *
 * To perform I/O to an isochronous endpoint, allocate the transfer by calling
 * libusb_alloc_transfer() with an appropriate number of isochronous packets.
 *
 * During filling, set \ref libusb_transfer::type "type" to
 * \ref libusb_transfer_type::LIBUSB_TRANSFER_TYPE_ISOCHRONOUS
 * "LIBUSB_TRANSFER_TYPE_ISOCHRONOUS", and set
 * \ref libusb_transfer::num_iso_packets "num_iso_packets" to a value less than
 * or equal to the number of packets you requested during allocation.
 * libusb_alloc_transfer() does not set either of these fields for you, given
 * that you might not even use the transfer on an isochronous endpoint.
 *
 * Next, populate the length field for the first num_iso_packets entries in
 * the \ref libusb_transfer::iso_packet_desc "iso_packet_desc" array. Section
 * 5.6.3 of the USB2 specifications describe how the maximum isochronous
 * packet length is determined by wMaxPacketSize field in the endpoint
 * descriptor. Two functions can help you here:
 *
 * - libusb_get_max_packet_size() is an easy way to determine the max
 *   packet size for an endpoint.
 * - libusb_set_iso_packet_lengths() assigns the same length to all packets
 *   within a transfer, which is usually what you want.
 *
 * For outgoing transfers, you'll obviously fill the buffer and populate the
 * packet descriptors in hope that all the data gets transferred. For incoming
 * transfers, you must ensure the buffer has sufficient capacity for
 * the situation where all packets transfer the full amount of requested data.
 *
 * Completion handling requires some extra consideration. The
 * \ref libusb_transfer::actual_length "actual_length" field of the transfer
 * is meaningless and should not be examined; instead you must refer to the
 * \ref libusb_iso_packet_descriptor::actual_length "actual_length" field of
 * each individual packet.
 *
 * The \ref libusb_transfer::status "status" field of the transfer is also a
 * little misleading:
 *  - If the packets were submitted and the isochronous data microframes
 *    completed normally, status will have value
 *    \ref libusb_transfer_status::LIBUSB_TRANSFER_COMPLETED
 *    "LIBUSB_TRANSFER_COMPLETED". Note that bus errors and software-incurred
 *    delays are not counted as transfer errors; the transfer.status field may
 *    indicate COMPLETED even if some or all of the packets failed. Refer to
 *    the \ref libusb_iso_packet_descriptor::status "status" field of each
 *    individual packet to determine packet failures.
 *  - The status field will have value
 *    \ref libusb_transfer_status::LIBUSB_TRANSFER_ERROR
 *    "LIBUSB_TRANSFER_ERROR" only when serious errors were encountered.
 *  - Other transfer status codes occur with normal behaviour.
 *
 * The data for each packet will be found at an offset into the buffer that
 * can be calculated as if each prior packet completed in full. The
 * libusb_get_iso_packet_buffer() and libusb_get_iso_packet_buffer_simple()
 * functions may help you here.
 *
 * \section asyncmem Memory caveats
 *
 * In most circumstances, it is not safe to use stack memory for transfer
 * buffers. This is because the function that fired off the asynchronous
 * transfer may return before libusb has finished using the buffer, and when
 * the function returns it's stack gets destroyed. This is true for both
 * host-to-device and device-to-host transfers.
 *
 * The only case in which it is safe to use stack memory is where you can
 * guarantee that the function owning the stack space for the buffer does not
 * return until after the transfer's callback function has completed. In every
 * other case, you need to use heap memory instead.
 *
 * \section asyncflags Fine control
 *
 * Through using this asynchronous interface, you may find yourself repeating
 * a few simple operations many times. You can apply a bitwise OR of certain
 * flags to a transfer to simplify certain things:
 * - \ref libusb_transfer_flags::LIBUSB_TRANSFER_SHORT_NOT_OK
 *   "LIBUSB_TRANSFER_SHORT_NOT_OK" results in transfers which transferred
 *   less than the requested amount of data being marked with status
 *   \ref libusb_transfer_status::LIBUSB_TRANSFER_ERROR "LIBUSB_TRANSFER_ERROR"
 *   (they would normally be regarded as COMPLETED)
 * - \ref libusb_transfer_flags::LIBUSB_TRANSFER_FREE_BUFFER
 *   "LIBUSB_TRANSFER_FREE_BUFFER" allows you to ask libusb to free the transfer
 *   buffer when freeing the transfer.
 * - \ref libusb_transfer_flags::LIBUSB_TRANSFER_FREE_TRANSFER
 *   "LIBUSB_TRANSFER_FREE_TRANSFER" causes libusb to automatically free the
 *   transfer after the transfer callback returns.
 *
 * \section asyncevent Event handling
 *
 * In accordance of the aim of being a lightweight library, libusb does not
 * create threads internally. This means that libusb code does not execute
 * at any time other than when your application is calling a libusb function.
 * However, an asynchronous model requires that libusb perform work at various
 * points in time - namely processing the results of previously-submitted
 * transfers and invoking the user-supplied callback function.
 *
 * This gives rise to the libusb_handle_events() function which your
 * application must call into when libusb has work do to. This gives libusb
 * the opportunity to reap pending transfers, invoke callbacks, etc.
 *
 * The first issue to discuss here is how your application can figure out
 * when libusb has work to do. In fact, there are two naive options which
 * do not actually require your application to know this:
 * -# Periodically call libusb_handle_events() in non-blocking mode at fixed
 *    short intervals from your main loop
 * -# Repeatedly call libusb_handle_events() in blocking mode from a dedicated
 *    thread.
 *
 * The first option is plainly not very nice, and will cause unnecessary 
 * CPU wakeups leading to increased power usage and decreased battery life.
 * The second option is not very nice either, but may be the nicest option
 * available to you if the "proper" approach can not be applied to your
 * application (read on...).
 * 
 * The recommended option is to integrate libusb with your application main
 * event loop. libusb exposes a set of file descriptors which allow you to do
 * this. Your main loop is probably already calling poll() or select() or a
 * variant on a set of file descriptors for other event sources (e.g. keyboard
 * button presses, mouse movements, network sockets, etc). You then add
 * libusb's file descriptors to your poll()/select() calls, and when activity
 * is detected on such descriptors you know it is time to call
 * libusb_handle_events().
 *
 * There is one final event handling complication. libusb supports
 * asynchronous transfers which time out after a specified time period, and
 * this requires that libusb is called into at or after the timeout so that
 * the timeout can be handled. So, in addition to considering libusb's file
 * descriptors in your main event loop, you must also consider that libusb
 * sometimes needs to be called into at fixed points in time even when there
 * is no file descriptor activity.
 *
 * For the details on retrieving the set of file descriptors and determining
 * the next timeout, see the \ref poll "polling and timing" API documentation.
 */

/**
 * @defgroup poll Polling and timing
 *
 * This page documents libusb's functions for polling events and timing.
 * These functions are only necessary for users of the
 * \ref asyncio "asynchronous API". If you are only using the simpler
 * \ref syncio "synchronous API" then you do not need to ever call these
 * functions.
 *
 * The justification for the functionality described here has already been
 * discussed in the \ref asyncevent "event handling" section of the
 * asynchronous API documentation. In summary, libusb does not create internal
 * threads for event processing and hence relies on your application calling
 * into libusb at certain points in time so that pending events can be handled.
 * In order to know precisely when libusb needs to be called into, libusb
 * offers you a set of pollable file descriptors and information about when
 * the next timeout expires.
 *
 * If you are using the asynchronous I/O API, you must take one of the two
 * following options, otherwise your I/O will not complete.
 *
 * \section pollsimple The simple option
 *
 * If your application revolves solely around libusb and does not need to
 * handle other event sources, you can have a program structure as follows:
\code
// initialize libusb
// find and open device
// maybe fire off some initial async I/O

while (user_has_not_requested_exit)
	libusb_handle_events(ctx);

// clean up and exit
\endcode
 *
 * With such a simple main loop, you do not have to worry about managing
 * sets of file descriptors or handling timeouts. libusb_handle_events() will
 * handle those details internally.
 *
 * \section pollmain The more advanced option
 *
 * In more advanced applications, you will already have a main loop which
 * is monitoring other event sources: network sockets, X11 events, mouse
 * movements, etc. Through exposing a set of file descriptors, libusb is
 * designed to cleanly integrate into such main loops.
 *
 * In addition to polling file descriptors for the other event sources, you
 * take a set of file descriptors from libusb and monitor those too. When you
 * detect activity on libusb's file descriptors, you call
 * libusb_handle_events_timeout() in non-blocking mode.
 *
 * You must also consider the fact that libusb sometimes has to handle events
 * at certain known times which do not generate activity on file descriptors.
 * Your main loop must also consider these times, modify it's poll()/select()
 * timeout accordingly, and track time so that libusb_handle_events_timeout()
 * is called in non-blocking mode when timeouts expire.
 *
 * In pseudo-code, you want something that looks like:
\code
// initialise libusb

libusb_get_pollfds(ctx)
while (user has not requested application exit) {
	libusb_get_next_timeout(ctx);
	select(on libusb file descriptors plus any other event sources of interest,
		using a timeout no larger than the value libusb just suggested)
	if (select() indicated activity on libusb file descriptors)
		libusb_handle_events_timeout(ctx, 0);
	if (time has elapsed to or beyond the libusb timeout)
		libusb_handle_events_timeout(ctx, 0);
}

// clean up and exit
\endcode
 *
 * The set of file descriptors that libusb uses as event sources may change
 * during the life of your application. Rather than having to repeatedly
 * call libusb_get_pollfds(), you can set up notification functions for when
 * the file descriptor set changes using libusb_set_pollfd_notifiers().
 *
 * \section mtissues Multi-threaded considerations
 *
 * Unfortunately, the situation is complicated further when multiple threads
 * come into play. If two threads are monitoring the same file descriptors,
 * the fact that only one thread will be woken up when an event occurs causes
 * some headaches.
 *
 * The events lock, event waiters lock, and libusb_handle_events_locked()
 * entities are added to solve these problems. You do not need to be concerned
 * with these entities otherwise.
 *
 * See the extra documentation: \ref mtasync
 */

/** \page mtasync Multi-threaded applications and asynchronous I/O
 *
 * libusb is a thread-safe library, but extra considerations must be applied
 * to applications which interact with libusb from multiple threads.
 *
 * The underlying issue that must be addressed is that all libusb I/O
 * revolves around monitoring file descriptors through the poll()/select()
 * system calls. This is directly exposed at the
 * \ref asyncio "asynchronous interface" but it is important to note that the
 * \ref syncio "synchronous interface" is implemented on top of the
 * asynchonrous interface, therefore the same considerations apply.
 *
 * The issue is that if two or more threads are concurrently calling poll()
 * or select() on libusb's file descriptors then only one of those threads
 * will be woken up when an event arrives. The others will be completely
 * oblivious that anything has happened.
 *
 * Consider the following pseudo-code, which submits an asynchronous transfer
 * then waits for its completion. This style is one way you could implement a
 * synchronous interface on top of the asynchronous interface (and libusb
 * does something similar, albeit more advanced due to the complications
 * explained on this page).
 *
\code
void cb(struct libusb_transfer *transfer)
{
	int *completed = transfer->user_data;
	*completed = 1;
}

void myfunc() {
	struct libusb_transfer *transfer;
	unsigned char buffer[LIBUSB_CONTROL_SETUP_SIZE];
	int completed = 0;

	transfer = libusb_alloc_transfer(0);
	libusb_fill_control_setup(buffer,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, 0x04, 0x01, 0, 0);
	libusb_fill_control_transfer(transfer, dev, buffer, cb, &completed, 1000);
	libusb_submit_transfer(transfer);

	while (!completed) {
		poll(libusb file descriptors, 120*1000);
		if (poll indicates activity)
			libusb_handle_events_timeout(ctx, 0);
	}
	printf("completed!");
	// other code here
}
\endcode
 *
 * Here we are <em>serializing</em> completion of an asynchronous event
 * against a condition - the condition being completion of a specific transfer.
 * The poll() loop has a long timeout to minimize CPU usage during situations
 * when nothing is happening (it could reasonably be unlimited).
 *
 * If this is the only thread that is polling libusb's file descriptors, there
 * is no problem: there is no danger that another thread will swallow up the
 * event that we are interested in. On the other hand, if there is another
 * thread polling the same descriptors, there is a chance that it will receive
 * the event that we were interested in. In this situation, <tt>myfunc()</tt>
 * will only realise that the transfer has completed on the next iteration of
 * the loop, <em>up to 120 seconds later.</em> Clearly a two-minute delay is
 * undesirable, and don't even think about using short timeouts to circumvent
 * this issue!
 * 
 * The solution here is to ensure that no two threads are ever polling the
 * file descriptors at the same time. A naive implementation of this would
 * impact the capabilities of the library, so libusb offers the scheme
 * documented below to ensure no loss of functionality.
 *
 * Before we go any further, it is worth mentioning that all libusb-wrapped
 * event handling procedures fully adhere to the scheme documented below.
 * This includes libusb_handle_events() and all the synchronous I/O functions - 
 * libusb hides this headache from you. You do not need to worry about any
 * of these issues if you stick to that level.
 *
 * The problem is when we consider the fact that libusb exposes file
 * descriptors to allow for you to integrate asynchronous USB I/O into
 * existing main loops, effectively allowing you to do some work behind
 * libusb's back. If you do take libusb's file descriptors and pass them to
 * poll()/select() yourself, you need to be aware of the associated issues.
 *
 * \section eventlock The events lock
 *
 * The first concept to be introduced is the events lock. The events lock
 * is used to serialize threads that want to handle events, such that only
 * one thread is handling events at any one time.
 *
 * You must take the events lock before polling libusb file descriptors,
 * using libusb_lock_events(). You must release the lock as soon as you have
 * aborted your poll()/select() loop, using libusb_unlock_events().
 *
 * \section threadwait Letting other threads do the work for you
 *
 * Although the events lock is a critical part of the solution, it is not
 * enough on it's own. You might wonder if the following is sufficient...
\code
	libusb_lock_events(ctx);
	while (!completed) {
		poll(libusb file descriptors, 120*1000);
		if (poll indicates activity)
			libusb_handle_events_timeout(ctx, 0);
	}
	libusb_unlock_events(ctx);
\endcode
 * ...and the answer is that it is not. This is because the transfer in the
 * code shown above may take a long time (say 30 seconds) to complete, and
 * the lock is not released until the transfer is completed.
 *
 * Another thread with similar code that wants to do event handling may be
 * working with a transfer that completes after a few milliseconds. Despite
 * having such a quick completion time, the other thread cannot check that
 * status of its transfer until the code above has finished (30 seconds later)
 * due to contention on the lock.
 *
 * To solve this, libusb offers you a mechanism to determine when another
 * thread is handling events. It also offers a mechanism to block your thread
 * until the event handling thread has completed an event (and this mechanism
 * does not involve polling of file descriptors).
 *
 * After determining that another thread is currently handling events, you
 * obtain the <em>event waiters</em> lock using libusb_lock_event_waiters().
 * You then re-check that some other thread is still handling events, and if
 * so, you call libusb_wait_for_event().
 *
 * libusb_wait_for_event() puts your application to sleep until an event
 * occurs, or until a thread releases the events lock. When either of these
 * things happen, your thread is woken up, and should re-check the condition
 * it was waiting on. It should also re-check that another thread is handling
 * events, and if not, it should start handling events itself.
 *
 * This looks like the following, as pseudo-code:
\code
retry:
if (libusb_try_lock_events(ctx) == 0) {
	// we obtained the event lock: do our own event handling
	while (!completed) {
		if (!libusb_event_handling_ok(ctx)) {
			libusb_unlock_events(ctx);
			goto retry;
		}
		poll(libusb file descriptors, 120*1000);
		if (poll indicates activity)
			libusb_handle_events_locked(ctx, 0);
	}
	libusb_unlock_events(ctx);
} else {
	// another thread is doing event handling. wait for it to signal us that
	// an event has completed
	libusb_lock_event_waiters(ctx);

	while (!completed) {
		// now that we have the event waiters lock, double check that another
		// thread is still handling events for us. (it may have ceased handling
		// events in the time it took us to reach this point)
		if (!libusb_event_handler_active(ctx)) {
			// whoever was handling events is no longer doing so, try again
			libusb_unlock_event_waiters(ctx);
			goto retry;
		}
	
		libusb_wait_for_event(ctx);
	}
	libusb_unlock_event_waiters(ctx);
}
printf("completed!\n");
\endcode
 *
 * A naive look at the above code may suggest that this can only support
 * one event waiter (hence a total of 2 competing threads, the other doing
 * event handling), because the event waiter seems to have taken the event
 * waiters lock while waiting for an event. However, the system does support
 * multiple event waiters, because libusb_wait_for_event() actually drops
 * the lock while waiting, and reaquires it before continuing.
 *
 * We have now implemented code which can dynamically handle situations where
 * nobody is handling events (so we should do it ourselves), and it can also
 * handle situations where another thread is doing event handling (so we can
 * piggyback onto them). It is also equipped to handle a combination of
 * the two, for example, another thread is doing event handling, but for
 * whatever reason it stops doing so before our condition is met, so we take
 * over the event handling.
 *
 * Four functions were introduced in the above pseudo-code. Their importance
 * should be apparent from the code shown above.
 * -# libusb_try_lock_events() is a non-blocking function which attempts
 *    to acquire the events lock but returns a failure code if it is contended.
 * -# libusb_event_handling_ok() checks that libusb is still happy for your
 *    thread to be performing event handling. Sometimes, libusb needs to
 *    interrupt the event handler, and this is how you can check if you have
 *    been interrupted. If this function returns 0, the correct behaviour is
 *    for you to give up the event handling lock, and then to repeat the cycle.
 *    The following libusb_try_lock_events() will fail, so you will become an
 *    events waiter. For more information on this, read \ref fullstory below.
 * -# libusb_handle_events_locked() is a variant of
 *    libusb_handle_events_timeout() that you can call while holding the
 *    events lock. libusb_handle_events_timeout() itself implements similar
 *    logic to the above, so be sure not to call it when you are
 *    "working behind libusb's back", as is the case here.
 * -# libusb_event_handler_active() determines if someone is currently
 *    holding the events lock
 *
 * You might be wondering why there is no function to wake up all threads
 * blocked on libusb_wait_for_event(). This is because libusb can do this
 * internally: it will wake up all such threads when someone calls
 * libusb_unlock_events() or when a transfer completes (at the point after its
 * callback has returned).
 *
 * \subsection fullstory The full story
 *
 * The above explanation should be enough to get you going, but if you're
 * really thinking through the issues then you may be left with some more
 * questions regarding libusb's internals. If you're curious, read on, and if
 * not, skip to the next section to avoid confusing yourself!
 *
 * The immediate question that may spring to mind is: what if one thread
 * modifies the set of file descriptors that need to be polled while another
 * thread is doing event handling?
 *
 * There are 2 situations in which this may happen.
 * -# libusb_open() will add another file descriptor to the poll set,
 *    therefore it is desirable to interrupt the event handler so that it
 *    restarts, picking up the new descriptor.
 * -# libusb_close() will remove a file descriptor from the poll set. There
 *    are all kinds of race conditions that could arise here, so it is
 *    important that nobody is doing event handling at this time.
 *
 * libusb handles these issues internally, so application developers do not
 * have to stop their event handlers while opening/closing devices. Here's how
 * it works, focusing on the libusb_close() situation first:
 *
 * -# During initialization, libusb opens an internal pipe, and it adds the read
 *    end of this pipe to the set of file descriptors to be polled.
 * -# During libusb_close(), libusb writes some dummy data on this control pipe.
 *    This immediately interrupts the event handler. libusb also records
 *    internally that it is trying to interrupt event handlers for this
 *    high-priority event.
 * -# At this point, some of the functions described above start behaving
 *    differently:
 *   - libusb_event_handling_ok() starts returning 1, indicating that it is NOT
 *     OK for event handling to continue.
 *   - libusb_try_lock_events() starts returning 1, indicating that another
 *     thread holds the event handling lock, even if the lock is uncontended.
 *   - libusb_event_handler_active() starts returning 1, indicating that
 *     another thread is doing event handling, even if that is not true.
 * -# The above changes in behaviour result in the event handler stopping and
 *    giving up the events lock very quickly, giving the high-priority
 *    libusb_close() operation a "free ride" to acquire the events lock. All
 *    threads that are competing to do event handling become event waiters.
 * -# With the events lock held inside libusb_close(), libusb can safely remove
 *    a file descriptor from the poll set, in the safety of knowledge that
 *    nobody is polling those descriptors or trying to access the poll set.
 * -# After obtaining the events lock, the close operation completes very
 *    quickly (usually a matter of milliseconds) and then immediately releases
 *    the events lock.
 * -# At the same time, the behaviour of libusb_event_handling_ok() and friends
 *    reverts to the original, documented behaviour.
 * -# The release of the events lock causes the threads that are waiting for
 *    events to be woken up and to start competing to become event handlers
 *    again. One of them will succeed; it will then re-obtain the list of poll
 *    descriptors, and USB I/O will then continue as normal.
 *
 * libusb_open() is similar, and is actually a more simplistic case. Upon a
 * call to libusb_open():
 *
 * -# The device is opened and a file descriptor is added to the poll set.
 * -# libusb sends some dummy data on the control pipe, and records that it
 *    is trying to modify the poll descriptor set.
 * -# The event handler is interrupted, and the same behaviour change as for
 *    libusb_close() takes effect, causing all event handling threads to become
 *    event waiters.
 * -# The libusb_open() implementation takes its free ride to the events lock.
 * -# Happy that it has successfully paused the events handler, libusb_open()
 *    releases the events lock.
 * -# The event waiter threads are all woken up and compete to become event
 *    handlers again. The one that succeeds will obtain the list of poll
 *    descriptors again, which will include the addition of the new device.
 *
 * \subsection concl Closing remarks
 *
 * The above may seem a little complicated, but hopefully I have made it clear
 * why such complications are necessary. Also, do not forget that this only
 * applies to applications that take libusb's file descriptors and integrate
 * them into their own polling loops.
 *
 * You may decide that it is OK for your multi-threaded application to ignore
 * some of the rules and locks detailed above, because you don't think that
 * two threads can ever be polling the descriptors at the same time. If that
 * is the case, then that's good news for you because you don't have to worry.
 * But be careful here; remember that the synchronous I/O functions do event
 * handling internally. If you have one thread doing event handling in a loop
 * (without implementing the rules and locking semantics documented above)
 * and another trying to send a synchronous USB transfer, you will end up with
 * two threads monitoring the same descriptors, and the above-described
 * undesirable behaviour occuring. The solution is for your polling thread to
 * play by the rules; the synchronous I/O functions do so, and this will result
 * in them getting along in perfect harmony.
 *
 * If you do have a dedicated thread doing event handling, it is perfectly
 * legal for it to take the event handling lock for long periods of time. Any
 * synchronous I/O functions you call from other threads will transparently
 * fall back to the "event waiters" mechanism detailed above. The only
 * consideration that your event handling thread must apply is the one related
 * to libusb_event_handling_ok(): you must call this before every poll(), and
 * give up the events lock if instructed.
 */

int usbi_io_init(struct libusb_context *ctx)
{
	int r;

	pthread_mutex_init(&ctx->flying_transfers_lock, NULL);
	pthread_mutex_init(&ctx->pollfds_lock, NULL);
	pthread_mutex_init(&ctx->pollfd_modify_lock, NULL);
	pthread_mutex_init(&ctx->events_lock, NULL);
	pthread_mutex_init(&ctx->event_waiters_lock, NULL);
	pthread_cond_init(&ctx->event_waiters_cond, NULL);
	list_init(&ctx->flying_transfers);
	list_init(&ctx->pollfds);

	/* FIXME should use an eventfd on kernels that support it */
	r = pipe(ctx->ctrl_pipe);
	if (r < 0)
		return LIBUSB_ERROR_OTHER;

	r = usbi_add_pollfd(ctx, ctx->ctrl_pipe[0], POLLIN);
	if (r < 0)
		return r;

	return 0;
}

void usbi_io_exit(struct libusb_context *ctx)
{
	usbi_remove_pollfd(ctx, ctx->ctrl_pipe[0]);
	close(ctx->ctrl_pipe[0]);
	close(ctx->ctrl_pipe[1]);
}

static int calculate_timeout(struct usbi_transfer *transfer)
{
	int r;
	struct timespec current_time;
	unsigned int timeout =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(transfer)->timeout;

	if (!timeout)
		return 0;

	r = clock_gettime(CLOCK_MONOTONIC, &current_time);
	if (r < 0) {
		usbi_err(ITRANSFER_CTX(transfer),
			"failed to read monotonic clock, errno=%d", errno);
		return r;
	}

	current_time.tv_sec += timeout / 1000;
	current_time.tv_nsec += (timeout % 1000) * 1000000;

	if (current_time.tv_nsec > 1000000000) {
		current_time.tv_nsec -= 1000000000;
		current_time.tv_sec++;
	}

	TIMESPEC_TO_TIMEVAL(&transfer->timeout, &current_time);
	return 0;
}

static void add_to_flying_list(struct usbi_transfer *transfer)
{
	struct usbi_transfer *cur;
	struct timeval *timeout = &transfer->timeout;
	struct libusb_context *ctx = ITRANSFER_CTX(transfer);
	
	pthread_mutex_lock(&ctx->flying_transfers_lock);

	/* if we have no other flying transfers, start the list with this one */
	if (list_empty(&ctx->flying_transfers)) {
		list_add(&transfer->list, &ctx->flying_transfers);
		goto out;
	}

	/* if we have infinite timeout, append to end of list */
	if (!timerisset(timeout)) {
		list_add_tail(&transfer->list, &ctx->flying_transfers);
		goto out;
	}

	/* otherwise, find appropriate place in list */
	list_for_each_entry(cur, &ctx->flying_transfers, list) {
		/* find first timeout that occurs after the transfer in question */
		struct timeval *cur_tv = &cur->timeout;

		if (!timerisset(cur_tv) || (cur_tv->tv_sec > timeout->tv_sec) ||
				(cur_tv->tv_sec == timeout->tv_sec &&
					cur_tv->tv_usec > timeout->tv_usec)) {
			list_add_tail(&transfer->list, &cur->list);
			goto out;
		}
	}

	/* otherwise we need to be inserted at the end */
	list_add_tail(&transfer->list, &ctx->flying_transfers);
out:
	pthread_mutex_unlock(&ctx->flying_transfers_lock);
}

/** \ingroup asyncio
 * Allocate a libusb transfer with a specified number of isochronous packet
 * descriptors. The returned transfer is pre-initialized for you. When the new
 * transfer is no longer needed, it should be freed with
 * libusb_free_transfer().
 *
 * Transfers intended for non-isochronous endpoints (e.g. control, bulk,
 * interrupt) should specify an iso_packets count of zero.
 *
 * For transfers intended for isochronous endpoints, specify an appropriate
 * number of packet descriptors to be allocated as part of the transfer.
 * The returned transfer is not specially initialized for isochronous I/O;
 * you are still required to set the
 * \ref libusb_transfer::num_iso_packets "num_iso_packets" and
 * \ref libusb_transfer::type "type" fields accordingly.
 *
 * It is safe to allocate a transfer with some isochronous packets and then
 * use it on a non-isochronous endpoint. If you do this, ensure that at time
 * of submission, num_iso_packets is 0 and that type is set appropriately.
 *
 * \param iso_packets number of isochronous packet descriptors to allocate
 * \returns a newly allocated transfer, or NULL on error
 */
API_EXPORTED struct libusb_transfer *libusb_alloc_transfer(int iso_packets)
{
	size_t os_alloc_size = usbi_backend->transfer_priv_size
		+ (usbi_backend->add_iso_packet_size * iso_packets);
	int alloc_size = sizeof(struct usbi_transfer)
		+ sizeof(struct libusb_transfer)
		+ (sizeof(struct libusb_iso_packet_descriptor) * iso_packets)
		+ os_alloc_size;
	struct usbi_transfer *itransfer = malloc(alloc_size);
	if (!itransfer)
		return NULL;

	memset(itransfer, 0, alloc_size);
	itransfer->num_iso_packets = iso_packets;
	return __USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
}

/** \ingroup asyncio
 * Free a transfer structure. This should be called for all transfers
 * allocated with libusb_alloc_transfer().
 *
 * If the \ref libusb_transfer_flags::LIBUSB_TRANSFER_FREE_BUFFER
 * "LIBUSB_TRANSFER_FREE_BUFFER" flag is set and the transfer buffer is
 * non-NULL, this function will also free the transfer buffer using the
 * standard system memory allocator (e.g. free()).
 *
 * It is legal to call this function with a NULL transfer. In this case,
 * the function will simply return safely.
 *
 * \param transfer the transfer to free
 */
API_EXPORTED void libusb_free_transfer(struct libusb_transfer *transfer)
{
	struct usbi_transfer *itransfer;
	if (!transfer)
		return;

	if (transfer->flags & LIBUSB_TRANSFER_FREE_BUFFER && transfer->buffer)
		free(transfer->buffer);

	itransfer = __LIBUSB_TRANSFER_TO_USBI_TRANSFER(transfer);
	free(itransfer);
}

/** \ingroup asyncio
 * Submit a transfer. This function will fire off the USB transfer and then
 * return immediately.
 *
 * It is undefined behaviour to submit a transfer that has already been
 * submitted but has not yet completed.
 *
 * \param transfer the transfer to submit
 * \returns 0 on success
 * \returns LIBUSB_ERROR_NO_DEVICE if the device has been disconnected
 * \returns another LIBUSB_ERROR code on other failure
 */
API_EXPORTED int libusb_submit_transfer(struct libusb_transfer *transfer)
{
	struct usbi_transfer *itransfer =
		__LIBUSB_TRANSFER_TO_USBI_TRANSFER(transfer);
	int r;

	itransfer->transferred = 0;
	itransfer->flags = 0;
	r = calculate_timeout(itransfer);
	if (r < 0)
		return LIBUSB_ERROR_OTHER;

	add_to_flying_list(itransfer);
	r = usbi_backend->submit_transfer(itransfer);
	if (r) {
		pthread_mutex_lock(&TRANSFER_CTX(transfer)->flying_transfers_lock);
		list_del(&itransfer->list);
		pthread_mutex_unlock(&TRANSFER_CTX(transfer)->flying_transfers_lock);
	}

	return r;
}

/** \ingroup asyncio
 * Asynchronously cancel a previously submitted transfer.
 * It is undefined behaviour to call this function on a transfer that is
 * already being cancelled or has already completed.
 * This function returns immediately, but this does not indicate cancellation
 * is complete. Your callback function will be invoked at some later time
 * with a transfer status of
 * \ref libusb_transfer_status::LIBUSB_TRANSFER_CANCELLED
 * "LIBUSB_TRANSFER_CANCELLED."
 *
 * \param transfer the transfer to cancel
 * \returns 0 on success
 * \returns a LIBUSB_ERROR code on failure
 */
API_EXPORTED int libusb_cancel_transfer(struct libusb_transfer *transfer)
{
	struct usbi_transfer *itransfer =
		__LIBUSB_TRANSFER_TO_USBI_TRANSFER(transfer);
	int r;

	usbi_dbg("");
	r = usbi_backend->cancel_transfer(itransfer);
	if (r < 0)
		usbi_err(TRANSFER_CTX(transfer),
			"cancel transfer failed error %d", r);
	return r;
}

/* Handle completion of a transfer (completion might be an error condition).
 * This will invoke the user-supplied callback function, which may end up
 * freeing the transfer. Therefore you cannot use the transfer structure
 * after calling this function, and you should free all backend-specific
 * data before calling it. */
void usbi_handle_transfer_completion(struct usbi_transfer *itransfer,
	enum libusb_transfer_status status)
{
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct libusb_context *ctx = TRANSFER_CTX(transfer);
	uint8_t flags;

	pthread_mutex_lock(&ctx->flying_transfers_lock);
	list_del(&itransfer->list);
	pthread_mutex_unlock(&ctx->flying_transfers_lock);

	if (status == LIBUSB_TRANSFER_COMPLETED
			&& transfer->flags & LIBUSB_TRANSFER_SHORT_NOT_OK) {
		int rqlen = transfer->length;
		if (transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL)
			rqlen -= LIBUSB_CONTROL_SETUP_SIZE;
		if (rqlen != itransfer->transferred) {
			usbi_dbg("interpreting short transfer as error");
			status = LIBUSB_TRANSFER_ERROR;
		}
	}

	flags = transfer->flags;
	transfer->status = status;
	transfer->actual_length = itransfer->transferred;
	if (transfer->callback)
		transfer->callback(transfer);
	/* transfer might have been freed by the above call, do not use from
	 * this point. */
	if (flags & LIBUSB_TRANSFER_FREE_TRANSFER)
		libusb_free_transfer(transfer);
	pthread_mutex_lock(&ctx->event_waiters_lock);
	pthread_cond_broadcast(&ctx->event_waiters_cond);
	pthread_mutex_unlock(&ctx->event_waiters_lock);
}

/* Similar to usbi_handle_transfer_completion() but exclusively for transfers
 * that were asynchronously cancelled. The same concerns w.r.t. freeing of
 * transfers exist here.
 */
void usbi_handle_transfer_cancellation(struct usbi_transfer *transfer)
{
	/* if the URB was cancelled due to timeout, report timeout to the user */
	if (transfer->flags & USBI_TRANSFER_TIMED_OUT) {
		usbi_dbg("detected timeout cancellation");
		usbi_handle_transfer_completion(transfer, LIBUSB_TRANSFER_TIMED_OUT);
		return;
	}

	/* otherwise its a normal async cancel */
	usbi_handle_transfer_completion(transfer, LIBUSB_TRANSFER_CANCELLED);
}

/** \ingroup poll
 * Attempt to acquire the event handling lock. This lock is used to ensure that
 * only one thread is monitoring libusb event sources at any one time.
 *
 * You only need to use this lock if you are developing an application
 * which calls poll() or select() on libusb's file descriptors directly.
 * If you stick to libusb's event handling loop functions (e.g.
 * libusb_handle_events()) then you do not need to be concerned with this
 * locking.
 *
 * While holding this lock, you are trusted to actually be handling events.
 * If you are no longer handling events, you must call libusb_unlock_events()
 * as soon as possible.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \returns 0 if the lock was obtained successfully
 * \returns 1 if the lock was not obtained (i.e. another thread holds the lock)
 * \see \ref mtasync
 */
API_EXPORTED int libusb_try_lock_events(libusb_context *ctx)
{
	int r;
	USBI_GET_CONTEXT(ctx);

	/* is someone else waiting to modify poll fds? if so, don't let this thread
	 * start event handling */
	pthread_mutex_lock(&ctx->pollfd_modify_lock);
	r = ctx->pollfd_modify;
	pthread_mutex_unlock(&ctx->pollfd_modify_lock);
	if (r) {
		usbi_dbg("someone else is modifying poll fds");
		return 1;
	}

	r = pthread_mutex_trylock(&ctx->events_lock);
	if (r)
		return 1;

	ctx->event_handler_active = 1;	
	return 0;
}

/** \ingroup poll
 * Acquire the event handling lock, blocking until successful acquisition if
 * it is contended. This lock is used to ensure that only one thread is
 * monitoring libusb event sources at any one time.
 *
 * You only need to use this lock if you are developing an application
 * which calls poll() or select() on libusb's file descriptors directly.
 * If you stick to libusb's event handling loop functions (e.g.
 * libusb_handle_events()) then you do not need to be concerned with this
 * locking.
 *
 * While holding this lock, you are trusted to actually be handling events.
 * If you are no longer handling events, you must call libusb_unlock_events()
 * as soon as possible.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \see \ref mtasync
 */
API_EXPORTED void libusb_lock_events(libusb_context *ctx)
{
	USBI_GET_CONTEXT(ctx);
	pthread_mutex_lock(&ctx->events_lock);
	ctx->event_handler_active = 1;
}

/** \ingroup poll
 * Release the lock previously acquired with libusb_try_lock_events() or
 * libusb_lock_events(). Releasing this lock will wake up any threads blocked
 * on libusb_wait_for_event().
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \see \ref mtasync
 */
API_EXPORTED void libusb_unlock_events(libusb_context *ctx)
{
	USBI_GET_CONTEXT(ctx);
	ctx->event_handler_active = 0;
	pthread_mutex_unlock(&ctx->events_lock);

	/* FIXME: perhaps we should be a bit more efficient by not broadcasting
	 * the availability of the events lock when we are modifying pollfds
	 * (check ctx->pollfd_modify)? */
	pthread_mutex_lock(&ctx->event_waiters_lock);
	pthread_cond_broadcast(&ctx->event_waiters_cond);
	pthread_mutex_unlock(&ctx->event_waiters_lock);
}

/** \ingroup poll
 * Determine if it is still OK for this thread to be doing event handling.
 *
 * Sometimes, libusb needs to temporarily pause all event handlers, and this
 * is the function you should use before polling file descriptors to see if
 * this is the case.
 *
 * If this function instructs your thread to give up the events lock, you
 * should just continue the usual logic that is documented in \ref mtasync.
 * On the next iteration, your thread will fail to obtain the events lock,
 * and will hence become an event waiter.
 *
 * This function should be called while the events lock is held: you don't
 * need to worry about the results of this function if your thread is not
 * the current event handler.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \returns 1 if event handling can start or continue
 * \returns 0 if this thread must give up the events lock
 * \see \ref fullstory "Multi-threaded I/O: the full story"
 */
API_EXPORTED int libusb_event_handling_ok(libusb_context *ctx)
{
	int r;
	USBI_GET_CONTEXT(ctx);

	/* is someone else waiting to modify poll fds? if so, don't let this thread
	 * continue event handling */
	pthread_mutex_lock(&ctx->pollfd_modify_lock);
	r = ctx->pollfd_modify;
	pthread_mutex_unlock(&ctx->pollfd_modify_lock);
	if (r) {
		usbi_dbg("someone else is modifying poll fds");
		return 0;
	}

	return 1;
}


/** \ingroup poll
 * Determine if an active thread is handling events (i.e. if anyone is holding
 * the event handling lock).
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \returns 1 if a thread is handling events
 * \returns 0 if there are no threads currently handling events
 * \see \ref mtasync
 */
API_EXPORTED int libusb_event_handler_active(libusb_context *ctx)
{
	int r;
	USBI_GET_CONTEXT(ctx);

	/* is someone else waiting to modify poll fds? if so, don't let this thread
	 * start event handling -- indicate that event handling is happening */
	pthread_mutex_lock(&ctx->pollfd_modify_lock);
	r = ctx->pollfd_modify;
	pthread_mutex_unlock(&ctx->pollfd_modify_lock);
	if (r) {
		usbi_dbg("someone else is modifying poll fds");
		return 1;
	}

	return ctx->event_handler_active;
}

/** \ingroup poll
 * Acquire the event waiters lock. This lock is designed to be obtained under
 * the situation where you want to be aware when events are completed, but
 * some other thread is event handling so calling libusb_handle_events() is not
 * allowed.
 *
 * You then obtain this lock, re-check that another thread is still handling
 * events, then call libusb_wait_for_event().
 *
 * You only need to use this lock if you are developing an application
 * which calls poll() or select() on libusb's file descriptors directly,
 * <b>and</b> may potentially be handling events from 2 threads simultaenously.
 * If you stick to libusb's event handling loop functions (e.g.
 * libusb_handle_events()) then you do not need to be concerned with this
 * locking.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \see \ref mtasync
 */
API_EXPORTED void libusb_lock_event_waiters(libusb_context *ctx)
{
	USBI_GET_CONTEXT(ctx);
	pthread_mutex_lock(&ctx->event_waiters_lock);
}

/** \ingroup poll
 * Release the event waiters lock.
 * \param ctx the context to operate on, or NULL for the default context
 * \see \ref mtasync
 */
API_EXPORTED void libusb_unlock_event_waiters(libusb_context *ctx)
{
	USBI_GET_CONTEXT(ctx);
	pthread_mutex_unlock(&ctx->event_waiters_lock);
}

/** \ingroup poll
 * Wait for another thread to signal completion of an event. Must be called
 * with the event waiters lock held, see libusb_lock_event_waiters().
 *
 * This function will block until any of the following conditions are met:
 * -# The timeout expires
 * -# A transfer completes
 * -# A thread releases the event handling lock through libusb_unlock_events()
 *
 * Condition 1 is obvious. Condition 2 unblocks your thread <em>after</em>
 * the callback for the transfer has completed. Condition 3 is important
 * because it means that the thread that was previously handling events is no
 * longer doing so, so if any events are to complete, another thread needs to
 * step up and start event handling.
 *
 * This function releases the event waiters lock before putting your thread
 * to sleep, and reacquires the lock as it is being woken up.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \param tv maximum timeout for this blocking function. A NULL value
 * indicates unlimited timeout.
 * \returns 0 after a transfer completes or another thread stops event handling
 * \returns 1 if the timeout expired
 * \see \ref mtasync
 */
API_EXPORTED int libusb_wait_for_event(libusb_context *ctx, struct timeval *tv)
{
	struct timespec timeout;
	int r;

	USBI_GET_CONTEXT(ctx);
	if (tv == NULL) {
		pthread_cond_wait(&ctx->event_waiters_cond, &ctx->event_waiters_lock);
		return 0;
	}

	r = clock_gettime(CLOCK_REALTIME, &timeout);
	if (r < 0) {
		usbi_err(ctx, "failed to read realtime clock, error %d", errno);
		return LIBUSB_ERROR_OTHER;
	}

	timeout.tv_sec += tv->tv_sec;
	timeout.tv_nsec += tv->tv_usec * 1000;
	if (timeout.tv_nsec > 1000000000) {
		timeout.tv_nsec -= 1000000000;
		timeout.tv_sec++;
	}

	r = pthread_cond_timedwait(&ctx->event_waiters_cond,
		&ctx->event_waiters_lock, &timeout);
	return (r == ETIMEDOUT);
}

static void handle_timeout(struct usbi_transfer *itransfer)
{
	struct libusb_transfer *transfer =
		__USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	int r;

	itransfer->flags |= USBI_TRANSFER_TIMED_OUT;
	r = libusb_cancel_transfer(transfer);
	if (r < 0)
		usbi_warn(TRANSFER_CTX(transfer),
			"async cancel failed %d errno=%d", r, errno);
}

static int handle_timeouts(struct libusb_context *ctx)
{
	struct timespec systime_ts;
	struct timeval systime;
	struct usbi_transfer *transfer;
	int r = 0;

	USBI_GET_CONTEXT(ctx);
	pthread_mutex_lock(&ctx->flying_transfers_lock);
	if (list_empty(&ctx->flying_transfers))
		goto out;

	/* get current time */
	r = clock_gettime(CLOCK_MONOTONIC, &systime_ts);
	if (r < 0)
		goto out;

	TIMESPEC_TO_TIMEVAL(&systime, &systime_ts);

	/* iterate through flying transfers list, finding all transfers that
	 * have expired timeouts */
	list_for_each_entry(transfer, &ctx->flying_transfers, list) {
		struct timeval *cur_tv = &transfer->timeout;

		/* if we've reached transfers of infinite timeout, we're all done */
		if (!timerisset(cur_tv))
			goto out;

		/* ignore timeouts we've already handled */
		if (transfer->flags & USBI_TRANSFER_TIMED_OUT)
			continue;

		/* if transfer has non-expired timeout, nothing more to do */
		if ((cur_tv->tv_sec > systime.tv_sec) ||
				(cur_tv->tv_sec == systime.tv_sec &&
					cur_tv->tv_usec > systime.tv_usec))
			goto out;
	
		/* otherwise, we've got an expired timeout to handle */
		handle_timeout(transfer);
	}

out:
	pthread_mutex_unlock(&ctx->flying_transfers_lock);
	return r;
}

/* do the actual event handling. assumes that no other thread is concurrently
 * doing the same thing. */
static int handle_events(struct libusb_context *ctx, struct timeval *tv)
{
	int r;
	struct usbi_pollfd *ipollfd;
	nfds_t nfds = 0;
	struct pollfd *fds;
	int i = -1;
	int timeout_ms;

	pthread_mutex_lock(&ctx->pollfds_lock);
	list_for_each_entry(ipollfd, &ctx->pollfds, list)
		nfds++;

	/* TODO: malloc when number of fd's changes, not on every poll */
	fds = malloc(sizeof(*fds) * nfds);
	if (!fds)
		return LIBUSB_ERROR_NO_MEM;

	list_for_each_entry(ipollfd, &ctx->pollfds, list) {
		struct libusb_pollfd *pollfd = &ipollfd->pollfd;
		int fd = pollfd->fd;
		i++;
		fds[i].fd = fd;
		fds[i].events = pollfd->events;
		fds[i].revents = 0;
	}
	pthread_mutex_unlock(&ctx->pollfds_lock);

	timeout_ms = (tv->tv_sec * 1000) + (tv->tv_usec / 1000);

	/* round up to next millisecond */
	if (tv->tv_usec % 1000)
		timeout_ms++;

	usbi_dbg("poll() %d fds with timeout in %dms", nfds, timeout_ms);
	r = poll(fds, nfds, timeout_ms);
	usbi_dbg("poll() returned %d", r);
	if (r == 0) {
		free(fds);
		return handle_timeouts(ctx);
	} else if (r == -1 && errno == EINTR) {
		free(fds);
		return LIBUSB_ERROR_INTERRUPTED;
	} else if (r < 0) {
		free(fds);
		usbi_err(ctx, "poll failed %d err=%d\n", r, errno);
		return LIBUSB_ERROR_IO;
	}

	/* fd[0] is always the ctrl pipe */
	if (fds[0].revents) {
		/* another thread wanted to interrupt event handling, and it succeeded!
		 * handle any other events that cropped up at the same time, and
		 * simply return */
		usbi_dbg("caught a fish on the control pipe");

		if (r == 1) {
			r = 0;
			goto handled;
		} else {
			/* prevent OS backend from trying to handle events on ctrl pipe */
			fds[0].revents = 0;
		}
	}

	r = usbi_backend->handle_events(ctx, fds, nfds, r);
	if (r)
		usbi_err(ctx, "backend handle_events failed with error %d", r);

handled:
	free(fds);
	return r;
}

/* returns the smallest of:
 *  1. timeout of next URB
 *  2. user-supplied timeout
 * returns 1 if there is an already-expired timeout, otherwise returns 0
 * and populates out
 */
static int get_next_timeout(libusb_context *ctx, struct timeval *tv,
	struct timeval *out)
{
	struct timeval timeout;
	int r = libusb_get_next_timeout(ctx, &timeout);
	if (r) {
		/* timeout already expired? */
		if (!timerisset(&timeout))
			return 1;

		/* choose the smallest of next URB timeout or user specified timeout */
		if (timercmp(&timeout, tv, <))
			*out = timeout;
		else
			*out = *tv;
	} else {
		*out = *tv;
	}
	return 0;
}

/** \ingroup poll
 * Handle any pending events.
 *
 * libusb determines "pending events" by checking if any timeouts have expired
 * and by checking the set of file descriptors for activity.
 *
 * If a zero timeval is passed, this function will handle any already-pending
 * events and then immediately return in non-blocking style.
 *
 * If a non-zero timeval is passed and no events are currently pending, this
 * function will block waiting for events to handle up until the specified
 * timeout. If an event arrives or a signal is raised, this function will
 * return early.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \param tv the maximum time to block waiting for events, or zero for
 * non-blocking mode
 * \returns 0 on success, or a LIBUSB_ERROR code on failure
 */
API_EXPORTED int libusb_handle_events_timeout(libusb_context *ctx,
	struct timeval *tv)
{
	int r;
	struct timeval poll_timeout;

	USBI_GET_CONTEXT(ctx);
	r = get_next_timeout(ctx, tv, &poll_timeout);
	if (r) {
		/* timeout already expired */
		return handle_timeouts(ctx);
	}

retry:
	if (libusb_try_lock_events(ctx) == 0) {
		/* we obtained the event lock: do our own event handling */
		r = handle_events(ctx, &poll_timeout);
		libusb_unlock_events(ctx);
		return r;
	}

	/* another thread is doing event handling. wait for pthread events that
	 * notify event completion. */
	libusb_lock_event_waiters(ctx);

	if (!libusb_event_handler_active(ctx)) {
		/* we hit a race: whoever was event handling earlier finished in the
		 * time it took us to reach this point. try the cycle again. */
		libusb_unlock_event_waiters(ctx);
		usbi_dbg("event handler was active but went away, retrying");
		goto retry;
	}

	usbi_dbg("another thread is doing event handling");
	r = libusb_wait_for_event(ctx, &poll_timeout);
	libusb_unlock_event_waiters(ctx);

	if (r < 0)
		return r;
	else if (r == 1)
		return handle_timeouts(ctx);
	else
		return 0;
}

/** \ingroup poll
 * Handle any pending events in blocking mode with a sensible timeout. This
 * timeout is currently hardcoded at 2 seconds but we may change this if we
 * decide other values are more sensible. For finer control over whether this
 * function is blocking or non-blocking, or the maximum timeout, use
 * libusb_handle_events_timeout() instead.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \returns 0 on success, or a LIBUSB_ERROR code on failure
 */
API_EXPORTED int libusb_handle_events(libusb_context *ctx)
{
	struct timeval tv;
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	return libusb_handle_events_timeout(ctx, &tv);
}

/** \ingroup poll
 * Handle any pending events by polling file descriptors, without checking if
 * any other threads are already doing so. Must be called with the event lock
 * held, see libusb_lock_events().
 *
 * This function is designed to be called under the situation where you have
 * taken the event lock and are calling poll()/select() directly on libusb's
 * file descriptors (as opposed to using libusb_handle_events() or similar).
 * You detect events on libusb's descriptors, so you then call this function
 * with a zero timeout value (while still holding the event lock).
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \param tv the maximum time to block waiting for events, or zero for
 * non-blocking mode
 * \returns 0 on success, or a LIBUSB_ERROR code on failure
 * \see \ref mtasync
 */
API_EXPORTED int libusb_handle_events_locked(libusb_context *ctx,
	struct timeval *tv)
{
	int r;
	struct timeval poll_timeout;

	USBI_GET_CONTEXT(ctx);
	r = get_next_timeout(ctx, tv, &poll_timeout);
	if (r) {
		/* timeout already expired */
		return handle_timeouts(ctx);
	}

	return handle_events(ctx, &poll_timeout);
}

/** \ingroup poll
 * Determine the next internal timeout that libusb needs to handle. You only
 * need to use this function if you are calling poll() or select() or similar
 * on libusb's file descriptors yourself - you do not need to use it if you
 * are calling libusb_handle_events() or a variant directly.
 * 
 * You should call this function in your main loop in order to determine how
 * long to wait for select() or poll() to return results. libusb needs to be
 * called into at this timeout, so you should use it as an upper bound on
 * your select() or poll() call.
 *
 * When the timeout has expired, call into libusb_handle_events_timeout()
 * (perhaps in non-blocking mode) so that libusb can handle the timeout.
 *
 * This function may return 1 (success) and an all-zero timeval. If this is
 * the case, it indicates that libusb has a timeout that has already expired
 * so you should call libusb_handle_events_timeout() or similar immediately.
 * A return code of 0 indicates that there are no pending timeouts.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \param tv output location for a relative time against the current
 * clock in which libusb must be called into in order to process timeout events
 * \returns 0 if there are no pending timeouts, 1 if a timeout was returned,
 * or LIBUSB_ERROR_OTHER on failure
 */
API_EXPORTED int libusb_get_next_timeout(libusb_context *ctx,
	struct timeval *tv)
{
	struct usbi_transfer *transfer;
	struct timespec cur_ts;
	struct timeval cur_tv;
	struct timeval *next_timeout;
	int r;
	int found = 0;

	USBI_GET_CONTEXT(ctx);
	pthread_mutex_lock(&ctx->flying_transfers_lock);
	if (list_empty(&ctx->flying_transfers)) {
		pthread_mutex_unlock(&ctx->flying_transfers_lock);
		usbi_dbg("no URBs, no timeout!");
		return 0;
	}

	/* find next transfer which hasn't already been processed as timed out */
	list_for_each_entry(transfer, &ctx->flying_transfers, list) {
		if (!(transfer->flags & USBI_TRANSFER_TIMED_OUT)) {
			found = 1;
			break;
		}
	}
	pthread_mutex_unlock(&ctx->flying_transfers_lock);

	if (!found) {
		usbi_dbg("all URBs have already been processed for timeouts");
		return 0;
	}

	next_timeout = &transfer->timeout;

	/* no timeout for next transfer */
	if (!timerisset(next_timeout)) {
		usbi_dbg("no URBs with timeouts, no timeout!");
		return 0;
	}

	r = clock_gettime(CLOCK_MONOTONIC, &cur_ts);
	if (r < 0) {
		usbi_err(ctx, "failed to read monotonic clock, errno=%d", errno);
		return LIBUSB_ERROR_OTHER;
	}
	TIMESPEC_TO_TIMEVAL(&cur_tv, &cur_ts);

	if (timercmp(&cur_tv, next_timeout, >=)) {
		usbi_dbg("first timeout already expired");
		timerclear(tv);
	} else {
		timersub(next_timeout, &cur_tv, tv);
		usbi_dbg("next timeout in %d.%06ds", tv->tv_sec, tv->tv_usec);
	}

	return 1;
}

/** \ingroup poll
 * Register notification functions for file descriptor additions/removals.
 * These functions will be invoked for every new or removed file descriptor
 * that libusb uses as an event source.
 *
 * To remove notifiers, pass NULL values for the function pointers.
 *
 * Note that file descriptors may have been added even before you register
 * these notifiers (e.g. at libusb_init() time).
 *
 * Additionally, note that the removal notifier may be called during
 * libusb_exit() (e.g. when it is closing file descriptors that were opened
 * and added to the poll set at libusb_init() time). If you don't want this,
 * remove the notifiers immediately before calling libusb_exit().
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \param added_cb pointer to function for addition notifications
 * \param removed_cb pointer to function for removal notifications
 * \param user_data User data to be passed back to callbacks (useful for
 * passing context information)
 */
API_EXPORTED void libusb_set_pollfd_notifiers(libusb_context *ctx,
	libusb_pollfd_added_cb added_cb, libusb_pollfd_removed_cb removed_cb,
	void *user_data)
{
	USBI_GET_CONTEXT(ctx);
	ctx->fd_added_cb = added_cb;
	ctx->fd_removed_cb = removed_cb;
	ctx->fd_cb_user_data = user_data;
}

/* Add a file descriptor to the list of file descriptors to be monitored.
 * events should be specified as a bitmask of events passed to poll(), e.g.
 * POLLIN and/or POLLOUT. */
int usbi_add_pollfd(struct libusb_context *ctx, int fd, short events)
{
	struct usbi_pollfd *ipollfd = malloc(sizeof(*ipollfd));
	if (!ipollfd)
		return LIBUSB_ERROR_NO_MEM;

	usbi_dbg("add fd %d events %d", fd, events);
	ipollfd->pollfd.fd = fd;
	ipollfd->pollfd.events = events;
	pthread_mutex_lock(&ctx->pollfds_lock);
	list_add_tail(&ipollfd->list, &ctx->pollfds);
	pthread_mutex_unlock(&ctx->pollfds_lock);

	if (ctx->fd_added_cb)
		ctx->fd_added_cb(fd, events, ctx->fd_cb_user_data);
	return 0;
}

/* Remove a file descriptor from the list of file descriptors to be polled. */
void usbi_remove_pollfd(struct libusb_context *ctx, int fd)
{
	struct usbi_pollfd *ipollfd;
	int found = 0;

	usbi_dbg("remove fd %d", fd);
	pthread_mutex_lock(&ctx->pollfds_lock);
	list_for_each_entry(ipollfd, &ctx->pollfds, list)
		if (ipollfd->pollfd.fd == fd) {
			found = 1;
			break;
		}

	if (!found) {
		usbi_dbg("couldn't find fd %d to remove", fd);
		pthread_mutex_unlock(&ctx->pollfds_lock);
		return;
	}

	list_del(&ipollfd->list);
	pthread_mutex_unlock(&ctx->pollfds_lock);
	free(ipollfd);
	if (ctx->fd_removed_cb)
		ctx->fd_removed_cb(fd, ctx->fd_cb_user_data);
}

/** \ingroup poll
 * Retrieve a list of file descriptors that should be polled by your main loop
 * as libusb event sources.
 *
 * The returned list is NULL-terminated and should be freed with free() when
 * done. The actual list contents must not be touched.
 *
 * \param ctx the context to operate on, or NULL for the default context
 * \returns a NULL-terminated list of libusb_pollfd structures, or NULL on
 * error
 */
API_EXPORTED const struct libusb_pollfd **libusb_get_pollfds(
	libusb_context *ctx)
{
	struct libusb_pollfd **ret = NULL;
	struct usbi_pollfd *ipollfd;
	size_t i = 0;
	size_t cnt = 0;
	USBI_GET_CONTEXT(ctx);

	pthread_mutex_lock(&ctx->pollfds_lock);
	list_for_each_entry(ipollfd, &ctx->pollfds, list)
		cnt++;

	ret = calloc(cnt + 1, sizeof(struct libusb_pollfd *));
	if (!ret)
		goto out;

	list_for_each_entry(ipollfd, &ctx->pollfds, list)
		ret[i++] = (struct libusb_pollfd *) ipollfd;
	ret[cnt] = NULL;

out:
	pthread_mutex_unlock(&ctx->pollfds_lock);
	return (const struct libusb_pollfd **) ret;
}

/* Backends call this from handle_events to report disconnection of a device.
 * The transfers get cancelled appropriately.
 */
void usbi_handle_disconnect(struct libusb_device_handle *handle)
{
	struct usbi_transfer *cur;
	struct usbi_transfer *to_cancel;

	usbi_dbg("device %d.%d",
		handle->dev->bus_number, handle->dev->device_address);

	/* terminate all pending transfers with the LIBUSB_TRANSFER_NO_DEVICE
	 * status code.
	 * 
	 * this is a bit tricky because:
	 * 1. we can't do transfer completion while holding flying_transfers_lock
	 * 2. the transfers list can change underneath us - if we were to build a
	 *    list of transfers to complete (while holding look), the situation
	 *    might be different by the time we come to free them
	 *
	 * so we resort to a loop-based approach as below
	 * FIXME: is this still potentially racy?
	 */

	while (1) {
		pthread_mutex_lock(&HANDLE_CTX(handle)->flying_transfers_lock);
		to_cancel = NULL;
		list_for_each_entry(cur, &HANDLE_CTX(handle)->flying_transfers, list)
			if (__USBI_TRANSFER_TO_LIBUSB_TRANSFER(cur)->dev_handle == handle) {
				to_cancel = cur;
				break;
			}
		pthread_mutex_unlock(&HANDLE_CTX(handle)->flying_transfers_lock);

		if (!to_cancel)
			break;

		usbi_backend->clear_transfer_priv(to_cancel);
		usbi_handle_transfer_completion(to_cancel, LIBUSB_TRANSFER_NO_DEVICE);
	}

}


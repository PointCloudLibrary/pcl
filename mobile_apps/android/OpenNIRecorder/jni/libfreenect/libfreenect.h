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

#ifndef LIBFREENECT_H
#define LIBFREENECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FREENECT_COUNTS_PER_G 819 /**< Ticks per G for accelerometer as set per http://www.kionix.com/Product%20Sheets/KXSD9%20Product%20Brief.pdf */

/// Flags representing devices to open when freenect_open_device() is called.
/// In particular, this allows libfreenect to grab only a subset of the devices
/// in the Kinect, so you could (for instance) use libfreenect to handle audio
/// and motor support while letting OpenNI have access to the cameras.
/// If a device is not supported on a particular platform, its flag will be ignored.
typedef enum {
	FREENECT_DEVICE_MOTOR  = 0x01,
	FREENECT_DEVICE_CAMERA = 0x02,
	FREENECT_DEVICE_AUDIO  = 0x04,
} freenect_device_flags;

/// Enumeration of available resolutions.
/// Not all available resolutions are actually supported for all video formats.
/// Frame modes may not perfectly match resolutions.  For instance,
/// FREENECT_RESOLUTION_MEDIUM is 640x488 for the IR camera.
typedef enum {
	FREENECT_RESOLUTION_LOW    = 0, /**< QVGA - 320x240 */
	FREENECT_RESOLUTION_MEDIUM = 1, /**< VGA  - 640x480 */
	FREENECT_RESOLUTION_HIGH   = 2, /**< SXGA - 1280x1024 */
	FREENECT_RESOLUTION_DUMMY  = 2147483647, /**< Dummy value to force enum to be 32 bits wide */
} freenect_resolution;

/// Enumeration of video frame information states.
/// See http://openkinect.org/wiki/Protocol_Documentation#RGB_Camera for more information.
typedef enum {
	FREENECT_VIDEO_RGB             = 0, /**< Decompressed RGB mode (demosaicing done by libfreenect) */
	FREENECT_VIDEO_BAYER           = 1, /**< Bayer compressed mode (raw information from camera) */
	FREENECT_VIDEO_IR_8BIT         = 2, /**< 8-bit IR mode  */
	FREENECT_VIDEO_IR_10BIT        = 3, /**< 10-bit IR mode */
	FREENECT_VIDEO_IR_10BIT_PACKED = 4, /**< 10-bit packed IR mode */
	FREENECT_VIDEO_YUV_RGB         = 5, /**< YUV RGB mode */
	FREENECT_VIDEO_YUV_RAW         = 6, /**< YUV Raw mode */
	FREENECT_VIDEO_DUMMY           = 2147483647, /**< Dummy value to force enum to be 32 bits wide */
} freenect_video_format;

/// Enumeration of depth frame states
/// See http://openkinect.org/wiki/Protocol_Documentation#RGB_Camera for more information.
typedef enum {
	FREENECT_DEPTH_11BIT        = 0, /**< 11 bit depth information in one uint16_t/pixel */
	FREENECT_DEPTH_10BIT        = 1, /**< 10 bit depth information in one uint16_t/pixel */
	FREENECT_DEPTH_11BIT_PACKED = 2, /**< 11 bit packed depth information */
	FREENECT_DEPTH_10BIT_PACKED = 3, /**< 10 bit packed depth information */
	FREENECT_DEPTH_DUMMY        = 2147483647, /**< Dummy value to force enum to be 32 bits wide */
} freenect_depth_format;

/// Structure to give information about the width, height, bitrate,
/// framerate, and buffer size of a frame in a particular mode, as
/// well as the total number of bytes needed to hold a single frame.
typedef struct {
	uint32_t reserved;              /**< unique ID used internally.  The meaning of values may change without notice.  Don't touch or depend on the contents of this field.  We mean it. */
	freenect_resolution resolution; /**< Resolution this freenect_frame_mode describes, should you want to find it again with freenect_find_*_frame_mode(). */
	union {
		int32_t dummy;
		freenect_video_format video_format;
		freenect_depth_format depth_format;
	};                              /**< The video or depth format that this freenect_frame_mode describes.  The caller should know which of video_format or depth_format to use, since they called freenect_get_*_frame_mode() */
	int32_t bytes;                  /**< Total buffer size in bytes to hold a single frame of data.  Should be equivalent to width * height * (data_bits_per_pixel+padding_bits_per_pixel) / 8 */
	int16_t width;                  /**< Width of the frame, in pixels */
	int16_t height;                 /**< Height of the frame, in pixels */
	int8_t data_bits_per_pixel;     /**< Number of bits of information needed for each pixel */
	int8_t padding_bits_per_pixel;  /**< Number of bits of padding for alignment used for each pixel */
	int8_t framerate;               /**< Approximate expected frame rate, in Hz */
	int8_t is_valid;                /**< If 0, this freenect_frame_mode is invalid and does not describe a supported mode.  Otherwise, the frame_mode is valid. */
} freenect_frame_mode;

/// Enumeration of LED states
/// See http://openkinect.org/wiki/Protocol_Documentation#Setting_LED for more information.
typedef enum {
	LED_OFF              = 0, /**< Turn LED off */
	LED_GREEN            = 1, /**< Turn LED to Green */
	LED_RED              = 2, /**< Turn LED to Red */
	LED_YELLOW           = 3, /**< Turn LED to Yellow */
	LED_BLINK_GREEN      = 4, /**< Make LED blink Green */
	// 5 is same as 4, LED blink Green
	LED_BLINK_RED_YELLOW = 6, /**< Make LED blink Red/Yellow */
} freenect_led_options;


/// Enumeration of tilt motor status
typedef enum {
	TILT_STATUS_STOPPED = 0x00, /**< Tilt motor is stopped */
	TILT_STATUS_LIMIT   = 0x01, /**< Tilt motor has reached movement limit */
	TILT_STATUS_MOVING  = 0x04, /**< Tilt motor is currently moving to new position */
} freenect_tilt_status_code;

/// Data from the tilt motor and accelerometer
typedef struct {
	int16_t                   accelerometer_x; /**< Raw accelerometer data for X-axis, see FREENECT_COUNTS_PER_G for conversion */
	int16_t                   accelerometer_y; /**< Raw accelerometer data for Y-axis, see FREENECT_COUNTS_PER_G for conversion */
	int16_t                   accelerometer_z; /**< Raw accelerometer data for Z-axis, see FREENECT_COUNTS_PER_G for conversion */
	int8_t                    tilt_angle;      /**< Raw tilt motor angle encoder information */
	freenect_tilt_status_code tilt_status;     /**< State of the tilt motor (stopped, moving, etc...) */
} freenect_raw_tilt_state;

struct _freenect_context;
typedef struct _freenect_context freenect_context; /**< Holds information about the usb context. */

struct _freenect_device;
typedef struct _freenect_device freenect_device; /**< Holds device information. */

// usb backend specific section
#ifdef _WIN32
  /* frees Windows users of the burden of specifying the path to <libusb-1.0/libusb.h> */
  typedef void freenect_usb_context;
#else
  #include "libusb.h"
  typedef libusb_context freenect_usb_context; /**< Holds libusb-1.0 specific information */
#endif
//

/// If Win32, export all functions for DLL usage
#ifndef _WIN32
  #define FREENECTAPI /**< DLLExport information for windows, set to nothing on other platforms */
#else
  /**< DLLExport information for windows, set to nothing on other platforms */
  #ifdef __cplusplus
    #define FREENECTAPI extern "C" __declspec(dllexport)
  #else
    // this is required when building from a Win32 port of gcc without being
    // forced to compile all of the library files (.c) with g++...
    #define FREENECTAPI __declspec(dllexport)
  #endif
#endif

/// Enumeration of message logging levels
typedef enum {
	FREENECT_LOG_FATAL = 0,     /**< Log for crashing/non-recoverable errors */
	FREENECT_LOG_ERROR,         /**< Log for major errors */
	FREENECT_LOG_WARNING,       /**< Log for warning messages */
	FREENECT_LOG_NOTICE,        /**< Log for important messages */
	FREENECT_LOG_INFO,          /**< Log for normal messages */
	FREENECT_LOG_DEBUG,         /**< Log for useful development messages */
	FREENECT_LOG_SPEW,          /**< Log for slightly less useful messages */
	FREENECT_LOG_FLOOD,         /**< Log EVERYTHING. May slow performance. */
} freenect_loglevel;

/**
 * Initialize a freenect context and do any setup required for
 * platform specific USB libraries.
 *
 * @param ctx Address of pointer to freenect context struct to allocate and initialize
 * @param usb_ctx USB context to initialize. Can be NULL if not using multiple contexts.
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_init(freenect_context **ctx, freenect_usb_context *usb_ctx);

/**
 * Closes the device if it is open, and frees the context
 *
 * @param ctx freenect context to close/free
 *
 * @return 0 on success
 */
FREENECTAPI int freenect_shutdown(freenect_context *ctx);

/// Typedef for logging callback functions
typedef void (*freenect_log_cb)(freenect_context *dev, freenect_loglevel level, const char *msg);

/**
 * Set the log level for the specified freenect context
 *
 * @param ctx context to set log level for
 * @param level log level to use (see freenect_loglevel enum)
 */
FREENECTAPI void freenect_set_log_level(freenect_context *ctx, freenect_loglevel level);

/**
 * Callback for log messages (i.e. for rerouting to a file instead of
 * stdout)
 *
 * @param ctx context to set log callback for
 * @param cb callback function pointer
 */
FREENECTAPI void freenect_set_log_callback(freenect_context *ctx, freenect_log_cb cb);

/**
 * Calls the platform specific usb event processor
 *
 * @param ctx context to process events for
 *
 * @return 0 on success, other values on error, platform/library dependant
 */
FREENECTAPI int freenect_process_events(freenect_context *ctx);

/**
 * Return the number of kinect devices currently connected to the
 * system
 *
 * @param ctx Context to access device count through
 *
 * @return Number of devices connected, < 0 on error
 */
FREENECTAPI int freenect_num_devices(freenect_context *ctx);

/**
 * Set which subdevices any subsequent calls to freenect_open_device()
 * should open.  This will not affect devices which have already been
 * opened.  The default behavior, should you choose not to call this
 * function at all, is to open all supported subdevices - motor, cameras,
 * and audio, if supported on the platform.
 *
 * @param ctx Context to set future subdevice selection for
 * @param subdevs Flags representing the subdevices to select
 */
FREENECTAPI void freenect_select_subdevices(freenect_context *ctx, freenect_device_flags subdevs);

/**
 * Opens a kinect device via a context. Index specifies the index of
 * the device on the current state of the bus. Bus resets may cause
 * indexes to shift.
 *
 * @param ctx Context to open device through
 * @param dev Device structure to assign opened device to
 * @param index Index of the device on the bus
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_open_device(freenect_context *ctx, freenect_device **dev, int index);

/**
 * Closes a device that is currently open
 *
 * @param dev Device to close
 *
 * @return 0 on success
 */
FREENECTAPI int freenect_close_device(freenect_device *dev);

/**
 * Set the device user data, for passing generic information into
 * callbacks
 *
 * @param dev Device to attach user data to
 * @param user User data to attach
 */
FREENECTAPI void freenect_set_user(freenect_device *dev, void *user);

/**
 * Retrieve the pointer to user data from the device struct
 *
 * @param dev Device from which to get user data
 *
 * @return Pointer to user data
 */
FREENECTAPI void *freenect_get_user(freenect_device *dev);

/// Typedef for depth image received event callbacks
typedef void (*freenect_depth_cb)(freenect_device *dev, void *depth, uint32_t timestamp);
/// Typedef for video image received event callbacks
typedef void (*freenect_video_cb)(freenect_device *dev, void *video, uint32_t timestamp);

/**
 * Set callback for depth information received event
 *
 * @param dev Device to set callback for
 * @param cb Function pointer for processing depth information
 */
FREENECTAPI void freenect_set_depth_callback(freenect_device *dev, freenect_depth_cb cb);

/**
 * Set callback for video information received event
 *
 * @param dev Device to set callback for
 * @param cb Function pointer for processing video information
 */
FREENECTAPI void freenect_set_video_callback(freenect_device *dev, freenect_video_cb cb);

/**
 * Set the buffer to store depth information to. Size of buffer is
 * dependant on depth format. See FREENECT_DEPTH_*_SIZE defines for
 * more information.
 *
 * @param dev Device to set depth buffer for.
 * @param buf Buffer to store depth information to.
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_set_depth_buffer(freenect_device *dev, void *buf);

/**
 * Set the buffer to store depth information to. Size of buffer is
 * dependant on video format. See FREENECT_VIDEO_*_SIZE defines for
 * more information.
 *
 * @param dev Device to set video buffer for.
 * @param buf Buffer to store video information to.
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_set_video_buffer(freenect_device *dev, void *buf);

/**
 * Start the depth information stream for a device.
 *
 * @param dev Device to start depth information stream for.
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_start_depth(freenect_device *dev);

/**
 * Start the video information stream for a device.
 *
 * @param dev Device to start video information stream for.
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_start_video(freenect_device *dev);

/**
 * Stop the depth information stream for a device
 *
 * @param dev Device to stop depth information stream on.
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_stop_depth(freenect_device *dev);

/**
 * Stop the video information stream for a device
 *
 * @param dev Device to stop video information stream on.
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_stop_video(freenect_device *dev);

/**
 * Updates the accelerometer state using a blocking control message
 * call.
 *
 * @param dev Device to get accelerometer data from
 *
 * @return 0 on success, < 0 on error. Accelerometer data stored to
 * device struct.
 */
FREENECTAPI int freenect_update_tilt_state(freenect_device *dev);

/**
 * Retrieve the tilt state from a device
 *
 * @param dev Device to retrieve tilt state from
 *
 * @return The tilt state struct of the device
 */
FREENECTAPI freenect_raw_tilt_state* freenect_get_tilt_state(freenect_device *dev);

/**
 * Return the tilt state, in degrees with respect to the horizon
 *
 * @param state The tilt state struct from a device
 *
 * @return Current degree of tilt of the device
 */
FREENECTAPI double freenect_get_tilt_degs(freenect_raw_tilt_state *state);

/**
 * Set the tilt state of the device, in degrees with respect to the
 * horizon. Uses blocking control message call to update
 * device. Function return does not reflect state of device, device
 * may still be moving to new position after the function returns. Use
 * freenect_get_tilt_status() to find current movement state.
 *
 * @param dev Device to set tilt state
 * @param angle Angle the device should tilt to
 *
 * @return 0 on success, < 0 on error.
 */
FREENECTAPI int freenect_set_tilt_degs(freenect_device *dev, double angle);

/**
 * Return the movement state of the tilt motor (moving, stopped, etc...)
 *
 * @param state Raw state struct to get the tilt status code from
 *
 * @return Status code of the tilt device. See
 * freenect_tilt_status_code enum for more info.
 */
FREENECTAPI freenect_tilt_status_code freenect_get_tilt_status(freenect_raw_tilt_state *state);

/**
 * Set the state of the LED. Uses blocking control message call to
 * update device.
 *
 * @param dev Device to set the LED state
 * @param option LED state to set on device. See freenect_led_options enum.
 *
 * @return 0 on success, < 0 on error
 */
FREENECTAPI int freenect_set_led(freenect_device *dev, freenect_led_options option);

/**
 * Get the axis-based gravity adjusted accelerometer state, as laid
 * out via the accelerometer data sheet, which is available at
 *
 * http://www.kionix.com/Product%20Sheets/KXSD9%20Product%20Brief.pdf
 *
 * @param state State to extract accelerometer data from
 * @param x Stores X-axis accelerometer state
 * @param y Stores Y-axis accelerometer state
 * @param z Stores Z-axis accelerometer state
 */
FREENECTAPI void freenect_get_mks_accel(freenect_raw_tilt_state *state, double* x, double* y, double* z);

/**
 * Get the number of video camera modes supported by the driver.  This includes both RGB and IR modes.
 *
 * @return Number of video modes supported by the driver
 */
FREENECTAPI int freenect_get_video_mode_count();

/**
 * Get the frame descriptor of the nth supported video mode for the
 * video camera.
 *
 * @param n Which of the supported modes to return information about
 *
 * @return A freenect_frame_mode describing the nth video mode
 */
FREENECTAPI const freenect_frame_mode freenect_get_video_mode(int mode_num);

/**
 * Get the frame descriptor of the current video mode for the specified
 * freenect device.
 *
 * @param dev Which device to return the currently-set video mode for
 *
 * @return A freenect_frame_mode describing the current video mode of the specified device
 */
FREENECTAPI const freenect_frame_mode freenect_get_current_video_mode(freenect_device *dev);

/**
 * Convenience function to return a mode descriptor matching the
 * specified resolution and video camera pixel format, if one exists.
 *
 * @param res Resolution desired
 * @param fmt Pixel format desired
 *
 * @return A freenect_frame_mode that matches the arguments specified, if such a valid mode exists; otherwise, an invalid freenect_frame_mode.
 */
FREENECTAPI const freenect_frame_mode freenect_find_video_mode(freenect_resolution res, freenect_video_format fmt);

/**
 * Sets the current video mode for the specified device.  If the
 * freenect_frame_mode specified is not one provided by the driver
 * e.g. from freenect_get_video_mode() or freenect_find_video_mode()
 * then behavior is undefined.  The current video mode cannot be
 * changed while streaming is active.
 *
 * @param dev Device for which to set the video mode
 * @param mode Frame mode to set
 *
 * @return 0 on success, < 0 if error
 */
FREENECTAPI int freenect_set_video_mode(freenect_device* dev, const freenect_frame_mode mode);

/**
 * Get the number of depth camera modes supported by the driver.  This includes both RGB and IR modes.
 *
 * @return Number of depth modes supported by the driver
 */
FREENECTAPI int freenect_get_depth_mode_count();

/**
 * Get the frame descriptor of the nth supported depth mode for the
 * depth camera.
 *
 * @param n Which of the supported modes to return information about
 *
 * @return A freenect_frame_mode describing the nth depth mode
 */
FREENECTAPI const freenect_frame_mode freenect_get_depth_mode(int mode_num);

/**
 * Get the frame descriptor of the current depth mode for the specified
 * freenect device.
 *
 * @param dev Which device to return the currently-set depth mode for
 *
 * @return A freenect_frame_mode describing the current depth mode of the specified device
 */
FREENECTAPI const freenect_frame_mode freenect_get_current_depth_mode(freenect_device *dev);

/**
 * Convenience function to return a mode descriptor matching the
 * specified resolution and depth camera pixel format, if one exists.
 *
 * @param res Resolution desired
 * @param fmt Pixel format desired
 *
 * @return A freenect_frame_mode that matches the arguments specified, if such a valid mode exists; otherwise, an invalid freenect_frame_mode.
 */
FREENECTAPI const freenect_frame_mode freenect_find_depth_mode(freenect_resolution res, freenect_depth_format fmt);

/**
 * Sets the current depth mode for the specified device.  The mode
 * cannot be changed while streaming is active.
 *
 * @param dev Device for which to set the depth mode
 * @param mode Frame mode to set
 *
 * @return 0 on success, < 0 if error
 */
FREENECTAPI int freenect_set_depth_mode(freenect_device* dev, const freenect_frame_mode mode);

#ifdef __cplusplus
}
#endif

#endif //


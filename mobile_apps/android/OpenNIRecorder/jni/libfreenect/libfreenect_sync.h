/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 Brandyn White (bwhite@dappervision.com)
 *                    Andrew Miller (amiller@dappervision.com)
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

#ifndef FREENECT_SYNC_H
#define FREENECT_SYNC_H
#include <libfreenect.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int freenect_sync_get_video(void **video, uint32_t *timestamp, int index, freenect_video_format fmt);
/*  Synchronous video function, starts the runloop if it isn't running

    The returned buffer is valid until this function is called again, after which the buffer must not
    be used again.  Make a copy if the data is required.

    Args:
        video: Populated with a pointer to a video buffer with a size of the requested type
        timestamp: Populated with the associated timestamp
        index: Device index (0 is the first)
        fmt: Valid format

    Returns:
        Nonzero on error.
*/


int freenect_sync_get_depth(void **depth, uint32_t *timestamp, int index, freenect_depth_format fmt);
/*  Synchronous depth function, starts the runloop if it isn't running

    The returned buffer is valid until this function is called again, after which the buffer must not
    be used again.  Make a copy if the data is required.

    Args:
        depth: Populated with a pointer to a depth buffer with a size of the requested type
        timestamp: Populated with the associated timestamp
        index: Device index (0 is the first)
        fmt: Valid format

    Returns:
        Nonzero on error.
*/

int freenect_sync_set_tilt_degs(int angle, int index);
/*  Tilt function, starts the runloop if it isn't running

    Args:
        angle: Set the angle to tilt the device
		    index: Device index (0 is the first)

    Returns:
        Nonzero on error.
*/

int freenect_sync_get_tilt_state(freenect_raw_tilt_state **state, int index);
/*  Tilt state function, starts the runloop if it isn't running

    Args:
        state: Populated with an updated tilt state pointer
		    index: Device index (0 is the first)

    Returns:
        Nonzero on error.
*/

int freenect_sync_set_led(freenect_led_options led, int index);
/*  Led function, starts the runloop if it isn't running

    Args:
        led: The LED state to set the device to
		    index: Device index (0 is the first)

    Returns:
        Nonzero on error.
*/


void freenect_sync_stop(void);
#ifdef __cplusplus
}
#endif

#endif

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

#ifndef LIBFREENECT_AUDIO_H
#define LIBFREENECT_AUDIO_H

#include <libfreenect.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Structure to represent a single 16-bit signed little-endian PCM sample.
typedef struct {
	int16_t left;
	int16_t right;
	int16_t center;
	int16_t lfe;
	int16_t surround_left;
	int16_t surround_right;
} freenect_sample_51;

/**
 * Typedef for "you wanted this microphone data, here it is" event callbacks.
 * TODO: Timestamp details
 * The format of the unknown stream is as of yet undetermined.
 *
 * @param dev Device which triggered this callback
 * @param num_samples Number of samples provided in each of the audio data arrays (mic[1-4] and cancelled)
 * @param mic1 Microphone data for the leftmost microphone: 32-bit PCM little-endian samples at 16kHz.
 * @param mic2 Microphone data for the left-middle microphone: 32-bit PCM little-endian samples at 16kHz.
 * @param mic3 Microphone data for the right-middle microphone: 32-bit PCM little-endian samples at 16kHz.
 * @param mic4 Microphone data for the rightmost microphone: 32-bit PCM little-endian samples at 16kHz.
 * @param cancelled Noise-cancelled audio data: 16-bit PCM little-endian samples at 16kHz.
 */
typedef void (*freenect_audio_in_cb)(freenect_device *dev, int num_samples,
                                     int32_t* mic1, int32_t* mic2,
                                     int32_t* mic3, int32_t* mic4,
                                     int16_t* cancelled, void *unknown/*, timestamp_t timestamp*/);

/**
 * Typedef for "you're playing audio, the library needs you to fill up the outgoing audio buffer" event callbacks
 * The library will request samples at a rate of 48000Hz.
 *
 * @param dev Device this callback was triggered for
 * @param samples Pointer to the memory where the library expects you to copy the next sample_count freenect_sample_51's to.
 * @param sample_count Bidirectional. in: maximum number of samples the driver wants (don't copy in more than this, you'll clobber memory).  out: actual number of samples provided to the driver.
 */
typedef void (*freenect_audio_out_cb)(freenect_device *dev, freenect_sample_51* samples, int* sample_count);

/**
 * Set the audio in callback.  This is the function called when the library
 * has new microphone samples.  It will be called approximately 62.5 times per
 * second (16kHz sample rate, expect 512 samples/callback)
 *
 * @param dev Device for which to set the callback
 * @param callback Callback function to set
 */
FREENECTAPI void freenect_set_audio_in_callback(freenect_device *dev, freenect_audio_in_cb callback);

/**
 * Set the audio out callback.  This is the "tell me what audio you're about
 * to play through the speakers so the Kinect can subtract it out" callback for
 * a given device.  If you choose not set an audio_out_callback, the library
 * will send silence to the Kinect for you - it requires data either way.
 *
 * @param dev Device for which to set the callback
 * @param callback Callback function to set
 */
FREENECTAPI void freenect_set_audio_out_callback(freenect_device *dev, freenect_audio_out_cb callback);

/**
 * Start streaming audio for the specified device.
 *
 * @param dev Device for which to start audio streaming
 *
 * @return 0 on success, < 0 if error
 */
FREENECTAPI int freenect_start_audio(freenect_device* dev);

/**
 * Stop streaming audio for the specified device.
 *
 * @param dev Device for which to stop audio streaming
 *
 * @return 0 on success, < 0 if error
 */
FREENECTAPI int freenect_stop_audio(freenect_device* dev);

#ifdef __cplusplus
}
#endif

#endif //



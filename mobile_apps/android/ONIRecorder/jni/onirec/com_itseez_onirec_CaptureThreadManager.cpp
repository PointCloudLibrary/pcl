#include <android/bitmap.h>
#include <android/log.h>

#include "com_itseez_onirec_CaptureThreadManager.h"

namespace {
  const char * TAG = "CaptureThreadManager";
}

JNIEXPORT void JNICALL
Java_com_itseez_onirec_CaptureThreadManager_imageMapToBitmap
  (JNIEnv * env, jclass clazz, jlong ptr, jobject bm)
{
  AndroidBitmapInfo info;
  if (AndroidBitmap_getInfo(env, bm, &info) != ANDROID_BITMAP_RESUT_SUCCESS) {
    __android_log_write(ANDROID_LOG_ERROR, TAG, "Couldn't get bitmap info.");
    return;
  }

  if (info.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Unsupported bitmap format: %d.", info.format);
    return;
  }

  void * bm_pixels;
  if (AndroidBitmap_lockPixels(env, bm, &bm_pixels) != ANDROID_BITMAP_RESUT_SUCCESS) {
    __android_log_write(ANDROID_LOG_ERROR, TAG, "Couldn't lock bitmap pixels.");
    return;
  }

  int * bm_pixels_int = static_cast<int *>(bm_pixels);

  unsigned int * buf_int = reinterpret_cast<unsigned int *>(ptr);

  for (int i = 0; i < info.height; ++i) {
    int * pixel = bm_pixels_int + i * info.stride / sizeof(int);
    
    // we will assume width is a multiple of 4 (as is the case with all Kinect output formats)
    for (int j = 0; j < info.width; j += 4, pixel += 4, buf_int += 3) {
      // buf_int -> R1 G1 B1 R2 G2 B2 R3 G3 B3 R4 G4 B4

      unsigned i1 = buf_int[0]; // R2 B1 G1 R1
      unsigned i2 = buf_int[1]; // G3 R3 B2 G2
      unsigned i3 = buf_int[2]; // B4 G4 R4 B3

      pixel[0] = 0xFF000000 | i1; // FF B1 G1 R1
      pixel[1] = 0xFF000000 | (i2 << 8) | (i1 >> 24); // FF B2 G2 R2
      pixel[2] = 0xFF000000 | (i3 << 16) | (i2 >> 16); // FF B3 G3 R3
      pixel[3] = 0xFF000000 | (i3 >> 8); // FF B4 G4 R4
    }
  }

  AndroidBitmap_unlockPixels(env, bm);
}

JNIEXPORT void JNICALL
Java_com_itseez_onirec_CaptureThreadManager_depthMapToBitmap
  (JNIEnv * env, jclass clazz, jlong ptr, jobject bm, jint maxZ)
{
  AndroidBitmapInfo info;
  if (AndroidBitmap_getInfo(env, bm, &info) != ANDROID_BITMAP_RESUT_SUCCESS) {
    __android_log_write(ANDROID_LOG_ERROR, TAG, "Couldn't get bitmap info.");
    return;
  }

  if (info.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Unsupported bitmap format: %d.", info.format);
    return;
  }

  void * bm_pixels;
  if (AndroidBitmap_lockPixels(env, bm, &bm_pixels) != ANDROID_BITMAP_RESUT_SUCCESS) {
    __android_log_write(ANDROID_LOG_ERROR, TAG, "Couldn't lock bitmap pixels.");
    return;
  }

  int * bm_pixels_int = static_cast<int *>(bm_pixels);

  unsigned int * buf_int = reinterpret_cast<unsigned int *>(ptr);

  for (int i = 0; i < info.height; ++i) {
    int * pixel = bm_pixels_int + i * info.stride / sizeof(int);

    // we will assume width is a multiple of 2 (as is the case with all Kinect output formats)
    for (int j = 0; j < info.width; j += 2, pixel += 2, buf_int += 1) {
      unsigned int depths = *buf_int;
      unsigned char gray1 = unsigned(double(depths >> 16) / double(maxZ) * 255.);
      unsigned char gray2 = unsigned(double(depths & 0xFFFF) / double(maxZ) * 255.);

      pixel[0] = 0xFF000000 | (gray1 << 16) | (gray1 << 8) | gray1;
      pixel[1] = 0xFF000000 | (gray2 << 16) | (gray2 << 8) | gray2;
    }
  }

  AndroidBitmap_unlockPixels(env, bm);
}

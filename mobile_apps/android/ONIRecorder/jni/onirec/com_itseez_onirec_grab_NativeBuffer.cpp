#include "com_itseez_onirec_grab_NativeBuffer.h"

JNIEXPORT jlong JNICALL
Java_com_itseez_onirec_grab_NativeBuffer_getPtr
  (JNIEnv * env, jclass clazz, jobject buffer)
{
  return reinterpret_cast<jlong>(env->GetDirectBufferAddress(buffer));
}

JNIEXPORT jlong JNICALL
Java_com_itseez_onirec_grab_NativeBuffer_fillBuffer
  (JNIEnv * env, jclass clazz, jobject buffer)
{
  unsigned char * ptr = static_cast<unsigned char *>(env->GetDirectBufferAddress(buffer));
  jlong size = env->GetDirectBufferCapacity(buffer);

  static int start = 0;
  ++start; // sure, this is thread-unsafe, but we don't care

  for (jlong i = 0; i < size; ++i) ptr[i] = start + i;
}

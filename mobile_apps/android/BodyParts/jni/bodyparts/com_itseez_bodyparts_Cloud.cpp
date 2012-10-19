#include "com_itseez_bodyparts_Cloud.h"
#include "jni_helper.hpp"
#include "cloud.h"

namespace
{
  jmethodID meth_ctor;
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Cloud_cacheIds
  (JNIEnv * env, jclass clazz)
{
  cachePtrID<Cloud> (env, clazz);
  meth_ctor = env->GetMethodID (clazz, "<init>", "(J)V");
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Cloud_create
  (JNIEnv * env, jobject object)
{
  Cloud * ptr = new Cloud;

  setPtr (env, object, ptr);
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Cloud_free
  (JNIEnv * env, jobject object)
{
  freePtr<Cloud> (env, object);
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Cloud_readColors
  (JNIEnv * env, jobject object, jintArray colors)
{
  Cloud * ptr = getPtr<Cloud> (env, object);
  ChannelRef<RGB> rgb = ptr->get<TagColor>();

  jint * colors_elements = env->GetIntArrayElements (colors, NULL);

  for (int i = 0; i < rgb.size; ++i)
    colors_elements[i] = (0xFF << 24) | (rgb.data[i].r << 16) | (rgb.data[i].g << 8) | (rgb.data[i].b << 0);

  env->ReleaseIntArrayElements (colors, colors_elements, 0);
}

JNIEXPORT jint JNICALL
Java_com_itseez_bodyparts_Cloud_getHeight
  (JNIEnv * env, jobject object)
{
  Cloud * ptr = getPtr<Cloud> (env, object);
  return ptr->getHeight();
}

JNIEXPORT jint JNICALL
Java_com_itseez_bodyparts_Cloud_getWidth
  (JNIEnv * env, jobject object)
{
  Cloud * ptr = getPtr<Cloud> (env, object);
  return ptr->getWidth();
}

#include "com_itseez_bodyparts_RGBDImage.h"
#include "jni_helper.hpp"
#include "rgbd_image.h"

namespace
{
  jmethodID meth_ctor;
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_RGBDImage_cacheIds
  (JNIEnv * env, jclass clazz)
{
  cachePtrID<RGBDImage> (env, clazz);
  meth_ctor = env->GetMethodID (clazz, "<init>", "(J)V");
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_RGBDImage_create
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = new RGBDImage;

  setPtr (env, object, ptr);
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_RGBDImage_free
  (JNIEnv * env, jobject object)
{
  freePtr<RGBDImage> (env, object);
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_RGBDImage_readColors
  (JNIEnv * env, jobject object, jintArray colors)
{
  RGBDImage * ptr = getPtr<RGBDImage> (env, object);

  jint * colors_elements = env->GetIntArrayElements (colors, NULL);

  ptr->readColors (colors_elements);

  env->ReleaseIntArrayElements (colors, colors_elements, 0);
}

JNIEXPORT jint JNICALL
Java_com_itseez_bodyparts_RGBDImage_getHeight
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = getPtr<RGBDImage> (env, object);
  return ptr->height;
}

JNIEXPORT jint JNICALL
Java_com_itseez_bodyparts_RGBDImage_getWidth
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = getPtr<RGBDImage> (env, object);
  return ptr->width;
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_RGBDImage_parse
  (JNIEnv * env, jobject object, jbyteArray bytes)
{
  RGBDImage * ptr = getPtr<RGBDImage> (env, object);
  jbyte * bytes_elements = env->GetByteArrayElements (bytes, NULL);

  ptr->parse (reinterpret_cast<char *> (bytes_elements));

  env->ReleaseByteArrayElements (bytes, bytes_elements, 0);
}

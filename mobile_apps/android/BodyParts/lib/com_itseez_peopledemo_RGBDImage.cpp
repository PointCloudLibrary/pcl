#include "com_itseez_peopledemo_RGBDImage.h"
#include "jni_helper.hpp"
#include "rgbd_image.h"

jmethodID meth_ctor;

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_RGBDImage_cacheIds
  (JNIEnv * env, jclass clazz)
{
  cachePtrID<RGBDImage> (env, clazz);
  meth_ctor = env->GetMethodID (clazz, "<init>", "()V");
}

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_RGBDImage_free
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = getPtr<RGBDImage> (env, object);
  if (!ptr) return;

  delete ptr;

  setPtr<RGBDImage> (env, object, NULL);
}

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_RGBDImage_readColors
  (JNIEnv * env, jobject object, jintArray colors)
{
  RGBDImage * ptr = getPtr<RGBDImage> (env, object);

  jint * colors_elements = env->GetIntArrayElements (colors, NULL);

  ptr->readColors (colors_elements);

  env->ReleaseIntArrayElements (colors, colors_elements, 0);
}

JNIEXPORT jint JNICALL
Java_com_itseez_peopledemo_RGBDImage_getHeight
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = getPtr<RGBDImage> (env, object);
  return ptr->height;
}

JNIEXPORT jint JNICALL
Java_com_itseez_peopledemo_RGBDImage_getWidth
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = getPtr<RGBDImage> (env, object);
  return ptr->width;
}

JNIEXPORT jobject JNICALL
Java_com_itseez_peopledemo_RGBDImage_parse
  (JNIEnv * env, jclass clazz, jbyteArray bytes)
{
  jbyte * bytes_elements = env->GetByteArrayElements (bytes, NULL);

  RGBDImage * ptr = RGBDImage::parse (reinterpret_cast<char *> (bytes_elements));

  env->ReleaseByteArrayElements (bytes, bytes_elements, 0);

  jobject object = env->NewObject (clazz, meth_ctor);
  setPtr (env, object, ptr);
  return object;
}

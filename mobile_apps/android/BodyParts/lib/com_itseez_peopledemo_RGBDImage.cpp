#include "com_itseez_peopledemo_RGBDImage.h"
#include "rgbd_image.h"

jfieldID fld_ptr;
jmethodID meth_ctor;

RGBDImage * getPtr(JNIEnv * env, jobject object)
{
  return reinterpret_cast<RGBDImage *> (env->GetLongField (object, fld_ptr));
}

void setPtr(JNIEnv * env, jobject object, RGBDImage * ptr)
{
  env->SetLongField (object, fld_ptr, reinterpret_cast<jlong>(ptr));
}

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_RGBDImage_cacheIds
  (JNIEnv * env, jclass clazz)
{
  fld_ptr = env->GetFieldID (clazz, "ptr", "J");
  meth_ctor = env->GetMethodID(clazz, "<init>", "()V");
}

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_RGBDImage_free
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = getPtr(env, object);
  if (!ptr) return;

  delete ptr;

  setPtr(env, object, NULL);
}

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_RGBDImage_readColors
  (JNIEnv * env, jobject object, jintArray colors)
{
  RGBDImage * ptr = getPtr (env, object);

  jint * colors_elements = env->GetIntArrayElements(colors, NULL);

  ptr->readColors(colors_elements);

  env->ReleaseIntArrayElements(colors, colors_elements, 0);
}

JNIEXPORT jint JNICALL
Java_com_itseez_peopledemo_RGBDImage_getHeight
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = getPtr (env, object);
  return ptr->height;
}

JNIEXPORT jint JNICALL
Java_com_itseez_peopledemo_RGBDImage_getWidth
  (JNIEnv * env, jobject object)
{
  RGBDImage * ptr = getPtr (env, object);
  return ptr->width;
}

JNIEXPORT jobject JNICALL
Java_com_itseez_peopledemo_RGBDImage_parse
  (JNIEnv * env, jclass clazz, jbyteArray bytes)
{
  jbyte * bytes_elements = env->GetByteArrayElements (bytes, NULL);

  RGBDImage * ptr = RGBDImage::parse(reinterpret_cast<char *> (bytes_elements));

  env->ReleaseByteArrayElements(bytes, bytes_elements, 0);

  jobject object = env->NewObject(clazz, meth_ctor);
  setPtr (env, object, ptr);
  return object;
}

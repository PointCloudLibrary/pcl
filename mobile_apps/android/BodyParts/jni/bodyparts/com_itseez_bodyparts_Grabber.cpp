#include "com_itseez_bodyparts_Grabber.h"
#include "jni_helper.hpp"
#include "grabber.h"

namespace
{
  jmethodID meth_ctor;
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Grabber_cacheIds
  (JNIEnv * env, jclass clazz)
{
  cachePtrID<Grabber> (env, clazz);
  meth_ctor = env->GetMethodID (clazz, "<init>", "()V");
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Grabber_free
  (JNIEnv * env, jobject object)
{
  freePtr<Grabber> (env, object);
}

JNIEXPORT jobject JNICALL
Java_com_itseez_bodyparts_Grabber_createOpenNIGrabber
  (JNIEnv * env, jclass clazz)
{
  Grabber * ptr = Grabber::createOpenNIGrabber();

  jobject object = env->NewObject (clazz, meth_ctor);
  setPtr (env, object, ptr);
  return object;
}

JNIEXPORT jobject JNICALL
Java_com_itseez_bodyparts_Grabber_createFileGrabber
  (JNIEnv * env, jclass clazz, jstring directory)
{
  const char * directory_chars = env->GetStringUTFChars(directory, NULL);
  Grabber * ptr = Grabber::createFileGrabber(directory_chars);
  env->ReleaseStringUTFChars(directory, directory_chars);

  jobject object = env->NewObject (clazz, meth_ctor);
  setPtr (env, object, ptr);
  return object;
}

JNIEXPORT jboolean JNICALL
Java_com_itseez_bodyparts_Grabber_isConnected
  (JNIEnv * env, jobject object)
{
  Grabber * ptr = getPtr<Grabber>(env, object);
  return ptr->isConnected();
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Grabber_getFrame
  (JNIEnv * env, jobject object, jobject frame)
{
  Grabber * ptr = getPtr<Grabber>(env, object);
  RGBDImage * frame_ptr = getPtr<RGBDImage>(env, frame);
  ptr->getFrame(frame_ptr);
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Grabber_start
  (JNIEnv * env, jobject object)
{
  Grabber * ptr = getPtr<Grabber>(env, object);
  ptr->start();
}

JNIEXPORT void JNICALL
Java_com_itseez_bodyparts_Grabber_stop
  (JNIEnv * env, jobject object)
{
  Grabber * ptr = getPtr<Grabber>(env, object);
  ptr->stop();
}

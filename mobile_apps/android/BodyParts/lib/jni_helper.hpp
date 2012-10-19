#ifndef JNI_HELPER_
#define JNI_HELPER_

#include <jni.h>

template<typename T>
struct JNIHelper
{
  static jfieldID fld_ptr;
};

template<typename T>
jfieldID JNIHelper<T>::fld_ptr;

template <typename T> T *
getPtr(JNIEnv * env, jobject object)
{
  return reinterpret_cast<T *> (env->GetLongField (object, JNIHelper<T>::fld_ptr));
}

template <typename T> void
setPtr(JNIEnv * env, jobject object, T * ptr)
{
  env->SetLongField (object, JNIHelper<T>::fld_ptr, reinterpret_cast<jlong>(ptr));
}

template <typename T> void
cachePtrID(JNIEnv * env, jclass clazz)
{
  JNIHelper<T>::fld_ptr = env->GetFieldID (clazz, "ptr", "J");
}

#endif // JNI_HELPER_

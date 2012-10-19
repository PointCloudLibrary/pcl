#include "body_parts_recognizer.h"
#include "com_itseez_peopledemo_BodyPartsRecognizer.h"
#include "jni_helper.hpp"

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_BodyPartsRecognizer_cacheIds
  (JNIEnv * env, jclass clazz)
{
  cachePtrID<BodyPartsRecognizer> (env, clazz);
}

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_BodyPartsRecognizer_create
  (JNIEnv * env, jobject object)
{
  setPtr(env, object, new BodyPartsRecognizer);
}

JNIEXPORT void JNICALL
Java_com_itseez_peopledemo_BodyPartsRecognizer_free
  (JNIEnv * env, jobject object)
{
  freePtr<BodyPartsRecognizer> (env, object);
}

JNIEXPORT jbyteArray JNICALL
Java_com_itseez_peopledemo_BodyPartsRecognizer_recognize
  (JNIEnv * env, jobject object, jobject image)
{
  BodyPartsRecognizer * bpr = getPtr<BodyPartsRecognizer> (env, object);
  RGBDImage * image_ptr = getPtr<RGBDImage> (env, image);

  std::vector<signed char> labels;
  bpr->recognize(*image_ptr, labels);

  jbyteArray result = env->NewByteArray(labels.size());
  env->SetByteArrayRegion(result, 0, labels.size(), &labels.front());

  return result;
}

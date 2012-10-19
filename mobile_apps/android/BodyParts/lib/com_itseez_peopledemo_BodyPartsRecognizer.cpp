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
  (JNIEnv * env, jobject object, jobjectArray trees)
{
  std::vector<jbyteArray> trees_elements (env->GetArrayLength (trees));
  std::vector<char *> trees_elements_elements;

  for (std::size_t i = 0; i < trees_elements.size (); ++i)
  {
    trees_elements[i] = static_cast<jbyteArray> (env->GetObjectArrayElement (trees, 0));
    trees_elements_elements[i] = reinterpret_cast<char *>
        (env->GetByteArrayElements (trees_elements[i], NULL));
  }

  // We need const pointers; JNI needs non-const ones. We'll just make a copy.
  std::vector<const char *> trees_data (trees_elements.size ());
  std::copy(trees_elements_elements.begin (), trees_elements_elements.end (),
            trees_data.begin ());

  setPtr (env, object, new BodyPartsRecognizer (
            trees_data.size (), &trees_data.front()));

  for (std::size_t i = 0; i < trees_elements.size (); ++i)
  {
    env->ReleaseByteArrayElements (trees_elements[i],
                                   reinterpret_cast<jbyte *> (trees_elements_elements[i]), 0);
    env->DeleteGlobalRef (trees_elements[i]);
  }
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

  std::vector<Label> labels;
  bpr->recognize (*image_ptr, labels);

  jbyteArray result = env->NewByteArray (labels.size ());
  env->SetByteArrayRegion (result, 0, labels.size (), reinterpret_cast<jbyte *> (&labels.front ()));

  return result;
}

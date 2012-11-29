#include <stdlib.h>

#include "org_pointclouds_onirec_Application.h"

JNIEXPORT void JNICALL
Java_org_pointclouds_onirec_Application_setenv
  (JNIEnv * env, jclass, jstring name, jstring value)
{
  const char * name_chars = env->GetStringUTFChars(name, NULL);
  const char * value_chars = env->GetStringUTFChars(value, NULL);

  setenv(name_chars, value_chars, 1);

  env->ReleaseStringUTFChars(value, value_chars);
  env->ReleaseStringUTFChars(name, name_chars);
}


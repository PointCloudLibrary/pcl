/*
 *  Copyright 2011-12 ARM Limited
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*
 * NE10 Library : source/NE10_normalize.c
 */

#include "NE10.h"
#include "../headers/macros.h"

#include <assert.h>

#include <math.h>

arm_result_t normalize_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src, unsigned int count)
{
  float len;

  NE10_LEN_OPERATION_X_C
  (
    len = sqrt( src[ itr ].x * src[ itr ].x +
                src[ itr ].y * src[ itr ].y ) ;

    dst[ itr ].x = src[ itr ].x / len;
    dst[ itr ].y = src[ itr ].y / len;
  );
}

arm_result_t normalize_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src, unsigned int count)
{
  float len;

  NE10_LEN_OPERATION_X_C
  (
    len = sqrt( src[ itr ].x * src[ itr ].x +
                src[ itr ].y * src[ itr ].y +
                src[ itr ].z * src[ itr ].z );

    dst[ itr ].x = src[ itr ].x / len;
    dst[ itr ].y = src[ itr ].y / len;
    dst[ itr ].z = src[ itr ].z / len;
  );
}

arm_result_t normalize_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src, unsigned int count)
{
  float len;

  NE10_LEN_OPERATION_X_C
  (
    len = sqrt( src[ itr ].x * src[ itr ].x +
                src[ itr ].y * src[ itr ].y +
                src[ itr ].z * src[ itr ].z +
                src[ itr ].w * src[ itr ].w );

    dst[ itr ].x = src[ itr ].x / len;
    dst[ itr ].y = src[ itr ].y / len;
    dst[ itr ].z = src[ itr ].z / len;
    dst[ itr ].w = src[ itr ].w / len;
  );
}

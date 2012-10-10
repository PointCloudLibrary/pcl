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
 * NE10 Library : source/NE10_mul.c
 */

#include "NE10.h"
#include "../headers/macros.h"

#include <assert.h>

arm_result_t mul_float_c(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count)
{
  NE10_X_OPERATION_FLOAT_C
  (
    dst[ itr ] = src1[ itr ] * src2[ itr ];
  );
}

arm_result_t vmul_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count)
{
  NE10_X_OPERATION_FLOAT_C
  (
    dst[ itr ].x = src1[ itr ].x * src2[ itr ].x;
    dst[ itr ].y = src1[ itr ].y * src2[ itr ].y;
  );
}

arm_result_t vmul_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count)
{
  NE10_X_OPERATION_FLOAT_C
  (
    dst[ itr ].x = src1[ itr ].x * src2[ itr ].x;
    dst[ itr ].y = src1[ itr ].y * src2[ itr ].y;
    dst[ itr ].z = src1[ itr ].z * src2[ itr ].z;
  );
}

arm_result_t vmul_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count)
{
  NE10_X_OPERATION_FLOAT_C
  (
    dst[ itr ].x = src1[ itr ].x * src2[ itr ].x;
    dst[ itr ].y = src1[ itr ].y * src2[ itr ].y;
    dst[ itr ].z = src1[ itr ].z * src2[ itr ].z;
    dst[ itr ].w = src1[ itr ].w * src2[ itr ].w;
  );
}

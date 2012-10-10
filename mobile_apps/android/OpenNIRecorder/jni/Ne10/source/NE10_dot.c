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
 * NE10 Library : source/NE10_dot.c
 */

#include "NE10.h"
#include "../headers/macros.h"

#include <assert.h>

arm_result_t dot_vec2f_c(arm_float_t * dst, arm_vec2f_t * src1,  arm_vec2f_t * src2, unsigned int count)
{
  NE10_DOT_OPERATION_X_C
  (
    dst[ itr ] =       src1[ itr ].x * src2[ itr ].x +
                       src1[ itr ].y * src2[ itr ].y   ;
  );
}

arm_result_t dot_vec3f_c(arm_float_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count)
{
  NE10_DOT_OPERATION_X_C
  (
    dst[ itr ] =       src1[ itr ].x * src2[ itr ].x +
                       src1[ itr ].y * src2[ itr ].y +
                       src1[ itr ].z * src2[ itr ].z  ;
  );
}

arm_result_t dot_vec4f_c(arm_float_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count)
{
  NE10_DOT_OPERATION_X_C
  (
    dst[ itr ] =       src1[ itr ].x * src2[ itr ].x +
                       src1[ itr ].y * src2[ itr ].y +
                       src1[ itr ].z * src2[ itr ].z +
                       src1[ itr ].w * src2[ itr ].w  ;
  );
}

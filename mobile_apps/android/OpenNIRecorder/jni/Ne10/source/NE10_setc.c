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
 * NE10 Library : source/NE10_setc.c
 */

#include "NE10.h"
#include "../headers/macros.h"

#include <assert.h>

arm_result_t setc_float_c(arm_float_t * dst, const arm_float_t cst, unsigned int count)
{
  NE10_SETC_OPERATION_X_C
  (
    dst[itr] = cst;
  );
}

arm_result_t setc_vec2f_c(arm_vec2f_t * dst, const arm_vec2f_t * cst, unsigned int count)
{
  NE10_SETC_OPERATION_X_C
  (
    dst[itr].x = cst->x;
    dst[itr].y = cst->y;
  );
}

arm_result_t setc_vec3f_c(arm_vec3f_t * dst, const arm_vec3f_t * cst, unsigned int count)
{
  NE10_SETC_OPERATION_X_C
  (
    dst[itr].x = cst->x;
    dst[itr].y = cst->y;
    dst[itr].z = cst->z;
  );
}

arm_result_t setc_vec4f_c(arm_vec4f_t * dst, const arm_vec4f_t * cst, unsigned int count)
{
  NE10_SETC_OPERATION_X_C
  (
    dst[itr].x = cst->x;
    dst[itr].y = cst->y;
    dst[itr].z = cst->z;
    dst[itr].w = cst->w;
  );
}

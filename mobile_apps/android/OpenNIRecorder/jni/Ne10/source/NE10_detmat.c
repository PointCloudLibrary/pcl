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
 * NE10 Library : source/NE10_detmat.c
 */

#include "NE10.h"
#include "../headers/macros.h"
#include "NE10_detmat.c.h"

#include <assert.h>

arm_result_t detmat_2x2f_c(arm_float_t * dst, arm_mat2x2f_t * src, unsigned int count)
{
   NE10_DETMAT_OPERATION_X_C
   (
     dst[ itr ] = DET2x2( &src[ itr ] );
   );
}

arm_result_t detmat_3x3f_c(arm_float_t * dst, arm_mat3x3f_t * src, unsigned int count)
{
   NE10_DETMAT_OPERATION_X_C
   (
     dst[ itr ] = DET3x3( &(src[ itr ]) );

   );
}

arm_result_t detmat_4x4f_c(arm_float_t * dst, arm_mat4x4f_t * src, unsigned int count)
{
   NE10_DETMAT_OPERATION_X_C
   (
     dst[ itr ] = DET4x4( &src[ itr ] );
   );
}

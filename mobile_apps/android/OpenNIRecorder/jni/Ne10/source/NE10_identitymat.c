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
 * NE10 Library : source/NE10_identitymat.c
 */

#include "NE10.h"
#include "../headers/macros.h"

arm_result_t identitymat_2x2f_c(arm_mat2x2f_t * dst, unsigned int count)
{
  arm_mat2x2f_t *src = dst; // dummy placeholder

  NE10_DETMAT_OPERATION_X_C
  (
      dst[ itr ].c1.r1 =  1.0f;
      dst[ itr ].c1.r2 =  0.0f;
      dst[ itr ].c2.r1 =  0.0f;
      dst[ itr ].c2.r2 =  1.0f;
  );
}

arm_result_t identitymat_3x3f_c(arm_mat3x3f_t * dst, unsigned int count)
{
  arm_mat3x3f_t *src = dst; // dummy placeholder

  NE10_DETMAT_OPERATION_X_C
  (
    dst[ itr ].c1.r1 =  1.0f;
    dst[ itr ].c1.r2 =  0.0f;
    dst[ itr ].c1.r3 =  0.0f;

    dst[ itr ].c2.r1 =  0.0f;
    dst[ itr ].c2.r2 =  1.0f;
    dst[ itr ].c2.r3 =  0.0f;

    dst[ itr ].c3.r1 =  0.0f;
    dst[ itr ].c3.r2 =  0.0f;
    dst[ itr ].c3.r3 =  1.0f;
  );
}

arm_result_t identitymat_4x4f_c(arm_mat4x4f_t * dst, unsigned int count)
{
  arm_mat4x4f_t *src = dst; // dummy placeholder

  NE10_DETMAT_OPERATION_X_C
  (
    dst[ itr ].c1.r1 =  1.0f;
    dst[ itr ].c1.r2 =  0.0f;
    dst[ itr ].c1.r3 =  0.0f;
    dst[ itr ].c1.r4 =  0.0f;

    dst[ itr ].c2.r1 =  0.0f;
    dst[ itr ].c2.r2 =  1.0f;
    dst[ itr ].c2.r3 =  0.0f;
    dst[ itr ].c2.r4 =  0.0f;

    dst[ itr ].c3.r1 =  0.0f;
    dst[ itr ].c3.r2 =  0.0f;
    dst[ itr ].c3.r3 =  1.0f;
    dst[ itr ].c3.r4 =  0.0f;

    dst[ itr ].c4.r1 =  0.0f;
    dst[ itr ].c4.r2 =  0.0f;
    dst[ itr ].c4.r3 =  0.0f;
    dst[ itr ].c4.r4 =  1.0f;
  );
}

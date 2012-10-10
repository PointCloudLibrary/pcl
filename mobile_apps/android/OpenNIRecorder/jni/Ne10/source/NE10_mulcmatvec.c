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
 * NE10 Library : source/NE10_mulcmatvec.neon.s
 */

#include "NE10.h"
#include "../headers/macros.h"

#include <assert.h>

arm_result_t mulcmatvec_cm2x2f_v2f_c (arm_vec2f_t * dst, const arm_mat2x2f_t * cst, arm_vec2f_t * src, unsigned int count)
{
  #define A1 cst->c1.r1
  #define B1 cst->c1.r2
  #define C1 cst->c2.r1
  #define D1 cst->c2.r2

    NE10_CMATVEC_OPERATION_X_C
    (
      dst[ itr ].x = A1 * src[ itr ].x + C1 * src[ itr ].y;
      dst[ itr ].y = B1 * src[ itr ].x + D1 * src[ itr ].y;
    );

  #undef A1
  #undef B1
  #undef C1
  #undef D1
}

arm_result_t mulcmatvec_cm3x3f_v3f_c (arm_vec3f_t * dst, const arm_mat3x3f_t * cst, arm_vec3f_t * src, unsigned int count)
{
  #define A1 cst->c1.r1
  #define B1 cst->c1.r2
  #define C1 cst->c1.r3
  #define D1 cst->c2.r1
  #define E1 cst->c2.r2
  #define F1 cst->c2.r3
  #define G1 cst->c3.r1
  #define H1 cst->c3.r2
  #define I1 cst->c3.r3

    NE10_CMATVEC_OPERATION_X_C
    (
      dst[ itr ].x = A1 * src[ itr ].x + D1 * src[ itr ].y + G1 * src[ itr ].z;
      dst[ itr ].y = B1 * src[ itr ].x + E1 * src[ itr ].y + H1 * src[ itr ].z;
      dst[ itr ].z = C1 * src[ itr ].x + F1 * src[ itr ].y + I1 * src[ itr ].z;
    );

  #undef A1
  #undef B1
  #undef C1
  #undef D1
  #undef E1
  #undef F1
  #undef G1
  #undef H1
  #undef I1
}

extern arm_result_t mulcmatvec_cm4x4f_v4f_c (arm_vec4f_t * dst, const arm_mat4x4f_t * cst, arm_vec4f_t * src, unsigned int count)
{
  #define A1 cst->c1.r1
  #define B1 cst->c1.r2
  #define C1 cst->c1.r3
  #define D1 cst->c1.r4
  #define E1 cst->c2.r1
  #define F1 cst->c2.r2
  #define G1 cst->c2.r3
  #define H1 cst->c2.r4
  #define I1 cst->c3.r1
  #define J1 cst->c3.r2
  #define K1 cst->c3.r3
  #define L1 cst->c3.r4
  #define M1 cst->c4.r1
  #define N1 cst->c4.r2
  #define O1 cst->c4.r3
  #define P1 cst->c4.r4

  NE10_CMATVEC_OPERATION_X_C
  (
      dst[ itr ].x = A1 * src[ itr ].x + E1 * src[ itr ].y + I1 * src[ itr ].z + M1 * src[ itr ].w;
      dst[ itr ].y = B1 * src[ itr ].x + F1 * src[ itr ].y + J1 * src[ itr ].z + N1 * src[ itr ].w;
      dst[ itr ].z = C1 * src[ itr ].x + G1 * src[ itr ].y + K1 * src[ itr ].z + O1 * src[ itr ].w;
      dst[ itr ].w = D1 * src[ itr ].x + H1 * src[ itr ].y + L1 * src[ itr ].z + P1 * src[ itr ].w;
  );

  #undef A1
  #undef B1
  #undef C1
  #undef D1
  #undef E1
  #undef F1
  #undef G1
  #undef H1
  #undef I1
  #undef J1
  #undef K1
  #undef L1
  #undef M1
  #undef N1
  #undef O1
  #undef P1
}

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
 * NE10 Library : source/NE10_addc.neon.c
 */

#include "NE10.h"
#include "../headers/macros.h"

#include <assert.h>
#include <arm_neon.h>


arm_result_t addc_float_neon(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count)
{
    NE10_XC_OPERATION_FLOAT_NEON
    (
        n_dst = vaddq_f32( n_src , n_cst );
        ,
        n_tmp_src = vadd_f32( n_tmp_src, n_tmp_cst );
    );
}

arm_result_t addc_vec2f_neon(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count)
{
    NE10_XC_OPERATION_VEC2F_NEON
    (
       n_dst = vaddq_f32( n_src , n_cst );
       ,
       n_tmp_src = vadd_f32( n_tmp_src, n_tmp_cst );
    );
}

arm_result_t addc_vec3f_neon(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count)
{
    NE10_XC_OPERATION_VEC3F_NEON
    (
        n_dst1 = vaddq_f32( n_src1 , n_cst1 );
        n_dst2 = vaddq_f32( n_src2 , n_cst2 );
        n_dst3 = vaddq_f32( n_src3 , n_cst3 );
        ,
        n_tmp_src.val[0] = vadd_f32( n_tmp_src.val[0], n_tmp_cst.val[0] );  /* the X lane */
        n_tmp_src.val[1] = vadd_f32( n_tmp_src.val[1], n_tmp_cst.val[1] );  /* the Y lane */
        n_tmp_src.val[2] = vadd_f32( n_tmp_src.val[2], n_tmp_cst.val[2] );  /* the Z lane */
     );
}

arm_result_t addc_vec4f_neon(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count)
{
    NE10_XC_OPERATION_VEC4F_NEON
    (
        n_dst = vaddq_f32( n_src , n_cst );
    );
}

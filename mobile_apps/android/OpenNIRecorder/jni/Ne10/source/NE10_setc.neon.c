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
 * NE10 Library : source/NE10_setc.neon.c
 */

#include "NE10.h"
#include "../headers/macros.h"

#include <assert.h>
#include <arm_neon.h>


arm_result_t setc_float_neon(arm_float_t * dst, const arm_float_t cst, unsigned int count)
{
    NE10_SETC_OPERATION_FLOAT_NEON
    (
        ;// The cst need not be altered
        ,
        ;// n_tmp_cst need not be altered
    );
}

arm_result_t setc_vec2f_neon(arm_vec2f_t * dst, const arm_vec2f_t * cst, unsigned int count)
{
    NE10_SETC_OPERATION_VEC2F_NEON
    (
        ;// The cst need not be altered
        ,
        ;// n_tmp_cst need not be altered
    );
}

arm_result_t setc_vec3f_neon(arm_vec3f_t * dst, const arm_vec3f_t * cst, unsigned int count)
{
    NE10_SETC_OPERATION_VEC3F_NEON
    (
        ;// cst1, cst2, and cst3 need not be altered
        ,
        ;// n_tmp_cst.val[0], .val[1], and .val[2] need not be altered
     );
}

arm_result_t setc_vec4f_neon(arm_vec4f_t * dst, const arm_vec4f_t * cst, unsigned int count)
{
    NE10_SETC_OPERATION_VEC4F_NEON
    (
        ;// n_cst need not be altered
    );
}

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

#include "NE10.h"

arm_result_t submat_2x2f_neon(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count)
{
	return sub_vec2f_neon( (arm_vec2f_t*)dst, (arm_vec2f_t*)src1, (arm_vec2f_t*)src2, count*2 );
}

arm_result_t submat_3x3f_neon(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count)
{
        return sub_vec3f_neon( (arm_vec3f_t*)dst, (arm_vec3f_t*)src1, (arm_vec3f_t*)src2, count*3 );
}

arm_result_t submat_4x4f_neon(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count)
{
        return sub_vec4f_neon( (arm_vec4f_t*)dst, (arm_vec4f_t*)src1, (arm_vec4f_t*)src2, count*4 );
}


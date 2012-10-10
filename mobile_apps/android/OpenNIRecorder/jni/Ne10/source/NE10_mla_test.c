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
 * NE10 Library : source/NE10_mla_test.c
 */

//Make sure the following are defined before including "unit_test.h"

// length of the data arrays
#define ARRLEN TEST_ARRLEN
// number of the operations in a given unit
#define OP_COUNT 4
// number of the different implementations of each of the functions (C, ASM, NEON, ...)
#define IMPL_COUNT 3


#include "../headers/unit_test_mla_operation_x.h"

arm_result_t mla_float_c   (arm_float_t * dst, arm_float_t * acc, arm_float_t * src1, arm_float_t * src2, unsigned int count);
//arm_result_t mla_float_asm (arm_float_t * dst, arm_float_t * acc, arm_float_t * src1, arm_float_t * src2, unsigned int count); // the assembly versions haven't been implemented; these are for future use
arm_result_t mla_float_neon(arm_float_t * dst, arm_float_t * acc, arm_float_t * src1, arm_float_t * src2, unsigned int count);

arm_result_t vmla_vec2f_c   (arm_vec2f_t * dst, arm_vec2f_t * acc, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
//arm_result_t vmla_vec2f_asm (arm_vec2f_t * dst, arm_vec2f_t * acc, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
arm_result_t vmla_vec2f_neon(arm_vec2f_t * dst, arm_vec2f_t * acc, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);

arm_result_t vmla_vec3f_c   (arm_vec3f_t * dst, arm_vec3f_t * acc, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
//arm_result_t vmla_vec3f_asm (arm_vec3f_t * dst, arm_vec4f_t * acc, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
arm_result_t vmla_vec3f_neon(arm_vec3f_t * dst, arm_vec3f_t * acc, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);

arm_result_t vmla_vec4f_c   (arm_vec4f_t * dst, arm_vec4f_t * acc, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);
//arm_result_t vmla_vec4f_asm (arm_vec4f_t * dst, arm_vec4f_t * acc, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);
arm_result_t vmla_vec4f_neon(arm_vec4f_t * dst, arm_vec4f_t * acc, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);

void init_ftbl()
{
   // manually initialize the global function table with
   //  those functions that do have an actual implementation.
   ftbl[ 0] = (arm_func_5args_t) mla_float_c;
   ftbl[ 1] = (arm_func_5args_t) mla_float_c; // using the c version in place of the assembly version
   ftbl[ 2] = (arm_func_5args_t) mla_float_neon;

   ftbl[ 3] = (arm_func_5args_t) vmla_vec2f_c;
   ftbl[ 4] = (arm_func_5args_t) vmla_vec2f_c; // using the c version in place of the assembly version
   ftbl[ 5] = (arm_func_5args_t) vmla_vec2f_neon;

   ftbl[ 6] = (arm_func_5args_t) vmla_vec3f_c;
   ftbl[ 7] = (arm_func_5args_t) vmla_vec3f_c; // using the c version in place of the assembly version
   ftbl[ 8] = (arm_func_5args_t) vmla_vec3f_neon;

   ftbl[ 9] = (arm_func_5args_t) vmla_vec4f_c;
   ftbl[10] = (arm_func_5args_t) vmla_vec4f_c; // using the c version in place of the assembly version
   ftbl[11] = (arm_func_5args_t) vmla_vec4f_neon;
}

arm_result_t main( int argc, char **argv )
{
   return run_test( argc, argv ); // defined in "unit_test.h"
}

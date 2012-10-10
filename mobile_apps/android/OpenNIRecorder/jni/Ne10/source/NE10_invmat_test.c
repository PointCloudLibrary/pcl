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
 * NE10 Library : source/NE10_invmat_test.c
 */

//Make sure the following are defined before including "unit_test.h"

// length of the data arrays
#define ARRLEN TEST_ARRLEN_MATRICES
// number of the operations in a given unit
#define OP_COUNT 3 
// number of the different implementations of each of the functions (C, ASM, NEON, ...)
#define IMPL_COUNT 3


#include "../headers/unit_test_invmat_operation_x.h"

extern arm_result_t invmat_2x2f_c   (arm_mat2x2f_t * dst, arm_mat2x2f_t * src, unsigned int count);
extern arm_result_t invmat_2x2f_neon(arm_mat2x2f_t * dst, arm_mat2x2f_t * src, unsigned int count);

extern arm_result_t invmat_3x3f_c   (arm_mat3x3f_t * dst, arm_mat3x3f_t * src, unsigned int count);
extern arm_result_t invmat_3x3f_neon(arm_mat3x3f_t * dst, arm_mat3x3f_t * src, unsigned int count);

extern arm_result_t invmat_4x4f_c   (arm_mat4x4f_t * dst, arm_mat4x4f_t * src, unsigned int count);
extern arm_result_t invmat_4x4f_neon(arm_mat4x4f_t * dst, arm_mat4x4f_t * src, unsigned int count);

void init_ftbl()
{
   // manually initialize the global function table with
   //  those functions that do have an actual implementation.
   ftbl[ 0] = (arm_func_3args_t) invmat_2x2f_c;
   ftbl[ 1] = (arm_func_3args_t) invmat_2x2f_c; // using the c version in place of the assembly version
   ftbl[ 2] = (arm_func_3args_t) invmat_2x2f_neon;

   ftbl[ 3] = (arm_func_3args_t) invmat_3x3f_c;
   ftbl[ 4] = (arm_func_3args_t) invmat_3x3f_c; // using the c version in place of the assembly version
   ftbl[ 5] = (arm_func_3args_t) invmat_3x3f_neon;

   ftbl[ 6] = (arm_func_3args_t) invmat_4x4f_c;
   ftbl[ 7] = (arm_func_3args_t) invmat_4x4f_c; // using the c version in place of the assembly version
   ftbl[ 8] = (arm_func_3args_t) invmat_4x4f_neon;
}

arm_result_t main( int argc, char **argv )
{
   return run_test( argc, argv ); // defined in "unit_test.h"
}

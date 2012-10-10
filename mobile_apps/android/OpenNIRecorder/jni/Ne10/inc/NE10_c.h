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
 * NE10 Library : inc/NE10_c.h
 */

//#include "../headers/versionheader.h"
#include <NE10_types.h>

#ifndef NE10_C_H
#define NE10_C_H

#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////
// function prototypes:
///////////////////////////


// ## Vector-Constant Arithmetic ##

extern arm_result_t addc_float_c(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
extern arm_result_t addc_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
extern arm_result_t addc_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
extern arm_result_t addc_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



extern arm_result_t subc_float_c(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count); // subtract cst from the element(s)
extern arm_result_t subc_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count); // subtract cst from the element(s)
extern arm_result_t subc_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count); // subtract cst from the element(s)
extern arm_result_t subc_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count); // subtract cst from the element(s)



extern arm_result_t rsbc_float_c(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count); // subtract element(s) from a cst
extern arm_result_t rsbc_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count); // subtract element(s) from a cst
extern arm_result_t rsbc_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count); // subtract element(s) from a cst
extern arm_result_t rsbc_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count); // subtract element(s) from a cst



extern arm_result_t mulc_float_c(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
extern arm_result_t mulc_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
extern arm_result_t mulc_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
extern arm_result_t mulc_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



extern arm_result_t divc_float_c(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
extern arm_result_t divc_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
extern arm_result_t divc_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
extern arm_result_t divc_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



extern arm_result_t setc_float_c(arm_float_t * dst, const arm_float_t cst, unsigned int count);
extern arm_result_t setc_vec2f_c(arm_vec2f_t * dst, const arm_vec2f_t * cst, unsigned int count);
extern arm_result_t setc_vec3f_c(arm_vec3f_t * dst, const arm_vec3f_t * cst, unsigned int count);
extern arm_result_t setc_vec4f_c(arm_vec4f_t * dst, const arm_vec4f_t * cst, unsigned int count);



extern arm_result_t mlac_float_c(arm_float_t * dst, arm_float_t * acc, arm_float_t * src, const arm_float_t cst, unsigned int count);
extern arm_result_t mlac_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * acc, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
extern arm_result_t mlac_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * acc, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
extern arm_result_t mlac_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * acc, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);

// ## Arithmetic functions over arrays of cst values ##
extern arm_result_t add_float_c(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
extern arm_result_t sub_float_c(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
extern arm_result_t mul_float_c(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
extern arm_result_t div_float_c(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
extern arm_result_t mla_float_c(arm_float_t * dst, arm_float_t * acc, arm_float_t * src1, arm_float_t * src2, unsigned int count);
extern arm_result_t abs_float_c(arm_float_t * dst, arm_float_t * src, unsigned int count);

// ## Operations on Vectors ##
extern arm_result_t len_vec2f_c(arm_float_t * dst, arm_vec2f_t * src, unsigned int count);
extern arm_result_t len_vec3f_c(arm_float_t * dst, arm_vec3f_t * src, unsigned int count);
extern arm_result_t len_vec4f_c(arm_float_t * dst, arm_vec4f_t * src, unsigned int count);



extern arm_result_t normalize_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src, unsigned int count);
extern arm_result_t normalize_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src, unsigned int count);
extern arm_result_t normalize_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src, unsigned int count);



extern arm_result_t abs_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src, unsigned int count);
extern arm_result_t abs_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src, unsigned int count);
extern arm_result_t abs_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src, unsigned int count);



// ## SIMD Component-wise Arithmetic on Two Vectors ##
extern arm_result_t vmul_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
extern arm_result_t vmul_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
extern arm_result_t vmul_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



extern arm_result_t vdiv_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
extern arm_result_t vdiv_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
extern arm_result_t vdiv_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



extern arm_result_t vmla_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * acc, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
extern arm_result_t vmla_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * acc, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
extern arm_result_t vmla_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * acc, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



// ## Vector-Vector Algebra ##
extern arm_result_t add_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
extern arm_result_t add_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
extern arm_result_t add_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



extern arm_result_t sub_vec2f_c(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
extern arm_result_t sub_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
extern arm_result_t sub_vec4f_c(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



extern arm_result_t dot_vec2f_c(arm_float_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
extern arm_result_t dot_vec3f_c(arm_float_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
extern arm_result_t dot_vec4f_c(arm_float_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



extern arm_result_t cross_vec3f_c(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);



// ## Matrix-Constant Arithmetic ##

// arm_mat4x4f_t
extern arm_result_t addmat_4x4f_c(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t submat_4x4f_c(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t mulmat_4x4f_c(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t divmat_4x4f_c(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t setmat_4x4f_c(arm_mat4x4f_t * dst, const arm_float_t cst, unsigned int count);

extern arm_result_t addmat_3x3f_c(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t submat_3x3f_c(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t mulmat_3x3f_c(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t divmat_3x3f_c(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t setmat_3x3f_c(arm_mat3x3f_t * dst, const arm_float_t cst, unsigned int count);

extern arm_result_t addmat_2x2f_c(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
extern arm_result_t submat_2x2f_c(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
extern arm_result_t mulmat_2x2f_c(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
extern arm_result_t divmat_2x2f_c(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
extern arm_result_t setmat_2x2f_c(arm_mat2x2f_t * dst, const arm_float_t cst, unsigned int count);



// ## Operations on Matrices ##

extern arm_result_t detmat_4x4f_c(arm_float_t * dst, arm_mat4x4f_t * src, unsigned int count);
extern arm_result_t detmat_3x3f_c(arm_float_t * dst, arm_mat3x3f_t * src, unsigned int count);
extern arm_result_t detmat_2x2f_c(arm_float_t * dst, arm_mat2x2f_t * src, unsigned int count);

extern arm_result_t invmat_4x4f_c(arm_mat4x4f_t * dst, arm_mat4x4f_t * src, unsigned int count);
extern arm_result_t invmat_3x3f_c(arm_mat3x3f_t * dst, arm_mat3x3f_t * src, unsigned int count);
extern arm_result_t invmat_2x2f_c(arm_mat2x2f_t * dst, arm_mat2x2f_t * src, unsigned int count);

extern arm_result_t transmat_4x4f_c(arm_mat4x4f_t * dst, arm_mat4x4f_t * src, unsigned int count);
extern arm_result_t identitymat_4x4f_c(arm_mat4x4f_t * dst, unsigned int count);

extern arm_result_t transmat_3x3f_c(arm_mat3x3f_t * dst, arm_mat3x3f_t * src, unsigned int count);
extern arm_result_t identitymat_3x3f_c(arm_mat3x3f_t * dst, unsigned int count);

extern arm_result_t transmat_2x2f_c(arm_mat2x2f_t * dst, arm_mat2x2f_t * src, unsigned int count);
extern arm_result_t identitymat_2x2f_c(arm_mat2x2f_t * dst, unsigned int count);



// ## Matrix-Vector Algebra ##
extern arm_result_t mulcmatvec_cm4x4f_v4f_c(arm_vec4f_t * dst, const arm_mat4x4f_t * cst, arm_vec4f_t * src, unsigned int count);
extern arm_result_t mulcmatvec_cm3x3f_v3f_c(arm_vec3f_t * dst, const arm_mat3x3f_t * cst, arm_vec3f_t * src, unsigned int count);
extern arm_result_t mulcmatvec_cm2x2f_v2f_c(arm_vec2f_t * dst, const arm_mat2x2f_t * cst, arm_vec2f_t * src, unsigned int count);


// ## Matrix-Matrix Algebra ##
extern arm_result_t multrans_mat4x4f_c(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t multrans_mat3x3f_c(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t multrans_mat2x2f_c(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);

#ifdef __cplusplus
}
#endif

#endif

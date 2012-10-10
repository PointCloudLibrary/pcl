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
 * NE10 Library : inc/NE10.h
 */

/*! \file NE10.h
    \brief All NE10 routines declarations.

    The routines that are provided by this library are all declared in this header file.
 */

//#include "../headers/versionheader.h"
#include <NE10_types.h>
#include <NE10_c.h>
#include <NE10_asm.h>
#include <NE10_neon.h>

#ifndef NE10_H
#define NE10_H

#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////
// function prototypes:
///////////////////////////


// ## Vector-Constant Arithmetic ##

/*!
    Adds a constant scalar value to all the elements of an input array and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   The constant scalar added to the input values
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*addc_float)(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
/*!
    Adds a constant 2D vector to all of the vectors in an input array and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 2D vector added to the input values
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*addc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
/*!
    Adds a constant 3D vector to all of the vectors in an input array and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 3D vector added to the input values
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*addc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
/*!
    Adds a constant 4D vector to all of the vectors in an input array and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 4D vector added to the input values
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*addc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



/*!
    Subtracts a constant scalar from all the elements of an input array and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   The constant scalar subtracted from the input values
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*subc_float)(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
/*!
    Subtracts a constant 2D vector from all of the vectors in an input array and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 2D vector subtracted from the input values
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*subc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
/*!
    Subtracts a constant 3D vector from all of the vectors in an input array and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 3D vector subtracted from the input values
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*subc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
/*!
    Subtracts a constant 4D vector from all of the vectors in an input array and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 4D vector subtracted from the input values
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*subc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



/*!
    Subtracts the elements of an input array from a constant scalar and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   The constant scalar to subtract the input values from
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*rsbc_float)(arm_float_t * dst, arm_float_t *src, const arm_float_t cst, unsigned int count);
/*!
    Subtracts the vectors in an input array from a constant 2D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 2D vector to subtract the input values from
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*rsbc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
/*!
    Subtracts the vectors in an input array from a constant 3D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 3D vector to subtract the input values from
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*rsbc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
/*!
    Subtracts the vectors in an input array from a constant 4D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 4D vector to subtract the input values from
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*rsbc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



/*!
    Multiplies the elements of an input array by a constant scalar and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   The constant scalar to multiply the input values with
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*mulc_float)(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
/*!
    Multiplies the components of 2D vectors in an input array by the components of a constant 2D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 2D vector to multiply the input values with
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*mulc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
/*!
    Multiplies the components of 3D vectors in an input array by the components of a constant 3D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 3D vector to multiply the input values with
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*mulc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
/*!
    Multiplies the components of 4D vectors in an input array by the components of a constant 4D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 4D vector to multiply the input values with
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*mulc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



/*!
    Divides the elements of an input array by a constant scalar and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   The constant scalar to divide the input values by
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*divc_float)(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
/*!
    Divides the components of 2D vectors in an input array with the components of a constant 2D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 2D vector to divide the input values by
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*divc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
/*!
    Divides the components of 3D vectors in an input array with the components of a constant 3D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 3D vector to divide the input values by
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*divc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
/*!
    Divides the components of 4D vectors in an input array with the components of a constant 4D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 4D vector to divide the input values by
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*divc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



/*!
    Sets the elements of an input array to a constant scalar and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  cst   The constant scalar to set the input values to
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*setc_float)(arm_float_t * dst, const arm_float_t cst, unsigned int count);
/*!
    Sets the components of 2D vectors in an input array to the components of a constant 2D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  cst   Pointer to the 2D vector to set the input values to
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*setc_vec2f)(arm_vec2f_t * dst, const arm_vec2f_t * cst, unsigned int count);
/*!
    Sets the components of 3D vectors in an input array to the components of a constant 3D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  cst   Pointer to the 3D vector to set the input values to
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*setc_vec3f)(arm_vec3f_t * dst, const arm_vec3f_t * cst, unsigned int count);
/*!
    Sets the components of 3D vectors in an input array to the components of a constant 3D vector and stores the results in an output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  cst   Pointer to the 4D vector to set the input values to
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*setc_vec4f)(arm_vec4f_t * dst, const arm_vec4f_t * cst, unsigned int count);



/*!
    Multiplies each entry in the source array (src) by cst, then adds the result to
     the corresponding item of the accumulation array (acc), and stores the result in the destination array.
    @param[out] dst   Pointer to the destination array
    @param[in]  acc   The corresponding elemetn is added to the result of the multiplication
    @param[in]  src   Pointer to the source array
    @param[in]  cst   The constant scalar to multiply the input elements with
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*mlac_float)(arm_float_t * dst, arm_float_t * acc, arm_float_t * src, const arm_float_t cst, unsigned int count);
/*!
   Multiplies each entry in the source array (src) by the 2D vector cst, then adds the result to
     the corresponding item of the accumulation array (acc), and stores the result in the destination array.
    @param[out] dst   Pointer to the destination array
    @param[in]  acc   The corresponding elemetn is added to the result of the multiplication
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 2D vector to multiply the input vectors with
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*mlac_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * acc, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
/*!
   Multiplies each entry in the source array (src) by the 3D vector cst, then adds the result to
     the corresponding item of the accumulation array (acc), and stores the result in the destination array.
    @param[out] dst   Pointer to the destination array
    @param[in]  acc   The corresponding elemetn is added to the result of the multiplication
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 3D vector to multiply the input vectors with
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*mlac_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * acc, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
/*!
   Multiplies each entry in the source array (src) by the 4D vector cst, then adds the result to
     the corresponding item of the accumulation array (acc), and stores the result in the destination array.
    @param[out] dst   Pointer to the destination array
    @param[in]  acc   The corresponding elemetn is added to the result of the multiplication
    @param[in]  src   Pointer to the source array
    @param[in]  cst   Pointer to the 4D vector to multiply the input vectors with
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*mlac_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * acc, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);



// ## Arithmetic functions over arrays of cst values ##

/*!
    Adds the elements of src1 to the elements of src2 and stores the results in the dst.
    @param[out] dst   Pointer to the destination array
    @param[in]  src1  The first array to use as the input array
    @param[in]  src2  The second array to use as the input array
    @param[in]  count The number of items in the two input arrays
 */
extern arm_result_t (*add_float)(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
/*!
    Subtracts the elements of src2 from the elements of src2 and stores the results in the dst.
    @param[out] dst   Pointer to the destination array
    @param[in]  src1  The first array to use as the input array
    @param[in]  src2  The second array to use as the input array
    @param[in]  count The number of items in the two input arrays
 */
extern arm_result_t (*sub_float)(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
/*!
    Multiplies the elements of src1 by the elements of src2 and stores the results in the dst.
    @param[out] dst   Pointer to the destination array
    @param[in]  src1  The first array to use as the input array
    @param[in]  src2  The second array to use as the input array
    @param[in]  count The number of items in the two input arrays
 */
extern arm_result_t (*mul_float)(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
/*!
    Divides the elements of src1 by the elements of src2 and stores the results in the dst.
    @param[out] dst   Pointer to the destination array
    @param[in]  src1  The first array to use as the input array
    @param[in]  src2  The second array to use as the input array
    @param[in]  count The number of items in the two input arrays
 */
extern arm_result_t (*div_float)(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
/*!
    Performs a multiply and accumulate operation using the corresponding elements in acc, src1, and src2.
    @param[out] dst   Pointer to the destination array
    @param[in]  acc   These elemtns are added to the result of the multiplication operation
    @param[in]  src1  The first array to use as the input array
    @param[in]  src2  The second array to use as the input array
    @param[in]  count The number of items in the two input arrays
 */
extern arm_result_t (*mla_float)(arm_float_t * dst, arm_float_t * acc, arm_float_t * src1, arm_float_t * src2, unsigned int count);
/*!
    Calculates the absolute value of each element in the source array and stores the result in the corresponding entry of the destination array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*abs_float)(arm_float_t * dst, arm_float_t * src, unsigned int count);



// ## Operations on Vectors ##
/*!
    Returns length of 2D vectors in corresponding elements of the output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*len_vec2f)(arm_float_t * dst, arm_vec2f_t * src, unsigned int count);
/*!
    Returns length of 3D vectors in corresponding elements of the output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*len_vec3f)(arm_float_t * dst, arm_vec3f_t * src, unsigned int count);
/*!
    Returns length of 4D vectors in corresponding elements of the output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*len_vec4f)(arm_float_t * dst, arm_vec4f_t * src, unsigned int count);



/*!
    Normalizes 2D vectors of the input array and stores them in the corresponding elements of the output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*normalize_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, unsigned int count);
/*!
    Normalizes 3D vectors of the input array and stores them in the corresponding elements of the output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*normalize_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, unsigned int count);
/*!
    Normalizes 4D vectors of the input array and stores them in the corresponding elements of the output array.
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*normalize_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, unsigned int count);




/*!
    Generates a 2D vector from the absolute values of each of the components of an input vector
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*abs_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, unsigned int count);
/*!
    Generates a 3D vector from the absolute values of each of the components of an input vector
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*abs_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, unsigned int count);
/*!
    Generates a 4D vector from the absolute values of each of the components of an input vector
    @param[out] dst   Pointer to the destination array
    @param[in]  src   Pointer to the source array
    @param[in]  count The number of items in the input array
 */
extern arm_result_t (*abs_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, unsigned int count);



// ## SIMD Component-wise Arithmetic on Two Vectors ##

/*!
    Multiplies the components of a 2D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vmul_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
/*!
    Multiplies the components of a 3D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vmul_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
/*!
    Multiplies the components of a 4D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vmul_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



/*!
    Divides the components of a 2D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the nominators' source array
    @param[in]  src2   Pointer to the denominators' source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vdiv_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
/*!
    Divides the components of a 3D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the nominators' source array
    @param[in]  src2   Pointer to the denominators' source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vdiv_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
/*!
    Divides the components of a 4D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the nominators' source array
    @param[in]  src2   Pointer to the denominators' source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vdiv_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



/*!
    Performs a multiply and accumulate operation on the components of a 2D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vmla_vec2f)(arm_vec2f_t * acc, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
/*!
    Performs a multiply and accumulate operation on the components of a 3D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vmla_vec3f)(arm_vec3f_t * acc, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
/*!
    Performs a multiply and accumulate operation on the components of a 4D vector with the corresponding components of another
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*vmla_vec4f)(arm_vec4f_t * acc, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



// ## Vector-Vector Algebra ##

/*!
    Vector addition of two 2D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*add_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
/*!
    Vector addition of two 3D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*add_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
/*!
    Vector addition of two 4D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*add_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



/*!
    Vector subtraction of two 2D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*sub_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
/*!
    Vector subtraction of two 3D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*sub_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
/*!
    Vector subtraction of two 4D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*sub_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



/*!
    Dot product of two 2D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*dot_vec2f)(arm_float_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
/*!
    Dot product of two 3D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*dot_vec3f)(arm_float_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
/*!
    Dot product of two 4D vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*dot_vec4f)(arm_float_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);



/*!
    Performs a cross product operation on the two input vectors
    @param[out] dst   Pointer to the destination array
    @param[in]  src1   Pointer to the first source array
    @param[in]  src2   Pointer to the second source array
    @param[in]  count The number of items in the input arrays
 */
extern arm_result_t (*cross_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);




// ## Matrix-Constant Arithmetic ##

// arm_mat4x4f_t
extern arm_result_t (*addmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t (*submat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t (*mulmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t (*divmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t (*setmat_4x4f)(arm_mat4x4f_t * dst, const arm_float_t cst, unsigned int count);

extern arm_result_t (*addmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t (*submat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t (*mulmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t (*divmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t (*setmat_3x3f)(arm_mat3x3f_t * dst, const arm_float_t cst, unsigned int count);

extern arm_result_t (*addmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
extern arm_result_t (*submat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
extern arm_result_t (*mulmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
extern arm_result_t (*divmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
extern arm_result_t (*setmat_2x2f)(arm_mat2x2f_t * dst, const arm_float_t cst, unsigned int count);



// ## Operations on Matrices ##

extern arm_result_t (*detmat_4x4f)(arm_float_t * dst, arm_mat4x4f_t * src, unsigned int count);
extern arm_result_t (*detmat_3x3f)(arm_float_t * dst, arm_mat3x3f_t * src, unsigned int count);
extern arm_result_t (*detmat_2x2f)(arm_float_t * dst, arm_mat2x2f_t * src, unsigned int count);

extern arm_result_t (*invmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src, unsigned int count);
extern arm_result_t (*invmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src, unsigned int count);
extern arm_result_t (*invmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src, unsigned int count);

extern arm_result_t (*transmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src, unsigned int count);
extern arm_result_t (*identitymat_4x4f)(arm_mat4x4f_t * dst, unsigned int count);

extern arm_result_t (*transmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src, unsigned int count);
extern arm_result_t (*identitymat_3x3f)(arm_mat3x3f_t * dst, unsigned int count);

extern arm_result_t (*transmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src, unsigned int count);
extern arm_result_t (*identitymat_2x2f)(arm_mat2x2f_t * dst, unsigned int count);



// ## Matrix-Vector Algebra ##
extern arm_result_t (*mulcmatvec_cm4x4f_v4f)(arm_vec4f_t * dst, const arm_mat4x4f_t * cst, arm_vec4f_t * src, unsigned int count);
extern arm_result_t (*mulcmatvec_cm3x3f_v3f)(arm_vec3f_t * dst, const arm_mat3x3f_t * cst, arm_vec3f_t * src, unsigned int count);
extern arm_result_t (*mulcmatvec_cm2x2f_v2f)(arm_vec2f_t * dst, const arm_mat2x2f_t * cst, arm_vec2f_t * src, unsigned int count);


// ## Matrix-Matrix Algebra ##
extern arm_result_t (*multrans_mat4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
extern arm_result_t (*multrans_mat3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
extern arm_result_t (*multrans_mat2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);

#ifdef __cplusplus
}
#endif

#endif

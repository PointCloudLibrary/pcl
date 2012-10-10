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
 * NE10 Library : inc/NE10_types.h
 */

/** NE10 defines a number of types for use in its function signatures.
 *  The types are defined within this header file.
 */

#ifndef NE10_TYPES_H
#define NE10_TYPES_H

#include <stdio.h>
#include <assert.h>

/////////////////////////////////////////////////////////
// constant values that are used across the library
/////////////////////////////////////////////////////////
#define NE10_OK 0
#define NE10_ERR -1

/////////////////////////////////////////////////////////
// some external definitions to be exposed to the users
/////////////////////////////////////////////////////////
typedef float arm_float_t;      // a single float value
typedef int   arm_result_t;     // resulting [error-]code

typedef struct
{
        float x;
        float y;
} arm_vec2f_t; // a 2-tuple of float values

typedef struct
{
        float x;
        float y;
        float z;
} arm_vec3f_t; // a 3-tuple of float values

typedef struct
{
        float x;
        float y;
        float z;
        float w;
} arm_vec4f_t; // a 4-tuple of float values


typedef struct { float r1; float r2; } __attribute__((packed)) arm_mat_row2f;

typedef struct
{
        arm_mat_row2f c1;
        arm_mat_row2f c2;

} __attribute__((packed)) arm_mat2x2f_t;     // a 2x2 matrix

static inline void createColumnMajorMatrix2x2( arm_mat2x2f_t * outMat, arm_float_t m11, arm_float_t m21, arm_float_t m12, arm_float_t m22)
{
   assert( NULL != outMat );

    outMat->c1.r1 = m11;
    outMat->c1.r2 = m21;
    outMat->c2.r1 = m12;
    outMat->c2.r2 = m22;
}


typedef struct { float r1; float r2; float r3; } __attribute__((packed)) arm_mat_row3f;

typedef struct
{
        arm_mat_row3f c1;
        arm_mat_row3f c2;
        arm_mat_row3f c3;

} __attribute__((packed)) arm_mat3x3f_t;     // a 3x3 matrix

static inline void createColumnMajorMatrix3x3( arm_mat3x3f_t * outMat, arm_float_t m11, arm_float_t m21, arm_float_t m31,
                                                                       arm_float_t m12, arm_float_t m22, arm_float_t m32,
                                                                       arm_float_t m13, arm_float_t m23, arm_float_t m33)
{
    assert( NULL != outMat );

    outMat->c1.r1 = m11;
    outMat->c1.r2 = m21;
    outMat->c1.r3 = m31;

    outMat->c2.r1 = m12;
    outMat->c2.r2 = m22;
    outMat->c2.r3 = m32;

    outMat->c3.r1 = m13;
    outMat->c3.r2 = m23;
    outMat->c3.r3 = m33;
}


typedef struct { float r1; float r2; float r3; float r4; } __attribute__((packed)) arm_mat_row4f;

typedef struct
{
        arm_mat_row4f c1;
        arm_mat_row4f c2;
        arm_mat_row4f c3;
        arm_mat_row4f c4;

} __attribute__((packed)) arm_mat4x4f_t;     // a 4x4 matrix

static inline void createColumnMajorMatrix4x4( arm_mat4x4f_t * outMat, arm_float_t m11, arm_float_t m21, arm_float_t m31, arm_float_t m41,
                                                                       arm_float_t m12, arm_float_t m22, arm_float_t m32, arm_float_t m42,
                                                                       arm_float_t m13, arm_float_t m23, arm_float_t m33, arm_float_t m43,
                                                                       arm_float_t m14, arm_float_t m24, arm_float_t m34, arm_float_t m44)
{
    assert( NULL != outMat );

    outMat->c1.r1 = m11;
    outMat->c1.r2 = m21;
    outMat->c1.r3 = m31;
    outMat->c1.r4 = m41;

    outMat->c2.r1 = m12;
    outMat->c2.r2 = m22;
    outMat->c2.r3 = m32;
    outMat->c2.r4 = m42;

    outMat->c3.r1 = m13;
    outMat->c3.r2 = m23;
    outMat->c3.r3 = m33;
    outMat->c3.r4 = m43;

    outMat->c4.r1 = m14;
    outMat->c4.r2 = m24;
    outMat->c4.r3 = m34;
    outMat->c4.r4 = m44;
}


#endif

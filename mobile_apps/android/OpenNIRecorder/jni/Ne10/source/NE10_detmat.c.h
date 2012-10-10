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
 * NE10 Library : source/NE10_detmat.c.h
 */

#ifndef __NE10_DETMAT_C_H__
#define __NE10_DETMAT_C_H__

#include "NE10.h"
#include "../headers/macros.h"

#include <assert.h>

static inline arm_float_t DET2x2( arm_mat2x2f_t * mat )
{
    // 2x2 matrix layout
    //  c1r1 c2r1
    //  c1r2 c2r2

    return (  (mat->c1.r1 * mat->c2.r2)
             -(mat->c2.r1 * mat->c1.r2) );
}

static inline arm_float_t DET3x3( arm_mat3x3f_t * mat )
{
    // 3x3 matrix layout
    //  c1r1 c2r1 c3r1
    //  c1r2 c2r2 c3r2
    //  c1r3 c2r3 c3r3

    arm_mat2x2f_t subm11 = { {mat->c2.r2, mat->c2.r3}, {mat->c3.r2, mat->c3.r3} };
    arm_mat2x2f_t subm21 = { {mat->c1.r2, mat->c1.r3}, {mat->c3.r2, mat->c3.r3} };
    arm_mat2x2f_t subm31 = { {mat->c1.r2, mat->c1.r3}, {mat->c2.r2, mat->c2.r3} };
    return    (mat->c1.r1*DET2x2( &subm11 ))
            - (mat->c2.r1*DET2x2( &subm21 ))
            + (mat->c3.r1*DET2x2( &subm31 ));
}

static inline arm_float_t DET4x4( arm_mat4x4f_t * mat )
{
    // 4x4 matrix layout
    //  c1r1 c2r1 c3r1 c4r1
    //  c1r2 c2r2 c3r2 c4r2
    //  c1r3 c2r3 c3r3 c4r3
    //  c1r4 c2r4 c3r4 c4r4

    arm_mat3x3f_t subm11 = { {mat->c2.r2, mat->c2.r3, mat->c2.r4},
                             {mat->c3.r2, mat->c3.r3, mat->c3.r4},
                             {mat->c4.r2, mat->c4.r3, mat->c4.r4} };

    arm_mat3x3f_t subm21 = { {mat->c1.r2, mat->c1.r3, mat->c1.r4},
                             {mat->c3.r2, mat->c3.r3, mat->c3.r4},
                             {mat->c4.r2, mat->c4.r3, mat->c4.r4} };

    arm_mat3x3f_t subm31 = { {mat->c1.r2, mat->c1.r3, mat->c1.r4},
                             {mat->c2.r2, mat->c2.r3, mat->c2.r4},
                             {mat->c4.r2, mat->c4.r3, mat->c4.r4} };

    arm_mat3x3f_t subm41 = { {mat->c1.r2, mat->c1.r3, mat->c1.r4},
                             {mat->c2.r2, mat->c2.r3, mat->c2.r4},
                             {mat->c3.r2, mat->c3.r3, mat->c3.r4} };

    return    (mat->c1.r1*DET3x3( &subm11 ))
            - (mat->c2.r1*DET3x3( &subm21 ))
            + (mat->c3.r1*DET3x3( &subm31 ))
            - (mat->c4.r1*DET3x3( &subm41 ));
}




#endif

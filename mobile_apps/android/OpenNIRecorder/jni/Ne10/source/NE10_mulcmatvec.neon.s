@
@  Copyright 2011-12 ARM Limited
@
@  Licensed under the Apache License, Version 2.0 (the "License");
@  you may not use this file except in compliance with the License.
@  You may obtain a copy of the License at
@
@      http://www.apache.org/licenses/LICENSE-2.0
@
@  Unless required by applicable law or agreed to in writing, software
@  distributed under the License is distributed on an "AS IS" BASIS,
@  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
@  See the License for the specific language governing permissions and
@  limitations under the License.
@

@
@ NE10 Library : source/NE10_mulcmatvec.neon.s
@




        .text
        .syntax   unified

.include "headers/NE10header.s"




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro multiplies a single 2x2 matrix by eight vec2's
        @ The elements of the vectors are loaded into registers q8-q11
        @ by the caller (mulcmatvec_cm2x2f_v2f_neon) in the following
        @ order:
        @
        @       d16=(x1,x3) d18=(y1,y3) d20=(x2,x4) d22=(y2,y4);
        @       d17=(x5,x7) d19=(y5,y7) d21=(x6,x8) d23=(y6,y8);
        @
        @ This macro multiplies these eight vectors by the 2x2 matrix
        @ which is stored in registers d0[0],d1[0],d2[0], and d3[0].
        @ The resulting eight vectors are returned in q12-q15
        @ in the same order as shown above.
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro MUL_MAT2x2_VEC2
          vmul.f32        q10,  q8 ,  d0[0]    @ a*x1,x2,x3,x4
          vmul.f32        q8 ,  q8 ,  d1[0]    @ b*x1,x2,x3,x4
          vmul.f32        q11,  q9 ,  d2[0]    @ c*y1,y2,y3,y4
          vmul.f32        q9 ,  q9 ,  d3[0]    @ d*y1,y2,y3,y4

          vadd.f32        q12,  q10,  q11      @ 3) res24.x = a*(x1,x2,x3,x4) + c*(y1,y2,y3,y4)   @ These results need to be stored in the order noted
          vadd.f32        q13,  q8,   q9       @ 4) res24.y = b*(x1,x2,x3,x4) + d*(y1,y2,y3,y4)
        .endm




        .balign   4
        .global   mulcmatvec_cm2x2f_v2f_neon
        .thumb
        .thumb_func

mulcmatvec_cm2x2f_v2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @  arm_result_t mulcmatvec_cm2x2f_v2f ( arm_vec2f_t * dst,
        @                                       const arm_mat2x2f_t * cst,
        @                                       arm_vec2f_t * src,
        @                                       unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @      (this register is updated and mvoed to the next entry
        @       after every store operation)
        @  r1: *cst, memory pointer to where the constant matrix is kept
        @  r2: *src & current src entry's address
        @  r3: int count & the number of items in the input array
        @
        @  r4:  the number of items that are left to be processed at the
        @                   end of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4;
        sub               r3, r3, r4          @ count = count - r3; This is what's left to be processed after this loop

        @ First we load the constant 2x2 matrix, then each time we load
        @ eight vectors of 2-floats, multiply each vector with the matrix,
        @ finally store the resutlting vectors in the destination memory
        @ address, and move on to the next four vectors.

        @ load the constant matrix
        @ d0 = m11(a)  d2 = m12(c)
        @ d1 = m21(b)  d3 = m22(d)
        vld4.32         { d0[0], d1[0], d2[0], d3[0] }, [r1]

        cmp               r3, #0
        beq               .L_check_mat2x2

        @ load the 1st set of values
        @ if {V1, V2, V3, V4} are 4 vec2's in memory
        @ then after the load operations the 4 vectors
        @ are stored in registers q8-q9 like so:
        @
        @       q8=(x1,x2,x3,x4)
        @       q9=(y1,y2,y3,y4)

        vld2.32         {  d16,  d17,  d18,  d19 }, [r2]!

        subs            r3, r3, #4          @ 8 for this set

        @ calculate values for the 1st set
        MUL_MAT2x2_VEC2

        ble             .L_mainloopend_mat2x2

.L_mainloop_mat2x2:
        @ store the result for the current set
        vst2.32        { d24, d25, d26, d27 }, [r0]!

        @ load the next set of values
        vld2.32         {  d16,  d17,  d18,  d19 }, [r2]!
        subs            r3, r3, #4

        @ calculate values for the next set
        MUL_MAT2x2_VEC2

        bgt             .L_mainloop_mat2x2             @ loop if r2 is > r3, if we have at least another 4 vectors (8 floats) to process

.L_mainloopend_mat2x2:
        @ the last iteration for this call
        @ store the result for the last set
        vst2.32        { d24, d25, d26, d27 }, [r0]!

.L_check_mat2x2:
        @ check if anything left to process at the end of the input array
        cmp               r4, #0
        ble               .L_return_mat2x2

.L_secondloop_mat2x2:
        @ process the last few items left in the input array
        vld2.32         {  d16[0],  d18[0] }, [r2]!

        subs              r4, r4, #1

        @ calculate values
        MUL_MAT2x2_VEC2

        @ store the results
        vst2.32        { d24[0], d26[0] }, [r0]!

        bgt               .L_secondloop_mat2x2

.L_return_mat2x2:
       @ return
        pop               {r4}
        mov               r0, #0
        bx                lr




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro to load four vec3's into registers q8-q10
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro LOAD_FOUR_VEC3
            vld3.32         { d16, d18, d20  }, [r2]!
            vld3.32         { d17, d19, d21  }, [r2]!
        .endm


        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro multiplies the constant 3x3 matrix loaded into
        @ registers d0-d5 by four vec3's that the above macro LOAD_FOUR_VEC3
        @ loads. The resuls are returned in registers q11, q12, and and q13
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro MUL_MAT3x3_VEC3

          vmul.f32        q11,  q8 ,  d0[0]
          vmla.f32        q11,  q9 ,  d0[1]
          vmla.f32        q11,  q10,  d1[0]

          vmul.f32        q12,  q8 ,  d2[0]
          vmla.f32        q12,  q9 ,  d2[1]
          vmla.f32        q12,  q10,  d3[0]

          vmul.f32        q13,  q8 ,  d4[0]
          vmla.f32        q13,  q9 ,  d4[1]
          vmla.f32        q13,  q10,  d5[0]

        .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro to store the resulting vec3's that were returned in
        @ registers q11 to q13 in the above macro MUL_MAT3x3_VEC3.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro STORE_FOUR_VEC3

            vst3.32         { d22, d24, d26  }, [r0]!
            vst3.32         { d23, d25, d27  }, [r0]!

        .endm




        .align  2
        .global   mulcmatvec_cm3x3f_v3f_neon
        .thumb
        .thumb_func

mulcmatvec_cm3x3f_v3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @  arm_result_t mulcmatvec_cm3x3f_v3f ( arm_vec3f_t * dst,
        @                                       const arm_mat3x3f_t * cst,
        @                                       arm_vec3f_t * src,
        @                                       unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @      (this register is updated and mvoed to the next entry
        @       after every store operation)
        @  r1: *cst, memory pointer to where the constant matrix is kep
        @  r2: *src & current src entry's gddress
        @  r3: int count & the number of items in the input array
        @
        @  r4:  the number of items that are left to be processed at the
        @                   end of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              { r4 }
        and               r4, r3, #3          @ r3 = count % 4;
        sub               r3, r3, r4          @ count = count - r3; This is what's left to be processed after this loop

        @ First we load the constant 3x3 matrix, then each time we load
        @ four vectors of 3-floats, multiply each vector with the matrix,
        @ finally store the resutlting vectors in the destination memory
        @ address, and move on to the next four vectors.

        @ load the constant matrix into q0-q2
        vld3.32         { d0   , d2   , d4    }, [r1]!
        vld3.32         { d1[0], d3[0], d5[0] }, [r1]

        cmp               r3, #0
        beq               .L_check_mat3x3


        @ load the 1st set of values
        LOAD_FOUR_VEC3
        subs            r3, r3, #4          @ 4 for this set

        @ calculate values for the 1st set
        MUL_MAT3x3_VEC3

          ble             .L_mainloopend_mat3x3

.L_mainloop_mat3x3:
        @ store the result for the current set
        STORE_FOUR_VEC3

        @ load the next set of values
        LOAD_FOUR_VEC3
        subs            r3, r3, #4

        @ calculate values for the next set
        MUL_MAT3x3_VEC3

        bgt               .L_mainloop_mat3x3             @ loop if r2 is > r3, if we have at least another 4 vectors (12 floats) to process

.L_mainloopend_mat3x3:
        @ the last iteration for this call
        @ store the result for the last set
        STORE_FOUR_VEC3

.L_check_mat3x3:
        @ check if anything left to process at the end of the input array
        cmp               r4, #0
        ble               .L_return_mat3x3

.L_secondloop_mat3x3:
        @ process the last few items left in the input array
        vld3.32         { d16[0], d18[0], d20[0]  }, [r2]!

        subs            r4, r4, #1

        MUL_MAT3x3_VEC3

        vst3.32         { d22[0], d24[0], d26[0]  }, [r0]!

        bgt               .L_secondloop_mat3x3

.L_return_mat3x3:
        @ return
        pop               { r4 }
        mov               r0, #0
        bx                lr




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro to load four vec4's into registers q8-q11.
        @ This macro uses r2 (the thirs parameter in
        @ mulcmatvec_cm4x4f_v4f_neon) as the address register.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro LOAD_FOUR_VEC4
            vld4.32         { d16, d18, d20, d22  }, [r2]!
            vld4.32         { d17, d19, d21, d23  }, [r2]!
        .endm


        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro multiplies the constant 4x4 matrix that is loaded
        @ in mulcmatvec_cm4x4f_v4f_neon by four vec4's that are loaded in
        @ the above macro LOAD_FOUR_VEC4.
        @ The resulting four vectors are returned in registers q12 to q15.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro MUL_MAT4x4_VEC4

          vmul.f32        q12,  q8 ,  d0[0]
          vmla.f32        q12,  q9 ,  d0[1]
          vmla.f32        q12,  q10,  d1[0]
          vmla.f32        q12,  q11,  d1[1]

          vmul.f32        q13,  q8 ,  d2[0]
          vmla.f32        q13,  q9 ,  d2[1]
          vmla.f32        q13,  q10,  d3[0]
          vmla.f32        q13,  q11,  d3[1]

          vmul.f32        q14,  q8 ,  d4[0]
          vmla.f32        q14,  q9 ,  d4[1]
          vmla.f32        q14,  q10,  d5[0]
          vmla.f32        q14,  q11,  d5[1]

          vmul.f32        q15,  q8 ,  d6[0]
          vmla.f32        q15,  q9 ,  d6[1]
          vmla.f32        q15,  q10,  d7[0]
          vmla.f32        q15,  q11,  d7[1]

        .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro stores the results from the above macro MUL_MAT4x4_VEC4
        @ from registers q12-q15 in to the destination memory (r0) which is
        @ the first parameter of mulcmatvec_cm4x4f_v4f_neon().
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro STORE_FOUR_VEC4

            vst4.32         { d24, d26, d28, d30  }, [r0]!
            vst4.32         { d25, d27, d29, d31  }, [r0]!

        .endm




        .align  2
        .global   mulcmatvec_cm4x4f_v4f_neon
        .thumb
        .thumb_func

mulcmatvec_cm4x4f_v4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @  arm_result_t mulcmatvec_cm4x4f_v4f ( arm_vec4f_t * dst,
        @                                       const arm_mat4x4f_t * cst,
        @                                       arm_vec4f_t * src,
        @                                       unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @      (this register is updated and mvoed to the next entry
        @       after every store operation)
        @  r1: *cst, pointer to memory where the constant matrix is kept
        @  r2: *src & current src entry's address
        @  r3: int count & the number of items in the input array
        @
        @  r4:  the number of items that are left to be processed at the
        @                   end of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4;
        sub               r3, r3, r4          @ count = count - r4; This is what's left to be processed after this loop

        @ First we load the constant 4x4 matrix, then each time we load
        @ four vectors of 4-floats, multiply each vector with the matrix,
        @ finally store the resutlting vectors in the destination memory
        @ address, and move on to the next four vectors.

        @ load the constant matrix into q0-q3
        vld4.32         { d0, d2, d4, d6 }, [r1]!
        vld4.32         { d1, d3, d5, d7 }, [r1]

        cmp               r3, #0
        beq               .L_check_mat4x4

        @ load the 1st set of values
        LOAD_FOUR_VEC4
        subs            r3, r3, #4

        @ calculate values for the 1st set
        MUL_MAT4x4_VEC4

        ble             .L_mainloopend_mat4x4

.L_mainloop_mat4x4:
        @ store the result for the current set
        STORE_FOUR_VEC4

        @ load the next set of values
        LOAD_FOUR_VEC4
        subs            r3, r3, #4

        @ calculate values for the next set
        MUL_MAT4x4_VEC4

        bgt               .L_mainloop_mat4x4             @ loop if r2 is > r3, if we have at least another 4 vectors (16 floats) to process

.L_mainloopend_mat4x4:
        @ the last iteration for this call
        @ store the result for the last set
        STORE_FOUR_VEC4

.L_check_mat4x4:
        @ check if anything left to process at the end of the input array
        cmp               r4, #0
        ble               .L_return_mat4x4

.L_secondloop_mat4x4:
        @ process the last few items left in the input array
        vld4.32         { d16[0], d18[0], d20[0], d22[0]  }, [r2]!

        subs            r4, r4, #1

        @ calculate values
        MUL_MAT4x4_VEC4

        @ store the results
        vst4.32         { d24[0], d26[0], d28[0], d30[0]  }, [r0]!

        bgt               .L_secondloop_mat4x4

.L_return_mat4x4:
        @ return
        pop               {r4}
        mov               r0, #0
        bx                lr

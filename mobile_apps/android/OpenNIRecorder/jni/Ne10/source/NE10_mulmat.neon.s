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
@ NE10 Library : source/NE10_mulmat.neon.s
@




        .text
        .syntax   unified

.include "headers/NE10header.s"




        .balign   4
        .global   mulmat_2x2f_neon
        .thumb
        .thumb_func

mulmat_2x2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t mulmat_2x2f(arm_mat2x2f_t * dst,
        @                 arm_mat2x2f_t * src1,
        @                 arm_mat2x2f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: *src2 & current src2 entry's address
        @  r3: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r4:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4;
        sub               r3, r3, r4          @ count = count - r3; This is what's left to be processed after this loop

        cmp               r3, #0
        beq               .L_check_mat2x2

        @ We load four 2x2 matrices at a time, multiply them to
        @ get two resulting 2x2 matrices, store them in the destination
        @ and then move on to the next four matrices.

        @ load the 1st set of values
          vld4.32         {  d0,  d1,  d2,  d3 }, [r1]!
          vld4.32         {  d4,  d5,  d6,  d7 }, [r2]!
          subs            r3, r3, #4          @ 2 for this set, and 2 for the 2nd set

        @ calculate values for the 1st set
          vmul.f32        d16,  d0,  d4
          vmul.f32        d17,  d1,  d4
          vmul.f32        d18,  d0,  d6
          vmul.f32        d19,  d1,  d6

          vmla.f32        d16,  d2,  d5
          vmla.f32        d17,  d3,  d5
          vmla.f32        d18,  d2,  d7
          vmla.f32        d19,  d3,  d7


        @ load the 2nd set of values
          vld4.32         {  d0,  d1,  d2,  d3 }, [r1]!
          vld4.32         {  d4,  d5,  d6,  d7 }, [r2]!

          ble             .L_mainloopend_mat2x2

.L_mainloop_mat2x2:
        @ store the result for the 1st/next (e.g. 3rd) set
          vst4.32         { d16, d17, d18, d19}, [r0]!

        @ calculate values for the 2nd/next (e.g. 3rd) set
          vmul.f32        d16,  d0,  d4
          vmul.f32        d17,  d1,  d4
          vmul.f32        d18,  d0,  d6
          vmul.f32        d19,  d1,  d6

          vmla.f32        d16,  d2,  d5
          vmla.f32        d17,  d3,  d5
          vmla.f32        d18,  d2,  d7
          vmla.f32        d19,  d3,  d7

       @ load the next (e.g. 3rd) set of values
          subs            r3, r3, #2
          vld4.32         {  d0,  d1,  d2,  d3 }, [r1]!
          vld4.32         {  d4,  d5,  d6,  d7 }, [r2]!


        bgt             .L_mainloop_mat2x2             @ loop if r2 is > r3, if we have at least another 4 vectors (8 floats) to process

.L_mainloopend_mat2x2:
        @ the last iteration for this call
        @ store the result for the set of values before the last one (e.g 2nd set)
          vst4.32         { d16, d17, d18, d19}, [r0]!

        @ calculate values for the last (e.g. 3rd) set
          vmul.f32        d16,  d0,  d4
          vmul.f32        d17,  d1,  d4
          vmul.f32        d18,  d0,  d6
          vmul.f32        d19,  d1,  d6

          vmla.f32        d16,  d2,  d5
          vmla.f32        d17,  d3,  d5
          vmla.f32        d18,  d2,  d7
          vmla.f32        d19,  d3,  d7

        @ store the result for the last (e.g. 3rd) set
          vst4.32         { d16, d17, d18, d19}, [r0]!


.L_check_mat2x2:
     @ check if anything left to process at the end of the input array
        cmp               r4, #0
        ble               .L_return_mat2x2

.L_secondloop_mat2x2:
     @ process the last few items left in the input array
       vld4.32         {  d0[0],  d1[0],  d2[0],  d3[0] }, [r1]!
       vld4.32         {  d4[0],  d5[0],  d6[0],  d7[0] }, [r2]!

        subs              r4, r4, #1

        @ calculate values
          vmul.f32        d16,  d0,  d4
          vmul.f32        d17,  d1,  d4
          vmul.f32        d18,  d0,  d6
          vmul.f32        d19,  d1,  d6

          vmla.f32        d16,  d2,  d5
          vmla.f32        d17,  d3,  d5
          vmla.f32        d18,  d2,  d7
          vmla.f32        d19,  d3,  d7

       vst4.32           { d16[0], d17[0], d18[0], d19[0] }, [r0]!

        bgt               .L_secondloop_mat2x2

.L_return_mat2x2:
     @ return
        pop               {r4}
        mov               r0, #0
        bx                lr




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro to load four 3x3 matrices, two from the first source which
        @ according to the function signatures is src1 (r1) and
        @ another two from the second source which is src2 (r2)
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro LOAD_3x3MATS

            # load two 3x3 matrices from src1
            vld1.32         {  q0-q1 }, [r1]!
            vld1.32         {  d8[0] }, [r1]!
            vld1.32         {  q2-q3 }, [r1]!
            vld1.32         {  d8[1] }, [r1]!

            # load two 3x3 matrices from src2
            vld1.32         {   q8-q9 }, [r2]!
            vld1.32         {   d9[0] }, [r2]!
            vld1.32         { q10-q11 }, [r2]!
            vld1.32         {   d9[1] }, [r2]!


             # rearrange them both
             vtrn.32     q0,  q2
             vtrn.32     q1,  q3

             vtrn.32     q8, q10
             vtrn.32     q9, q11

        .endm




       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       @ This macro multiplies two pairs of 3x3 matrices that were
       @ loaded using the above LOAD_3x3MATS macro in registers q0-q11.
       @ The two resulting matrices are returned in q12, q13, q14, q15, & d9 
       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       .macro MULTIPLY_3x3MATS

           @ a =  d0  &  d16
           @ b =  d4  &  d20
           @ c =  d1  &  d17
           @ d =  d5  &  d21
           @ e =  d2  &  d18
           @ f =  d6  &  d22
           @ g =  d3  &  d19
           @ h =  d7  &  d23
           @ i =  d8  &   d9
 
           vmul.f32     d24, d0, d16
           vmul.f32     d28, d4, d16
           vmul.f32     d25, d1, d16
           vmul.f32     d29, d0, d21
           vmul.f32     d26, d4, d21
           vmul.f32     d30, d1, d21
           vmul.f32     d27, d0, d19
           vmul.f32     d31, d4, d19
           vmul.f32     d10, d1, d19

           vmla.f32     d24, d5, d20
           vmla.f32     d28, d2, d20
           vmla.f32     d25, d6, d20
           vmla.f32     d29, d5, d18
           vmla.f32     d26, d2, d18
           vmla.f32     d30, d6, d18
           vmla.f32     d27, d5, d23
           vmla.f32     d31, d2, d23
           vmla.f32     d10, d6, d23

           vmla.f32     d24, d3, d17
           vmla.f32     d28, d7, d17
           vmla.f32     d25, d8, d17
           vmla.f32     d29, d3, d22
           vmla.f32     d26, d7, d22
           vmla.f32     d30, d8, d22
           vmla.f32     d27, d3,  d9
           vmla.f32     d31, d7,  d9
           vmla.f32     d10, d8,  d9

       .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro to store the two resulting 3x3 matrices from
        @ the above MULTIPLY_3x3MATS macro (q12-q15, & d9 are stored)
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro STORE_3x3MATS

             # rearrange them both
             vtrn.32     q12,  q14
             vtrn.32     q13,  q15

            # store two 3x3 matrices to dst
            vst1.32         { q12-q13 }, [r0]!
            vst1.32         {  d10[0] }, [r0]!
            vst1.32         { q14-q15 }, [r0]!
            vst1.32         {  d10[1] }, [r0]!

        .endm




        .align  2
        .global mulmat_3x3f_neon
        .thumb
        .thumb_func
mulmat_3x3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t mulmat_3x3f(arm_mat3x3f_t * dst,
        @                 arm_mat3x3f_t * src1,
        @                 arm_mat3x3f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: *src2 & current src2 entry's address
        @  r3: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r4:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              { r4 }
        vpush             { d8, d9, d10 }
        and               r4, r3, #3          @ r3 = count % 4;
        sub               r3, r3, r4          @ count = count - r3; This is what's left to be processed after this loop

        cmp               r3, #0
        beq               .L_check_mat3x3

        @ load the 1st set of values
          LOAD_3x3MATS
          subs            r3, r3, #4          @ 2 for this set, and 2 for the 2nd set

        @ calculate values for the 1st set
          MULTIPLY_3x3MATS

        @ load the 2nd set of values
          LOAD_3x3MATS
          ble             .L_mainloopend_mat3x3

.L_mainloop_mat3x3:
        @ store the result for the 1st/next (e.g. 3rd) set
          STORE_3x3MATS

        @ calculate values for the 2nd/next (e.g. 3rd) set
          MULTIPLY_3x3MATS

        @ load the next (e.g. 3rd) set of values
          LOAD_3x3MATS

          subs            r3, r3, #2

        bgt               .L_mainloop_mat3x3             @ loop if r2 is > r3, if we have at least another 4 vectors (12 floats) to process

.L_mainloopend_mat3x3:
        @ the last iteration for this call
        @ store the result for the set of values before the last one (e.g 2nd set)
          STORE_3x3MATS

        @ calculate values for the last (e.g. 3rd) set
          MULTIPLY_3x3MATS

        @ store the result for the last (e.g. 3rd) set
          STORE_3x3MATS

.L_check_mat3x3:
     @ check if anything left to process at the end of the input array
        cmp               r4, #0
        ble               .L_return_mat3x3

.L_secondloop_mat3x3:
     @ process the last few items left in the input array
       @ load the next (e.g. 3rd) set of values
            vld1.32         {  q0-q1 }, [r1]!
            vld1.32         {  d8[0] }, [r1]!
            vld1.32         {   q8-q9 }, [r2]!
            vld1.32         {   d9[0] }, [r2]!

             vtrn.32     q0,  q2
             vtrn.32     q1,  q3

             vtrn.32     q8, q10
             vtrn.32     q9, q11

          subs            r4, r4, #1

        @ calculate values for the last (e.g. 3rd) set
          MULTIPLY_3x3MATS

        @ store the result for the last (e.g. 3rd) set
             vtrn.32     q12,  q14
             vtrn.32     q13,  q15

            vst1.32         { q12-q13 }, [r0]!
            vst1.32         {  d10[0] }, [r0]!


        bgt               .L_secondloop_mat3x3

.L_return_mat3x3:
     @ return
        vpop              { d8, d9, d10 }
        pop               { r4 }
        mov               r0, #0
        bx                lr




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro to load a pair of 4x4 matrices from src1 (r1) and
        @ src2 (r2) into registers q0-q3 & q8-q11.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro LOAD_4x4MATS

            # load a 4x4 matrix from src1
            vld1.32         { q8-q9 }, [r1]!
            vld1.32         {q10-q11}, [r1]!

            # load a 4x4 matrix from src2
            vld1.32         {q0-q1}, [r2]!
            vld1.32         {q2-q3}, [r2]!
        .endm




       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       @ This macro multiplies the two 4x4 matrices loaded in the
       @ above LOAD_4x4MATS macro and returns the resulting 4x4
       @ matrix in q12-q15.
       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       .macro MULTIPLY_4x4MATS

           vmul.f32   q12,  q8, d0[0]
           vmul.f32   q13,  q8, d2[0]
           vmul.f32   q14,  q8, d4[0]
           vmul.f32   q15,  q8, d6[0]

           vmla.f32   q12,  q9, d0[1]
           vmla.f32   q13,  q9, d2[1]
           vmla.f32   q14,  q9, d4[1]
           vmla.f32   q15,  q9, d6[1]


           vmla.f32   q12, q10, d1[0]
           vmla.f32   q13, q10, d3[0]
           vmla.f32   q14, q10, d5[0]
           vmla.f32   q15, q10, d7[0]

           vmla.f32   q12, q11, d1[1]
           vmla.f32   q13, q11, d3[1]
           vmla.f32   q14, q11, d5[1]
           vmla.f32   q15, q11, d7[1]

       .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro stores the resulting 4x4 matrix which is
        @ returned by the above MULTIPLY_4x4MATS macro from registers
        @ q12-q15 into the dst (r0).
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro STORE_4x4MATS

            # store two 3x3 matrices to dst
            vst1.32         { q12-q13 }, [r0]!
            vst1.32         { q14-q15 }, [r0]!

        .endm




        .align  2
        .global mulmat_4x4f_neon
        .thumb
        .thumb_func
mulmat_4x4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t mulmat_4x4f(arm_mat4x4f_t * dst,
        @                 arm_mat4x4f_t * src1,
        @                 arm_mat4x4f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: *src2 & current src2 entry's address
        @  r3: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r4:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4;
        sub               r3, r3, r4          @ count = count - r4; This is what's left to be processed after this loop

        cmp               r3, #0
        beq               .L_check_mat4x4

        @ load the 1st set of values
          LOAD_4x4MATS

          subs            r3, r3, #2

        @ calculate values for the 1st set
          MULTIPLY_4x4MATS

        @ load the 2nd set of values
          LOAD_4x4MATS

          ble             .L_mainloopend_mat4x4

.L_mainloop_mat4x4:
        @ store the result for the 1st/next (e.g. 3rd) set
          STORE_4x4MATS

        @ calculate values for the 2nd/next (e.g. 3rd) set
          MULTIPLY_4x4MATS

       @ load the next (e.g. 3rd) set of values
          subs            r3, r3, #1
          LOAD_4x4MATS

        bgt               .L_mainloop_mat4x4             @ loop if r2 is > r3, if we have at least another 4 vectors (16 floats) to process

.L_mainloopend_mat4x4:
        @ the last iteration for this call
        @ store the result for the set of values before the last one (e.g 2nd set)
          STORE_4x4MATS

        @ calculate values for the last (e.g. 3rd) set
          MULTIPLY_4x4MATS

        @ store the result for the last (e.g. 3rd) set
          STORE_4x4MATS

.L_check_mat4x4:
     @ check if anything left to process at the end of the input array
        cmp               r4, #0
        ble               .L_return_mat4x4

.L_secondloop_mat4x4:
     @ process the last few items left in the input array
          LOAD_4x4MATS

          subs            r4, r4, #1

        @ calculate values
          MULTIPLY_4x4MATS

        @ store the results
          STORE_4x4MATS

        bgt               .L_secondloop_mat4x4

.L_return_mat4x4:
     @ return
        pop               {r4}
        mov               r0, #0
        bx                lr

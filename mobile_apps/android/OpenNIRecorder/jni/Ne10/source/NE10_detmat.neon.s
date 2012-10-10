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
@ NE10 Library : source/NE10_detmat.neon.s
@




        .text
        .syntax   unified

.include "headers/NE10header.s"
.include "source/NE10_detmat.neon.inc.s"



        .align   4
        .global   detmat_2x2f_neon
        .thumb
        .thumb_func

detmat_2x2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t detmat_2x2f(arm_float_t * dst,
        @                 arm_mat2x2f_t * src,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src & current src1 entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 matrices
        @
        @  r3:  the number of items that are left to be processed at the end
        @                   of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4;
        sub               r2, r2, r3          @ count = count - r2; This is what's left to be processed after this loop

        cbz               r2, .L_check_mat2x2

        @ We load four 2x2 matrices each time, calculate their
        @ determinants, store the results in the destination
        @ memory address, and move onto the next four.

        @ load the 1st set of values
        vld4.32         {d0, d2, d4, d6}, [r1]!
        vld4.32         {d1, d3, d5, d7}, [r1]!
        subs            r2, r2, #4

        @ calculate values for current set
        vmul.f32        q15, q0, q3
        vmls.f32        q15, q1, q2

        ble             .L_mainloopend_mat2x2

.L_mainloop_mat2x2:
        @ store the result for current set
        vst1.32         {q15}, [r0]!

        @ load the next set of values
        vld4.32         {d0, d2, d4, d6}, [r1]!
        vld4.32         {d1, d3, d5, d7}, [r1]!
        subs            r2, r2, #4

        @ calculate values for next set
        vmul.f32        q15, q0, q3
        vmls.f32        q15, q1, q2

        bgt             .L_mainloop_mat2x2             @ loop if r2 > 0, if we have at least another 4 vectors (8 floats) to process

.L_mainloopend_mat2x2:
        @ the last iteration for this call
        @ store the result for the last set
        vst1.32         {q15}, [r0]!

.L_check_mat2x2:
        @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_mat2x2

.L_secondloop_mat2x2:
        @ process the last few items left in the input array
        vld1.32         {d0, d1}, [r1]! @ Load matrix [A]

        subs            r3, r3, #1

        @ calculate det([A]) = |A|
        vrev64.32        d1, d1
        vmul.f32         d2, d0, d1
        vrev64.32        d2, d2
        vmls.f32         d2, d0, d1  @ At this point d2 = { -|A|, |A| }

        @ store the result which is in d2[1]
        vst1.32           {d2[1]}, [r0]!

        bgt               .L_secondloop_mat2x2

.L_return_mat2x2:
     @ return
        mov               r0, #0
        bx                lr




        .align  4
        .global detmat_3x3f_neon
        .thumb
        .thumb_func
detmat_3x3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t detmat_3x3f(arm_float_t * dst,
        @                 arm_mat3x3f_t * src,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 matrices
        @
        @  r3:  the number of items that are left to be processed at the end
        @                   of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r2 = count % 4;
        sub               r2, r2, r3          @ count = count - r2; This is what's left to be processed after this loop

        cmp               r2, #0
        beq               .L_check_mat3x3

        @ We load two 3x3 matrices each time, calculate their
        @ determinants, store the results in the destination 
        @ memory address, and move onto the next two.

        @ load the 1st set of values
        LOAD_3x3MATS_ARGS  d0, d1, d2, d3, d4, d5,  d16, d17, d18, d19, d20, d21,  q0, q1, q2, q8, q9, q10, r1
        subs            r2, r2, #2

        @ calculate values for the current set
        GET_DETERMINANT_of_3x3MATS_ARGS  d0, d2, d4, d16, d18, d20, d1, d3, d5, d22, d24, d26

        ble             .L_mainloopend_mat3x3

.L_mainloop_mat3x3:
        @ store the result for the current set
        vst1.32         {d22}, [r0]!

        @ load the next set of values
        LOAD_3x3MATS_ARGS  d0, d1, d2, d3, d4, d5,  d16, d17, d18, d19, d20, d21,  q0, q1, q2, q8, q9, q10, r1
        subs            r2, r2, #2

        @ calculate values for the next set
        GET_DETERMINANT_of_3x3MATS_ARGS  d0, d2, d4, d16, d18, d20, d1, d3, d5, d22, d24, d26

        bgt               .L_mainloop_mat3x3             @ loop if r2 > 0, if we have at least another 4 vectors (12 floats) to process

.L_mainloopend_mat3x3:
        @ the last iteration for this call
        @ store the result for the last set
          vst1.32         {d22}, [r0]!

.L_check_mat3x3:
     @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_mat3x3

.L_secondloop_mat3x3:
     @ process the last few items left in the input array

       @ load the next (e.g. 3rd) set of values
        vld3.32     { d0[0],  d2[0],  d4[0]}, [r1]!
        vld3.32     { d1[0],  d3[0],  d5[0]}, [r1]!
        vld3.32     {d16[0], d18[0], d20[0]}, [r1]!

          subs            r3, r3, #1

        @ calculate values for the last (e.g. 3rd) set
          GET_DETERMINANT_of_3x3MATS_ARGS  d0, d2, d4, d1, d3, d5, d16, d18, d20, d22, d24, d26

        @ store the result for the last (e.g. 3rd) set
         vst1.32         {d22[0]}, [r0]!

        bgt               .L_secondloop_mat3x3

.L_return_mat3x3:
     @ return

        mov               r0, #0
        bx                lr




        .align  4
        .global detmat_4x4f_neon
        .thumb
        .thumb_func
detmat_4x4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t detmat_float(arm_float_t * dst,
        @                 arm_mat4x4f_t * src1,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r3:  the number of items that are left to be processed at the end
        @                   of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4;
        sub               r2, r2, r3          @ count = count - r3; This is what's left to be processed after this loop

        cmp               r2, #0
        beq               .L_check_mat4x4


        @ We load two 4x4 matrices each time, calculate their
        @ determinants, store the results in the destination
        @ memory address, and move onto the next two.

        @ load the 1st set of values
         LOAD_4x4MATS_ARGS  d0, d1, d2, d3, d4, d5, d6, d7,  d16, d17, d18, d19, d20, d21, d22, d23,  q0, q1, q2, q3, q8, q9, q10, q11, r1
         subs            r2, r2, #2

        @ calculate values for the current set
         GET_DETERMINANT_of_4x4MATS_ARGS   d0, d2, d4, d6, d16, d18, d20, d22, d1, d3, d5, d7, d17, d19, d21, d23,  d24, d26, d28, d30, d25, d27

         ble             .L_mainloopend_mat4x4

.L_mainloop_mat4x4:
        @ store the result for the current set
         vst1.32         {d24}, [r0]!

       @ load the next set of values
         LOAD_4x4MATS_ARGS  d0, d1, d2, d3, d4, d5, d6, d7,  d16, d17, d18, d19, d20, d21, d22, d23,  q0, q1, q2, q3, q8, q9, q10, q11, r1
         subs            r2, r2, #2

        @ calculate values for the next set
         GET_DETERMINANT_of_4x4MATS_ARGS   d0, d2, d4, d6, d16, d18, d20, d22, d1, d3, d5, d7, d17, d19, d21, d23,  d24, d26, d28, d30, d25, d27

        bgt               .L_mainloop_mat4x4             @ loop if xx is > r2, if we have at least another 4 vectors (16 floats) to process

.L_mainloopend_mat4x4:
        @ the last iteration for this call
        @ store the result for the last set
         vst1.32         {d24}, [r0]!

.L_check_mat4x4:
     @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_mat4x4

.L_secondloop_mat4x4:
     @ process the last few items left in the input array
         vld4.32          {  d0[0],  d2[0],  d4[0],  d6[0]}, [r1]!
         vld4.32          {  d1[0],  d3[0],  d5[0],  d7[0]}, [r1]!
         vld4.32          { d16[0], d18[0], d20[0], d22[0]}, [r1]!
         vld4.32          { d17[0], d19[0], d21[0], d23[0]}, [r1]!



          subs            r3, r3, #1

        @ calculate values
         GET_DETERMINANT_of_4x4MATS_ARGS   d0, d2, d4, d6, d1, d3, d5, d7, d16, d18, d20, d22, d17, d19, d21, d23,  d24, d26, d28, d30, d25, d27

        @ store the results
         vst1.32         {d24[0]}, [r0]!

        bgt               .L_secondloop_mat4x4

.L_return_mat4x4:
     @ return
        mov               r0, #0
        bx                lr

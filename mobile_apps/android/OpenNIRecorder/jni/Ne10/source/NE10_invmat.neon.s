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
@ NE10 Library : source/NE10_invmat.neon.s
@




        .text
        .syntax   unified

.include "headers/NE10header.s"
.include "source/NE10_detmat.neon.inc.s"




CONST_FLOAT_ONE:
        .word           0x3f800000	@ This is the hex value for 1.0f in IEEE-754
        .word           0x3f800000
        .word           0x3f800000
        .word           0x3f800000

CONST_FLOAT_1Em12:
	.word		0x2B8CBCCC	@ This is the hex representation of 1.0e-12 in IEEE-754
        .word           0x2B8CBCCC      @  Any determinant smaller than this value is
        .word           0x2B8CBCCC      @  considered near zero and refused for
        .word           0x2B8CBCCC      @  calculating the inverse of a matrix.




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro calculates the inverse of four 2x2 matrices.
        @ It reads in the matrices from registers q8-q11 and returns
        @ its results in registers q12-q15
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro GET_INVERSE_2x2MATS
           @ get the determinant of these four matrices in q15
          vmul.f32        q15, q8, q11
          vmls.f32        q15, q9, q10

           @ compare them to find the ones that are too small and set those to 1.0f
            vacge.f32       q14, q15,  q0     @ dst = q14

          vand.f32        q13, q14, q15     @ tmp = q13
          vbic.s32        q14,  q1, q14     @ NOTE: This must be of type S32, the type F32 only negates the sign bits
          vorr.f32        q14, q14, q13     @ at this point q14 lanes that are too small are set to one and the rest are the determinants

           @ q15 = 1.0f / q14
          vrecpe.f32 q15, q14
          vrecps.f32 q14, q15, q14
          vmul.f32   q14, q14, q15


           @ now multiply all the entries with q14 = { 1/det(M1-M4) )
          vmul.f32   q12, q11, q14
          vmul.f32   q15,  q8, q14

          vneg.f32 q14, q14

          vmul.f32   q13,  q9, q14
          vmul.f32   q14, q10, q14

        .endm




        .align   4
        .global   invmat_2x2f_neon
        .thumb
        .thumb_func

invmat_2x2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t invmat_2x2f(arm_mat2x2f_t * dst,
        @                 arm_mat2x2f_t * src,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r3:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}

	adr               r4, CONST_FLOAT_1Em12
        vld1.32           {q0}, [r4]
        adr               r4, CONST_FLOAT_ONE
        vld1.32           {q1}, [r4]

        and               r3, r2, #3          @ r3 = count % 4;
        sub               r2, r2, r3          @ count = count - r2; This is what's left to be processed after this loop

        cmp               r2, #0
        beq               .L_check_mat2x2

        @ We load four 2x2 matrices each time, inverse them using the
        @ provided macro above, and store the four resulting matrices
        @ back into the memory location pointed to by the first parameter dst (r0)

        @ load the 1st set of values
          vld4.32         {d16, d18, d20, d22}, [r1]!
          vld4.32         {d17, d19, d21, d23}, [r1]!
          subs            r2, r2, #4          @ 4 for this set

        @ calculate values for the 1st set
          GET_INVERSE_2x2MATS

          ble             .L_mainloopend_mat2x2

.L_mainloop_mat2x2:
        @ store the result for the current set
          vst4.32         {d24, d26, d28, d30}, [r0]!
          vst4.32         {d25, d27, d29, d31}, [r0]!

        @ load the next set of values
          vld4.32         {d16, d18, d20, d22}, [r1]!
          vld4.32         {d17, d19, d21, d23}, [r1]!
          subs            r2, r2, #4

        @ calculate values for the next set
          GET_INVERSE_2x2MATS


        bgt             .L_mainloop_mat2x2             @ loop if r2 > 0, if we have at least another 4 vectors (8 floats) to process

.L_mainloopend_mat2x2:
        @ the last iteration for this call
        @ store the result for the last set
          vst4.32         {d24, d26, d28, d30}, [r0]!
          vst4.32         {d25, d27, d29, d31}, [r0]!

.L_check_mat2x2:
     @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_mat2x2

.L_secondloop_mat2x2:
     @ process the last few items left in the input array
        vld4.32         {d16[0], d18[0], d20[0], d22[0]}, [r1]!

        subs              r3, r3, #1

       @ calculate values
         GET_INVERSE_2x2MATS

      @ store the results
          vst4.32         {d24[0], d26[0], d28[0], d30[0]}, [r0]!

        bgt               .L_secondloop_mat2x2

.L_return_mat2x2:
     @ return
        pop               {r4}
        mov               r0, #0
        bx                lr




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro calculates the inverse of two 3x3 matrices.
        @ It reads in the matrices from registers q0-q5 and returns
        @ its results in registers q10-q15.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro GET_INVERSE_3x3MATS
           @ get the determinant of these two matrices in q15
          GET_DETERMINANT_of_3x3MATS_ARGS  d0, d2, d4, d6, d8, d10, d1, d3, d5,   d16, d9, d11    @ stores the results in d16

           @ compare them to find the ones that are too small and set those to 1.0f
            vacge.f32      d9, d16, d12     @ dst = d9 - the lanes that are too small are set to all (0)b

          vand.f32         d11,  d9,  d16     @ tmp = d11
          vbic.s32          d9, d14,  d9     @ NOTE: This must be of type S32, the type F32 only negates the sign bits
          vorr.f32          d9,  d9, d11     @ at this point d9 lanes that are too small are set to one and the rest are the determinants

           @ d16 = 1.0f / d9
          vrecpe.f32     d16,   d9
          vrecps.f32     d9,   d16,  d9
          vmul.f32       d16,   d9,  d16

          vmov.f32   d17,  d16              @ So q8 = { d16={1/det(M1), 1/det(M2)}, d17={1/det(M1), 1/det(M2)} }

           @ get the coefficients in q10 to q15
             GET_DET_2x2MATS_ARGS           d8, d10, d3,  d5,  d20
             GET_NEG_DET_2x2MATS_ARGS       d6, d10, d1,  d5,  d26
             GET_DET_2x2MATS_ARGS           d6,  d8, d1,  d3,  d21

             GET_NEG_DET_2x2MATS_ARGS       d2,  d4, d3,  d5,  d22
             GET_DET_2x2MATS_ARGS           d0,  d4, d1,  d5,  d28
             GET_NEG_DET_2x2MATS_ARGS       d0,  d2, d1,  d3,  d23

             GET_DET_2x2MATS_ARGS           d2,  d4, d8, d10,  d24
             GET_NEG_DET_2x2MATS_ARGS       d0,  d4, d6, d10,  d30
             GET_DET_2x2MATS_ARGS           d0,  d2, d6,  d8,  d25



           @ now multiply all the entries with q8 = { d16={1/det(M1), 1/det(M2)}, d17={1/det(M1), 1/det(M2)} }

          vmul.f32   q10,  q10, q8
          vmul.f32   q11,  q11, q8
          vmul.f32   q12,  q12, q8

          vmul.f32   q13,  q13, q8
          vmul.f32   q14,  q14, q8
          vmul.f32   q15,  q15, q8

        .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro stores two 3x3 matrices returned by the above macro
        @ GET_INVERSE_3x3MATS from registers q10-q15 and into the memory
        @ address pointed to by the register r0 (dst)
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro STORE_3x3INVMATS
            @ rearrange the results for use in a "vst3" instruction...
            vtrn.32     q10, q13
            vtrn.32     q11, q14
            vtrn.32     q12, q15

          vst3.32         { d20   , d22   , d24   }, [r0]!
          vst3.32         { d21[0], d23[0], d25[0]}, [r0]!
          vst3.32         { d26   , d28   , d30   }, [r0]!
          vst3.32         { d27[0], d29[0], d31[0]}, [r0]!
        .endm




        .align  4
        .global invmat_3x3f_neon
        .thumb
        .thumb_func
invmat_3x3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t invmat_3x3f(arm_mat3x3f_t * dst,
        @                 arm_mat3x3f_t * src1,
        @                 arm_mat3x3f_t * src2,
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

        push              {r4}
	vpush             {q4, q5, q6, q7}

        adr               r4, CONST_FLOAT_1Em12
        vld1.32           {q6}, [r4]
        adr               r4, CONST_FLOAT_ONE
        vld1.32           {q7}, [r4]

        and               r3, r2, #3          @ r2 = count % 4;
        sub               r2, r2, r3          @ count = count - r2; This is what's left to be processed after this loop

        cmp               r2, #0
        beq               .L_check_mat3x3

        @ We load two 3x3 matrices each time, inverse them using the
        @ provided macro above, and store the two resulting matrices
        @ back into the memory location pointed to by the first parameter dst (r0)

        @ load the 1st set of values
          LOAD_3x3MATS_ARGS  d0, d1, d2, d3, d4, d5,  d6, d7, d8, d9, d10, d11,  q0, q1, q2, q3, q4, q5, r1

          subs            r2, r2, #2          @ 2 for this set

        @ calculate values for the 1st set
          GET_INVERSE_3x3MATS


          ble             .L_mainloopend_mat3x3

.L_mainloop_mat3x3:
        @ store the result for the current set
          STORE_3x3INVMATS

        @ load the next set of values
          LOAD_3x3MATS_ARGS  d0, d1, d2, d3, d4, d5,  d6, d7, d8, d9, d10, d11,  q0, q1, q2, q3, q4, q5, r1
          subs            r2, r2, #2

        @ calculate values for the next set
          GET_INVERSE_3x3MATS

	bgt               .L_mainloop_mat3x3             @ loop if r2 > 0, if we have at least another 4 vectors (12 floats) to process

.L_mainloopend_mat3x3:
        @ the last iteration for this call
        @ store the result for the last set
          STORE_3x3INVMATS


.L_check_mat3x3:
     @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_mat3x3

.L_secondloop_mat3x3:
     @ process the last few items left in the input array
       @ load the next (e.g. 3rd) set of values
            vld3.32     {    d0,    d2,    d4 }, [r1]!
            vld3.32     { d1[0], d3[0], d5[0] }, [r1]!

             vtrn.32     q0, q3
             vtrn.32     q1, q4
             vtrn.32     q2, q5

          subs            r3, r3, #1

        @ calculate values for the last (e.g. 3rd) set
          GET_INVERSE_3x3MATS

        @ store the result for the last (e.g. 3rd) set
            vtrn.32     q10, q13
            vtrn.32     q11, q14
            vtrn.32     q12, q15

          vst3.32         { d20   , d22   , d24   }, [r0]!
          vst3.32         { d21[0], d23[0], d25[0]}, [r0]!

        bgt               .L_secondloop_mat3x3

.L_return_mat3x3:
     @ return
        vpop              {q4, q5, q6, q7}
        pop               {r4}
        mov               r0, #0
        bx                lr




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro calculates the inverse of two 4x4 matrices.
        @ It reads in the matrices from registers q0-q7 and returns
        @ its results in registers q8-q15.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro GET_INVERSE_4x4MATS
        vld1.32           {q10}, [r4]
        vld1.32           {q11}, [r5]

           @ get the determinant of these two matrices in q15
          GET_DETERMINANT_of_4x4MATS_ARGS   d0, d2, d4, d6, d8, d10, d12, d14, d1, d3, d5, d7, d9, d11, d13, d15,  d30, d28, d26, d31, d29, d27

           @ compare them to find the ones that are too small and set those to 1.0f
            vacge.f32      d24, d30, d20     @ dst = d24

          vand.f32         d25,  d24,  d30     @ tmp = d25
          vbic.s32         d24,  d22,  d24     @ NOTE: The instruction here must be of type S32, the type F32 only negates the sign bits
          vorr.f32         d24,  d24,  d25     @ at this point all d24 lanes that are too small are set to one and the rest are the determinants

           @ d30 = 1.0f (=q1) / d24
          vrecpe.f32 d30,  d24
          vrecps.f32 d24,  d30, d24
          vmul.f32   d30,  d24, d30

          vmov.f32   d31,  d30              @ So q15 = { d30={1/det(M1), 1/det(M2)}, d31={1/det(M1), 1/det(M2)} }


           @ get the coefficients
              GET_DETERMINANT_of_3x3MATS_ARGS        d0 , d4 , d6 , d8 , d12, d14, d1 , d5 , d7 ,   d18, d20, d22
              GET_DETERMINANT_of_3x3MATS_ARGS        d0 , d2 , d4 , d8 , d10, d12, d1 , d3 , d5 ,   d19, d20, d22

              GET_DETERMINANT_of_3x3MATS_ARGS        d10, d12, d14, d3 , d5 , d7 , d11, d13, d15,   d16, d20, d22
              GET_NEG_DET_3x3MATS_ARGS               d8 , d12, d14, d1 , d5 , d7 , d9 , d13, d15,   d24, d20, d22
              GET_DETERMINANT_of_3x3MATS_ARGS        d8 , d10, d14, d1 , d3 , d7 , d9 , d11, d15,   d17, d20, d22
              GET_NEG_DET_3x3MATS_ARGS               d8 , d10, d12, d1 , d3 , d5 , d9 , d11, d13,   d25, d20, d22

            vpush {d16, d17, d18, d19}

              GET_NEG_DET_3x3MATS_ARGS               d2 , d4 , d6 , d3 , d5 , d7 , d11, d13, d15,   d18, d16, d17
              GET_DETERMINANT_of_3x3MATS_ARGS        d0 , d4 , d6 , d1 , d5 , d7 , d9 , d13, d15,   d26, d16, d17
              GET_NEG_DET_3x3MATS_ARGS               d0 , d2 , d6 , d1 , d3 , d7 , d9 , d11, d15,   d19, d16, d17
              GET_DETERMINANT_of_3x3MATS_ARGS        d0 , d2 , d4 , d1 , d3 , d5 , d9 , d11, d13,   d27, d16, d17

              GET_DETERMINANT_of_3x3MATS_ARGS        d2 , d4 , d6 , d10, d12, d14, d11, d13, d15,   d20, d16, d17
              GET_NEG_DET_3x3MATS_ARGS               d0 , d4 , d6 , d8 , d12, d14, d9 , d13, d15,   d28, d16, d17
              GET_DETERMINANT_of_3x3MATS_ARGS        d0 , d2 , d6 , d8 , d10, d14, d9 , d11, d15,   d21, d16, d17
              GET_NEG_DET_3x3MATS_ARGS               d0 , d2 , d4 , d8 , d10, d12, d9 , d11, d13,   d29, d16, d17

              GET_NEG_DET_3x3MATS_ARGS               d2 , d4 , d6 , d10, d12, d14, d3 , d5 , d7 ,   d22, d16, d17
              @@  GET_DETERMINANT_of_3x3MATS_ARGS    d0 , d4 , d6 , d8 , d12, d14, d1 , d5 , d7 ,   d30, d16, d17     @ This is moved to the top of this section as q15 must remain unchanged
              GET_NEG_DET_3x3MATS_ARGS               d0 , d2 , d6 , d8 , d10, d14, d1 , d3 , d7 ,   d23, d16, d17
              @@  GET_DETERMINANT_of_3x3MATS_ARGS    d0 , d2 , d4 , d8 , d10, d12, d1 , d3 , d5 ,   d31, d16, d17     @ This is moved to the top of this section as q15 must remain unchanged

            vpop {d16, d17}

           @ now multiply all the entries with q15 = { d30={1/det(M1), 1/det(M2)}, d31={1/det(M1), 1/det(M2)} }

          vmul.f32   q11, q11, q15
          vmul.f32   q10, q10, q15
          vmul.f32    q9,  q9, q15
          vmul.f32    q8,  q8, q15

            vpop {d0, d1}

          vmul.f32   q12, q12, q15
          vmul.f32   q13, q13, q15
          vmul.f32   q14, q14, q15
          vmul.f32   q15,  q0, q15

        .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro stores two 4x4 matrices returned by the above macro
        @ GET_INVERSE_4x4MATS from registers q8-q15 and into the memory
        @ address pointed to by the register r0 (dst)
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro STORE_4x4INVMATS
            @ rearrange the results for use in a "vst4" instruction...
            vtrn.32      q8, q12
            vtrn.32      q9, q13
            vtrn.32     q10, q14
            vtrn.32     q11, q15

          vst4.32         { d16   , d18   , d20 ,  d22  }, [r0]!
          vst4.32         { d17   , d19   , d21 ,  d23  }, [r0]!
          vst4.32         { d24   , d26   , d28 ,  d30  }, [r0]!
          vst4.32         { d25   , d27   , d29 ,  d31  }, [r0]!
        .endm




        .align  4
        .global invmat_4x4f_neon
        .thumb
        .thumb_func
invmat_4x4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t invmat_4x4f(arm_mat4x4f_t * dst,
        @                 arm_mat4x4f_t * src1,
        @                 arm_mat4x4f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r3:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4, r5}
        vpush             {q4, q5, q6, q7}

        adr               r4, CONST_FLOAT_1Em12
        adr               r5, CONST_FLOAT_ONE

        and               r3, r2, #3          @ r3 = count % 4;
        sub               r2, r2, r3          @ count = count - r3; This is what's left to be processed after this loop

        cmp               r2, #0
        beq               .L_check_mat4x4

        @ We load two 4x4 matrices each time, inverse them using the
        @ provided macro above, and store the two resulting matrices
        @ back into the memory location pointed to by the first parameter dst (r0)

        @ load the 1st set of values
         LOAD_4x4MATS_ARGS  d0, d1, d2, d3, d4, d5, d6, d7,  d8, d9, d10, d11, d12, d13, d14, d15,  q0, q1, q2, q3, q4, q5, q6, q7, r1
          subs            r2, r2, #2          @ two for the first set

        @ calculate values for the 1st set
          GET_INVERSE_4x4MATS

          ble             .L_mainloopend_mat4x4

.L_mainloop_mat4x4:
        @ store the result for the 1st/next (e.g. 3rd) set
          STORE_4x4INVMATS

        @ load the next (e.g. 3rd) set of values
          LOAD_4x4MATS_ARGS  d0, d1, d2, d3, d4, d5, d6, d7,  d8, d9, d10, d11, d12, d13, d14, d15,  q0, q1, q2, q3, q4, q5, q6, q7, r1
          subs            r2, r2, #2

        @ calculate values for the 2nd/next (e.g. 3rd) set
          GET_INVERSE_4x4MATS


        bgt               .L_mainloop_mat4x4             @ loop if r2 > 0, if we have at least another 4 vectors (16 floats) to process

.L_mainloopend_mat4x4:
        @ the last iteration for this call
        @ store the result for the last set
          STORE_4x4INVMATS

.L_check_mat4x4:
     @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_mat4x4

.L_secondloop_mat4x4:
     @ process the last few items left in the input array
            vld4.32     { d0, d2, d4, d6 }, [r1]!
            vld4.32     { d1, d3, d5, d7 }, [r1]!

             vtrn.32     q0, q4
             vtrn.32     q1, q5
             vtrn.32     q2, q6
             vtrn.32     q3, q7

          subs            r3, r3, #1
        @ calculate values
          GET_INVERSE_4x4MATS

        @ store the results
            vtrn.32      q8, q12
            vtrn.32      q9, q13
            vtrn.32     q10, q14
            vtrn.32     q11, q15

          vst4.32         { d16   , d18   , d20 ,  d22  }, [r0]!
          vst4.32         { d17   , d19   , d21 ,  d23  }, [r0]!


        bgt               .L_secondloop_mat4x4

.L_return_mat4x4:
     @ return
        vpop              {q4, q5, q6, q7}
	pop               {r4, r5}
        mov               r0, #0
        bx                lr

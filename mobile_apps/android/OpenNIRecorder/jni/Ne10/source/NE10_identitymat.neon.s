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
@ NE10 Library : source/NE10_identitymat.neon.s
@




        .text
        .syntax   unified

.include "headers/NE10header.s"




        .balign   4
        .global   identitymat_2x2f_neon
        .thumb
        .thumb_func

identitymat_2x2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t identitymat_2x2f(arm_mat2x2f_t * dst,
        @                 arm_mat2x2f_t * src,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r2:  the number of items that are left to be processed at the end
        @                of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r2, r1, #3          @ r2 = count % 4;
        sub               r1, r1, r2          @ count = count - r1; This is what's left to be processed after this loop

        vmov.f32              d2, 0.0
        vmov.f32              d3, 0.0
        vmov.f32              d0, 1.0
        vmov.f32              d1, 1.0


        vmov              q3, q0
        vmov              q2, q1

        cmp               r1, #0
        beq               .L_check_mat2x2

.L_mainloop_mat2x2:

          subs            r1, r1, #4

          vst4.32         {d0, d2, d4, d6}, [r0]!
          vst4.32         {d1, d3, d5, d7}, [r0]!

        bgt             .L_mainloop_mat2x2             @ loop if r1 > 0, if we have at least another 4 vectors (8 floats) to process

.L_mainloopend_mat2x2:

.L_check_mat2x2:
     @ check if anything left to process at the end of the input array
        cmp               r2, #0
        ble               .L_return_mat2x2

.L_secondloop_mat2x2:
     @ process the last few items left in the input array
        vswp            d18, d20

        subs              r2, r2, #1

          vst4.32         {d0[0], d2[0], d4[0], d6[0]}, [r0]!

        bgt               .L_secondloop_mat2x2

.L_return_mat2x2:
     @ return
        mov               r0, #0
        bx                lr




        .align  2
        .global identitymat_3x3f_neon
        .thumb
        .thumb_func
identitymat_3x3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t identitymat_3x3f(arm_mat3x3f_t * dst,
        @                 arm_mat3x3f_t * src1,
        @                 arm_mat3x3f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r2:  the number of items that are left to be processed at the end
        @                   of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r2, r1, #3          @ r1 = count % 4;
        sub               r1, r1, r2          @ count = count - r1; This is what's left to be processed after this loop

        vmov.f32              d2, 0.0
        vmov.f32              d3, 0.0
        vmov.f32              d0, 1.0
        vmov.f32              d1, 1.0

        vmov              q8 , q1
        vmov              q9 , q1
        vmov              q10, q1
        vmov              q11, q1
        vmov              q12, q1
        vmov              q13, q1

        vtrn.32           d2, d0                @  d0 = {0.0f, 1.0f}
        vtrn.32           d1, d3                @  d1 = {1.0f, 0.0f}

        vmov              d16, d1
        vmov              d18, d0
        vmov              d21, d1
        vmov              d22, d1
        vmov              d24, d0
        vmov              d27, d1

        cmp               r1, #0
        beq               .L_check_mat3x3

.L_mainloop_mat3x3:

          subs            r1, r1, #2

          vst3.32         { d16   , d18   , d20   }, [r0]!
          vst3.32         { d17[0], d19[0], d21[0]}, [r0]!
          vst3.32         { d22   , d24   , d26   }, [r0]!
          vst3.32         { d23[0], d25[0], d27[0]}, [r0]!

	bgt               .L_mainloop_mat3x3             @ loop if r1 > 0, if we have at least another 4 vectors (12 floats) to process

.L_mainloopend_mat3x3:

.L_check_mat3x3:
     @ check if anything left to process at the end of the input array
        cmp               r2, #0
        ble               .L_return_mat3x3

.L_secondloop_mat3x3:
     @ process the last few items left in the input array

          subs            r2, r2, #1

          vst3.32         { d16   , d18   , d20   }, [r0]!
          vst3.32         { d17[0], d19[0], d21[0]}, [r0]!

        bgt               .L_secondloop_mat3x3

.L_return_mat3x3:
     @ return
        mov               r0, #0
        bx                lr




        .align  2
        .global identitymat_4x4f_neon
        .thumb
        .thumb_func
identitymat_4x4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t identitymat_4x4f(arm_mat4x4f_t * dst,
        @                 arm_mat4x4f_t * src1,
        @                 arm_mat4x4f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r2:  the number of items that are left to be processed at the end
        @                of the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r2, r1, #3          @ r2 = count % 4;
        sub               r1, r1, r2          @ count = count - r2; This is what's left to be processed after this loop

        vmov.f32              d2, 0.0
        vmov.f32              d3, 0.0
        vmov.f32              d0, 1.0
        vmov.f32              d1, 1.0

        vmov              q8 , q1
        vmov              q9 , q1
        vmov              q10, q1
        vmov              q11, q1
        vmov              q12, q1
        vmov              q13, q1
        vmov              q14, q1
        vmov              q15, q1

        vtrn.32           d2, d0                @  d0 = {0.0f, 1.0f}
        vtrn.32           d1, d3                @  d1 = {1.0f, 0.0f}

        vmov              d16, d1
        vmov              d18, d0
        vmov              d21, d1
        vmov              d23, d0

        vmov              d24, d1
        vmov              d26, d0
        vmov              d29, d1
        vmov              d31, d0

        cmp               r1, #0
        beq               .L_check_mat4x4

.L_mainloop_mat4x4:

          subs            r1, r1, #2

          vst4.32         { d16   , d18   , d20 ,  d22  }, [r0]!
          vst4.32         { d17   , d19   , d21 ,  d23  }, [r0]!
          vst4.32         { d24   , d26   , d28 ,  d30  }, [r0]!
          vst4.32         { d25   , d27   , d29 ,  d31  }, [r0]!

        bgt               .L_mainloop_mat4x4             @ loop if r1 > 0, if we have at least another 4 vectors (16 floats) to process

.L_mainloopend_mat4x4:

.L_check_mat4x4:
     @ check if anything left to process at the end of the input array
        cmp               r2, #0
        ble               .L_return_mat4x4

.L_secondloop_mat4x4:
     @ process the last few items left in the input array

          subs            r2, r2, #1

          vst4.32         { d16   , d18   , d20 ,  d22  }, [r0]!
          vst4.32         { d17   , d19   , d21 ,  d23  }, [r0]!


        bgt               .L_secondloop_mat4x4

.L_return_mat4x4:
     @ return
        mov               r0, #0
        bx                lr


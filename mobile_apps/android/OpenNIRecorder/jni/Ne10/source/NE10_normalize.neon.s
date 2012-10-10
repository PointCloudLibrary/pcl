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
@ NE10 Library : source/NE10_normalize.neon.s
@

        .text
        .syntax   unified

.include "headers/NE10header.s"




        .balign   4
        .global   normalize_vec2f_neon
        .thumb
        .thumb_func

normalize_vec2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t normalize_vec2f(arm_vec2f_t * dst,
        @                 arm_vec2f_t * src,
        @                 unsigned int count);
        @
        @  r0: *dst & the current dst entry's address
        @  r1: *src & current src entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @  r3: the number of items that are left to be processed at the end of
        @                   the input array
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4;
        sub               r2, r2, r3          @ count = count - r3; This is what's left to be processed after this loop

        cbz               r2, .L_check_vec2

        @ load values for the first iteration
          vld2.32         {q0-q1}, [r1]!
          subs            r2, r2, #4

        @ calculate sum of square of the components
          vmul.f32        q2, q0, q0
          vmla.f32        q2, q1, q1

          ble             .L_mainloopend_vec2

.L_mainloop_vec2:
       @ load the next set of values
        vmov.f32          q10, q0
        vmov.f32          q11, q1
        vld2.32           {q0-q1}, [r1]!
        subs              r2, r2, #4

        @ get reciprocal SQRT of the last vector while loading a new vector
          vrsqrte.f32     q3, q2
          vmul.f32        q4, q2, q3
          vrsqrts.f32     q4, q4, q3
          vmul.f32        q4, q3, q4

        @ normalize the components
          vmul.f32        q3, q10, q4         @ q3 = q0(8) * q4
          vmul.f32        q4, q11, q4         @ q4 = q1(9) * q4

          vst2.32         {d6,d7,d8,d9}, [r0]!

        @ calculate sum of square of the components
        vmul.f32        q2, q0, q0
        vmla.f32        q2, q1, q1

        bgt             .L_mainloop_vec2             @ loop if r2 is > r3, if we have at least another 4 vectors (8 floats) to process

.L_mainloopend_vec2:
        @ the last iteration for this call
        @ get reciprocal SQRT of the last vector
          vrsqrte.f32     q3, q2
          vmul.f32        q4, q2, q3
          vrsqrts.f32     q4, q4, q3
          vmul.f32        q4, q3, q4

        @ normalize the components
          vmul.f32        q3, q0, q4          @ q3 = q0 * q4
          vmul.f32        q4, q1, q4          @ q4 = q1 * q4

        vst2.32         {d6,d7,d8,d9}, [r0]!

.L_check_vec2:
     @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_vec2

.L_secondloop_vec2:
     @ process the last few items left in the input array
        vld1.f32          d0, [r1]!           @ Fill in d0 = { V.x, V.y };

        subs              r3, r3, #1

        @ calculate sum of square of the components
        vmul.f32          d1, d0, d0          @  d1= { V.x^2, V.y^2 };
        vpadd.f32         d3, d1, d1          @  d3= { V.x^2 + (V.y^2), V.y^2 + (V.x^2) };


        @ get reciprocal SQRT of the last vector
        vrsqrte.f32       d2, d3
        vmul.f32          d1, d3, d2
        vrsqrts.f32       d1, d1, d2
        vmul.f32          d1, d2, d1

        @ normalize the components
        vmul.f32          d0, d0, d1

        vst1.32           {d0}, [r0]!

        bgt               .L_secondloop_vec2

.L_return_vec2:
     @ return
        mov               r0, #0
        bx                lr




        .align  2
        .global normalize_vec3f_neon
        .thumb
        .thumb_func
normalize_vec3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t normalize_vec3f(arm_vec3t_t * dst,
        @                 arm_vec3f_t * src,
        @                 unsigned int count);
        @
        @  r0: *dst & the current dst entry's address
        @  r1: *src & current src entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @  r3: the number of items that are left to be processed at the end of
        @                   the input array
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4;
        sub               r2, r2, r3          @ count = count - r3; This is what's left to be processed after this loop

        cmp               r2, #0
        beq               .L_check_vec3

        @ load values for the first iteration
          vld3.32         {d0, d2, d4}, [r1]!
          vld3.32         {d1, d3, d5}, [r1]!
          subs            r2, r2, #4

        @ calculate sum of square of the components
          vmul.f32        q3, q0, q0
          vmla.f32        q3, q1, q1
          vmla.f32        q3, q2, q2


          ble             .L_mainloopend_vec3

.L_mainloop_vec3:
       @ load the next set of values
        vmov.f32          q10, q0
        vmov.f32          q11, q1
        vmov.f32          q12, q2

        vld3.32           {d0,d2,d4}, [r1]!
        vld3.32           {d1,d3,d5}, [r1]!
        subs              r2, r2, #4

        @ get reciprocal SQRT of the last vector while loading a new vector
          vrsqrte.f32     q5, q3
          vmul.f32        q4, q3, q5
          vrsqrts.f32     q4, q4, q5
          vmul.f32        q4, q5, q4

        @ normalize the components
          vmul.f32        q5, q10, q4
          vmul.f32        q6, q11, q4
          vmul.f32        q7, q12, q4

          vst3.32         {d10, d12, d14}, [r0]!
          vst3.32         {d11, d13, d15}, [r0]!

        @ calculate sum of square of the components
        vmul.f32          q3, q0, q0
        vmla.f32          q3, q1, q1
        vmla.f32          q3, q2, q2

        bgt               .L_mainloop_vec3             @ loop if r2 is > r3, if we have at least another 4 vectors (12 floats) to process

.L_mainloopend_vec3:
        @ the last iteration for this call
        @ get reciprocal SQRT of the last vector
          vrsqrte.f32     q5, q3
          vmul.f32        q4, q3, q5
          vrsqrts.f32     q4, q4, q5
          vmul.f32        q4, q5, q4

        @ normalize the components
          vmul.f32        q5, q0, q4
          vmul.f32        q6, q1, q4
          vmul.f32        q7, q2, q4

          vst3.32         {d10, d12, d14}, [r0]!
          vst3.32         {d11, d13, d15}, [r0]!

.L_check_vec3:
     @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_vec3

.L_secondloop_vec3:
     @ process the last few items left in the input array
        vld3.f32          {d0[0], d2[0], d4[0]}, [r1]!     @ The values are loaded like so:
                                                           @      q0 = { V.x, -, -, - };
                                                           @      q1 = { V.y, -, -, - };
                                                           @      q2 = { V.z, -, -, - };
        subs              r3, r3, #1

        @ calculate sum of square of the components
        vmul.f32          q3, q0, q0          @  V.x^2
        vmla.f32          q3, q1, q1          @  V.x^2 + V.y^2
        vmla.f32          q3, q2, q2          @  V.x^2 + V.y^2 + V.z^2


        @ get reciprocal SQRT of the last vector
        vrsqrte.f32     q5, q3
        vmul.f32        q4, q3, q5
        vrsqrts.f32     q4, q4, q5
        vmul.f32        q4, q5, q4

        @ normalize the components
        vmul.f32          q0, q0, q4
        vmul.f32          q1, q1, q4
        vmul.f32          q2, q2, q4

        vst3.32           {d0[0], d2[0], d4[0]}, [r0]!

        bgt               .L_secondloop_vec3

.L_return_vec3:
     @ return
        mov               r0, #0
        bx                lr




        .align  2
        .global normalize_vec4f_neon
        .thumb
        .thumb_func
normalize_vec4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t normalize_vec4f(arm_vec4f_t * dst,
        @                 arm_vec4f_t * src,
        @                 unsigned int count);
        @
        @  r0: *dst & the current dst entry's address
        @  r1: *src & current src entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @  r3: the number of items that are left to be processed at the end of
        @                   the input array
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4;
        sub               r2, r2, r3          @ count = count - r3; This is what's left to be processed after this loop

        cmp               r2, #0
        beq               .L_check_vec4

        @ load values for the first iteration
          vld4.32         {d0, d2, d4, d6}, [r1]!
          vld4.32         {d1, d3, d5, d7}, [r1]!
          subs            r2, r2, #4

        @ calculate sum of square of the components
          vmul.f32        q5, q0, q0
          vmla.f32        q5, q1, q1
          vmla.f32        q5, q2, q2
          vmla.f32        q5, q3, q3

          ble             .L_mainloopend_vec4

.L_mainloop_vec4:
       @ load the next set of values
        vmov              q10, q0
        vmov              q11, q1
        vmov              q12, q2
        vmov              q13, q3

        vld4.32           {d0, d2, d4, d6}, [r1]!
        vld4.32           {d1, d3, d5, d7}, [r1]!
        subs              r2, r2, #4

        @ get reciprocal SQRT of the last vector while loading a new vector
          vrsqrte.f32     q6, q5
          vmul.f32        q4, q5, q6
          vrsqrts.f32     q4, q4, q6
          vmul.f32        q4, q6, q4

        @ normalize the components
          vmul.f32        q10, q10, q4
          vmul.f32        q11, q11, q4
          vmul.f32        q12, q12, q4
          vmul.f32        q13, q13, q4

          vst4.32         {d20, d22, d24, d26}, [r0]!
          vst4.32         {d21, d23, d25, d27}, [r0]!

        @ calculate sum of square of the components
        vmul.f32        q5, q0, q0
        vmla.f32        q5, q1, q1
        vmla.f32        q5, q2, q2
        vmla.f32        q5, q3, q3

        bgt               .L_mainloop_vec4             @ loop if r2 is > r3, if we have at least another 4 vectors (12 floats) to process

.L_mainloopend_vec4:
        @ the last iteration for this call
        @ get reciprocal SQRT of the last vector
          vrsqrte.f32     q6, q5
          vmul.f32        q4, q5, q6
          vrsqrts.f32     q4, q4, q6
          vmul.f32        q4, q6, q4

        @ normalize the components
          vmul.f32        q0, q0, q4
          vmul.f32        q1, q1, q4
          vmul.f32        q2, q2, q4
          vmul.f32        q3, q3, q4

          vst4.32         {d0, d2, d4, d6}, [r0]!
          vst4.32         {d1, d3, d5, d7}, [r0]!

.L_check_vec4:
     @ check if anything left to process at the end of the input array
        cmp               r3, #0
        ble               .L_return_vec4

.L_secondloop_vec4:
     @ process the last few items left in the input array
        vld4.f32          {d0[0], d2[0], d4[0], d6[0]}, [r1]!     @ The values are loaded like so:
                                                                  @      q0 = { V.x, -, -, - };
                                                                  @      q1 = { V.y, -, -, - };
                                                                  @      q2 = { V.z, -, -, - };
        subs              r3, r3, #1

        @ calculate sum of square of the components
        vmul.f32          q4, q0, q0          @  V.x^2
        vmla.f32          q4, q1, q1          @  V.x^2 + V.y^2
        vmla.f32          q4, q2, q2          @  V.x^2 + V.y^2 + V.z^2
        vmla.f32          q4, q3, q3          @  V.x^2 + V.y^2 + V.z^2 + V.w^2

        @ get reciprocal SQRT of the last vector
        vrsqrte.f32       q5, q4
        vmul.f32          q6, q4, q5
        vrsqrts.f32       q6, q6, q5
        vmul.f32          q6, q5, q6

        @ normalize the components
        vmul.f32          q0, q0, q6
        vmul.f32          q1, q1, q6
        vmul.f32          q2, q2, q6
        vmul.f32          q3, q3, q6

        vst4.32          {d0[0], d2[0], d4[0], d6[0]}, [r0]!     @ The values are loaded like so:

        bgt               .L_secondloop_vec4

.L_return_vec4:
     @ return
        mov               r0, #0
        bx                lr

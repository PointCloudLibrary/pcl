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
@ NE10 Library : source/NE10_add.neon.s
@

        .text
        .syntax   unified

.include "headers/NE10header.s"




        .align   4
        .global   add_float_neon
        .thumb
        .thumb_func

add_float_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t add_float(arm_float_t * dst,
        @                 arm_float_t * src1,
        @                 arm_float_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: *src2 & current src2 entry's address
        @  r3: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r4: the number of items that are residual that will be processed at the begin of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4; calculate the residual loop
        asr               r3, r3, #2          @ r3 = count >> 2; calculate the main loop

        cbz               r4, .L_check_mainloop_float

.L_residualloop_float:
        @ process the residual items in the input array
        vld1.f32          d0[0], [r1]!           @ Fill in d0[0]
        vld1.f32          d1[0], [r2]!           @ Fill in d1[1]

        subs              r4, r4, #1

        @ values
        vadd.f32          d0, d0, d1

        vst1.32           {d0[0]}, [r0]!

        bgt               .L_residualloop_float

.L_check_mainloop_float:
        cbz               r3, .L_return_float


        @ load the current set of values
        vld1.32         {q0}, [r1]!
        vld1.32         {q1}, [r2]!        @ for current set

.L_mainloop_float:
        @ calculate values for current set
        vadd.f32        q3, q0, q1         @ q3 = q0 + q1

        @ store the result for current set
        vst1.32         {d6,d7}, [r0]!

        subs            r3, r3, #1

        @ load the next set of values
        vld1.32         {q0}, [r1]!
        vld1.32         {q1}, [r2]!

        bgt             .L_mainloop_float             @ loop if r3 > 0, if we have at least another 4 floats

.L_return_float:
     @ return
        pop               {r4}
        mov               r0, #0
        bx                lr




        .align   4
        .global   add_vec2f_neon
        .thumb
        .thumb_func

add_vec2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t add_float(arm_vec2f_t * dst,
        @                 arm_vec2f_t * src1,
        @                 arm_vec2f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: *src2 & current src2 entry's address
        @  r3: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r4: the number of items that are residual that will be processed at the begin of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4; calculate the residual loop
        asr               r3, r3, #2          @ r3 = count >> 2; calculate the main loop

        cbz               r4, .L_check_mainloop_vec2

.L_residualloop_vec2:
        @  process the residual items in the input array
        vld1.f32          d0, [r1]!
        vld1.f32          d1, [r2]!

        subs              r4, r4, #1

        @ calculate values
        vadd.f32          d0, d0, d1

        vst1.32           {d0}, [r0]!
        bgt               .L_residualloop_vec2

.L_check_mainloop_vec2:
        cbz               r3, .L_return_vec2

        @ load the current set of values
        vld2.32         {q0-q1}, [r1]!
        vld2.32         {q2-q3}, [r2]!

.L_mainloop_vec2:
        @ calculate values for current set
        vadd.f32        q8, q0, q2
        vadd.f32        q9, q1, q3

        @ store the result for current set
        vst2.32         {d16,d17,d18,d19}, [r0]!
        subs            r3, r3, #1

        @ load the next set of values
        vld2.32         {q0-q1}, [r1]!
        vld2.32         {q2-q3}, [r2]!

        bgt             .L_mainloop_vec2             @ loop if r3 > 0, if we have at least another 4 vectors (8 floats) to process

.L_return_vec2:
     @ return
        pop               {r4}
        mov               r0, #0
        bx                lr




        .align  4
        .global add_vec3f_neon
        .thumb
        .thumb_func
add_vec3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t add_float(arm_vec3f_t * dst,
        @                 arm_vec3f_t * src1,
        @                 arm_vec3f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: *src2 & current src2 entry's address
        @  r3: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r4:  the number of items that are residual that will be processed at the begin of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4; calculate the residual loop
        asr               r3, r3, #2          @ r3 = count >> 2; calculate the main loop

        cbz               r4, .L_check_mainloop_vec3

.L_residualloop_vec3:
        @  process the residual items in the input array
        vld3.f32          {d0[0], d2[0], d4[0]}, [r1]!     @ The values are loaded like so:
                                                           @      q0 = { V1.x, -, -, - };
                                                           @      q1 = { V1.y, -, -, - };
                                                           @      q2 = { V1.z, -, -, - };
        vld3.f32          {d1[0], d3[0], d5[0]}, [r2]!     @ The values are loaded like so:
                                                           @      q0 = { V1.x, -, V2.x, - };
                                                           @      q1 = { V1.y, -, V2.y, - };
                                                           @      q2 = { V1.z, -, V2.z, - };

        subs              r4, r4, #1

        @ calculate values for
        vadd.f32          d0, d0, d1
        vadd.f32          d2, d2, d3
        vadd.f32          d4, d4, d5

        vst3.32           {d0[0], d2[0], d4[0]}, [r0]!

        bgt               .L_residualloop_vec3

.L_check_mainloop_vec3:
        cbz               r3, .L_return_vec3

        @ load current set of values
        vld3.32         {d0, d2, d4}, [r1]!
        vld3.32         {d1, d3, d5}, [r1]!
        vld3.32         {d18, d20, d22}, [r2]!
        vld3.32         {d19, d21, d23}, [r2]!

.L_mainloop_vec3:
        @ calculate values for current set
        vadd.f32        q12, q0, q9
        vadd.f32        q13, q1, q10
        vadd.f32        q14, q2, q11

        @ store the result for current set
        vst3.32         {d24, d26, d28}, [r0]!
        vst3.32         {d25, d27, d29}, [r0]!
        subs            r3, r3, #1

        @ load the next set of values
        vld3.32         {d0, d2, d4}, [r1]!
        vld3.32         {d1, d3, d5}, [r1]!
        vld3.32         {d18, d20, d22}, [r2]!
        vld3.32         {d19, d21, d23}, [r2]!

        bgt               .L_mainloop_vec3             @ loop if r3 > 0, if we have at least another 4 vectors (12 floats) to process

.L_return_vec3:
     @ return
        pop               {r4}
        mov               r0, #0
        bx                lr




        .align  4
        .global add_vec4f_neon
        .thumb
        .thumb_func
add_vec4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t add_float(arm_vec4f_t * dst,
        @                 arm_vec4f_t * src1,
        @                 arm_vec4f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *src1 & current src1 entry's address
        @  r2: *src2 & current src2 entry's address
        @  r3: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r4:  the number of items that are residual that will be processed at the begin of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4; calculate the residual loop
        asr               r3, r3, #2          @ r3 = count >> 2; calculate the main loop

        cbz               r4, .L_check_mainloop_vec4

.L_residualloop_vec4:
        @ process the last few items left in the input array
        vld1.f32          {d0, d1}, [r1]!     @ The values are loaded like so:
                                                                  @      q0 = { V1.x, V1.y, V1.z, V1.w };
        vld1.f32          {d2, d3}, [r2]!     @ The values are loaded like so:
                                                                  @      q1 = { V2.x, V2.y, V2.z, V2.w };

        subs              r4, r4, #1

        @ calculate values
        vadd.f32          q0, q0, q1

        vst1.32          {d0, d1}, [r0]!

        bgt               .L_residualloop_vec4

.L_check_mainloop_vec4:
        cbz               r3, .L_return_vec4

        @ load the current set of values
        vld4.32         {d0, d2, d4, d6}, [r1]!
        vld4.32         {d1, d3, d5, d7}, [r1]!
        vld4.32         {d16, d18, d20, d22}, [r2]!
        vld4.32         {d17, d19, d21, d23}, [r2]!

.L_mainloop_vec4:
        @ calculate values for the current set
        vadd.f32        q12, q0, q8
        vadd.f32        q13, q1, q9
        vadd.f32        q14, q2, q10
        vadd.f32        q15, q3, q11

        @ store the result for the current set
        vst4.32         {d24, d26, d28, d30}, [r0]!
        vst4.32         {d25, d27, d29, d31}, [r0]!
        subs            r3, r3, #1

        @ load the next set of values
        vld4.32         {d0, d2, d4, d6}, [r1]!
        vld4.32         {d1, d3, d5, d7}, [r1]!
        vld4.32         {d16, d18, d20, d22}, [r2]!
        vld4.32         {d17, d19, d21, d23}, [r2]!

        bgt               .L_mainloop_vec4             @ loop if r3 > 0, if we have at least another 4 vectors (16 floats) to process

.L_return_vec4:
     @ return
        pop               {r4}
        mov               r0, #0
        bx                lr

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
@ NE10 Library : source/NE10_abs.neon.s
@

        .text
        .syntax   unified

.include "headers/NE10header.s"




        .align   4
        .global   abs_float_neon
        .thumb
        .thumb_func

abs_float_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t abs_float(arm_float_t * dst,
        @                 arm_float_t * src,
        @                 unsigned int count);
        @
        @  r0: *dst & the current dst entry's address
        @  r1: *src & current src entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @  r3: the number of items that are residual that will be processed at the begin of
        @                   the input array
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4; calculate the residual loop
        asr               r2, r2, #2          @ r2 = count >> 2; calculate the main loop

        cbz               r3, .L_check_mainloop_float

.L_residualloop_float:
        @ process the residual items in the input array
        vld1.f32          d0[0], [r1]!           @ Fill in d0 = { V.x, 0 };

        subs              r3, r3, #1

        @ absolute values
        vabs.f32          d0, d0

        vst1.32           {d0[0]}, [r0]!

        bgt               .L_residualloop_float

.L_check_mainloop_float:
        cbz               r2, .L_return_float

        @ load the current set of values
        vld1.32         {q0}, [r1]!         @ for current set

.L_mainloop_float:
        @ absolute values of the current set
        vabs.f32        q3, q0         @ q3 = abs( q0 )

        @ store the result for the current set
        vst1.32         {d6,d7}, [r0]!

        subs              r2, r2, #1

        @ load the next set
        vld1.32           {q0}, [r1]!

        bgt             .L_mainloop_float             @ loop if r2 > 0, if we have another 4 floats

.L_return_float:
        @ return
        mov               r0, #0
        bx                lr




        .align   4
        .global   abs_vec2f_neon
        .thumb
        .thumb_func

abs_vec2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t abs_vec2f(arm_vec2f_t * dst,
        @                 arm_vec2f_t * src,
        @                 unsigned int count);
        @
        @  r0: *dst & the current dst entry's address
        @  r1: *src & current src entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @  r3: the number of items that are residual that will be processed at the begin of
        @                   the input array
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4; calculate the residual loop
        asr               r2, r2, #2          @ r2 = count >> 2; calculate the main loop

        cbz               r3, .L_check_mainloop_vec2

.L_residualloop_vec2:
        @ process the residual items in the input array
        vld1.f32          d0, [r1]!           @ Fill in d0 = { V.x, V.y };

        subs              r3, r3, #1

        @ absolute values
        vabs.f32          d0, d0

        vst1.32           {d0}, [r0]!

        bgt               .L_residualloop_vec2

.L_check_mainloop_vec2:
        cbz               r2, .L_return_vec2

        @ load the current set of values
        vld2.32         {q0-q1}, [r1]!        @ for current set

.L_mainloop_vec2:
        @ absolute values of the current set
        vabs.f32        q3, q0         @ q3 = abs( q0 )
        vabs.f32        q4, q1         @ q4 = abs( q1 )

        @ store the result for the current set
        vst2.32         {d6,d7,d8,d9}, [r0]!

        subs              r2, r2, #1

        @ load the next set
        vld2.32         {q0-q1}, [r1]!

        bgt             .L_mainloop_vec2             @ loop if r2 > 0, if we have another 4 vec2s

.L_return_vec2:
        @ return
        mov               r0, #0
        bx                lr


        .align  4
        .global abs_vec3f_neon
        .thumb
        .thumb_func
abs_vec3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t abs_vec3f(arm_vec3t_t * dst,
        @                 arm_vec3f_t * src,
        @                 unsigned int count);
        @
        @  r0: *dst & the current dst entry's address
        @  r1: *src & current src entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @  r3: the number of items that are residual that will be processed at the begin of
        @                   the input array
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4; calculate the residual loop
        asr               r2, r2, #2          @ r2 = count >> 2; calculate the main loop

        cbz               r3, .L_check_mainloop_vec3

.L_residualloop_vec3:
        @ process the residual items in the input array
        vld3.f32          {d0[0], d2[0], d4[0]}, [r1]!     @ The values are loaded like so:
                                                           @      q0 = { V.x, -, -, - };
                                                           @      q1 = { V.y, -, -, - };
                                                           @      q2 = { V.z, -, -, - };
        subs              r3, r3, #1

        @ absolute values
        vabs.f32          d0, d0
        vabs.f32          d2, d2
        vabs.f32          d4, d4

        vst3.32           {d0[0], d2[0], d4[0]}, [r0]!

        bgt               .L_residualloop_vec3

.L_check_mainloop_vec3:
        cbz               r2, .L_return_vec3

        @ load the current set of values
        vld3.32         {d0, d2, d4}, [r1]!
        vld3.32         {d1, d3, d5}, [r1]!        @ for current set

.L_mainloop_vec3:
        @ absolute values of the current set
        vabs.f32        q5, q0
        vabs.f32        q6, q1
        vabs.f32        q7, q2

        @ store the result for the current set
        vst3.32         {d10, d12, d14}, [r0]!
        vst3.32         {d11, d13, d15}, [r0]!

        subs              r2, r2, #1

        @ load the next set
        vld3.32         {d0, d2, d4}, [r1]!
        vld3.32         {d1, d3, d5}, [r1]!        @ for next set

        bgt             .L_mainloop_vec3             @ loop if r2 > 0, if we have another 4 vec3s

.L_return_vec3:
        @ return
        mov               r0, #0
        bx                lr




        .align  4
        .global abs_vec4f_neon
        .thumb
        .thumb_func
abs_vec4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t abs_vec4f(arm_vec4f_t * dst,
        @                 arm_vec4f_t * src,
        @                 unsigned int count);
        @
        @  r0: *dst & the current dst entry's address
        @  r1: *src & current src entry's address
        @  r2: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @  r3: the number of items that are residual that will be processed at the begin of
        @                   the input array
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        and               r3, r2, #3          @ r3 = count % 4; calculate the residual loop
        asr               r2, r2, #2          @ r2 = count >> 2; calculate the main loop

        cbz               r3, .L_check_mainloop_vec4

.L_residualloop_vec4:
        @ process the residual items in the input array
        vld1.f32          {d0, d1}, [r1]!     @ The values are loaded like so:
                                              @      q0 = { V.x, V.y, V.z, V.w };
        subs              r3, r3, #1

        @ absolute values
        vabs.f32          q0, q0

        vst1.32          {d0, d1}, [r0]!

        bgt               .L_residualloop_vec4

.L_check_mainloop_vec4:
        cbz               r2, .L_return_vec4

        @ load the current set of values
        vld4.32         {d0, d2, d4, d6}, [r1]!
        vld4.32         {d1, d3, d5, d7}, [r1]!     @ for current set

.L_mainloop_vec4:
        @ absolute values of the current set
        vabs.f32        q10, q0
        vabs.f32        q11, q1
        vabs.f32        q12, q2
        vabs.f32        q13, q3

        @ store the result for the current set
        vst4.32         {d20, d22, d24, d26}, [r0]!
        vst4.32         {d21, d23, d25, d27}, [r0]!

        subs              r2, r2, #1

        @ load the next set
        vld4.32         {d0, d2, d4, d6}, [r1]!
        vld4.32         {d1, d3, d5, d7}, [r1]!      @ for next set

        bgt             .L_mainloop_vec4             @ loop if r2 > 0, if we have another 4 vec4s

.L_return_vec4:
        @ return
        mov               r0, #0
        bx                lr


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
@ NE10 Library : source/NE10_cross.neon.s
@

        .text
        .syntax   unified

.include "headers/NE10header.s"




        .align  4
        .global cross_vec3f_neon
        .thumb
        .thumb_func
cross_vec3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t cross_vec3f(arm_vec3f_t * dst,
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
        @  r4:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4}
        and               r4, r3, #3          @ r4 = count % 4; calculate the residual loop
        asr               r3, r3, #2          @ r3 = count >> 2; calculate the main loop

        cbz               r4, .L_check_mainloop_vec3

.L_residualloop_vec3:
        @ process the last few items left in the input array
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
        vmul.f32          d20, d2, d5
        vmul.f32          d21, d4, d1
        vmul.f32          d22, d0, d3

        vmls.f32          d20, d3, d4
        vmls.f32          d21, d5, d0
        vmls.f32          d22, d1, d2

        vst3.32           {d20[0], d21[0], d22[0]}, [r0]!

        bgt               .L_residualloop_vec3

.L_check_mainloop_vec3:
        cbz               r3, .L_return_vec3

        @ load current set of values
        vld3.32         {d0, d2, d4}, [r1]!
        vld3.32         {d1, d3, d5}, [r1]!
        vld3.32         {d26, d28, d30}, [r2]!
        vld3.32         {d27, d29, d31}, [r2]!

.L_mainloop_vec3:
        @ calculate values for the 2nd/next (e.g. 3rd) set
        vmul.f32        q10, q1, q15
        vmul.f32        q11, q2, q13
        vmul.f32        q12, q0, q14

        vmls.f32        q10, q14, q2
        vmls.f32        q11, q15, q0
        vmls.f32        q12, q13, q1

        @ store the result for the 1st/next (e.g. 3rd) set
        vst3.32         {d20, d22, d24}, [r0]!
        vst3.32         {d21, d23, d25}, [r0]!
        subs            r3, r3, #1

        @ load the next (e.g. 3rd) set of values
        vld3.32         {d0, d2, d4}, [r1]!
        vld3.32         {d1, d3, d5}, [r1]!
        vld3.32         {d26, d28, d30}, [r2]!
        vld3.32         {d27, d29, d31}, [r2]!

        bgt               .L_mainloop_vec3             @ loop if r2 is > r3, if we have at least another 4 vectors (12 floats) to process

.L_return_vec3:
     @ return
        pop               {r4}
        mov               r0, #0
        bx                lr

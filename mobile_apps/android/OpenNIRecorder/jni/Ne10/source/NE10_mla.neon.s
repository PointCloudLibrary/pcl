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
@ NE10 Library : source/NE10_mla.neon.s
@

        .text
        .syntax   unified

.include "headers/NE10header.s"




        .balign   4
        .global   mla_float_neon
        .thumb
        .thumb_func

mla_float_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t mla_float(arm_float_t * dst,
        @                 arm_float_t * acc,
        @                 arm_float_t * src1,
        @                 arm_float_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *acc & current acc entry's address
        @  r2: *src1 & current src1 entry's address
        @  r3: *src2 & current src2 entry's address
        @  r4: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r5:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4, r5}
        ldr               r4, [r13, #8]       @ r4 = count; r13 is the stack pointer (sp)

        and               r5, r4, #3          @ r5 = count % 4; ; This is what's left to be processed after this loop
        sub               r4, r4, r5          @ count = count - r5

        cbz               r4, .L_check_float

        @ load the 1st set of values
          vld1.32         {q0}, [r2]!
          vld1.32         {q1}, [r3]!
          vld1.32         {q3}, [r1]!
          subs            r4, r4, #4

        @ calculate values for the 1st set
          vmla.f32        q3, q0, q1         @ q3 += q0 * q1

          ble             .L_mainloopend_float

.L_mainloop_float:
       @ load the next (e.g. 2nd) set of values, leave loading acc until later
          vld1.32         {q0}, [r2]!
          vld1.32         {q1}, [r3]!

        @ store the result for the 1st/next (e.g. 2nd) set
         vst1.32         {d6,d7}, [r0]!

       @ load the next (e.g. 2nd) acc, and decrease the counter
          vld1.32         {q3}, [r1]!
          subs            r4, r4, #4

        @ calculate values for the next (e.g. 2nd) set
          vmla.f32        q3, q0, q1         @ q3 += q0 * q1

        bgt             .L_mainloop_float             @ loop if r4 > 0, if we have at least another 4 floats

.L_mainloopend_float:
        @ the last iteration for this call
        @ store the result for the last set of values (e.g 2nd set)
         vst1.32         {d6,d7}, [r0]!

.L_check_float:
     @ check if anything left to process at the end of the input array
        cmp               r5, #0
        ble               .L_return_float

.L_secondloop_float:
     @ process the last few items left in the input array
        vld1.f32          d0[0], [r2]!           @ Fill in d0[0]
        vld1.f32          d1[0], [r3]!           @ Fill in d1[0]
        vld1.f32          d2[0], [r1]!           @ Fill in d2[0]

        subs              r5, r5, #1

        @ values
        vmla.f32          d2, d0, d1

        vst1.32           {d2[0]}, [r0]!

        bgt               .L_secondloop_float

.L_return_float:
     @ return
        pop               {r4, r5}
        mov               r0, #0
        bx                lr




        .balign   4
        .global   vmla_vec2f_neon
        .thumb
        .thumb_func

vmla_vec2f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t vmla_vec2f(arm_vec2f_t * dst,
        @                 arm_vec2f_t * acc,
        @                 arm_vec2f_t * src1,
        @                 arm_vec2f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *acc & current acc entry's address
        @  r2: *src1 & current src1 entry's address
        @  r3: *src2 & current src2 entry's address
        @  r4: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r5:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4, r5}
        ldr               r4, [r13, #8]       @ r5 = count; r13 is the stack pointer (sp)

        and               r5, r4, #3          @ r5 = count % 4;
        sub               r4, r4, r5          @ count = count - r4; This is what's left to be processed after this loop

        cbz               r4, .L_check_vec2

        @ load the 1st set of values
          vld2.32         {q0-q1}, [r2]!
          vld2.32         {q2-q3}, [r3]!
          vld2.32         {q8-q9}, [r1]!
          subs            r4, r4, #4

        @ calculate values for the 1st set
          vmla.f32        q8, q0, q2
          vmla.f32        q9, q1, q3

          ble             .L_mainloopend_vec2

.L_mainloop_vec2:
        @ load the 2nd set of values
          vld2.32         {q0-q1}, [r2]!
          vld2.32         {q2-q3}, [r3]!

        @ store the result for the 1st/next (e.g. 2nd) set
          vst2.32         {d16,d17,d18,d19}, [r0]!

       @ load the next (e.g. 2nd) set of values
          vld2.32         {q8-q9}, [r1]!
          subs            r4, r4, #4

        @ calculate values for the 2nd set
          vmla.f32        q8, q0, q2
          vmla.f32        q9, q1, q3

        bgt             .L_mainloop_vec2             @ loop if r3 is > r4, if we have at least another 4 vectors (8 floats) to process

.L_mainloopend_vec2:
        @ the last iteration for this call
        @ store the result for the last set of values
          vst2.32         {d16,d17,d18,d19}, [r0]!

.L_check_vec2:
     @ check if anything left to process at the end of the input array
        cmp               r5, #0
        ble               .L_return_vec2

.L_secondloop_vec2:
     @ process the last few items left in the input array
        vld1.f32          d0, [r2]!
        vld1.f32          d1, [r3]!
        vld1.f32          d2, [r1]!

        subs              r5, r5, #1

        @ calculate values
        vmla.f32          d2, d0, d1

        vst1.32           {d2}, [r0]!

        bgt               .L_secondloop_vec2

.L_return_vec2:
     @ return
        pop               {r4, r5}
        mov               r0, #0
        bx                lr




        .align  2
        .global vmla_vec3f_neon
        .thumb
        .thumb_func
vmla_vec3f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t vmla_vec3f(arm_vec3f_t * dst,
        @                 arm_vec3f_t * acc,
        @                 arm_vec3f_t * src1,
        @                 arm_vec3f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *acc & current acc entry's address
        @  r2: *src1 & current src1 entry's address
        @  r3: *src2 & current src2 entry's address
        @  r4: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r5:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4, r5}
        ldr               r4, [r13, #8]       @ r4 = count; r13 is the stack pointer (sp)

        and               r5, r4, #3          @ r4 = count % 4;
        sub               r4, r4, r5          @ count = count - r4; This is what's left to be processed after this loop

        cmp               r4, #0
        beq               .L_check_vec3

        @ load the 1st set of values
          vld3.32         {d0, d2, d4}, [r2]!
          vld3.32         {d1, d3, d5}, [r2]!
          vld3.32         {d18, d20, d22}, [r3]!
          vld3.32         {d19, d21, d23}, [r3]!
          vld3.32         {d24, d26, d28}, [r1]!  @ part of q12, q13, and q14
          vld3.32         {d25, d27, d29}, [r1]!  @ part of q12, q13, and q14
          subs            r4, r4, #4

        @ calculate values for the 1st set
          vmla.f32        q12, q0, q9
          vmla.f32        q13, q1, q10
          vmla.f32        q14, q2, q11

          ble             .L_mainloopend_vec3

.L_mainloop_vec3:
       @ load the next (e.g. 2nd) set of values
          vld3.32         {d0, d2, d4}, [r2]!
          vld3.32         {d1, d3, d5}, [r2]!
          vld3.32         {d18, d20, d22}, [r3]!
          vld3.32         {d19, d21, d23}, [r3]!

        @ store the result for the 1st/next (e.g. 2nd) set
          vst3.32         {d24, d26, d28}, [r0]!
          vst3.32         {d25, d27, d29}, [r0]!

       @ finish loading ...
          vld3.32         {d24, d26, d28}, [r1]!  @ part of q12, q13, and q14
          vld3.32         {d25, d27, d29}, [r1]!  @ part of q12, q13, and q14
          subs            r4, r4, #4

        @ calculate values for the next (e.g. 2nd) set
          vmla.f32        q12, q0, q9
          vmla.f32        q13, q1, q10
          vmla.f32        q14, q2, q11

        bgt               .L_mainloop_vec3             @ loop if r3 is > r4, if we have at least another 4 vectors (12 floats) to process

.L_mainloopend_vec3:
        @ the last iteration for this call
        @ store the result for the last set of value
          vst3.32         {d24, d26, d28}, [r0]!
          vst3.32         {d25, d27, d29}, [r0]!

.L_check_vec3:
     @ check if anything left to process at the end of the input array
        cmp               r5, #0
        ble               .L_return_vec3

.L_secondloop_vec3:
     @ process the last few items left in the input array
        vld3.f32          {d0[0], d2[0], d4[0]}, [r2]!     @ The values are loaded like so:
                                                            @      q0 = { V1.x, -, -, - };
                                                            @      q1 = { V1.y, -, -, - };
                                                            @      q2 = { V1.z, -, -, - };
        vld3.f32          {d1[0], d3[0], d5[0]}, [r3]!     @ The values are loaded like so:
                                                            @      q0 = { V1.x, -, V2.x, - };
                                                            @      q1 = { V1.y, -, V2.y, - };
                                                            @      q2 = { V1.z, -, V2.z, - };
        vld3.f32          {d18[0], d20[0], d22[0]}, [r1]!    @ The values are loaded like so:
                                                            @      q9 = { acc.x, -, -, - };
                                                            @      q10 = { acc.y, -, -, - };
                                                            @      q11 = { acc.z, -, -, - };

        subs              r5, r5, #1

        @ calculate values for
        vmla.f32          d18,  d0, d1
        vmla.f32          d20,  d2, d3
        vmla.f32          d22,  d4, d5

        vst3.32           {d18[0], d20[0], d22[0]}, [r0]!

        bgt               .L_secondloop_vec3

.L_return_vec3:
     @ return
        pop               {r4, r5}
        mov               r0, #0
        bx                lr




        .align  2
        .global vmla_vec4f_neon
        .thumb
        .thumb_func
vmla_vec4f_neon:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t vmla_vec4f(arm_vec4f_t * dst,
        @                 arm_vec4f_t * acc,
        @                 arm_vec4f_t * src1,
        @                 arm_vec4f_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current dst entry's address
        @  r1: *acc & current acc entry's address
        @  r2: *src1 & current src1 entry's address
        @  r3: *src2 & current src2 entry's address
        @  r4: int count & the number of items in the input array that can be
        @                   processed in chunks of 4 vectors
        @
        @  r5:  the number of items that are left to be processed at the end of
        @                   the input array
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push              {r4, r5}
        ldr               r4, [r13, #8]       @ r4 = count; r13 is the stack pointer (sp)

        and               r5, r4, #3          @ r5 = count % 4;
        sub               r4, r4, r5          @ count = count - r5; This is what's left to be processed after this loop

        cmp               r4, #0
        beq               .L_check_vec4

        @ load the 1st set of values
          vld4.32         {d0, d2, d4, d6}, [r2]!
          vld4.32         {d1, d3, d5, d7}, [r2]!
          vld4.32         {d16, d18, d20, d22}, [r3]!
          vld4.32         {d17, d19, d21, d23}, [r3]!
          vld4.32         {d24, d26, d28, d30}, [r1]!  @ part of q12, q13, q14, and q15
          vld4.32         {d25, d27, d29, d31}, [r1]!  @ part of q12, q13, q14, and q15
          subs            r4, r4, #4

        @ calculate values for the 1st set
          vmla.f32        q12, q0, q8
          vmla.f32        q13, q1, q9
          vmla.f32        q14, q2, q10
          vmla.f32        q15, q3, q11

          ble             .L_mainloopend_vec4

.L_mainloop_vec4:
       @ load the next (e.g. 2nd) set of values
          vld4.32         {d0, d2, d4, d6}, [r2]!
          vld4.32         {d1, d3, d5, d7}, [r2]!
          vld4.32         {d16, d18, d20, d22}, [r3]!
          vld4.32         {d17, d19, d21, d23}, [r3]!

        @ store the result for the 1st/next (e.g. 2nd) set
          vst4.32         {d24, d26, d28, d30}, [r0]!
          vst4.32         {d25, d27, d29, d31}, [r0]!

        @ finish loading ....
          vld4.32         {d24, d26, d28, d30}, [r1]!  @ part of q12, q13, q14, and q15
          vld4.32         {d25, d27, d29, d31}, [r1]!  @ part of q12, q13, q14, and q15
          subs            r4, r4, #4

        @ calculate values for the next (e.g. 2nd) set
          vmla.f32        q12, q0, q8
          vmla.f32        q13, q1, q9
          vmla.f32        q14, q2, q10
          vmla.f32        q15, q3, q11

        bgt               .L_mainloop_vec4             @ loop if r3 is > r4, if we have at least another 4 vectors (16 floats) to process

.L_mainloopend_vec4:
        @ the last iteration for this call
        @ store the result for the last set of values
          vst4.32         {d24, d26, d28, d30}, [r0]!
          vst4.32         {d25, d27, d29, d31}, [r0]!

.L_check_vec4:
     @ check if anything left to process at the end of the input array
        cmp               r5, #0
        ble               .L_return_vec4

.L_secondloop_vec4:
     @ process the last few items left in the input array
        vld4.f32          {d0[0], d2[0], d4[0], d6[0]}, [r2]!     @ The values are loaded like so:
                                                                   @      q0 = { V1.x, -, -, - };
                                                                   @      q1 = { V1.y, -, -, - };
                                                                   @      q2 = { V1.z, -, -, - };
                                                                   @      q3 = { V1.w, -, -, - };
        vld4.f32          {d1[0], d3[0], d5[0], d7[0]}, [r3]!     @ The values are loaded like so:
                                                                   @      q0 = { V1.x, -, V2.x, - };
                                                                   @      q1 = { V1.y, -, V2.y, - };
                                                                   @      q2 = { V1.z, -, V2.z, - };
                                                                   @      q3 = { V1.w, -, V2.w, - };
        vld4.f32          {d24[0], d26[0], d28[0], d30[0]}, [r1]! @ The values are loaded like so:
                                                                   @      q12 = { acc.x, -, -, - };
                                                                   @      q13 = { acc.y, -, -, - };
                                                                   @      q14 = { acc.z, -, -, - };
                                                                   @      q15 = { acc.w, -, -, - };

        subs              r5, r5, #1

        @ calculate values
        vmla.f32          d24, d0, d1
        vmla.f32          d26, d2, d3
        vmla.f32          d28, d4, d5
        vmla.f32          d30, d6, d7

        vst4.32          {d24[0], d26[0], d28[0], d30[0]}, [r0]!

        bgt               .L_secondloop_vec4

.L_return_vec4:
     @ return
        pop               {r4, r5}
        mov               r0, #0
        bx                lr

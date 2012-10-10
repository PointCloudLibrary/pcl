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
@ NE10 Library : source/NE10_div.asm.s
@

        .text
        .syntax   unified

.include "Ne10/headers/NE10header.s"

        .balign   4
        .global   div_float_asm
        .thumb
        .thumb_func

div_float_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t div_float(arm_vec2f_t * dst,
        @                 arm_float_t * src1, const arm_float_t * src2,
        @                 unsigned int count)
        @
        @  r0: *dst & current src1 entry's address - made of base(r0)+offset(r5)
        @  r1: *src1 & current src1 entry's address - made of base(r1)+offset(r5)
        @  r2: *src2 & current src2 entry's address - made of base(r2)+offset(r5)
        @  r3: int count
        @
        @  r3: loop counter
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        cbz     r3, .LoopEndFloat

.LoopBeginFloat:
        vldr      s1, [r1]                @ Load s1 = src1[i]
        add       r1, r1, #4              @ move to the next entry
        vldr      s2, [r2]                @ Load s2 = src2[i]
        add       r2, r2, #4              @ next entry
        vdiv.f32  s10, s1, s2             @ s10 = src1[i] / src2[i]
        vstr      s10, [r0]               @ Store the result back into the main memory
        add       r0, r0, #4              @ next entry in the dst
        subs      r3, r3, #1              @ count down using the current index (i--)
        bne        .LoopBeginFloat        @ Continue if  "i < count"

.LoopEndFloat:
        mov     r0, NE10_OK             @ Return NE10_OK
        bx      lr

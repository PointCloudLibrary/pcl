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
@ NE10 Library : source/NE10_abs.asm.s
@

        .text
        .syntax   unified

.include "Ne10/headers/NE10header.s"

        .balign   4
        .global   abs_float_asm
        .thumb
        .thumb_func

abs_float_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t abs_float(arm_float_t * dst,
        @                 arm_float_t * src,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: *src
        @  r2: int count
        @
        @  r2: loop counter
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        cbz     r2, .LoopEndFloat
        mov     r3, #0
        vmov    s2, r3

.LoopBeginFloat:
        vldr      s1, [r1]                @ Load s1 = src[i]
        add       r1, r1, #4              @ move to the next item
        vabs.f32  s1, s1                  @ get the absolute value; s1 = abs(s1 - 0)
        vstr      s1, [r0]                @ Store it back into the main memory; dst[i] = s1
        add       r0, r0, #4              @ move to the next entry
        subs      r2, r2, #1              @ count down using the current index (i--)
        bne        .LoopBeginFloat        @ Continue if  "i < count"

.LoopEndFloat:
        mov     r0, NE10_OK             @ Return NE10_OK
        bx      lr

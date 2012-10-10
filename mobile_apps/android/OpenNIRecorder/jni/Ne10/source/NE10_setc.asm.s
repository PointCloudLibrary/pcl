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
@ NE10 Library : source/NE10_setc.asm.s
@

        .text
        .syntax   unified

.include "Ne10/headers/NE10header.s"

        .balign   4
        .global   setc_float_asm
        .thumb
        .thumb_func

setc_float_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t setc_float(arm_float_t * dst,
        @                 const arm_float_t cst,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: cst
        @  r2: int count
        @
        @  r2: loop counter
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        cbz     r2, .LoopEndFloat

.LoopBeginFloat:
        str       r1, [r0], #4            @ Store it back into the main memory
        subs      r2, r2, #1              @ count down using the current index (i--)
        bne        .LoopBeginFloat        @ Continue if  "i < count"

.LoopEndFloat:
        mov     r0, NE10_OK             @ Return NE10_OK
        bx      lr




        .balign   4
        .global   setc_vec2f_asm
        .thumb
        .thumb_func

setc_vec2f_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t setc_vec2f(arm_vec2f_t * dst,
        @                 const arm_vec2f_t * cst,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: *cst
        @  r2: int count
        @
        @  r2: loop counter
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push    {r4, r5}
        cbz     r2, .LoopEndVec2F
        ldr       r4, [r1, #0]            @ Load cst->x into r4
        ldr       r5, [r1, #4]            @ Load cst->y into r5

.LoopBeginVec2F:
        str       r4, [r0], #4            @ Store them in the destination
        str       r5, [r0], #4
        subs      r2, r2, #1              @ count down using the current index (i--)
        bne        .LoopBeginVec2F        @ Continue if  "i < count"

.LoopEndVec2F:
        mov     r0, NE10_OK             @ Return NE10_OK
        pop     {r4, r5}
        bx      lr




        .balign   4
        .global   setc_vec3f_asm
        .thumb
        .thumb_func

setc_vec3f_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t setc_vec3f(arm_vec3f_t * dst,
        @                 const arm_vec3f_t * cst,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: *cst
        @  r2: int count
        @
        @  r2: loop counter
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push    {r4, r5, r6}
        cbz     r2, .LoopEndVec3F
        ldr       r4, [r1, #0]            @ Load cst->x into r4
        ldr       r5, [r1, #4]            @ Load cst->y into r5
        ldr       r6, [r1, #8]            @ r6 = cst->z

.LoopBeginVec3F:
        str       r4, [r0], #4            @ Store them in the destination
        str       r5, [r0], #4
        str       r6, [r0], #4
        subs      r2, r2, #1              @ count down using the current index (i--)
        bne        .LoopBeginVec3F        @ Continue if  "i < count"

.LoopEndVec3F:
        mov     r0, NE10_OK             @ Return NE10_OK
        pop     {r4, r5, r6}
        bx      lr




        .balign   4
        .global   setc_vec4f_asm
        .thumb
        .thumb_func

setc_vec4f_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t setc_vec4f(arm_vec4f_t * dst,
        @                 const arm_vec4f_t * cst,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: *cst
        @  r2: int count
        @
        @  r2: loop counter
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push    {r4, r5, r6, r7}
        cbz     r2, .LoopEndVec4F
        ldr       r4, [r1, #0]            @ Load cst->x into r4
        ldr       r5, [r1, #4]            @ Load cst->y into r5
        ldr       r6, [r1, #8]            @ r6 = cst->z
        ldr       r7, [r1, #12]           @ r7 = cst->w

.LoopBeginVec4F:
        str       r4, [r0], #4            @ Store them in the destination
        str       r5, [r0], #4
        str       r6, [r0], #4
        str       r7, [r0], #4
        subs      r2, r2, #1              @ count down using the current index (i--)
        bne        .LoopBeginVec4F        @ Continue if  "i < count"

.LoopEndVec4F:
        mov     r0, NE10_OK             @ Return NE10_OK
        pop     {r4, r5, r6, r7}
        bx      lr

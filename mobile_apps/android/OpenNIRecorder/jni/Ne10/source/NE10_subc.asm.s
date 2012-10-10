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
@ NE10 Library : source/NE10_subc.asm.s
@

        .text
        .syntax   unified

.include "Ne10/headers/NE10header.s"

        .balign   4
        .global   subc_float_asm
        .thumb
        .thumb_func

subc_float_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t subc_float(arm_vec2f_t * dst,
        @                 arm_float_t * src, const arm_float_t * cst,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: *src
        @  r2: cst
        @  r3: int count
        @
        @  r3: loop counter
        @  r5: current item's offset in both src[] and dst[]
        @  r6: current source item's address made of base(r1)+offset(r5)
        @  r7: current destination item's address made of base(r0)+offset(r5)
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push    {r4, r5, r6, r7}
        cbz     r3, .LoopEndFloat
        mov     r5, #0

.LoopBeginFloat:
        add       r6, r1, r5              @ Get current source item's address in memory
        vldr      s1, [r6, #0]            @ Load src[i]
        vmov      s3, r2                  @ Get cst into register s3
        vsub.f32  s10, s1, s3             @ s10 = src[i] - cst
        add       r7, r0, r5              @ Get current destination item's address in memory
        vstr      s10, [r7, #0]           @ Store the result back into the main memory
        add       r5, r5, #4              @ increase the offset by 1*sizeof(float) @@ (for x and y)
        subs      r3, r3, #1              @ count down using the current index (i--)
        bne        .LoopBeginFloat        @ Continue if  "i < count"

.LoopEndFloat:
        mov     r0, NE10_OK             @ Return NE10_OK
        pop     {r4, r5, r6, r7}
        bx      lr




        .balign   4
        .global   subc_vec2f_asm
        .thumb
        .thumb_func

subc_vec2f_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t subc_vec2f(arm_vec2f_t * dst,
        @                 arm_vec2f_t * src, const arm_vec2f_t * cst,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: *src
        @  r2: *cst
        @  r3: int count
        @
        @  r3: loop counter
        @  r5: current item's offset in both src[] and dst[]
        @  r6: current source item's address made of base(r1)+offset(r5)
        @  r7: current destination item's address made of base(r0)+offset(r5)
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push    {r4, r5, r6, r7}
        cbz     r3, .LoopEndVec2F
        mov     r5, #0

.LoopBeginVec2F:
        add       r6, r1, r5              @ Get current source item's address in memory
        vldr      s1, [r6, #0]            @ Load src[i].x and src[i].y
        vldr      s2, [r6, #4]
        vldr      s3, [r2, #0]            @ Load cst->x and cst->y
        vldr      s4, [r2, #4]
        vsub.f32  s10, s1, s3             @ s10 = src[i].x - cst->x
        vsub.f32  s11, s2, s4             @ s11 = src[i].y - cst->y
        add       r7, r0, r5              @ Get current destination item's address in memory
        vstr      s10, [r7, #0]           @ Store the results back into the main memory
        vstr      s11, [r7, #4]
        add       r5, r5, #8              @ increase the offset by 2*sizeof(float) @@ (for x and y)
        subs      r3, r3, #1              @ count down using the current index (i--)
        bne        .LoopBeginVec2F        @ Continue if  "i < count"

.LoopEndVec2F:
        mov     r0, NE10_OK             @ Return NE10_OK
        pop     {r4, r5, r6, r7}
        bx      lr




        .balign   4
        .global   subc_vec3f_asm
        .thumb
        .thumb_func

subc_vec3f_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t subc_vec3f(arm_vec3f_t * dst,
        @                 arm_vec3f_t * src, const arm_vec3f_t * cst,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: *src
        @  r2: *cst
        @  r3: int count
        @
        @  r3: loop counter
        @  r5: current item's offset in both src[] and dst[]
        @  r6: current source item's address made of base(r1)+offset(r5)
        @  r7: current destination item's address made of base(r0)+offset(r5)
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push    {r4, r5, r6, r7}
        cbz     r3, .LoopEndVec3F
        mov     r5, #0

.LoopBeginVec3F:
        add       r6, r1, r5              @ Get current source item's address in memory
        vldr      s1, [r6, #0]            @ Load src[i].x, src[i].y , and src[i].z
        vldr      s2, [r6, #4]
        vldr      s3, [r6, #8]
        vldr      s4, [r2, #0]            @ Load cst->x, cst->y, and cst->z
        vldr      s5, [r2, #4]
        vldr      s6, [r2, #8]
        vsub.f32  s10, s1, s4             @ s10 = src[i].x - cst->x
        vsub.f32  s11, s2, s5             @ s11 = src[i].y - cst->y
        vsub.f32  s12, s3, s6             @ s12 = src[i].z - cst->z
        add       r7, r0, r5              @ Get current destination item's address in memory
        vstr      s10, [r7, #0]           @ Store the results back into the main memory
        vstr      s11, [r7, #4]
        vstr      s12, [r7, #8]
        add       r5, r5, #12             @ increase the offset by 3*sizeof(float) @@ (for x, y, and z)
        subs      r3, r3, #1              @ count down using the current index (i--)
        bne        .LoopBeginVec3F        @ Continue if  "i < count"

.LoopEndVec3F:
        mov     r0, NE10_OK             @ Return NE10_OK
        pop     {r4, r5, r6, r7}
        bx      lr




        .balign   4
        .global   subc_vec4f_asm
        .thumb
        .thumb_func

subc_vec4f_asm:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ arm_result_t subc_vec4f(arm_vec4f_t * dst,
        @                 arm_vec4f_t * src, const arm_vec4f_t * cst,
        @                 unsigned int count)
        @
        @  r0: *dst
        @  r1: *src
        @  r2: *cst
        @  r3: int count
        @
        @  r3: loop counter
        @  r5: current item's offset in both src[] and dst[]
        @  r6: current source item's address made of base(r1)+offset(r5)
        @  r7: current destination item's address made of base(r0)+offset(r5)
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        push    {r4, r5, r6, r7}
        cbz     r3, .LoopEndVec4F
        mov     r5, #0

.LoopBeginVec4F:
        add       r6, r1, r5              @ Get current source item's address in memory
        vldr      s1, [r6, #0]            @ Load src[i].x, src[i].y , src[i].z, and w
        vldr      s2, [r6, #4]
        vldr      s3, [r6, #8]
        vldr      s4, [r6, #12]
        vldr      s5, [r2, #0]            @ Load cst->x, cst->y, cst->z, and w
        vldr      s6, [r2, #4]
        vldr      s7, [r2, #8]
        vldr      s8, [r2, #12]
        vsub.f32  s10, s1, s5             @ s10 = src[i].x - cst->x
        vsub.f32  s11, s2, s6             @ s11 = src[i].y - cst->y
        vsub.f32  s12, s3, s7             @ s12 = src[i].z - cst->z
        vsub.f32  s13, s4, s8             @ s13 = src[i].w - cst->w
        add       r7, r0, r5              @ Get current destination item's address in memory
        vstr      s10, [r7, #0]           @ Store the results back into the main memory
        vstr      s11, [r7, #4]
        vstr      s12, [r7, #8]
        vstr      s13, [r7, #12]
        add       r5, r5, #16             @ increase the offset by 4*sizeof(float) @@ (for x, y, z, and w)
        subs      r3, r3, #1              @ count down using the current index (i--)
        bne        .LoopBeginVec4F        @ Continue if  "i < count"

.LoopEndVec4F:
        mov     r0, NE10_OK             @ Return NE10_OK
        pop     {r4, r5, r6, r7}
        bx      lr

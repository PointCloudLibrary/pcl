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
@ NE10 Library : source/NE10_detmat.neon.inc.s
@




         @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
         @ Get determinants of two 2x2 matrices in dRes
         @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
         .macro GET_DET_2x2MATS_ARGS  dA, dB, dC, dD, dRes
           vmul.f32        \dRes, \dA, \dD
           vmls.f32        \dRes, \dB, \dC
         .endm




         @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
         @ Get negated determinants of two 2x2 matrices in dRes
         @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
         .macro GET_NEG_DET_2x2MATS_ARGS  dA, dB, dC, dD, dRes
            GET_DET_2x2MATS_ARGS \dC, \dD, \dA, \dB, \dRes
         .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro used inside detmat_3x3f_neon() to load 3x3 matrices.
        @ Two 3x3 matrices are loaded from the source address
        @ into registers dst00-11. The corresponding qr00-qr05
        @ registers are then rearranged so the order of the data fits the
        @ code written in other macros below.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro LOAD_3x3MATS_ARGS  dst00, dst01, dst02, dst03, dst04, dst05,  dst06, dst07, dst08, dst09, dst10, dst11,  qr00, qr01, qr02, qr03, qr04, qr05, addr

            vld3.32     { \dst00, \dst02, \dst04 }, [\addr]!
            vld3.32     { \dst01[0], \dst03[0], \dst05[0] }, [\addr]!
            vld3.32     { \dst06, \dst08, \dst10 }, [\addr]!
            vld3.32     { \dst07[0], \dst09[0], \dst11[0] }, [\addr]!

             vtrn.32     \qr00, \qr03
             vtrn.32     \qr01, \qr04
             vtrn.32     \qr02, \qr05
        .endm




       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       @ This macro calculates the determinant of two 3x3 matrices
       @ loaded using the above LOAD_3x3MATS_ARGS macro.
       @ The result is stored in the \res register.
       @ Registers \tmp2 and \tmp3 are used as scratch registers and will
       @ not be restored in this macro - the caller needs to resotre them
       @ if needed. Each of the aa-ii parameters can be a "d" register
       @ containing two floating-point values which correspond to the
       @ following reference matrix:
       @
       @      |aa dd gg|
       @  M = |bb ee hh|
       @      |cc ff ii|
       @
       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       .macro GET_DETERMINANT_of_3x3MATS_ARGS   aa, bb, cc, dd, ee, ff, gg, hh, ii, res, tmp2, tmp3
           @ det = a*(ei-fh) - d*(bi-ch) + g*(bf-ec)

           vmul.f32    \res, \ee, \ii     @ t1 = ei
           vmul.f32   \tmp2, \bb, \ii     @ t2 = bi
           vmul.f32   \tmp3, \bb, \ff     @ t3 = bf

           vmls.f32    \res, \ff, \hh     @ t1 = ei-fh
           vmls.f32   \tmp2, \cc, \hh     @ t2 = bi-ch
           vmls.f32   \tmp3, \ee, \cc     @ t3 = bf-ec

           vmul.f32    \res, \aa, \res    @ t1 = a*(ei-fh)
           vmls.f32    \res, \dd, \tmp2   @ t1 = a*(ei-fh) - d*(bi-ch)
           vmla.f32    \res, \gg, \tmp3   @ t1 = a*(ei-fh) - d*(bi-ch) + g*(bf-ec) = det(M1), det(M2)
       .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ This macro calculates nagated determinant of two 3x3 matrices
        @ The result is stored in \res
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro GET_NEG_DET_3x3MATS_ARGS   aa, bb, cc, dd, ee, ff, gg, hh, ii, res, tmp2, tmp3
           @ det = - a*(ei-fh) + d*(bi-ch) - g*(bf-ec)
           GET_DETERMINANT_of_3x3MATS_ARGS   \dd, \ee, \ff, \aa, \bb, \cc, \gg, \hh, \ii, \res, \tmp2, \tmp3    @ Using the column exchange property
        .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro used inside detmat_4x4f_neon() to load 4x4 matrices.
        @ Two 4x4 matrices are loaded from the source address register \addr
        @ into registers dst00-15. The corresponding qr00-qr07
        @ registers are then rearranged so the order of the data fits the
        @ code written in other macros below.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro LOAD_4x4MATS_ARGS  dst00, dst01, dst02, dst03, dst04, dst05, dst06, dst07,  dst08, dst09, dst10, dst11, dst12, dst13, dst14, dst15,  qr00, qr01, qr02, qr03, qr04, qr05, qr06, qr07, addr

            vld4.32     { \dst00, \dst02, \dst04, \dst06 }, [\addr]!
            vld4.32     { \dst01, \dst03, \dst05, \dst07 }, [\addr]!
            vld4.32     { \dst08, \dst10, \dst12, \dst14 }, [\addr]!
            vld4.32     { \dst09, \dst11, \dst13, \dst15 }, [\addr]!

             vtrn.32     \qr00, \qr04
             vtrn.32     \qr01, \qr05
             vtrn.32     \qr02, \qr06
             vtrn.32     \qr03, \qr07
        .endm




       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       @ This macro calculates the determinant of 4x4 matrices
       @ loaded using the above LOAD_4x4MATS_ARGS macro.
       @ The result is stored in the \res register.
       @ Registers \tmp2 to \tmp6 are used as scratch registers and will
       @ not be restored in this macro - the caller needs to resotre them
       @ if needed. Each of the aa-pp parameters can be a "d" register
       @ containing two floating-point values which correspond to the
       @ following reference matrix:
       @
       @      |aa ee ii mm|
       @  M = |bb ff jj nn|
       @      |cc gg kk oo|
       @      |dd hh ll pp|
       @
       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       .macro GET_DETERMINANT_of_4x4MATS_ARGS   aa, bb, cc, dd, ee, ff, gg, hh, ii, jj, kk, ll, mm, nn, oo, pp,  res, tmp2, tmp3, tmp4, tmp5, tmp6

          @ res  = det(SubM11)
          GET_DETERMINANT_of_3x3MATS_ARGS   \ff, \gg, \hh, \jj, \kk, \ll, \nn, \oo, \pp,  \res, \tmp5, \tmp6

          @ tmp2 = det(SubM12)
          GET_DETERMINANT_of_3x3MATS_ARGS   \bb, \cc, \dd, \jj, \kk, \ll, \nn, \oo, \pp, \tmp2, \tmp5, \tmp6

          @ tmp3 = det(SubM13)
          GET_DETERMINANT_of_3x3MATS_ARGS   \bb, \cc, \dd, \ff, \gg, \hh, \nn, \oo, \pp, \tmp3, \tmp5, \tmp6

          @ tmp4 = det(SubM14)
          GET_DETERMINANT_of_3x3MATS_ARGS   \bb, \cc, \dd, \ff, \gg, \hh, \jj, \kk, \ll, \tmp4, \tmp5, \tmp6


           vmul.f32    \res, \aa, \res
           vmls.f32    \res, \ee, \tmp2
           vmla.f32    \res, \ii, \tmp3
           vmls.f32    \res, \mm, \tmp4
       .endm




        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @ A macro used inside detmat_4x4f_neon() to load four 4x4 matrices
        @ from the memory location pointed to by the \addr register.
        @ The loaded matrices are stored in registers dst00-07 and
        @ finaklly rearranged using the corresponding registers qr00-qr03.
        @ qtmp1-qtmp4 are scratch registers which are not resotred in this
        @ maroc. The caller must restored them if needed.
        @ NOTE: Through out Ne10, matrices are loaded and stored in
        @ column major format.
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .macro LOAD_SINGLE_4x4MAT_ARGS dst00, dst01, dst02, dst03, dst04, dst05, dst06, dst07, qr00, qr01, qr02, qr03, qtmp1, qtmp2, qtmp3, qtmp4, addr

            vld4.32     { \dst00, \dst02, \dst04, \dst06 }, [\addr]!
            vld4.32     { \dst01, \dst03, \dst05, \dst07 }, [\addr]!

             vtrn.32     \qr00, \qtmp1
             vtrn.32     \qr01, \qtmp2
             vtrn.32     \qr02, \qtmp3
             vtrn.32     \qr03, \qtmp4
         .endm

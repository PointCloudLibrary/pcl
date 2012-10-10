/*
 *  Copyright 2011-12 ARM Limited
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*
 * NE10 Library : headers/macros.h
 */

#include "factor.h"

// Macros used in actual implementations

///// The "DstSrcCst" group of functions - FLOAT /////

#define NE10_XC_OPERATION_X_C(loopCode) { \
   NE10_TEMPLATE_XC_OPERATION_X_C( \
      NE10_CHECKPOINTER_DstSrcCst_OPERATION; , \
      loopCode); \
  }

#define NE10_XC_OPERATION_FLOAT_NEON(loopCode1, loopCode2) { \
   float32x4_t n_cst = { cst, cst, cst, cst }; \
   NE10_DstSrcCst_OPERATION_FLOAT_NEON(  \
    NE10_CHECKPOINTER_DstSrcCst_OPERATION; , \
    NE10_DstSrcCst_MAINLOOP_FLOAT_NEON(loopCode1); , \
    NE10_DstSrcCst_SECONDLOOP_FLOAT_NEON(loopCode2); \
   ); \
  }

#define NE10_XC_OPERATION_VEC2F_NEON(loopCode1, loopCode2) { \
   NE10_DstSrcCst_OPERATION_VEC2F_NEON(  \
    NE10_CHECKPOINTER_DstSrcCst_OPERATION; , \
    NE10_DstSrcCst_MAINLOOP_VEC2F_NEON(loopCode1); , \
    NE10_DstSrcCst_SECONDLOOP_VEC2F_NEON(loopCode2); \
   ); \
  }

/* This macro uses interleaving to boost the performance */
#define NE10_XC_OPERATION_VEC3F_NEON(loopCode1, loopCode2) { \
   NE10_DstSrcCst_OPERATION_VEC3F_NEON(  \
    NE10_CHECKPOINTER_DstSrcCst_OPERATION; , \
    NE10_DstSrcCst_MAINLOOP_VEC3F_NEON(loopCode1); , \
    NE10_DstSrcCst_SECONDLOOP_VEC3F_NEON(loopCode2); \
   ); \
  }

#define NE10_XC_OPERATION_VEC4F_NEON(loopCode) { \
   NE10_DstSrcCst_OPERATION_VEC4F_NEON( \
    NE10_CHECKPOINTER_DstSrcCst_OPERATION; , \
    NE10_DstSrcCst_MAINLOOP_VEC4F_NEON(loopCode); \
   ); \
  }

///// The "DstAccSrcCst" group of functions - FLOAT //////

#define NE10_MLAC_OPERATION_X_C(loopCode) { \
   NE10_TEMPLATE_XC_OPERATION_X_C( \
    NE10_CHECKPOINTER_DstAccSrcCst_OPERATION; , \
    loopCode); \
 }

#define NE10_MLAC_OPERATION_FLOAT_NEON(loopCode1, loopCode2) { \
   float32x4_t n_acc; \
   float32x4_t n_cst = { cst, cst, cst, cst }; \
   NE10_DstAccSrcCst_OPERATION_FLOAT_NEON(  \
    NE10_CHECKPOINTER_DstAccSrcCst_OPERATION; , \
    NE10_DstAccSrcCst_MAINLOOP_FLOAT_NEON(loopCode1); , \
    NE10_DstAccSrcCst_SECONDLOOP_FLOAT_NEON(loopCode2); \
   ); \
  }

#define NE10_MLAC_OPERATION_VEC2F_NEON(loopCode1, loopCode2) { \
   float32x4_t n_acc; \
   NE10_DstAccSrcCst_OPERATION_VEC2F_NEON(  \
    NE10_CHECKPOINTER_DstAccSrcCst_OPERATION; , \
    NE10_DstAccSrcCst_MAINLOOP_VEC2F_NEON(loopCode1); , \
    NE10_DstAccSrcCst_SECONDLOOP_VEC2F_NEON(loopCode2); \
   ); \
  }

#define NE10_MLAC_OPERATION_VEC3F_NEON(loopCode1, loopCode2) { \
   float32x4_t n_acc1, n_acc2, n_acc3; \
   NE10_DstAccSrcCst_OPERATION_VEC3F_NEON(  \
    NE10_CHECKPOINTER_DstAccSrcCst_OPERATION; , \
    NE10_DstAccSrcCst_MAINLOOP_VEC3F_NEON(loopCode1); , \
    NE10_DstAccSrcCst_SECONDLOOP_VEC3F_NEON(loopCode2); \
   ); \
  }

#define NE10_MLAC_OPERATION_VEC4F_NEON(loopCode) { \
   float32x4_t n_acc; \
   NE10_DstAccSrcCst_OPERATION_VEC4F_NEON( \
    NE10_CHECKPOINTER_DstAccSrcCst_OPERATION; , \
    NE10_DstAccSrcCst_MAINLOOP_VEC4F_NEON(loopCode); \
   ); \
  }

///// The "DstCst" group of functions - FLOAT /////

#define NE10_SETC_OPERATION_X_C(loopCode) { \
   NE10_TEMPLATE_XC_OPERATION_X_C( \
    NE10_CHECKPOINTER_DstCst_OPERATION; , \
    loopCode); \
  }

#define NE10_SETC_OPERATION_FLOAT_NEON(loopCode1, loopCode2) { \
   float32x4_t n_cst = { cst, cst, cst, cst }; \
   NE10_DstCst_OPERATION_FLOAT_NEON(  \
    NE10_CHECKPOINTER_DstCst_OPERATION; , \
    NE10_DstCst_MAINLOOP_FLOAT_NEON(loopCode1); , \
    NE10_DstCst_SECONDLOOP_FLOAT_NEON(loopCode2); \
   ); \
  }

#define NE10_SETC_OPERATION_VEC2F_NEON(loopCode1, loopCode2) { \
   NE10_DstCst_OPERATION_VEC2F_NEON(  \
    NE10_CHECKPOINTER_DstCst_OPERATION; , \
    NE10_DstCst_MAINLOOP_VEC2F_NEON(loopCode1); , \
    NE10_DstCst_SECONDLOOP_VEC2F_NEON(loopCode2); \
   ); \
  }

/* This macro uses interleaving to boost the performance */
#define NE10_SETC_OPERATION_VEC3F_NEON(loopCode1, loopCode2) { \
   NE10_DstCst_OPERATION_VEC3F_NEON(  \
    NE10_CHECKPOINTER_DstCst_OPERATION; , \
    NE10_DstCst_MAINLOOP_VEC3F_NEON(loopCode1); , \
    NE10_DstCst_SECONDLOOP_VEC3F_NEON(loopCode2); \
   ); \
  }

#define NE10_SETC_OPERATION_VEC4F_NEON(loopCode) { \
   NE10_DstCst_OPERATION_VEC4F_NEON( \
    NE10_CHECKPOINTER_DstCst_OPERATION; , \
    NE10_DstCst_MAINLOOP_VEC4F_NEON(loopCode); \
   ); \
  }

///// The "DstSrc1Src2" group of functions //////

#define NE10_X_OPERATION_FLOAT_C(loopCode) { \
   NE10_TEMPLATE_XC_OPERATION_X_C( \
      NE10_CHECKPOINTER_DstSrc1Src2_OPERATION; , \
      loopCode); \
  }

#define NE10_X_OPERATION_FLOAT_NEON(loopCode1, loopCode2) { \
   float32x4_t n_src2; \
   NE10_DstSrc1Src2_OPERATION_FLOAT_NEON(  \
    NE10_CHECKPOINTER_DstSrc1Src2_OPERATION; , \
    NE10_DstSrc1Src2_MAINLOOP_FLOAT_NEON(loopCode1); , \
    NE10_DstSrc1Src2_SECONDLOOP_FLOAT_NEON(loopCode2); \
   ); \
  }

#define NE10_DOT_OPERATION_X_C NE10_X_OPERATION_FLOAT_C

///// The "DstSrc" group of functions //////

#define NE10_ABS_OPERATION_X_C(loopCode) { \
   NE10_TEMPLATE_XC_OPERATION_X_C( \
    NE10_CHECKPOINTER_DstSrc_OPERATION, \
    loopCode); \
  }

#define NE10_ABS_OPERATION_FLOAT_C NE10_ABS_OPERATION_X_C

#define NE10_ABS_OPERATION_FLOAT_NEON(loopCode1, loopCode2) { \
   arm_float_t cst = 0.0f; /* this is used to compare the values against. */ \
   float32x4_t n_cst = { cst, cst, cst, cst }; \
   NE10_DstSrc_OPERATION_FLOAT_NEON(  \
    NE10_CHECKPOINTER_DstSrc_OPERATION; , \
    NE10_DstSrc_MAINLOOP_FLOAT_NEON(loopCode1); , \
    NE10_DstSrc_SECONDLOOP_FLOAT_NEON(loopCode2); \
   ); \
  }

#define NE10_LEN_OPERATION_X_C NE10_ABS_OPERATION_X_C

#define NE10_LEN_OPERATION_X_C NE10_ABS_OPERATION_X_C

#define NE10_CMATVEC_OPERATION_X_C NE10_ABS_OPERATION_X_C

#define NE10_LEN_OPERATION_VEC2F_NEON(loopCode1, loopCode2) { \
    NE10_DstSrc_OPERATION_VEC2F_NEON( \
     NE10_CHECKPOINTER_DstSrcCst_OPERATION, \
     NE10_DstSrc_MAINLOOP_VEC2F_NEON(loopCode1), \
     NE10_DstSrc_SECONDLOOP_VEC2F_NEON(loopCode2) \
     ); \
  }

#define NE10_LEN_OPERATION_VEC3F_NEON(loopCode1, loopCode2) { \
    NE10_DstSrc_OPERATION_VEC3F_NEON( \
     NE10_CHECKPOINTER_DstSrcCst_OPERATION, \
     NE10_DstSrc_MAINLOOP_VEC3F_NEON(loopCode1), \
     NE10_DstSrc_SECONDLOOP_VEC3F_NEON(loopCode2) \
     ); \
  }

#define NE10_LEN_OPERATION_VEC4F_NEON(loopCode) { \
    NE10_DstSrc_OPERATION_VEC4F_NEON( \
     NE10_CHECKPOINTER_DstSrcCst_OPERATION, \
     NE10_DstSrc_MAINLOOP_VEC4F_NEON(loopCode) \
     ); \
  }

#define NE10_DETMAT_OPERATION_X_C NE10_ABS_OPERATION_X_C

///// The "DstAccSrc1Src2" group of functions //////

#define NE10_MLA_OPERATION_FLOAT_NEON(loopCode1, loopCode2) { \
   float32x4_t n_acc; \
   float32x4_t n_src2; \
   NE10_DstAccSrc1Src2_OPERATION_FLOAT_NEON(  \
    NE10_CHECKPOINTER_DstAccSrc1Src2_OPERATION; , \
    NE10_DstAccSrc1Src2_MAINLOOP_FLOAT_NEON(loopCode1); , \
    NE10_DstAccSrc1Src2_SECONDLOOP_FLOAT_NEON(loopCode2); \
   ); \
  }

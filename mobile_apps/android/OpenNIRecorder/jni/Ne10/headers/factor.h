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
 * NE10 Library : headers/factor.h
 */

// Typebuilding MACROs
// - Slight difference between toolchain versions on intrinsics
#define FLOAT32_2x3(x1,y1,x2,y2,x3,y3) \
    {{ \
        {x1, y1}, {x2,y2}, {x3,y3} \
    }}

// Unit test use this macro to index into their function table
// "opc" stands for operation's code (which function),
// and "imp" stands for implementation (which implementation of the function)
#define FTBL_IDX(opc, imp) ((opc-1)*IMPL_COUNT+(imp-1))

// This macro helps measure the performance of the code passed to it through the "code" argument
// It is used in the unit tests
#define MEASURE(res, code) \
   { \
    gettimeofday (&before, &zone); \
      code \
    gettimeofday (&after, &zone); \
    if (before.tv_usec > after.tv_usec) \
    { \
      after.tv_usec += 1000000; \
      after.tv_sec--; \
    } \
    lapsed.tv_usec = after.tv_usec - before.tv_usec; \
    lapsed.tv_sec  = after.tv_sec  - before.tv_sec; \
    res = lapsed.tv_sec + ((double)lapsed.tv_usec / 1000000.0); \
   }

// There are several categories of functions that share common code:

// Different groups of functions take different number of inputs
//
// Group 1 = Functions that take a dst, a src, and a cst ("DstSrcCst" for short)
// Group 2 = Those that take a dst, an acc, a src, and a cst ("DstAccSrcCst" for short)
// Group 3 = The ones that take a dst, and a cst only ("DstCst" for short)
//
// Group 4 = These take a dst, and two src inputs, src2 and scr2 ("DstSrc1Src2")
// Group 5 = These take a dst, an acc, and two src inputs ("DstAccSrc1Src2")
// Group 6 = These take a dst, and a src ("DstSrc")
//

// The naming convention used in the following macros is as follows:
//   SNAPP_<A>_OPERATION_<T>_<I>
//   where
//   <A> Stands for the title of the operation (add, mul, etc) followed by its type (C = const as in addc).
//       The letter X - if used - means any such operation.
//   <T> Indicates the type of the operation (float, vec2, etc.)
//       The letter X - is used - means any type.
//   <I> This indicates the implementation (it can be C, ASM, or NEON).

// A few macros to check pointers and their address range to make sure there's
//  no unwanted overlap between any two of them
#define NE10_CHECKPOINTER_DstSrcCst_OPERATION \
   if ( dst < src ) \
    { assert ( dst + count <= src ); } \
   else if ( dst > src ) \
    { assert ( src + count <= dst ); }

#define NE10_CHECKPOINTER_DstSrc_OPERATION NE10_CHECKPOINTER_DstSrcCst_OPERATION

#define NE10_CHECKPOINTER_3POINTER_OPERATION(arg1, arg2, arg3) \
   if ( arg1 < arg2 ) \
    { assert ( arg1 + count <= arg2 ); } \
   else if ( arg1 > arg2 ) \
    { assert ( arg2 + count <= arg1 ); } \
   if ( arg1 < arg3 ) \
    { assert ( arg1 + count <= arg3 ); } \
   else if ( arg1 > arg3 ) \
    { assert ( arg3 + count <= arg1 ); } \
   if ( arg3 < arg2 ) \
    { assert ( arg3 + count <= arg2 ); } \
   else if ( arg3 > arg2 ) \
    { assert ( arg2 + count <= arg3 ); }

#define NE10_CHECKPOINTER_4POINTER_OPERATION(arg1, arg2, arg3, arg4) \
   NE10_CHECKPOINTER_3POINTER_OPERATION(arg1, arg2, arg3) \
   if ( arg1 < arg4 ) \
    { assert ( arg1 + count <= arg4 ); } \
   else if ( arg1 > arg4 ) \
    { assert ( arg4 + count <= arg1 ); } \
   if ( arg2 < arg4 ) \
    { assert ( arg2 + count <= arg4 ); } \
   else if ( arg2 > arg4 ) \
    { assert ( arg4 + count <= arg2 ); } \
   if ( arg4 < arg3 ) \
    { assert ( arg4 + count <= arg3 ); } \
   else if ( arg4 > arg3 ) \
    { assert ( arg3 + count <= arg4 ); }



#define NE10_CHECKPOINTER_DstAccSrcCst_OPERATION { \
   NE10_CHECKPOINTER_3POINTER_OPERATION(dst, acc, src); }

#define NE10_CHECKPOINTER_DstCst_OPERATION  {}

#define NE10_CHECKPOINTER_DstSrc1Src2_OPERATION { \
   NE10_CHECKPOINTER_3POINTER_OPERATION(dst, src1, src2); }

#define NE10_CHECKPOINTER_DstAccSrc1Src2_OPERATION { \
   NE10_CHECKPOINTER_4POINTER_OPERATION(dst, acc, src1, src2); }

// These macros generalise implementation of the functions.

// Macros used in C implementations
#define NE10_TEMPLATE_XC_OPERATION_X_C(checkPointer, loopCode) { \
   arm_result_t res = NE10_OK; \
   unsigned int itr = 0; \
   checkPointer; \
   for ( itr = 0; itr < count; itr++ ) \
   { loopCode ; /* this loop iterates through each and every float item one at a time */ \
   } \
   return res; \
  }

// macros used in the NEON implementations

// Main Loop = The loop where the number of items to be processed is exactly the
//              number that we can process in a single iteration.
//
// Secondary Loop = The loop that follows a Main Loop to fill in the entries that
//                   did not fit into the Main Loop. This is needed when the number of
//                   input items is not a multiple of the number of items that we
//                   process in every iteration of the Main Loop.


/****************************************************
 *                                                  *
 *  The "DstSrcCst" group of functions              *
 *                                                  *
 ****************************************************/

///// - FLOAT - /////

#define NE10_DstSrcCst_MAINLOOP_FLOAT_NEON(loopCode) { \
     /* load 4 values  */ \
     n_src = vld1q_f32( (float32_t*)src ); \
     src += 4; /* move to the next 4 float items; 4*float */ \
     loopCode; /* the actual operation is placed here... */ /* The main loop iterates through four float values each time */ \
     vst1q_f32 ( (float32_t*)dst , n_dst ); /* store the results back */ \
     dst += 4; /* move to the next items; 4*float */ \
    }

#define NE10_DstSrcCst_SECONDLOOP_FLOAT_NEON(loopCode) { \
      float32x2_t n_tmp_src = { 0.0f , 0.0f }; /* temporary storage to be used with NEON load/store intrinsics */ \
      float32x2_t n_tmp_cst = { cst, cst }; /* temporary constant value for use in the main NEON operation */ \
      n_tmp_src = vld1_lane_f32 ( (float32_t*)src, n_tmp_src, 0); /* load into the first lane of d0 */ \
      loopCode; /* the actual operation is placed here ... */ /* exceptional cases where the count is not a multiple of 4 */ \
      vst1_lane_f32( (float32_t*)dst, n_tmp_src, 0); /* store the lane back into the memory */ \
      /* move to the next item in the stream */ \
      src++; \
      dst++; \
     }

#define NE10_DstSrcCst_OPERATION_FLOAT_NEON(checkPointer, loopCode1, loopCode2) { \
   arm_result_t res = NE10_OK; \
   float32x4_t n_src; \
   float32x4_t n_dst; \
   checkPointer; \
   int dif = 0; \
   dif = count % 4; /* either 0 or one of 1,2,3; in the latter cases the second path is taken */ \
   for (; count > dif; count -= 4) { \
     loopCode1; \
    } \
   if ( 0 != dif ) { \
    unsigned int idx; \
    for ( idx = 0 ; idx < dif; idx++ ) { \
      loopCode2; \
     } \
    } \
   return res; \
  }

///// - VEC2F - /////

#define NE10_DstSrcCst_MAINLOOP_VEC2F_NEON(loopCode) { \
     n_src = vld1q_f32( (float32_t*)src ); /* load two vectors */ \
     src += 2; /* move to the next two vectors */ \
     loopCode; /* actual operation */ /* The main loop iterates through two 2D vectors each time */ \
     vst1q_f32 ( (float32_t*)dst , n_dst ); /* store back */ \
     dst += 2; /* move to the next 2 vectors */ \
    }

#define NE10_DstSrcCst_SECONDLOOP_VEC2F_NEON(loopCode) { \
     float32x2_t n_tmp_src; \
     float32x2_t n_tmp_cst = { cst->x, cst->y }; \
     n_tmp_src = vld1_f32( (float32_t*)src  ); \
     loopCode; /* exceptional cases where the count isn't a multiple of 2 */ \
     vst1_f32( (float32_t*)dst, n_tmp_src); \
    }

#define NE10_DstSrcCst_OPERATION_VEC2F_NEON(checkPointer, loopCode1, loopCode2) { \
   arm_result_t res = NE10_OK; \
   float32x4_t n_cst = { cst->x, cst->y, cst->x, cst->y }; \
   float32x4_t n_src; \
   float32x4_t n_dst; \
   checkPointer; \
   int dif = count % 2; \
   for (; count > dif; count -= 2) { \
    loopCode1; \
   } \
   if ( 0 != dif ) { \
    loopCode2; \
   } \
   return res; \
  }

///// - VEC3F - /////

#define NE10_DstSrcCst_MAINLOOP_VEC3F_NEON(loopCode) { \
     n_src1 = vld1q_f32( (float32_t*)src ); \
     src = ((void*)src)+(4*sizeof(arm_float_t)); \
     n_src2 = vld1q_f32( (float32_t*)src ); \
     src = ((void*)src)+(4*sizeof(arm_float_t)); \
     n_src3 = vld1q_f32( (float32_t*)src ); \
     src = ((void*)src)+(4*sizeof(arm_float_t)); \
     loopCode; /* The main loop iterates through three 3D vectors each time */ \
     vst1q_f32 ( (float32_t*)dst , n_dst1 ); \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
     vst1q_f32 ( (float32_t*)dst , n_dst2 ); \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
     vst1q_f32 ( (float32_t*)dst , n_dst3 ); \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
  }

#define NE10_DstSrcCst_SECONDLOOP_VEC3F_NEON(loopCode) { \
      float32x2x3_t n_tmp_src = FLOAT32_2x3( \
        0.0f, 0.0f, 0.0f , 0.0f, 0.0f , 0.0f); \
      float32x2x3_t n_tmp_cst = FLOAT32_2x3( \
        cst->x, 0, cst->y, 0, cst->z, 0); \
      n_tmp_src = vld3_lane_f32 ( (float32_t*)src, n_tmp_src, 0); \
      loopCode; /* exceptional cases where the count isn't a multiple of 3 */ \
      vst3_lane_f32( (float32_t*)dst, n_tmp_src, 0); \
      src++; \
      dst++; \
     }

#define NE10_DstSrcCst_OPERATION_VEC3F_NEON(checkPointer, loopCode1, loopCode2) { \
   arm_result_t res = NE10_OK; \
   float32x4_t n_cst1 = { cst->x, cst->y, cst->z, cst->x }; \
   float32x4_t n_cst2 = { cst->y, cst->z, cst->x, cst->y }; \
   float32x4_t n_cst3 = { cst->z, cst->x, cst->y, cst->z }; \
    float32x4_t n_src1, n_src2, n_src3; \
   float32x4_t n_dst1, n_dst2, n_dst3; \
   checkPointer; \
   int dif = count % 4;  \
   for (; count > dif; count -= 4) { \
    loopCode1; \
  } \
  if ( 0 != dif ) { \
    unsigned int idx; \
    for ( idx = 0 ; idx < dif; idx++ ) { \
      loopCode2; \
     } \
    } \
   return res; \
  }

///// - VEC4F - /////

/* Note that for the VEC4* types, we do not need a second loop as the number
    of input items is always a multiple of four. */

#define NE10_DstSrcCst_MAINLOOP_VEC4F_NEON(loopCode) { \
     n_src = vld1q_f32( (float32_t*)src ); \
     src ++; \
     loopCode; \
     vst1q_f32 ( (float32_t*)dst , n_dst );  /* The main loop iterates through one 4D vector each time */ \
     dst ++; \
   }

#define NE10_DstSrcCst_OPERATION_VEC4F_NEON(checkPointer, loopCode) { \
   arm_result_t res = NE10_OK; \
   float32x4_t n_cst = { cst->x, cst->y, cst->z, cst->w }; \
   float32x4_t n_src; \
   float32x4_t n_dst; \
   checkPointer; \
   for (; count != 0; count --) { \
     loopCode; \
    } \
   return res; \
  }

/****************************************************
 *                                                  *
 *  The "DstAccSrcCst" group of functions           *
 *                                                  *
 ****************************************************/

///// - FLOAT - /////

#define NE10_DstAccSrcCst_MAINLOOP_FLOAT_NEON(loopCode) { \
     /* load 4 values  */ \
     n_acc = vld1q_f32( (float32_t*)acc ); \
     n_src = vld1q_f32( (float32_t*)src ); \
     acc += 4; /* move to the next 4 float items; 4*float */ \
     src += 4; \
     loopCode; /* the actual operation is placed here... */ /* The main loop iterates through four float values each time */ \
     vst1q_f32 ( (float32_t*)dst , n_dst ); /* store theresults back */ \
     dst += 4; /* move to the next items; 4*float */ \
    }

#define NE10_DstAccSrcCst_SECONDLOOP_FLOAT_NEON(loopCode) { \
      float32x2_t n_tmp_acc = { 0.0f , 0.0f }; /* temporary storage to be used with NEON load/store intrinsics */ \
      float32x2_t n_tmp_src = { 0.0f , 0.0f }; /* temporary storage to be used with NEON load/store intrinsics */ \
      float32x2_t n_tmp_cst = { cst, cst }; /* temporary constant value for use in the main NEON operation */ \
      n_tmp_acc = vld1_lane_f32 ( (float32_t*)acc, n_tmp_acc, 0); /* load into the first lane of d0 */ \
      n_tmp_src = vld1_lane_f32 ( (float32_t*)src, n_tmp_src, 0); /* load into the first lane of d1 */ \
      loopCode; /* the actual operation is palced here ... */ /* exceptional cases where the count is not a multiple of 4 */ \
      vst1_lane_f32( (float32_t*)dst, n_tmp_src, 0); /* store the lane back into the memory */ \
      /* move to the next item in the stream */ \
      acc++; \
      src++; \
      dst++; \
     }

#define NE10_DstAccSrcCst_OPERATION_FLOAT_NEON    NE10_DstSrcCst_OPERATION_FLOAT_NEON

///// - VEC2F - /////

#define NE10_DstAccSrcCst_MAINLOOP_VEC2F_NEON(loopCode) { \
     n_acc = vld1q_f32( (float32_t*)acc ); /* load two vectors */ \
     n_src = vld1q_f32( (float32_t*)src ); /* load two vectors */ \
     acc += 2; /* move to the next two vectors */ \
     src += 2; \
     loopCode; /* actual operation */ /* The main loop iterates through two 2D vectors each time */ \
     vst1q_f32 ( (float32_t*)dst , n_dst ); /* store back */ \
     dst += 2; /* move to the next 2 vectors */ \
    }

#define NE10_DstAccSrcCst_SECONDLOOP_VEC2F_NEON(loopCode) { \
     float32x2_t n_tmp_acc; \
     float32x2_t n_tmp_src; \
     float32x2_t n_tmp_cst = { cst->x, cst->y }; \
     n_tmp_acc = vld1_f32( (float32_t*)acc  ); \
     n_tmp_src = vld1_f32( (float32_t*)src  ); \
     loopCode; /* exceptional cases where the count isn't a multiple of 2 */ \
     vst1_f32( (float32_t*)dst, n_tmp_src); \
    }

#define NE10_DstAccSrcCst_OPERATION_VEC2F_NEON    NE10_DstSrcCst_OPERATION_VEC2F_NEON

///// - VEC3F - /////

#define NE10_DstAccSrcCst_MAINLOOP_VEC3F_NEON(loopCode) { \
     n_acc1 = vld1q_f32( (float32_t*)acc ); /* Load accumulator values */ \
     acc = ((void*)acc)+(4*sizeof(arm_float_t)); \
     n_acc2 = vld1q_f32( (float32_t*)acc ); \
     acc = ((void*)acc)+(4*sizeof(arm_float_t)); \
     n_acc3 = vld1q_f32( (float32_t*)acc ); \
     acc = ((void*)acc)+(4*sizeof(arm_float_t)); \
     n_src1 = vld1q_f32( (float32_t*)src ); /* Load source values */ \
     src = ((void*)src)+(4*sizeof(arm_float_t)); \
     n_src2 = vld1q_f32( (float32_t*)src ); \
     src = ((void*)src)+(4*sizeof(arm_float_t)); \
     n_src3 = vld1q_f32( (float32_t*)src ); \
     src = ((void*)src)+(4*sizeof(arm_float_t)); \
     loopCode; /* The main loop iterates through three 3D vectors each time */ \
     vst1q_f32 ( (float32_t*)dst , n_dst1 ); /* Store the results back into the memory */ \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
     vst1q_f32 ( (float32_t*)dst , n_dst2 ); \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
     vst1q_f32 ( (float32_t*)dst , n_dst3 ); \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
  }

#define NE10_DstAccSrcCst_SECONDLOOP_VEC3F_NEON(loopCode) { \
      float32x2x3_t n_tmp_acc = FLOAT32_2x3( \
         0.0f, 0.0f, \
         0.0f, 0.0f, \
         0.0f, 0.0f  \
      ); \
      float32x2x3_t n_tmp_src = FLOAT32_2x3( \
        0.0f, 0.0f, \
        0.0f, 0.0f, \
        0.0f, 0.0f  \
      ); \
      float32x2x3_t n_tmp_cst = FLOAT32_2x3( \
         cst->x, 0, \
         cst->y, 0, \
         cst->z, 0 \
      ); \
      n_tmp_acc = vld3_lane_f32 ( (float32_t*)acc, n_tmp_acc, 0); \
      n_tmp_src = vld3_lane_f32 ( (float32_t*)src, n_tmp_src, 0); \
      loopCode; /* exceptional cases where the count isn't a multiple of 3 */ \
      vst3_lane_f32( (float32_t*)dst, n_tmp_src, 0); \
      acc++; \
      src++; \
      dst++; \
  }

#define NE10_DstAccSrcCst_OPERATION_VEC3F_NEON    NE10_DstSrcCst_OPERATION_VEC3F_NEON

///// - VEC4F - /////

#define NE10_DstAccSrcCst_MAINLOOP_VEC4F_NEON(loopCode) { \
     n_acc = vld1q_f32( (float32_t*)acc ); \
     n_src = vld1q_f32( (float32_t*)src ); \
     acc ++; \
     src ++; \
     loopCode; \
     vst1q_f32 ( (float32_t*)dst , n_dst );  /* The main loop iterates through one 4D vector each time */ \
     dst ++; \
  }

#define NE10_DstAccSrcCst_OPERATION_VEC4F_NEON    NE10_DstSrcCst_OPERATION_VEC4F_NEON

/****************************************************
 *                                                  *
 *  The "DstCst" group of functions                 *
 *                                                  *
 ****************************************************/

///// - FLOAT - /////

#define NE10_DstCst_MAINLOOP_FLOAT_NEON(loopCode) { \
     /* load 4 values  */ \
     loopCode; /* the actual operation is placed here... */ /* The main loop iterates through four float values each time */ \
     vst1q_f32 ( (float32_t*)dst , n_cst ); /* store theresults back */ \
     dst += 4; /* move to the next items; 4*float */ \
    }

#define NE10_DstCst_SECONDLOOP_FLOAT_NEON(loopCode) { \
      float32x2_t n_tmp_cst = { cst, cst }; /* temporary constant value for use in the main NEON operation */ \
      loopCode; /* the actual operation is palced here ... */ /* exceptional cases where the count is not a multiple of 4 */ \
      vst1_lane_f32( (float32_t*)dst, n_tmp_cst, 0); /* store the lane back into the memory */ \
      /* move to the next item in the stream */ \
      dst++; \
     }

#define NE10_DstCst_OPERATION_FLOAT_NEON(checkPointer, loopCode1, loopCode2) { \
   arm_result_t res = NE10_OK; \
   checkPointer; \
   int dif = 0; \
   dif = count % 4; /* either 0 or one of 1,2,3; in the latter cases the second path is taken */ \
   for (; count > dif; count -= 4) { \
     loopCode1; \
    } \
   if ( 0 != dif ) { \
    unsigned int idx; \
    for ( idx = 0 ; idx < dif; idx++ ) { \
      loopCode2; \
     } \
    } \
   return res; \
  }

///// - VEC2F - /////


#define NE10_DstCst_MAINLOOP_VEC2F_NEON(loopCode) { \
     loopCode; /* actual operation */ /* The main loop iterates through two 2D vectors each time */ \
     vst1q_f32 ( (float32_t*)dst , n_cst ); /* store back */ \
     dst += 2; /* move to the next 2 vectors */ \
    }

#define NE10_DstCst_SECONDLOOP_VEC2F_NEON(loopCode) { \
     float32x2_t n_tmp_cst = { cst->x, cst->y }; \
     loopCode; /* exceptional cases where the count isn't a multiple of 2 */ \
     vst1_f32( (float32_t*)dst, n_tmp_cst); \
    }

#define NE10_DstCst_OPERATION_VEC2F_NEON(checkPointer, loopCode1, loopCode2) { \
   arm_result_t res = NE10_OK; \
   float32x4_t n_cst = { cst->x, cst->y, cst->x, cst->y }; \
   checkPointer; \
   int dif = count % 2; \
   for (; count > dif; count -= 2) { \
    loopCode1; \
   } \
   if ( 0 != dif ) { \
    loopCode2; \
   } \
   return res; \
  }

///// - VEC3F - /////

#define NE10_DstCst_MAINLOOP_VEC3F_NEON(loopCode) { \
     loopCode; /* The main loop iterates through three 3D vectors each time */ \
     vst1q_f32 ( (float32_t*)dst , n_cst1 ); \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
     vst1q_f32 ( (float32_t*)dst , n_cst2 ); \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
     vst1q_f32 ( (float32_t*)dst , n_cst3 ); \
     dst = ((void*)dst)+(4*sizeof(arm_float_t)); \
  }

#define NE10_DstCst_SECONDLOOP_VEC3F_NEON(loopCode) { \
      float32x2x3_t n_tmp_cst = FLOAT32_2x3( \
        cst->x, 0, \
        cst->y, 0, \
        cst->z, 0 \
      ); \
      loopCode; /* exceptional cases where the count isn't a multiple of 3 */ \
      vst3_lane_f32( (float32_t*)dst, n_tmp_cst, 0); \
      dst++; \
     }

#define NE10_DstCst_OPERATION_VEC3F_NEON(checkPointer, loopCode1, loopCode2) { \
   arm_result_t res = NE10_OK; \
   float32x4_t n_cst1 = { cst->x, cst->y, cst->z, cst->x }; \
   float32x4_t n_cst2 = { cst->y, cst->z, cst->x, cst->y }; \
   float32x4_t n_cst3 = { cst->z, cst->x, cst->y, cst->z }; \
   checkPointer; \
   int dif = count % 4;  \
   for (; count > dif; count -= 4) { \
    loopCode1; \
  } \
  if ( 0 != dif ) { \
    unsigned int idx; \
    for ( idx = 0 ; idx < dif; idx++ ) { \
      loopCode2; \
     } \
    } \
   return res; \
  }

///// - VEC4F - /////

#define NE10_DstCst_MAINLOOP_VEC4F_NEON(loopCode) { \
     loopCode; \
     vst1q_f32 ( (float32_t*)dst , n_cst );  /* The main loop iterates through one 4D vector each time */ \
     dst ++; \
   }

#define NE10_DstCst_OPERATION_VEC4F_NEON(checkPointer, loopCode) { \
   arm_result_t res = NE10_OK; \
   float32x4_t n_cst = { cst->x, cst->y, cst->z, cst->w }; \
   checkPointer; \
   for (; count != 0; count --) { \
     loopCode; \
    } \
   return res; \
  }

/****************************************************
 *                                                  *
 *  The "DstSrc1Src2" group of functions            *
 *                                                  *
 ****************************************************/

///// - FLOAT - /////

#define NE10_DstSrc1Src2_MAINLOOP_FLOAT_NEON(loopCode) { \
     /* load 4 values  */ \
     n_src = vld1q_f32( (float32_t*)src1 ); \
     src1 += 4; /* move to the next 4 float items; 4*float */ \
     n_src2 = vld1q_f32( (float32_t*)src2 ); \
     src2 += 4; /* move to the next 4 float items; 4*float */ \
     loopCode; /* the actual operation is placed here... */ /* The main loop iterates through four float values each time */ \
     vst1q_f32 ( (float32_t*)dst , n_dst ); /* store the results back */ \
     dst += 4; /* move to the next items; 4*float */ \
    }

#define NE10_DstSrc1Src2_SECONDLOOP_FLOAT_NEON(loopCode) { \
      float32x2_t n_tmp_src = { 0.0f , 0.0f }; /* temporary storage to be used with NEON load/store intrinsics */ \
      float32x2_t n_tmp_src2 = { 0.0f , 0.0f }; \
      n_tmp_src = vld1_lane_f32 ( (float32_t*)src1, n_tmp_src, 0); /* load into the first lane of d0 */ \
      n_tmp_src2 = vld1_lane_f32 ( (float32_t*)src2, n_tmp_src, 0); \
      loopCode; /* the actual operation is placed here ... */ /* exceptional cases where the count is not a multiple of 4 */ \
      vst1_lane_f32( (float32_t*)dst, n_tmp_src, 0); /* store the lane back into the memory */ \
      /* move to the next item in the stream */ \
      src1++; \
      src2++; \
      dst++; \
     }

#define NE10_DstSrc1Src2_OPERATION_FLOAT_NEON NE10_DstSrcCst_OPERATION_FLOAT_NEON

/****************************************************
 *                                                  *
 *  The "DstAccSrc1Src2" group of functions         *
 *                                                  *
 ****************************************************/

///// - FLOAT - /////

#define NE10_DstAccSrc1Src2_MAINLOOP_FLOAT_NEON(loopCode) { \
     /* load 4 values  */ \
     n_acc = vld1q_f32( (float32_t*)acc ); \
     n_src = vld1q_f32( (float32_t*)src1 ); \
     n_src2 = vld1q_f32( (float32_t*)src2 ); \
     acc += 4; /* move to the next 4 float items; 4*float */ \
     src1 += 4; \
     src2 += 4; \
     loopCode; /* the actual operation is placed here... */ /* The main loop iterates through four float values each time */ \
     vst1q_f32 ( (float32_t*)dst , n_dst ); /* store theresults back */ \
     dst += 4; /* move to the next items; 4*float */ \
    }

#define NE10_DstAccSrc1Src2_SECONDLOOP_FLOAT_NEON(loopCode) { \
      float32x2_t n_tmp_acc = { 0.0f , 0.0f }; /* temporary storage to be used with NEON load/store intrinsics */ \
      float32x2_t n_tmp_src = { 0.0f , 0.0f }; \
      float32x2_t n_tmp_src2 = { 0.0f, 0.0f }; \
      n_tmp_acc = vld1_lane_f32 ( (float32_t*)acc, n_tmp_acc, 0); /* load into the first lane of d0 */ \
      n_tmp_src = vld1_lane_f32 ( (float32_t*)src1, n_tmp_src, 0); /* load into the first lane of d1 */ \
      n_tmp_src2 = vld1_lane_f32 ( (float32_t*)src2, n_tmp_src2, 0); /* load into the first lane of d2 */ \
      loopCode; /* the actual operation is palced here ... */ /* exceptional cases where the count is not a multiple of 4 */ \
      vst1_lane_f32( (float32_t*)dst, n_tmp_src, 0); /* store the lane back into the memory */ \
      /* move to the next item in the stream */ \
      acc++; \
      src1++; \
      src2++; \
      dst++; \
     }

#define NE10_DstAccSrc1Src2_OPERATION_FLOAT_NEON NE10_DstAccSrcCst_OPERATION_FLOAT_NEON

/****************************************************
 *                                                  *
 *  The "DstSrc" group of functions                 *
 *                                                  *
 ****************************************************/

///// - FLOAT - /////

#define NE10_DstSrc_MAINLOOP_FLOAT_NEON NE10_DstSrcCst_MAINLOOP_FLOAT_NEON

#define NE10_DstSrc_SECONDLOOP_FLOAT_NEON NE10_DstSrcCst_SECONDLOOP_FLOAT_NEON

#define NE10_DstSrc_OPERATION_FLOAT_NEON NE10_DstSrcCst_OPERATION_FLOAT_NEON

///// - VEC2F - /////

#define NE10_DstSrc_MAINLOOP_VEC2F_NEON(loopCode) { \
     n_src = vld2_f32( (float32_t*)src ); /* load two vectors */ \
     src += 2; /* move to the next two vectors */ \
     loopCode; /* actual operation */ /* The main loop iterates through two 2D vectors each time */ \
     /* store the results and increment the destination pointer within the loopCode */ \
    }

#define NE10_DstSrc_SECONDLOOP_VEC2F_NEON(loopCode) { \
     loopCode; /* exceptional cases where the count isn't a multiple of 2 */ \
     /* store the results within the loopCode */ \
    }

#define NE10_DstSrc_OPERATION_VEC2F_NEON(checkPointer, loopCode1, loopCode2) { \
   arm_result_t res = NE10_OK; \
   float32x2x2_t n_src; \
   float32x2_t n_dst; \
   checkPointer; \
   int dif = count % 2; \
   for (; count > dif; count -= 2) { \
    loopCode1; \
   } \
   if ( 0 != dif ) { \
    loopCode2; \
   } \
   return res; \
  }

///// - VEC3F - /////

#define NE10_DstSrc_MAINLOOP_VEC3F_NEON(loopCode) { \
     n_src = vld3q_f32( (float32_t*)src ); \
     src = ((void*)src)+(12*sizeof(arm_float_t)); \
     loopCode; /* The main loop iterates through four 3D vectors each time */ \
     /* store the results and increment the destination pointer within the loopCode */ \
  }

#define NE10_DstSrc_SECONDLOOP_VEC3F_NEON(loopCode) { \
      loopCode; /* exceptional cases where the count isn't a multiple of 4 */ \
      /* store the results within the loopCode */ \
     }

#define NE10_DstSrc_OPERATION_VEC3F_NEON(checkPointer, loopCode1, loopCode2) { \
   arm_result_t res = NE10_OK; \
   float32x4x3_t n_src; \
   float32x4_t n_dst; \
   checkPointer; \
   int dif = count % 4; \
   for (; count > dif; count -= 4) { \
    loopCode1; \
   } \
   if ( 0 != dif ) { \
     unsigned int idx; \
     for ( idx = 0 ; idx < dif; idx++ ) { \
       loopCode2; \
      } \
     } \
    return res; \
   }

///// - VEC4F - /////

/* Note that for the VEC4* types, we do not need a second loop as the number
    of input items is always a multiple of four. */

#define NE10_DstSrc_MAINLOOP_VEC4F_NEON(loopCode) { \
     n_src = vld1q_f32( (float32_t*)src ); \
     src ++; \
     loopCode; \
     /* store the results and increment the destination pointer within the loopCode */ \
   }

#define NE10_DstSrc_OPERATION_VEC4F_NEON(checkPointer, loopCode) { \
   arm_result_t res = NE10_OK; \
   float32x4_t n_src; \
   checkPointer; \
   for (; count != 0; count --) { \
     loopCode; \
    } \
   return res; \
  }


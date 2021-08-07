/*
 * Software License Agreement (Simplified BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 * Copyright (c) 2012, Piotr Dollar & Ron Appel.[pdollar-at-caltech.edu]
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *this list of conditions and the following disclaimer in the documentation
 *and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 *
 * Taken from Piotr Dollar's MATLAB Image&Video ToolboxVersion 3.00.
 *
 */

#pragma once

#if defined(__SSE2__)
#include <emmintrin.h> // SSE2:<e*.h>, SSE3:<p*.h>, SSE4:<s*.h>

#define RETf inline __m128
#define RETi inline __m128i

namespace pcl {

// set, load and store values
RETf sse_set( const float &x ) { return _mm_set1_ps(x); }
RETf sse_set( float x, float y, float z, float w ) { return _mm_set_ps(x,y,z,w); }
RETi sse_set( const int &x ) { return _mm_set1_epi32(x); }
RETf sse_ld( const float &x ) { return _mm_load_ps(&x); }
RETf sse_ldu( const float &x ) { return _mm_loadu_ps(&x); }
RETf sse_str( float &x, const __m128 y ) { _mm_store_ps(&x,y); return y; }
RETf sse_str1( float &x, const __m128 y ) { _mm_store_ss(&x,y); return y; }
RETf sse_stru( float &x, const __m128 y ) { _mm_storeu_ps(&x,y); return y; }
RETf sse_str( float &x, const float y ) { return sse_str(x,sse_set(y)); }

// arithmetic operators
RETi sse_add( const __m128i x, const __m128i y ) { return _mm_add_epi32(x,y); }
RETf sse_add( const __m128 x, const __m128 y ) { return _mm_add_ps(x,y); }
RETf sse_add( const __m128 x, const __m128 y, const __m128 z ) {
  return sse_add(sse_add(x,y),z); }
RETf sse_add( const __m128 a, const __m128 b, const __m128 c, const __m128 &d ) {
  return sse_add(sse_add(sse_add(a,b),c),d); }
RETf sse_sub( const __m128 x, const __m128 y ) { return _mm_sub_ps(x,y); }
RETf sse_mul( const __m128 x, const __m128 y ) { return _mm_mul_ps(x,y); }
RETf sse_mul( const __m128 x, const float y ) { return sse_mul(x,sse_set(y)); }
RETf sse_mul( const float x, const __m128 y ) { return sse_mul(sse_set(x),y); }
RETf sse_inc( __m128 &x, const __m128 y ) { return x = sse_add(x,y); }
RETf sse_inc( float &x, const __m128 y ) { __m128 t=sse_add(sse_ld(x),y); return sse_str(x,t); }
RETf sse_dec( __m128 &x, const __m128 y ) { return x = sse_sub(x,y); }
RETf sse_dec( float &x, const __m128 y ) { __m128 t=sse_sub(sse_ld(x),y); return sse_str(x,t); }
RETf sse_min( const __m128 x, const __m128 y ) { return _mm_min_ps(x,y); }
RETf sse_rcp( const __m128 x ) { return _mm_rcp_ps(x); }
RETf sse_rcpsqrt( const __m128 x ) { return _mm_rsqrt_ps(x); }

// logical operators
RETf sse_and( const __m128 x, const __m128 y ) { return _mm_and_ps(x,y); }
RETi sse_and( const __m128i x, const __m128i y ) { return _mm_and_si128(x,y); }
RETf sse_andnot( const __m128 x, const __m128 y ) { return _mm_andnot_ps(x,y); }
RETf sse_or( const __m128 x, const __m128 y ) { return _mm_or_ps(x,y); }
RETf sse_xor( const __m128 x, const __m128 y ) { return _mm_xor_ps(x,y); }

// comparison operators
RETf sse_cmpgt( const __m128 x, const __m128 y ) { return _mm_cmpgt_ps(x,y); }
RETi sse_cmpgt( const __m128i x, const __m128i y ) { return _mm_cmpgt_epi32(x,y); }

// conversion operators
RETf sse_cvt( const __m128i x ) { return _mm_cvtepi32_ps(x); }
RETi sse_cvt( const __m128 x ) { return _mm_cvttps_epi32(x); }

} // namespace pcl

#undef RETf
#undef RETi
#endif /* defined(__SSE2__) */

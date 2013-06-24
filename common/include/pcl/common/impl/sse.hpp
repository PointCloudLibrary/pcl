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

#ifndef PCL_COMMON_SSE_HPP_
#define PCL_COMMON_SSE_HPP_
#if defined(__SSE2__)
#include <emmintrin.h> // SSE2:<e*.h>, SSE3:<p*.h>, SSE4:<s*.h>

#define RETf inline __m128
#define RETi inline __m128i

// set, load and store values
RETf SET( const float &x ) { return _mm_set1_ps(x); }
RETf SET( float x, float y, float z, float w ) { return _mm_set_ps(x,y,z,w); }
RETi SET( const int &x ) { return _mm_set1_epi32(x); }
RETf LD( const float &x ) { return _mm_load_ps(&x); }
RETf LDu( const float &x ) { return _mm_loadu_ps(&x); }
RETf STR( float &x, const __m128 y ) { _mm_store_ps(&x,y); return y; }
RETf STR1( float &x, const __m128 y ) { _mm_store_ss(&x,y); return y; }
RETf STRu( float &x, const __m128 y ) { _mm_storeu_ps(&x,y); return y; }
RETf STR( float &x, const float y ) { return STR(x,SET(y)); }

// arithmetic operators
RETi ADD( const __m128i x, const __m128i y ) { return _mm_add_epi32(x,y); }
RETf ADD( const __m128 x, const __m128 y ) { return _mm_add_ps(x,y); }
RETf ADD( const __m128 x, const __m128 y, const __m128 z ) {
  return ADD(ADD(x,y),z); }
RETf ADD( const __m128 a, const __m128 b, const __m128 c, const __m128 &d ) {
  return ADD(ADD(ADD(a,b),c),d); }
RETf SUB( const __m128 x, const __m128 y ) { return _mm_sub_ps(x,y); }
RETf MUL( const __m128 x, const __m128 y ) { return _mm_mul_ps(x,y); }
RETf MUL( const __m128 x, const float y ) { return MUL(x,SET(y)); }
RETf MUL( const float x, const __m128 y ) { return MUL(SET(x),y); }
RETf INC( __m128 &x, const __m128 y ) { return x = ADD(x,y); }
RETf INC( float &x, const __m128 y ) { __m128 t=ADD(LD(x),y); return STR(x,t); }
RETf DEC( __m128 &x, const __m128 y ) { return x = SUB(x,y); }
RETf DEC( float &x, const __m128 y ) { __m128 t=SUB(LD(x),y); return STR(x,t); }
RETf MIN( const __m128 x, const __m128 y ) { return _mm_min_ps(x,y); }
RETf RCP( const __m128 x ) { return _mm_rcp_ps(x); }
RETf RCPSQRT( const __m128 x ) { return _mm_rsqrt_ps(x); }

// logical operators
RETf AND( const __m128 x, const __m128 y ) { return _mm_and_ps(x,y); }
RETi AND( const __m128i x, const __m128i y ) { return _mm_and_si128(x,y); }
RETf ANDNOT( const __m128 x, const __m128 y ) { return _mm_andnot_ps(x,y); }
RETf OR( const __m128 x, const __m128 y ) { return _mm_or_ps(x,y); }
RETf XOR( const __m128 x, const __m128 y ) { return _mm_xor_ps(x,y); }

// comparison operators
RETf CMPGT( const __m128 x, const __m128 y ) { return _mm_cmpgt_ps(x,y); }
RETi CMPGT( const __m128i x, const __m128i y ) { return _mm_cmpgt_epi32(x,y); }

// conversion operators
RETf CVT( const __m128i x ) { return _mm_cvtepi32_ps(x); }
RETi CVT( const __m128 x ) { return _mm_cvttps_epi32(x); }

#undef RETf
#undef RETi
#else
PCL_ERROR("SSE2 instructions not supported");
#endif /* defined(__SSE2__) && !defined(__i386__) */
#endif /* PCL_COMMON_SSE_HPP_ */

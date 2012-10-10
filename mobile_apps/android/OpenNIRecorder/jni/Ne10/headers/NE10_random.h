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
 * NE10 Library : headers/NE10_random.h
 */


#ifndef NE10_RANDOM
#define NE10_RANDOM

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

// Please look at http://en.wikipedia.org/wiki/Linear_congruential_generator
// According to this page, these values are the ones used in "glibc"

  //uint32_t _M                 = 4294967296L; // 2^32 // modulus, must be 0 < _M
  const uint32_t _A     = 1103515245L; // a, must be 0 < _A < _M
  const uint32_t _C     = 12345L; // c, must be 0 < _C < _M
  // uint32_t m_X_NM1           = 0; // X(n-1), at first this value is the seed or the start value

// used for creating different instances of random number generators with different seeds and states
typedef struct
{
  // these are used as internal values, please do not change them directly
  uint32_t _private_m_A              ;// = 1103515245L; // a, must be 0 < _A < _M
  uint32_t _private_m_C              ;// = 12345L; // c, must be 0 < _C < _M
  uint32_t _private_m_X_NM1          ;// = 0; // X(n-1), at first this value is the seed or the start value
} NE10_rng_t;

typedef struct
{
  // these are used as internal values, please do not change them directly
  // there are three separate seeds for 1) the sign, 2) the exponent, 3) and the fraction bits.
  NE10_rng_t _private_m_rngs[3];
} NE10_float_rng_t;


// generic functions
void NE10_rng_init_g(NE10_rng_t *rng, uint32_t seed)
{
  assert( rng != NULL );
  rng->_private_m_A = _A;
  rng->_private_m_C = _C;
  rng->_private_m_X_NM1 = seed;
}

uint32_t NE10_rng_next_g(NE10_rng_t *rng)
{
  assert( rng != NULL );
  // Linear Congruential Generator
  rng->_private_m_X_NM1 = ( rng->_private_m_A * rng->_private_m_X_NM1 + rng->_private_m_C ); // % _M; // excluded by the nature of using a 32-bit data type
  return rng->_private_m_X_NM1;
}

const uint32_t NE10_rng_max_g(NE10_rng_t *rng)
{
  return 0xffffffff; // this is 2^32 - 1
}



// the same functions using a rng which is shared across the library
static NE10_rng_t __NE10_rng; // used as the global random number generator shared across the library

void NE10_rng_init(uint32_t seed)
{
   NE10_rng_init_g( &__NE10_rng, seed );
}

uint32_t NE10_rng_next()
{
   return NE10_rng_next_g( &__NE10_rng );
}

const uint32_t NE10_rng_max()
{
  return NE10_rng_max_g(NULL);
}



// a random number generator that generates IEEE 754 float numbers

// NAN_OR_INF is to check whether the value is a NAN or an INF
#define NAN_OR_INF (0xFF << 23)
#define IS_NAN_OR_INF(x) ( ((x & NAN_OR_INF) == NAN_OR_INF)?1:0 )

#define EXPONENT_MASK 0x807FFFFF
#define IS_SUBNORMAL(x) ( ((x & EXPONENT_MASK) == x)?1:0 )

void NE10_float_rng_init_g(NE10_float_rng_t* float_rng, uint32_t seed)
{
   // we can use [0] for the fraction, [1] for the exponent, and [2] for the sign bit

   NE10_rng_t seed_generator;
   NE10_rng_init_g( &seed_generator, seed );

   NE10_rng_init_g( &float_rng->_private_m_rngs[0], NE10_rng_next_g( &seed_generator ) );
   NE10_rng_init_g( &float_rng->_private_m_rngs[1], NE10_rng_next_g( &seed_generator ) );
   NE10_rng_init_g( &float_rng->_private_m_rngs[2], NE10_rng_next_g( &seed_generator ) );
}

float NE10_float_rng_next_g(NE10_float_rng_t* float_rng)
{
   uint32_t frc, exp, sgn, ret;
   float __ret;

   do
   {
      // generate three random numbers
      frc = NE10_rng_next_g( &float_rng->_private_m_rngs[0] );
      exp = NE10_rng_next_g( &float_rng->_private_m_rngs[1] );
      sgn = NE10_rng_next_g( &float_rng->_private_m_rngs[2] );

      // take the top bits ( the sign uses the 17th bit)
      frc =   ( frc >> 9  ) & 0x7FFFFF        ; // (1)b^23
      exp = ( ( exp >> 24 ) & 0x0000FF ) << 23; // (1)b^ 8
      sgn = ( ( sgn >> 16 ) & 0x000001 ) << 31;

      // generate the final float value
      ret = frc | exp | sgn;

   } while ( IS_NAN_OR_INF(ret) || IS_SUBNORMAL(ret) );

   memcpy( &__ret, &ret, 1*sizeof(float) );
   return __ret;
}

float NE10_float_rng_max_g(NE10_float_rng_t* float_rng)
{
  return FLT_MAX;
}


// the same functions using a float_rng which is shared across the library

static NE10_float_rng_t __NE10_float_rng; // local array for internal use only

void NE10_float_rng_init(uint32_t seed)
{
  NE10_float_rng_init_g( &__NE10_float_rng , seed );
}

float NE10_float_rng_next()
{
   return NE10_float_rng_next_g( &__NE10_float_rng );
}

float NE10_float_rng_max()
{
  return NE10_float_rng_max_g(NULL);
}

// the same as above functions except the range of values are limited

#define IS_TOO_SMALL(f) ((fabs(f)<1.0e-6)?1:0)
#define   IS_TOO_BIG(f) ((fabs(f)>1.0e12)?1:0)

static NE10_float_rng_t __NE10_float_rng_limit; // local array for internal use only

void NE10_float_rng_limit_init(uint32_t seed)
{
   NE10_float_rng_init_g( &__NE10_float_rng_limit , seed );
}

float NE10_float_rng_limit_next()
{
   float ret = 0.0f;

   do
   {
      ret = NE10_float_rng_next_g( &__NE10_float_rng_limit );
   } while ( IS_TOO_SMALL(ret) || IS_TOO_BIG(ret) );

   return ret;
}

float NE10_float_rng_limit_max()
{
  return NE10_float_rng_max_g(NULL);
}

// the same as above functions except the range of values are limited and all the values are greater than 1.0e-6

#define IS_TOO_SMALL_GT1(f) ((fabs(f)<1.0e-6)?1:0)
#define   IS_TOO_BIG_GT1(f) ((fabs(f)>1.0e+3)?1:0)

static NE10_float_rng_t __NE10_float_rng_limit_gt1; // local array for internal use only

void NE10_float_rng_limit_gt1_init(uint32_t seed)
{
   NE10_float_rng_init_g( &__NE10_float_rng_limit , seed );
}

float NE10_float_rng_limit_gt1_next()
{
   float ret = 0.0f;

   do
   {
      ret = NE10_float_rng_next_g( &__NE10_float_rng_limit );
   } while ( IS_TOO_SMALL_GT1(ret) || IS_TOO_BIG_GT1(ret) );

   return ret;
}

float NE10_float_rng_limit_gt1_max()
{
  return NE10_float_rng_max_g(NULL);
}

#endif // NE10_RANDOM


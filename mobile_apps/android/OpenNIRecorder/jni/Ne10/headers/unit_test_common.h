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
 * NE10 Library : headers/unit_test_common.h
 */

#ifndef __UNIT_TEST_COMMON__
#define __UNIT_TEST_COMMON__

// Make sure the following values are defined before including this header file:
// 1- length of the data arrays
//     #define ARRLEN
// 2- number of the operations in a given unit
//     #define OP_COUNT
// 3- number of the different implementations of each of the functions (C, ASM, NEON, ...)
//     #define IMPL_COUNT
#ifndef ARRLEN
 #error Pelease define ARRLEN
#endif
#ifndef OP_COUNT
 #error Please define OP_COUNT
#endif
#ifndef IMPL_COUNT
 #error Please define IMPL_COUNT
#endif


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../headers/macros.h"
#include "NE10.h"
#include "../headers/NE10_random.h"

// length of the test data arrays
// A number that is not divisible by 2 3 and 4 so that all the
//  execution paths are tested; The larger the number the more
//  number of random values are stored in the array and passed
//  into the array as the input stream.
// 2^11 + 3 = 2051, it is not divisible by 2, 3, or 4
#define TEST_ARRLEN          2051
#define TEST_ARRLEN_MATRICES 1051

// NAN_OR_INF is to check whether the value is a NAN or an INF
#define NAN_OR_INF (0xFF << 23)
// The sign bit mask
#define SIGNBIT_MASK  0x7FFFFFFF
#define EXPONENT_MASK 0x807FFFFF

// What's the acceptable error between the integer representations of two float values
#define ERROR_MARGIN_SMALL 0x02
#define ERROR_MARGIN_LARGE 0xFF

// What's the acceptable number of warnings in a test
#define ACCEPTABLE_WARNS 12
#define ACCEPTABLE_WARNS_MATRICES 48

inline void FILL_FLOAT_ARRAY( arm_float_t *arr, unsigned int count )
{
    unsigned int i = 0;

    sleep ( 1 );

    NE10_float_rng_init( time(NULL) );

    for ( i = 0; i < count; i++ )
    {
      arr[i] = NE10_float_rng_next();
    }
}

inline void FILL_FLOAT_ARRAY_LIMIT( arm_float_t *arr, unsigned int count )
{
    unsigned int i = 0;

    sleep ( 1 );

    NE10_float_rng_limit_init( time(NULL) );

    for ( i = 0; i < count; i++ )
    {
        arr[ i ] = NE10_float_rng_limit_next();
    }
}

inline void FILL_FLOAT_ARRAY_LIMIT_GT1( arm_float_t *arr, unsigned int count )
{
    unsigned int i = 0;

    sleep ( 1 );

    NE10_float_rng_limit_gt1_init( time(NULL) );

    for ( i = 0; i < count; i++ )
    {
        arr[ i ] = NE10_float_rng_limit_gt1_next();
    }
}

// this function checks whether the difference between two float values is within the acceptable error range
inline int EQUALS_FLOAT( float fa, float fb , unsigned int err )
{
  union
  {
    int          vi;
    float        vf;
  } conv1, conv2;

  unsigned int ui1, ui2;

  if ( fa == fb ) return 1; // if identical, then return TRUE

  conv1.vf = fa;
  conv2.vf = fb;

  if ( (conv1.vi & NAN_OR_INF) == NAN_OR_INF )
  {
     fprintf( stderr, "HINT: The 1st floating-point value is either \'Not a number\' or \'Infinity\'. " );
     return 0; // INF or NAN, unacceptable return FALSE
  }

  if ( (conv2.vi & NAN_OR_INF) == NAN_OR_INF )
  {
     fprintf( stderr, "HINT: The 1st floating-point value is either \'Not a number\' or \'Infinity\'. " );
     return 0; // INF or NAN, unacceptable return FALSE
  }

  int cut1 = conv1.vi & SIGNBIT_MASK; // drop the sign bit - i.e. the left most bit
  int cut2 = conv2.vi & SIGNBIT_MASK;

  if ( (cut1 & EXPONENT_MASK) == cut1 ) { cut1 = 0; } // zero out subnormal float values
  if ( (cut2 & EXPONENT_MASK) == cut2 ) { cut2 = 0; } // zero out subnormal float values

  memcpy( &ui1,  &fa, sizeof(arm_float_t) );
  memcpy( &ui2,  &fb, sizeof(arm_float_t) );

  if ( abs( cut1 - cut2 ) > err ) // this is the log() of the actual error
  {  // then we have an unacceptable error

     // report an unacceptable error
     fprintf( stderr, "HINT: %e (0x%04X) != %e (0x%04X) ", fa, ui1, fb, ui2 );

     return 0;
  }

  if ( fb*fa < 0.0f )
  {

     fprintf( stderr, "HINT: %e (0x%04X) is the opposite of %e (0x%04X) ", fa, ui1, fb, ui2 );

     return 0;
  }

  return 1; // acceptable, return TRUE
}

char ARRAY_GUARD_SIG[] = {      0x66, 0xAB, 0xCD, 0xAB,
                                0xCD, 0xAB, 0xCD, 0xAB,
                                0xCD, 0xAB, 0xCD, 0xAB,
                                0xCD, 0xAB, 0xCD, 0x99 };
#define ARRAY_GUARD_LEN 16

// this function adds a ARRAY_GUARD_LEN byte signature to the begining and the end of an array, minimum acceptable size for the array is 2*ARRAY_GUARD_LEN bytes.
inline int GUARD_ARRAY( void* array, unsigned int array_length )
{
  char* the_array = (char*) array;
  if ( array_length < (2*ARRAY_GUARD_LEN) ) return 0;
  memcpy( the_array, ARRAY_GUARD_SIG, ARRAY_GUARD_LEN );
  memcpy( &the_array[array_length-ARRAY_GUARD_LEN], ARRAY_GUARD_SIG, ARRAY_GUARD_LEN );
  return 1;
}

// this function returns TRUE if the signature matches the guard and returns FALSE otherwise
inline int CHECK_ARRAY_GUARD( void* array, unsigned int array_length )
{
  char* the_array = (char*) array;
  if ( strncmp(the_array, ARRAY_GUARD_SIG, ARRAY_GUARD_LEN) ) {
      fprintf( stderr, " ERROR: Array guard signature is wrong. \n" );
      return 0; // Match not found, return FALSE
  }

  if ( strncmp(&the_array[array_length-ARRAY_GUARD_LEN], ARRAY_GUARD_SIG, ARRAY_GUARD_LEN)  ) {
      fprintf( stderr, " ERROR: Array guard signature is wrong. \n" );
      return 0; // Match not found, return FALSE
  }

  return 1;
}

#endif // __UNIT_TEST_COMMON


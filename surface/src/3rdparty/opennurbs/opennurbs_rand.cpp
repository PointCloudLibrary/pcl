/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2012 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////
*/

#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

// This source code is from 
// http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/MT2002/emt19937ar.html
// and its copyright and license are reproduced below.
// It is included in opennurbs because we need a thread safe and
// platform independent way to get a decent 32 bit random number.

/* 
   A C-program for MT19937, with initialization improved 2002/1/26.
   Coded by Takuji Nishimura and Makoto Matsumoto.

   Before using, initialize the state by using init_genrand(seed)  
   or init_by_array(init_key, key_length).

   Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
   All rights reserved.                          

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

     3. The names of its contributors may not be used to endorse or promote 
        products derived from this software without specific prior written 
        permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


   Any feedback is very welcome.
   http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/emt.html
   email: m-mat @ math.sci.hiroshima-u.ac.jp (remove space)
*/

/* Period parameters */  
#define N 624 /* If you change the value of N, update the length of ON_RANDOM_NUMBER_CONTEXT m_t[] to match. */
#define M 397
#define MATRIX_A 0x9908b0dfUL   /* constant vector a */
#define UPPER_MASK 0x80000000UL /* most significant w-r bits */
#define LOWER_MASK 0x7fffffffUL /* least significant r bits */




//static unsigned long mt[N]; /* the array for the state vector  */
//static int mti=N+1; /* mti==N+1 means mt[N] is not initialized */

/* initializes mt[N] with a seed */
void on_random_number_seed(ON__UINT32 s,ON_RANDOM_NUMBER_CONTEXT* randcontext)
{
  ON__UINT32 i, u;

#if defined(ON_COMPILER_MSC)
#pragma warning( push )
#pragma warning( disable : 4127 ) // warning C4127: conditional expression is constant
#endif
  if ( N*sizeof(randcontext->mt[0]) != sizeof(randcontext->mt) )
  {
    ON_ERROR("the mt[] array in struct ON_RANDOM_NUMBER_CONTEXT must have length N.");
  }
#if defined(ON_COMPILER_MSC)
#pragma warning( pop )
#endif

  randcontext->mt[0] = u = s & 0xffffffffUL;
  for (i=1; i<N; i++) 
  {
    u = (1812433253UL * (u ^ (u >> 30)) + i); 
    /* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
    /* In the previous versions, MSBs of the seed affect   */
    /* only MSBs of the array mt[].                        */
    /* 2002/01/09 modified by Makoto Matsumoto             */

    u &= 0xffffffffUL;
    /* for confused people who end up with sizeof(ON__UINT32) > 4*/

    randcontext->mt[i] = u;
  }

  randcontext->mti = N;
}


///* initialize by an array with array-length */
///* init_key is the array for initializing keys */
///* key_length is its length */
///* slight change for C++, 2004/2/26 */
//void on_srand_by_array(unsigned long init_key[], int key_length)
//{
//    int i, j, k;
//    init_genrand(19650218UL);
//    i=1; j=0;
//    k = (N>key_length ? N : key_length);
//    for (; k; k--) {
//        mt[i] = (mt[i] ^ ((mt[i-1] ^ (mt[i-1] >> 30)) * 1664525UL))
//          + init_key[j] + j; /* non linear */
//        mt[i] &= 0xffffffffUL; /* for WORDSIZE > 32 machines */
//        i++; j++;
//        if (i>=N) { mt[0] = mt[N-1]; i=1; }
//        if (j>=key_length) j=0;
//    }
//    for (k=N-1; k; k--) {
//        mt[i] = (mt[i] ^ ((mt[i-1] ^ (mt[i-1] >> 30)) * 1566083941UL))
//          - i; /* non linear */
//        mt[i] &= 0xffffffffUL; /* for WORDSIZE > 32 machines */
//        i++;
//        if (i>=N) { mt[0] = mt[N-1]; i=1; }
//    }
//
//    mt[0] = 0x80000000UL; /* MSB is 1; assuring non-zero initial array */ 
//}

/* generates a random number on [0,0xffffffff]-interval */
ON__UINT32 on_random_number(struct ON_RANDOM_NUMBER_CONTEXT* randcontext)
{
  static const ON__UINT32 mag01[2]={0x0UL, MATRIX_A};
  ON__UINT32 kk, y;
  /* mag01[x] = x * MATRIX_A  for x=0,1 */

  if (randcontext->mti >= N) 
  {
    /* generate N words at one time */
    if (randcontext->mti >= N+1)
    {
      /* if randcontext has never been initialized */
      on_random_number_seed(5489UL,randcontext); /* a default initial seed is used */
    }

    for (kk=0;kk<N-M;kk++)
    {
      y = (randcontext->mt[kk]&UPPER_MASK)|(randcontext->mt[kk+1]&LOWER_MASK);
      randcontext->mt[kk] = randcontext->mt[kk+M] ^ (y >> 1) ^ mag01[y & 0x1UL];
    }
    for (;kk<N-1;kk++) 
    {
      y = (randcontext->mt[kk]&UPPER_MASK)|(randcontext->mt[kk+1]&LOWER_MASK);
      randcontext->mt[kk] = randcontext->mt[kk+(M-N)] ^ (y >> 1) ^ mag01[y & 0x1UL];
    }
    y = (randcontext->mt[N-1]&UPPER_MASK)|(randcontext->mt[0]&LOWER_MASK);
    randcontext->mt[N-1] = randcontext->mt[M-1] ^ (y >> 1) ^ mag01[y & 0x1UL];

    randcontext->mti = 0;
  }

  y = randcontext->mt[randcontext->mti++];

  /* Tempering */
  y ^= (y >> 11);
  y ^= (y << 7) & 0x9d2c5680UL;
  y ^= (y << 15) & 0xefc60000UL;
  y ^= (y >> 18);

  return y;
}





// non-thread safe context used by on_srand() and on_rand().
static struct ON_RANDOM_NUMBER_CONTEXT static_randcontext = {N+1,{0}};

void on_srand(ON__UINT32 s)
{
  // This function is not thread safe!  
  // It initializes a static global which is also used by on_rand().
  struct ON_RANDOM_NUMBER_CONTEXT randcontext;
  on_random_number_seed(s,&randcontext);
  memcpy(&static_randcontext,&randcontext,sizeof(static_randcontext));
}

/* generates a random number on [0,0xffffffff]-interval */
ON__UINT32 on_rand(void)
{
  // This function is not thread safe!  
  // It modifies a static global which is also used by on_srand().
  return on_random_number(&static_randcontext);
}






///* generates a random number on [0,0x7fffffff]-interval */
//long genrand_int31(void)
//{
//    return (long)(on_rand()>>1);
//}
//
///* generates a random number on [0,1]-real-interval */
//double genrand_real1(void)
//{
//    return on_rand()*(1.0/4294967295.0); 
//    /* divided by 2^32-1 */ 
//}
//
///* generates a random number on [0,1)-real-interval */
//double genrand_real2(void)
//{
//    return on_rand()*(1.0/4294967296.0); 
//    /* divided by 2^32 */
//}
//
///* generates a random number on (0,1)-real-interval */
//double genrand_real3(void)
//{
//    return (((double)on_rand()) + 0.5)*(1.0/4294967296.0); 
//    /* divided by 2^32 */
//}
//
///* generates a random number on [0,1) with 53-bit resolution*/
//double genrand_res53(void) 
//{ 
//    unsigned long a=genrand_int32()>>5, b=genrand_int32()>>6; 
//    return(a*67108864.0+b)*(1.0/9007199254740992.0); 
//} 
///* These real versions are due to Isaku Wada, 2002/01/09 added */


ON_RandomNumberGenerator::ON_RandomNumberGenerator()
{
  m_rand_context.mti = 0xFFFFFFFF;
}

void ON_RandomNumberGenerator::Seed( ON__UINT32 s )
{
  on_random_number_seed(s,&m_rand_context);
}

ON__UINT32 ON_RandomNumberGenerator::RandomNumber()
{
  return on_random_number(&m_rand_context);
}

double ON_RandomNumberGenerator::RandomDouble()
{
  return ((double)on_random_number(&m_rand_context))/4294967295.0;
}

double ON_RandomNumberGenerator::RandomDouble(double t0, double t1)
{
  const double s = ((double)on_random_number(&m_rand_context))/4294967295.0;
  return ((1.0-s)*t0 + s*t1);
}

static void Swap1(std::size_t count, unsigned char* a, unsigned char* b)
{
  unsigned char t;
  while (count--)
  {
    t = *a;
    *a = *b;
    *b = t;
    a++;
    b++;
  }
}

static void Swap4(std::size_t count, ON__UINT32* a, ON__UINT32* b)
{
  ON__UINT32 t;
  while (count--)
  {
    t = *a;
    *a = *b;
    *b = t;
    a++;
    b++;
  }
}

static void Swap8(std::size_t count, ON__UINT64* a, ON__UINT64* b)
{
  ON__UINT64 t;
  while (count--)
  {
    t = *a;
    *a = *b;
    *b = t;
    a++;
    b++;
  }
}

void ON_RandomNumberGenerator::RandomPermutation(void* base, std::size_t nel, std::size_t sizeof_element )
{
  ON__UINT32 i, j, n;

  if ( 0 == base || nel <= 1 || sizeof_element <= 0 )
    return;

#if defined(ON_64BIT_POINTER)
  if ( nel > 0xFFFFFFFF || sizeof_element > 0xFFFFFFFF)
    return;
#endif

  n = (ON__UINT32)nel; // for 64 bit systems, nel is wider than n.

  // References: 
  //  http://en.wikipedia.org/wiki/Random_permutation
  //  http://en.wikipedia.org/wiki/Knuth_shuffle

  // Note:
  //   There is the usual "sloppy bias" in the code below because 
  //   (on_random_number(&m_rand_context) % N) is used to get a random
  //   number int the range 0 to N-1 when N is not a factor of 2^32.
  //   As usual, this bias is not worth worrying about
  //   unlsess 2^32 / N is smallish.  If you need a random
  //   permuation of a very large array, look elsewhere.

  if ( 0 == sizeof_element % sizeof(ON__UINT64) )
  {
    ON__UINT64* a = (ON__UINT64*)base;
    sizeof_element /= sizeof(a[0]);
    for ( i = 0; i < n; i++ )
    {
      j = on_random_number(&m_rand_context) % (n-i);
      if ( j )
      {
        Swap8(sizeof_element, a+i, a+i+j);
      }
    }
  }
  else if ( 0 == sizeof_element % sizeof(ON__UINT32) )
  {
    ON__UINT32* a = (ON__UINT32*)base;
    sizeof_element /= sizeof(a[0]);
    for ( i = 0; i < n; i++ )
    {
      j = on_random_number(&m_rand_context) % (n-i);
      if ( j )
      {
        Swap4(sizeof_element, a+i, a+i+j);
      }
    }
  }
  else
  {
    unsigned char* a = (unsigned char*)base;
    for ( i = 0; i < n; i++ )
    {
      j = on_random_number(&m_rand_context) % (n-i);
      if ( j )
      {
        Swap1(sizeof_element, a+i, a+i+j);
      }
    }
  }
}


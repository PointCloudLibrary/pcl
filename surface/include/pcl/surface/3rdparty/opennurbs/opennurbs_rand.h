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

#if !defined(OPENNURBS_RANDOM_NUMBER_INC_)
#define OPENNURBS_RANDOM_NUMBER_INC_

ON_BEGIN_EXTERNC

struct ON_RANDOM_NUMBER_CONTEXT
{
  ON__UINT32 mti;     /* mti = 0xFFFFFFFF means mt[] is not initialized */
  ON__UINT32 mt[624]; /* the array for the state vector  */
};


/*
Description:
  Seed a context for on_random_number().
Parameters:
  s - [in]
  rand_context - [out] context to seed.

Remarks:
  on_random_number_seed() does not use any static memory.
Example:
          ON_RAND_CONTEXT rand_context;

          ON__UINT seed = 123;
          on_random_number_seed( seed, &rand_context );

          ON__UINT32 r1 = on_random_number( &rand_context );
          ON__UINT32 r2 = on_random_number( &rand_context );
          ON__UINT32 r3 = on_random_number( &rand_context );
*/
void on_random_number_seed(
        ON__UINT32 s,
        struct ON_RANDOM_NUMBER_CONTEXT* rand_context
        );

/*
Description:
  Get a random number.
Parameters:
  rand_context - [in/out]
    random number context.  The first time rand_context is
    used it must be either initialized by calling on_random_number_seed()
    or rand_context->mti must be set to 0xFFFFFFFF.  Otherwise do not 
    modify randcontext between calls to on_random_number.
Returns:
  A random number.
Remarks:
  on_random_number() does not use any static memory.
Example:
          ON_RAND_CONTEXT rand_context;

          ON__UINT seed = 123;
          on_random_number_seed( seed, &rand_context );

          ON__UINT32 r1 = on_random_number( &rand_context );
          ON__UINT32 r2 = on_random_number( &rand_context );
          ON__UINT32 r3 = on_random_number( &rand_context );
*/
ON__UINT32 on_random_number(
        struct ON_RANDOM_NUMBER_CONTEXT* rand_context
        );


/*
Description:
  Seed the random number generator used by on_rand().
Parameters:
  s - [in]
Remarks:
  on_srand() is not thread safe.  It used static global memory
  that is modified by on_srand() and on_rand().
*/
void on_srand(ON__UINT32 s);

/*
Description:
  Get a random number.
Returns:
  A random number.
Remarks:
  on_rand() is not thread safe.  It used static global memory
  that is modified by on_srand() and on_rand().
*/
ON__UINT32 on_rand(void);


ON_END_EXTERNC


#if defined(ON_CPLUSPLUS)

class ON_CLASS ON_RandomNumberGenerator
{
public:
  ON_RandomNumberGenerator();

  /*
  Description:
    Seed the random number generator.
  Parameters:
    s - [in]
  */
  void Seed( ON__UINT32 s );

  /*
  Returns:
    32 bit unsigned random number [0,0xFFFFFFFF] [0,4294967295]
  */
  ON__UINT32 RandomNumber();

  /*
  Returns:
    double in the interval [0.0 and 1.0]
  */
  double RandomDouble();

  /*
  Returns:
    double in the interval [t0,t1]
  */
  double RandomDouble(double t0, double t1);

  /*
  Description:
    Perform a random permuation on an array.
  Parameters:
    base - [in/out]
      Array of element to permute
    nel - [in]
      number of elements in the array.
    sizeof_element
      size of an element in bytes.
  */
  void RandomPermutation(void* base, size_t nel, size_t sizeof_element );

private:
  struct ON_RANDOM_NUMBER_CONTEXT m_rand_context;
};

#endif


#endif

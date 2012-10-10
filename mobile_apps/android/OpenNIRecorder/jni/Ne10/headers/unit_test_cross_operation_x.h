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
 * NE10 Library : headers/unit_test_cross_operation_x.h
 */

#include "./unit_test_common.h"

// This function signature applies the operations with the format "*c_*_*" (e.g. 'add'c_'float'_'neon')
typedef arm_result_t (*arm_func_4args_t)(void * dst, void * src1, void * src2, unsigned int count);
arm_func_4args_t ftbl[ OP_COUNT * IMPL_COUNT ];


// this function is implemented in the unit test source files
// it is meant to initialise the function table defined above.
extern void init_ftbl();


unsigned int i = 0;   // loop iterator
unsigned int max = 0; // number of iterations in each function
int opcode = -1; // the operation which will be tested (a single unit can have any number of operations/functions)
int impl   = -1; // selects which particular implementation of the chosen operation must run
int mute   = 0;   // 0 == print output;   1 == do not print anything;

struct timeval  before, after, lapsed, dummy;
double dt_test_overhead = 0.0;
double dt_test_sample = 0.0;
double elapsed = 0.0;
struct timezone zone;

// there is a max of "4" components in a vec
#define MAX_VEC_COMPONENTS 4

arm_float_t * guarded_src1 = NULL;
arm_float_t * guarded_src2 = NULL;
arm_float_t * guarded_dst[IMPL_COUNT];

arm_float_t * thesrc1 = NULL;
arm_float_t * thesrc2 = NULL;
arm_float_t * thedst[IMPL_COUNT]; // output from different implementations are stored in separate arrays for varification
int done_init = 0;

// Eight buffers that are used for especial test cases such as when the destination and source point to the same address.
// They may vary in size from one case to another and from one function to another.
arm_float_t*  esp_buf[8];

arm_result_t test_operation()
{
  const unsigned int fixed_length = ARRLEN * sizeof(arm_float_t) * MAX_VEC_COMPONENTS;

  // initialize if not done so
  if ( 0 == done_init )
  {
    guarded_src1 = (arm_float_t*) malloc( (2*ARRAY_GUARD_LEN) + fixed_length ); // 16 extra bytes at the begining and 16 extra bytes at the end
    GUARD_ARRAY( guarded_src1, (2*ARRAY_GUARD_LEN) + fixed_length );
    thesrc1 = (arm_float_t*) ( (void*)guarded_src1 + 16);
    FILL_FLOAT_ARRAY_LIMIT( thesrc1, ARRLEN * MAX_VEC_COMPONENTS ); // random initialization

    guarded_src2 = (arm_float_t*) malloc( (2*ARRAY_GUARD_LEN) + fixed_length ); // 16 extra bytes at the begining and 16 extra bytes at the end
    GUARD_ARRAY( guarded_src2, (2*ARRAY_GUARD_LEN) + fixed_length );
    thesrc2 = (arm_float_t*) ( (void*)guarded_src2 + 16);
    FILL_FLOAT_ARRAY_LIMIT( thesrc2, ARRLEN * MAX_VEC_COMPONENTS ); // random initialization

    for ( i = 0; i<IMPL_COUNT; i++ )
    {
      guarded_dst[i] = (arm_float_t*) malloc( (2*ARRAY_GUARD_LEN) + fixed_length ); // 16 extra bytes at the begining and 16 extra bytes at the end
      GUARD_ARRAY( guarded_dst[i], (2*ARRAY_GUARD_LEN) + fixed_length );
      thedst[i] = (arm_float_t*) ( (void*)guarded_dst[i] + 16);
    }

    done_init = 1;
  }


  // test the especial case where dst == src
  unsigned int tmp_len = 13; // Just an odd number bigger than 8
  unsigned int inbytes = tmp_len * MAX_VEC_COMPONENTS * sizeof(arm_float_t);
  esp_buf[0] = (arm_float_t*) malloc( inbytes ); // input 1
  esp_buf[1] = (arm_float_t*) malloc( inbytes ); // input 2
  esp_buf[2] = (arm_float_t*) malloc( inbytes ); // copy of 1st input
  esp_buf[3] = (arm_float_t*) malloc( inbytes ); // copy of 2nd input
  esp_buf[4] = (arm_float_t*) malloc( inbytes ); // use this as the output buffer

  FILL_FLOAT_ARRAY_LIMIT( esp_buf[0], tmp_len * MAX_VEC_COMPONENTS ); // initialization the array with random numbers
  FILL_FLOAT_ARRAY_LIMIT( esp_buf[1], tmp_len * MAX_VEC_COMPONENTS ); // initialization the array with random numbers
  memcpy( esp_buf[2], esp_buf[0], inbytes );
  memcpy( esp_buf[3], esp_buf[1], inbytes );

  ftbl [ FTBL_IDX(opcode, impl) ] ( esp_buf[0] , esp_buf[0], esp_buf[1], tmp_len );
  ftbl [ FTBL_IDX(opcode, impl) ] ( esp_buf[4] , esp_buf[2], esp_buf[3], tmp_len );

  fprintf ( stderr, "** NTOE: Due to the nature of this test we cannot use an assert - the values may or may not be the same... make sure NAN values are not geenrated by using FILL_FLOAT_ARRAY_\'LIMIT\'. \n" );

  for ( i = 0;  i < tmp_len * opcode; i++ ) // at this point the two outputs must be identical
  {
      // assert( esp_buf[0][i] == esp_buf[4][i] ); // check for not-a-number

      if ( ! EQUALS_FLOAT( esp_buf[0][i] , esp_buf[4][i], ERROR_MARGIN_LARGE*10 ) )
      {
          fprintf ( stderr, "\t FATAL ERROR: Operation number %d implementation [%d] has failed the dst==src test case. \n", opcode, impl );
          fprintf ( stderr, "\t NOTE: Usually implementation 1=C, 2=ASM/VFP, and 3=ASM/NEON. \n");
          exit( NE10_ERR );
     }
  }

  free(esp_buf[0]); free(esp_buf[1]); free(esp_buf[2]); free(esp_buf[3]); free(esp_buf[4]);


  // sample run
  MEASURE( dt_test_sample,
    ftbl [ FTBL_IDX(opcode, impl) ] ( thedst[ impl-1 ] , thesrc1, thesrc2, ARRLEN );
  );
  if ( ! CHECK_ARRAY_GUARD(guarded_dst[ impl -1 ], (2*ARRAY_GUARD_LEN) + fixed_length) ||
       ! CHECK_ARRAY_GUARD(guarded_src1, (2*ARRAY_GUARD_LEN) + fixed_length) ||
       ! CHECK_ARRAY_GUARD(guarded_src2, (2*ARRAY_GUARD_LEN) + fixed_length) )
  {
                fprintf ( stderr, "\t FATAL ERROR: Operation number %d implementation [%d] has failed the guard test. \n", opcode, impl );
                exit( NE10_ERR );
  }

  // this test to make sure passing zero as the length won't cause segmentation faults
  ftbl [ FTBL_IDX(opcode, impl) ] ( thedst[ impl-1 ] , thesrc1, thesrc2, 0 );

          MEASURE( elapsed,
            for ( i = 0; i < max; i++  )
            {
               // call the function
               ftbl [ FTBL_IDX(opcode, impl) ] ( thedst[ impl -1 ] , thesrc1, thesrc2, ARRLEN );
            }
           );

  if ( !mute )
       printf( "%02.8f;%013.3f\n", elapsed - dt_test_overhead,
                              ( 1.0f * max * ARRLEN / ( elapsed - dt_test_overhead ))  );

 return NE10_OK;
}

arm_result_t run_test( int argc, char **argv )
{
   if ( argc == 2 ) // requesting the number of available operations/routines in this unit
   {
      opcode = atoi ( argv[1] ); // get the command being requested, 0 = return the number of functions in this unit
      if ( opcode == 0 ) return OP_COUNT;
      exit( NE10_ERR );
   } else if ( argc == 4 ) // requesting a particular implementation of one of the operations
   {
      opcode = atoi ( argv[1] );
      if ( opcode <= 0 ) exit( NE10_ERR );
      impl   = atoi ( argv[2] );
      if ( impl   < 0 ) exit( NE10_ERR ); // impl == 0 means run all and compare the results to verify they produce identical outputs
      max = atoi ( argv[3] );
      if ( max <= 0 ) exit( NE10_ERR );
   } else exit( NE10_ERR );

   // initialize the table with NULL
   memset( ftbl, 0, sizeof(ftbl));

   // manually initialize the functions which have actual implementations
   init_ftbl(); // this function is implemented in the unit test source file

  if ( opcode <= 0 || opcode > OP_COUNT
       || impl < 0 || impl > IMPL_COUNT )
  {
      fprintf ( stderr, "\t WARNING: Operation number %d and/or implementaion number %d are not acceptable values. \n", opcode, impl );
      exit( NE10_ERR );
  }

  if ( impl == 0 ) // run all implementations and verify
  {
      // first, make sure all of the implementations do exist
      for ( i = FTBL_IDX(opcode, 1); i <= FTBL_IDX(opcode, IMPL_COUNT); i++ )
      {
        if ( NULL == ftbl[i] )
        {
                fprintf ( stderr, "\t WARNING: One or more implementations of operation number %d were not found. \n", opcode );
                exit( NE10_ERR );
        }
      }

      // try all the implementatins here..
             mute = 1; // do not print anything

             // opcode remains the same but we iterate through different implementations here..
             for ( impl= 1; impl <= IMPL_COUNT; impl ++ )
             {
                 test_operation();
             }

             // now verify
             arm_float_t * _output = NULL; // [ IMPL_COUNT * MAX_VEC_COMPONENTS ]; // one for each implementation, c, asm, neon...
             int warns = 0;
             int item_width = 3; // cross has only one form at the moment, the vec3 with 3 components, x, y, and z
             _output = (arm_float_t*) malloc( IMPL_COUNT * sizeof(arm_float_t) * item_width );
             for ( i = 0; i < ARRLEN; i++ )
             {
                 for ( impl= 1; impl <= IMPL_COUNT; impl ++ )
                 {
                     memcpy ( &_output[  (impl-1) * item_width  ], &thedst[ impl-1 ][ i * item_width ],  sizeof(arm_float_t) * item_width  );
                 }

                 int pos = 0;
                 for ( impl = 2; impl <= IMPL_COUNT; impl ++ ) // compare the output from the 2nd, 3rd, 4th, etc. to the first one so start at 2
                 {
                     for ( pos = 0; pos < item_width; pos++ ) // compare corresponding components of the items
                     {
                         assert ( _output[ ((1-1)*item_width)+pos ] == _output[ ((1-1)*item_width)+pos ] ); // check for not-a-number
                         assert ( _output[ ((impl-1)*item_width)+pos ] == _output[ ((impl-1)*item_width)+pos ] );  // check for not-a-number

                         if ( ! EQUALS_FLOAT( _output[ ((1-1)*item_width)+pos ] , _output[ ((impl-1)*item_width)+pos ], ERROR_MARGIN_SMALL ) )
                         { fprintf( stderr, "\t\t WARNING: In opcode [%d], implementation [1] != implemenation [%d] on item [%d -> %d]\n",
                                    opcode, impl, i, pos+1 );
                             warns++; }

                         // stop after 10 warnings
                         if ( warns >= ACCEPTABLE_WARNS )
                         {    fprintf ( stderr, "\t WARNING: One or more mismatching values were found. \n" );
                              exit( NE10_ERR );
                         }
                     }
                 }
             }
             free( _output ); _output = (arm_float_t *) NULL;

             if ( warns < ACCEPTABLE_WARNS )
             {
               return NE10_OK;
             }

             fprintf ( stderr, "\t WARNING: One or more mismatching values were found. \n" );
             exit( NE10_ERR );
  }
  else // run a particular implementation
  {
      if ( !mute ) printf( "opcode=%d;impl=%d;%d;%d;", opcode, impl, ARRLEN, max );

      // ge the overhead
      MEASURE( dt_test_overhead,
               for ( i = 0 ; i < max; i++ )
               {
               }
              );

       test_operation();
  }



  // free any allocated memory...
  free( guarded_src1 );
  free( guarded_src2 );
  for ( i = 0; i<IMPL_COUNT; i++ )
  {
    free( guarded_dst[i] );
  }

  return NE10_OK;
}

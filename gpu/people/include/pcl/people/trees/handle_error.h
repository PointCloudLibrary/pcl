/**
 * @authors: Cedric Cagniart, Koen Buys
 **/

#ifndef PCL_PEOPLE_TREES_HANDLE_ERROR_H_
#define PCL_PEOPLE_TREES_HANDLE_ERROR_H_
#include <stdio.h>
#include <cuda_runtime_api.h>
namespace pcl
{
  namespace people
  {
    namespace trees
    {
      static void HandleError( cudaError_t err,
                               const char *file,
                               int line ) {
          if (err != cudaSuccess) {
              printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
                      file, line );
              exit( EXIT_FAILURE );
          }
      }

      #define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))

      #define HANDLE_NULL( a ) { if ((a) == NULL) { printf( "Host memory failed in %s at line %d\n", __FILE__, __LINE__ ); exit( EXIT_FAILURE );} }
	}
  }
}
#endif  // PCL_PEOPLE_TREES_HANDLE_ERROR_H_

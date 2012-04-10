/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: $
 * @authors: Koen Buys, Cedric Cagniart
 */

#ifndef PCL_GPU_PEOPLE_HANDLE_ERROR_H_
#define PCL_GPU_PEOPLE_HANDLE_ERROR_H_
#include <stdio.h>
#include <cuda_runtime_api.h>
namespace pcl
{
  namespace gpu
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
}
#endif  // PCL_PEOPLE_TREES_HANDLE_ERROR_H_

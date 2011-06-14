/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_CUDA_TIME_GPU_H_
#define PCL_CUDA_TIME_GPU_H_

#include <cuda.h>
#include <pcl/cuda/cutil_inline.h>

namespace pcl
{
  namespace cuda
  {
    /**
     * \brief Class to measure the time spent in a scope
     *
     * To use this class, e.g. to measure the time spent in a function,
     * just create an instance at the beginning of the function.
     */
    class ScopeTimeGPU 
    {
      public: 
        /** \brief Constructor. */
        inline ScopeTimeGPU (const char* title) : title_ (title)
        {
          start ();
        }
  
        /** \brief Destructor. */
        inline ~ScopeTimeGPU ()
        {
          stop ();
          std::cerr << title_ << " took " << elapsed_time_ << "ms.\n";
        }
  
        /** \brief Start the timer. */
        inline void 
        start ()
        {
          CUT_CHECK_ERROR ("dude");
          cutilSafeCall (cudaEventCreate (&start_));
          cutilSafeCall (cudaEventCreate (&end_));
  
          cutilSafeCall (cudaEventRecord (start_, 0));
        }
  
        /** \brief Stop the timer. */
        inline void 
        stop ()
        {
          CUT_CHECK_ERROR ("dude");
          // Measure time needed to copy data
          cutilSafeCall (cudaThreadSynchronize ());
          cutilSafeCall (cudaEventRecord (end_, 0));
          cutilSafeCall (cudaEventSynchronize (end_));
          cutilSafeCall (cudaEventElapsedTime (&elapsed_time_, start_, end_));
  
          cutilSafeCall (cudaEventDestroy (end_));
          cutilSafeCall (cudaEventDestroy (start_));
        }
  
        /** \brief Stop and print the timer. */
        inline void stop_print ()
        {
          stop ();
          std::cerr << title_ << " took " << elapsed_time_ << "ms.\n";
        }
  
  
      private:
        cudaEvent_t start_;
        cudaEvent_t end_;
  
        float elapsed_time_;
  
        std::string title_;
    };
  } // namespace
} // namespace

#endif  //#ifndef PCL_CUDA_TIMER_H_

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
 */

#ifndef PCL_CUDA_TIME_CPU_H_
#define PCL_CUDA_TIME_CPU_H_

#include <iostream>
#include <cmath>
#include <string>

#include <sys/time.h>

/////////////////////////////////////////////////////////////////////
// Note: this file is here because NVCC can't deal with all boost
//       things, e.g. the new pcl::ScopeTime implementation
/////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace cuda
  {
    /**
     * \brief Class to measure the time spent in a scope
     *
     * To use this class, e.g. to measure the time spent in a function,
     * just create an instance at the beginning of the function.
     * \ingroup common
     */
    class ScopeTimeCPU
    {
      public: 
        inline ScopeTimeCPU (const char* title);
        inline ~ScopeTimeCPU ();
      private:
        std::string title_;
        timeval start_time_;
    };


#ifndef MEASURE_FUNCTION_TIME
#define MEASURE_FUNCTION_TIME \
  ScopeTimeCPU scopeTime(__func__)
#endif

    inline double getTime ();

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code) \
if (1) {\
  static double s_lastDone_ = 0.0; \
  double s_now_ = (currentTime); \
  if (s_lastDone_ > s_now_) \
    s_lastDone_ = s_now_; \
  if (s_now_ - s_lastDone_ > (secs)) { \
    code; \
    s_lastDone_ = s_now_; \
  }\
} else \
  (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) \
  DO_EVERY_TS(secs, pcl::cuda::getTime(), code)
#endif

  }  // end namespace
}  // end namespace


inline double 
  pcl::cuda::getTime ()
{
  timeval current_time;
  gettimeofday (&current_time, NULL);
  return (current_time.tv_sec + 1e-6 * current_time.tv_usec);
}

inline pcl::cuda::ScopeTimeCPU::ScopeTimeCPU (const char* title) : title_ (title)
{
  gettimeofday (&start_time_, NULL);
}

inline pcl::cuda::ScopeTimeCPU::~ScopeTimeCPU ()
{
  timeval end_time;
  gettimeofday (&end_time, NULL);
  double duration = end_time.tv_sec - start_time_.tv_sec + 1e-6 * (end_time.tv_usec - start_time_.tv_usec);
  std::cerr << title_ << " took " << 1000 * duration << "ms.\n";
}

#endif  //#ifndef PCL_NORMS_H_

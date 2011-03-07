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
 *  FOR a PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#include <iostream>

inline double 
  pcl::getTime ()
{
  timeval current_time;
#ifdef _WIN32
  DWORD time = timeGetTime ();
  current_time.tv_sec  = time / 1000;
  current_time.tv_usec = time % 1000;
#else
  gettimeofday (&current_time, NULL);
#endif
  return (current_time.tv_sec + 1e-6 * current_time.tv_usec);
}

inline pcl::ScopeTime::ScopeTime (const char* title) : title_ (title)
{
#ifdef _WIN32
  QueryPerformanceFrequency (&frequency_);
  QueryPerformanceCounter (&start_time_);
#else
  gettimeofday (&start_time_, NULL);
#endif
  //std::cerr << "start time is ("<<_startTime.tv_sec<<", "<<_startTime.tv_usec<<").\n";
}

inline pcl::ScopeTime::~ScopeTime ()
{
#ifdef _WIN32
  LARGE_INTEGER end_time;
  QueryPerformanceCounter (&end_time);
  double duration = (end_time.QuadPart - start_time_.QuadPart) * 1000.0 / frequency_.QuadPart;
#else
  timeval end_time;
  gettimeofday (&end_time, NULL);
  double duration = end_time.tv_sec - start_time_.tv_sec + 1e-6 * (end_time.tv_usec - start_time_.tv_usec);
#endif
  //std::cerr << "start time is ("<<_startTime.tv_sec<<", "<<_startTime.tv_usec<<")."
  //          << " End time is ("<<endTime.tv_sec<<", "<<endTime.tv_usec<<") => "<<std::fixed<<duration<<"\n";
  std::cerr << title_ << " took " << 1000 * duration << "ms.\n";
}

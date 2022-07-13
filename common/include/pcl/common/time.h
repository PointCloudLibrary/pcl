/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#pragma once

#include <chrono>
#include <iostream>
#include <queue>
#include <string>

/**
  * \file pcl/common/time.h
  * Define methods for measuring time spent in code blocks
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  /** \brief Simple stopwatch.
    * \ingroup common
    */
  class StopWatch
  {
    public:
      /** \brief Constructor. */
      StopWatch () = default;

      /** \brief Retrieve the time in milliseconds spent since the last call to \a reset(). */
      inline double
      getTime () const
      {
        auto end_time = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::ratio<1, 1000>>(end_time - start_time_).count();
      }

      /** \brief Retrieve the time in seconds spent since the last call to \a reset(). */
      inline double
      getTimeSeconds () const
      {
        return (getTime () * 0.001);
      }

      /** \brief Reset the stopwatch to 0. */
      inline void
      reset ()
      {
        start_time_ = std::chrono::steady_clock::now();
      }

    protected:
      std::chrono::time_point<std::chrono::steady_clock> start_time_ = std::chrono::steady_clock::now();
  };

  /** \brief Class to measure the time spent in a scope
    *
    * To use this class, e.g. to measure the time spent in a function,
    * just create an instance at the beginning of the function. Example:
    *
    * \code
    * {
    *   pcl::ScopeTime t1 ("calculation");
    *
    *   // ... perform calculation here
    * }
    * \endcode
    *
    * \ingroup common
    */
  class ScopeTime : public StopWatch
  {
    public:
      inline ScopeTime (const std::string &title = "") : 
        title_ (title)
      {
      }

      inline ~ScopeTime ()
      {
        double val = this->getTime ();
        std::cerr << title_ << " took " << val << "ms.\n";
      }

    private:
      std::string title_;
  };

  /** \brief A helper class to measure frequency of a certain event.
    *
    * To use this class create an instance and call event() function every time
    * the event in question occurs. The estimated frequency can be retrieved
    * with getFrequency() function.
    *
    * \author Sergey Alexandrov
    * \ingroup common
    */
  class EventFrequency
  {

    public:

      /** \brief Constructor.
        *
        * \param[in] window_size number of most recent events that are
        * considered in frequency estimation (default: 30) */
      EventFrequency (std::size_t window_size = 30)
      : window_size_ (window_size)
      {
        stop_watch_.reset ();
      }

      /** \brief Notifies the class that the event occurred. */
      void event ()
      {
        event_time_queue_.push (stop_watch_.getTimeSeconds ());
        if (event_time_queue_.size () > window_size_)
          event_time_queue_.pop ();
      }

      /** \brief Retrieve the estimated frequency. */
      double
      getFrequency () const
      {
        if (event_time_queue_.size () < 2)
          return (0.0);
        return ((event_time_queue_.size () - 1) /
                (event_time_queue_.back () - event_time_queue_.front ()));
      }

      /** \brief Reset frequency computation. */
      void reset ()
      {
        stop_watch_.reset ();
        event_time_queue_ = std::queue<double> ();
      }

    private:

      pcl::StopWatch stop_watch_;
      std::queue<double> event_time_queue_;
      const std::size_t window_size_;

  };

#ifndef MEASURE_FUNCTION_TIME
#define MEASURE_FUNCTION_TIME \
  ScopeTime scopeTime(__func__)
#endif

inline double 
getTime ()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code) \
if (1) {\
  static double s_lastDone_ = 0.0; \
  double s_now_ = (currentTime); \
  if (s_lastDone_ > s_now_) \
    s_lastDone_ = s_now_; \
  if ((s_now_ - s_lastDone_) > (secs)) {        \
    code; \
    s_lastDone_ = s_now_; \
  }\
} else \
  (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) \
  DO_EVERY_TS(secs, pcl::getTime(), code)
#endif

}  // end namespace
/*@}*/

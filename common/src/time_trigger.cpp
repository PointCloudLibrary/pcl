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

#include <pcl/common/time_trigger.h>
#include <pcl/common/time.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::TimeTrigger::TimeTrigger (double interval, const callback_type& callback)
: interval_ (interval)
, quit_ (false)
, running_ (false)
{
  timer_thread_ = std::thread (&TimeTrigger::thread_function, this);
  registerCallback (callback);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::TimeTrigger::TimeTrigger (double interval)
: interval_ (interval)
, quit_ (false)
, running_ (false)
{
  timer_thread_ = std::thread (&TimeTrigger::thread_function, this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::TimeTrigger::~TimeTrigger ()
{
  std::unique_lock<std::mutex> lock (condition_mutex_);
  quit_ = true;
  condition_.notify_all (); // notify all threads about updated quit_
  lock.unlock (); // unlock, to join all threads (needs to be done after notify_all)

  timer_thread_.join ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection 
pcl::TimeTrigger::registerCallback (const callback_type& callback)
{
  return (callbacks_.connect (callback));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimeTrigger::setInterval (double interval_seconds)
{
  std::unique_lock<std::mutex> lock (condition_mutex_);
  interval_ = interval_seconds;
  // notify, since we could switch from a large interval to a shorter one -> interrupt waiting for timeout!
  condition_.notify_all ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimeTrigger::start ()
{
  std::unique_lock<std::mutex> lock (condition_mutex_);
  if (!running_)
  {
    running_ = true;
    condition_.notify_all ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimeTrigger::stop ()
{
  std::unique_lock<std::mutex> lock (condition_mutex_);
  if (running_)
  {
    running_ = false;
    condition_.notify_all ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TimeTrigger::thread_function ()
{
  while (true)
  {
    double time = getTime ();
    std::unique_lock<std::mutex> lock (condition_mutex_);
    if(quit_)
      break;
    if (!running_)
      condition_.wait (lock); // wait util start is called or destructor is called
    else
    {
      using namespace std::chrono_literals;

      callbacks_();
      double rest = interval_ + time - getTime ();
      condition_.wait_for (lock, rest * 1s);
    }
  }
}


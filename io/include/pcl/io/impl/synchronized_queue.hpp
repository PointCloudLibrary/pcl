/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012
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

/* SynchronizedQueue Template, taken from
 * http://stackoverflow.com/questions/10139251/shared-queue-c
 */
#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

namespace pcl
{

  template<typename T>
  class SynchronizedQueue
  {
    public:

      SynchronizedQueue () :
        queue_(), request_to_end_(false), enqueue_data_(true) { }

      void
      enqueue (const T& data)
      {
        std::unique_lock<std::mutex> lock (mutex_);

        if (enqueue_data_)
        {
          queue_.push (data);
          cond_.notify_one ();
        }
      }

      bool
      dequeue (T& result)
      {
        std::unique_lock<std::mutex> lock (mutex_);

        while (queue_.empty () && (!request_to_end_))
        {
          cond_.wait (lock);
        }

        if (request_to_end_)
        {
          doEndActions ();
          return false;
        }

        result = queue_.front ();
        queue_.pop ();

        return true;
      }

      void
      stopQueue ()
      {
        std::unique_lock<std::mutex> lock (mutex_);
        request_to_end_ = true;
        cond_.notify_one ();
      }

      unsigned int
      size ()
      {
        std::unique_lock<std::mutex> lock (mutex_);
        return static_cast<unsigned int> (queue_.size ());
      }

      bool
      isEmpty () const
      {
        std::unique_lock<std::mutex> lock (mutex_);
        return (queue_.empty ());
      }

    private:
      void
      doEndActions ()
      {
        enqueue_data_ = false;

        while (!queue_.empty ())
        {
          queue_.pop ();
        }
      }

      std::queue<T> queue_;              // Use STL queue to store data
      mutable std::mutex mutex_;       // The mutex to synchronise on
      std::condition_variable cond_;   // The condition to wait for

      bool request_to_end_;
      bool enqueue_data_;
  };
}

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#pragma once

#include <pcl/memory.h>
#include <pcl/cuda/point_cloud.h>


namespace pcl
{
namespace cuda
{
  ///////////////////////////////////////////////////////////////////////////////////////////
  /** \brief PCL base class. Implements methods that are used by all PCL objects. 
    */
  template <typename CloudT>
  class PCLCUDABase
  {
    public:
      using PointCloud = CloudT;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      /** \brief Empty constructor. */
      PCLCUDABase () : input_() {};

      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      virtual inline void 
      setInputCloud (const PointCloudConstPtr &cloud) 
      { 
        input_ = cloud; 
      }

      /** \brief Get a pointer to the input host point cloud dataset. */
      inline PointCloudConstPtr const 
      getInputCloud () 
      { 
        return (input_); 
      }

    protected:
      /** \brief The input point cloud dataset. */
      PointCloudConstPtr input_;

      /** \brief This method should get called before starting the actual computation. */
      bool
      initCompute ()
      {
        // Check if input was set
        if (!input_)
          return (false);
        return (true);
      }

      /** \brief This method should get called after finishing the actual computation. */
      bool
      deinitCompute ()
      {
        return (true);
      }
  };
} // namespace
} // namespace

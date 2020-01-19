/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#pragma once

#include <pcl/pcl_macros.h>

#include <pcl/cuda/common/eigen.h>

namespace pcl
{
  namespace cuda
  {

    // normal estimation using PCA on neighborhood. DANGER: neighborhood is sampled with a bias!! contact Nico for details :P
    template <typename InputIteratorT, typename OutputIteratorT, template <typename> class Storage>
      void computePointNormals (InputIteratorT begin, InputIteratorT end, OutputIteratorT output, float focallength, const typename PointCloudAOS<Storage>::ConstPtr &input, float radius, int desired_number_neighbors);
  
    template <template <typename> class Storage, typename InputIteratorT>
      shared_ptr<typename Storage<float4>::type> computePointNormals (InputIteratorT begin, InputIteratorT end, float focallength, const typename PointCloudAOS<Storage>::ConstPtr &input, float radius, int desired_number_neighbors);

    // fast normal computations
    template <typename OutputIteratorT, template <typename> class Storage>
      void computeFastPointNormals (OutputIteratorT output, const typename PointCloudAOS<Storage>::ConstPtr &input);
  
    template <template <typename> class Storage>
      shared_ptr<typename Storage<float4>::type> computeFastPointNormals (const typename PointCloudAOS<Storage>::ConstPtr &input);

    // Weird normal estimation (normal deviations - more of an art project..)
    template <typename InputIteratorT, typename OutputIteratorT, template <typename> class Storage>
      void computeWeirdPointNormals (InputIteratorT begin, InputIteratorT end, OutputIteratorT output, float focallength, const typename PointCloudAOS<Storage>::ConstPtr &input, float radius, int desired_number_neighbors);
  
    template <template <typename> class Storage, typename InputIteratorT>
      shared_ptr<typename Storage<float4>::type> computeWeirdPointNormals (InputIteratorT begin, InputIteratorT end, float focallength, const typename PointCloudAOS<Storage>::ConstPtr &input, float radius, int desired_number_neighbors);
  } // namespace
} // namespace

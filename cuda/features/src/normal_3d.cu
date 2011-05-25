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

#include "pcl/cuda/features/normal_3d.h"

namespace pcl
{
  namespace cuda
  {
    template <typename InputIteratorT, typename OutputIteratorT, template <typename> class Storage>
      void computePointNormals (InputIteratorT begin, InputIteratorT end, OutputIteratorT output, float focallength, const boost::shared_ptr <const PointCloudAOS <Storage> > &input, float radius, int desired_number_neighbors)
    {
      NormalEstimationKernel<Storage> ne = NormalEstimationKernel<Storage> (input, focallength, radius*radius, sqrt ((float)desired_number_neighbors));
      thrust::transform (begin, end, output, ne);
    };
  
    template <template <typename> class Storage, typename InputIteratorT>
      typename Storage<float4>::type computePointNormals (InputIteratorT begin, InputIteratorT end, float focallength, const boost::shared_ptr <const PointCloudAOS <Storage> > &input, float radius, int desired_number_neighbors)
    {
      typename Storage<float4>::type normals;
      normals.resize (end - begin);
      computePointNormals (begin, end, normals.begin(), focallength, input, radius, desired_number_neighbors);
      return normals;
    };
  
  
    // Aaaand, a couple of instantiations
    template void computePointNormals<typename Device<PointXYZRGB>::type::const_iterator, typename Device<float4>::type::iterator, Device>
                  (typename Device<PointXYZRGB>::type::const_iterator begin,
                   typename Device<PointXYZRGB>::type::const_iterator end,
                   typename Device<float4>::type::iterator output,
                   float focallength,
                   const boost::shared_ptr <const PointCloudAOS <Device> > &input,
                   float radius,
                   int desired_number_neighbors);
    
    template void computePointNormals<typename Host<PointXYZRGB>::type::const_iterator, typename Host<float4>::type::iterator, Host>
                  (typename Host<PointXYZRGB>::type::const_iterator begin,
                   typename Host<PointXYZRGB>::type::const_iterator end,
                   typename Host<float4>::type::iterator output,
                   float focallength,
                   const boost::shared_ptr <const PointCloudAOS <Host> > &input,
                   float radius,
                   int desired_number_neighbors);
    
    template typename Device<float4>::type computePointNormals<Device, typename PointIterator<Device,PointXYZRGB>::type >
                  (typename PointIterator<Device,PointXYZRGB>::type begin,
                   typename PointIterator<Device,PointXYZRGB>::type end,
                   float focallength,
                   const boost::shared_ptr <const PointCloudAOS <Device> > &input,
                   float radius,
                   int desired_number_neighbors);
    
    template typename Host<float4>::type computePointNormals<Host, typename PointIterator<Host,PointXYZRGB>::type >
                  (typename PointIterator<Host,PointXYZRGB>::type begin,
                   typename PointIterator<Host,PointXYZRGB>::type end,
                   float focallength,
                   const boost::shared_ptr <const PointCloudAOS <Host> > &input,
                   float radius,
                   int desired_number_neighbors);

  } // namespace
} // namespace


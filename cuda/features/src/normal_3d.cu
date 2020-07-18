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

#include "pcl/cuda/features/normal_3d_kernels.h"

namespace pcl
{
  namespace cuda
  {

    template <typename InputIteratorT, typename OutputIteratorT, template <typename> class Storage>
      void computePointNormals (InputIteratorT begin, InputIteratorT end, OutputIteratorT output, float focallength, const typename PointCloudAOS<Storage>::ConstPtr &input, float radius, int desired_number_neighbors)
    {
      NormalEstimationKernel<Storage> ne = NormalEstimationKernel<Storage> (input, focallength, radius*radius, sqrt ((float)desired_number_neighbors));
      thrust::transform (begin, end, output, ne);
    }
  
    template <template <typename> class Storage, typename InputIteratorT>
      shared_ptr<typename Storage<float4>::type> computePointNormals (InputIteratorT begin, InputIteratorT end, 
          float focallength, const shared_ptr <const PointCloudAOS <Storage> > &input, float radius, int desired_number_neighbors)
    {
      shared_ptr<typename Storage<float4>::type> normals (new typename Storage<float4>::type);
      normals->resize (end - begin);
      computePointNormals<InputIteratorT, typename Storage<float4>::type::iterator, Storage> (begin, end, normals->begin(), focallength, input, radius, desired_number_neighbors);
      return normals;
    }

    template <typename OutputIteratorT, template <typename> class Storage> 
      void computeFastPointNormals (OutputIteratorT output, const typename PointCloudAOS<Storage>::ConstPtr &input)
    {
      FastNormalEstimationKernel<Storage> ne = FastNormalEstimationKernel<Storage> (input, input->width, input->height);
      thrust::transform (thrust::counting_iterator<int>(0), thrust::counting_iterator<int>(0) + input->width * input->height, output, ne);
    }

    template <template <typename> class Storage>
      shared_ptr<typename Storage<float4>::type> computeFastPointNormals (const typename PointCloudAOS<Storage>::ConstPtr &input)
    {
      shared_ptr<typename Storage<float4>::type> normals (new typename Storage<float4>::type);
      normals->resize (input->points.size());
      computeFastPointNormals<typename Storage<float4>::type::iterator, Storage> (normals->begin(), input);
      return normals;
    }

    template <typename InputIteratorT, typename OutputIteratorT, template <typename> class Storage>
      void computeWeirdPointNormals (InputIteratorT begin, InputIteratorT end, OutputIteratorT output, float focallength, const typename PointCloudAOS<Storage>::ConstPtr &input, float radius, int desired_number_neighbors)
    {
      NormalEstimationKernel<Storage> ne = NormalEstimationKernel<Storage> (input, focallength, radius*radius, sqrt ((float)desired_number_neighbors));
      thrust::transform (begin, end, output, ne);
      
      // we have normals now.

      NormalDeviationKernel<Storage> nd = NormalDeviationKernel<Storage> (input, focallength, radius*radius, sqrt ((float)desired_number_neighbors));
      thrust::transform (thrust::make_zip_iterator (thrust::make_tuple (begin, output)), thrust::make_zip_iterator (thrust::make_tuple (begin, output)) + (end-begin), output, nd);
    }
  
    template <template <typename> class Storage, typename InputIteratorT>
      shared_ptr<typename Storage<float4>::type> computeWeirdPointNormals (InputIteratorT begin, InputIteratorT end, float focallength, const typename PointCloudAOS<Storage>::ConstPtr &input, float radius, int desired_number_neighbors)
    {
      shared_ptr<typename Storage<float4>::type> normals (new typename Storage<float4>::type);
      normals->resize (end - begin);
      computeWeirdPointNormals<InputIteratorT, typename Storage<float4>::type::iterator, Storage> (begin, end, normals->begin(), focallength, input, radius, desired_number_neighbors);
      return normals;
    }
  
  
    // Aaaand, a couple of instantiations
    template PCL_EXPORTS void computePointNormals<typename Device<PointXYZRGB>::type::const_iterator, typename Device<float4>::type::iterator, Device>
                  (Device<PointXYZRGB>::type::const_iterator begin,
                   Device<PointXYZRGB>::type::const_iterator end,
                   Device<float4>::type::iterator output,
                   float focallength,
                   const PointCloudAOS<Device>::ConstPtr &input,
                   float radius,
                   int desired_number_neighbors);
    
    template PCL_EXPORTS void computePointNormals<typename Host<PointXYZRGB>::type::const_iterator, typename Host<float4>::type::iterator, Host>
                  (Host<PointXYZRGB>::type::const_iterator begin,
                   Host<PointXYZRGB>::type::const_iterator end,
                   Host<float4>::type::iterator output,
                   float focallength,
                   const PointCloudAOS<Host>::ConstPtr &input,
                   float radius,
                   int desired_number_neighbors);
    
    template PCL_EXPORTS shared_ptr<typename Device<float4>::type> computePointNormals<Device, typename PointIterator<Device,PointXYZRGB>::type >
                  (PointIterator<Device,PointXYZRGB>::type begin,
                   PointIterator<Device,PointXYZRGB>::type end,
                   float focallength,
                   const PointCloudAOS<Device>::ConstPtr &input,
                   float radius,
                   int desired_number_neighbors);
    
    template PCL_EXPORTS shared_ptr<typename Host<float4>::type> computePointNormals<Host, typename PointIterator<Host,PointXYZRGB>::type >
                  (PointIterator<Host,PointXYZRGB>::type begin,
                   PointIterator<Host,PointXYZRGB>::type end,
                   float focallength,
                   const PointCloudAOS<Host>::ConstPtr &input,
                   float radius,
                   int desired_number_neighbors);

    // Aaaand, a couple of instantiations
    template PCL_EXPORTS void computeFastPointNormals<typename Device<float4>::type::iterator, Device>
                  (Device<float4>::type::iterator output,
                   const PointCloudAOS<Device>::ConstPtr &input);
    
    template PCL_EXPORTS void computeFastPointNormals<typename Host<float4>::type::iterator, Host>
                  (Host<float4>::type::iterator output,
                   const PointCloudAOS<Host>::ConstPtr &input);
    
    template PCL_EXPORTS shared_ptr<typename Device<float4>::type> computeFastPointNormals<Device>
                  (const PointCloudAOS<Device>::ConstPtr &input);
    
    template PCL_EXPORTS shared_ptr<typename Host<float4>::type> computeFastPointNormals<Host>
                  (const PointCloudAOS<Host>::ConstPtr &input);

    // Aaaand, a couple of instantiations
    template PCL_EXPORTS void computeWeirdPointNormals<typename Device<PointXYZRGB>::type::const_iterator, typename Device<float4>::type::iterator, Device>
                  (Device<PointXYZRGB>::type::const_iterator begin,
                   Device<PointXYZRGB>::type::const_iterator end,
                   Device<float4>::type::iterator output,
                   float focallength,
                   const PointCloudAOS<Device>::ConstPtr &input,
                   float radius,
                   int desired_number_neighbors);
    
    template PCL_EXPORTS void computeWeirdPointNormals<typename Host<PointXYZRGB>::type::const_iterator, typename Host<float4>::type::iterator, Host>
                  (Host<PointXYZRGB>::type::const_iterator begin,
                   Host<PointXYZRGB>::type::const_iterator end,
                   Host<float4>::type::iterator output,
                   float focallength,
                   const PointCloudAOS<Host>::ConstPtr &input,
                   float radius,
                   int desired_number_neighbors);
    
    template PCL_EXPORTS shared_ptr<typename Device<float4>::type> computeWeirdPointNormals<Device, typename PointIterator<Device,PointXYZRGB>::type >
                  (PointIterator<Device,PointXYZRGB>::type begin,
                   PointIterator<Device,PointXYZRGB>::type end,
                   float focallength,
                   const PointCloudAOS<Device>::ConstPtr &input,
                   float radius,
                   int desired_number_neighbors);
    
    template PCL_EXPORTS shared_ptr<typename Host<float4>::type> computeWeirdPointNormals<Host, typename PointIterator<Host,PointXYZRGB>::type >
                  (PointIterator<Host,PointXYZRGB>::type begin,
                   PointIterator<Host,PointXYZRGB>::type end,
                   float focallength,
                   const PointCloudAOS<Host>::ConstPtr &input,
                   float radius,
                   int desired_number_neighbors);

  } // namespace
} // namespace


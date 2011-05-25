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
 * $Id: io.h 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#include "pcl/cuda/point_cloud.h"
#include "pcl/cuda/io/extract_indices.h"
#include "pcl/cuda/io/predicate.h"
#include <thrust/copy.h>

namespace pcl
{
namespace cuda
{


template <template <typename> class Storage>
void extractIndices (const typename PointCloudAOS<Storage>::Ptr &input,
                               typename Storage<int>::type& indices, 
                               typename PointCloudAOS<Storage>::Ptr &output)
{
  if (!output)
    output.reset (new PointCloudAOS<Storage>);

  output->points.resize (input->points.size ());

  typename PointCloudAOS<Storage>::iterator it = thrust::copy_if (input->points.begin (), input->points.end (), indices.begin (), output->points.begin (), isInlier ());
  output->points.resize (it - output->points.begin ());

  output->width = output->points.size();
  output->height = 1;
  output->is_dense = false;
}

template <template <typename> class Storage>
void removeIndices  (const typename PointCloudAOS<Storage>::Ptr &input,
                               typename Storage<int>::type& indices, 
                               typename PointCloudAOS<Storage>::Ptr &output)
{
  if (!output)
    output.reset (new PointCloudAOS<Storage>);

  output->points.resize (input->points.size ());

  typename PointCloudAOS<Storage>::iterator it = thrust::copy_if (input->points.begin (), input->points.end (), indices.begin (), output->points.begin (), isNotInlier ());
  output->points.resize (it - output->points.begin ());

  output->width = output->points.size();
  output->height = 1;
  output->is_dense = false;
}

template <template <typename> class Storage>
void extractIndices (const typename PointCloudAOS<Storage>::Ptr &input,
               typename Storage<int>::type& indices, 
               typename PointCloudAOS<Storage>::Ptr &output, const OpenNIRGB& color)
{
  extractIndices<Storage> (input, indices, output);
  thrust::for_each ( output->points.begin(), output->points.end(), SetColor (color) );
}

template <template <typename> class Storage>
void removeIndices  (const typename PointCloudAOS<Storage>::Ptr &input,
               typename Storage<int>::type& indices, 
               typename PointCloudAOS<Storage>::Ptr &output, const OpenNIRGB& color)
{
  removeIndices<Storage> (input, indices, output);
  thrust::for_each ( output->points.begin(), output->points.end(), SetColor (color) );
}

template <template <typename> class Storage>
void colorIndices  (typename PointCloudAOS<Storage>::Ptr &input,
               boost::shared_ptr<typename Storage<int>::type> indices, 
               const OpenNIRGB& color)
{
  thrust::transform_if (input->points.begin (), input->points.end (), indices->begin (), input->points.begin (), ChangeColor (color), isInlier());
}


template void extractIndices<Host>(const PointCloudAOS<Host>::Ptr &input,
                                                       Host<int>::type& indices, 
                                                       PointCloudAOS<Host>::Ptr &output);
template void extractIndices<Device> (const PointCloudAOS<Device>::Ptr &input,
                                                          Device<int>::type& indices, 
                                                          PointCloudAOS<Device>::Ptr &output);

template void removeIndices<Host>(const PointCloudAOS<Host>::Ptr &input,
                                                       Host<int>::type& indices, 
                                                       PointCloudAOS<Host>::Ptr &output);
template void removeIndices<Device> (const PointCloudAOS<Device>::Ptr &input,
                                                          Device<int>::type& indices, 
                                                          PointCloudAOS<Device>::Ptr &output);

template void extractIndices<Host>(const PointCloudAOS<Host>::Ptr &input,
                                                       Host<int>::type& indices, 
                                                       PointCloudAOS<Host>::Ptr &output, const OpenNIRGB& color);
template void extractIndices<Device> (const PointCloudAOS<Device>::Ptr &input,
                                                          Device<int>::type& indices, 
                                                          PointCloudAOS<Device>::Ptr &output, const OpenNIRGB& color);

template void removeIndices<Host>(const PointCloudAOS<Host>::Ptr &input,
                                                       Host<int>::type& indices, 
                                                       PointCloudAOS<Host>::Ptr &output, const OpenNIRGB& color);
template void removeIndices<Device> (const PointCloudAOS<Device>::Ptr &input,
                                                          Device<int>::type& indices, 
                                                          PointCloudAOS<Device>::Ptr &output, const OpenNIRGB& color);

template void colorIndices<Host>(PointCloudAOS<Host>::Ptr &input,
                                                       boost::shared_ptr<Host<int>::type> indices, 
                                                       const OpenNIRGB& color);
template void colorIndices<Device> (PointCloudAOS<Device>::Ptr &input,
                                                          boost::shared_ptr<Device<int>::type> indices, 
                                                          const OpenNIRGB& color);
} // namespace
} // namespace


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
 * $Id$
 *
 */

#include <pcl/pcl_exports.h>

#include <pcl/cuda/point_cloud.h>
//#include <pcl/cuda/io/extract_indices.h>
#include <pcl/cuda/io/predicate.h>
#include <pcl/cuda/thrust.h>

namespace pcl
{
namespace cuda
{

template <template <typename> class Storage, class T>
void extractMask (const typename PointCloudAOS<Storage>::Ptr &input,
                        T* mask, 
                        typename PointCloudAOS<Storage>::Ptr &output)
{
  if (!output)
    output.reset (new PointCloudAOS<Storage>);

  output->points.resize (input->points.size ());

  typename Storage<T>::type mask_device (input->points.size());
  thrust::copy (mask, (T*)(&mask[input->points.size()]), mask_device.begin ());

  typename PointCloudAOS<Storage>::iterator it = thrust::copy_if (input->points.begin (), input->points.end (), mask_device.begin (), output->points.begin (), isNotZero<T> ());
  output->points.resize (it - output->points.begin ());

  output->width = (unsigned int) output->points.size();
  output->height = 1;
  output->is_dense = false;
}

template <template <typename> class Storage, class DataT, class MaskT>
void extractMask (const boost::shared_ptr<typename Storage<DataT>::type> &input,
                        MaskT* mask, 
                        boost::shared_ptr<typename Storage<DataT>::type> &output)
{
  if (!output)
    output.reset (new typename Storage<DataT>::type);
  output->resize (input->size ());

  typename Storage<MaskT>::type mask_device (input->size());
  thrust::copy (mask, (MaskT*)(&mask[input->size()]), mask_device.begin ());

  typename Storage<DataT>::type::iterator it = 
    thrust::copy_if (input->begin (), input->end (), mask_device.begin (), output->begin (), isNotZero<MaskT> ());
  output->resize (it - output->begin ());
}


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

  output->width = (unsigned int) output->points.size();
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

  output->width = (unsigned int) output->points.size();
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

struct ColorCloudFromImage
{
  ColorCloudFromImage (char4* colors) : colors_(colors)
  {}
  char4 * colors_;

  template <typename Tuple>
  inline __host__ __device__
  PointXYZRGB operator () (Tuple &t)
  {
    PointXYZRGB &pt = thrust::get<0>(t);
    char4 rgb = colors_[thrust::get<1>(t)];
    pt.rgb = ((unsigned char)rgb.x << 16) + ((unsigned char)rgb.y << 8) + (unsigned char)rgb.z;
    return pt;
  }
};


template <template <typename> class Storage>
void colorCloud  (typename PointCloudAOS<Storage>::Ptr &input,
                  typename Storage<char4>::type &colors)
{
  thrust::transform (thrust::make_zip_iterator(thrust::make_tuple (input->points.begin(), thrust::counting_iterator<int>(0))),
                     thrust::make_zip_iterator(thrust::make_tuple (input->points.begin(), thrust::counting_iterator<int>(0))) + input->width * input->height,
                     input->points.begin (), ColorCloudFromImage (thrust::raw_pointer_cast(&colors[0])));
}



template PCL_EXPORTS void extractIndices<Host>(const PointCloudAOS<Host>::Ptr &input,
                                                       Host<int>::type& indices, 
                                                       PointCloudAOS<Host>::Ptr &output);
template PCL_EXPORTS void extractIndices<Device> (const PointCloudAOS<Device>::Ptr &input,
                                                          Device<int>::type& indices, 
                                                          PointCloudAOS<Device>::Ptr &output);

template PCL_EXPORTS void removeIndices<Host>(const PointCloudAOS<Host>::Ptr &input,
                                                       Host<int>::type& indices, 
                                                       PointCloudAOS<Host>::Ptr &output);
template PCL_EXPORTS void removeIndices<Device> (const PointCloudAOS<Device>::Ptr &input,
                                                          Device<int>::type& indices, 
                                                          PointCloudAOS<Device>::Ptr &output);

template PCL_EXPORTS void extractIndices<Host>(const PointCloudAOS<Host>::Ptr &input,
                                                       Host<int>::type& indices, 
                                                       PointCloudAOS<Host>::Ptr &output, const OpenNIRGB& color);
template PCL_EXPORTS void extractIndices<Device> (const PointCloudAOS<Device>::Ptr &input,
                                                          Device<int>::type& indices, 
                                                          PointCloudAOS<Device>::Ptr &output, const OpenNIRGB& color);

template PCL_EXPORTS void removeIndices<Host>(const PointCloudAOS<Host>::Ptr &input,
                                                       Host<int>::type& indices, 
                                                       PointCloudAOS<Host>::Ptr &output, const OpenNIRGB& color);
template PCL_EXPORTS void removeIndices<Device> (const PointCloudAOS<Device>::Ptr &input,
                                                          Device<int>::type& indices, 
                                                          PointCloudAOS<Device>::Ptr &output, const OpenNIRGB& color);

template PCL_EXPORTS void colorIndices<Host> (PointCloudAOS<Host>::Ptr &input,
                                                       boost::shared_ptr<Host<int>::type> indices, 
                                                       const OpenNIRGB& color);
template PCL_EXPORTS void colorIndices<Device> (PointCloudAOS<Device>::Ptr &input,
                                                          boost::shared_ptr<Device<int>::type> indices, 
                                                          const OpenNIRGB& color);
template PCL_EXPORTS void colorCloud<Host>  (PointCloudAOS<Host>::Ptr &input, Host<char4>::type &colors);
template PCL_EXPORTS void colorCloud<Device>(PointCloudAOS<Device>::Ptr &input, Device<char4>::type &colors);

template PCL_EXPORTS 
void extractMask<Device,unsigned char> (const PointCloudAOS<Device>::Ptr &input, unsigned char* mask, PointCloudAOS<Device>::Ptr &output);
template PCL_EXPORTS 
void extractMask<Host,unsigned char> (const PointCloudAOS<Host>::Ptr &input, unsigned char* mask, PointCloudAOS<Host>::Ptr &output);
template PCL_EXPORTS
void extractMask<Device,float4,unsigned char> (const boost::shared_ptr<Device<float4>::type> &input,
                        unsigned char* mask, 
                        boost::shared_ptr<Device<float4>::type> &output);
template PCL_EXPORTS
void extractMask<Host,float4,unsigned char> (const boost::shared_ptr<Host<float4>::type> &input,
                        unsigned char* mask, 
                        boost::shared_ptr<Host<float4>::type> &output);

} // namespace
} // namespace


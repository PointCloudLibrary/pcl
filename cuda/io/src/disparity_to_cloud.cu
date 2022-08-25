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

#include "pcl/cuda/io/disparity_to_cloud.h"
#include "pcl/cuda/io/debayering.h"
#include "pcl/cuda/io/cloud_to_pcl.h"
#include "pcl/cuda/io/kinect_smoothing.h"

#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>

#include <thrust/iterator/constant_iterator.h>
#include <thrust/copy.h>
#include <string>
#include <iostream>
#include <fstream>

namespace pcl
{
namespace cuda
{


//////////////////////////////////////////////////////////////////////////
template <typename Tuple> PointXYZRGB
ComputeXYZ::operator () (const Tuple &t)
{
  PointXYZRGB pt;

  if (thrust::get<0>(t) == 0) // assume no_sample_value and shadow_value are also 0
    pt.x = pt.y = pt.z = bad_point;
  else
  {
    // Fill in XYZ (and copy NaNs with it)
    pt.x = ((thrust::get<1>(t) % width) - center_x) * thrust::get<0>(t) * constant * 0.001f;
    pt.y = ((thrust::get<1>(t) / width) - center_y) * thrust::get<0>(t) * constant * 0.001f;
    pt.z = thrust::get<0>(t) * 0.001f;
  }
  return (pt);
}

//////////////////////////////////////////////////////////////////////////
template <typename Tuple> PointXYZRGB
ComputeXYZRGB::operator () (const Tuple &t)
{
  PointXYZRGB pt;

  if (thrust::get<0>(t) == 0) // assume no_sample_value and shadow_value are also 0
    pt.x = pt.y = pt.z = bad_point;
  else
  {
    // Fill in XYZ (and copy NaNs with it)
    pt.x   = ((thrust::get<2>(t) % width) - center_x) * thrust::get<0>(t) * constant * 0.001f;
    pt.y   = ((thrust::get<2>(t) / width) - center_y) * thrust::get<0>(t) * constant * 0.001f;
    pt.z   = thrust::get<0>(t) * 0.001f;
  }
  pt.rgb = thrust::get<1>(t).r << 16 | thrust::get<1>(t).g << 8 | thrust::get<1>(t).b;
  return (pt);
}

//////////////////////////////////////////////////////////////////////////
//void
//DisparityToCloud::compute (const pcl::PCLImage::ConstPtr &depth_image,
//                                     const pcl::PCLImage::ConstPtr &rgb_image,
//                                     const pcl::CameraInfo::ConstPtr &info,
//                                     PointCloudAOS<Device>::Ptr &output) 
//{
//  if (!output)
//    output.reset (new PointCloudAOS<Device>);
//
//  // Prepare the output
//  output->height = depth_image->height;
//  output->width  = depth_image->width;
//  output->is_dense = false;
//  output->points.resize (output->width * output->height);
//
//  // Copy the depth data and the RGB data on the card
//  device_vector<float> depth (depth_image->data.size () / sizeof (float));
//  thrust::copy ((float*)(&depth_image->data[0]), (float*)(&depth_image->data[0]) + depth.size (), depth.begin ());
//
//  // Prepare constants
//  if (rgb_image)
//  {
//    device_vector<OpenNIRGB> rgb (rgb_image->width * rgb_image->height);
//    thrust::copy ((OpenNIRGB*)(&rgb_image->data[0]),
//                  (OpenNIRGB*)(&rgb_image->data[rgb_image->width * rgb_image->height * 3]), rgb.begin ());
//
//    // Suat: implement the rgb/depth stepping
//    assert (rgb.size () == depth.size ());
//
//    // Send the data to the device
//    transform (
//        thrust::make_zip_iterator (make_tuple (depth.begin (), rgb.begin (), thrust::counting_iterator<int>(0))),
//        thrust::make_zip_iterator (make_tuple (depth.begin (), rgb.begin (), thrust::counting_iterator<int>(0))) + 
//                           depth_image->width * depth_image->height,
//        output->points.begin (), 
//        ComputeXYZRGB (depth_image->width, depth_image->height, 
//                       depth_image->width >> 1, depth_image->height >> 1, 1.0 / info->P[0]));
//  }
//  else
//  {
//    // Send the data to the device
//    transform (
//        thrust::make_zip_iterator (make_tuple (depth.begin (), thrust::counting_iterator<int>(0))),
//        thrust::make_zip_iterator (make_tuple (depth.begin (), thrust::counting_iterator<int>(0))) + 
//                           depth_image->width * depth_image->height,
//        output->points.begin (), 
//        ComputeXYZ (depth_image->width, depth_image->height, 
//                    depth_image->width >> 1, depth_image->height >> 1, 1.0 / info->P[0]));
//  }
//
//}
//
////////////////////////////////////////////////////////////////////////////
//void
//DisparityToCloud::compute (const pcl::PCLImage::ConstPtr &depth_image,
//                                     const pcl::PCLImage::ConstPtr &rgb_image,
//                                     const pcl::CameraInfo::ConstPtr &info,
//                                     PointCloudAOS<Host>::Ptr &output) 
//{
//  if (!output)
//    output.reset (new PointCloudAOS<Host>);
//
//  PointCloudAOS<Device>::Ptr data;
//  compute (depth_image, rgb_image, info, data);
//  *output << *data;
//}

    template <typename T>
    void save_pgm (const T* depth, std::string filename, int w, int h)
    {   
      std::ofstream ofs (filename.c_str());
      ofs << "P2" << std::endl << w << " " << h << std::endl;
      T max_el = 0;
      for (int depthIdx = 0; depthIdx < h * w; ++depthIdx)
        if (depth[depthIdx] > max_el)
          max_el = depth[depthIdx];
      ofs << (int)max_el << std::endl;
      for (int yIdx = 0; yIdx < h; ++yIdx)
      {   
        for (int xIdx = 0; xIdx < w; ++xIdx)
        {   
          ofs << (int)depth[w*yIdx+xIdx] << " ";
        }   
        ofs << std::endl;
      }   
    }   

//////////////////////////////////////////////////////////////////////////
template <template <typename> class Storage> void
DisparityToCloud::compute (const std::uint16_t* depth_image,
                           const OpenNIRGB* rgb_image,
                           int width, int height,
                           float constant,
                           typename PointCloudAOS<Storage>::Ptr &output,
                           int smoothing_nr_iterations, int smoothing_filter_size) 
{
  if (!output)
    output.reset (new PointCloudAOS<Storage>);

  int output_size = width * height;
  float baseline = 0.075f;
  float disp_thresh = 0.001f/8.0f;

  output->height = height;
  output->width  = width;

  output->is_dense = false;
  output->points.resize (output_size);

  // copy to float
  typename Storage<float>::type depth (output_size);
  copy (depth_image, (const unsigned short*)(&depth_image[output_size]), depth.begin ());

  typename Storage<OpenNIRGB>::type rgb (output_size);
  copy (rgb_image, (const OpenNIRGB*)(&rgb_image[output_size]), rgb.begin());

  if (smoothing_nr_iterations > 0)
  {
    typename Storage<float3>::type disp_helper_map (output_size);

    float* depth_ptr = thrust::raw_pointer_cast(&depth[0]);

    transform (thrust::counting_iterator<int>(0),
               thrust::counting_iterator<int>(0) + output_size,
               disp_helper_map.begin (), 
               DisparityHelperMap (depth_ptr, width, height, smoothing_filter_size, baseline, 1.0f/constant, disp_thresh));

    for (int iter = 0; iter < smoothing_nr_iterations; iter++)
    {
      transform (
          thrust::make_zip_iterator (make_tuple (depth.begin (), thrust::counting_iterator<int>(0))),
          thrust::make_zip_iterator (make_tuple (depth.begin (), thrust::counting_iterator<int>(0))) + output_size,
          depth.begin (), DisparityClampedSmoothing (thrust::raw_pointer_cast(&depth[0]), thrust::raw_pointer_cast(&disp_helper_map[0]), width, height, smoothing_filter_size));
    }

    // Send the data to the device
    transform (
        thrust::make_zip_iterator (make_tuple (depth.begin (), rgb.begin(), thrust::counting_iterator<int>(0))),
        thrust::make_zip_iterator (make_tuple (depth.begin (), rgb.begin(), thrust::counting_iterator<int>(0))) + output_size,
        output->points.begin (), 
        ComputeXYZRGB (width, height, 
                       width >> 1, height >> 1, constant));
  }
  else
  {
    transform (
        thrust::make_zip_iterator (make_tuple (depth.begin(), rgb.begin(), thrust::counting_iterator<int>(0))),
        thrust::make_zip_iterator (make_tuple (depth.begin(), rgb.begin(), thrust::counting_iterator<int>(0))) + output_size,
        output->points.begin (), 
        ComputeXYZRGB (width, height, 
                       width >> 1, height >> 1, constant));
  }
}

//////////////////////////////////////////////////////////////////////////
template <template <typename> class Storage> void
DisparityToCloud::compute (const openni_wrapper::DepthImage::Ptr& depth_image,
                                     const openni_wrapper::Image::Ptr& rgb_image,
                                     float constant,
                                     typename PointCloudAOS<Storage>::Ptr &output,
                                     bool downsample, int stride, int smoothing_nr_iterations, int smoothing_filter_size) 
{
  if (!output)
    output.reset (new PointCloudAOS<Storage>);

  int depth_width = depth_image->getWidth ();
  int depth_height = depth_image->getHeight ();

  // Prepare the output
  if (downsample)
  {
    output->height = depth_height / stride;
    output->width  = depth_width / stride;
    constant *= stride;
  }
  else
  {
    output->height = depth_height;
    output->width  = depth_width;
  }

  output->is_dense = false;
  output->points.resize (output->width * output->height);

  // Copy the depth data and the RGB data on the card
  typename Storage<float>::type depth (output->width * output->height);
  unsigned short* depth_buffer = (unsigned short*)depth_image->getDepthMetaData ().Data ();

  if (downsample)
  {
    typename Storage<float>::type depth_device (depth_width * depth_height);
    // first, copy from host to Storage
    thrust::copy (depth_buffer, (unsigned short*)(&depth_buffer[depth_width * depth_height]), depth_device.begin ());
    
    thrust::counting_iterator<int> counter (0);
    //typename Storage<int>::type downsampled_indices;
    //downsampled_indices.resize ((output->width/2) * (output->height/2));
    //thrust::copy_if (thrust::counting_iterator<int>(0), thrust::counting_iterator<int>(0)+depth_width*depth_height, downsampled_indices.begin (), downsampleIndices (output->width, output->height, 2));
    thrust::copy_if (depth_device.begin (), 
                    depth_device.end (),
                    //thrust::make_constant_iterator (12), thrust::make_constant_iterator (12) + depth_width * depth_height,
                    counter, 
                    depth.begin (),
                    downsampleIndices (depth_width, depth_height, stride));
    //host_vector<float> h_depth = depth;
    //save_pgm<float> ((float*)&(h_depth[0]), "outpoutbla", output->width, output->height);
  }
  else
    thrust::copy (depth_buffer, (unsigned short*)(&depth_buffer[output->width * output->height]), depth.begin ());

  // Prepare constants
  if (rgb_image)
  {
    int image_width = rgb_image->getWidth ();
    int image_height = rgb_image->getHeight ();
    if (downsample)
    {
      image_width /= stride;
      image_height /= stride;
    }

    typename Storage<OpenNIRGB>::type rgb (image_width * image_height);


    if (rgb_image->getEncoding () == openni_wrapper::Image::BAYER_GRBG)
    {
      if (downsample)
      {
        DebayeringDownsampling<Storage> debayering;
        debayering.compute (rgb_image, rgb);
      }
      else
      {
        Debayering<Storage> debayering;
        debayering.computeBilinear (rgb_image, rgb);
      }
    }
    else if (rgb_image->getEncoding () == openni_wrapper::Image::YUV422)
    {
      YUV2RGB<Storage> yuv;
      yuv.compute (rgb_image, rgb);
//      OpenNIRGB c;
//      c.r = c.g = c.b = (unsigned char) 128;
//      thrust::fill (rgb.begin (), rgb.end (), c);
    }


    int output_size = output->width * output->height;
    float baseline = 0.075f;
    float disp_thresh = 0.001f/8.0f;

    if (smoothing_nr_iterations > 0)
    {
#if 1
    typename Storage<float3>::type disp_helper_map (output_size);

    transform (thrust::counting_iterator<int>(0),
               thrust::counting_iterator<int>(0) + output_size,
               disp_helper_map.begin (), 
               DisparityHelperMap (thrust::raw_pointer_cast(&depth[0]), output->width, output->height, smoothing_filter_size, baseline, 1.0f/constant, disp_thresh));

    for (int iter = 0; iter < smoothing_nr_iterations; iter++)
    {
      transform (
          thrust::make_zip_iterator (make_tuple (depth.begin (), thrust::counting_iterator<int>(0))),
          thrust::make_zip_iterator (make_tuple (depth.begin (), thrust::counting_iterator<int>(0))) + output_size,
          depth.begin (), DisparityClampedSmoothing (thrust::raw_pointer_cast(&depth[0]), thrust::raw_pointer_cast(&disp_helper_map[0]), output->width, output->height, smoothing_filter_size));
    }

    // Send the data to the device
    transform (
        thrust::make_zip_iterator (make_tuple (depth.begin (), rgb.begin (), thrust::counting_iterator<int>(0))),
        thrust::make_zip_iterator (make_tuple (depth.begin (), rgb.begin (), thrust::counting_iterator<int>(0))) + output_size,
        output->points.begin (), 
        ComputeXYZRGB (output->width, output->height, 
                       output->width >> 1, output->height >> 1, constant));
#else 
//      typename Storage<float>::type smooth_depth1 (output_size);
//      typename Storage<float>::type smooth_depth2 (output_size);
//
//      transform (thrust::counting_iterator<int>(0),
//                 thrust::counting_iterator<int>(0) + output_size,
//                 smooth_depth1.begin (),
//                 DisparityBoundSmoothing (output->width, output->height, smoothing_filter_size, 1.0f/constant, baseline, disp_thresh, thrust::raw_pointer_cast<float>(&depth[0]), thrust::raw_pointer_cast<float>(&depth[0])));
//
//      for (int iter = 0; iter < (smoothing_nr_iterations-1)/2; iter++)
//      {
//          transform (thrust::counting_iterator<int>(0),
//                     thrust::counting_iterator<int>(0) + output_size,
//                     smooth_depth2.begin (),
//                     DisparityBoundSmoothing (output->width, output->height, smoothing_filter_size, 1.0f/constant, baseline, disp_thresh, thrust::raw_pointer_cast<float>(&smooth_depth1[0]), thrust::raw_pointer_cast<float>(&depth[0])));
//          transform (thrust::counting_iterator<int>(0),
//                     thrust::counting_iterator<int>(0) + output_size,
//                     smooth_depth1.begin (),
//                     DisparityBoundSmoothing (output->width, output->height, smoothing_filter_size, 1.0f/constant, baseline, disp_thresh, thrust::raw_pointer_cast<float>(&smooth_depth2[0]), thrust::raw_pointer_cast<float>(&depth[0])));
//      }
//      // Suat: implement the rgb/depth stepping
//      //assert (rgb.size () == depth.size ());
//
//      // Send the data to the device
//      transform (
//          thrust::make_zip_iterator (make_tuple (smooth_depth1.begin (), rgb.begin (), thrust::counting_iterator<int>(0))),
//          thrust::make_zip_iterator (make_tuple (smooth_depth1.begin (), rgb.begin (), thrust::counting_iterator<int>(0))) + output_size,
//          output->points.begin (), 
//          ComputeXYZRGB (output->width, output->height, 
//                         output->width >> 1, output->height >> 1, constant));
#endif
    }
    else
    {
      // Send the data to the device
      transform (
          thrust::make_zip_iterator (make_tuple (depth.begin (), rgb.begin (), thrust::counting_iterator<int>(0))),
          thrust::make_zip_iterator (make_tuple (depth.begin (), rgb.begin (), thrust::counting_iterator<int>(0))) + output_size,
          output->points.begin (), 
          ComputeXYZRGB (output->width, output->height, 
                         output->width >> 1, output->height >> 1, constant));
    }
  }
  else
  {
    // Send the data to the device
    transform (
        thrust::make_zip_iterator (make_tuple (depth.begin (), thrust::counting_iterator<int>(0))),
        thrust::make_zip_iterator (make_tuple (depth.begin (), thrust::counting_iterator<int>(0))) + 
                           output->width * output->height,
        output->points.begin (), 
        ComputeXYZ (output->width, output->height, 
                    output->width >> 1, output->height >> 1, constant));
  }

}

//////////////////////////////////////////////////////////////////////////
/*void
DisparityToCloud::compute (const openni_wrapper::DepthImage::Ptr& depth_image,
                                     const openni_wrapper::Image::Ptr& rgb_image,
                                     float constant,
                                     PointCloudAOS<Host>::Ptr &output) 
{
  if (!output)
    output.reset (new PointCloudAOS<Host>);

  PointCloudAOS<Device>::Ptr data;
  compute (depth_image, rgb_image, constant, data);
  *output << *data;
}*/

//////////////////////////////////////////////////////////////////////////
//void
//DisparityToCloud::compute (const pcl::PCLImage::ConstPtr &depth_image,
//                                     const pcl::CameraInfo::ConstPtr &info,
//                                     PointCloudAOS<Device>::Ptr &output) 
//{
//  if (!output)
//    output.reset (new PointCloudAOS<Device>);
//
//  compute (depth_image, pcl::PCLImage::ConstPtr(), info, output);
//}
//
////////////////////////////////////////////////////////////////////////////
//void
//DisparityToCloud::compute (const pcl::PCLImage::ConstPtr &depth_image,
//                                     const pcl::CameraInfo::ConstPtr &info,
//                                     PointCloudAOS<Host>::Ptr &output) 
//{
//  if (!output)
//    output.reset (new PointCloudAOS<Host>);
//
//  PointCloudAOS<Device>::Ptr data;
//  compute (depth_image, info, data);
//  *output << *data;
//}

//////////////////////////////////////////////////////////////////////////
void
DisparityToCloud::compute (const openni_wrapper::DepthImage::Ptr& depth_image,
                                     float constant,
                                     PointCloudAOS<Device>::Ptr &output)
{
  if (!output)
    output.reset (new PointCloudAOS<Device>);

  compute<Device> (depth_image, openni_wrapper::Image::Ptr(), constant, output, false);
}

//////////////////////////////////////////////////////////////////////////
void
DisparityToCloud::compute (const openni_wrapper::DepthImage::Ptr& depth_image,
                                     float constant,
                                     PointCloudAOS<Host>::Ptr &output)
{
  if (!output)
    output.reset (new PointCloudAOS<Host>);

  compute<Host> (depth_image, openni_wrapper::Image::Ptr(), constant, output, false);
  //PointCloudAOS<Device>::Ptr data;
  //compute (depth_image, constant, data);
  //*output << *data;
}

template PCL_EXPORTS void
DisparityToCloud::compute<Host> (const openni_wrapper::DepthImage::Ptr& depth_image,
                                     const openni_wrapper::Image::Ptr& rgb_image,
                                     float constant,
                                     PointCloudAOS<Host>::Ptr &output,
                                     bool downsample, int stride, int, int);
template PCL_EXPORTS void
DisparityToCloud::compute<Device> (const openni_wrapper::DepthImage::Ptr& depth_image,
                                     const openni_wrapper::Image::Ptr& rgb_image,
                                     float constant,
                                     PointCloudAOS<Device>::Ptr &output,
                                     bool downsample, int stridem, int, int);
template PCL_EXPORTS void
DisparityToCloud::compute<Host> (const std::uint16_t* depth_image,
                                 const OpenNIRGB* rgb_image,
                                 int width, int height,
                                 float constant,
                                 typename PointCloudAOS<Host>::Ptr &output,
                                 int smoothing_nr_iterations, int smoothing_filter_size);
template PCL_EXPORTS void
DisparityToCloud::compute<Device> (const std::uint16_t* depth_image,
                                   const OpenNIRGB* rgb_image,
                                   int width, int height,
                                   float constant,
                                   typename PointCloudAOS<Device>::Ptr &output,
                                   int smoothing_nr_iterations, int smoothing_filter_size);
} // namespace
} // namespace


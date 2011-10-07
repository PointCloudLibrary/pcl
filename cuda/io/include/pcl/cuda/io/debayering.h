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

#ifndef PCL_CUDA_DEBAYERING_H_
#define PCL_CUDA_DEBAYERING_H_

#include <pcl/cuda/point_cloud.h>
#include <pcl/io/openni_camera/openni_image.h>

namespace pcl
{
  namespace cuda
  {

    struct downsampleIndices
    {
      downsampleIndices (int width, int height, int stride)
        : width (width), height (height), stride (stride)
      {}
  
      int width, height, stride;
  
      __host__ __device__
      bool operator () (int i)
      {
        int xIdx = i % width;
        int yIdx = i / width;
  
        return ((xIdx % stride == 0) & (yIdx % stride == 0));
      }
    };
  
    template <template <typename> class Storage>
    struct DebayerBilinear
    {
  		unsigned width;
  		unsigned height;
  		//static unsigned dataSize;
  		//static unsigned char* global_data; // has to be initialized only once!
  		//unsigned char* data;
  		unsigned char *data;
  		DebayerBilinear (unsigned char *bayer_image, unsigned width, unsigned height);
  		//DebayerBilinear (const boost::shared_ptr<openni_wrapper::Image>& bayer_image);
  
      __inline__ __host__ __device__ OpenNIRGB operator () (int index) const;
    };
  /*
  	struct DebayerEdgeAware
    {
  		unsigned width;
  		unsigned height;
  		static unsigned dataSize;
  		static unsigned char* global_data; // has to be initialized only once!
  		unsigned char* data;
      DebayerEdgeAware (const boost::shared_ptr<openni_wrapper::Image>& bayer_image);
      ~DebayerEdgeAware ();
  
      __inline__ __host__ __device__ OpenNIRGB operator () (int index) const;
    };
    */
    template<template <typename> class Storage>
    class DebayeringDownsampling
    {
      public:
        typedef typename Storage<OpenNIRGB>::type RGBImageType;
        void
        compute (const boost::shared_ptr<openni_wrapper::Image>& bayer_image, RGBImageType& rgb_image) const;
    };

    template <template <typename> class Storage>
    struct YUV2RGBKernel
    {
  		unsigned width;
  		unsigned height;
  		unsigned char *data;
  		YUV2RGBKernel (unsigned char *yuv_image, unsigned width, unsigned height);
  
      __inline__ __host__ __device__ OpenNIRGB operator () (int index) const;
    };
 
    template<template <typename> class Storage>
    class YUV2RGB
    {
      public:
        typedef typename Storage<OpenNIRGB>::type RGBImageType;
        void
        compute (const boost::shared_ptr<openni_wrapper::Image>& yuv_image, RGBImageType& rgb_image) const;
    };

    template<template <typename> class Storage>
    class Debayering
    {
      public:
        typedef typename Storage<OpenNIRGB>::type RGBImageType;
        void
        computeBilinear (const boost::shared_ptr<openni_wrapper::Image>& bayer_image, RGBImageType& rgb_image) const;
        
        //void
        //computeEdgeAware (const boost::shared_ptr<openni_wrapper::Image>& bayer_image, thrust::host_vector<OpenNIRGB>& rgb_image) const;
        
        //void
        //computeEdgeAware (const boost::shared_ptr<openni_wrapper::Image>& bayer_image, thrust::device_vector<OpenNIRGB>& rgb_image) const;
    };

  } // namespace
} // namespace

#endif

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

#include <pcl/cuda/io/debayering.h>
#include <iostream>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/thrust.h>

#define ABS(x) ((x)>0)?(x):(-(x))
namespace pcl
{
  namespace cuda
  {
    template <template <typename> class Storage>
    YUV2RGBKernel<Storage>::YUV2RGBKernel
    (unsigned char *yuv_image, unsigned width, unsigned height)
      : width(width), height(height), data(yuv_image)
    {
    }
  
    template <template <typename> class Storage>
    OpenNIRGB YUV2RGBKernel<Storage>::operator () (int index) const
    {
      //TODO: assert width/height is modulo 2

      int yuv_index = index * 2; // actual index in byte array

      int u, v;
      if (index % 2)
      {
        u = data[yuv_index + 0] - 128; // current pixel has u value
        v = data[yuv_index + 2] - 128; // get v from next pixel
      }
      else
      {
        u = data[yuv_index - 2] - 128; // get u from last pixel
        v = data[yuv_index + 0] - 128; // current pixel has v value
      }

      int y = data[yuv_index + 1];
      
#define CLIP_CHAR(c) ((c)>255?255:(c)<0?0:(c))
    	OpenNIRGB result;
      result.b =  CLIP_CHAR (y + ((v * 18678 + 8192 ) >> 14));
      result.g =  CLIP_CHAR (y + ((v * -9519 - u * 6472 + 8192 ) >> 14));
      result.r =  CLIP_CHAR (y + ((u * 33292 + 8192 ) >> 14));

    	return result;
    }

    template <template <typename> class Storage>
    DebayerBilinear<Storage>::DebayerBilinear
    (unsigned char *bayer_image, unsigned width, unsigned height)
      : width(width), height(height), data(bayer_image)
    {/*
      if (dataSize < bayer_image->getWidth () * bayer_image->getHeight ())
    	{
    		if (!global_data)
    			cudaFree (global_data);
    		dataSize = bayer_image->getWidth () * bayer_image->getHeight ();
    		cudaError_t status = cudaMalloc( &data, dataSize );
    		if (status != cudaSuccess )	std::cout << "malloc failed!" << std::endl;
    		
    		data = global_data;
    	}
    	*/
    	
    /*	thrust::device_vector<unsigned char>* data;
    	cudaError_t status = cudaMemcpy(data, (void*)(bayer_image->getMetaData().Data()), bayer_image->getMetaData().DataSize(), cudaMemcpyHostToDevice);
    	if (status != cudaSuccess )
    	{
    		 std::cout << "memcpy failed! : " << status << std::endl;
    		 //std::cout << cudaErrorInvalidValue << " , " << cudaErrorInvalidDevicePointer << " , " << cudaErrorInvalidMemcpyDirection << std::endl;
    	}*/
    }
    
    template <template <typename> class Storage>
    OpenNIRGB DebayerBilinear<Storage>::operator () (int index) const
    {
    	// get position
    	int xIdx = index % width;
    	int yIdx = index / width;
    	
    	OpenNIRGB result;
      
    //  result.r = result.g = result.b = 0;
    //  return result;
      
    	// ToDo: need to be handled later on!
    	if (xIdx == 0 || xIdx == width-1 || yIdx == 0 || yIdx == height-1)
    	{
    		result.r = result.g = result.b = 0;
    		return result;
    	}
    	
    	if (yIdx & 1) // We are in the Blue line
    	{
    		if (xIdx & 1) // Odd -> We start with GBGBGBGBG
    		{
    			result.r = (unsigned char) ((data [index - width] + data [index + width]) >> 1);
    			result.g = data [index];
    			result.b = (data [index - 1] + data [index + 1]) >> 1;
    		}
    		else // BGBGBGBG
    		{
    			result.r = (data [index - width - 1] + data [index - width + 1] + data [index + width - 1] + data [index + width + 1] ) >> 2;
    			result.g = (data [index - 1] + data [index + 1] + data [index - width] + data [index + width]) >> 2;
    			result.b = data [index];
    		}
    	}
    	else // We are in the Red line
    	{
    		if (xIdx & 1) // Odd -> We start with RGRGRG
    		{
    			result.r = data [index];
    			result.g = (data [index - 1] + data [index + 1] + data [index - width] + data [index + width]) >> 2;
    			result.b = (data [index - width - 1] + data [index - width + 1] + data [index + width - 1] + data [index + width + 1] ) >> 2;
    		}
    		else // GRGRGR
    		{
    			result.r = (data [index - 1] + data [index + 1]) >> 1;
    			result.g = data [index];
    			result.b = (data [index - width] + data [index + width]) >> 1;
    		}
    	}
    	
    	return result;
    }
    
    /*unsigned char* DebayerEdgeAware::global_data = 0;
    unsigned DebayerEdgeAware::dataSize = 0;
    
    DebayerEdgeAware::DebayerEdgeAware (const boost::shared_ptr<openni_wrapper::Image>& bayer_image)
    {
    	if (dataSize < bayer_image->getWidth () * bayer_image->getHeight ())
    	{
    		if (!global_data)
    			cudaFree (global_data);
    		dataSize = bayer_image->getWidth () * bayer_image->getHeight ();
    		cudaError_t status = cudaMalloc( &data, dataSize );
    		if (status != cudaSuccess )	std::cout << "malloc failed!" << std::endl;
    		
    		data = global_data;
    	}
    	
    	cudaError_t status = cudaMemcpy(data, (void*)(bayer_image->getMetaData().Data()), bayer_image->getMetaData().DataSize(), cudaMemcpyHostToDevice);
    	if (status != cudaSuccess )
    	{
    		 std::cout << "memcpy failed! : " << status << std::endl;
    		 //std::cout << cudaErrorInvalidValue << " , " << cudaErrorInvalidDevicePointer << " , " << cudaErrorInvalidMemcpyDirection << std::endl;
    	}
    }
    
    DebayerEdgeAware::~DebayerEdgeAware ()
    {
    	//cudaFree (data);
    }
    
    OpenNIRGB DebayerEdgeAware::operator () (int index) const
    {
    	// get position
    	int xIdx = index % width;
    	int yIdx = index / width;
    	
    	OpenNIRGB result;
      
    	// ToDo: need to be handled later on!
    	if (xIdx == 0 || xIdx == width-1 || yIdx == 0 || yIdx == height-1)
    	{
    		result.r = result.g = result.b = 0;
    		return result;
    	}
    	
    	if (yIdx & 1) // We are in the Blue line
    	{
    		if (xIdx & 1) // Odd -> We start with GBGBGBGBG
    		{
    			result.r = (data [index - width] + data [index + width]) >> 1;
    			result.g = data [index];
    			result.b = (data [index - 1] + data [index + 1]) >> 1;
    		}
    		else // BGBGBGBG
    		{
    			result.r = (data [index - width - 1] + data [index - width + 1] + data [index + width - 1] + data [index + width + 1] ) >> 2;
    			
    			if ( ABS(data [index - 1] - data [index + 1]) > ABS(data [index - width] - data [index + width] ) )
    				result.g = (data [index - width] + data [index + width]) >> 1;
    			else
    				result.g = (data [index - 1] + data [index + 1]) >> 1;
    			result.b = data [index];
    		}
    	}
    	else // We are in the Red line
    	{
    		if (xIdx & 1) // Odd -> We start with RGRGRG
    		{
    			result.r = data [index];
    			if ( ABS(data [index - 1] - data [index + 1]) > ABS(data [index - width] - data [index + width]) )
    				result.g = (data [index - width] + data [index + width]) >> 1;
    			else
    			  result.g = (data [index - 1] + data [index + 1]) >> 1;
    			result.b = (data [index - width - 1] + data [index - width + 1] + data [index + width - 1] + data [index + width + 1] ) >> 2;
    		}
    		else // GRGRGR
    		{
    			result.r = (data [index - 1] + data [index + 1]) >> 1;
    			result.g = data [index];
    			result.b = (data [index - width] + data [index + width]) >> 1;
    		}
    	}
    	
    	return result;
    }*/
    
    template<template <typename> class Storage>
    void Debayering<Storage>::computeBilinear (const boost::shared_ptr<openni_wrapper::Image>& bayer_image, RGBImageType& rgb_image) const
    {
    	//pcl::ScopeTime t ("computeBilinear");
    	typename Storage<unsigned char>::type bayer_data (bayer_image->getWidth () * bayer_image->getHeight ());
    //	thrust::device_vector<unsigned char> bayer_data (bayer_image->getWidth () * bayer_image->getHeight ());
    	thrust::copy ((unsigned char*)(bayer_image->getMetaData().Data()),
    	              (unsigned char*)(bayer_image->getMetaData().Data() + bayer_image->getMetaData().DataSize()), 
    	              bayer_data.begin ());
    //	thrust::device_vector<int> indices(bayer_image->getWidth () * bayer_image->getHeight ());
    //	thrust::sequence (indices.begin(), indices.end() );
    //	thrust::transform (indices.begin(),
    //                     indices.end(),
    //                     rgb_image.begin(),
    //                     DebayerBilinear<Device> (bayer_data, bayer_image->getWidth (), bayer_image->getHeight ()));
    	thrust::counting_iterator<int> first (0);
    	thrust::transform (first,
      	                 first + (int)(bayer_image->getWidth () * bayer_image->getHeight ()),
    	                   rgb_image.begin(),
    	                   DebayerBilinear<Storage> (thrust::raw_pointer_cast (&bayer_data[0]), bayer_image->getWidth (), bayer_image->getHeight ()) );
    }

    template<template <typename> class Storage>
    void YUV2RGB<Storage>::compute (const boost::shared_ptr<openni_wrapper::Image>& yuv_image, RGBImageType& rgb_image) const
    {
    	typename Storage<unsigned char>::type yuv_data (yuv_image->getMetaData().DataSize());
    	thrust::copy ((unsigned char*)(yuv_image->getMetaData().Data()),
    	              (unsigned char*)(yuv_image->getMetaData().Data() + yuv_image->getMetaData().DataSize()), 
    	              yuv_data.begin ());
    	thrust::counting_iterator<int> first (0);
    	thrust::transform (first,
      	                 first + (int)(yuv_image->getWidth () * yuv_image->getHeight ()),
    	                   rgb_image.begin(),
    	                   YUV2RGBKernel<Storage> (thrust::raw_pointer_cast (&yuv_data[0]), yuv_image->getWidth (), yuv_image->getHeight ()) );
    }
    
    template <template <typename> class Storage>
    struct DebayerDownsample
    {
      DebayerDownsample (unsigned char *bayer_image, unsigned width, unsigned stride)
        : width(width), stride(stride), data(bayer_image) {}
    
      int width, height, stride;
      unsigned char *data;
    
      __host__ __device__
      OpenNIRGB operator() (int i)
      {
        OpenNIRGB result;
        result.r = data [i+1];
        result.g = (data [i] + data[i + width + 1]) / 2;
        result.b = data [i + width]; 
        return result;
      }
        
    };
    
    
    template<template <typename> class Storage>
    void DebayeringDownsampling<Storage>::compute (const boost::shared_ptr<openni_wrapper::Image>& bayer_image, RGBImageType& rgb_image) const
    {
    	//pcl::ScopeTime t ("computeBilinear");
    	typename Storage<unsigned char>::type bayer_data (bayer_image->getWidth () * bayer_image->getHeight ());
    //	thrust::device_vector<unsigned char> bayer_data (bayer_image->getWidth () * bayer_image->getHeight ());
    	thrust::copy ((unsigned char*)(bayer_image->getMetaData().Data()),
    	              (unsigned char*)(bayer_image->getMetaData().Data() + bayer_image->getMetaData().DataSize()), 
    	              bayer_data.begin ());
    
    	thrust::counting_iterator<int> first (0);
      typename Storage<int>::type downsampled_indices;
      downsampled_indices.resize ((bayer_image->getWidth()/2) * (bayer_image->getHeight()/2));
      thrust::copy_if (first,
                       first + (int)(bayer_image->getWidth () * bayer_image->getHeight ()),
                       downsampled_indices.begin(),
                       downsampleIndices(bayer_image->getWidth (), bayer_image->getHeight (), 2));
             
    	thrust::transform (downsampled_indices.begin (),
      	                 downsampled_indices.end (),
    	                   rgb_image.begin(),
    	                   DebayerDownsample<Storage> (thrust::raw_pointer_cast (&bayer_data[0]), bayer_image->getWidth (), 2) );
    }
    /*
    void Debayering::computeEdgeAware (const boost::shared_ptr<openni_wrapper::Image>& bayer_image, thrust::host_vector<OpenNIRGB>& rgb_image) const
    {
    	thrust::device_vector<OpenNIRGB> image (bayer_image->getWidth () * bayer_image->getHeight ()) ;
    	computeEdgeAware (bayer_image, image );
    	rgb_image = image;
    }
    
    void Debayering::computeEdgeAware (const boost::shared_ptr<openni_wrapper::Image>& bayer_image, thrust::device_vector<OpenNIRGB>& rgb_image) const
    {
    	//pcl::ScopeTime t ("computeEdgeAware");
    	thrust::device_vector<int> indices(bayer_image->getWidth () * bayer_image->getHeight ());
    	thrust::sequence( indices.begin(), indices.end() );
    	thrust::transform ( indices.begin(), indices.end(), rgb_image.begin(), DebayerEdgeAware (bayer_image) );
    }*/
    
    template class PCL_EXPORTS YUV2RGB<Device>;
    template class PCL_EXPORTS YUV2RGB<Host>;
    template class PCL_EXPORTS Debayering<Device>;
    template class PCL_EXPORTS Debayering<Host>;
    template class PCL_EXPORTS DebayeringDownsampling<Device>;
    template class PCL_EXPORTS DebayeringDownsampling<Host>;
  
  } // namespace
} // namespace


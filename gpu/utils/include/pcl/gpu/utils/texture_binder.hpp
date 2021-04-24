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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#ifndef PCL_GPU_UTILS_TEXTURE_BINDER_HPP_
#define PCL_GPU_UTILS_TEXTURE_BINDER_HPP_

#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/containers/device_array.h>

namespace pcl
{
  namespace gpu
  {
    class TextureBinder
    {
    public:        
      template<class T, enum cudaTextureReadMode readMode>
      TextureBinder(const DeviceArray2D<T>& arr, const struct texture<T, 2, readMode>& tex) : texref(&tex)
      {
        cudaChannelFormatDesc desc = cudaCreateChannelDesc<T>();  
        cudaSafeCall( cudaBindTexture2D(0, tex, arr.ptr(), desc, arr.cols(), arr.rows(), arr.step()) );        
      }

      template<class T, enum cudaTextureReadMode readMode>
      TextureBinder(const DeviceArray<T>& arr, const struct texture<T, 1, readMode> &tex) : texref(&tex)
      {
        cudaChannelFormatDesc desc = cudaCreateChannelDesc<T>();  
        cudaSafeCall( cudaBindTexture(0, tex, arr.ptr(), desc, arr.sizeBytes()) );
      }

      template<class T, enum cudaTextureReadMode readMode>
      TextureBinder(const PtrStepSz<T>& arr, const struct texture<T, 2, readMode>& tex) : texref(&tex)
      {
        cudaChannelFormatDesc desc = cudaCreateChannelDesc<T>();  
        cudaSafeCall( cudaBindTexture2D(0, tex, arr.data, desc, arr.cols, arr.rows, arr.step) );        
      }

      template<class T, enum cudaTextureReadMode readMode>
      TextureBinder(const PtrSz<T>& arr, const struct texture<T, 1, readMode> &tex) : texref(&tex)
      {
        cudaChannelFormatDesc desc = cudaCreateChannelDesc<T>();  
        cudaSafeCall( cudaBindTexture(0, tex, arr.data, desc, arr.size * arr.elemSize()) );
      }

      ~TextureBinder()
      {
        cudaSafeCall( cudaUnbindTexture(texref) );
      }
    private:
      const struct textureReference *texref;    
    };
  }

  namespace device
  {
      using pcl::gpu::TextureBinder;
  }
}

#endif /* PCL_GPU_UTILS_TEXTURE_BINDER_HPP_*/
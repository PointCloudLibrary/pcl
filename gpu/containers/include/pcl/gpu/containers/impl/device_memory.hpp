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

#ifndef PCL_GPU_CONTAINER_DEVICE_MEMORY_IMPL_HPP_
#define PCL_GPU_CONTAINER_DEVICE_MEMORY_IMPL_HPP_

/////////////////////  Inline implementations of DeviceMemory ////////////////////////////////////////////

template<class T> inline       T* pcl::gpu::DeviceMemory::ptr()       { return (      T*)data_; }
template<class T> inline const T* pcl::gpu::DeviceMemory::ptr() const { return (const T*)data_; }
                        
template <class U> inline pcl::gpu::DeviceMemory::operator pcl::gpu::PtrSz<U>() const
{
    PtrSz<U> result;
    result.data = (U*)ptr<U>();
    result.size = sizeBytes_/sizeof(U);
    return result; 
}

/////////////////////  Inline implementations of DeviceMemory2D ////////////////////////////////////////////
               
template<class T>        T* pcl::gpu::DeviceMemory2D::ptr(int y_arg)       { return (      T*)((      char*)data_ + y_arg * step_); }
template<class T>  const T* pcl::gpu::DeviceMemory2D::ptr(int y_arg) const { return (const T*)((const char*)data_ + y_arg * step_); }
  
template <class U> pcl::gpu::DeviceMemory2D::operator pcl::gpu::PtrStep<U>() const
{
    PtrStep<U> result;
    result.data = (U*)ptr<U>();
    result.step = step_;
    return result;
}

template <class U> pcl::gpu::DeviceMemory2D::operator pcl::gpu::PtrStepSz<U>() const
{
    PtrStepSz<U> result;
    result.data = (U*)ptr<U>();
    result.step = step_;
    result.cols = colsBytes_/sizeof(U);
    result.rows = rows_;
    return result;
}

#endif /* PCL_GPU_CONTAINER_DEVICE_MEMORY_IMPL_HPP_ */ 


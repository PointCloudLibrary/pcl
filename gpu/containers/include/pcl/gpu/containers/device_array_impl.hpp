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

#ifndef PCL_GPU_CONTAINER_DEVICE_ARRAY_IMPL_HPP_
#define PCL_GPU_CONTAINER_DEVICE_ARRAY_IMPL_HPP_


/////////////////////  Inline implementations of DeviceArray ////////////////////////////////////////////

template<class T> inline pcl::gpu::DeviceArray<T>::DeviceArray() {}
template<class T> inline pcl::gpu::DeviceArray<T>::DeviceArray(size_t size) : DeviceMemory(size * elem_size) {}
template<class T> inline pcl::gpu::DeviceArray<T>::DeviceArray(T *ptr, size_t size) : DeviceMemory(ptr, size * elem_size) {}
template<class T> inline pcl::gpu::DeviceArray<T>::DeviceArray(const DeviceArray& other) : DeviceMemory(other) {}
template<class T> inline pcl::gpu::DeviceArray<T>& pcl::gpu::DeviceArray<T>::operator=(const DeviceArray& other)
{ DeviceMemory::operator=(other); return *this; }

template<class T> inline void pcl::gpu::DeviceArray<T>::create(size_t size) 
{ DeviceMemory::create(size * elem_size); }
template<class T> inline void pcl::gpu::DeviceArray<T>::release()  
{ DeviceMemory::release(); }

template<class T> inline void pcl::gpu::DeviceArray<T>::copyTo(DeviceArray& other) const
{ DeviceMemory::copyTo(other); }
template<class T> inline void pcl::gpu::DeviceArray<T>::upload(const T *host_ptr, size_t size) 
{ DeviceMemory::upload(host_ptr, size * elem_size); }
template<class T> inline void pcl::gpu::DeviceArray<T>::download(T *host_ptr) const 
{ DeviceMemory::download( host_ptr ); }

template<class T> void pcl::gpu::DeviceArray<T>::swap(DeviceArray& other_arg) { DeviceMemory::swap(other_arg); }

template<class T> inline pcl::gpu::DeviceArray<T>::operator T*() { return ptr(); }
template<class T> inline pcl::gpu::DeviceArray<T>::operator const T*() const { return ptr(); }
template<class T> inline size_t pcl::gpu::DeviceArray<T>::size() const { return sizeBytes() / elem_size; }

template<class T> inline       T* pcl::gpu::DeviceArray<T>::ptr()       { return DeviceMemory::ptr<T>(); }
template<class T> inline const T* pcl::gpu::DeviceArray<T>::ptr() const { return DeviceMemory::ptr<T>(); }

template<class T> template<class A> inline void pcl::gpu::DeviceArray<T>::upload(const std::vector<T, A>& data) { upload(&data[0], data.size()); }
template<class T> template<class A> inline void pcl::gpu::DeviceArray<T>::download(std::vector<T, A>& data) const { data.resize(size()); if (!data.empty()) download(&data[0]); }

/////////////////////  Inline implementations of DeviceArray2D ////////////////////////////////////////////

template<class T> inline pcl::gpu::DeviceArray2D<T>::DeviceArray2D() {}
template<class T> inline pcl::gpu::DeviceArray2D<T>::DeviceArray2D(int rows, int cols) : DeviceMemory2D(rows, cols * elem_size) {}
template<class T> inline pcl::gpu::DeviceArray2D<T>::DeviceArray2D(int rows, int cols, void *data, size_t stepBytes) : DeviceMemory2D(rows, cols * elem_size, data, stepBytes) {}
template<class T> inline pcl::gpu::DeviceArray2D<T>::DeviceArray2D(const DeviceArray2D& other) : DeviceMemory2D(other) {}
template<class T> inline pcl::gpu::DeviceArray2D<T>& pcl::gpu::DeviceArray2D<T>::operator=(const DeviceArray2D& other)
{ DeviceMemory2D::operator=(other); return *this; }

template<class T> inline void pcl::gpu::DeviceArray2D<T>::create(int rows, int cols) 
{ DeviceMemory2D::create(rows, cols * elem_size); }
template<class T> inline void pcl::gpu::DeviceArray2D<T>::release()  
{ DeviceMemory2D::release(); }

template<class T> inline void pcl::gpu::DeviceArray2D<T>::copyTo(DeviceArray2D& other) const
{ DeviceMemory2D::copyTo(other); }
template<class T> inline void pcl::gpu::DeviceArray2D<T>::upload(const void *host_ptr, size_t host_step, int rows, int cols) 
{ DeviceMemory2D::upload(host_ptr, host_step, rows, cols * elem_size); }
template<class T> inline void pcl::gpu::DeviceArray2D<T>::download(void *host_ptr, size_t host_step) const 
{ DeviceMemory2D::download( host_ptr, host_step ); }

template<class T> template<class A> inline void pcl::gpu::DeviceArray2D<T>::upload(const std::vector<T, A>& data, int cols) 
{ upload(&data[0], cols * elem_size, data.size()/cols, cols); }

template<class T> template<class A> inline void pcl::gpu::DeviceArray2D<T>::download(std::vector<T, A>& data, int& elem_step) const 
{ elem_step = cols(); data.resize(cols() * rows()); if (!data.empty()) download(&data[0], colsBytes());  }

template<class T> void  pcl::gpu::DeviceArray2D<T>::swap(DeviceArray2D& other_arg) { DeviceMemory2D::swap(other_arg); }

template<class T> inline       T* pcl::gpu::DeviceArray2D<T>::ptr(int y)       { return DeviceMemory2D::ptr<T>(y); }
template<class T> inline const T* pcl::gpu::DeviceArray2D<T>::ptr(int y) const { return DeviceMemory2D::ptr<T>(y); }
            
template<class T> inline pcl::gpu::DeviceArray2D<T>::operator T*() { return ptr(); }
template<class T> inline pcl::gpu::DeviceArray2D<T>::operator const T*() const { return ptr(); }

template<class T> inline int pcl::gpu::DeviceArray2D<T>::cols() const { return DeviceMemory2D::colsBytes()/elem_size; }
template<class T> inline int pcl::gpu::DeviceArray2D<T>::rows() const { return DeviceMemory2D::rows(); }

template<class T> inline size_t pcl::gpu::DeviceArray2D<T>::elem_step() const { return DeviceMemory2D::step()/elem_size; }


#endif /* PCL_GPU_CONTAINER_DEVICE_ARRAY_IMPL_HPP_ */ 

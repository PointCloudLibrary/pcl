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
 */

#ifndef PCL_GPU_DEVICE_ARRAY_
#define PCL_GPU_DEVICE_ARRAY_

#include <vector>
#include "pcl/pcl_macros.h"

namespace pcl
{
    namespace gpu
    {
        class PCL_EXPORTS DeviceArray
        {
        public:
            DeviceArray();
            ~DeviceArray();            

            DeviceArray(size_t sizeBytes);
            DeviceArray(void *ptr, size_t sizeBytes);

            DeviceArray(const DeviceArray& other);
            DeviceArray& operator=(const DeviceArray& other);

            void create(size_t size);
            void release();

            void upload(const void *host_ptr, size_t sizeBytes);
            void download(void *host_ptr) const;
            
            template<class T> T* ptr() { return (T*)data_; }
            template<class T> const T* ptr() const { return (const T*)data_; }

            size_t sizeBytes() const;

        private:
            void *data_;
            size_t size_;

            int* refcount_;
        };

        class PCL_EXPORTS DeviceArray2D
        {
        public:
            DeviceArray2D();
            ~DeviceArray2D();            

            DeviceArray2D(int rows, int colsBytes);
            DeviceArray2D(int rows, int colsBytes, void *data, size_t stepBytes);

            DeviceArray2D(const DeviceArray2D& other);
            DeviceArray2D& operator=(const DeviceArray2D& other);

            void create(int rows, int colsBytes);
            void release();

            void upload(const void *host_ptr, size_t host_step, int rows, int colsBytes);
            void download(void *host_ptr, size_t host_step) const;
            
            template<class T> T* ptr(int y = 0) { return (T*)(data_ + y * step_); }
            template<class T> const T* ptr(int y = 0) const { return (const T*)(data_ + y * step_); }            

            size_t step() const;
            int colsBytes() const;
            int rows() const;

        private:
            int colsBytes_;
            int rows_;

            char *data_;
            size_t step_;

            int* refcount_;
        };


        ////////////////////// Template versions /////////////////////////////

        template<class T> 
        class PCL_EXPORTS DeviceArray_ : private DeviceArray
        {
        public:
            typedef T type;
            enum { elem_size = sizeof(T) };

            DeviceArray_();
            DeviceArray_(size_t sizeBytes);
            DeviceArray_(T *ptr, size_t sizeBytes);

            DeviceArray_(const DeviceArray_& other);
            DeviceArray_& operator = (const DeviceArray_& other);

            void create(size_t sizeBytes);
            void release();

            void upload(const T *host_ptr, size_t sizeBytes);
            void download(T *host_ptr) const;

            void upload(const std::vector<T>& data);
            void download(std::vector<T>& data) const;

            T* ptr(); 
            const T* ptr() const;            

            operator T*();
            operator const T*() const;

            size_t size() const;
        };

        template<class T> 
        class PCL_EXPORTS DeviceArray2D_ : private DeviceArray2D
        {
        public:
            typedef T type;
            enum { elem_size = sizeof(T) };

            DeviceArray2D_();
            DeviceArray2D_(int rows, int cols);
            DeviceArray2D_(int rows, int cols, void *data, size_t stepBytes);

            DeviceArray2D_(const DeviceArray2D_& other);
            DeviceArray2D_& operator = (const DeviceArray2D_& other);

            void create(int rows, int cols);
            void release();

            void upload(const void *host_ptr, size_t host_step, int rows, int cols);
            void download(void *host_ptr, size_t host_step) const;

            void upload(const std::vector<T>& data, int cols);
            void download(std::vector<T>& data, int& cols) const;
                                   
            T* ptr(int y = 0); 
            const T* ptr(int y = 0) const;            

            operator T*();
            operator const T*() const;

            size_t step() const;
            int cols() const;
            int rows() const;
        };        
        
    }
}

/////////////////////  Inline implementations of DeviceArray_ ////////////////////////////////////////////

template<class T> inline pcl::gpu::DeviceArray_<T>::DeviceArray_() {}
template<class T> inline pcl::gpu::DeviceArray_<T>::DeviceArray_(size_t size) : DeviceArray(size * elem_size) {}
template<class T> inline pcl::gpu::DeviceArray_<T>::DeviceArray_(T *ptr, size_t size) : DeviceArray(ptr, size * elem_size) {}
template<class T> inline pcl::gpu::DeviceArray_<T>::DeviceArray_(const DeviceArray_& other) : DeviceArray(other) {}
template<class T> inline pcl::gpu::DeviceArray_<T>& pcl::gpu::DeviceArray_<T>::operator=(const DeviceArray_& other)
{ DeviceArray::operator=(other); return *this; }

template<class T> inline void pcl::gpu::DeviceArray_<T>::create(size_t size) { DeviceArray::create(size * elem_size); }
template<class T> inline void pcl::gpu::DeviceArray_<T>::release()  { DeviceArray::release(); }
template<class T> inline void pcl::gpu::DeviceArray_<T>::upload(const T *host_ptr, size_t size) { DeviceArray::upload(host_ptr, size * elem_size); }
template<class T> inline void pcl::gpu::DeviceArray_<T>::download(T *host_ptr) const { DeviceArray::download( host_ptr ); }
template<class T> inline T* pcl::gpu::DeviceArray_<T>::ptr() { return DeviceArray::ptr<T>(); };
template<class T> inline const T* pcl::gpu::DeviceArray_<T>::ptr() const { return DeviceArray::ptr<T>(); };
template<class T> inline pcl::gpu::DeviceArray_<T>::operator T*() { return ptr(); }
template<class T> inline pcl::gpu::DeviceArray_<T>::operator const T*() const { return ptr(); }
template<class T> inline size_t pcl::gpu::DeviceArray_<T>::size() const { return DeviceArray::sizeBytes() / elem_size; }

template<class T> inline void pcl::gpu::DeviceArray_<T>::upload(const std::vector<T>& data) { upload(&data[0], data.size()); }
template<class T> inline void pcl::gpu::DeviceArray_<T>::download(std::vector<T>& data) const { data.resize(size()); download(&data[0]); }


/////////////////////  Inline implementations of DeviceArray2D_ ////////////////////////////////////////////

template<class T> inline pcl::gpu::DeviceArray2D_<T>::DeviceArray2D_() {}
template<class T> inline pcl::gpu::DeviceArray2D_<T>::DeviceArray2D_(int rows, int cols) : DeviceArray2D(rows, cols * elem_size) {}
template<class T> inline pcl::gpu::DeviceArray2D_<T>::DeviceArray2D_(int rows, int cols, void *data, size_t stepBytes) : DeviceArray2D(rows, cols * elem_size, data, stepBytes) {}
template<class T> inline pcl::gpu::DeviceArray2D_<T>::DeviceArray2D_(const DeviceArray2D_& other) : DeviceArray2D(other) {}
template<class T> inline pcl::gpu::DeviceArray2D_<T>& pcl::gpu::DeviceArray2D_<T>::operator=(const DeviceArray2D_& other)
{ DeviceArray2D::operator=(other); return *this; }

template<class T> inline void pcl::gpu::DeviceArray2D_<T>::create(int rows, int cols) { DeviceArray2D::create(rows, cols * elem_size); }
template<class T> inline void pcl::gpu::DeviceArray2D_<T>::release()  { DeviceArray2D::release(); }

template<class T> inline void pcl::gpu::DeviceArray2D_<T>::upload(const void *host_ptr, size_t host_step, int rows, int cols) 
{ DeviceArray2D::upload(host_ptr, host_step, rows, cols * elem_size); }

template<class T> inline void pcl::gpu::DeviceArray2D_<T>::download(void *host_ptr, size_t host_step) const 
{ DeviceArray2D::download( host_ptr, host_step ); }

template<class T> inline void pcl::gpu::DeviceArray2D_<T>::upload(const std::vector<T>& data, int cols) 
{ upload(&data[0], cols * elem_size, data.size()/cols, cols); }

template<class T> inline void pcl::gpu::DeviceArray2D_<T>::download(std::vector<T>& data, int& elem_step) const 
{ int rows, cols; size(rows, cols); data.resize(cols * rows); download(&data[0], cols * elem_size);  }

template<class T> inline T* pcl::gpu::DeviceArray2D_<T>::ptr(int y) { return DeviceArray2D::ptr<T>(y); };
template<class T> inline const T* pcl::gpu::DeviceArray2D_<T>::ptr(int y) const { return DeviceArray2D::ptr<T>(y); };
template<class T> inline pcl::gpu::DeviceArray2D_<T>::operator T*() { return ptr(); }
template<class T> inline pcl::gpu::DeviceArray2D_<T>::operator const T*() const { return ptr(); }

template<class T> inline size_t pcl::gpu::DeviceArray2D_<T>::step() const { return DeviceArray2D::step(); }
template<class T> inline int pcl::gpu::DeviceArray2D_<T>::cols() const { return DeviceArray2D::colsBytes()/elem_size; }
template<class T> inline int pcl::gpu::DeviceArray2D_<T>::rows() const { return DeviceArray2D::rows(); }




#endif /* PCL_GPU_DEVICE_ARRAY_ */
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

#ifndef PCL_GPU_CONTAINER_DEVICE_ARRAY_HPP_
#define PCL_GPU_CONTAINER_DEVICE_ARRAY_HPP_

#include <pcl/gpu/containers/device_memory.hpp>

#include <vector>

namespace pcl
{
    namespace gpu
    {              
         /**
          * @brief Typed container for GPU memory
          */
        template<class T> 
        class PCL_EXPORTS DeviceArray : public DeviceMemory
        {
        public:
            typedef T type;
            enum { elem_size = sizeof(T) };

            DeviceArray();
            DeviceArray(size_t size);
            DeviceArray(T *ptr, size_t size);

            DeviceArray(const DeviceArray& other);
            DeviceArray& operator = (const DeviceArray& other);

            void create(size_t size);
            void release();

            void upload(const T *host_ptr, size_t size);
            void download(T *host_ptr) const;

            void upload(const std::vector<T>& data);
            void download(std::vector<T>& data) const;

            T* ptr(); 
            const T* ptr() const;

            //using DeviceMemory::ptr;
            
            operator T*();
            operator const T*() const;

            size_t size() const;            
        };

        /**
          * @brief Typed container for pitched GPU memory
          */
        template<class T> 
        class PCL_EXPORTS DeviceArray2D : public DeviceMemory2D
        {
        public:
            typedef T type;
            enum { elem_size = sizeof(T) };

            DeviceArray2D();
            DeviceArray2D(int rows, int cols);
            DeviceArray2D(int rows, int cols, void *data, size_t stepBytes);

            DeviceArray2D(const DeviceArray2D& other);
            DeviceArray2D& operator = (const DeviceArray2D& other);

            void create(int rows, int cols);
            void release();

            void upload(const void *host_ptr, size_t host_step, int rows, int cols);
            void download(void *host_ptr, size_t host_step) const;

            void upload(const std::vector<T>& data, int cols);
            void download(std::vector<T>& data, int& cols) const;
                                          
            T* ptr(int y = 0);             
            const T* ptr(int y = 0) const;            
            
            //using DeviceMemory2D::ptr;            

            operator T*();
            operator const T*() const;                        
            
            int cols() const;
            int rows() const;

            int elem_step() const;
        };        
    }
}

#include "pcl/gpu/containers/impl/device_array_impl.hpp"

#endif /* PCL_GPU_CONTAINER_DEVICE_ARRAY_HPP_ */
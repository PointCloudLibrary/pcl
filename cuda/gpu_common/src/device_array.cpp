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

#include "pcl/gpu/common/device_array.hpp"
#include "pcl/gpu/common/safe_call.hpp"

#include "cuda_runtime_api.h"

//////////////////////////    XADD    ///////////////////////////////

#ifdef __GNUC__
    
    #if __GNUC__*10 + __GNUC_MINOR__ >= 42

        #if !defined WIN32 && (defined __i486__ || defined __i586__ || defined __i686__ || defined __MMX__ || defined __SSE__  || defined __ppc__)
            #define CV_XADD __sync_fetch_and_add
        #else
            #include <ext/atomicity.h>
            #define CV_XADD __gnu_cxx::__exchange_and_add
        #endif
    #else
        #include <bits/atomicity.h>
        #if __GNUC__*10 + __GNUC_MINOR__ >= 34
            #define CV_XADD __gnu_cxx::__exchange_and_add
        #else
            #define CV_XADD __exchange_and_add
        #endif
  #endif
    
#elif defined WIN32 || defined _WIN32
    #include <intrin.h>
    #define CV_XADD(addr,delta) _InterlockedExchangeAdd((long volatile*)(addr), (delta))
#else

    template<typename _Tp> static inline _Tp CV_XADD(_Tp* addr, _Tp delta)
    { int tmp = *addr; *addr += delta; return tmp; }
    
#endif

////////////////////////    DeviceArray    /////////////////////////////
    
pcl::gpu::DeviceArray::DeviceArray() : data_(0), size_(0), refcount_(0) {}
pcl::gpu::DeviceArray::DeviceArray(void *ptr, size_t size) : data_(ptr), size_(size), refcount_(0) {}
pcl::gpu::DeviceArray::DeviceArray(size_t size)  : data_(0), size_(0), refcount_(0)  { create(size); }
pcl::gpu::DeviceArray::~DeviceArray() { release(); }


pcl::gpu::DeviceArray::DeviceArray(const DeviceArray& other) : data_(other.data_), size_(other.size_), refcount_(other.refcount_)
{
    if( refcount_ )
        CV_XADD(refcount_, 1);
}

pcl::gpu::DeviceArray& pcl::gpu::DeviceArray::operator = (const pcl::gpu::DeviceArray& other)
{
    if( this != &other )
    {
        if( other.refcount_ )
            CV_XADD(other.refcount_, 1);
        release();
        
        data_ = other.data_;
        size_ = other.size_;        
        refcount_ = other.refcount_;
    }
    return *this;
}

void pcl::gpu::DeviceArray::create(size_t size)
{
    if (size_ == size)
        return;
            
    if( size > 0)
    {        
        if( data_ )
            release();

        size_ = size;
                        
        cudaSafeCall( cudaMalloc(&data_, size_) );        

        //refcount_ = (int*)cv::fastMalloc(sizeof(*refcount_));
        refcount_ = new int;
        *refcount_ = 1;
    }
}

void pcl::gpu::DeviceArray::release()
{
    if( refcount_ && CV_XADD(refcount_, -1) == 1 )
    {
        //cv::fastFree(refcount_);
        delete refcount_;
        cudaSafeCall( cudaFree(data_) );
    }
    data_ = 0;
    size_ = 0;
    refcount_ = 0;
}

void pcl::gpu::DeviceArray::upload(const void *host_ptr, size_t size)
{
    create(size);
    cudaSafeCall( cudaMemcpy(data_, host_ptr, size_, cudaMemcpyHostToDevice) );        
}

void pcl::gpu::DeviceArray::download(void *host_ptr) const
{    
    cudaSafeCall( cudaMemcpy(host_ptr, data_, size_, cudaMemcpyDeviceToHost) );
}
        
      
size_t pcl::gpu::DeviceArray::sizeBytes() const { return size_; }



////////////////////////    DeviceArray2D    /////////////////////////////

pcl::gpu::DeviceArray2D::DeviceArray2D() : colsBytes_(0), rows_(0), data_(0), step_(0), refcount_(0) {}

pcl::gpu::DeviceArray2D::DeviceArray2D(int rows, int colsBytes) 
    : colsBytes_(0), rows_(0), data_(0), step_(0), refcount_(0) 
{ 
    create(rows, colsBytes); 
}

pcl::gpu::DeviceArray2D::DeviceArray2D(int rows, int colsBytes, void *data, size_t stepBytes) 
    : colsBytes_(colsBytes), rows_(rows), data_((char*)data), step_(stepBytes), refcount_(0) {}

pcl::gpu::DeviceArray2D::~DeviceArray2D() { release(); }


pcl::gpu::DeviceArray2D::DeviceArray2D(const DeviceArray2D& other) : 
    colsBytes_(other.colsBytes_), rows_(other.rows_), data_(other.data_), step_(other.step_), refcount_(other.refcount_)
{
    if( refcount_ )
        CV_XADD(refcount_, 1);
}

pcl::gpu::DeviceArray2D& pcl::gpu::DeviceArray2D::operator = (const pcl::gpu::DeviceArray2D& other)
{
    if( this != &other )
    {
        if( other.refcount_ )
            CV_XADD(other.refcount_, 1);
        release();
        
        colsBytes_ = other.colsBytes_;
        rows_ = other.rows_;
        data_ = other.data_;
        step_ = other.step_;
                
        refcount_ = other.refcount_;
    }
    return *this;
}

void pcl::gpu::DeviceArray2D::create(int rows, int colsBytes)
{
    if (colsBytes_ == colsBytes && rows_ == rows)
        return;
            
    if( rows > 0 && colsBytes > 0)
    {        
        if( data_ )
            release();
              
        colsBytes_ = colsBytes;
        rows_ = rows;
                        
        cudaSafeCall( cudaMallocPitch( (void**)&data_, &step_, colsBytes, rows) );        

        //refcount_ = (int*)cv::fastMalloc(sizeof(*refcount_));
        refcount_ = new int;
        *refcount_ = 1;
    }
}

void pcl::gpu::DeviceArray2D::release()
{
    if( refcount_ && CV_XADD(refcount_, -1) == 1 )
    {
        //cv::fastFree(refcount_);
        delete refcount_;
        cudaSafeCall( cudaFree(data_) );
    }

    colsBytes_ = 0;
    rows_ = 0;    
    data_ = 0;    
    step_ = 0;
    refcount_ = 0;
}

void pcl::gpu::DeviceArray2D::upload(const void *host_ptr, size_t host_step, int rows, int colsBytes)
{
    create(rows, colsBytes);
    cudaSafeCall( cudaMemcpy2D(data_, step_, host_ptr, host_step, colsBytes_, rows_, cudaMemcpyHostToDevice) );        
}

void pcl::gpu::DeviceArray2D::download(void *host_ptr, size_t host_step) const
{    
    cudaSafeCall( cudaMemcpy2D(host_ptr, host_step, data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToHost) );
}
        
size_t pcl::gpu::DeviceArray2D::step() const { return step_; }
int pcl::gpu::DeviceArray2D::colsBytes() const { return colsBytes_; }
int pcl::gpu::DeviceArray2D::rows() const { return rows_; }
      

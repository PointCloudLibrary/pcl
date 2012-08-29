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

#include <pcl/gpu/containers/device_memory.h>
#include <pcl/gpu/utils/safe_call.hpp>

#include "cuda_runtime_api.h"
#include "assert.h"

#define HAVE_CUDA
//#include <pcl_config.h>

#if !defined(HAVE_CUDA)

void throw_nogpu() { throw "PCL 2.0 exception"; }

pcl::gpu::DeviceMemory::DeviceMemory() { throw_nogpu(); }
pcl::gpu::DeviceMemory::DeviceMemory(void *, size_t) { throw_nogpu(); }
pcl::gpu::DeviceMemory::DeviceMemory(size_t) { throw_nogpu(); }
pcl::gpu::DeviceMemory::~DeviceMemory() { throw_nogpu(); }
pcl::gpu::DeviceMemory::DeviceMemory(const DeviceMemory& ) { throw_nogpu(); }
pcl::gpu::DeviceMemory& pcl::gpu::DeviceMemory::operator=(const pcl::gpu::DeviceMemory&) { throw_nogpu(); return *this;}
void pcl::gpu::DeviceMemory::create(size_t) { throw_nogpu(); }
void pcl::gpu::DeviceMemory::release() { throw_nogpu(); }
void pcl::gpu::DeviceMemory::copyTo(DeviceMemory&) const { throw_nogpu(); }
void pcl::gpu::DeviceMemory::upload(const void*, size_t) { throw_nogpu(); }
void pcl::gpu::DeviceMemory::download(void*) const { throw_nogpu(); }
bool pcl::gpu::DeviceMemory::empty() const { throw_nogpu(); }
pcl::gpu::DeviceMemory2D::DeviceMemory2D() { throw_nogpu(); }
pcl::gpu::DeviceMemory2D::DeviceMemory2D(int, int)  { throw_nogpu(); }
pcl::gpu::DeviceMemory2D::DeviceMemory2D(int, int, void*, size_t)  { throw_nogpu(); }
pcl::gpu::DeviceMemory2D::~DeviceMemory2D() { throw_nogpu(); }
pcl::gpu::DeviceMemory2D::DeviceMemory2D(const DeviceMemory2D&)  { throw_nogpu(); }
pcl::gpu::DeviceMemory2D& pcl::gpu::DeviceMemory2D::operator=(const pcl::gpu::DeviceMemory2D&) { throw_nogpu(); return *this;}
void pcl::gpu::DeviceMemory2D::create(int, int )  { throw_nogpu(); }
void pcl::gpu::DeviceMemory2D::release()  { throw_nogpu(); }
void pcl::gpu::DeviceMemory2D::copyTo(DeviceMemory2D&) const  { throw_nogpu(); }
void pcl::gpu::DeviceMemory2D::upload(const void *, size_t, int, int )  { throw_nogpu(); }
void pcl::gpu::DeviceMemory2D::download(void *, size_t ) const  { throw_nogpu(); }
bool pcl::gpu::DeviceMemory2D::empty() const { throw_nogpu(); }

#else

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
    
pcl::gpu::DeviceMemory::DeviceMemory() : data_(0), sizeBytes_(0), refcount_(0) {}
pcl::gpu::DeviceMemory::DeviceMemory(void *ptr_arg, size_t sizeBytes_arg) : data_(ptr_arg), sizeBytes_(sizeBytes_arg), refcount_(0){}
pcl::gpu::DeviceMemory::DeviceMemory(size_t sizeBtes_arg)  : data_(0), sizeBytes_(0), refcount_(0) { create(sizeBtes_arg); }
pcl::gpu::DeviceMemory::~DeviceMemory() { release(); }

pcl::gpu::DeviceMemory::DeviceMemory(const DeviceMemory& other_arg) 
    : data_(other_arg.data_), sizeBytes_(other_arg.sizeBytes_), refcount_(other_arg.refcount_)
{
    if( refcount_ )
        CV_XADD(refcount_, 1);
}

pcl::gpu::DeviceMemory& pcl::gpu::DeviceMemory::operator = (const pcl::gpu::DeviceMemory& other_arg)
{
    if( this != &other_arg )
    {
        if( other_arg.refcount_ )
            CV_XADD(other_arg.refcount_, 1);
        release();
        
        data_      = other_arg.data_;
        sizeBytes_ = other_arg.sizeBytes_;                
        refcount_  = other_arg.refcount_;
    }
    return *this;
}

void pcl::gpu::DeviceMemory::create(size_t sizeBytes_arg)
{
    if (sizeBytes_arg == sizeBytes_)
        return;
            
    if( sizeBytes_arg > 0)
    {        
        if( data_ )
            release();

        sizeBytes_ = sizeBytes_arg;
                        
        cudaSafeCall( cudaMalloc(&data_, sizeBytes_) );
        
        //refcount_ = (int*)cv::fastMalloc(sizeof(*refcount_));
        refcount_ = new int;
        *refcount_ = 1;
    }
}

void pcl::gpu::DeviceMemory::copyTo(DeviceMemory& other) const
{
    if (empty())
        other.release();
    else
    {    
        other.create(sizeBytes_);    
        cudaSafeCall( cudaMemcpy(other.data_, data_, sizeBytes_, cudaMemcpyDeviceToDevice) );
        cudaSafeCall( cudaDeviceSynchronize() );
    }
}

void pcl::gpu::DeviceMemory::release()
{
    if( refcount_ && CV_XADD(refcount_, -1) == 1 )
    {
        //cv::fastFree(refcount);
        delete refcount_;
        cudaSafeCall( cudaFree(data_) );
    }
    data_ = 0;
    sizeBytes_ = 0;
    refcount_ = 0;
}

void pcl::gpu::DeviceMemory::upload(const void *host_ptr_arg, size_t sizeBytes_arg)
{
    create(sizeBytes_arg);
    cudaSafeCall( cudaMemcpy(data_, host_ptr_arg, sizeBytes_, cudaMemcpyHostToDevice) );
    cudaSafeCall( cudaDeviceSynchronize() );
}

void pcl::gpu::DeviceMemory::download(void *host_ptr_arg) const
{    
    cudaSafeCall( cudaMemcpy(host_ptr_arg, data_, sizeBytes_, cudaMemcpyDeviceToHost) );
    cudaSafeCall( cudaDeviceSynchronize() );
}          

void pcl::gpu::DeviceMemory::swap(DeviceMemory& other_arg)
{
    std::swap(data_, other_arg.data_);
    std::swap(sizeBytes_, other_arg.sizeBytes_);
    std::swap(refcount_, other_arg.refcount_);
}

bool pcl::gpu::DeviceMemory::empty() const { return !data_; }
size_t pcl::gpu::DeviceMemory::sizeBytes() const { return sizeBytes_; }


////////////////////////    DeviceArray2D    /////////////////////////////

pcl::gpu::DeviceMemory2D::DeviceMemory2D() : data_(0), step_(0), colsBytes_(0), rows_(0), refcount_(0) {}

pcl::gpu::DeviceMemory2D::DeviceMemory2D(int rows_arg, int colsBytes_arg) 
    : data_(0), step_(0), colsBytes_(0), rows_(0), refcount_(0)
{ 
    create(rows_arg, colsBytes_arg); 
}

pcl::gpu::DeviceMemory2D::DeviceMemory2D(int rows_arg, int colsBytes_arg, void *data_arg, size_t step_arg) 
    :  data_(data_arg), step_(step_arg), colsBytes_(colsBytes_arg), rows_(rows_arg), refcount_(0) {}

pcl::gpu::DeviceMemory2D::~DeviceMemory2D() { release(); }


pcl::gpu::DeviceMemory2D::DeviceMemory2D(const DeviceMemory2D& other_arg) : 
    data_(other_arg.data_), step_(other_arg.step_), colsBytes_(other_arg.colsBytes_), rows_(other_arg.rows_), refcount_(other_arg.refcount_)
{
    if( refcount_ )
        CV_XADD(refcount_, 1);
}

pcl::gpu::DeviceMemory2D& pcl::gpu::DeviceMemory2D::operator = (const pcl::gpu::DeviceMemory2D& other_arg)
{
    if( this != &other_arg )
    {
        if( other_arg.refcount_ )
            CV_XADD(other_arg.refcount_, 1);
        release();
        
        colsBytes_ = other_arg.colsBytes_;
        rows_ = other_arg.rows_;
        data_ = other_arg.data_;
        step_ = other_arg.step_;
                
        refcount_ = other_arg.refcount_;
    }
    return *this;
}

void pcl::gpu::DeviceMemory2D::create(int rows_arg, int colsBytes_arg)
{
    if (colsBytes_ == colsBytes_arg && rows_ == rows_arg)
        return;
            
    if( rows_arg > 0 && colsBytes_arg > 0)
    {        
        if( data_ )
            release();
              
        colsBytes_ = colsBytes_arg;
        rows_ = rows_arg;
                        
        cudaSafeCall( cudaMallocPitch( (void**)&data_, &step_, colsBytes_, rows_) );        

        //refcount = (int*)cv::fastMalloc(sizeof(*refcount));
        refcount_ = new int;
        *refcount_ = 1;
    }
}

void pcl::gpu::DeviceMemory2D::release()
{
    if( refcount_ && CV_XADD(refcount_, -1) == 1 )
    {
        //cv::fastFree(refcount);
        delete refcount_;
        cudaSafeCall( cudaFree(data_) );
    }

    colsBytes_ = 0;
    rows_ = 0;    
    data_ = 0;    
    step_ = 0;
    refcount_ = 0;
}

void pcl::gpu::DeviceMemory2D::copyTo(DeviceMemory2D& other) const
{
    if (empty())
        other.release();
    else
    {
        other.create(rows_, colsBytes_);    
        cudaSafeCall( cudaMemcpy2D(other.data_, other.step_, data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToDevice) );
        cudaSafeCall( cudaDeviceSynchronize() );
    }
}

void pcl::gpu::DeviceMemory2D::upload(const void *host_ptr_arg, size_t host_step_arg, int rows_arg, int colsBytes_arg)
{
    create(rows_arg, colsBytes_arg);
    cudaSafeCall( cudaMemcpy2D(data_, step_, host_ptr_arg, host_step_arg, colsBytes_, rows_, cudaMemcpyHostToDevice) );        
    cudaSafeCall( cudaDeviceSynchronize() );
}

void pcl::gpu::DeviceMemory2D::download(void *host_ptr_arg, size_t host_step_arg) const
{    
    cudaSafeCall( cudaMemcpy2D(host_ptr_arg, host_step_arg, data_, step_, colsBytes_, rows_, cudaMemcpyDeviceToHost) );
    cudaSafeCall( cudaDeviceSynchronize() );
}      

void pcl::gpu::DeviceMemory2D::swap(DeviceMemory2D& other_arg)
{    
    std::swap(data_, other_arg.data_);
    std::swap(step_, other_arg.step_);

    std::swap(colsBytes_, other_arg.colsBytes_);
    std::swap(rows_, other_arg.rows_);
    std::swap(refcount_, other_arg.refcount_);                 
}

bool pcl::gpu::DeviceMemory2D::empty() const { return !data_; }
int pcl::gpu::DeviceMemory2D::colsBytes() const { return colsBytes_; }
int pcl::gpu::DeviceMemory2D::rows() const { return rows_; }
size_t pcl::gpu::DeviceMemory2D::step() const { return step_; }

#endif

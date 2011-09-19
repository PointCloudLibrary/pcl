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

#include "pcl/gpu/containers/device_memory.hpp"
#include "pcl/gpu/utils/safe_call.hpp"

#include "cuda_runtime_api.h"
#include "assert.h"

#define HAVE_CUDA
//#include "pcl_config.h"

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
    
pcl::gpu::DeviceMemory::DeviceMemory() : data(0), sizeBytes(0), refcount(0) {}
pcl::gpu::DeviceMemory::DeviceMemory(void *ptr_arg, size_t sizeBytes_arg) : data((char*)ptr_arg), sizeBytes(sizeBytes_arg), refcount(0){}
pcl::gpu::DeviceMemory::DeviceMemory(size_t sizeBtes_arg)  : data(0), sizeBytes(0), refcount(0) { create(sizeBtes_arg); }
pcl::gpu::DeviceMemory::~DeviceMemory() { release(); }


pcl::gpu::DeviceMemory::DeviceMemory(const DeviceMemory& other_arg) 
    : data(other_arg.data), sizeBytes(other_arg.sizeBytes), refcount(other_arg.refcount)
{
    if( refcount )
        CV_XADD(refcount, 1);
}

pcl::gpu::DeviceMemory& pcl::gpu::DeviceMemory::operator = (const pcl::gpu::DeviceMemory& other_arg)
{
    if( this != &other_arg )
    {
        if( other_arg.refcount )
            CV_XADD(other_arg.refcount, 1);
        release();
        
        data      = other_arg.data;
        sizeBytes = other_arg.sizeBytes;                
        refcount  = other_arg.refcount;
    }
    return *this;
}

void pcl::gpu::DeviceMemory::create(size_t sizeBytes_arg)
{
    if (sizeBytes_arg == sizeBytes)
        return;
            
    if( sizeBytes_arg > 0)
    {        
        if( data )
            release();

        sizeBytes = sizeBytes_arg;
                        
        cudaSafeCall( cudaMalloc((void**)&data, sizeBytes) );        

        //refcount_ = (int*)cv::fastMalloc(sizeof(*refcount_));
        refcount = new int;
        *refcount = 1;
    }
}

void pcl::gpu::DeviceMemory::copyTo(DeviceMemory& other) const
{
    assert(data);

    other.create(sizeBytes);    
    cudaSafeCall( cudaMemcpy(other.data, data, sizeBytes, cudaMemcpyDeviceToDevice) );
    cudaSafeCall( cudaDeviceSynchronize() );
}

void pcl::gpu::DeviceMemory::release()
{
    if( refcount && CV_XADD(refcount, -1) == 1 )
    {
        //cv::fastFree(refcount);
        delete refcount;
        cudaSafeCall( cudaFree(data) );
    }
    data = 0;
    sizeBytes = 0;
    refcount = 0;
}

void pcl::gpu::DeviceMemory::upload(const void *host_ptr_arg, size_t sizeBytes_arg)
{
    create(sizeBytes_arg);
    cudaSafeCall( cudaMemcpy(data, host_ptr_arg, sizeBytes, cudaMemcpyHostToDevice) );        
}

void pcl::gpu::DeviceMemory::download(void *host_ptr_arg) const
{    
    cudaSafeCall( cudaMemcpy(host_ptr_arg, data, sizeBytes, cudaMemcpyDeviceToHost) );
}          

bool pcl::gpu::DeviceMemory::empty() const { return !data; }

////////////////////////    DeviceArray2D    /////////////////////////////

pcl::gpu::DeviceMemory2D::DeviceMemory2D() : colsBytes(0), rows(0), data(0), step(0), refcount(0) {}

pcl::gpu::DeviceMemory2D::DeviceMemory2D(int rows_arg, int colsBytes_arg) 
    : colsBytes(0), rows(0), data(0), step(0), refcount(0) 
{ 
    create(rows_arg, colsBytes_arg); 
}

pcl::gpu::DeviceMemory2D::DeviceMemory2D(int rows_arg, int colsBytes_arg, void *data_arg, size_t step_arg) 
    : colsBytes(colsBytes_arg), rows(rows_arg), data((char*)data_arg), step(step_arg), refcount(0) {}

pcl::gpu::DeviceMemory2D::~DeviceMemory2D() { release(); }


pcl::gpu::DeviceMemory2D::DeviceMemory2D(const DeviceMemory2D& other_arg) : 
    colsBytes(other_arg.colsBytes), rows(other_arg.rows), data(other_arg.data), step(other_arg.step), refcount(other_arg.refcount)
{
    if( refcount )
        CV_XADD(refcount, 1);
}

pcl::gpu::DeviceMemory2D& pcl::gpu::DeviceMemory2D::operator = (const pcl::gpu::DeviceMemory2D& other_arg)
{
    if( this != &other_arg )
    {
        if( other_arg.refcount )
            CV_XADD(other_arg.refcount, 1);
        release();
        
        colsBytes = other_arg.colsBytes;
        rows = other_arg.rows;
        data = other_arg.data;
        step = other_arg.step;
                
        refcount = other_arg.refcount;
    }
    return *this;
}

void pcl::gpu::DeviceMemory2D::create(int rows_arg, int colsBytes_arg)
{
    if (colsBytes == colsBytes_arg && rows == rows_arg)
        return;
            
    if( rows_arg > 0 && colsBytes_arg > 0)
    {        
        if( data )
            release();
              
        colsBytes = colsBytes_arg;
        rows = rows_arg;
                        
        cudaSafeCall( cudaMallocPitch( (void**)&data, &step, colsBytes, rows) );        

        //refcount = (int*)cv::fastMalloc(sizeof(*refcount));
        refcount = new int;
        *refcount = 1;
    }
}

void pcl::gpu::DeviceMemory2D::release()
{
    if( refcount && CV_XADD(refcount, -1) == 1 )
    {
        //cv::fastFree(refcount);
        delete refcount;
        cudaSafeCall( cudaFree(data) );
    }

    colsBytes = 0;
    rows = 0;    
    data = 0;    
    step = 0;
    refcount = 0;
}

void pcl::gpu::DeviceMemory2D::copyTo(DeviceMemory2D& other) const
{
    assert(data);
    other.create(rows, colsBytes);    
    cudaSafeCall( cudaMemcpy2D(other.data, other.step, data, step, colsBytes, rows, cudaMemcpyDeviceToDevice) );
    cudaSafeCall( cudaDeviceSynchronize() );
}

void pcl::gpu::DeviceMemory2D::upload(const void *host_ptr_arg, size_t host_step_arg, int rows_arg, int colsBytes_arg)
{
    create(rows_arg, colsBytes_arg);
    cudaSafeCall( cudaMemcpy2D(data, step, host_ptr_arg, host_step_arg, colsBytes, rows, cudaMemcpyHostToDevice) );        
}

void pcl::gpu::DeviceMemory2D::download(void *host_ptr_arg, size_t host_step_arg) const
{    
    cudaSafeCall( cudaMemcpy2D(host_ptr_arg, host_step_arg, data, step, colsBytes, rows, cudaMemcpyDeviceToHost) );
}      

bool pcl::gpu::DeviceMemory2D::empty() const { return !data; }

#endif
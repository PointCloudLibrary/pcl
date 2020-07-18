/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (C) 2009-2010, NVIDIA Corporation, all rights reserved.
 *  Third party copyrights are property of their respective owners.
 *
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
 * $Id:  $
 * Ported to PCL by Koen Buys : Attention Work in progress!
 */

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "NCV.hpp"


//==============================================================================
//
// Error handling helpers
//
//==============================================================================


static void stdDebugOutput(const std::string &msg)
{
    std::cout << msg;
}


static NCVDebugOutputHandler *debugOutputHandler = stdDebugOutput;


void ncvDebugOutput(const std::string &msg)
{
    debugOutputHandler(msg);
}


void ncvSetDebugOutputHandler(NCVDebugOutputHandler *func)
{
    debugOutputHandler = func;
}


//==============================================================================
//
// Memory wrappers and helpers
//
//==============================================================================


Ncv32u alignUp(Ncv32u what, Ncv32u alignment)
{
    Ncv32u alignMask = alignment-1;
    Ncv32u inverseAlignMask = ~alignMask;
    Ncv32u res = (what + alignMask) & inverseAlignMask;
    return res;
}


void NCVMemPtr::clear()
{
    ptr = NULL;
    memtype = NCVMemoryTypeNone;
}


void NCVMemSegment::clear()
{
    begin.clear();
    size = 0;
}


NCVStatus memSegCopyHelper(void *dst, NCVMemoryType dstType, const void *src, NCVMemoryType srcType, std::size_t sz, cudaStream_t cuStream)
{
    NCVStatus ncvStat;
    switch (dstType)
    {
    case NCVMemoryTypeHostPageable:
    case NCVMemoryTypeHostPinned:
        switch (srcType)
        {
        case NCVMemoryTypeHostPageable:
        case NCVMemoryTypeHostPinned:
            memcpy(dst, src, sz);
            ncvStat = NCV_SUCCESS;
            break;
        case NCVMemoryTypeDevice:
            if (cuStream != 0)
            {
                ncvAssertCUDAReturn(cudaMemcpyAsync(dst, src, sz, cudaMemcpyDeviceToHost, cuStream), NCV_CUDA_ERROR);
            }
            else
            {
                ncvAssertCUDAReturn(cudaMemcpy(dst, src, sz, cudaMemcpyDeviceToHost), NCV_CUDA_ERROR);
            }
            ncvStat = NCV_SUCCESS;
            break;
        default:
            ncvStat = NCV_MEM_RESIDENCE_ERROR;
        }
        break;
    case NCVMemoryTypeDevice:
        switch (srcType)
        {
        case NCVMemoryTypeHostPageable:
        case NCVMemoryTypeHostPinned:
            if (cuStream != 0)
            {
                ncvAssertCUDAReturn(cudaMemcpyAsync(dst, src, sz, cudaMemcpyHostToDevice, cuStream), NCV_CUDA_ERROR);
            }
            else
            {
                ncvAssertCUDAReturn(cudaMemcpy(dst, src, sz, cudaMemcpyHostToDevice), NCV_CUDA_ERROR);
            }
            ncvStat = NCV_SUCCESS;
            break;
        case NCVMemoryTypeDevice:
            if (cuStream != 0)
            {
                ncvAssertCUDAReturn(cudaMemcpyAsync(dst, src, sz, cudaMemcpyDeviceToDevice, cuStream), NCV_CUDA_ERROR);
            }
            else
            {
                ncvAssertCUDAReturn(cudaMemcpy(dst, src, sz, cudaMemcpyDeviceToDevice), NCV_CUDA_ERROR);
            }
            ncvStat = NCV_SUCCESS;
            break;
        default:
            ncvStat = NCV_MEM_RESIDENCE_ERROR;
        }
        break;
    default:
        ncvStat = NCV_MEM_RESIDENCE_ERROR;
    }

    return ncvStat;
}


NCVStatus memSegCopyHelper2D(void *dst, Ncv32u dstPitch, NCVMemoryType dstType,
                             const void *src, Ncv32u srcPitch, NCVMemoryType srcType,
                             Ncv32u widthbytes, Ncv32u height, cudaStream_t cuStream)
{
    NCVStatus ncvStat;
    switch (dstType)
    {
    case NCVMemoryTypeHostPageable:
    case NCVMemoryTypeHostPinned:
        switch (srcType)
        {
        case NCVMemoryTypeHostPageable:
        case NCVMemoryTypeHostPinned:
            for (Ncv32u i=0; i<height; i++)
            {
                memcpy((char*)dst + i * dstPitch, (char*)src + i * srcPitch, widthbytes);
            }
            ncvStat = NCV_SUCCESS;
            break;
        case NCVMemoryTypeDevice:
            if (cuStream != 0)
            {
                ncvAssertCUDAReturn(cudaMemcpy2DAsync(dst, dstPitch, src, srcPitch, widthbytes, height, cudaMemcpyDeviceToHost, cuStream), NCV_CUDA_ERROR);
            }
            else
            {
                ncvAssertCUDAReturn(cudaMemcpy2D(dst, dstPitch, src, srcPitch, widthbytes, height, cudaMemcpyDeviceToHost), NCV_CUDA_ERROR);
            }
            ncvStat = NCV_SUCCESS;
            break;
        default:
            ncvStat = NCV_MEM_RESIDENCE_ERROR;
        }
        break;
    case NCVMemoryTypeDevice:
        switch (srcType)
        {
        case NCVMemoryTypeHostPageable:
        case NCVMemoryTypeHostPinned:
            if (cuStream != 0)
            {
                ncvAssertCUDAReturn(cudaMemcpy2DAsync(dst, dstPitch, src, srcPitch, widthbytes, height, cudaMemcpyHostToDevice, cuStream), NCV_CUDA_ERROR);
            }
            else
            {
                ncvAssertCUDAReturn(cudaMemcpy2D(dst, dstPitch, src, srcPitch, widthbytes, height, cudaMemcpyHostToDevice), NCV_CUDA_ERROR);
            }
            ncvStat = NCV_SUCCESS;
            break;
        case NCVMemoryTypeDevice:
            if (cuStream != 0)
            {
                ncvAssertCUDAReturn(cudaMemcpy2DAsync(dst, dstPitch, src, srcPitch, widthbytes, height, cudaMemcpyDeviceToDevice, cuStream), NCV_CUDA_ERROR);
            }
            else
            {
                ncvAssertCUDAReturn(cudaMemcpy2D(dst, dstPitch, src, srcPitch, widthbytes, height, cudaMemcpyDeviceToDevice), NCV_CUDA_ERROR);
            }
            ncvStat = NCV_SUCCESS;
            break;
        default:
            ncvStat = NCV_MEM_RESIDENCE_ERROR;
        }
        break;
    default:
        ncvStat = NCV_MEM_RESIDENCE_ERROR;
    }

    return ncvStat;
}


//===================================================================
//
// NCVMemStackAllocator class members implementation
//
//===================================================================


NCVMemStackAllocator::NCVMemStackAllocator(Ncv32u alignment)
    :
    currentSize(0),
    _maxSize(0),
    allocBegin(NULL),
    begin(NULL),
    end(NULL),
    _memType(NCVMemoryTypeNone),
    _alignment(alignment),
    bReusesMemory(false)
{
    NcvBool bProperAlignment = (alignment & (alignment-1)) == 0;
    ncvAssertPrintCheck(bProperAlignment, "NCVMemStackAllocator ctor:: alignment not power of 2");
}


NCVMemStackAllocator::NCVMemStackAllocator(NCVMemoryType memT, std::size_t capacity, Ncv32u alignment, void *reusePtr)
    :
    currentSize(0),
    _maxSize(0),
    allocBegin(NULL),
    _memType(memT),
    _alignment(alignment)
{
    NcvBool bProperAlignment = (alignment & (alignment-1)) == 0;
    ncvAssertPrintCheck(bProperAlignment, "NCVMemStackAllocator ctor:: _alignment not power of 2");
    ncvAssertPrintCheck(memT != NCVMemoryTypeNone, "NCVMemStackAllocator ctor:: Incorrect allocator type");

    allocBegin = NULL;

    if (reusePtr == NULL && capacity != 0)
    {
        bReusesMemory = false;
        switch (memT)
        {
        case NCVMemoryTypeDevice:
            ncvAssertCUDAReturn(cudaMalloc(&allocBegin, capacity), );
            break;
        case NCVMemoryTypeHostPinned:
            ncvAssertCUDAReturn(cudaMallocHost(&allocBegin, capacity), );
            break;
        case NCVMemoryTypeHostPageable:
            allocBegin = (Ncv8u *)malloc(capacity);
            break;
        default:;
        }
    }
    else
    {
        bReusesMemory = true;
        allocBegin = (Ncv8u *)reusePtr;
    }

    if (capacity == 0)
    {
        allocBegin = (Ncv8u *)(0x1);
    }

    if (!isCounting())
    {
        begin = allocBegin;
        end = begin + capacity;
    }
}


NCVMemStackAllocator::~NCVMemStackAllocator()
{
    if (allocBegin != NULL)
    {
        ncvAssertPrintCheck(currentSize == 0, "NCVMemStackAllocator dtor:: not all objects were deallocated properly, forcing destruction");

        if (!bReusesMemory && (allocBegin != (Ncv8u *)(0x1)))
        {
            switch (_memType)
            {
            case NCVMemoryTypeDevice:
                ncvAssertCUDAReturn(cudaFree(allocBegin), );
                break;
            case NCVMemoryTypeHostPinned:
                ncvAssertCUDAReturn(cudaFreeHost(allocBegin), );
                break;
            case NCVMemoryTypeHostPageable:
                free(allocBegin);
                break;
            default:;
            }
        }

        allocBegin = NULL;
    }
}


NCVStatus NCVMemStackAllocator::alloc(NCVMemSegment &seg, std::size_t size)
{
    seg.clear();
    ncvAssertReturn(isInitialized(), NCV_ALLOCATOR_BAD_ALLOC);

    size = alignUp(size, this->_alignment);
    this->currentSize += size;
    this->_maxSize = max(this->_maxSize, this->currentSize);

    if (!isCounting())
    {
        std::size_t availSize = end - begin;
        ncvAssertReturn(size <= availSize, NCV_ALLOCATOR_INSUFFICIENT_CAPACITY);
    }

    seg.begin.ptr = begin;
    seg.begin.memtype = this->_memType;
    seg.size = size;
    begin += size;

    return NCV_SUCCESS;
}


NCVStatus NCVMemStackAllocator::dealloc(NCVMemSegment &seg)
{
    ncvAssertReturn(isInitialized(), NCV_ALLOCATOR_BAD_ALLOC);
    ncvAssertReturn(seg.begin.memtype == this->_memType, NCV_ALLOCATOR_BAD_DEALLOC);
    ncvAssertReturn(seg.begin.ptr != NULL || isCounting(), NCV_ALLOCATOR_BAD_DEALLOC);
    ncvAssertReturn(seg.begin.ptr == begin - seg.size, NCV_ALLOCATOR_DEALLOC_ORDER);

    currentSize -= seg.size;
    begin -= seg.size;

    seg.clear();

    ncvAssertReturn(allocBegin <= begin, NCV_ALLOCATOR_BAD_DEALLOC);

    return NCV_SUCCESS;
}


NcvBool NCVMemStackAllocator::isInitialized(void) const
{
    return ((this->_alignment & (this->_alignment-1)) == 0) && isCounting() || this->allocBegin != NULL;
}


NcvBool NCVMemStackAllocator::isCounting(void) const
{
    return this->_memType == NCVMemoryTypeNone;
}


NCVMemoryType NCVMemStackAllocator::memType(void) const
{
    return this->_memType;
}


Ncv32u NCVMemStackAllocator::alignment(void) const
{
    return this->_alignment;
}


size_t NCVMemStackAllocator::maxSize(void) const
{
    return this->_maxSize;
}


//===================================================================
//
// NCVMemNativeAllocator class members implementation
//
//===================================================================


NCVMemNativeAllocator::NCVMemNativeAllocator(NCVMemoryType memT, Ncv32u alignment)
    :
    currentSize(0),
    _maxSize(0),
    _memType(memT),
    _alignment(alignment)
{
    ncvAssertPrintReturn(memT != NCVMemoryTypeNone, "NCVMemNativeAllocator ctor:: counting not permitted for this allocator type", );
}


NCVMemNativeAllocator::~NCVMemNativeAllocator()
{
    ncvAssertPrintCheck(currentSize == 0, "NCVMemNativeAllocator dtor:: detected memory leak");
}


NCVStatus NCVMemNativeAllocator::alloc(NCVMemSegment &seg, std::size_t size)
{
    seg.clear();
    ncvAssertReturn(isInitialized(), NCV_ALLOCATOR_BAD_ALLOC);

    switch (this->_memType)
    {
    case NCVMemoryTypeDevice:
        ncvAssertCUDAReturn(cudaMalloc(&seg.begin.ptr, size), NCV_CUDA_ERROR);
        break;
    case NCVMemoryTypeHostPinned:
        ncvAssertCUDAReturn(cudaMallocHost(&seg.begin.ptr, size), NCV_CUDA_ERROR);
        break;
    case NCVMemoryTypeHostPageable:
        seg.begin.ptr = (Ncv8u *)malloc(size);
        break;
    default:;
    }

    this->currentSize += alignUp(size, this->_alignment);
    this->_maxSize = max(this->_maxSize, this->currentSize);

    seg.begin.memtype = this->_memType;
    seg.size = size;

    return NCV_SUCCESS;
}


NCVStatus NCVMemNativeAllocator::dealloc(NCVMemSegment &seg)
{
    ncvAssertReturn(isInitialized(), NCV_ALLOCATOR_BAD_ALLOC);
    ncvAssertReturn(seg.begin.memtype == this->_memType, NCV_ALLOCATOR_BAD_DEALLOC);
    ncvAssertReturn(seg.begin.ptr != NULL, NCV_ALLOCATOR_BAD_DEALLOC);

    ncvAssertReturn(currentSize >= alignUp(seg.size, this->_alignment), NCV_ALLOCATOR_BAD_DEALLOC);
    currentSize -= alignUp(seg.size, this->_alignment);

    switch (this->_memType)
    {
    case NCVMemoryTypeDevice:
        ncvAssertCUDAReturn(cudaFree(seg.begin.ptr), NCV_CUDA_ERROR);
        break;
    case NCVMemoryTypeHostPinned:
        ncvAssertCUDAReturn(cudaFreeHost(seg.begin.ptr), NCV_CUDA_ERROR);
        break;
    case NCVMemoryTypeHostPageable:
        free(seg.begin.ptr);
        break;
    default:;
    }

    seg.clear();

    return NCV_SUCCESS;
}


NcvBool NCVMemNativeAllocator::isInitialized(void) const
{
    return (this->_alignment != 0);
}


NcvBool NCVMemNativeAllocator::isCounting(void) const
{
    return false;
}


NCVMemoryType NCVMemNativeAllocator::memType(void) const
{
    return this->_memType;
}


Ncv32u NCVMemNativeAllocator::alignment(void) const
{
    return this->_alignment;
}


size_t NCVMemNativeAllocator::maxSize(void) const
{
    return this->_maxSize;
}


//===================================================================
//
// Operations with rectangles
//
//===================================================================


template <class T>
static NCVStatus drawRectsWrapperHost(T *h_dst,
                                      Ncv32u dstStride,
                                      Ncv32u dstWidth,
                                      Ncv32u dstHeight,
                                      NcvRect32u *h_rects,
                                      Ncv32u numRects,
                                      T color)
{
    ncvAssertReturn(h_dst != NULL && h_rects != NULL, NCV_NULL_PTR);
    ncvAssertReturn(dstWidth > 0 && dstHeight > 0, NCV_DIMENSIONS_INVALID);
    ncvAssertReturn(dstStride >= dstWidth, NCV_INVALID_STEP);
    ncvAssertReturn(numRects != 0, NCV_SUCCESS);
    ncvAssertReturn(numRects <= dstWidth * dstHeight, NCV_DIMENSIONS_INVALID);

    for (Ncv32u i=0; i<numRects; i++)
    {
        NcvRect32u rect = h_rects[i];

        if (rect.x < dstWidth)
        {
            for (Ncv32u i=rect.y; i<rect.y+rect.height && i<dstHeight; i++)
            {
                h_dst[i*dstStride+rect.x] = color;
            }
        }
        if (rect.x+rect.width-1 < dstWidth)
        {
            for (Ncv32u i=rect.y; i<rect.y+rect.height && i<dstHeight; i++)
            {
                h_dst[i*dstStride+rect.x+rect.width-1] = color;
            }
        }
        if (rect.y < dstHeight)
        {
            for (Ncv32u j=rect.x; j<rect.x+rect.width && j<dstWidth; j++)
            {
                h_dst[rect.y*dstStride+j] = color;
            }
        }
        if (rect.y + rect.height - 1 < dstHeight)
        {
            for (Ncv32u j=rect.x; j<rect.x+rect.width && j<dstWidth; j++)
            {
                h_dst[(rect.y+rect.height-1)*dstStride+j] = color;
            }
        }
    }

    return NCV_SUCCESS;
}


NCVStatus ncvDrawRects_8u_host(Ncv8u *h_dst,
                               Ncv32u dstStride,
                               Ncv32u dstWidth,
                               Ncv32u dstHeight,
                               NcvRect32u *h_rects,
                               Ncv32u numRects,
                               Ncv8u color)
{
    return drawRectsWrapperHost(h_dst, dstStride, dstWidth, dstHeight, h_rects, numRects, color);
}


NCVStatus ncvDrawRects_32u_host(Ncv32u *h_dst,
                                Ncv32u dstStride,
                                Ncv32u dstWidth,
                                Ncv32u dstHeight,
                                NcvRect32u *h_rects,
                                Ncv32u numRects,
                                Ncv32u color)
{
    return drawRectsWrapperHost(h_dst, dstStride, dstWidth, dstHeight, h_rects, numRects, color);
}


const Ncv32u NUMTHREADS_DRAWRECTS = 32;
const Ncv32u NUMTHREADS_DRAWRECTS_LOG2 = 5;


template <class T>
__global__ void drawRects(T *d_dst,
                          Ncv32u dstStride,
                          Ncv32u dstWidth,
                          Ncv32u dstHeight,
                          NcvRect32u *d_rects,
                          Ncv32u numRects,
                          T color)
{
    Ncv32u blockId = blockIdx.y * 65535 + blockIdx.x;
    if (blockId > numRects * 4)
    {
        return;
    }

    NcvRect32u curRect = d_rects[blockId >> 2];
    NcvBool bVertical = blockId & 0x1;
    NcvBool bTopLeft = blockId & 0x2;

    Ncv32u pt0x, pt0y;
    if (bVertical)
    {
        Ncv32u numChunks = (curRect.height + NUMTHREADS_DRAWRECTS - 1) >> NUMTHREADS_DRAWRECTS_LOG2;

        pt0x = bTopLeft ? curRect.x : curRect.x + curRect.width - 1;
        pt0y = curRect.y;

        if (pt0x < dstWidth)
        {
            for (Ncv32u chunkId = 0; chunkId < numChunks; chunkId++)
            {
                Ncv32u ptY = pt0y + chunkId * NUMTHREADS_DRAWRECTS + threadIdx.x;
                if (ptY < pt0y + curRect.height && ptY < dstHeight)
                {
                    d_dst[ptY * dstStride + pt0x] = color;
                }
            }
        }
    }
    else
    {
        Ncv32u numChunks = (curRect.width + NUMTHREADS_DRAWRECTS - 1) >> NUMTHREADS_DRAWRECTS_LOG2;

        pt0x = curRect.x;
        pt0y = bTopLeft ? curRect.y : curRect.y + curRect.height - 1;

        if (pt0y < dstHeight)
        {
            for (Ncv32u chunkId = 0; chunkId < numChunks; chunkId++)
            {
                Ncv32u ptX = pt0x + chunkId * NUMTHREADS_DRAWRECTS + threadIdx.x;
                if (ptX < pt0x + curRect.width && ptX < dstWidth)
                {
                    d_dst[pt0y * dstStride + ptX] = color;
                }
            }
        }
    }
}


template <class T>
static NCVStatus drawRectsWrapperDevice(T *d_dst,
                                        Ncv32u dstStride,
                                        Ncv32u dstWidth,
                                        Ncv32u dstHeight,
                                        NcvRect32u *d_rects,
                                        Ncv32u numRects,
                                        T color,
                                        cudaStream_t cuStream)
{
    ncvAssertReturn(d_dst != NULL && d_rects != NULL, NCV_NULL_PTR);
    ncvAssertReturn(dstWidth > 0 && dstHeight > 0, NCV_DIMENSIONS_INVALID);
    ncvAssertReturn(dstStride >= dstWidth, NCV_INVALID_STEP);
    ncvAssertReturn(numRects <= dstWidth * dstHeight, NCV_DIMENSIONS_INVALID);

    if (numRects == 0)
    {
        return NCV_SUCCESS;
    }

    dim3 grid(numRects * 4);
    dim3 block(NUMTHREADS_DRAWRECTS);
    if (grid.x > 65535)
    {
        grid.y = (grid.x + 65534) / 65535;
        grid.x = 65535;
    }

    drawRects<T><<<grid, block>>>(d_dst, dstStride, dstWidth, dstHeight, d_rects, numRects, color);

    ncvAssertCUDALastErrorReturn(NCV_CUDA_ERROR);

    return NCV_SUCCESS;
}


NCVStatus ncvDrawRects_8u_device(Ncv8u *d_dst,
                                 Ncv32u dstStride,
                                 Ncv32u dstWidth,
                                 Ncv32u dstHeight,
                                 NcvRect32u *d_rects,
                                 Ncv32u numRects,
                                 Ncv8u color,
                                 cudaStream_t cuStream)
{
    return drawRectsWrapperDevice(d_dst, dstStride, dstWidth, dstHeight, d_rects, numRects, color, cuStream);
}


NCVStatus ncvDrawRects_32u_device(Ncv32u *d_dst,
                                  Ncv32u dstStride,
                                  Ncv32u dstWidth,
                                  Ncv32u dstHeight,
                                  NcvRect32u *d_rects,
                                  Ncv32u numRects,
                                  Ncv32u color,
                                  cudaStream_t cuStream)
{
    return drawRectsWrapperDevice(d_dst, dstStride, dstWidth, dstHeight, d_rects, numRects, color, cuStream);
}

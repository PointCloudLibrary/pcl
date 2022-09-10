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

#ifndef PCL_GPU_PEOPLE__NCV_HPP_
#define PCL_GPU_PEOPLE__NCV_HPP_

#if (defined _WIN32 || defined WINCE) && defined CVAPI_EXPORTS
    #define NCV_EXPORTS __declspec(dllexport)
#else
    #define NCV_EXPORTS
#endif

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
#endif

#include <cuda_runtime.h>
#include <sstream>
#include <iostream>
#include <pcl/console/print.h>

//==============================================================================
//
// Compile-time assert functionality
//
//==============================================================================

/**
* Compile-time assert namespace
*/
namespace NcvCTprep
{
    template <bool x>
    struct CT_ASSERT_FAILURE;

    template <>
    struct CT_ASSERT_FAILURE<true> {};

    template <int x>
    struct assertTest{};
}

#define NCV_CT_PREP_PASTE_AUX(a,b)      a##b                         ///< Concatenation indirection macro
#define NCV_CT_PREP_PASTE(a,b)          NCV_CT_PREP_PASTE_AUX(a, b)  ///< Concatenation macro

/**
* Performs compile-time assertion of a condition on the file scope
*/
#define NCV_CT_ASSERT(X) \
    typedef NcvCTprep::assertTest<sizeof(NcvCTprep::CT_ASSERT_FAILURE< (bool)(X) >)> \
    NCV_CT_PREP_PASTE(__ct_assert_typedef_, __LINE__)

//==============================================================================
//
// Alignment macros
//
//==============================================================================

#if !defined(__align__) && !defined(__CUDACC__)
    #if defined(_WIN32) || defined(_WIN64)
        #define __align__(n)         __declspec(align(n))
    #elif defined(__unix__)
        #define __align__(n)         __attribute__((__aligned__(n)))
    #endif
#endif

//==============================================================================
//
// Integral and compound types of guaranteed size
//
//==============================================================================

using NcvBool = bool;
using Ncv64s = long long;

#if defined(__APPLE__) && !defined(__CUDACC__)
    using Ncv64u = std::uint64_t;
#else
    using Ncv64u = unsigned long long;
#endif

using Ncv32s = int;
using Ncv32u = unsigned int;
using Ncv16s = short;
using Ncv16u = unsigned short;
using Ncv8s = char;
using Ncv8u = unsigned char;
using Ncv32f = float;
using Ncv64f = double;

struct NcvRect8u
{
    Ncv8u x;
    Ncv8u y;
    Ncv8u width;
    Ncv8u height;
    __host__ __device__ NcvRect8u() : x(0), y(0), width(0), height(0) {};
    __host__ __device__ NcvRect8u(Ncv8u x, Ncv8u y, Ncv8u width, Ncv8u height) : x(x), y(y), width(width), height(height) {}
};

struct NcvRect32s
{
    Ncv32s x;          ///< x-coordinate of upper left corner.
    Ncv32s y;          ///< y-coordinate of upper left corner.
    Ncv32s width;      ///< Rectangle width.
    Ncv32s height;     ///< Rectangle height.
    __host__ __device__ NcvRect32s() : x(0), y(0), width(0), height(0) {};
    __host__ __device__ NcvRect32s(Ncv32s x, Ncv32s y, Ncv32s width, Ncv32s height) : x(x), y(y), width(width), height(height) {}
};

struct NcvRect32u
{
    Ncv32u x;          ///< x-coordinate of upper left corner.
    Ncv32u y;          ///< y-coordinate of upper left corner.
    Ncv32u width;      ///< Rectangle width.
    Ncv32u height;     ///< Rectangle height.
    __host__ __device__ NcvRect32u() : x(0), y(0), width(0), height(0) {};
    __host__ __device__ NcvRect32u(Ncv32u x, Ncv32u y, Ncv32u width, Ncv32u height) : x(x), y(y), width(width), height(height) {}
};

struct NcvSize32s
{
    Ncv32s width;  ///< Rectangle width.
    Ncv32s height; ///< Rectangle height.
    __host__ __device__ NcvSize32s() : width(0), height(0) {};
    __host__ __device__ NcvSize32s(Ncv32s width, Ncv32s height) : width(width), height(height) {}
};

struct NcvSize32u
{
    Ncv32u width;  ///< Rectangle width.
    Ncv32u height; ///< Rectangle height.
    __host__ __device__ NcvSize32u() : width(0), height(0) {};
    __host__ __device__ NcvSize32u(Ncv32u width, Ncv32u height) : width(width), height(height) {}
    __host__ __device__ bool operator == (const NcvSize32u &another) const {return this->width == another.width && this->height == another.height;}
};

struct NcvPoint2D32s
{
    Ncv32s x; ///< Point X.
    Ncv32s y; ///< Point Y.
    __host__ __device__ NcvPoint2D32s() : x(0), y(0) {};
    __host__ __device__ NcvPoint2D32s(Ncv32s x, Ncv32s y) : x(x), y(y) {}
};

struct NcvPoint2D32u
{
    Ncv32u x; ///< Point X.
    Ncv32u y; ///< Point Y.
    __host__ __device__ NcvPoint2D32u() : x(0), y(0) {};
    __host__ __device__ NcvPoint2D32u(Ncv32u x, Ncv32u y) : x(x), y(y) {}
};

NCV_CT_ASSERT(sizeof(NcvBool) <= 4);
NCV_CT_ASSERT(sizeof(Ncv64s) == 8);
NCV_CT_ASSERT(sizeof(Ncv64u) == 8);
NCV_CT_ASSERT(sizeof(Ncv32s) == 4);
NCV_CT_ASSERT(sizeof(Ncv32u) == 4);
NCV_CT_ASSERT(sizeof(Ncv16s) == 2);
NCV_CT_ASSERT(sizeof(Ncv16u) == 2);
NCV_CT_ASSERT(sizeof(Ncv8s) == 1);
NCV_CT_ASSERT(sizeof(Ncv8u) == 1);
NCV_CT_ASSERT(sizeof(Ncv32f) == 4);
NCV_CT_ASSERT(sizeof(Ncv64f) == 8);
NCV_CT_ASSERT(sizeof(NcvRect8u) == sizeof(Ncv32u));
NCV_CT_ASSERT(sizeof(NcvRect32s) == 4 * sizeof(Ncv32s));
NCV_CT_ASSERT(sizeof(NcvRect32u) == 4 * sizeof(Ncv32u));
NCV_CT_ASSERT(sizeof(NcvSize32u) == 2 * sizeof(Ncv32u));
NCV_CT_ASSERT(sizeof(NcvPoint2D32u) == 2 * sizeof(Ncv32u));


//==============================================================================
//
// Persistent constants
//
//==============================================================================

const Ncv32u K_WARP_SIZE = 32;
const Ncv32u K_LOG2_WARP_SIZE = 5;

//==============================================================================
//
// Error handling
//
//==============================================================================

NCV_EXPORTS void ncvDebugOutput(const std::string &msg);

using NCVDebugOutputHandler = void (const std::string &);

NCV_EXPORTS void ncvSetDebugOutputHandler(NCVDebugOutputHandler* func);

#define ncvAssertPrintCheck(pred, msg) \
    do \
    { \
        if (!(pred)) \
        { \
            std::ostringstream oss; \
            oss << "NCV Assertion Failed: " << msg << ", file=" << __FILE__ << ", line=" << __LINE__ << std::endl; \
            ncvDebugOutput(oss.str()); \
        } \
    } while (0)

#define ncvAssertPrintReturn(pred, msg, err) \
    do \
    { \
        ncvAssertPrintCheck(pred, msg); \
        if (!(pred)) return err; \
    } while (0)

#define ncvAssertReturn(pred, err) \
    ncvAssertPrintReturn(pred, "retcode=" << (int)err, err)

#define ncvAssertReturnNcvStat(ncvOp) \
    do \
    { \
        NCVStatus _ncvStat = ncvOp; \
        ncvAssertPrintReturn(NCV_SUCCESS==_ncvStat, "NcvStat=" << (int)_ncvStat, _ncvStat); \
    } while (0)

#define ncvAssertCUDAReturn(cudacall, errCode) \
    do \
    { \
        cudaError_t res = cudacall; \
        ncvAssertPrintReturn(cudaSuccess==res, "cudaError_t=" << res, errCode); \
    } while (0)

#define ncvAssertCUDALastErrorReturn(errCode) \
    do \
    { \
        cudaError_t res = cudaGetLastError(); \
        ncvAssertPrintReturn(cudaSuccess==res, "cudaError_t=" << res, errCode); \
    } while (0)

/**
* \brief Return-codes for status notification, errors and warnings
*/
enum
{
    //NCV statuses
    NCV_SUCCESS,
    NCV_UNKNOWN_ERROR,

    NCV_CUDA_ERROR,
    NCV_NPP_ERROR,
    NCV_FILE_ERROR,

    NCV_NULL_PTR,
    NCV_INCONSISTENT_INPUT,
    NCV_TEXTURE_BIND_ERROR,
    NCV_DIMENSIONS_INVALID,

    NCV_INVALID_ROI,
    NCV_INVALID_STEP,
    NCV_INVALID_SCALE,

    NCV_ALLOCATOR_NOT_INITIALIZED,
    NCV_ALLOCATOR_BAD_ALLOC,
    NCV_ALLOCATOR_BAD_DEALLOC,
    NCV_ALLOCATOR_INSUFFICIENT_CAPACITY,
    NCV_ALLOCATOR_DEALLOC_ORDER,
    NCV_ALLOCATOR_BAD_REUSE,

    NCV_MEM_COPY_ERROR,
    NCV_MEM_RESIDENCE_ERROR,
    NCV_MEM_INSUFFICIENT_CAPACITY,

    NCV_HAAR_INVALID_PIXEL_STEP,
    NCV_HAAR_TOO_MANY_FEATURES_IN_CLASSIFIER,
    NCV_HAAR_TOO_MANY_FEATURES_IN_CASCADE,
    NCV_HAAR_TOO_LARGE_FEATURES,
    NCV_HAAR_XML_LOADING_EXCEPTION,

    NCV_NOIMPL_HAAR_TILTED_FEATURES,
    NCV_NOT_IMPLEMENTED,

    NCV_WARNING_HAAR_DETECTIONS_VECTOR_OVERFLOW,

    //NPP statuses
    NPPST_SUCCESS = NCV_SUCCESS,              ///< Successful operation (same as NPP_NO_ERROR)
    NPPST_ERROR,                              ///< Unknown error
    NPPST_CUDA_KERNEL_EXECUTION_ERROR,        ///< CUDA kernel execution error
    NPPST_NULL_POINTER_ERROR,                 ///< NULL pointer argument error
    NPPST_TEXTURE_BIND_ERROR,                 ///< CUDA texture binding error or non-zero offset returned
    NPPST_MEMCPY_ERROR,                       ///< CUDA memory copy error
    NPPST_MEM_ALLOC_ERR,                      ///< CUDA memory allocation error
    NPPST_MEMFREE_ERR,                        ///< CUDA memory deallocation error

    //NPPST statuses
    NPPST_INVALID_ROI,                        ///< Invalid region of interest argument
    NPPST_INVALID_STEP,                       ///< Invalid image lines step argument (check sign, alignment, relation to image width)
    NPPST_INVALID_SCALE,                      ///< Invalid scale parameter passed
    NPPST_MEM_INSUFFICIENT_BUFFER,            ///< Insufficient user-allocated buffer
    NPPST_MEM_RESIDENCE_ERROR,                ///< Memory residence error detected (check if pointers should be device or pinned)
    NPPST_MEM_INTERNAL_ERROR,                 ///< Internal memory management error

    NCV_LAST_STATUS                           ///< Marker to continue error numeration in other files
};

using NCVStatus = Ncv32u;

#define NCV_SET_SKIP_COND(x) \
    bool __ncv_skip_cond = x

#define NCV_RESET_SKIP_COND(x) \
    __ncv_skip_cond = x

#define NCV_SKIP_COND_BEGIN \
    if (!__ncv_skip_cond) {

#define NCV_SKIP_COND_END \
    }


//==============================================================================
//
// Memory management classes template compound types
//
//==============================================================================


/**
* Calculates the aligned top bound value
*/
NCV_EXPORTS Ncv32u alignUp(Ncv32u what, Ncv32u alignment);


/**
* NCVMemoryType
*/
enum NCVMemoryType
{
    NCVMemoryTypeNone,
    NCVMemoryTypeHostPageable,
    NCVMemoryTypeHostPinned,
    NCVMemoryTypeDevice
};


/**
* NCVMemPtr
*/
struct NCV_EXPORTS NCVMemPtr
{
    void *ptr;
    NCVMemoryType memtype;
    void clear();
};


/**
* NCVMemSegment
*/
struct NCV_EXPORTS NCVMemSegment
{
    NCVMemPtr begin;
    std::size_t size;
    void clear();
};


/**
* INCVMemAllocator (Interface)
*/
class NCV_EXPORTS INCVMemAllocator
{
public:
    virtual ~INCVMemAllocator() = 0;

    virtual NCVStatus alloc(NCVMemSegment &seg, std::size_t size) = 0;
    virtual NCVStatus dealloc(NCVMemSegment &seg) = 0;

    virtual NcvBool isInitialized() const = 0;
    virtual NcvBool isCounting() const = 0;
    
    virtual NCVMemoryType memType() const = 0;
    virtual Ncv32u alignment() const = 0;
    virtual std::size_t maxSize() const = 0;
};

inline INCVMemAllocator::~INCVMemAllocator() {}


/**
* NCVMemStackAllocator
*/
class NCV_EXPORTS NCVMemStackAllocator : public INCVMemAllocator
{
    NCVMemStackAllocator();
    NCVMemStackAllocator(const NCVMemStackAllocator &);

public:

    explicit NCVMemStackAllocator(Ncv32u alignment);
    NCVMemStackAllocator(NCVMemoryType memT, std::size_t capacity, Ncv32u alignment, void *reusePtr=nullptr);
    virtual ~NCVMemStackAllocator();

    virtual NCVStatus alloc(NCVMemSegment &seg, std::size_t size);
    virtual NCVStatus dealloc(NCVMemSegment &seg);

    virtual NcvBool isInitialized() const;
    virtual NcvBool isCounting() const;

    virtual NCVMemoryType memType() const;
    virtual Ncv32u alignment() const;
    virtual std::size_t maxSize() const;

private:

    NCVMemoryType _memType;
    Ncv32u _alignment;
    Ncv8u *allocBegin;
    Ncv8u *begin;
    Ncv8u *end;
    std::size_t currentSize;
    std::size_t _maxSize;
    NcvBool bReusesMemory;
};


/**
* NCVMemNativeAllocator
*/
class NCV_EXPORTS NCVMemNativeAllocator : public INCVMemAllocator
{
public:

    NCVMemNativeAllocator(NCVMemoryType memT, Ncv32u alignment);
    virtual ~NCVMemNativeAllocator();

    virtual NCVStatus alloc(NCVMemSegment &seg, std::size_t size);
    virtual NCVStatus dealloc(NCVMemSegment &seg);

    virtual NcvBool isInitialized() const;
    virtual NcvBool isCounting() const;

    virtual NCVMemoryType memType() const;
    virtual Ncv32u alignment() const;
    virtual std::size_t maxSize() const;

private:

    NCVMemNativeAllocator();
    NCVMemNativeAllocator(const NCVMemNativeAllocator &);

    NCVMemoryType _memType;
    Ncv32u _alignment;
    std::size_t currentSize;
    std::size_t _maxSize;
};


/**
* Copy dispatchers
*/
NCV_EXPORTS NCVStatus memSegCopyHelper(void *dst, NCVMemoryType dstType,
                                       const void *src, NCVMemoryType srcType,
                                       std::size_t sz, cudaStream_t cuStream);


NCV_EXPORTS NCVStatus memSegCopyHelper2D(void *dst, Ncv32u dstPitch, NCVMemoryType dstType,
                                         const void *src, Ncv32u srcPitch, NCVMemoryType srcType,
                                         Ncv32u widthbytes, Ncv32u height, cudaStream_t cuStream);


/**
* NCVVector (1D)
*/
template <class T>
class NCVVector
{
    NCVVector(const NCVVector &) = delete;

public:

    NCVVector()
    {
        clear();
    }

    virtual ~NCVVector() {}

    void clear()
    {
        _ptr = nullptr;
        _length = 0;
        _memtype = NCVMemoryTypeNone;
    }

    NCVStatus copySolid(NCVVector<T> &dst, cudaStream_t cuStream, std::size_t howMuch=0) const
    {
        if (howMuch == 0)
        {
            ncvAssertReturn(dst._length == this->_length, NCV_MEM_COPY_ERROR);
            howMuch = this->_length * sizeof(T);
        }
        else
        {
            ncvAssertReturn(dst._length * sizeof(T) >= howMuch && 
                this->_length * sizeof(T) >= howMuch &&
                howMuch > 0, NCV_MEM_COPY_ERROR);
        }
        ncvAssertReturn((this->_ptr != nullptr || this->_memtype == NCVMemoryTypeNone) && 
                        (dst._ptr != nullptr || dst._memtype == NCVMemoryTypeNone), NCV_NULL_PTR);

        NCVStatus ncvStat = NCV_SUCCESS;
        if (this->_memtype != NCVMemoryTypeNone)
        {
            ncvStat = memSegCopyHelper(dst._ptr, dst._memtype,
                                       this->_ptr, this->_memtype,
                                       howMuch, cuStream);
        }

        return ncvStat;
    }

    T *ptr() const {return this->_ptr;}
    std::size_t length() const {return this->_length;}
    NCVMemoryType memType() const {return this->_memtype;}

protected:

    T *_ptr;
    std::size_t _length;
    NCVMemoryType _memtype;
};


/**
* NCVVectorAlloc
*/
template <class T>
class NCVVectorAlloc : public NCVVector<T>
{
    NCVVectorAlloc() = delete;
    NCVVectorAlloc(const NCVVectorAlloc &) = delete;
    NCVVectorAlloc& operator=(const NCVVectorAlloc<T>&) = delete;	

public:

    NCVVectorAlloc(INCVMemAllocator &allocator, Ncv32u length)
        :
        allocator(allocator)
    {
        NCVStatus ncvStat;

        this->clear();
        this->allocatedMem.clear();

        ncvStat = allocator.alloc(this->allocatedMem, length * sizeof(T));
        ncvAssertPrintReturn(ncvStat == NCV_SUCCESS, "NCVVectorAlloc ctor:: alloc failed", );

        this->_ptr = (T *)this->allocatedMem.begin.ptr;
        this->_length = length;
        this->_memtype = this->allocatedMem.begin.memtype;
    }

    ~NCVVectorAlloc()
    {
        NCVStatus ncvStat;

        ncvStat = allocator.dealloc(this->allocatedMem);
        ncvAssertPrintCheck(ncvStat == NCV_SUCCESS, "NCVVectorAlloc dtor:: dealloc failed");

        this->clear();
    }

    NcvBool isMemAllocated() const
    {
        return (this->allocatedMem.begin.ptr != nullptr) || (this->allocator.isCounting());
    }

    Ncv32u getAllocatorsAlignment() const
    {
        return allocator.alignment();
    }

    NCVMemSegment getSegment() const
    {
        return allocatedMem;
    }

private:
    INCVMemAllocator &allocator;
    NCVMemSegment allocatedMem;
};


/**
* NCVVectorReuse
*/
template <class T>
class NCVVectorReuse : public NCVVector<T>
{
    NCVVectorReuse() = delete;
    NCVVectorReuse(const NCVVectorReuse &) = delete;

public:

    explicit NCVVectorReuse(const NCVMemSegment &memSegment)
    {
        this->bReused = false;
        this->clear();

        this->_length = memSegment.size / sizeof(T);
        this->_ptr = (T *)memSegment.begin.ptr;
        this->_memtype = memSegment.begin.memtype;

        this->bReused = true;
    }

    NCVVectorReuse(const NCVMemSegment &memSegment, Ncv32u length)
    {
        this->bReused = false;
        this->clear();

        ncvAssertPrintReturn(length * sizeof(T) <= memSegment.size, \
            "NCVVectorReuse ctor:: memory binding failed due to size mismatch", );

        this->_length = length;
        this->_ptr = (T *)memSegment.begin.ptr;
        this->_memtype = memSegment.begin.memtype;

        this->bReused = true;
    }

    NcvBool isMemReused() const
    {
        return this->bReused;
    }

private:

    NcvBool bReused;
};


/**
* NCVMatrix (2D)
*/
template <class T>
class NCVMatrix
{
    NCVMatrix(const NCVMatrix &) = delete;

public:

    NCVMatrix()
    {
        clear();
    }

    virtual ~NCVMatrix() {}

    void clear()
    {
        _ptr = nullptr;
        _pitch = 0;
        _width = 0;
        _height = 0;
        _memtype = NCVMemoryTypeNone;
    }

    Ncv32u stride() const
    {
        return _pitch / sizeof(T);
    }

    //a side effect of this function is that it copies everything in a single chunk, so the "padding" will be overwritten
    NCVStatus copySolid(NCVMatrix<T> &dst, cudaStream_t cuStream, std::size_t howMuch=0) const
    {
        if (howMuch == 0)
        {
            ncvAssertReturn(dst._pitch == this->_pitch &&
                            dst._height == this->_height, NCV_MEM_COPY_ERROR);
            howMuch = this->_pitch * this->_height;
        }
        else
        {
            ncvAssertReturn(dst._pitch * dst._height >= howMuch && 
                            this->_pitch * this->_height >= howMuch &&
                            howMuch > 0, NCV_MEM_COPY_ERROR);
        }
        ncvAssertReturn((this->_ptr != nullptr || this->_memtype == NCVMemoryTypeNone) && 
                        (dst._ptr != nullptr || dst._memtype == NCVMemoryTypeNone), NCV_NULL_PTR);

        NCVStatus ncvStat = NCV_SUCCESS;
        if (this->_memtype != NCVMemoryTypeNone)
        {
            ncvStat = memSegCopyHelper(dst._ptr, dst._memtype, 
                                       this->_ptr, this->_memtype, 
                                       howMuch, cuStream);
        }

        return ncvStat;
    }

    NCVStatus copy2D(NCVMatrix<T> &dst, NcvSize32u roi, cudaStream_t cuStream) const
    {
        ncvAssertReturn(this->width() >= roi.width && this->height() >= roi.height &&
                        dst.width() >= roi.width && dst.height() >= roi.height, NCV_MEM_COPY_ERROR);
        ncvAssertReturn((this->_ptr != NULL || this->_memtype == NCVMemoryTypeNone) && 
                        (dst._ptr != NULL || dst._memtype == NCVMemoryTypeNone), NCV_NULL_PTR);

        NCVStatus ncvStat = NCV_SUCCESS;
        if (this->_memtype != NCVMemoryTypeNone)
        {
            ncvStat = memSegCopyHelper2D(dst._ptr, dst._pitch, dst._memtype,
                                         this->_ptr, this->_pitch, this->_memtype,
                                         roi.width * sizeof(T), roi.height, cuStream);
        }

        return ncvStat;
    }

    T &at(Ncv32u x, Ncv32u y) const
    {
        NcvBool bOutRange = (x >= this->_width || y >= this->_height);
        ncvAssertPrintCheck(!bOutRange, "Error addressing matrix at [" << x << ", " << y << "]");
        if (bOutRange)
        {
            return *this->_ptr;
        }
        return ((T *)((Ncv8u *)this->_ptr + y * this->_pitch))[x];
    }

    T *ptr() const {return this->_ptr;}
    Ncv32u width() const {return this->_width;}
    Ncv32u height() const {return this->_height;}
    NcvSize32u size() const {return NcvSize32u(this->_width, this->_height);}
    Ncv32u pitch() const {return this->_pitch;}
    NCVMemoryType memType() const {return this->_memtype;}

protected:

    T *_ptr;
    Ncv32u _width;
    Ncv32u _height;
    Ncv32u _pitch;
    NCVMemoryType _memtype;
};


/**
* NCVMatrixAlloc
*/
template <class T>
class NCVMatrixAlloc : public NCVMatrix<T>
{
    NCVMatrixAlloc() = delete;
    NCVMatrixAlloc(const NCVMatrixAlloc &) = delete;
    NCVMatrixAlloc& operator=(const NCVMatrixAlloc &) = delete;
public:

    NCVMatrixAlloc(INCVMemAllocator &allocator, Ncv32u width, Ncv32u height, Ncv32u pitch=0)
        :
        allocator(allocator)
    {
        NCVStatus ncvStat;

        this->clear();
        this->allocatedMem.clear();

        Ncv32u widthBytes = width * sizeof(T);
        Ncv32u pitchBytes = alignUp(widthBytes, allocator.alignment());

        if (pitch != 0)
        {
            ncvAssertPrintReturn(pitch >= pitchBytes &&
                (pitch & (allocator.alignment() - 1)) == 0,
                "NCVMatrixAlloc ctor:: incorrect pitch passed", );
            pitchBytes = pitch;
        }

        Ncv32u requiredAllocSize = pitchBytes * height;

        ncvStat = allocator.alloc(this->allocatedMem, requiredAllocSize);
        ncvAssertPrintReturn(ncvStat == NCV_SUCCESS, "NCVMatrixAlloc ctor:: alloc failed", );

        this->_ptr = (T *)this->allocatedMem.begin.ptr;
        this->_width = width;
        this->_height = height;
        this->_pitch = pitchBytes;
        this->_memtype = this->allocatedMem.begin.memtype;
    }

    ~NCVMatrixAlloc()
    {
        NCVStatus ncvStat;

        ncvStat = allocator.dealloc(this->allocatedMem);
        ncvAssertPrintCheck(ncvStat == NCV_SUCCESS, "NCVMatrixAlloc dtor:: dealloc failed");

        this->clear();
    }

    NcvBool isMemAllocated() const
    {
        return (this->allocatedMem.begin.ptr != nullptr) || (this->allocator.isCounting());
    }

    Ncv32u getAllocatorsAlignment() const
    {
        return allocator.alignment();
    }

    NCVMemSegment getSegment() const
    {
        return allocatedMem;
    }

private:

    INCVMemAllocator &allocator;
    NCVMemSegment allocatedMem;
};


/**
* NCVMatrixReuse
*/
template <class T>
class NCVMatrixReuse : public NCVMatrix<T>
{
    NCVMatrixReuse() = delete;
    NCVMatrixReuse(const NCVMatrixReuse &) = delete;

public:

    NCVMatrixReuse(const NCVMemSegment &memSegment, Ncv32u alignment, Ncv32u width, Ncv32u height, Ncv32u pitch=0, NcvBool bSkipPitchCheck=false)
    {
        this->bReused = false;
        this->clear();

        Ncv32u widthBytes = width * sizeof(T);
        Ncv32u pitchBytes = alignUp(widthBytes, alignment);

        if (pitch != 0)
        {
            if (!bSkipPitchCheck)
            {
                ncvAssertPrintReturn(pitch >= pitchBytes &&
                    (pitch & (alignment - 1)) == 0,
                    "NCVMatrixReuse ctor:: incorrect pitch passed", );
            }
            else
            {
                ncvAssertPrintReturn(pitch >= widthBytes, "NCVMatrixReuse ctor:: incorrect pitch passed", );
            }
            pitchBytes = pitch;
        }

        ncvAssertPrintReturn(pitchBytes * height <= memSegment.size, \
            "NCVMatrixReuse ctor:: memory binding failed due to size mismatch", );

        this->_width = width;
        this->_height = height;
        this->_pitch = pitchBytes;
        this->_ptr = (T *)memSegment.begin.ptr;
        this->_memtype = memSegment.begin.memtype;

        this->bReused = true;
    }

    NCVMatrixReuse(const NCVMatrix<T> &mat, NcvRect32u roi)
    {
        this->bReused = false;
        this->clear();

        ncvAssertPrintReturn(roi.x < mat.width() && roi.y < mat.height() && \
            roi.x + roi.width <= mat.width() && roi.y + roi.height <= mat.height(),
            "NCVMatrixReuse ctor:: memory binding failed due to mismatching ROI and source matrix dims", );

        this->_width = roi.width;
        this->_height = roi.height;
        this->_pitch = mat.pitch();
        this->_ptr = &mat.at(roi.x, roi.y);
        this->_memtype = mat.memType();

        this->bReused = true;
    }

    NcvBool isMemReused() const
    {
        return this->bReused;
    }

private:

    NcvBool bReused;
};

/**
* Operations with rectangles
*/
NCV_EXPORTS NCVStatus ncvGroupRectangles_host(NCVVector<NcvRect32u> &hypotheses, Ncv32u &numHypotheses,
                                              Ncv32u minNeighbors, Ncv32f intersectEps, NCVVector<Ncv32u> *hypothesesWeights);

NCV_EXPORTS NCVStatus ncvDrawRects_8u_host(Ncv8u *h_dst, Ncv32u dstStride, Ncv32u dstWidth, Ncv32u dstHeight,
                                           NcvRect32u *h_rects, Ncv32u numRects, Ncv8u color);

NCV_EXPORTS NCVStatus ncvDrawRects_32u_host(Ncv32u *h_dst, Ncv32u dstStride, Ncv32u dstWidth, Ncv32u dstHeight,
                                            NcvRect32u *h_rects, Ncv32u numRects, Ncv32u color);

NCV_EXPORTS NCVStatus ncvDrawRects_8u_device(Ncv8u *d_dst, Ncv32u dstStride, Ncv32u dstWidth, Ncv32u dstHeight,
                                             NcvRect32u *d_rects, Ncv32u numRects, Ncv8u color, cudaStream_t cuStream);

NCV_EXPORTS NCVStatus ncvDrawRects_32u_device(Ncv32u *d_dst, Ncv32u dstStride, Ncv32u dstWidth, Ncv32u dstHeight,
                                              NcvRect32u *d_rects, Ncv32u numRects, Ncv32u color, cudaStream_t cuStream);

#define CLAMP(x,a,b)        ( (x) > (b) ? (b) : ( (x) < (a) ? (a) : (x) ) )
#define CLAMP_TOP(x, a)     (((x) > (a)) ? (a) : (x))
#define CLAMP_BOTTOM(x, a)  (((x) < (a)) ? (a) : (x))
#define CLAMP_0_255(x)      CLAMP(x,0,255)

#define SUB_BEGIN(type, name)    struct { __inline type name
#define SUB_END(name)            } name;
#define SUB_CALL(name)           name.name

#define SQR(x)              ((x)*(x))

#define ncvSafeMatAlloc(name, type, alloc, width, height, err) \
    NCVMatrixAlloc<type> name(alloc, width, height); \
    ncvAssertReturn(name.isMemAllocated(), err);

#endif // PCL_GPU_PEOPLE__NCV_HPP_

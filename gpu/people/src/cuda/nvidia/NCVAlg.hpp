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

#ifndef _ncv_alg_hpp_
#define _ncv_alg_hpp_

#include "NCV.hpp"


template <class T>
static void swap(T &p1, T &p2)
{
    T tmp = p1;
    p1 = p2;
    p2 = tmp;
}


template<typename T>
static T divUp(T a, T b)
{
    return (a + b - 1) / b;
}


template<typename T>
struct functorAddValues
{
    static __device__ __inline__ void assign(volatile T *dst, volatile T *src)
    {
        //Works only for integral types. If you see compiler error here, then you have to specify how to copy your object as a set of integral fields.
        *dst = *src;
    }
    static __device__ __inline__ void reduce(volatile T &in1out, const volatile T &in2)
    {
        in1out += in2;
    }
};


template<typename T>
struct functorMinValues
{
    static __device__ __inline__ void assign(volatile T *dst, volatile T *src)
    {
        //Works only for integral types. If you see compiler error here, then you have to specify how to copy your object as a set of integral fields.
        *dst = *src;
    }
    static __device__ __inline__ void reduce(volatile T &in1out, const volatile T &in2)
    {
        in1out = in1out > in2 ? in2 : in1out;
    }
};


template<typename T>
struct functorMaxValues
{
    static __device__ __inline__ void assign(volatile T *dst, volatile T *src)
    {
        //Works only for integral types. If you see compiler error here, then you have to specify how to copy your object as a set of integral fields.
        *dst = *src;
    }
    static __device__ __inline__ void reduce(volatile T &in1out, const volatile T &in2)
    {
        in1out = in1out > in2 ? in1out : in2;
    }
};


template<typename Tdata, class Tfunc, Ncv32u nThreads>
static __device__ Tdata subReduce(Tdata threadElem)
{
    Tfunc functor;

    __shared__ Tdata _reduceArr[nThreads];
    volatile Tdata *reduceArr = _reduceArr;
    functor.assign(reduceArr + threadIdx.x, &threadElem);
    __syncthreads();

    if (nThreads >= 256 && threadIdx.x < 128)
    {
        functor.reduce(reduceArr[threadIdx.x], reduceArr[threadIdx.x + 128]);
    }
    __syncthreads();

    if (nThreads >= 128 && threadIdx.x < 64)
    {
        functor.reduce(reduceArr[threadIdx.x], reduceArr[threadIdx.x + 64]);
    }
    __syncthreads();

    if (threadIdx.x < 32)
    {
        if (nThreads >= 64)
        {
            functor.reduce(reduceArr[threadIdx.x], reduceArr[threadIdx.x + 32]);
        }
        if (nThreads >= 32 && threadIdx.x < 16)
        {
            functor.reduce(reduceArr[threadIdx.x], reduceArr[threadIdx.x + 16]);
            functor.reduce(reduceArr[threadIdx.x], reduceArr[threadIdx.x + 8]);
            functor.reduce(reduceArr[threadIdx.x], reduceArr[threadIdx.x + 4]);
            functor.reduce(reduceArr[threadIdx.x], reduceArr[threadIdx.x + 2]);
            functor.reduce(reduceArr[threadIdx.x], reduceArr[threadIdx.x + 1]);
        }
    }

    __syncthreads();
    Tdata reduceRes;
    functor.assign(&reduceRes, reduceArr);
    return reduceRes;
}


#endif //_ncv_alg_hpp_

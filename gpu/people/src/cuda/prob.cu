/*
  * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 * @authors: Cedric Cagniart, Koen Buys, Anatoly Baksheev
 *
 */

#include <pcl/gpu/people/tree.h>
#include <pcl/gpu/people/label_common.h>
#include <pcl/gpu/utils/device/limits.hpp>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/utils/texture_binder.hpp>
#include <stdio.h>
#include <limits>
#include <assert.h>
#include "internal.h"

using pcl::gpu::people::trees::Node;
using pcl::gpu::people::trees::Label;
using pcl::gpu::people::trees::AttribLocation;
using pcl::gpu::people::trees::Attrib;
using pcl::gpu::people::trees::focal;
using pcl::gpu::people::trees::NUM_LABELS;

using namespace std;
typedef unsigned int uint;

namespace pcl
{
  namespace device
  {
    /**
     * \brief This combines two probabilities into a single according to their weight = p(d|{I,x})
     **/
    __global__ void
    KernelCUDA_CombineProb (PtrStepSz<prob_histogram> probIn1,
        float weight1,
        PtrStepSz<prob_histogram> probIn2,
        float weight2,
        PtrStepSz<prob_histogram> probOut)
    {
      // map block and thread onto image coordinates
      int u = blockIdx.x * blockDim.x + threadIdx.x;
      int v = blockIdx.y * blockDim.y + threadIdx.y;

      if( u >= probIn1.cols || v >= probIn1.rows )
          return;

      for(int l = 0; l < NUM_LABELS; ++l)
      {
        // TODO: replace this with a histogram copy
        probOut.ptr(v)[u].probs[l] = weight1 * probIn1.ptr(v)[u].probs[l] + weight2 * probIn2.ptr(v)[u].probs[l];
      }
    }

    /**
     * \brief This sums a probabilities into a another according to its weight = p(d|{I,x})
     **/
    __global__ void
    KernelCUDA_WeightedSumProb (PtrStepSz<prob_histogram> probIn,
                                float weight,
                                PtrStepSz<prob_histogram> probOut)
    {
      // map block and thread onto image coordinates
      int u = blockIdx.x * blockDim.x + threadIdx.x;
      int v = blockIdx.y * blockDim.y + threadIdx.y;

      if( u >= probIn.cols || v >= probIn.rows )
        return;

      for(int l = 0; l < NUM_LABELS; ++l)
      {
        // TODO: replace this with a histogram copy
        probOut.ptr(v)[u].probs[l] += weight * probIn.ptr(v)[u].probs[l];
      }
    }

    /** \brief This merges the histogram of probabilities into a final label **/
    __global__ void
    KernelCUDA_SelectLabel (PtrStepSz<Label> labels, PtrStepSz<prob_histogram> Prob)
    {
      // map block and thread onto image coordinates
      int u = blockIdx.x * blockDim.x + threadIdx.x;
      int v = blockIdx.y * blockDim.y + threadIdx.y;

      if( u >= labels.cols || v >= labels.rows )
          return;

      float maxValue = 0;
      int maxID = 31;             // 31 equals to NOLABEL in label_common.h for some reason not resolved here
      prob_histogram p = Prob.ptr(v)[u];
      for(int l = 0; l < NUM_LABELS; ++l)
      {
        // TODO: problem is that this one doesn't handle ties very well
        if(maxValue < p.probs[l])
        {
          maxValue = p.probs[l];
          maxID = l;
        }
        if(maxValue == p.probs[l])
        {
          //DAMN WE HAVE A TIE
          //TODO: solve this
        }
      }
      labels.ptr(v)[u] = maxID;
    }

    /** \brief This will merge the votes from the different trees into one final vote, including probabilistic's **/
    void
    ProbabilityProc::CUDA_SelectLabel ( const Depth& depth,
                                   Labels& labels,
                                   LabelProbability& probabilities)
    {
      std::cout << "(I) : ProbabilityProc::CUDA_SelectLabel() called" << std::endl;
      //labels.create(depth.rows(), depth.cols());
      //probabilities.create(depth.rows(), depth.cols());

      dim3 block(32, 8);
      dim3 grid(divUp(depth.cols(), block.x), divUp(depth.rows(), block.y) );

      KernelCUDA_SelectLabel<<< grid, block >>>( labels, probabilities );

      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );
    }

    /** \brief This will combine two probabilities according their weight **/
    void
    ProbabilityProc::CUDA_CombineProb ( const Depth& depth,
                                   LabelProbability& probIn1,
                                   float weight1,
                                   LabelProbability& probIn2,
                                   float weight2,
                                   LabelProbability& probOut)
    {
      dim3 block(32, 8);
      dim3 grid(divUp(depth.cols(), block.x), divUp(depth.rows(), block.y) );

      // CUDA kernel call
      KernelCUDA_CombineProb<<< grid, block >>>( probIn1, weight1, probIn2, weight2, probOut );

      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );
    }

    /** \brief This will combine two probabilities according their weight **/
    void
    ProbabilityProc::CUDA_WeightedSumProb ( const Depth& depth,
                                   LabelProbability& probIn,
                                   float weight,
                                   LabelProbability& probOut)
    {
      dim3 block(32, 8);
      dim3 grid(divUp(depth.cols(), block.x), divUp(depth.rows(), block.y) );

      // CUDA kernel call
      KernelCUDA_WeightedSumProb<<< grid, block >>>( probIn, weight, probOut );

      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );
    }
  }
}


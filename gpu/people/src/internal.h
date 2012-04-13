/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * @authors: Cedric Cagniart, Koen Buys, Anatoly Baksheev
 */

#ifndef PCL_GPU_PEOPLE_INTERNAL_H_
#define PCL_GPU_PEOPLE_INTERNAL_H_

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/utils/safe_call.hpp>

namespace pcl
{
  namespace device
  {
    typedef DeviceArray2D<unsigned short> Depth;

    struct float8
    {
       float x, y, z, w, f1, f2, f3, f4;
    };

    void convertCloud2Depth(const DeviceArray<float8>& cloud, int rows, int cols, Depth& depth);

  }
}

void CUDA_runTree( const int    W,
                   const int    H,
                   const float  focal,
                   const int    treeHeight,
                   const int    numNodes,
                   const void*  nodes_device,
                   const void*  leaves_device,
                   const void*  depth_in_device,
                   void*        label_out_device );

void CUDA_runTree_masked( const int    W,
                          const int    H,
                          const float  focal,
                          const int    treeHeight,
                          const int    numNodes,
                          const void*  nodes_device,
                          const void*  leaves_device,
                          const void*  depth_in_device,
                          const void*  mask_in_device,
                          void*        label_out_device );

namespace pcl
{
    namespace device
    {
        void CUDA_runMultiTreePass( int          treeId,
                            const float  focal,
                            const int    treeHeight,
                            const int    numNodes,
                            const void*  nodes_device,
                            const void*  leaves_device,
                            const Depth& depth,
                            void*        multilabel_device );

        void CUDA_runMultiTreePassFG( int          treeId,
                            const int    FGThresh,
                            const float  focal,
                            const int    treeHeight,
                            const int    numNodes,
                            const void*  nodes_device,
                            const void*  leaves_device,
                            const        Depth& depth,
                            void*        multilabel_device );

        void CUDA_runMultiTreeMerge( int          numTrees,
                             const Depth& depth,
                             void*        multilabel_device, 
                             DeviceArray2D<unsigned char>& label_out_device);

    }

}

#endif /* PCL_GPU_PEOPLE_INTERNAL_H_ */

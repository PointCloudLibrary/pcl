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
 */

#ifndef PCL_GPU_PEOPLE_CUDATREE_H_
#define PCL_GPU_PEOPLE_CUDATREE_H_

#include <pcl/gpu/people/tree.h>
#include <pcl/gpu/containers/device_array.h>
#include <vector>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        struct CUDATree
        {                                                       
            int treeHeight;
            int numNodes;

            DeviceArray<Node> nodes_device;
            DeviceArray<Label> leaves_device;                      

            CUDATree (int treeHeight_, const std::vector<Node>& nodes, const std::vector<Label>& leaves)                
            {
              treeHeight = treeHeight_;
              numNodes = (1 << treeHeight) - 1;
              assert( nodes.size()  == (size_t)numNodes );
              assert( leaves.size() == (size_t)(1 << treeHeight) );
              
              nodes_device.upload(nodes);
              leaves_device.upload(leaves);          
            }
            ~CUDATree() {}
        };
      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
#endif //PCL_PEOPLE_TREES_CUDATREE_H_

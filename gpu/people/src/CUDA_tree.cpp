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
 * @author: Cedric Cagniart, Koen Buys
 * 
 */

#include <pcl/gpu/people/CUDA_tree.h>
// CUDA
#include <cuda_runtime_api.h>
// general
#include <cmath>
#include <cassert>
#include <iostream>

#include <pcl/gpu/people/handle_error.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        using pcl::gpu::people::trees::Node;
        using pcl::gpu::people::trees::Label;

        CUDATree::CUDATree(int                             treeHeight, 
                           const std::vector<pcl::gpu::people::trees::Node>&  nodes,
                           const std::vector<pcl::gpu::people::trees::Label>& leaves ):
        m_treeHeight(treeHeight)
        {
          m_numNodes = (int)pow(2.0, treeHeight) - 1;
          assert( int(nodes.size()) == m_numNodes );
          assert( int(leaves.size()) == pow(2.0,treeHeight) );

          // allocate device memory
          HANDLE_ERROR( cudaMalloc(&m_nodes_device, sizeof(Node)*nodes.size() ));
          HANDLE_ERROR( cudaMalloc(&m_leaves_device, sizeof(Label)*leaves.size()));

          // copy to device memory
          HANDLE_ERROR( cudaMemcpy( m_nodes_device, &nodes[0], sizeof(Node)*nodes.size(), cudaMemcpyHostToDevice));
          HANDLE_ERROR( cudaMemcpy( m_leaves_device, &leaves[0], sizeof(Label)*leaves.size(), cudaMemcpyHostToDevice));
        }

        CUDATree::~CUDATree() {
          cudaFree(m_nodes_device);
          cudaFree(m_leaves_device);
        }
      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl

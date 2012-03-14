/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart 
 * ************************************************* */

#include "CUDA_tree.h"
// CUDA
#include <cuda_runtime_api.h>
// general
#include <cmath>
#include <cassert>
#include <iostream>

#include <pcl/people/trees/HandleError.h>

namespace pcl
{
  namespace people
  {
    namespace trees
    {
      using pcl::people::trees::Node;
      using pcl::people::trees::Label;

      CUDATree::CUDATree(int                             treeHeight, 
                         const std::vector<Tree::Node>&  nodes,
                         const std::vector<Tree::Label>& leaves ):
      m_treeHeight(treeHeight)
      {
        m_numNodes = pow(2, treeHeight) - 1;
        assert( int(nodes.size()) == m_numNodes );
        assert( int(leaves.size()) == pow(2,treeHeight) );

        // allocate device memory
        HANDLE_ERROR( cudaMalloc(&m_nodes_device, sizeof(Node)*nodes.size() ));
        HANDLE_ERROR( cudaMalloc(&m_leaves_device, sizeof(Label)*leaves.size()));

        // copy to device memory
        HANDLE_ERROR( cudaMemcpy( m_nodes_device, nodes.data(), sizeof(Node)*nodes.size(), cudaMemcpyHostToDevice));
        HANDLE_ERROR( cudaMemcpy( m_leaves_device, leaves.data(), sizeof(Label)*leaves.size(), cudaMemcpyHostToDevice));
      }

      CUDATree::~CUDATree() {
        cudaFree(m_nodes_device);
        cudaFree(m_leaves_device);
      }
    } // end namespace trees
  } // end namespace people
} // end namespace pcl

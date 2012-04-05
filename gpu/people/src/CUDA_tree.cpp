/* *************************************************
 *
 * @copyright (2011) Willow Garage
 *
 * @author : Cedric Cagniart, Koen Buys
 * ************************************************* */

#include <pcl/gpu/people/trees/CUDA_tree.h>
// CUDA
#include <cuda_runtime_api.h>
// general
#include <cmath>
#include <cassert>
#include <iostream>

#include <pcl/gpu/people/trees/handle_error.h>

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

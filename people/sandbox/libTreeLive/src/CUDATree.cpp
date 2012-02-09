/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart 
 * ************************************************* */

#include "CUDATree.h"
// CUDA
#include <cuda_runtime_api.h>
// general
#include <cmath>
#include <cassert>
#include <iostream>

#include "commonTrees/HandleError.h"

namespace TreeLive
{

using Tree::Node;
using Tree::Label;

CUDATree::CUDATree(int                             treeHeight, 
                   const std::vector<Tree::Node>&  nodes,
                   const std::vector<Tree::Label>& leaves ):
m_treeHeight(treeHeight)
{
#ifdef VERBOSE
        std::cout<<"(I) : CUDATree(): before asserts" << std::endl;
#endif
	m_numNodes = pow(2, treeHeight) - 1;
	assert( int(nodes.size()) == m_numNodes );
	assert( int(leaves.size()) == pow(2,treeHeight) );
#ifdef VERBOSE
        std::cout<<"(I) : CUDATree(): going to cudaMalloc now" << std::endl;
#endif
	// allocate device memory
	HANDLE_ERROR( cudaMalloc(&m_nodes_device, sizeof(Node)*nodes.size() ));
	HANDLE_ERROR( cudaMalloc(&m_leaves_device, sizeof(Label)*leaves.size()));
#ifdef VERBOSE
        std::cout<<"(I) : CUDATree(): going to cudaMemcpy now" << std::endl;
#endif
	// copy to device memory
	HANDLE_ERROR( cudaMemcpy( m_nodes_device, nodes.data(), sizeof(Node)*nodes.size(), cudaMemcpyHostToDevice));
	HANDLE_ERROR( cudaMemcpy( m_leaves_device, leaves.data(), sizeof(Label)*leaves.size(), cudaMemcpyHostToDevice));
#ifdef VERBOSE
        std::cout<<"(I) : CUDATree(): cudaMemcpy finished, constructor finished" << std::endl;
#endif
}

CUDATree::~CUDATree() {
	cudaFree(m_nodes_device);
	cudaFree(m_leaves_device);
}

} // end namespace Tree

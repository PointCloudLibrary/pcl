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
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/utils/timers_cuda.hpp>
#include <stdio.h>
#include <assert.h>
#include "internal.h"



using pcl::gpu::people::trees::Node;
using pcl::gpu::people::trees::Label;
using pcl::gpu::people::trees::AttribLocation;
using pcl::gpu::people::trees::NUMLABELS;
using pcl::gpu::people::trees::Attrib;
using pcl::gpu::people::trees::focal;

using namespace std;
typedef unsigned int uint;

namespace pcl
{
    namespace device
    {        

      texture<unsigned short, 2, cudaReadModeElementType> depthTex;
      texture<unsigned short, 2, cudaReadModeElementType> maskTex;
      texture<char4, 2, cudaReadModeElementType> multilabelTex;

    __global__ void KernelCUDA_runTree( const int    W,
                                        const int    H,
                                        const float  f,
                                        const int    treeHeight,
                                        const int    numNodes,
                                        const Node*  nodes,
                                        const Label* leaves,
                                        PtrStep<Label> labels)
    {
      uint u = blockIdx.x * blockDim.x + threadIdx.x;
      uint v = blockIdx.y * blockDim.y + threadIdx.y;

      if( u >=W ) return;
      if( v >=H ) return;

      // init
      int    depth = tex2D(depthTex, u,v );
      float  scale = f/float(depth);

      // go down the tree
      int nid = 0;
      for(int nodeDepth=0;nodeDepth<treeHeight;++nodeDepth)
      {
        const Node& node = nodes[nid];
        const AttribLocation& loc = node.loc;
        int d1 = tex2D(depthTex, u+float(loc.du1)*scale, v+float(loc.dv1)*scale);
        int d2 = tex2D(depthTex, u+float(loc.du2)*scale, v+float(loc.dv2)*scale);
        int delta = d1-d2;
        bool test = delta > int(node.thresh);
        if( test ) nid = nid*2+2;
        else       nid = nid*2+1;
      }

      // we try to synchronize the write
      __syncthreads();
      labels.ptr(v)[u] = leaves[nid-numNodes];
    }

    void CUDA_runTree( const float  focal,
                       const int    treeHeight,
                       const int    numNodes,
                       const void*  nodes_device,
                       const void*  leaves_device,
                       const Depth& depth,
                       Labels& labels )
    {
      labels.create( depth.rows(), depth.cols() );

      using pcl::gpu::divUp; 
      pcl::gpu::ScopeTimer scope(__FUNCTION__);  

      int W = depth.cols();
      int H = depth.rows();
      
      cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();

      depthTex.addressMode[0] = cudaAddressModeClamp;
      cudaSafeCall( cudaBindTexture2D(0, depthTex, depth.ptr(), channeldesc, W, H, depth.step()) );
        

      dim3 block(16, 16);
      dim3 grid( divUp(W, block.x), divUp(H, block.y) );
      
      KernelCUDA_runTree<<< grid, block >>>( W, H, focal, treeHeight, numNodes, 
                                                       (const Node*)  nodes_device, 
                                                       (const Label*) leaves_device, 
                                                       labels);

      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );
      cudaSafeCall( cudaUnbindTexture(depthTex) );        
    }



    void CUDA_runTree_masked( const float  focal,
                              const int    treeHeight,
                              const int    numNodes,
                              const void*  nodes_device,
                              const void*  leaves_device,
                              const Depth& depth,
                              const void*  mask_in_device,
                              Labels& labels )

    {
      labels.create( depth.rows(), depth.cols() );
      int W = depth.cols();
      int H = depth.rows();

      using pcl::gpu::divUp; 
      pcl::gpu::ScopeTimer scope(__FUNCTION__);  

      depthTex.addressMode[0] = cudaAddressModeClamp;
      cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();  
      cudaSafeCall( cudaBindTexture2D(0, depthTex, depth.ptr(), channeldesc, W, H, depth.step()) );
      cudaSafeCall( cudaBindTexture2D(0, maskTex, mask_in_device, channeldesc, W, H, W*sizeof(unsigned short)) );

      dim3 block(16, 16);
      dim3 grid( divUp(W, block.x), divUp(H, block.y) );
      
#if 0
      KernelCUDA_runTree_masked<<< grid, block >>>( W, H, focal, treeHeight, numNodes, 
                                                       (const Node*)  nodes_device, 
                                                       (const Label*) leaves_device, 
                                                       (Label*)       label_out_device);

#endif

      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );
      
      cudaSafeCall( cudaUnbindTexture(depthTex) );
      cudaSafeCall( cudaUnbindTexture(maskTex) );      
    }
  }
}





///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


namespace pcl
{
    namespace device
    {



__global__ void KernelCUDA_MultiTreePass( const int    treeId,
                                          const int    W,
                                          const int    H,
                                          const float  f,
                                          const int    treeHeight,
                                          const int    numNodes,
                                          const Node*  nodes,
                                          const Label* leaves,
                                          Label*       multiLabels)
{
  uint u = blockIdx.x * blockDim.x + threadIdx.x;
  uint v = blockIdx.y * blockDim.y + threadIdx.y;

  if( u >=W ) return;
  if( v >=H ) return;

  // init
  int    depth = tex2D(depthTex, u,v );
  float  scale = f/float(depth);

  // go down the tree
  int nid = 0;
  for(int nodeDepth=0;nodeDepth<treeHeight;++nodeDepth)
  {
    const Node& node = nodes[nid];
    const AttribLocation& loc = node.loc;
    int d1 = tex2D(depthTex, u+float(loc.du1)*scale, v+float(loc.dv1)*scale);
    int d2 = tex2D(depthTex, u+float(loc.du2)*scale, v+float(loc.dv2)*scale);
    int delta = d1-d2;
    bool test = delta > int(node.thresh);
    if( test ) nid = nid*2+2;
    else       nid = nid*2+1;
  }

  // we try to synchronize the write
  __syncthreads();
  multiLabels[(v*W+u)*4+treeId] = leaves[nid-numNodes];
}

__global__ void KernelCUDA_MultiTreePassFG( const int    treeId,
                                          const int    FGThresh,
                                          const int    W,
                                          const int    H,
                                          const float  f,
                                          const int    treeHeight,
                                          const int    numNodes,
                                          const Node*  nodes,
                                          const Label* leaves,
                                          Label*       multiLabels)
{
  uint u = blockIdx.x * blockDim.x + threadIdx.x;
  uint v = blockIdx.y * blockDim.y + threadIdx.y;

  if( u >=W ) return;
  if( v >=H ) return;

  // init
  int    depth = tex2D(depthTex, u,v );
  float  scale = f/float(depth);

  // go down the tree
  int nid = 0;
  for(int nodeDepth=0;nodeDepth<treeHeight;++nodeDepth)
  {
    const Node& node = nodes[nid];
    const AttribLocation& loc = node.loc;
    int d1 = tex2D(depthTex, u+float(loc.du1)*scale, v+float(loc.dv1)*scale);
    int d2 = tex2D(depthTex, u+float(loc.du2)*scale, v+float(loc.dv2)*scale);
    if( d1 - depth > FGThresh ) d1 = 32767; 
    if( d2 - depth > FGThresh ) d2 = 32767;
    int delta = d1-d2;
    bool test = delta > int(node.thresh);
    if( test ) nid = nid*2+2;
    else       nid = nid*2+1;
  }

  // we try to synchronize the write
  __syncthreads();
  multiLabels[(v*W+u)*4+treeId] = leaves[nid-numNodes];
}

__device__ int findMaxId( int numBins, char* bins ) 
{
  // HACK .. not testing against numBins = 0
  int maxId   = 0;
  char maxVal = bins[0];
  for(int i=1;i<numBins;++i) 
  {
    char val = bins[i];
    if( val > maxVal ) { maxId = i; maxVal = val; }
  }
  return maxId;
}

//this will find the max Index but return -1 if there is a tie
__device__ int findMaxId_testTie(int numBins, char* bins) 
{
  int maxId = 0;
  int maxId_other = -1;
  char maxVal = bins[0];

  for(int i=1;i<numBins;++i) {
    char val = bins[i];
    if( val == maxVal ) { maxId_other = i; }
    if( val > maxVal ) { maxId = i; maxId_other = -1; maxVal = val; }
  }

  if( maxId_other != -1) return -1;
  else                   return maxId;
}

__global__ void KernelCUDA_MultiTreeMerge( const int    numTrees,
                                           const int    W,
                                           const int    H,
                                           PtrStep<Label> labels )
{
  uint u = blockIdx.x * blockDim.x + threadIdx.x;
  uint v = blockIdx.y * blockDim.y + threadIdx.y;

  if( u >=W ) return;
  if( v >=H ) return;

  // reset the bins
  char bins[NUMLABELS];
  for(int li=0;li<NUMLABELS;++li) { bins[li] = 0; }

  // find a consensus with the current trees
  {
    char4 pixlabels = tex2D(multilabelTex, u ,v);
    char* bob = (char*)&pixlabels; //horrible but char4's have xyzw members
    for(int ti=0;ti<numTrees;++ti) {
      bins[ bob[ti] ]++;
    }
  }

  int res = findMaxId_testTie(NUMLABELS, bins);

  // if this fails... find a consensus in a 1 neighbourhood
  if( res < 0 ) {
    int depth = tex2D(depthTex, u,v);
    for(int i=-1;i<=1;++i) 
    {
      for(int j=-1;j<=1;++j) 
      {
        int   depth_neighbor  = tex2D(depthTex,u+i,v+j);
        char4 labels_neighbor = tex2D(multilabelTex, u+i,v+j); 
        char* bob = (char*)&labels_neighbor; //horrible but char4's have xyzw members
        int weight = abs(depth-depth_neighbor) < 50 ? 1:0; // 5cms
        for(int ti=0;ti<numTrees;++ti) 
          bins[ bob[ti] ] += weight;        
      }
    }
    res = findMaxId( NUMLABELS, bins );
  }
  __syncthreads();
  labels.ptr(v)[u] = res;
}



void CUDA_runMultiTreePassFG( int          treeId,
                            const int    FGThresh,                            
                            const float  focal,
                            const int    treeHeight,
                            const int    numNodes,
                            const void*  nodes_device,
                            const void*  leaves_device,
                            const Depth& depth,
                            void*        multilabel_device )
{
  assert( treeId >= 0 );
  assert( treeId < 4 );

  int W = depth.cols();
  int H = depth.rows();

  depthTex.addressMode[0] = cudaAddressModeClamp;
  cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();  
  cudaSafeCall( cudaBindTexture2D(0, depthTex, depth.ptr(), channeldesc, W, H, depth.step()) );

  dim3 block(16, 16);
  dim3 grid( divUp(W, block.x), divUp(H, block.y) );
  
  KernelCUDA_MultiTreePassFG<<< grid, block >>>( treeId, FGThresh, W, H, focal, treeHeight, numNodes, 
                                                 (const Node*)  nodes_device, 
                                                 (const Label*) leaves_device, 
                                                 (Label*)       multilabel_device);

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaThreadSynchronize() );
  cudaSafeCall( cudaUnbindTexture(depthTex) );
}

void CUDA_runMultiTreePass( int          treeId,                            
                            const float  focal,
                            const int    treeHeight,
                            const int    numNodes,
                            const void*  nodes_device,
                            const void*  leaves_device,
                            const Depth& depth,
                            void*        multilabel_device )
{
  assert( treeId >= 0 );
  assert( treeId < 4 );

  int W = depth.cols();
  int H = depth.rows();

  depthTex.addressMode[0] = cudaAddressModeClamp;
  cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();  
  cudaSafeCall( cudaBindTexture2D(0, depthTex, depth.ptr(), channeldesc, W, H, depth.step()) );
  
  dim3 block(16, 16);
  dim3 grid( divUp(W, block.x), divUp(H, block.y) );

  KernelCUDA_MultiTreePass<<< grid, block >>>( treeId, W, H, focal, treeHeight, numNodes, 
                                                 (const Node*)  nodes_device, 
                                                 (const Label*) leaves_device, 
                                                 (Label*)       multilabel_device);

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaThreadSynchronize() );
  cudaSafeCall( cudaUnbindTexture(depthTex) );
}

void CUDA_runMultiTreeMerge( int          numTrees,                             
                             const Depth& depth,
                             void*        multilabel_device, 
                             Labels& labels)
{

  labels.create(depth.rows(), depth.cols());

  assert( numTrees <= 4 );

  int W = depth.cols();
  int H = depth.rows();


  {
    depthTex.addressMode[0] = cudaAddressModeClamp;
    cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();    
    cudaSafeCall( cudaBindTexture2D(0, depthTex, depth.ptr(), channeldesc, W, H, depth.step()) );
  }
  {
    multilabelTex.addressMode[0] = cudaAddressModeClamp;
    cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<char4>();    
    cudaSafeCall( cudaBindTexture2D(0, multilabelTex, multilabel_device, channeldesc, W, H, W*sizeof(char4)) );
  }

  dim3 block(16, 16);
  dim3 grid( divUp(W, block.x), divUp(H, block.y) );
  
  KernelCUDA_MultiTreeMerge<<< grid, block >>>( numTrees, W, H, labels );

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaThreadSynchronize() );
  cudaSafeCall( cudaUnbindTexture(depthTex) );
  cudaSafeCall( cudaUnbindTexture(multilabelTex) );
}


}

}



//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

pcl::device::CUDATree::CUDATree(int treeHeight_arg, const vector<Node>& nodes, const vector<Label>& leaves)                
{
  treeHeight = treeHeight_arg;
  numNodes = (1 << treeHeight) - 1;
  assert( nodes.size()  == (size_t)numNodes );
  assert( leaves.size() == (size_t)(1 << treeHeight) );
  
  nodes_device.upload(nodes);
  leaves_device.upload(leaves);          
}



void pcl::device::MultiTreeLiveProc::process(const Depth& dmap, Labels& lmap)
{
  assert(!trees.empty());

  if (trees.size() == 1)
  {    
    const CUDATree& t = trees[0];    
    device::CUDA_runTree( focal, t.treeHeight, t.numNodes, t.nodes_device, t.leaves_device, dmap,  lmap );   
    return;
  }

  process(dmap, lmap, std::numeric_limits<Attrib>::max());    
}

void pcl::device::MultiTreeLiveProc::process(const Depth& dmap, Labels& lmap, int FGThresh)
{          
  assert(!trees.empty());

  int numTrees = (int)trees.size();

  multilmap_device.create(dmap.cols() * dmap.rows() * numTrees);                            

  // 1 - run the multi passes  
  for( int ti = 0; ti < numTrees; ++ti ) 
  {
    const CUDATree& t = trees[ti];

    if( FGThresh == std::numeric_limits<Attrib>::max() ) 
    {
      device::CUDA_runMultiTreePass( ti, (float)focal, t.treeHeight, t.numNodes, 
          t.nodes_device, t.leaves_device, dmap, multilmap_device );
    }
    else 
    {
        device::CUDA_runMultiTreePassFG( ti, FGThresh, (float)focal, t.treeHeight, t.numNodes, 
          t.nodes_device, t.leaves_device, dmap, multilmap_device );
    }
  }
  // 2 - run the merging 
  device::CUDA_runMultiTreeMerge(numTrees, dmap, multilmap_device, lmap);          
}
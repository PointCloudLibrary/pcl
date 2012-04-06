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
 * @authors: Cedric Cagniart, Koen Buys
 *
 */

#include <pcl/gpu/people/trees/tree.h>
#include "CUDA_run_multi_tree.h"
#include <cuda.h>
#include <assert.h>

using pcl::gpu::people::trees::Node;
using pcl::gpu::people::trees::Label;
using pcl::gpu::people::trees::AttribLocation;
using pcl::gpu::people::trees::NUMLABELS;

typedef unsigned int uint;

texture<unsigned short, 2, cudaReadModeElementType> depthTex;
texture<char4, 2, cudaReadModeElementType>          multilabelTex;

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
  uint u = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  uint v = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;

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
  uint u = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  uint v = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;

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

__device__ int findMaxId( int numBins, char* bins ) {
  // HACK .. not testing against numBins = 0
  int maxId   = 0;
  char maxVal = bins[0];
  for(int i=1;i<numBins;++i) {
    char val = bins[i];
    if( val > maxVal ) { maxId = i; maxVal = val; }
  }
  return maxId;
}

//this will find the max Index but return -1 if there is a tie
__device__ int findMaxId_testTie(int numBins, char* bins) {
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
                                           Label*       labels )
{
  uint u = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
  uint v = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;

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
    for(int i=-1;i<=1;++i) {
      for(int j=-1;j<=1;++j) {
        int   depth_neighbor  = tex2D(depthTex,u+i,v+j);
        char4 labels_neighbor = tex2D(multilabelTex, u+i,v+j); 
        char* bob = (char*)&labels_neighbor; //horrible but char4's have xyzw members
        int weight = abs(depth-depth_neighbor) < 50 ? 1:0; // 5cms
        for(int ti=0;ti<numTrees;++ti) {
          bins[ bob[ti] ] += weight;
        }
      }
    }
    res = findMaxId( NUMLABELS, bins );
  }
  __syncthreads();
  labels[v*W+u] = res;
}

void CUDA_runMultiTreePassFG( int          treeId,
                            const int    FGThresh,
                            const int    W,
                            const int    H,
                            const float  focal,
                            const int    treeHeight,
                            const int    numNodes,
                            const void*  nodes_device,
                            const void*  leaves_device,
                            const void*  depth_in_device,
                            void*        multilabel_device )
{
  assert( treeId >= 0 );
  assert( treeId < 4 );

  cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();
  depthTex.addressMode[0] = cudaAddressModeClamp;
  cudaBindTexture2D(0, depthTex, depth_in_device, channeldesc,
                                              W, H, W*sizeof(unsigned short));

  dim3 gridSize((W+16-1)/16, (H+16-1)/16);
  dim3 blockSize(16,16);

  KernelCUDA_MultiTreePassFG<<< gridSize, blockSize >>>( treeId, FGThresh, W, H, focal, treeHeight, numNodes, 
                                                        (const Node*)  nodes_device, 
                                                        (const Label*) leaves_device, 
                                                        (Label*)       multilabel_device);

  cudaUnbindTexture(depthTex);
}

void CUDA_runMultiTreePass( int          treeId,
                            const int    W,
                            const int    H,
                            const float  focal,
                            const int    treeHeight,
                            const int    numNodes,
                            const void*  nodes_device,
                            const void*  leaves_device,
                            const void*  depth_in_device,
                            void*        multilabel_device )
{
  assert( treeId >= 0 );
  assert( treeId < 4 );

  cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();
  depthTex.addressMode[0] = cudaAddressModeClamp;
  cudaBindTexture2D(0, depthTex, depth_in_device, channeldesc,
                                              W, H, W*sizeof(unsigned short));

  dim3 gridSize((W+16-1)/16, (H+16-1)/16);
  dim3 blockSize(16,16);

  KernelCUDA_MultiTreePass<<< gridSize, blockSize >>>( treeId, W, H, focal, treeHeight, numNodes, 
                                                        (const Node*)  nodes_device, 
                                                        (const Label*) leaves_device, 
                                                        (Label*)       multilabel_device);

  cudaUnbindTexture(depthTex);
}

void CUDA_runMultiTreeMerge( int          numTrees,
                             const int    W,
                             const int    H,
                             const void*  depth_in_device, 
                             void*        multilabel_device, 
                             void*        label_out_device)
{
  assert( numTrees <= 4 );

  {
    cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();
    depthTex.addressMode[0] = cudaAddressModeClamp;
    cudaBindTexture2D(0, depthTex, depth_in_device, channeldesc,
                                              W, H, W*sizeof(unsigned short));
  }
  {
    cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<char4>();
    multilabelTex.addressMode[0] = cudaAddressModeClamp;
    cudaBindTexture2D(0, multilabelTex, multilabel_device, channeldesc,
                                                       W, H, W*sizeof(char4));
  }

  dim3 gridSize((W+16-1)/16, (H+16-1)/16);
  dim3 blockSize(16,16);

  KernelCUDA_MultiTreeMerge<<< gridSize, blockSize >>>( numTrees, W, H,  
                                                         (Label*) label_out_device );

  cudaUnbindTexture(depthTex);
  cudaUnbindTexture(multilabelTex);
}

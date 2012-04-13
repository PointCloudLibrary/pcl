/** 
 * @authors: Cedric Cagniart, Koen Buys, Anatoly Baksheev
 */

//#include <cutil.h>
#include <pcl/gpu/people/tree.h>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/utils/timers_cuda.hpp>
#include <stdio.h>

#include "internal.h"

texture<unsigned short, 2, cudaReadModeElementType> depthTex;
texture<unsigned short, 2, cudaReadModeElementType> maskTex;

using pcl::gpu::people::trees::Node;
using pcl::gpu::people::trees::Label;
using pcl::gpu::people::trees::AttribLocation;

typedef unsigned int uint;

__global__ void KernelCUDA_runTree( const int    W,
                                    const int    H,
                                    const float  f,
                                    const int    treeHeight,
                                    const int    numNodes,
                                    const Node*  nodes,
                                    const Label* leaves,
                                    pcl::device::PtrStep<Label> labels)
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

void pcl::device::CUDA_runTree( const float  focal,
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



void pcl::device::CUDA_runTree_masked( const float  focal,
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


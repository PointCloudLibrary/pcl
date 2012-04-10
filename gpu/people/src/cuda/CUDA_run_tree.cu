/** 
 * @authors: Cedric Cagniart, Koen Buys
 */

//#include <cutil.h>
#include <pcl/gpu/people/tree.h>
#include <stdio.h>
#include <cuda.h>

texture<unsigned short, 2, cudaReadModeElementType> depthTex;
texture<unsigned short, 2, cudaReadModeElementType> maskTex;

using pcl::gpu::people::trees::Node;
using pcl::gpu::people::trees::Label;
using pcl::gpu::people::trees::AttribLocation;

typedef unsigned int uint;

#define CUDAPERROR \
{ \
	cudaError_t status = cudaGetLastError(); \
	if( status != cudaSuccess ) printf("(E) CUDA error : %s\n %s\n", __FUNCTION__, cudaGetErrorString(status) ); \
}

__global__ void KernelCUDA_runTree( const int    W,
                                    const int    H,
                                    const float  f,
                                    const int    treeHeight,
                                    const int    numNodes,
                                    const Node*  nodes,
                                    const Label* leaves,
                                    Label*       labels)
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
  labels[v*W+u] = leaves[nid-numNodes];
}

void CUDA_runTree( const int    W,
                   const int    H,
                   const float  focal,
                   const int    treeHeight,
                   const int    numNodes,
                   const void*  nodes_device,
                   const void*  leaves_device,
                   const void*  depth_in_device,
                   void*        label_out_device )
{
  cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();

  depthTex.addressMode[0] = cudaAddressModeClamp;
  cudaBindTexture2D(0, depthTex, depth_in_device, channeldesc, W, H, W*sizeof(unsigned short));

  CUDAPERROR

  dim3 gridSize((W+16-1)/16, (H+16-1)/16);
  dim3 blockSize(16,16);

  cudaEvent_t startEvt, stopEvt;
  cudaEventCreate(&startEvt);
  cudaEventCreate(&stopEvt);
  cudaEventRecord(startEvt, 0);

  KernelCUDA_runTree<<< gridSize, blockSize >>>( W, H, focal, treeHeight, numNodes, 
                                                   (const Node*)  nodes_device, 
                                                   (const Label*) leaves_device, 
                                                   (Label*)       label_out_device);

  cudaUnbindTexture(depthTex);
  cudaEventRecord(stopEvt,0);
  cudaThreadSynchronize();
  float ms = 0.;
  cudaEventElapsedTime(&ms, startEvt, stopEvt);
  cudaEventDestroy(startEvt);
  cudaEventDestroy(stopEvt);
#define CUDAKERNEL_VERBOSE
#ifdef CUDAKERNEL_VERBOSE
  printf("CUDA -- spent %f ms in the kernel %s \n", ms, __FUNCTION__ );
#endif

  CUDAPERROR
}


/*
void CUDA_runTree_masked( const int    W,
                          const int    H,
                          const float  focal,
                          const int    treeHeight,
                          const int    numNodes,
                          const void*  nodes_device,
                          const void*  leaves_device,
                          const void*  depth_in_device,
                          const void*  mask_in_device,
                          void*        label_out_device )
{
  cudaChannelFormatDesc channeldesc = cudaCreateChannelDesc<unsigned short>();

  depthTex.addressMode[0] = cudaAddressModeClamp;
  cudaBindTexture2D(0, depthTex, depth_in_device, channeldesc, W, H, W*sizeof(unsigned short));
  cudaBindTexture2D(0, maskTex, mask_in_device, channeldesc, W, H, W*sizeof(unsigned short));

  CUDAPERROR

  dim3 gridSize((W+16-1)/16, (H+16-1)/16);
  dim3 blockSize(16,16);

  cudaEvent_t startEvt, stopEvt;
  cudaEventCreate(&startEvt);
  cudaEventCreate(&stopEvt);
  cudaEventRecord(startEvt, 0);

  KernelCUDA_runTree_masked<<< gridSize, blockSize >>>( W, H, focal, treeHeight, numNodes, 
                                                   (const Node*)  nodes_device, 
                                                   (const Label*) leaves_device, 
                                                   (Label*)       label_out_device);

  cudaUnbindTexture(depthTex);
  cudaUnbindTexture(maskTex);
  cudaEventRecord(stopEvt,0);
  cudaThreadSynchronize();
  float ms = 0.;
  cudaEventElapsedTime(&ms, startEvt, stopEvt);
  cudaEventDestroy(startEvt);
  cudaEventDestroy(stopEvt);
#define CUDAKERNEL_VERBOSE
#ifdef CUDAKERNEL_VERBOSE
  printf("CUDA -- spent %f ms in the kernel %s \n", ms, __FUNCTION__ );
#endif

  CUDAPERROR
}
*/

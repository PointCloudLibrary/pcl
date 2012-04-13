#include "internal.h"
#include <pcl/gpu/utils/safe_call.hpp>


namespace pcl
{
  namespace device
  {
    __global__ void c2dKernel(const PtrSz<float8> cloud, int cols, PtrStep<unsigned short> depth)
    {
      int idx = threadIdx.x + blockIdx.x * blockDim.x;

      if (idx < cloud.size)
      {
        float d = cloud.data[idx].z * 1000; // m -> mm
        d = isnan(d) ? 0 : d;

        int x = idx % cols;
        int y = idx / cols;
        depth.ptr(y)[x] = d;
      }
    }
  }
}

void 
pcl::device::convertCloud2Depth(const DeviceArray<float8>& cloud, int rows, int cols, Depth& depth)
{
  depth.create(rows, cols);

  int block = 256;
  int total = (int)cloud.size();

  c2dKernel<<<divUp(total, block), block>>>(cloud, cols, depth);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaThreadSynchronize() );
}

namespace pcl
{
  namespace device
  {    
    texture<uchar4, cudaTextureType1D, cudaReadModeElementType> cmapTex;

    __global__ void colorKernel(const PtrStepSz<unsigned char> labels, PtrStep<uchar4> rgba)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x < labels.cols && y < labels.rows)
      {
        int l = labels.ptr(y)[x];
        rgba.ptr(y)[x] = tex1Dfetch(cmapTex, l);         
      }
    }
  }
}

void pcl::device::colorLMap(const Labels& labels, const DeviceArray<uchar4>& map, Image& rgba)
{  
  cmapTex.addressMode[0] = cudaAddressModeClamp;
  cudaChannelFormatDesc desc = cudaCreateChannelDesc<uchar4>();  
  cudaSafeCall( cudaBindTexture(0, cmapTex, map.ptr(), desc, map.size() * sizeof(uchar4) ) );
    
  dim3 block(32, 8);
  dim3 grid( divUp(labels.cols(), block.x), divUp(labels.rows(), block.y) );
  
  colorKernel<<< grid, block >>>( labels, rgba );

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaThreadSynchronize() );
  cudaSafeCall( cudaUnbindTexture(cmapTex) );        
}
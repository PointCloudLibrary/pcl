#include "internal.h"
#include <pcl/gpu/utils/safe_call.hpp>

namespace pcl
{
  namespace device
  {
    __global__ void c2dKernel(const PtrSz<float8> cloud, unsigned short *depth)
    {
      int idx = threadIdx.x + blockIdx.x * blockDim.x;

      if (idx < cloud.size)                
        depth[idx] = cloud.data[idx].z * 1000; // m -> mm      
    }
  }
}

void 
pcl::device::convertCloud2Depth(const DeviceArray<float8>& cloud, DeviceArray<unsigned short>& depth)
{
  depth.create(cloud.size());

  int block = 256;
  int total = (int)cloud.size();

  c2dKernel<<<divUp(total, block), block>>>(cloud, depth);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaThreadSynchronize() );
}
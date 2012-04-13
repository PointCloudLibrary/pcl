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

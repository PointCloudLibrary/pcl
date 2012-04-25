#include "internal.h"
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/utils/device/limits.hpp>
#include "npp.h"

#include <stdio.h>

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


///////////////////////////////////////////////////////////////////////////////////////////////////////
/// TODO implement getError string for NPP and move this to the same place with cudaSafeCall

#if defined(__GNUC__)
  #define nppSafeCall(expr)  pcl::gpu::___nppSafeCall(expr, __FILE__, __LINE__, __func__)    
#else /* defined(__CUDACC__) || defined(__MSVC__) */
  #define nppSafeCall(expr)  pcl::gpu::___nppSafeCall(expr, __FILE__, __LINE__)    
#endif

namespace pcl
{
  namespace gpu
  {

    void ___nppSafeCall(int err_code, const char *file, const int line, const char *func = "")
    {
      if (err_code < 0)
      {
          char buf[4096];
          sprintf(buf, "NppErrorCode = %d", err_code);
          error(buf, file, line, func);
      }
    }
  }
}


void pcl::device::setZero(Mask& mask)
{
  NppiSize sz;
  sz.width  = mask.cols();
  sz.height = mask.rows();   
  nppSafeCall( nppiSet_8u_C1R( 0, mask, (int)mask.step(), sz) );
}

void pcl::device::Dilatation::prepareRect5x5Kernel(DeviceArray<unsigned char>& kernel)
{
  if (kernel.size() == KSIZE_X * KSIZE_Y)
    return;

  std::vector<unsigned char> host(KSIZE_X * KSIZE_Y, (unsigned char)1);
  kernel.upload(host);
}

void pcl::device::Dilatation::invoke(const Mask& src, const Kernel& kernel, Mask& dst)
{
  dst.create(src.rows(), src.cols());  

  NppiSize sz;
  sz.width  = src.cols() - KSIZE_X;
  sz.height = src.rows() - KSIZE_Y; 

  NppiSize ksz;
  ksz.width  = KSIZE_X;
  ksz.height = KSIZE_Y;

  NppiPoint anchor;
  anchor.x = ANCH_X;
  anchor.y = ANCH_Y;

  nppSafeCall( nppiDilate_8u_C1R(src.ptr(ANCH_Y) + ANCH_X, (int)src.step(), 
                                 dst.ptr(ANCH_Y) + ANCH_X, (int)dst.step(), sz, kernel, ksz, anchor) );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace device
  {
    __global__ void fgDepthKernel(const PtrStepSz<unsigned short> depth1, const PtrStep<unsigned char> inv_mask, PtrStep<unsigned short> depth2)
    {
      int x = blockIdx.x * blockDim.x + threadIdx.x;
      int y = blockIdx.y * blockDim.y + threadIdx.y;

      if (x < depth1.cols && y < depth1.rows)
      {
        unsigned short d = depth1.ptr(y)[x];
        depth2.ptr(y)[x] = inv_mask.ptr(y)[x] ? d : numeric_limits<short>::max();
      }
    }
  }
}

void pcl::device::prepareForeGroundDepth(const Depth& depth1, Mask& inverse_mask, Depth& depth2)
{
  int cols = depth1.cols();
  int rows = depth1.rows();

  depth2.create(rows, cols);

  dim3 block(32, 8);
  dim3 grid( divUp(cols, block.x), divUp(rows, block.y) );

  fgDepthKernel<<< grid, block >>>( depth1, inverse_mask, depth2 );

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaThreadSynchronize() );
}

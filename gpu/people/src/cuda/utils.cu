#include "internal.h"
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/utils/texture_binder.hpp>
#include <pcl/gpu/utils/device/limits.hpp>
#include "npp.h"

#include <stdio.h>

namespace pcl
{
  namespace device
  {
    texture<uchar4, cudaTextureType1D, cudaReadModeElementType> cmapTex;

    __global__ void colorKernel(const PtrStepSz<unsigned char> labels, PtrStep<uchar4> output)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x < labels.cols && y < labels.rows)
      {
        int l = labels.ptr(y)[x];
        output.ptr(y)[x] = tex1Dfetch(cmapTex, l);
      }
    }

    __global__ void mixedColorKernel(const PtrStepSz<unsigned char> labels, PtrStepSz<uchar4> rgba, PtrStep<uchar4> output)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;

      if (x < labels.cols && y < labels.rows)
      {        
        uchar4 c = rgba.ptr(y)[x];

        int l = labels.ptr(y)[x];
        if (l != 8) // RHip but should be background
          c = tex1Dfetch(cmapTex, l);

        output.ptr(y)[x] = c;
      }
    }
  }
}

void pcl::device::colorLMap(const Labels& labels, const DeviceArray<uchar4>& map, Image& rgba)
{
  cmapTex.addressMode[0] = cudaAddressModeClamp;
  TextureBinder binder(map, cmapTex);
  
  dim3 block(32, 8);
  dim3 grid( divUp(labels.cols(), block.x), divUp(labels.rows(), block.y) );

  colorKernel<<< grid, block >>>( labels, rgba );

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaThreadSynchronize() );  
}

void pcl::device::mixedColorMap(const Labels& labels, const DeviceArray<uchar4>& map, const Image& rgba, Image& output)
{
  cmapTex.addressMode[0] = cudaAddressModeClamp;
  TextureBinder binder(map, cmapTex);

  dim3 block(32, 8);
  dim3 grid(divUp(labels.cols(), block.x), divUp(labels.rows(), block.y));

  mixedColorKernel<<<grid, block>>>(labels, rgba, output);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
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

  std::vector<unsigned char> host(KSIZE_X * KSIZE_Y, (unsigned char)255);
  kernel.upload(host);
}

void pcl::device::Dilatation::invoke(const Mask& src, const Kernel& kernel, Mask& dst)
{
  dst.create(src.rows(), src.cols());  
  setZero(dst);

  NppiSize sz;
  sz.width  = src.cols() - KSIZE_X;
  sz.height = src.rows() - KSIZE_Y; 

  NppiSize ksz;
  ksz.width  = KSIZE_X;
  ksz.height = KSIZE_Y;

  NppiPoint anchor;
  anchor.x = ANCH_X;
  anchor.y = ANCH_Y;

  // This one uses Nvidia performance primitives
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
        depth2.ptr(y)[x] = inv_mask.ptr(y)[x] ? d : numeric_limits<unsigned short>::max();
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



///////////////////////////////////////////////////////////////////////////////////////////////////////
/// compute hue functionality 

namespace pcl
{
  namespace device
  {
    __device__ __host__ __forceinline__ float computeHueFunc (int rgba)
    {
      int r = (rgba      ) & 0xFF;
      int g = (rgba >>  8) & 0xFF;
      int b = (rgba >> 16) & 0xFF;

      int v = max (r, max (g, b));
      float h;

      float div_inv = 1.f / (v - min (r, min (g, b)) );

      if (v == 0)
          return -1;

      if (r == v)
          h = (    (g - b)) * div_inv;
      else if (g == v)
          h = (2 + (b - r)) * div_inv;
      else 
          h = (4 + (r - g)) * div_inv;

      h *= 60;    
      if (h < 0)
          h += 360;

      return h;
    }

  }
}

float pcl::device::computeHue(int rgba) 
{ 
  return computeHueFunc(rgba); 
}

namespace pcl
{
  namespace device
  {
    __global__ void computeHueKernel(const PtrStepSz<int> rgba, const PtrStep<unsigned short> depth, PtrStep<float> hue)
    {
      int x = blockIdx.x * blockDim.x + threadIdx.x;
      int y = blockIdx.y * blockDim.y + threadIdx.y;

      if (x < rgba.cols && y < rgba.rows)
      {
        const float qnan = numeric_limits<float>::quiet_NaN();

        unsigned short d = depth.ptr(y)[x];            
        hue.ptr(y)[x] = (d == 0) ? qnan : computeHueFunc(rgba.ptr(y)[x]);           
      }
    }
  }
}


void pcl::device::computeHueWithNans(const Image& rgba, const Depth& depth, HueImage& hue)
{
  hue.create(rgba.rows(), rgba.cols());

  dim3 block(32, 8);
  dim3 grid;

  grid.x = divUp(rgba.cols(), block.x);
  grid.y = divUp(rgba.rows(), block.y);

  computeHueKernel<<<grid, block>>>(rgba, depth, hue);

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}

namespace pcl
{
  namespace device
  {
    __global__ void reprojectDepthKenrel(const PtrStepSz<unsigned short> depth, const Intr intr, PtrStep<float4> cloud)
    {
      int x = blockIdx.x * blockDim.x + threadIdx.x;
      int y = blockIdx.y * blockDim.y + threadIdx.y;

      const float qnan = numeric_limits<float>::quiet_NaN();

      if (x < depth.cols && y < depth.rows)
      {
        float4 p = make_float4(qnan, qnan, qnan, qnan);

        int d = depth.ptr(y)[x];
        float z = d * 0.001f; // mm -> meters

        p.x = z * (x - intr.cx) / intr.fx;
        p.y = z * (y - intr.cy) / intr.fy;
        p.z = z;

        cloud.ptr(y)[x] = p;
      }      
    }
  }
}

void pcl::device::computeCloud(const Depth& depth, const Intr& intr, Cloud& cloud)
{
  cloud.create(depth.rows(), depth.cols());

  dim3 block(32, 8);
  dim3 grid;
  grid.x = divUp(depth.cols(), block.x);
  grid.y = divUp(depth.rows(), block.y);

  reprojectDepthKenrel<<<grid, block>>>(depth, intr, cloud);

  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}

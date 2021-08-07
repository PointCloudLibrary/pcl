/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/utils/safe_call.hpp>

#include "cuda.h"

#include <array> // replace c-style array with std::array
#include <cstdio>

#define HAVE_CUDA
//#include <pcl_config.h>

#if !defined(HAVE_CUDA)

void
throw_nogpu()
{
  throw "PCL 2.0 exception";
}

int
pcl::gpu::getCudaEnabledDeviceCount()
{
  return 0;
}

void
pcl::gpu::setDevice(int /*device*/)
{
  throw_nogpu();
}

std::string
pcl::gpu::getDeviceName(int /*device*/)
{
  throw_nogpu();
}

void
pcl::gpu::printCudaDeviceInfo(int /*device*/)
{
  throw_nogpu();
}

void
pcl::gpu::printShortCudaDeviceInfo(int /*device*/)
{
  throw_nogpu();
}

#else

int
pcl::gpu::getCudaEnabledDeviceCount()
{
  int count;
  cudaError_t error = cudaGetDeviceCount(&count);

  if (error == cudaErrorInsufficientDriver)
    return -1;

  if (error == cudaErrorNoDevice)
    return 0;

  cudaSafeCall(error);
  return count;
}

void
pcl::gpu::setDevice(int device)
{
  cudaSafeCall(cudaSetDevice(device));
}

std::string
pcl::gpu::getDeviceName(int device)
{
  cudaDeviceProp prop;
  cudaSafeCall(cudaGetDeviceProperties(&prop, device));

  return prop.name;
}

bool
pcl::gpu::checkIfPreFermiGPU(int device)
{
  if (device < 0)
    cudaSafeCall(cudaGetDevice(&device));

  cudaDeviceProp prop;
  cudaSafeCall(cudaGetDeviceProperties(&prop, device));
  return prop.major < 2; // CC == 1.x
}

namespace {
template <class T>
inline void
getCudaAttribute(T* attribute, CUdevice_attribute device_attribute, int device)
{
  *attribute = T();
  CUresult error =
      CUDA_SUCCESS; // = cuDeviceGetAttribute( attribute, device_attribute, device );
  if (CUDA_SUCCESS == error)
    return;

  printf("Driver API error = %04d\n", error);
  pcl::gpu::error("driver API error", __FILE__, __LINE__);
}

inline int
convertSMVer2Cores(int major, int minor)
{
  // Defines for GPU Architecture types (using the SM version to determine the # of
  // cores per SM
  struct SMtoCores {
    int SM; // 0xMm (hexadecimal notation), M = SM Major version, and m = SM minor
            // version
    int Cores;
  };

  std::array<SMtoCores, 15> gpuArchCoresPerSM = {{{0x30, 192},
                                                  {0x32, 192},
                                                  {0x35, 192},
                                                  {0x37, 192},
                                                  {0x50, 128},
                                                  {0x52, 128},
                                                  {0x53, 128},
                                                  {0x60, 64},
                                                  {0x61, 128},
                                                  {0x62, 128},
                                                  {0x70, 64},
                                                  {0x72, 64},
                                                  {0x75, 64},
                                                  {0x80, 64},
                                                  {0x86, 128}}};
  for (const auto& sm2cores : gpuArchCoresPerSM) {
    if (sm2cores.SM == ((major << 4) + minor))
      return sm2cores.Cores;
  }
  printf(
      "\nCan't determine number of cores. Unknown SM version %d.%d!\n", major, minor);
  return 0;
}
} // namespace

void
pcl::gpu::printCudaDeviceInfo(int device)
{
  int count = getCudaEnabledDeviceCount();
  bool valid = (device >= 0) && (device < count);

  int beg = valid ? device : 0;
  int end = valid ? device + 1 : count;

  printf(
      "*** CUDA Device Query (Runtime API) version (CUDART static linking) *** \n\n");
  printf("Device count: %d\n", count);

  int driverVersion = 0, runtimeVersion = 0;
  cudaSafeCall(cudaDriverGetVersion(&driverVersion));
  cudaSafeCall(cudaRuntimeGetVersion(&runtimeVersion));

  const char* computeMode[] = {
      "Default (multiple host threads can use ::cudaSetDevice() simultaneously)",
      "Exclusive (only one host thread in one process can use ::cudaSetDevice())",
      "Prohibited (no host thread can use ::cudaSetDevice())",
      "Exclusive Process (many threads in one process can use ::cudaSetDevice())",
      "Unknown",
      nullptr};

  for (int dev = beg; dev < end; ++dev) {
    cudaDeviceProp prop;
    cudaSafeCall(cudaGetDeviceProperties(&prop, dev));

    int sm_cores = convertSMVer2Cores(prop.major, prop.minor);

    printf("\nDevice %d: \"%s\"\n", dev, prop.name);
    printf("  CUDA Driver Version / Runtime Version          %d.%d / %d.%d\n",
           driverVersion / 1000,
           driverVersion % 100,
           runtimeVersion / 1000,
           runtimeVersion % 100);
    printf("  CUDA Capability Major/Minor version number:    %d.%d\n",
           prop.major,
           prop.minor);
    printf(
        "  Total amount of global memory:                 %.0f MBytes (%llu bytes)\n",
        (float)prop.totalGlobalMem / 1048576.0f,
        (unsigned long long)prop.totalGlobalMem);
    printf("  (%2d) Multiprocessors x (%2d) CUDA Cores/MP:     %d CUDA Cores\n",
           prop.multiProcessorCount,
           sm_cores,
           sm_cores * prop.multiProcessorCount);
    printf("  GPU Clock Speed:                               %.2f GHz\n",
           prop.clockRate * 1e-6f);

    // This is not available in the CUDA Runtime API, so we make the necessary calls the
    // driver API to support this for output
    int memoryClock, memBusWidth, L2CacheSize;
    getCudaAttribute<int>(&memoryClock, CU_DEVICE_ATTRIBUTE_MEMORY_CLOCK_RATE, dev);
    getCudaAttribute<int>(
        &memBusWidth, CU_DEVICE_ATTRIBUTE_GLOBAL_MEMORY_BUS_WIDTH, dev);
    getCudaAttribute<int>(&L2CacheSize, CU_DEVICE_ATTRIBUTE_L2_CACHE_SIZE, dev);

    printf("  Memory Clock rate:                             %.2f Mhz\n",
           memoryClock * 1e-3f);
    printf("  Memory Bus Width:                              %d-bit\n", memBusWidth);
    if (L2CacheSize)
      printf("  L2 Cache Size:                                 %d bytes\n",
             L2CacheSize);

    printf("  Max Texture Dimension Size (x,y,z)             1D=(%d), 2D=(%d,%d), "
           "3D=(%d,%d,%d)\n",
           prop.maxTexture1D,
           prop.maxTexture2D[0],
           prop.maxTexture2D[1],
           prop.maxTexture3D[0],
           prop.maxTexture3D[1],
           prop.maxTexture3D[2]);
    printf("  Max Layered Texture Size (dim) x layers        1D=(%d) x %d, 2D=(%d,%d) "
           "x %d\n",
           prop.maxTexture1DLayered[0],
           prop.maxTexture1DLayered[1],
           prop.maxTexture2DLayered[0],
           prop.maxTexture2DLayered[1],
           prop.maxTexture2DLayered[2]);
    printf("  Total amount of constant memory:               %u bytes\n",
           (int)prop.totalConstMem);
    printf("  Total amount of shared memory per block:       %u bytes\n",
           (int)prop.sharedMemPerBlock);
    printf("  Total number of registers available per block: %d\n", prop.regsPerBlock);
    printf("  Warp size:                                     %d\n", prop.warpSize);
    printf("  Maximum number of threads per block:           %d\n",
           prop.maxThreadsPerBlock);
    printf("  Maximum sizes of each dimension of a block:    %d x %d x %d\n",
           prop.maxThreadsDim[0],
           prop.maxThreadsDim[1],
           prop.maxThreadsDim[2]);
    printf("  Maximum sizes of each dimension of a grid:     %d x %d x %d\n",
           prop.maxGridSize[0],
           prop.maxGridSize[1],
           prop.maxGridSize[2]);
    printf("  Maximum memory pitch:                          %u bytes\n",
           (int)prop.memPitch);
    printf("  Texture alignment:                             %u bytes\n",
           (int)prop.textureAlignment);

    printf(
        "  Concurrent copy and execution:                 %s with %d copy engine(s)\n",
        (prop.deviceOverlap ? "Yes" : "No"),
        prop.asyncEngineCount);
    printf("  Run time limit on kernels:                     %s\n",
           prop.kernelExecTimeoutEnabled ? "Yes" : "No");
    printf("  Integrated GPU sharing Host Memory:            %s\n",
           prop.integrated ? "Yes" : "No");
    printf("  Support host page-locked memory mapping:       %s\n",
           prop.canMapHostMemory ? "Yes" : "No");

    printf("  Concurrent kernel execution:                   %s\n",
           prop.concurrentKernels ? "Yes" : "No");
    printf("  Alignment requirement for Surfaces:            %s\n",
           prop.surfaceAlignment ? "Yes" : "No");
    printf("  Device has ECC support enabled:                %s\n",
           prop.ECCEnabled ? "Yes" : "No");
    printf("  Device is using TCC driver mode:               %s\n",
           prop.tccDriver ? "Yes" : "No");
    printf("  Device supports Unified Addressing (UVA):      %s\n",
           prop.unifiedAddressing ? "Yes" : "No");
    printf("  Device PCI Bus ID / PCI location ID:           %d / %d\n",
           prop.pciBusID,
           prop.pciDeviceID);
    printf("  Compute Mode:\n");
    printf("      %s \n", computeMode[prop.computeMode]);
  }

  printf("\n");
  printf("deviceQuery, CUDA Driver = CUDART");
  printf(", CUDA Driver Version  = %d.%d", driverVersion / 1000, driverVersion % 100);
  printf(", CUDA Runtime Version = %d.%d", runtimeVersion / 1000, runtimeVersion % 100);
  printf(", NumDevs = %d\n\n", count);
  fflush(stdout);
}

void
pcl::gpu::printShortCudaDeviceInfo(int device)
{
  int count = getCudaEnabledDeviceCount();
  bool valid = (device >= 0) && (device < count);

  int beg = valid ? device : 0;
  int end = valid ? device + 1 : count;

  int driverVersion = 0, runtimeVersion = 0;
  cudaSafeCall(cudaDriverGetVersion(&driverVersion));
  cudaSafeCall(cudaRuntimeGetVersion(&runtimeVersion));

  for (int dev = beg; dev < end; ++dev) {
    cudaDeviceProp prop;
    cudaSafeCall(cudaGetDeviceProperties(&prop, dev));

    const char* arch_str = prop.major < 2 ? " (pre-Fermi)" : "";
    printf("[pcl::gpu::printShortCudaDeviceInfo] : Device %d:  \"%s\"  %.0fMb",
           dev,
           prop.name,
           (float)prop.totalGlobalMem / 1048576.0f);
    printf(", sm_%d%d%s, %d cores",
           prop.major,
           prop.minor,
           arch_str,
           convertSMVer2Cores(prop.major, prop.minor) * prop.multiProcessorCount);
    printf(", Driver/Runtime ver.%d.%d/%d.%d\n",
           driverVersion / 1000,
           driverVersion % 100,
           runtimeVersion / 1000,
           runtimeVersion % 100);
  }
  fflush(stdout);
}

#endif

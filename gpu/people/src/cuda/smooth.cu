#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/utils/device/funcattrib.hpp>

#include <stdio.h>
#include <assert.h>

typedef pcl::gpu::DeviceArray2D<unsigned char> Labels;
typedef pcl::gpu::DeviceArray2D<unsigned short> Depth;

using namespace pcl::gpu;

namespace pcl
{
    namespace device
    {

        __global__ void smoothKernel(const PtrStep<unsigned char> src, const PtrStep<unsigned short> depth, PtrStepSz<unsigned char> dst,
                                     int patch, int depthThres, int num_parts)
        {
            int x = threadIdx.x + blockIdx.x * blockDim.x;
            int y = threadIdx.y + blockIdx.y * blockDim.y;

            int patch2 = patch/2;
            if (x >= (dst.cols - patch2*2) || y >= (dst.rows - patch2*2))
                return;

            int depth_center = depth.ptr(y + patch2)[x+patch2];

            unsigned int hist4[] = { 0, 0, 0, 0,/**/ 0, 0, 0/*, 0*/ };

            for (int dy = 0; dy < patch; ++dy)
            {
                const unsigned short* d = depth.ptr(dy + y);
                const unsigned char * s = src.ptr(dy + y);

                for (int dx = 0; dx < patch; ++dx)
                {
                    if (abs(d[dx+x] - depth_center) < depthThres)
                    {
                        int l = s[dx+x];
                        if (l < num_parts)
                        {
                            int bin = l / sizeof(int);
                            int off = l - bin * sizeof(int);

                            hist4[bin] += (unsigned)(1 << 8*off);
                        }
                    }
                }
            }

            int max_label = src.ptr(y + patch2)[x+patch2];
            int max_value = 0;

            #pragma unroll
            for(int i = 0; i < sizeof(hist4)/sizeof(hist4[0]); ++i)
            {
                int bin = hist4[i];
                int val;

                val = (bin >> 0) & 0xFF;
                if (max_value < val)
                {
                    max_value = val;
                    max_label = sizeof(int)*i+0;
                }

                val = (bin >> 8) & 0xFF;
                if (max_value < val)
                {
                    max_value = val;
                    max_label = sizeof(int)*i+1;
                }

                val = (bin >> 16) & 0xFF;
                if (max_value < val)
                {
                    max_value = val;
                    max_label = sizeof(int)*i+2;
                }

                val = (bin >> 24) & 0xFF;
                if (max_value < val)
                {
                    max_value = val;
                    max_label = sizeof(int)*i+3;
                }
            }

            dst.ptr(y + patch2)[x+patch2] = max_label;
        }

    }

}

void smoothLabelImage(const Labels& src, const Depth& depth, Labels& dst, int num_parts, int  patch_size, int  depthThres)
{
	assert( num_parts < 28 ); //should modify kernel otherwise, uncomment ending zero int hist4 array

    dst.create(src.rows(), src.cols());

    dim3 block(32, 8);
    dim3 grid(divUp(src.cols(), block.x), divUp(src.rows(), block.y));

    pcl::device::smoothKernel<<<grid, block>>>(src, depth, dst, patch_size, depthThres, num_parts);

    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}



#include "device.hpp"

namespace pcl
{
    namespace device
    {
        const float sigma_color = 10; //in mm
        const float sigma_space = 3; // in pixels
        const int R = 4; //static_cast<int>(sigma_space * 1.5);
        const int D = R * 2 + 1; 

        __global__ void bilateralKernel(const PtrStepSz<ushort> src, PtrStep<ushort> dst, float sigma_space2, float sigma_color2)
        {
            int x = threadIdx.x + blockIdx.x * blockDim.x;
            int y = threadIdx.y + blockIdx.y * blockDim.y;

            if (x >= src.cols || y >= src.rows)
                return;

            int value = src.ptr(y)[x];

            int tx = min(x - D/2 + D, src.cols-1);
            int ty = min(y - D/2 + D, src.rows-1);

            float sum1 = 0;
            float sum2 = 0;

            for (int cy = max(y - D/2, 0); cy < ty; ++cy)
                for (int cx = max(x - D/2, 0); cx < tx; ++cx)
                {
                    int tmp = src.ptr(cy)[cx];

                    float space2  = (x - cx)*(x - cx) + (y - cy)*(y - cy);
                    float color2 = (value - tmp) * (value - tmp);

                    float wd  = __expf(-space2/sigma_space2 * 0.5f); 
                    float wc =  __expf(-color2/sigma_color2 * 0.5f);

                    float weight = wd * wc;

                    sum1 += tmp * weight;
                    sum2 += weight;
                }

                int res = __float2int_rn(sum1/sum2);
                dst.ptr(y)[x] = max(0, min(res, numeric_limits<short>::max()));
        }


        __device__ __forceinline__ int fetch(int y, int x, const PtrStepSz<ushort>& src)
        {
            x = max(0, min(x, src.cols-1));
            y = max(0, min(y, src.rows-1));
            return src.ptr(y)[x];
        }

        __global__ void pyrDownKernel(const PtrStepSz<ushort> src, PtrStepSz<ushort> dst, float sigma_color)
        {            
            int x = blockIdx.x * blockDim.x + threadIdx.x;
            int y = blockIdx.y * blockDim.y + threadIdx.y;

            if (x >= dst.cols || y >= dst.rows)
                return;

            const int D = 5;

            int center = src.ptr(2*y)[2*x];

            int tx = min(2*x - D/2 + D, src.cols-1);
            int ty = min(2*y - D/2 + D, src.rows-1);
            int cy = max(0, 2*y - D/2);

            int sum = 0;
            int count = 0;

            for (; cy < ty; ++cy)
                for (int cx = max(0, 2*x - D/2); cx < tx; ++cx)
                {
                    int val = src.ptr(cy)[cx];
                    if (abs(val - center) < 3 * sigma_color)
                    {
                        sum += val;
                        ++count;
                    }                    
                }
            
            dst.ptr(y)[x] = sum / count;
        }
    }
}

void pcl::device::bilateralFilter(const DepthMap& src, DepthMap& dst)
{        
    dim3 block(32, 8);
    dim3 grid(divUp(src.cols(), block.x), divUp(src.rows(), block.y));

    bilateralKernel<<<grid, block, 0, stream>>>(src, dst, sigma_space * sigma_space, sigma_color * sigma_color);
    cudaSafeCall( cudaGetLastError() );	
    if (stream == 0)
        cudaSafeCall(cudaDeviceSynchronize());
};

void pcl::device::pyrDown(const DepthMap& src, DepthMap& dst)
{       
    dst.create(src.rows()/2,  src.cols()/2);

    dim3 block(32, 8);
    dim3 grid(divUp(dst.cols(), block.x), divUp(dst.rows(), block.y));
    
    pyrDownKernel<<<grid, block, 0, stream>>>(src, dst, sigma_color);    
    cudaSafeCall( cudaGetLastError() );	
    if (stream == 0)
        cudaSafeCall(cudaDeviceSynchronize());
};



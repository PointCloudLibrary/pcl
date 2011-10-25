#include "device.hpp"

using namespace pcl::device;

namespace pcl
{
    namespace device
    {        

        struct ImageGenerator
        {
            enum 
            {
                CTA_SIZE_X = 32, CTA_SIZE_Y = 8
            };

            PtrStep<float> vmap;
            PtrStep<float> nmap;

            LightSource light;

            mutable PtrStepSz<uchar3> dst;

            __device__ __forceinline__ void operator()()const
            {
                int x = threadIdx.x + blockIdx.x * CTA_SIZE_X;
                int y = threadIdx.y + blockIdx.y * CTA_SIZE_Y;

                if (x >= dst.cols || y >= dst.rows)
                    return;

                float3 v, n;
                v.x = vmap.ptr(y)[x];
                n.x = nmap.ptr(y)[x];

                uchar3 color = make_uchar3(0, 0, 0);

                if (!isnan(v.x) && !isnan(n.x))
                {
                    v.y = vmap.ptr(y+  dst.rows)[x];
                    v.z = vmap.ptr(y+2*dst.rows)[x];

                    n.y = nmap.ptr(y+  dst.rows)[x];
                    n.z = nmap.ptr(y+2*dst.rows)[x];

                    float weight = 1.f; 

                    for(int i = 0; i < light.number; ++i)
                    {						
                        float3 vec = normalized(light.pos[i] - v);

                        weight *= fabs(dot(vec, n));					
                    }

                    int br = (int)(255*weight);
                    br = max(50, min(255, br));
                    color = make_uchar3(br, br, br);
                }				
                dst.ptr(y)[x] = color;
            }
        };

        __global__ void generateImageKernel(const ImageGenerator ig) { ig(); }		
    }
}


void pcl::device::generateImage(const MapArr& vmap, const MapArr& nmap, const LightSource& light, PtrStepSz<uchar3> dst)
{
    ImageGenerator ig;
    ig.vmap = vmap;
    ig.nmap = nmap;
    ig.light = light;	
    ig.dst = dst;

    dim3 block(ImageGenerator::CTA_SIZE_X, ImageGenerator::CTA_SIZE_Y);
    dim3 grid(divUp(dst.cols, block.x), divUp(dst.rows, block.y));

    generateImageKernel<<<grid, block>>>(ig);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall(cudaDeviceSynchronize());
}
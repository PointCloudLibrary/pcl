#include "device.hpp"

#include "pcl/gpu/features/device/eigen.hpp"

namespace pcl
{
    namespace device
    {   
        enum
        {
            kx = 7,
            ky = 7,
            STEP = 2
        };

        __global__ void computeNmapKernelEigen(int rows, int cols, const PtrStep<float> vmap, PtrStep<float> nmap)
        {
            int u = threadIdx.x + blockIdx.x * blockDim.x;
            int v = threadIdx.y + blockIdx.y * blockDim.y;

            if (u >= cols || v >= rows)            
                return;

            nmap.ptr(v)[u] = numeric_limits<float>::quiet_NaN();

            if (isnan(vmap.ptr(v)[u]))
                return;

            int ty = min(v - ky/2 + ky, rows-1);
            int tx = min(u - kx/2 + kx, cols-1);

            float3 centroid = make_float3(0.f, 0.f, 0.f);
            int counter = 0;
            for (int cy = max(v - ky/2, 0); cy < ty; cy+=STEP)
                for (int cx = max(u - kx/2, 0); cx < tx; cx+=STEP)
                {                    
                    float v_x = vmap.ptr(cy)[cx];
                    if (!isnan(v_x))
                    {
                        centroid.x += v_x;
                        centroid.y += vmap.ptr(cy+  rows)[cx];
                        centroid.z += vmap.ptr(cy+2*rows)[cx];
                        ++counter;
                    }
                }

                if (counter < kx*ky/2)
                    return;

                centroid *= 1.f/counter;

                float cov[] = {0, 0, 0, 0, 0, 0};

                for (int cy = max(v - ky/2, 0); cy < ty; cy+=STEP)
                    for (int cx = max(u - kx/2, 0); cx < tx; cx+=STEP)
                    {
                        float3 v;
                        v.x = vmap.ptr(cy)[cx];
                        if (isnan(v.x))
                            continue;

                        v.y = vmap.ptr(cy+  rows)[cx];
                        v.z = vmap.ptr(cy+2*rows)[cx];

                        float3 d = v - centroid;

                        cov[0] += d.x * d.x; //cov (0, 0) 
                        cov[1] += d.x * d.y; //cov (0, 1) 
                        cov[2] += d.x * d.z; //cov (0, 2) 
                        cov[3] += d.y * d.y; //cov (1, 1) 
                        cov[4] += d.y * d.z; //cov (1, 2) 
                        cov[5] += d.z * d.z; //cov (2, 2)        
                    }

            typedef Eigen33::Mat33 Mat33;
            Eigen33 eigen33(cov);

            Mat33     tmp;
            Mat33 vec_tmp;
            Mat33 evecs;
            float3 evals;
            eigen33.compute(tmp, vec_tmp, evecs, evals);

            float3 n = normalized(evecs[0]);

            u = threadIdx.x + blockIdx.x * blockDim.x;
            v = threadIdx.y + blockIdx.y * blockDim.y;

            nmap.ptr(v       )[u] = n.x;
            nmap.ptr(v+  rows)[u] = n.y;
            nmap.ptr(v+2*rows)[u] = n.z;
        }        
    }
}

void pcl::device::compteNormalsEigen(const MapArr& vmap, MapArr& nmap)
{
    int cols = vmap.cols();
    int rows = vmap.rows()/3;

    nmap.create(vmap.rows(), vmap.cols());

    dim3 block(32, 8);
    dim3 grid(1,1,1);
    grid.x = divUp(cols, block.x);
    grid.y = divUp(rows, block.y);

    computeNmapKernelEigen<<<grid, block>>>(rows, cols, vmap, nmap);
    cudaSafeCall( cudaGetLastError() );	
    cudaSafeCall(cudaDeviceSynchronize());
}
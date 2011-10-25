#include "device.hpp"

using namespace pcl::device;
using namespace pcl::gpu;

namespace pcl
{
    namespace device
    {        
        __global__ void computeVmapKernel(const PtrStepSz<unsigned short> depth, PtrStep<float> vmap, float fx_inv, float fy_inv, float cx, float cy)
        {
            int u = threadIdx.x + blockIdx.x * blockDim.x;
            int v = threadIdx.y + blockIdx.y * blockDim.y;

            if (u < depth.cols && v < depth.rows)
            {
                int z = depth.ptr(v)[u];

                if (z != 0)
                {					
                    float vx = z * (u - cx) * fx_inv;		
                    float vy = z * (v - cy) * fy_inv;		
                    float vz = z;

                    vmap.ptr(v                 )[u] = vx;
                    vmap.ptr(v + depth.rows    )[u] = vy;
                    vmap.ptr(v + depth.rows * 2)[u] = vz;
                }
                else
                    vmap.ptr(v)[u] = numeric_limits<float>::quiet_NaN();

            }
        }

        __global__ void computeNmapKernel(int rows, int cols, const PtrStep<float> vmap, PtrStep<float> nmap)
        {
            int u = threadIdx.x + blockIdx.x * blockDim.x;
            int v = threadIdx.y + blockIdx.y * blockDim.y;

            if (u >= cols || v >= rows)
                return;

            if (u == cols - 1 || v == rows - 1)
            {		
                nmap.ptr(v)[u] = numeric_limits<float>::quiet_NaN();
                return;
            }

            float3 v00, v01, v10;
            v00.x = vmap.ptr(v  )[u  ];
            v01.x = vmap.ptr(v  )[u+1];
            v10.x = vmap.ptr(v+1)[u  ];

            if (!isnan(v00.x) && !isnan(v01.x) && !isnan(v10.x))
            {
                v00.y = vmap.ptr(v   + rows)[u  ];
                v01.y = vmap.ptr(v   + rows)[u+1];
                v10.y = vmap.ptr(v+1 + rows)[u  ];

                v00.z = vmap.ptr(v   + 2*rows)[u  ];
                v01.z = vmap.ptr(v   + 2*rows)[u+1];
                v10.z = vmap.ptr(v+1 + 2*rows)[u  ];

                float3 r = normalized(cross(v01-v00, v10-v00));

                nmap.ptr(v       )[u] = r.x;
                nmap.ptr(v+  rows)[u] = r.y;
                nmap.ptr(v+2*rows)[u] = r.z;
            }
            else
                nmap.ptr(v)[u] = numeric_limits<float>::quiet_NaN();
        }
    }
}


void pcl::device::createVMap(const Intr& intr, const DepthMap& depth, MapArr& vmap)
{
    vmap.create(depth.rows() * 3, depth.cols());

    dim3 block(32, 8);
    dim3 grid(1,1,1);
    grid.x = divUp(depth.cols(), block.x);
    grid.y = divUp(depth.rows(), block.y);

    float fx = intr.fx, cx = intr.cx;
    float fy = intr.fy, cy = intr.cy;

    computeVmapKernel<<<grid, block>>>(depth, vmap, 1.f/fx, 1.f/fy, cx, cy);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall(cudaDeviceSynchronize());
}

void pcl::device::createNMap(const MapArr& vmap, MapArr& nmap)
{
    nmap.create(vmap.rows(), vmap.cols());

    int rows = vmap.rows()/3;
    int cols = vmap.cols();

    dim3 block(32, 8);
    dim3 grid(1,1,1);
    grid.x = divUp(cols, block.x);
    grid.y = divUp(rows, block.y);

    computeNmapKernel<<<grid, block>>>(rows, cols, vmap, nmap);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall(cudaDeviceSynchronize());
}

namespace pcl
{
    namespace device
    {	
        __global__ void tranformMapsKernel(int rows, int cols, const PtrStep<float> vmap_src, const PtrStep<float> nmap_src, 
            const Mat33 Rmat, const float3 tvec, PtrStepSz<float> vmap_dst, PtrStep<float> nmap_dst)
        {
            int x = threadIdx.x + blockIdx.x * blockDim.x;
            int y = threadIdx.y + blockIdx.y * blockDim.y;

            const float qnan = pcl::device::numeric_limits<float>::quiet_NaN();

            if (x < cols && y < rows)
            {
                //vetexes
                float3 vsrc, vdst = make_float3(qnan, qnan, qnan);
                vsrc.x = vmap_src.ptr(y)[x];

                if (!isnan(vsrc.x))
                {
                    vsrc.y = vmap_src.ptr(y+  rows)[x];
                    vsrc.z = vmap_src.ptr(y+2*rows)[x];

                    vdst = Rmat * vsrc + tvec;					

                    vmap_dst.ptr(y+  rows)[x] = vdst.y;
                    vmap_dst.ptr(y+2*rows)[x] = vdst.z;
                }

                vmap_dst.ptr(y)[x] = vdst.x;				

                //normals
                float3 nsrc, ndst = make_float3(qnan, qnan, qnan);
                nsrc.x = nmap_src.ptr(y)[x];

                if (!isnan(nsrc.x))
                {				
                    nsrc.y = nmap_src.ptr(y+  rows)[x];
                    nsrc.z = nmap_src.ptr(y+2*rows)[x];

                    ndst = Rmat * nsrc;

                    nmap_dst.ptr(y+  rows)[x] = ndst.y;
                    nmap_dst.ptr(y+2*rows)[x] = ndst.z;
                }

                nmap_dst.ptr(y)[x] = ndst.x;				
            }
        }
    }
}

void pcl::device::tranformMaps(const MapArr& vmap_src, const MapArr& nmap_src, const Mat33& Rmat, const float3& tvec, MapArr& vmap_dst, MapArr& nmap_dst)
{
    int cols = vmap_src.cols();
    int rows = vmap_src.rows()/3;

    vmap_dst.create(rows * 3, cols);
    nmap_dst.create(rows * 3, cols);

    dim3 block(32, 8);
    dim3 grid(1,1,1);
    grid.x = divUp(cols, block.x);
    grid.y = divUp(rows, block.y);

    tranformMapsKernel<<<grid, block>>>(rows, cols, vmap_src, nmap_src, Rmat, tvec, vmap_dst, nmap_dst);
    cudaSafeCall( cudaGetLastError() );	
    cudaSafeCall(cudaDeviceSynchronize());
}


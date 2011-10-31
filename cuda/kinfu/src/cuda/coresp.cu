#include "device.hpp"

namespace pcl
{
    namespace device
    {
        struct CorespSearch
        {
            PtrStep<float> vmap_g_curr;
            PtrStep<float> nmap_g_curr;

            Mat33 Rprev_inv;
            float3 tprev;

            Intr intr;

            PtrStep<float> vmap_g_prev;
            PtrStep<float> nmap_g_prev;

            float  distThres;
            float angleThres;

            mutable PtrStepSz<short2> coresp;

            __device__ __forceinline__ operator()() const
            {
                int x = threadIdx.x + blockIdx.x * blockDim.x;
                int y = threadIdx.y + blockIdx.y * blockDim.y;

                if (x >= coresp.cols || y >= coresp.rows)
                    return;

                coresp.ptr(y)[x] = make_short2(-1, -1);

                float3 ncurr_g;                
                ncurr_g.x = nmap_g_curr.ptr(y)[x];

                if (isnan(ncurr_g.x))
                    return;
                
                float3 vcurr_g;
                vcurr_g.x = vmap_g_curr.ptr(y              )[x];
                vcurr_g.y = vmap_g_curr.ptr(y+  coresp.rows)[x];
                vcurr_g.z = vmap_g_curr.ptr(y+2*coresp.rows)[x];

                float3 vcurr_cp = Rprev_inv * (vcurr_g - tprev); // prev camera coo space
                
                int2 ukr; //projection
                ukr.x = __float2int_rn(vcurr_cp.x * intr.fx/vcurr_cp.z + intr.cx); //4
				ukr.y = __float2int_rn(vcurr_cp.y * intr.fy/vcurr_cp.z + intr.cy); //4

                if (ukr.x < 0 || ukr.y < 0 || ukr.x >= coresp.cols || ukr.y >= coresp.rows)
                    return;

                float3 nprev_g;
                nprev_g.x = nmap_g_prev.ptr(ukr.y)[ukr.x];

                if (isnan(nprev_g.x))
                    return;                

                float3 vprev_g;
                vprev_g.x = vmap_g_prev.ptr(y              )[x];
                vprev_g.y = vmap_g_prev.ptr(y+  coresp.rows)[x];
                vprev_g.z = vmap_g_prev.ptr(y+2*coresp.rows)[x];

                float dist = norm(vcurr_g - vprev_g);
                if (dist > distThres)
                    return;
                
                ncurr_g.y = nmap_g_curr.ptr(y+  coresp.rows)[x];
                ncurr_g.z = nmap_g_curr.ptr(y+2*coresp.rows)[x];
                
                nprev_g.y = nmap_g_prev.ptr(ukr.y+  coresp.rows)[ukr.x];
                nprev_g.z = nmap_g_prev.ptr(ukr.y+2*coresp.rows)[ukr.x];

                float sine = norm(cross(ncurr_g, nprev_g));

                if (sine < 1 && asin(sine) < angleThres)
                    coresp.ptr(y)[x] = make_short2(p.x, p.y);					                
            }
        };

        __global__ void corespKernel(const CorespSearch cs) { cs(); }
    }
}


void pcl::device::findCoresp(const MapArr& vmap_g_curr, const MapArr& nmap_g_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
                             const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, PtrStepSz<short2>& coresp)
{
    CorespSearch cs;

    cs.vmap_g_curr = vmap_g_curr;
    cs.nmap_g_curr = nmap_g_curr;

    cs.Rprev_inv = Rprev_inv;
    cs.tprev = tprev;

    cs.intr = intr;

    cs.vmap_g_prev = vmap_g_prev;
    cs.nmap_g_prev = nmap_g_prev;

    cs.distThres  = distThres;
    cs.angleThres = angleThres;

    cs.coresp = coresp;

    dim3 block(32, 8);
    dim3 grid(divUp(coresp.cols, block.x), divUp(coresp.rows, block.y));

    corespKernel<<<grid, block>>>(cs);
    
    cudaSafeCall( cudaGetLastError() );	
    cudaSafeCall(cudaDeviceSynchronize());
}

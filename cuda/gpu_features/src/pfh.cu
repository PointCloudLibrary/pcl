#include "internal.hpp"

#include "pcl/gpu/utils/safe_call.hpp"
#include "pcl/gpu/utils/device/warp.hpp"
#include "pcl/gpu/utils/device/block.hpp"
#include "utils/vector_operations.hpp"
#include "utils/pair_features.hpp"
#include "pcl/gpu/utils/device/funcattrib.hpp"

using namespace pcl::gpu;


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace pcl
{
    namespace device
    {    
        struct Repack
        {   
            enum 
            {
                CTA_SIZE = 256,
                WARPS = CTA_SIZE/Warp::WARP_SIZE
            };

            const PointType* cloud;
            const NormalType* normals;
            int work_size;

            PtrStep<int> gindices;            
            const int* sizes;         

            mutable PtrStep<float> output;
            int max_elems;

            __device__ void operator()() const
            {                
                int idx = WARPS * blockIdx.x + Warp::id();

                if (idx >= work_size)
                    return;

                const int *nbeg = gindices.ptr(idx);
                int size = sizes[idx];
                int idx_shift = max_elems * idx;

                for(int i = Warp::laneId(); i < size; i += Warp::STRIDE)
                {
                    int cloud_index = nbeg[i];
                    float3 p = fetch(cloud, cloud_index);
                    float3 n = fetch(normals, cloud_index);

                    output.ptr(0)[i + idx_shift] = p.x;
                    output.ptr(1)[i + idx_shift] = p.y;
                    output.ptr(2)[i + idx_shift] = p.z;

                    output.ptr(3)[i + idx_shift] = n.x;
                    output.ptr(4)[i + idx_shift] = n.y;
                    output.ptr(5)[i + idx_shift] = n.z;                                           
                }
            }

            template<class It> 
            __device__ __forceinline__ float3 fetch(It ptr, int index) const
            {
                //return tr(ptr[index]);
                return *(float3*)&ptr[index];
            }
        };

        struct Pfh125
        {
            enum 
            {
                CTA_SIZE = 256,

                NR_SPLIT = 5,
                NR_SPLIT_2 = NR_SPLIT * NR_SPLIT,

                FSize = NR_SPLIT * NR_SPLIT * NR_SPLIT
            };

            size_t work_size;
            const int* sizes;

            PtrStep<float> rpk;
            int max_elems;
                       
            mutable PtrStep<float> output;

            __device__ __forceinline__ void operator()() const
            {                                
                int idx = blockIdx.x;

                if (idx >= work_size)
                    return;                

                int size = sizes[idx];                
                int size2 = size * size;
                int idx_shift = max_elems * idx;

                float hist_incr = 100.f / (size2 - 1);
                
                __shared__ float pfh_histogram[FSize];
                Block::fill(pfh_histogram, pfh_histogram + FSize, 0.f);
                __syncthreads();

                // Iterate over all the points in the neighborhood
                int i = threadIdx.y * blockDim.x + threadIdx.x;
                int stride = Block::stride();

                for( ; i < size2; i += stride )
                {
                    int i_idx = i / size + idx_shift;
                    int j_idx = i % size + idx_shift;

                    if (i_idx != j_idx)
                    {
                        float3 pi, ni, pj, nj;
                        pi.x = rpk.ptr(0)[i_idx];
                        pj.x = rpk.ptr(0)[j_idx];

                        pi.y = rpk.ptr(1)[i_idx];
                        pj.y = rpk.ptr(1)[j_idx];

                        pi.z = rpk.ptr(2)[i_idx];
                        pj.z = rpk.ptr(2)[j_idx];

                        ni.x = rpk.ptr(3)[i_idx];
                        nj.x = rpk.ptr(3)[j_idx];

                        ni.y = rpk.ptr(4)[i_idx];
                        nj.y = rpk.ptr(4)[j_idx];

                        ni.z = rpk.ptr(5)[i_idx];                
                        nj.z = rpk.ptr(5)[j_idx];

                        float f1, f2, f3, f4;
                        // Compute the pair NNi to NNj                        
                        if (computePairFeatures (pi, ni, pj, nj, f1, f2, f3, f4))
                        {                            
                            // Normalize the f1, f2, f3 features and push them in the histogram
                            int find0 = floor( NR_SPLIT * ((f1 + M_PI) * (1.f / (2.f * M_PI))) );                            
                            find0 = min(NR_SPLIT - 1, max(0, find0));

                            int find1 = floor( NR_SPLIT * ( (f2 + 1.f) * 0.5f ) );
                            find1 = min(NR_SPLIT - 1, max(0, find1));

                            int find2 = floor( NR_SPLIT * ( (f3 + 1.f) * 0.5f ) );
                            find2 = min(NR_SPLIT - 1, max(0, find2));

                            int h_index = find0 + NR_SPLIT * find1 + NR_SPLIT_2 * find2;
                            atomicAdd(pfh_histogram + h_index, hist_incr);
                        }           
                    }
                }
                __syncthreads();
                Block::copy(pfh_histogram, pfh_histogram + FSize, output.ptr(idx));
            }

            template<class It> 
            __device__ __forceinline__ float3 fetch(It ptr, int index) const
            {
                //return tr(ptr[index]);
                return *(float3*)&ptr[index];
            }
        };

        __global__ void repackKernel(const Repack repack) { repack(); }
        __global__ void pfhKernel(const Pfh125 pfh125) { pfh125(); }
    }    
}

void pcl::device::PfhImpl::repack()
{   
    max_elems_rpk = (neighbours.max_elems/32 + 1) * 32;
    data_rpk.create(6, neighbours.sizes.size() * max_elems_rpk);

    Repack rpk;
    rpk.sizes = neighbours.sizes;
    rpk.gindices = neighbours;

    rpk.cloud = cloud;
    rpk.normals = normals;
    rpk.work_size = neighbours.sizes.size();
    
    rpk.output = data_rpk;    
    rpk.max_elems = max_elems_rpk;

    int block = Repack::CTA_SIZE;        
    int grid = divUp(rpk.work_size, Repack::WARPS);

    //printFuncAttrib(repackKernel);

    device::repackKernel<<<grid, block>>>(rpk);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}


void pcl::device::PfhImpl::compute(DeviceArray2D<PFHSignature125>& features)
{
    repack();

    Pfh125 fph;
    fph.work_size = neighbours.sizes.size();
    fph.sizes = neighbours.sizes;
    fph.rpk = data_rpk;
    fph.max_elems = max_elems_rpk;                       
    fph.output = features;

    int block = Pfh125::CTA_SIZE;        
    int grid = fph.work_size;

    //printFuncAttrib(pfhKernel);

    device::pfhKernel<<<grid, block>>>(fph);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}
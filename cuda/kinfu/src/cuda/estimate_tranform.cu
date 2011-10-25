#include "device.hpp"

#include "pcl/gpu/utils/device/block.hpp"

namespace pcl
{
    namespace device
    {        
        template<typename T>
        struct TransformEstimator
        {            
            enum
            {
                CTA_SIZE_X = 32,
                CTA_SIZE_Y = 8,
                CTA_SIZE = CTA_SIZE_X * CTA_SIZE_Y
            };

            struct plus 
            {              
                __forceinline__ __device__ T operator()(const T &lhs, const volatile T& rhs) const { return lhs + rhs; }
            }; 

            PtrStep<float> v_dst;
            PtrStep<float> n_dst;			
            PtrStep<float> v_src;
            int rows;
            int cols;

            mutable PtrStep<T> gbuf;

            __device__ __forceinline__ void operator()() const
            {
                int x = threadIdx.x + blockIdx.x * blockDim.x;
                int y = threadIdx.y + blockIdx.y * blockDim.y;

                if (x >= cols || y >= rows)
                    return;

                float row[7];               
                row[0] = row[1] = row[2] = row[3] = row[4] = row[5] = row[6] = 0.f;

                float3 n, s;

                n.x = n_dst.ptr(y)[x];
                s.x = v_src.ptr(y)[x];

                if (!isnan(n.x) && !isnan(s.x))
                {
                    n.y = n_dst.ptr(y+  rows)[x];
                    n.z = n_dst.ptr(y+2*rows)[x];                        

                    float3 d;
                    d.x = v_dst.ptr(y       )[x];
                    d.y = v_dst.ptr(y+  rows)[x];
                    d.z = v_dst.ptr(y+2*rows)[x];

                    float b = dot(n, d);

                    s.y = v_src.ptr(y+  rows)[x];
                    s.z = v_src.ptr(y+2*rows)[x];                        

                    b = b - dot(n, s);
                    row[6] = b;

                    *(float3*)&row[0] = cross(s, n);
                    *(float3*)&row[3] = n;                     
                }
                
                __shared__ T smem[CTA_SIZE];
                int tid = Block::straightenedThreadId();

                int shift = 0;
                
                
                for(int i = 0; i < 6; ++i) //rows
                    #pragma unroll
                    for(int j = i; j < 7; ++j) // cols + b
                    {
                        smem[tid] = row[i] * row[j];
                        __syncthreads();
                        Block::reduce<CTA_SIZE>(smem, plus());

                        if (tid == 0)
                            gbuf.ptr(shift++)[blockIdx.x + gridDim.x * blockIdx.y] = smem[0];
                    }
            }
        };

        template<typename T>
        struct TranformReduction
        {
            enum
            {
                CTA_SIZE = 512,
                STRIDE = CTA_SIZE,

                B = 6, COLS = 6, ROWS = 6, DIAG = 6,
                UPPER_DIAG_MAT = (COLS * ROWS - DIAG)/2 + DIAG,
                TOTAL = UPPER_DIAG_MAT + B, 

                GRID_X = TOTAL
            };

            PtrStep<T> gbuf;
            int length;
            mutable T* output;

            __device__ __forceinline__ void operator()()const
            {				
                const T *beg = gbuf.ptr(blockIdx.x);
                const T *end = beg + length;								

                int tid = threadIdx.x;

                T sum = 0.f;
                for(const T *t = beg + tid; t < end; t += STRIDE)
                    sum+=*t;

                __shared__ T smem[CTA_SIZE];

                smem[tid] = sum;
                __syncthreads();

                Block::reduce<CTA_SIZE>(smem, TransformEstimator<T>::plus());

                if (tid == 0)
                    output[blockIdx.x] = smem[0];					
            }
        };

        __global__ void TransformEstimatorKernel1(const TransformEstimator<double> te) { te(); }
        __global__ void TransformEstimatorKernel2(const TranformReduction<double> tr) { tr(); }

        __global__ void TransformEstimatorKernel1(const TransformEstimator<float> te) { te(); }
        __global__ void TransformEstimatorKernel2(const TranformReduction<float> tr) { tr(); }
    }
}

void pcl::device::estimateTransform(const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, 
                                    DeviceArray2D<work_type>& gbuf, DeviceArray<work_type>& mbuf, work_type* matrixA_host, work_type* vectorB_host)
{	
    typedef TransformEstimator<work_type> TEst;
    typedef TranformReduction<work_type> TRed;

    int cols = v_dst.cols();
    int rows = v_dst.rows()/3;

    dim3 block(TEst::CTA_SIZE_X, TEst::CTA_SIZE_Y);
    dim3 grid(1,1,1);
    grid.x = divUp(cols, block.x);
    grid.y = divUp(rows, block.y);

    gbuf.create(TRed::TOTAL, grid.x * grid.y);

    TEst te;	
    te.n_dst = n_dst;
    te.v_dst = v_dst;			
    te.v_src = v_src;
    te.gbuf = gbuf;
    te.rows = rows;
    te.cols = cols;

    TransformEstimatorKernel1<<<grid, block>>>(te);
    cudaSafeCall( cudaGetLastError() );	
    cudaSafeCall(cudaDeviceSynchronize());

    mbuf.create(TRed::TOTAL);

    TRed tr;
    tr.gbuf = gbuf;			
    tr.length = gbuf.cols();
    tr.output = mbuf;	

    TransformEstimatorKernel2<<<gbuf.rows(), TRed::CTA_SIZE>>>(tr);

    cudaSafeCall( cudaGetLastError() );	
    cudaSafeCall(cudaDeviceSynchronize());

    work_type host_data[TRed::TOTAL];
    mbuf.download(host_data);

    int shift = 0;
    for(int i = 0; i < 6; ++i) //rows
        for(int j = i; j < 7; ++j) // cols + b
        {
            work_type value = host_data[shift++];
            if (j == 6) // vector b
                vectorB_host[i] = value;
            else
                matrixA_host[j*6 + i] = matrixA_host[i*6 + j] = value;
        }    
}


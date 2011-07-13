#pragma once

#include <modified_load.hpp>

#include "pcl/gpu/common/safe_call.hpp"


namespace util 
{
    template <typename T>
    __global__ void MemsetKernel(T *d_out, T value, int length)
    {
        const int STRIDE = gridDim.x * blockDim.x;

        for (int idx = (blockIdx.x * blockDim.x) + threadIdx.x; idx < length; idx += STRIDE) 
        {
            d_out[idx] = value;
        }
    } 

    /**
    * Manages device storage needed for implementing a global software barrier between CTAs in a single grid
    */
    class GlobalBarrier
    {
    public:

        typedef unsigned int SyncFlag;

    protected :

        // Counters in global device memory
        SyncFlag* d_sync;
        
        //Simple wrapper for returning a CG-loaded SyncFlag at the specified pointer        
        __device__ __forceinline__ SyncFlag LoadCG(SyncFlag* d_ptr) const
        {
            SyncFlag retval;
            util::ld::ModifiedLoad<util::ld::cg>::Ld(retval, d_ptr);
            return retval;
        }

    public:

        GlobalBarrier() : d_sync(0) {}

        
        //Synchronize
        __device__ __forceinline__ void Sync() const
        {
            // Threadfence and syncthreads to make sure global writes are visible before
            // thread-0 reports in with its sync counter
            __threadfence();
            __syncthreads();

            if (blockIdx.x == 0) 
            {

                // Report in ourselves
                if (threadIdx.x == 0) 
                    d_sync[blockIdx.x] = 1;

                __syncthreads();

                // Wait for everyone else to report in
                for (int peer_block = threadIdx.x; peer_block < gridDim.x; peer_block += blockDim.x) 
                {
                    while (LoadCG(d_sync + peer_block) == 0) 
                        __threadfence_block();
                }

                __syncthreads();

                // Let everyone know it's safe to read their prefix sums
                for (int peer_block = threadIdx.x; peer_block < gridDim.x; peer_block += blockDim.x) 
                {
                    d_sync[peer_block] = 0;
                }

            } 
            else 
            {

                if (threadIdx.x == 0) 
                {
                    // Report in
                    d_sync[blockIdx.x] = 1;

                    // Wait for acknowledgement
                    while (LoadCG(d_sync + blockIdx.x) == 1) 
                        __threadfence_block();
                }

                __syncthreads();
            }
        }
    };


    /**
    * Version of global barrier with storage lifetime management.
    *
    * We can use this in host enactors, and pass the base GlobalBarrier as parameters to kernels.
    */
    class GlobalBarrierLifetime : public GlobalBarrier
    {
    protected:

        // Number of bytes backed by d_sync
        size_t sync_bytes;

    public:
        
        GlobalBarrierLifetime() : GlobalBarrier(), sync_bytes(0) {}


        /**
        * Deallocates and resets the progress counters
        */
        void HostReset()
        {            
            if (d_sync) 
            {
                cudaSafeCall( cudaFree(d_sync) );
                d_sync = 0;
            }
            sync_bytes = 0;            
        }

        virtual ~GlobalBarrierLifetime()
        {
            HostReset();
        }


        /**
        * Sets up the progress counters for the next kernel launch (lazily allocating and initializing them if necessary)
        */
        void Setup(int sweep_grid_size)
        {            
            size_t new_sync_bytes = sweep_grid_size * sizeof(SyncFlag);

            if (new_sync_bytes > sync_bytes) 
            {                
                HostReset();
                                            
                sync_bytes = new_sync_bytes;
                cudaSafeCall( cudaMalloc((void**) &d_sync, sync_bytes) );
                
                // Initialize to zero
                util::MemsetKernel<SyncFlag><<<(sweep_grid_size + 128 - 1) / 128, 128>>>(d_sync, 0, sweep_grid_size);
                cudaSafeCall( cudaThreadSynchronize() );                    
            }         
        }
    };

} // namespace util
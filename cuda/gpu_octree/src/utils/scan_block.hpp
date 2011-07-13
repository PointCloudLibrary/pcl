#pragma once


//template <unsigned int CTA_SIZE, typename T> __device__ void scan_block(T* array)
//{    
//    T val = array[threadIdx.x];
//
//    if (CTA_SIZE >   1) { if(threadIdx.x >=   1) { T tmp = array[threadIdx.x -   1]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (CTA_SIZE >   2) { if(threadIdx.x >=   2) { T tmp = array[threadIdx.x -   2]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (CTA_SIZE >   4) { if(threadIdx.x >=   4) { T tmp = array[threadIdx.x -   4]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (CTA_SIZE >   8) { if(threadIdx.x >=   8) { T tmp = array[threadIdx.x -   8]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (CTA_SIZE >  16) { if(threadIdx.x >=  16) { T tmp = array[threadIdx.x -  16]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (CTA_SIZE >  32) { if(threadIdx.x >=  32) { T tmp = array[threadIdx.x -  32]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (CTA_SIZE >  64) { if(threadIdx.x >=  64) { T tmp = array[threadIdx.x -  64]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (CTA_SIZE > 128) { if(threadIdx.x >= 128) { T tmp = array[threadIdx.x - 128]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (CTA_SIZE > 256) { if(threadIdx.x >= 256) { T tmp = array[threadIdx.x - 256]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }  
//    if (CTA_SIZE > 512) { if(threadIdx.x >= 512) { T tmp = array[threadIdx.x - 512]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }  
//}

//template <typename SharedArray, typename T, typename BinaryFunction> __device__ T scan_block(SharedArray array, T val, BinaryFunction binary_op)
//{
//    array[threadIdx.x] = val;
//
//    __syncthreads();
//
//    // copy to temporary so val and tmp have the same memory space
//    if (blockDim.x >   1) { if(threadIdx.x >=   1) { T tmp = array[threadIdx.x -   1]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >   2) { if(threadIdx.x >=   2) { T tmp = array[threadIdx.x -   2]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >   4) { if(threadIdx.x >=   4) { T tmp = array[threadIdx.x -   4]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >   8) { if(threadIdx.x >=   8) { T tmp = array[threadIdx.x -   8]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >  16) { if(threadIdx.x >=  16) { T tmp = array[threadIdx.x -  16]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >  32) { if(threadIdx.x >=  32) { T tmp = array[threadIdx.x -  32]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >  64) { if(threadIdx.x >=  64) { T tmp = array[threadIdx.x -  64]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x > 128) { if(threadIdx.x >= 128) { T tmp = array[threadIdx.x - 128]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x > 256) { if(threadIdx.x >= 256) { T tmp = array[threadIdx.x - 256]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }  
//    if (blockDim.x > 512) { if(threadIdx.x >= 512) { T tmp = array[threadIdx.x - 512]; val = binary_op(tmp, val); } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }  
//
//    return val;
//}

//template <typename SharedArray, typename T> __device__ T scan_block(SharedArray array, T val)
//{
//    array[threadIdx.x] = val;
//
//    __syncthreads();
//
//    // copy to temporary so val and tmp have the same memory space
//    if (blockDim.x >   1) { if(threadIdx.x >=   1) { T tmp = array[threadIdx.x -   1]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >   2) { if(threadIdx.x >=   2) { T tmp = array[threadIdx.x -   2]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >   4) { if(threadIdx.x >=   4) { T tmp = array[threadIdx.x -   4]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >   8) { if(threadIdx.x >=   8) { T tmp = array[threadIdx.x -   8]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >  16) { if(threadIdx.x >=  16) { T tmp = array[threadIdx.x -  16]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >  32) { if(threadIdx.x >=  32) { T tmp = array[threadIdx.x -  32]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x >  64) { if(threadIdx.x >=  64) { T tmp = array[threadIdx.x -  64]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x > 128) { if(threadIdx.x >= 128) { T tmp = array[threadIdx.x - 128]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }
//    if (blockDim.x > 256) { if(threadIdx.x >= 256) { T tmp = array[threadIdx.x - 256]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }  
//    if (blockDim.x > 512) { if(threadIdx.x >= 512) { T tmp = array[threadIdx.x - 512]; val = tmp + val; } __syncthreads(); array[threadIdx.x] = val; __syncthreads(); }  
//
//    return val;
//}

enum ScanKind { exclusive,  inclusive } ;

template <ScanKind Kind , class T> 
__device__ T scan_warp ( volatile T *ptr , const unsigned int idx = threadIdx.x )
{
    const unsigned int lane = idx & 31; // index of thread in warp (0..31)

    if ( lane >=  1) ptr [idx ] = ptr [idx -  1] + ptr [idx];
    if ( lane >=  2) ptr [idx ] = ptr [idx -  2] + ptr [idx];
    if ( lane >=  4) ptr [idx ] = ptr [idx -  4] + ptr [idx];
    if ( lane >=  8) ptr [idx ] = ptr [idx -  8] + ptr [idx];
    if ( lane >= 16) ptr [idx ] = ptr [idx - 16] + ptr [idx];

    if( Kind == inclusive ) 
        return ptr [idx ];
    else 
        return (lane > 0) ? ptr [idx - 1] : 0;
}

template <ScanKind Kind , class T>
__device__ T scan_block( volatile T *ptr , const unsigned int idx = threadIdx.x )
{        
    const unsigned int lane = idx & 31;
    const unsigned int warpid = idx >> 5;

    // Step 1: Intra - warp scan in each warp
    T val = scan_warp <Kind>( ptr , idx );
    
    __syncthreads ();

    // Step 2: Collect per - warp partial results
    if( lane == 31 ) 
        ptr [ warpid ] = ptr [idx ];    

    __syncthreads ();

    // Step 3: Use 1st warp to scan per - warp results
    if( warpid == 0 ) 
    {
        //#TODO HERE IS A ERROR!!!!
        /*if (lane == 31)
            ptr [ warpid ] = val;*/

        scan_warp<inclusive>( ptr , idx );
    }

    __syncthreads ();

    // Step 4: Accumulate results from Steps 1 and 3
    if ( warpid > 0) 
        val = ptr [warpid -1] + val;

    __syncthreads ();

    // Step 5: Write and return the final result
    ptr[idx] = val;

    __syncthreads ();

    return val ;
}
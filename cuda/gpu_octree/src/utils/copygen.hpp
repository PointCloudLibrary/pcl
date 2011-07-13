#pragma once


template <typename T>
__device__ void CopyKernel(const T* in, T *out, int length)
{
	int STRIDE = gridDim.x * blockDim.x;
	for (int idx = (blockIdx.x * blockDim.x) + threadIdx.x; idx < length; idx += STRIDE) 
    {
		out[idx] = in[idx];
	}
}

template <typename T>
__device__ void GenerateKernel(T* out, int beg, int end)
{
    int length = end - beg;
    int pos = beg;
    
	int STRIDE = blockDim.x;
	for (int idx = threadIdx.x; idx < length; idx += STRIDE, pos += STRIDE) 
    {
		out[idx] = pos + threadIdx.x;
	}
}


template <typename T>
__device__ void GenerateTasksKernel(T* out, int beg, int end, int level)
{
    int length = end - beg;
    int pos = beg;
    
	int STRIDE = blockDim.x;
	for (int idx = threadIdx.x; idx < length; idx += STRIDE, pos += STRIDE) 
    {
		out[idx] = ((pos + threadIdx.x) << 8) + level;
	}
}

template<typename T>
__device__ __forceinline__ void CopyWarpKernel(const T* in, T* out, int length)
{
    int STRIDE = warpSize;
    unsigned int laneId = LaneId();

    for (int idx = laneId; idx < length; idx += STRIDE) 
        out[idx] = in[idx];
}

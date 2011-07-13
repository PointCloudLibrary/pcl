#pragma once

// Returns the warp lane ID of the calling thread
__device__ __forceinline__ unsigned int LaneId()
{
	unsigned int ret;
	asm("mov.u32 %0, %laneid;" : "=r"(ret) );
	return ret;
}
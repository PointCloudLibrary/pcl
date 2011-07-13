#pragma once

struct TaskInfoGlobal
{
    int data;

    __device__ __host__ __forceinline__ TaskInfoGlobal() : data(0) {}
    __device__ __host__ __forceinline__ TaskInfoGlobal(int node_idx, int level) : data((node_idx << 8) + level) {}


    __device__ __host__ __forceinline__ int level() const { return data & 0xFF; }
    __device__ __host__ __forceinline__ int node_idx() const { return data >> 8; }
};

struct TasksGlobal
{
    typedef int LineType;

    LineType *line[2];
    int *count[2];
    int active_selector;  
    
    __device__ __host__ __forceinline__ const int* activeLine()     const { return  line[active_selector]; }
    __device__ __host__ __forceinline__ const int* activeCountPtr() const { return count[active_selector]; }
    

    __device__ __host__ __forceinline__ int* outLine()     { return  line[nextSelector()]; }
    __device__ __host__ __forceinline__ int* outCountPtr() { return count[nextSelector()]; }
    
    __device__ __host__ __forceinline__ void swap() { active_selector = nextSelector(); }
    __device__ __host__ __forceinline__ int nextSelector() const { return (active_selector + 1) & 1; }
};

struct TasksGlobalLifeTime : public TasksGlobal
{    
    size_t allocated_size;
 
    TasksGlobalLifeTime() : allocated_size(0) 
    {
        active_selector = 0;
    }

    TasksGlobalLifeTime(size_t size)
    {
        create(size);
    }

    void create(size_t size)
    {
        active_selector = 0;
        allocated_size = size;
        
        cudaSafeCall( cudaMalloc((void**)&line[0], size * sizeof(TasksGlobal::LineType)) );
        cudaSafeCall( cudaMalloc((void**)&line[1], size * sizeof(TasksGlobal::LineType)) );
        cudaSafeCall( cudaMalloc((void**)&count[0], sizeof(int)) );        
        cudaSafeCall( cudaMalloc((void**)&count[1], sizeof(int)) );

        cudaSafeCall( cudaMemset(count[0], 0, sizeof(int)) );        
        cudaSafeCall( cudaMemset(count[1], 0, sizeof(int)) );        
    }

    ~TasksGlobalLifeTime()
    {
        if ( line[0]) cudaSafeCall( cudaFree( line[0]) );
        if ( line[1]) cudaSafeCall( cudaFree( line[1]) );
        if (count[0]) cudaSafeCall( cudaFree(count[0]) );
        if (count[1]) cudaSafeCall( cudaFree(count[1]) );
    }
};
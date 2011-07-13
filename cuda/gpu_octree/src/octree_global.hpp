#pragma once

#include "pcl/gpu/common/safe_call.hpp"

struct OctreeGlobal
{    
    int *nodes;
    int *codes;
    int *begs;
    int *ends;
   
    int *nodes_num;

    OctreeGlobal() : nodes(0), codes(0), begs(0), ends(0), nodes_num(0) {}
};

struct OctreeGlobalWithBox : public OctreeGlobal
{    
    float3 minp, maxp;    
};

struct OctreeGlobalLifetime : public OctreeGlobalWithBox
{
public:
    size_t allocated_size;

 
    OctreeGlobalLifetime() : allocated_size(0) {}

    OctreeGlobalLifetime(size_t size)
    {
        create(size);        
    }

    void create(size_t size) 
    {   
        allocated_size = size;
        cudaSafeCall( cudaMalloc((void**)&nodes, allocated_size * sizeof(int)) );
        cudaSafeCall( cudaMalloc((void**)&codes, allocated_size * sizeof(int)) );
        cudaSafeCall( cudaMalloc((void**)&begs,  allocated_size * sizeof(int)) );
        cudaSafeCall( cudaMalloc((void**)&ends,  allocated_size * sizeof(int)) );

        cudaSafeCall( cudaMalloc((void**)&nodes_num, sizeof(int))  );        
    }
    ~OctreeGlobalLifetime()
    {
        if (nodes) cudaSafeCall( cudaFree(nodes) );
        if (codes) cudaSafeCall( cudaFree(codes) );
        if (begs)  cudaSafeCall( cudaFree(begs) );
        if (ends)  cudaSafeCall( cudaFree(ends) );

        if (nodes_num) cudaSafeCall( cudaFree(nodes_num) );
    }   
};

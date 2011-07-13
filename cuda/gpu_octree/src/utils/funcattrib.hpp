#pragma once

#include<cstdio>


template<class Func>
void printFuncAttrib(Func& func)
{

    cudaFuncAttributes attrs;
    cudaFuncGetAttributes(&attrs, func);  

    printf("=== Function stats ===\n");
    printf("Name: \n");
    printf("sharedSizeBytes    = %d\n", attrs.sharedSizeBytes);
    printf("constSizeBytes     = %d\n", attrs.constSizeBytes);
    printf("localSizeBytes     = %d\n", attrs.localSizeBytes);
    printf("maxThreadsPerBlock = %d\n", attrs.maxThreadsPerBlock);
    printf("numRegs            = %d\n", attrs.numRegs);
    printf("ptxVersion         = %d\n", attrs.ptxVersion);
    printf("binaryVersion      = %d\n", attrs.binaryVersion);
    printf("\n");
    fflush(stdout); 
}
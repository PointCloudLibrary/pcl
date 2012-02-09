#ifndef CUDA_RUNMULTITREE_H_DEFINED
#define CUDA_RUNMULTITREE_H_DEFINED









void CUDA_runMultiTreePass( int          treeId,
                            const int    W,
                            const int    H,
                            const float  focal,
                            const int    treeHeight,
                            const int    numNodes,
                            const void*  nodes_device,
                            const void*  leaves_device,
                            const void*  depth_in_device,
                            void*        multilabel_device );

void CUDA_runMultiTreePassFG( int          treeId,
                            const int    FGThresh,
                            const int    W,
                            const int    H,
                            const float  focal,
                            const int    treeHeight,
                            const int    numNodes,
                            const void*  nodes_device,
                            const void*  leaves_device,
                            const void*  depth_in_device,
                            void*        multilabel_device );

void CUDA_runMultiTreeMerge( int          numTrees,
                             const int    W,
                             const int    H,
                             const void*  depth_in_device, 
                             void*        multilabel_device, 
                             void*        label_out_device);


#endif

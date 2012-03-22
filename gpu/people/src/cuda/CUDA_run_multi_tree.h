/**
 * @authors: Cedric Cagniart, Koen Buys
 **/
#ifndef PCL_PEOPLE_TREES_CUDA_RUNMULTITREE_H_
#define PCL_PEOPLE_TREES_CUDA_RUNMULTITREE_H_

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

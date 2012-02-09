#ifndef CUDA_RUNTREE_H_DEFINED
#define CUDA_RUNTREE_H_DEFINED









void CUDA_runTree( const int    W,
				   const int    H,
				   const float  focal,
				   const int    treeHeight,
				   const int    numNodes,
				   const void*  nodes_device,
				   const void*  leaves_device,
				   const void*  depth_in_device,
				   void*        label_out_device );


void CUDA_runTree_masked( const int    W,
						  const int    H,
						  const float  focal,
						  const int    treeHeight,
						  const int    numNodes,
						  const void*  nodes_device,
						  const void*  leaves_device,
						  const void*  depth_in_device,
						  const void*  mask_in_device,
						  void*        label_out_device );



#endif

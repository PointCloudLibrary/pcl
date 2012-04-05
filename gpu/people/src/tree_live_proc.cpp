/** @authors: Cedric Cagniart, Koen Buys */

#include <pcl/gpu/people/trees/tree_run.h>
#include <pcl/gpu/people/trees/tree_live.h>
#include <cuda_runtime_api.h>

#include "cuda/CUDA_run_tree.h"
#include <pcl/gpu/people/trees/CUDA_tree.h>

using pcl::gpu::people::trees::Node;
using pcl::gpu::people::trees::Label;
using pcl::gpu::people::trees::loadTree;
using pcl::gpu::people::trees::focal;

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace trees
      {
        TreeLiveProc::TreeLiveProc(std::istream& is) {
          // load the tree file
          std::vector<Node>  nodes;
          std::vector<Label> leaves;

          // this might throw but we haven't done any malloc yet
          int height = loadTree(is, nodes, leaves );  
          m_tree = new CUDATree(height, nodes, leaves);

          // alloc cuda
          cudaMalloc(&m_dmap_device, 640*480*sizeof(uint16_t));
          cudaMalloc(&m_lmap_device, 640*480*sizeof(uint8_t));
        }

        TreeLiveProc::~TreeLiveProc() {
          delete m_tree;
          cudaFree(m_dmap_device);
          cudaFree(m_lmap_device);
        }

        #define TLIVEEXCEPT(msg) \
          throw std::runtime_error(msg);

        void TreeLiveProc::process (const cv::Mat& dmap,
                                    cv::Mat&       lmap )
        {
          if( dmap.depth()       != CV_16U ) TLIVEEXCEPT("depth has incorrect channel type")
          if( dmap.channels()    != 1 )      TLIVEEXCEPT("depth has incorrect channel count")
          if( dmap.size().width  != 640 )    TLIVEEXCEPT("depth has incorrect width")
          if( dmap.size().height != 480 )    TLIVEEXCEPT("depth has incorrect height")
          if( !dmap.isContinuous() )         TLIVEEXCEPT("depth has non contiguous rows")

          // alloc the buffer if it isn't done yet
          lmap.create( 480, 640, CV_8UC(1) );

          // copy depth to cuda
          cudaMemcpy(m_dmap_device, (const void*) dmap.data, 
                                    640*480*sizeof(uint16_t), cudaMemcpyHostToDevice);
          // process the dmap
          CUDA_runTree( 640,480, focal, 
                        m_tree->treeHeight(), 
                        m_tree->numNodes(), 
                        m_tree->nodes_device(), 
                        m_tree->leaves_device(),
                        m_dmap_device, 
                        m_lmap_device );
          // download back from cuda
          cudaMemcpy((Label*)(lmap.data), m_lmap_device,
                                       640*480*sizeof(Label), cudaMemcpyDeviceToHost);
        }
        #undef TLIVEEXCEPT
      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl


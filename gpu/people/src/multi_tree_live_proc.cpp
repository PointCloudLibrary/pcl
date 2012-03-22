/**
 * @authors: Cedric Cagniart, Koen Buys
 */
#include <pcl/people/trees/tree_run.h>
#include <pcl/people/trees/handle_error.h>
#include <pcl/people/trees/tree_live.h>

#include <cuda_runtime_api.h>
#include "cuda/CUDA_run_multi_tree.h"
#include "CUDA_tree.h"

#include <iostream>

using pcl::people::trees::Node;
using pcl::people::trees::Label;
using pcl::people::trees::loadTree;
using pcl::people::trees::focal;

namespace pcl
{
  namespace people
  {
    namespace trees
      {
        MultiTreeLiveProc::MultiTreeLiveProc(std::istream& is)
        {
          // load the tree file
          std::vector<Node>  nodes;
          std::vector<Label> leaves;
          // this might throw but we haven't done any malloc yet
          int height = loadTree(is, nodes, leaves );  
          CUDATree* tree = new CUDATree(height, nodes, leaves);
          m_trees.push_back(tree);

          // alloc cuda
          HANDLE_ERROR( cudaMalloc(&m_dmap_device,      640*480*sizeof(uint16_t)));
          HANDLE_ERROR( cudaMalloc(&m_multilmap_device, 640*480*4*sizeof(uint8_t)));
          HANDLE_ERROR( cudaMalloc(&m_lmap_device,      640*480*sizeof(uint8_t)));
        }

        MultiTreeLiveProc::~MultiTreeLiveProc() 
        {
          for( std::vector<CUDATree*>::iterator t_itr = m_trees.begin(); 
                                                   t_itr != m_trees.end(); t_itr ++ ){
            delete *t_itr;
          }

          cudaFree(m_dmap_device);
          cudaFree(m_multilmap_device);
          cudaFree(m_lmap_device);
        }

        #define TLIVEEXCEPT(msg) \
          throw std::runtime_error(msg);

        void MultiTreeLiveProc::addTree(std::istream& is)
        {
          if( m_trees.size() >= 4 ) TLIVEEXCEPT("there are already 4 trees in this Processor")

          std::vector<Node>  nodes;
          std::vector<Label> leaves;
          // this might throw but we haven't done any malloc yet
          int height = loadTree(is, nodes, leaves );  
          CUDATree* tree = new CUDATree(height, nodes, leaves);
          m_trees.push_back(tree);
        }

        void MultiTreeLiveProc::process(const cv::Mat& dmap,
                                        cv::Mat&       lmap )
        {
          process( dmap, lmap, std::numeric_limits<pcl::people::trees::Attrib>::max() );
        }

        void MultiTreeLiveProc::process(const cv::Mat& dmap,
                                        cv::Mat&       lmap,
                                        int        FGThresh)
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

          // 1 - run the multi passes
          int numTrees = m_trees.size();
          for(int ti=0; ti<numTrees; ++ti ) {
            CUDATree* t = m_trees[ti];
            if( FGThresh == std::numeric_limits<pcl::people::trees::Attrib>::max() ) {
              CUDA_runMultiTreePass( ti, 640,480, focal,  
                                                       t->treeHeight(), t->numNodes(),
                                                t->nodes_device(), t->leaves_device(),
                                                  m_dmap_device, m_multilmap_device );
            }
            else {
              CUDA_runMultiTreePassFG( ti, FGThresh, 640,480, focal,  
                                                       t->treeHeight(), t->numNodes(),
                                                t->nodes_device(), t->leaves_device(),
                                                  m_dmap_device, m_multilmap_device );
            }
          }
          // 2 - run the merging 
          CUDA_runMultiTreeMerge(m_trees.size(), 640,480, 
                                      m_dmap_device, m_multilmap_device, m_lmap_device);
          // 3 - download back from cuda ( the copy will execute all the kernels at once)
          cudaMemcpy((Label*)(lmap.data), m_lmap_device,
                                       640*480*sizeof(Label), cudaMemcpyDeviceToHost);
        }
        #undef TLIVEEXCEPT
    } // end namespace trees
  } // end namespace people
} // end namespace PCL



/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: $
 * @authors: Cedric Cagniart, Koen Buys
 *
 */

#include <pcl/gpu/people/tree_run.h>
#include <pcl/gpu/people/handle_error.h>
#include <pcl/gpu/people/tree_live.h>

#include <pcl/gpu/people/label_common.h>

#include <pcl/gpu/containers/device_array.h>

#include <cuda_runtime_api.h>
#include "cuda/CUDA_run_multi_tree.h"
#include <pcl/gpu/people/CUDA_tree.h>

#include <iostream>

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
        MultiTreeLiveProc::MultiTreeLiveProc(std::istream& is)
        {
          // load the tree file
          std::vector<Node>  nodes;
          std::vector<Label> leaves;

          // this might throw but we haven't done any malloc yet
          int height = loadTree (is, nodes, leaves );  
          CUDATree* tree = new CUDATree (height, nodes, leaves);
          m_trees.push_back (tree);

          // alloc cuda
          HANDLE_ERROR( cudaMalloc(&m_dmap_device,      RES_W * RES_H * sizeof(uint16_t)));
          HANDLE_ERROR( cudaMalloc(&m_multilmap_device, RES_W * RES_H * MAX_NR_TREES * sizeof(uint8_t)));
          HANDLE_ERROR( cudaMalloc(&m_lmap_device,      RES_W * RES_H * sizeof(uint8_t)));
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

        void MultiTreeLiveProc::addTree(std::istream& is)
        {
          if( m_trees.size() >= MAX_NR_TREES )
            std::cout << "There are already the max amount of trees in this Processor" << std::endl;

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
          process( dmap, lmap, std::numeric_limits<pcl::gpu::people::trees::Attrib>::max() );
        }

        void MultiTreeLiveProc::process(const cv::Mat& dmap,
                                        cv::Mat&       lmap,
                                        int        FGThresh)
        {
          if( dmap.depth()       != CV_16U ) std::cerr << "depth has incorrect channel type" << std::endl;
          if( dmap.channels()    != 1 )      std::cerr << "depth has incorrect channel count" << std::endl;
          if( dmap.size().width  != 640 )    std::cerr << "depth has incorrect width" << std::endl;
          if( dmap.size().height != 480 )    std::cerr << "depth has incorrect height" << std::endl;
          if( !dmap.isContinuous() )         std::cerr << "depth has non contiguous rows" << std::endl;

          // alloc the buffer if it isn't done yet
          lmap.create( 480, 640, CV_8UC(1) );

          // copy depth to cuda
          cudaMemcpy(m_dmap_device, (const void*) dmap.data, 
                                    640*480*sizeof(uint16_t), cudaMemcpyHostToDevice);

          // 1 - run the multi passes
          int numTrees = m_trees.size();
          for(int ti=0; ti<numTrees; ++ti ) {
            CUDATree* t = m_trees[ti];
            if( FGThresh == std::numeric_limits<pcl::gpu::people::trees::Attrib>::max() ) {
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
/*
        void MultiTreeLiveProc::process(const pcl::gpu::DeviceArray<unsigned short>&  DepthMap,
                                        pcl::gpu::DeviceArray<unsigned char>&         LabelMap,
                                        int                                           FGThresh)
        {
          // 1 - run the multi passes
          int numTrees = m_trees.size ();
          for(int ti=0; ti<numTrees; ++ti ) 
          {
            CUDATree* t = m_trees[ti];
            if( FGThresh == std::numeric_limits<pcl::people::trees::Attrib>::max () ) 
            {
              CUDA_runMultiTreePass ( ti, 640,480, focal,  
                                      t->treeHeight (), t->numNodes (),
                                      t->nodes_device (), t->leaves_device (),
                                      DepthMap, m_multilmap_device );
            }
            else 
            {
              CUDA_runMultiTreePassFG( ti, FGThresh, 640,480, focal,  
                                       t->treeHeight (), t->numNodes (),
                                       t->nodes_device (), t->leaves_device (),
                                       DepthMap, m_multilmap_device );
            }
          }
          // 2 - run the merging 
          CUDA_runMultiTreeMerge(m_trees.size (), 640,480, 
                                 m_dmap_device, m_multilmap_device, LabelMap);
        }
*/
      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl



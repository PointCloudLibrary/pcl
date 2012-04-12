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

#include <pcl/gpu/people/tree.h>

#include <pcl/gpu/people/label_common.h>
#include <pcl/gpu/containers/device_array.h>

#include <pcl/console/print.h>

#include "tree_live.h"
#include "cuda/CUDA_run_multi_tree.h"
#include "CUDA_tree.h"
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
          m_trees.push_back(CUDATree(height, nodes, leaves));

          const int cols = WIDTH; // default buffer sizes,
          const int rows = HEIGHT; // may change during 'process' method

          //preallocate buffers
          m_dmap_device.create(cols * rows);
          m_lmap_device.create(cols * rows);
          m_multilmap_device.create(cols * rows * MAX_NR_TREES);
        }

        MultiTreeLiveProc::~MultiTreeLiveProc()  {}

        void MultiTreeLiveProc::addTree(std::istream& is)
        {
          if( m_trees.size() >= MAX_NR_TREES )
          {
            PCL_INFO("There are already the max amount of trees in this Processor");
            return;
          }
          std::vector<Node>  nodes;
          std::vector<Label> leaves;
          // this might throw but we haven't done any malloc yet
          int height = loadTree(is, nodes, leaves );
          m_trees.push_back(CUDATree(height, nodes, leaves));
        }

        void MultiTreeLiveProc::process(const cv::Mat& dmap, cv::Mat& lmap )
        {
          process( dmap, lmap, std::numeric_limits<Attrib>::max() );
        }

        void MultiTreeLiveProc::process(const cv::Mat& dmap, cv::Mat& lmap, int FGThresh)
        {
          m_dmap_device.create(dmap.cols * dmap.rows);
          m_lmap_device.create(dmap.cols * dmap.rows);
          m_multilmap_device.create(dmap.cols * dmap.rows * MAX_NR_TREES);

          // alloc the buffer if it isn't done yet
          lmap.create( dmap.size(), CV_8U );

          // copy depth to cuda
          m_dmap_device.upload((Depth*)dmap.data, dmap.cols * dmap.rows);

          // 1 - run the multi passes
          int numTrees = m_trees.size();
          for( int ti = 0; ti < numTrees; ++ti ) 
          {
            const CUDATree& t = m_trees[ti];

            if( FGThresh == std::numeric_limits<Attrib>::max() ) 
            {
              CUDA_runMultiTreePass( ti, dmap.cols, dmap.rows, (float)focal, t.treeHeight, t.numNodes, 
                  t.nodes_device, t.leaves_device, m_dmap_device, m_multilmap_device );
            }
            else 
            {
              CUDA_runMultiTreePassFG( ti, FGThresh, dmap.cols, dmap.rows, (float)focal, t.treeHeight, t.numNodes, 
                  t.nodes_device, t.leaves_device, m_dmap_device, m_multilmap_device );
            }
          }
          // 2 - run the merging 
          CUDA_runMultiTreeMerge(m_trees.size(), dmap.cols, dmap.rows, m_dmap_device, m_multilmap_device, m_lmap_device);

          // 3 - download back from cuda ( the copy will execute all the kernels at once)
          m_lmap_device.download((Label*)lmap.data);
        }


      } // end namespace trees
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl



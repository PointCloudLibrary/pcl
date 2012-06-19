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
 * @authors: Cedric Cagniart, Koen Buys, Anatoly Baksheev
 *
 */

#include <pcl/gpu/people/tree.h>
#include <pcl/gpu/people/label_common.h>
#include <pcl/gpu/utils/device/limits.hpp>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/utils/texture_binder.hpp>
#include <stdio.h>
#include <limits>
#include <assert.h>
#include "internal.h"

using pcl::gpu::people::trees::Node;
using pcl::gpu::people::trees::Label;
using pcl::gpu::people::trees::AttribLocation;
using pcl::gpu::people::trees::Attrib;
using pcl::gpu::people::trees::focal;
using pcl::gpu::people::trees::NUM_LABELS;

using namespace std;
typedef unsigned int uint;

#ifdef __CDT_PARSER__ // This is an eclipse specific hack, does nothing to the code
#define __global__
#define __device__
#define __shared__
#define __forceinline__
#define __constant__
#endif

namespace pcl
{
  namespace device
  {
    texture<unsigned short, 2, cudaReadModeElementType> depthTex;
    texture<char4, 2, cudaReadModeElementType> multilabelTex;

    __constant__ int constFGThresh;

    template<bool testFG> __device__ __forceinline__ Label 
    evaluateTree(int u, int v, float f, int treeHeight, int numNodes, const Node* nodes, const Label* leaves)
    {
      int    depth = tex2D(depthTex, u, v);

      float  scale = f / depth;

      // go down the tree
      int nid = 0;
      for(int nodeDepth = 0; nodeDepth < treeHeight; ++nodeDepth)
      {
        const Node node = nodes[nid];

        const AttribLocation& loc = node.loc;
        int d1 = tex2D (depthTex, u + loc.du1 * scale, v + loc.dv1 * scale);
        int d2 = tex2D (depthTex, u + loc.du2 * scale, v + loc.dv2 * scale);

        if (testFG)
        {
          if( d1 - depth > constFGThresh ) 
            d1 = numeric_limits<short>::max();

          if( d2 - depth > constFGThresh ) 
            d2 = numeric_limits<short>::max();
        }

        int delta = d1-d2;
        bool test = delta > (int)node.thresh;
        if( test ) nid = nid*2+2;
        else       nid = nid*2+1;
      }
      return leaves[nid-numNodes];
    }

    /** \brief This is the CUDA kernel doing the actual RDF evaluation */    
    __global__ void
    KernelCUDA_runTree( const float     f,
                        const int       treeHeight,
                        const int       numNodes,
                        const Node*     nodes,
                        const Label*    leaves,
                        PtrStepSz<Label>  labels)
    {
      int u = blockIdx.x * blockDim.x + threadIdx.x;
      int v = blockIdx.y * blockDim.y + threadIdx.y;

      if( u < labels.cols && v < labels.rows)
        labels.ptr(v)[u] = evaluateTree<false>(u, v, f, treeHeight, numNodes, nodes, leaves);
    }

    template<bool testFG> __global__ void
    KernelCUDA_MultiTreePass( const int    treeId,
                              const float  f,
                              const int    treeHeight,
                              const int    numNodes,
                              const Node*  nodes,
                              const Label* leaves,
                              PtrStepSz<unsigned short> depth,
                              PtrStepSz<char4> multiLabels)
    {
      int u = blockIdx.x * blockDim.x + threadIdx.x;
      int v = blockIdx.y * blockDim.y + threadIdx.y;

      if(u < multiLabels.cols && v < multiLabels.rows)
      {
        // This maps a char4 pointer on a char pointer
        char* pixel = (char*)&multiLabels.ptr(v)[u];
        // This test assures that in next iterations the FGPreperation is taking into account see utils.cu
        if(depth.ptr(v)[u] == numeric_limits<unsigned short>::max())
          pixel[treeId] = 29;         // see label_common.h for Background label (=29)
                                      // TODO remove this hardcoded label with enum part_t label
        else
          pixel[treeId] = evaluateTree<testFG>(u, v, f, treeHeight, numNodes, nodes, leaves);
      }
    }

    /** \brief This function wraps the actual CUDA kernel doing the RDF evaluation */    
    void CUDA_runTree ( float focal, int treeHeight, int numNodes, const Node* nodes, const Label* leaves, const Depth& depth, Labels& labels )
    {
      labels.create( depth.rows(), depth.cols() );

      depthTex.addressMode[0] = cudaAddressModeClamp;
      TextureBinder binder(depth, depthTex);      

      dim3 block(32, 8);      
      dim3 grid(divUp(depth.cols(), block.x), divUp(depth.rows(), block.y) );      

      KernelCUDA_runTree<<< grid, block >>>( focal, treeHeight, numNodes, nodes, leaves, labels);
      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );      
    }

    void CUDA_runMultiTreePass ( int   FGThresh,
                                 int   treeId,
                                 float focal,
                                 int   treeHeight,
                                 int   numNodes,
                                 const Node*  nodes_device,
                                 const Label* leaves_device,
                                 const Depth& depth,
                                 MultiLabels& multilabel )
    {
      //std::cout << "(I) : CUDA_runMultiTreePass() called" << std::endl;
      depthTex.addressMode[0] = cudaAddressModeClamp;
      TextureBinder binder(depth, depthTex);                  

      dim3 block(32, 8);
      dim3 grid( divUp(depth.cols(), block.x), divUp(depth.rows(), block.y) );

      if(FGThresh == std::numeric_limits<int>::max())
      {
        KernelCUDA_MultiTreePass<false><<< grid, block >>>( treeId, focal, treeHeight, 
            numNodes, nodes_device, leaves_device, depth, multilabel);
      }
      else
      {
        cudaSafeCall( cudaMemcpyToSymbol(constFGThresh, &FGThresh,  sizeof(FGThresh)) );

        KernelCUDA_MultiTreePass<true><<< grid, block >>>( treeId, focal, treeHeight, 
            numNodes, nodes_device, leaves_device, depth, multilabel);
      }

      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );      
    }

    ///////////////////////////////////////////////////////////////////////////////////////

    __device__ int findMaxId( int numBins, char* bins ) 
    {
      // HACK .. not testing against numBins = 0
      int maxId   = 0;
      char maxVal = bins[0];
      for(int i=1;i<numBins;++i) 
      {
        char val = bins[i];
        if( val > maxVal ) { maxId = i; maxVal = val; }
      }
      return maxId;
    }

    //this will find the max Index but return -1 if there is a tie
    __device__ int findMaxId_testTie(int numBins, char* bins) 
    {
      int maxId = 0;
      int maxId_other = -1;
      char maxVal = bins[0];

      for(int i=1;i<numBins;++i) {
        char val = bins[i];
        if( val == maxVal ) { maxId_other = i; }
        if( val > maxVal ) { maxId = i; maxId_other = -1; maxVal = val; }
      }

      if( maxId_other != -1) return -1;
      else                   return maxId;
    }

    __global__ void KernelCUDA_MultiTreeMerge( const int numTrees, PtrStepSz<Label> labels )
    {
      int u = blockIdx.x * blockDim.x + threadIdx.x;
      int v = blockIdx.y * blockDim.y + threadIdx.y;

      if( u >= labels.cols || v >= labels.rows) 
        return;

      // reset the bins
      char bins[NUM_LABELS];
      for(int li = 0; li < NUM_LABELS; ++li) 
        bins[li] = 0;

      // find a consensus with the current trees
      {
        char4 pixlabels = tex2D(multilabelTex, u ,v);
        char* bob = (char*)&pixlabels; //horrible but char4's have xyzw members
        for(int ti = 0; ti < numTrees; ++ti)
          bins[ bob[ti] ]++;        
      }

      int res = findMaxId_testTie(NUM_LABELS, bins);

      // if this fails... find a consensus in a 1 neighbourhood
      if( res < 0 ) 
      {
        int depth = tex2D(depthTex, u,v);
        for(int i = -1 ; i <= 1; ++i) 
        {
          for(int j = -1; j <= 1; ++j) 
          {
            int   depth_neighbor  = tex2D(depthTex,u+i,v+j);
            char4 labels_neighbor = tex2D(multilabelTex, u+i,v+j); 
            char* bob = (char*)&labels_neighbor; //horrible but char4's have xyzw members
            //TODO: redo this part
            int weight = abs(depth-depth_neighbor) < 50 ? 1:0; // 5cms
            for(int ti = 0; ti < numTrees; ++ti)
              bins[ bob[ti] ] += weight;
          }
        }
        res = findMaxId( NUM_LABELS, bins );
      }      
      labels.ptr(v)[u] = res;
    }

    /** \brief This merges the labels from all trees into a histogram of probabilities **/
    __global__ void KernelCUDA_MultiTreeCreateProb (const int numTrees, PtrStepSz<prob_histogram> prob)
    {
      // map block and thread onto image coordinates
      int u = blockIdx.x * blockDim.x + threadIdx.x;
      int v = blockIdx.y * blockDim.y + threadIdx.y;

      if( u >= prob.cols || v >= prob.rows )
        return;

      char4 pixlabels = tex2D (multilabelTex, u ,v);
      char* bob = (char*)&pixlabels; //horrible but char4's have xyzw members

      // Reset prob first, this should become NUM_LABELS
      for(int in = 0; in < NUM_LABELS; in++)
      {
        prob.ptr(v)[u].probs[in] = 0;
      }

      for(int ti = 0; ti < numTrees; ++ti)
      {
        // Each tree casts a vote to the probability
        // TODO: replace this with a histogram copy
        prob.ptr(v)[u].probs[bob[ti]] += 0.25;
      }
    }

    /** \brief This will merge the votes from the different trees into one final vote */    
    void CUDA_runMultiTreeMerge( int numTrees, const Depth& depth, const MultiLabels& multilabel, Labels& labels)
    {     
      //std::cout << "(I) : CUDA_runMultiTreeMerge() called" << std::endl;
      labels.create(depth.rows(), depth.cols());

      depthTex.addressMode[0] = cudaAddressModeClamp;
      TextureBinder binder(depth, depthTex);                  

      multilabelTex.addressMode[0] = cudaAddressModeClamp;
      TextureBinder mlabels_binder(multilabel, multilabelTex);      

      dim3 block(32, 8);      
      dim3 grid(divUp(depth.cols(), block.x), divUp(depth.rows(), block.y) );      

      KernelCUDA_MultiTreeMerge<<< grid, block >>>( numTrees, labels );

      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );            
    }

    /** \brief This will merge the votes from the different trees into one final vote, including probabilistic's */
    void CUDA_runMultiTreeProb ( int numTrees,
                                 const Depth& depth,
                                 const MultiLabels& multilabel,
                                 Labels& labels,
                                 LabelProbability& probabilities)
    {
      std::cout << "(I) : CUDA_runMultiTreeProb() called" << std::endl;

      //labels.create(depth.rows(), depth.cols());
      //depthTex.addressMode[0] = cudaAddressModeClamp;
      //TextureBinder binder(depth, depthTex);

      multilabelTex.addressMode[0] = cudaAddressModeClamp;
      TextureBinder mlabels_binder(multilabel, multilabelTex);

      dim3 block(32, 8);      
      dim3 grid(divUp(depth.cols(), block.x), divUp(depth.rows(), block.y) );      

      KernelCUDA_MultiTreeCreateProb<<< grid, block >>>( numTrees, probabilities);

      cudaSafeCall( cudaGetLastError() );
      cudaSafeCall( cudaThreadSynchronize() );            
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::device::CUDATree::CUDATree (int treeHeight_arg, const vector<Node>& nodes, const vector<Label>& leaves)
{
  treeHeight = treeHeight_arg;
  numNodes = (1 << treeHeight) - 1;
  assert (static_cast<int> (nodes.size ())  == numNodes );
  assert (static_cast<int> (leaves.size ()) == (1 << treeHeight) );

  nodes_device.upload(nodes);
  leaves_device.upload(leaves);
}

void
pcl::device::MultiTreeLiveProc::process (const Depth& dmap, Labels& lmap)
{
  // TODO: is this assert needed if we only call process?
  //assert(!trees.empty());

  // TODO is this iteration needed when we call multitreepass in the process step?
  /*  if (trees.size() == 1)
  {
    const CUDATree& t = trees[0];
    CUDA_runTree( focal, t.treeHeight, t.numNodes, t.nodes_device, t.leaves_device, dmap,  lmap );
    return;
  }
   */
  process(dmap, lmap, std::numeric_limits<int>::max());
}

void
pcl::device::MultiTreeLiveProc::process (const Depth& dmap, Labels& lmap, int FGThresh)
{
  assert(!trees.empty());

  unsigned int numTrees = static_cast<int> (trees.size ());

  multilmap.create(dmap.rows(), dmap.cols());

  // 1 - run the multi passes  
  for( int ti = 0; ti < numTrees; ++ti ) 
  {
    const CUDATree& t = trees[ti];

    CUDA_runMultiTreePass ( FGThresh, ti, static_cast<float> (focal), t.treeHeight, t.numNodes, t.nodes_device, t.leaves_device, dmap, multilmap );    
  }
  // 2 - run the merging 
  assert( numTrees <= 4 );

  device::CUDA_runMultiTreeMerge(numTrees, dmap, multilmap, lmap);
}

void
pcl::device::MultiTreeLiveProc::processProb (const Depth& dmap, Labels& lmap, LabelProbability& prob, int FGThresh)
{
  assert(!trees.empty());

  unsigned int numTrees = static_cast<unsigned int> (trees.size ());
  assert( numTrees <= 4 );

  multilmap.create(dmap.rows(), dmap.cols());

  // 1 - run the multi passes
  for( int ti = 0; ti < numTrees; ++ti )
  {
    const CUDATree& t = trees[ti];
    CUDA_runMultiTreePass ( FGThresh, ti, static_cast<float> (focal), t.treeHeight, t.numNodes, t.nodes_device, t.leaves_device, dmap, multilmap );
  }

  device::CUDA_runMultiTreeProb(numTrees, dmap, multilmap, lmap, prob);
}

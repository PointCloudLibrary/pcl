/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * @author: Koen Buys, Anatoly Baksheev
 */

#include <pcl/gpu/people/bodyparts_detector.h>
#include <cassert>
#include <pcl/common/time.h>
#include "internal.h"

using namespace std;

const int MAX_CLUST_SIZE = 25000;
const float CLUST_TOL = 0.05f;

pcl::gpu::people::RDFBodyPartsDetector::RDFBodyPartsDetector( const vector<string>& tree_files, 
                                                              int rows, 
                                                              int cols)
    : labels_(rows, cols), 
      labels_smoothed_(rows, cols), 
      max_cluster_size_(MAX_CLUST_SIZE), 
      cluster_tolerance_(CLUST_TOL)
{
  //TODO replace all asserts with exceptions
  assert(!tree_files.empty());

  impl_.reset( new device::MultiTreeLiveProc(rows, cols) );

  for(size_t i = 0; i < tree_files.size(); ++i)
  {
    // load the tree file
    vector<trees::Node>  nodes;
    vector<trees::Label> leaves;

    // this might throw but we haven't done any malloc yet
    int height = loadTree (tree_files[i], nodes, leaves );
    impl_->trees.push_back(device::CUDATree(height, nodes, leaves));
  }

  vector<pcl::RGB> rgba(LUT_COLOR_LABEL_LENGTH);

  for(int i = 0; i < LUT_COLOR_LABEL_LENGTH; ++i)
  {
      // !!!! generate in RGB format, not BGR
      rgba[i].r = LUT_COLOR_LABEL[i*3 + 2]; 
      rgba[i].g = LUT_COLOR_LABEL[i*3 + 1];
      rgba[i].b = LUT_COLOR_LABEL[i*3 + 0];
      rgba[i].a = 255;
  }
  color_map_.upload(rgba);

  allocate_buffers();
}

size_t 
pcl::gpu::people::RDFBodyPartsDetector::treesNumber() const
{
  return impl_->trees.size();
}

const pcl::gpu::people::RDFBodyPartsDetector::Labels& 
pcl::gpu::people::RDFBodyPartsDetector::getLabels() const
{
  return labels_smoothed_;
}

void 
pcl::gpu::people::RDFBodyPartsDetector::colorizeLabels(const Labels& labels, Image& color_labels) const
{  
  color_labels.create(labels.rows(), labels.cols());

  const DeviceArray<uchar4>& map = (const DeviceArray<uchar4>&)color_map_;
  device::Image& img = (device::Image&)color_labels;
  device::colorLMap(labels, map, img);
}

void 
pcl::gpu::people::RDFBodyPartsDetector::computeLabels(const Depth& depth)
{
  allocate_buffers(depth.rows(), depth.cols());
  // Process the depthimage (CUDA)
  impl_->process(depth, labels_);
  device::smoothLabelImage(labels_, depth, labels_smoothed_, NUM_PARTS, 5, 300);
}

void 
pcl::gpu::people::RDFBodyPartsDetector::allocate_buffers(int rows, int cols)
{
    lmap_host_.resize(rows * cols);
    
    dst_labels_.resize(rows * cols);
    region_sizes_.resize(rows*cols);
    wavefront_.resize(rows*cols*2);                  
}

//////////////////////////////////////////////////////////////////////
///////////////////// in development (dirty) /////////////////////////

#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>
#include "cuda.h"
#include "vector_functions.h"

void 
pcl::gpu::people::RDFBodyPartsDetector::step2_selectBetterName(const PointCloud<PointXYZ>& cloud, int min_pts_per_cluster, BlobMatrix& sorted)
{
  int cols = labels_smoothed_.cols();
  int rows = labels_smoothed_.rows();
  
  int c;  
  labels_smoothed_.download(lmap_host_, c);
  
  // Create a new struct to put the results in    
  sorted.resize(NUM_PARTS);
  for(size_t i = 0; i < sorted.size(); ++i)
  {
      sorted[i].clear();
      sorted[i].reserve(1000);
  }

  vector< vector< vector<int> > > cluster_indices(NUM_PARTS);
  {
     //ScopeTime time("elec4");     
     optimized_elec4(cloud, NUM_PARTS);
     
     int *rsizes = &region_sizes_[0];
     int *remap = &wavefront_[0]; // sure that size is enought
     std::fill(remap, remap + cols * rows, -1);   

     std::vector<float3> means(cols * rows, make_float3(0, 0, 0));
     for(size_t k = 0; k < dst_labels_.size(); ++k)
     {
       int cc = dst_labels_[k];
       if (cc != -1 && min_pts_per_cluster <= rsizes[cc] && rsizes[cc] <= max_cluster_size_)
       {
         const PointXYZ& p = cloud.points[k];
         means[cc].x += p.x;
         means[cc].y += p.y;
         means[cc].z += p.z;
        }
     }

     for(size_t k = 0; k < dst_labels_.size(); ++k)
     {       
       int label = lmap_host_[k];
       int cc    = dst_labels_[k];
       
       if (cc != -1 && means[cc].z != 0)
       {
         int ccindex = remap[cc];
         if (ccindex == -1)
         {
           ccindex = (int)sorted[label].size();
           sorted[label].resize(ccindex + 1);
           remap[cc] = ccindex;

           sorted[label][ccindex].label = (part_t)label;
           sorted[label][ccindex].mean.coeffRef(0) = means[cc].x/rsizes[cc];
           sorted[label][ccindex].mean.coeffRef(1) = means[cc].y/rsizes[cc];
           sorted[label][ccindex].mean.coeffRef(2) = means[cc].z/rsizes[cc];
         }                 
         sorted[label][ccindex].indices.indices.push_back(k);
       }                           
     }

     int id = 0;
     for(size_t label = 0; label < sorted.size(); ++label)            
       for(size_t b = 0; b < sorted[label].size(); ++b)
       {         
         sorted[label][b].id = id++;                                
         sorted[label][b].lid = (int)b;                        
       }     
  }  //ScopeTime time("elec4");     
  

  label_skeleton::buildRelations ( sorted );  
}

static float sqnorm(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
    float dx = (p1.x - p2.x);
    float dy = (p1.y - p2.y);
    float dz = (p1.z - p2.z);
    return dx*dx + dy*dy + dz*dz;    
}

void pcl::gpu::people::RDFBodyPartsDetector::optimized_elec4(const PointCloud<pcl::PointXYZ>& cloud, int num_parts)
{      
  int width = cloud.width;
  int height = cloud.height;    
    
  std::fill(dst_labels_.begin(), dst_labels_.end(), -1);    
  int2 *wf = (int2*)&wavefront_[0];
  int *rsizes = &region_sizes_[0];       

  int cc = -1;

  float squared_radius = cluster_tolerance_ * cluster_tolerance_;

  for(int j = 0; j < height; ++j)
  {
    for(int i = 0; i < width; ++i)
    {           
      if (lmap_host_[j * width + i] >= num_parts || dst_labels_[j * width + i] != -1) // invalid label && has been labeled
        continue;

      int2* ws = wf; // initialize wavefront
      int2 p = make_int2(i, j);  // current pixel

      cc++;	// next label

      dst_labels_[j * width + i] = cc;            
      int count = 0;	// current region size

      // wavefront propagation
      while( ws >= wf ) // wavefront not empty
      {
        // put neighbors onto wavefront
        const unsigned char* sl = &lmap_host_[p.y * width + p.x];
        int*                 dl = &dst_labels_[p.y * width + p.x];
        const pcl::PointXYZ *sp = &cloud.points[p.y * width + p.x];

        //right
        if( p.x < width-1 && dl[+1] == -1 && sl[+1] == sl[0])
          if (sqnorm(sp[0], sp[+1]) <= squared_radius)
          {
            dl[+1] = cc;
            *ws++ = make_int2(p.x+1, p.y);
          }

        //left
        if( p.x > 0 && dl[-1] == -1 && sl[-1] == sl[0])
          if (sqnorm(sp[0], sp[-1]) <= squared_radius)
          {
            dl[-1] = cc;
            *ws++ = make_int2(p.x-1, p.y);
          }

        //top
        if( p.y < height-1 && dl[+width] == -1 && sl[+width] == sl[0])
          if (sqnorm(sp[0], sp[+width]) <= squared_radius)
          {
            dl[+width] = cc;
            *ws++ = make_int2(p.x, p.y+1);
          }

        //top
        if( p.y > 0 && dl[-width] == -1 && sl[-width] == sl[0])
          if (sqnorm(sp[0], sp[-width]) <= squared_radius)
          {
            dl[-width] = cc;
            *ws++ = make_int2(p.x, p.y-1);
          }

        // pop most recent and propagate
        p = *--ws;
        count++;
      }

      rsizes[cc] = count;
    } /* for(int i = 0; i < sz.width; ++i) */
  } /* for(int j = 0; j < sz.height; ++j) */    
}

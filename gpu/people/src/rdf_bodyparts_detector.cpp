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

#include <pcl/gpu/people/rdf_bodyparts_detector.h>
#include <cassert>
#include <opencv2/core/core.hpp>
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

  impl_.reset( new device::MultiTreeLiveProc(tree_files.size(), rows, cols) );

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
  // Process the depthimage (CUDA)
  impl_->process(depth, labels_);
  device::smoothLabelImage(labels_, depth, labels_smoothed_, NUM_PARTS, 5, 300);
}

//////////////////////////////////////////////////////////////////////
///////////////////// in development (dirty) /////////////////////////


#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>

void optimized_elec(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat& src_labels, float tolerance,
                    std::vector<std::vector<pcl::PointIndices> > &labeled_clusters,
                    unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster, unsigned int num_parts,
                    bool brute_force_border, float radius_scale);

void 
pcl::gpu::people::RDFBodyPartsDetector::step2_selectBetterName(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, 
                                                               int cluster_area_threshold, BlobMatrix& sorted)
{
  int cols = labels_smoothed_.cols();
  int rows = labels_smoothed_.rows();

  cv::Mat lmap(rows, cols, CV_8U);
  labels_smoothed_.download(lmap.data, lmap.step);

  // Make all the clusters
  vector<vector<pcl::PointIndices> > cluster_indices(NUM_PARTS);
  optimized_elec(*cloud, lmap, cluster_tolerance_, cluster_indices, cluster_area_threshold, max_cluster_size_, NUM_PARTS, false, 1.f);

  // Create a new struct to put the results in  
  sorted.clear();
  sorted.resize(NUM_PARTS);
  //create the blob2 matrix  
  label_skeleton::sortIndicesToBlob2 ( *cloud, cluster_area_threshold, sorted, cluster_indices );
    //Build relationships between the blobs
  label_skeleton::buildRelations ( sorted );
}

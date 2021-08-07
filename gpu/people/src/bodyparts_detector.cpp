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
//#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/label_common.h>
//#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>

#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include "cuda.h"
#include "emmintrin.h"

#include <cassert>
#include "internal.h"
#include "cuda_async_copy.h"

const int MAX_CLUST_SIZE = 25000;
const float CLUST_TOL = 0.05f;

pcl::gpu::people::RDFBodyPartsDetector::RDFBodyPartsDetector( const std::vector<std::string>& tree_files, int rows, int cols)
: max_cluster_size_(MAX_CLUST_SIZE), cluster_tolerance_(CLUST_TOL)
{
  PCL_DEBUG("[pcl::gpu::people::RDFBodyPartsDetector::RDFBodyPartsDetector] : (D) : Constructor called\n");
  //TODO replace all asserts with exceptions
  assert(!tree_files.empty());

  impl_.reset ( new device::MultiTreeLiveProc(rows, cols) );

  for(const auto &tree_file : tree_files)
  {
    // load the tree file
    std::vector<trees::Node>  nodes;
    std::vector<trees::Label> leaves;

    // this might throw but we haven't done any malloc yet
    int height = loadTree (tree_file, nodes, leaves );
    impl_->trees.emplace_back(height, nodes, leaves);
  }

  allocate_buffers(rows, cols);
}

////////////////////////////////////////////////////////////////////////////////////
/// getters

size_t 
pcl::gpu::people::RDFBodyPartsDetector::getNumberTrees() const
{
  return impl_->trees.size();
}

const pcl::device::Labels&
pcl::gpu::people::RDFBodyPartsDetector::getLabels() const
{
  return labels_smoothed_;
}

const pcl::device::LabelProbability&
pcl::gpu::people::RDFBodyPartsDetector::getProbability() const
{
  return P_l_;
}

const pcl::device::LabelProbability&
pcl::gpu::people::RDFBodyPartsDetector::getProbability1() const
{
  return P_l_1_;
}

const pcl::device::LabelProbability&
pcl::gpu::people::RDFBodyPartsDetector::getProbability2() const
{
  return P_l_2_;
}

const pcl::device::LabelProbability&
pcl::gpu::people::RDFBodyPartsDetector::getPrevProbability1() const
{
  return P_l_prev_1_;
}

const pcl::device::LabelProbability&
pcl::gpu::people::RDFBodyPartsDetector::getPrevProbability2() const
{
  return P_l_prev_2_;
}

const pcl::gpu::people::RDFBodyPartsDetector::BlobMatrix& 
pcl::gpu::people::RDFBodyPartsDetector::getBlobMatrix() const
{
  return blob_matrix_;
}

////////////////////////////////////////////////////////////////////////////////////

void 
pcl::gpu::people::RDFBodyPartsDetector::allocate_buffers(int rows, int cols)
{
  //std::cout << "(I) : RDFBodyPartsDetector::allocate_buffers called with: " << cols << "x" << rows << std::endl;

  labels_.create(rows, cols);
  labels_smoothed_.create(rows, cols);

  // Create all the label probabilities
  P_l_.create(rows,cols);
  P_l_Gaus_.create(rows,cols);
  P_l_Gaus_Temp_.create(rows,cols);
  P_l_1_.create(rows,cols);
  P_l_2_.create(rows,cols);
  P_l_prev_1_.create(rows,cols);
  P_l_prev_2_.create(rows,cols);

  lmap_host_.resize(rows * cols);

  dst_labels_.resize(rows * cols);
  region_sizes_.resize(rows*cols+1);
  remap_.resize(rows*cols);

  comps_.create(rows, cols);  
  device::ConnectedComponents::initEdges(rows, cols, edges_);

  means_storage_.resize((cols * rows + 1) * 3); // float3 * cols * rows and float3 for cc == -1.

  blob_matrix_.resize(NUM_PARTS);
  for(auto &matrix : blob_matrix_)
  {
    matrix.clear();
    matrix.reserve(5000);
  }
}

void 
pcl::gpu::people::RDFBodyPartsDetector::process (const pcl::device::Depth& depth, const PointCloud<PointXYZ>& cloud, int min_pts_per_cluster)
{
  //ScopeTime time("ev");

  int cols = depth.cols();
  int rows = depth.rows();

  allocate_buffers(rows, cols);

  {
    {
      //ScopeTime time("--");
      // Process the depthimage (CUDA)
      impl_->process(depth, labels_);
      device::smoothLabelImage(labels_, depth, labels_smoothed_, NUM_PARTS, 5, 300);
    }

    //AsyncCopy<unsigned char> async_labels_download(lmap_host_);

    int c;
    labels_smoothed_.download(lmap_host_, c);
    //async_labels_download.download(labels_smoothed_);

    // cc = generalized floodfill = approximation of euclidian clusterisation
    device::ConnectedComponents::computeEdges(labels_smoothed_, depth, NUM_PARTS, cluster_tolerance_ * cluster_tolerance_, edges_);
    device::ConnectedComponents::labelComponents(edges_, comps_);

    comps_.download(dst_labels_, c);

    //async_labels_download.waitForCompeltion();
  }

  // This was sort indices to blob (sortIndicesToBlob2) method (till line 236)
  {
    //ScopeTime time("cvt");
    std::fill(remap_.begin(), remap_.end(), -1);
    std::fill(region_sizes_.begin(), region_sizes_.end(), 0);

    std::fill(means_storage_.begin(), means_storage_.end(), 0);
    float3* means = (float3*) &means_storage_[3];
    int *rsizes = &region_sizes_[1];

    for(auto &matrix : blob_matrix_)
      matrix.clear();

    for(std::size_t k = 0; k < dst_labels_.size(); ++k)
    {
      const PointXYZ& p = cloud[k];
      int cc = dst_labels_[k];
      means[cc].x += p.x;
      means[cc].y += p.y;
      means[cc].z += p.z;
      ++rsizes[cc];
    }

    means[-1].z = 0; // cc == -1 means invalid

    for(std::size_t k = 0; k < dst_labels_.size(); ++k)
    {
      int label = lmap_host_[k];
      int cc    = dst_labels_[k];

      if (means[cc].z != 0 && min_pts_per_cluster <= rsizes[cc] && rsizes[cc] <= max_cluster_size_)
      {
        int ccindex = remap_[cc];
        if (ccindex == -1)
        {
          ccindex = static_cast<int> (blob_matrix_[label].size ());
          blob_matrix_[label].resize(ccindex + 1);
          remap_[cc] = ccindex;

          blob_matrix_[label][ccindex].label = static_cast<part_t> (label);
          blob_matrix_[label][ccindex].mean.coeffRef(0) = means[cc].x / static_cast<float> (rsizes[cc]);
          blob_matrix_[label][ccindex].mean.coeffRef(1) = means[cc].y / static_cast<float> (rsizes[cc]);
          blob_matrix_[label][ccindex].mean.coeffRef(2) = means[cc].z / static_cast<float> (rsizes[cc]);
          blob_matrix_[label][ccindex].indices.indices.reserve(rsizes[cc]);
        }
        blob_matrix_[label][ccindex].indices.indices.push_back(static_cast<int> (k));
      }
    }

    int id = 0;
    for(auto &matrix : blob_matrix_)
      for(std::size_t b = 0; b < matrix.size(); ++b)
      {
        matrix[b].id = id++;
        matrix[b].lid = static_cast<int> (b);
      }

    buildRelations ( blob_matrix_ );
  }
}

void
pcl::gpu::people::RDFBodyPartsDetector::processProb (const pcl::device::Depth& depth)
{
  int cols = depth.cols();
  int rows = depth.rows();

  allocate_buffers(rows, cols);

  // Process the depthimage into probabilities (CUDA)
  //impl_->process(depth, labels_);
  //impl_->processProb(depth, labels_, P_l_, (int) std::numeric_limits<std::int16_t>::max());
  impl_->processProb(depth, labels_, P_l_, std::numeric_limits<int>::max());
}

void
pcl::gpu::people::RDFBodyPartsDetector::processSmooth (const pcl::device::Depth& depth, const PointCloud<PointXYZ>& cloud, int min_pts_per_cluster)
{
  device::smoothLabelImage(labels_, depth, labels_smoothed_, NUM_PARTS, 5, 300);

  //AsyncCopy<unsigned char> async_labels_download(lmap_host_);

  int c;
  labels_smoothed_.download(lmap_host_, c);
  //async_labels_download.download(labels_smoothed_);

  // cc = generalized floodfill = approximation of euclidian clusterisation
  device::ConnectedComponents::computeEdges(labels_smoothed_, depth, NUM_PARTS, cluster_tolerance_ * cluster_tolerance_, edges_);
  device::ConnectedComponents::labelComponents(edges_, comps_);

  comps_.download(dst_labels_, c);

  //async_labels_download.waitForCompeltion();

  // This was sort indices to blob (sortIndicesToBlob2) method (till line 236)
  {
    ScopeTime time("[pcl::gpu::people::RDFBodyPartsDetector::processSmooth] : cvt");
    std::fill(remap_.begin(), remap_.end(), -1);
    std::fill(region_sizes_.begin(), region_sizes_.end(), 0);

    std::fill(means_storage_.begin(), means_storage_.end(), 0);
    float3* means = (float3*) &means_storage_[3];
    int *rsizes = &region_sizes_[1];

    for(auto &matrix : blob_matrix_)
      matrix.clear();

    for(std::size_t k = 0; k < dst_labels_.size(); ++k)
    {
      const PointXYZ& p = cloud[k];
      int cc = dst_labels_[k];
      means[cc].x += p.x;
      means[cc].y += p.y;
      means[cc].z += p.z;
      ++rsizes[cc];
    }

    means[-1].z = 0; // cc == -1 means invalid

    for(std::size_t k = 0; k < dst_labels_.size(); ++k)
    {
      int label = lmap_host_[k];
      int cc    = dst_labels_[k];

      if (means[cc].z != 0 && min_pts_per_cluster <= rsizes[cc] && rsizes[cc] <= max_cluster_size_)
      {
        int ccindex = remap_[cc];
        if (ccindex == -1)
        {
          ccindex = static_cast<int> (blob_matrix_[label].size ());
          blob_matrix_[label].resize(ccindex + 1);
          remap_[cc] = ccindex;

          blob_matrix_[label][ccindex].label = static_cast<part_t> (label);
          blob_matrix_[label][ccindex].mean.coeffRef(0) = means[cc].x / static_cast<float> (rsizes[cc]);
          blob_matrix_[label][ccindex].mean.coeffRef(1) = means[cc].y / static_cast<float> (rsizes[cc]);
          blob_matrix_[label][ccindex].mean.coeffRef(2) = means[cc].z / static_cast<float> (rsizes[cc]);
          blob_matrix_[label][ccindex].indices.indices.reserve(rsizes[cc]);
        }
        blob_matrix_[label][ccindex].indices.indices.push_back(static_cast<int> (k));
      }
    }

    int id = 0;
    for(auto &matrix : blob_matrix_)
      for(std::size_t b = 0; b < matrix.size(); ++b)
      {
        matrix[b].id = id++;
        matrix[b].lid = static_cast<int> (b);
      }
  }
}

int
pcl::gpu::people::RDFBodyPartsDetector::processRelations ()
{
  return buildRelations ( blob_matrix_ );
}

int
pcl::gpu::people::RDFBodyPartsDetector::processRelations (PersonAttribs::Ptr person_attribs)
{
  return buildRelations ( blob_matrix_, person_attribs );
}

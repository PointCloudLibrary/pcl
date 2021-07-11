/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id:$
 *
 */

#ifndef PCL_GPU_SEGMENTATION_IMPL_SEEDED_HUE_SEGMENTATION_H_
#define PCL_GPU_SEGMENTATION_IMPL_SEEDED_HUE_SEGMENTATION_H_

#include <pcl/gpu/segmentation/gpu_seeded_hue_segmentation.h>

//////////////////////////////////////////////////////////////////////////////////////////////
void
seededHueSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& host_cloud_,
                      const pcl::gpu::Octree::Ptr& tree,
                      float tolerance,
                      PointIndices& indices_in,
                      PointIndices& indices_out,
                      float delta_hue)
{

  // Create a bool vector of processed point indices, and initialize it to false
  // cloud is a DeviceArray<PointType>
  std::vector<bool> processed(host_cloud_->size(), false);

  const auto max_answers = host_cloud_->size();

  // Process all points in the indices vector
  for (std::size_t k = 0; k < indices_in.indices.size(); ++k) {
    int i = indices_in.indices[k];
    // if we already processed this point continue with the next one
    if (processed[i])
      continue;
    // now we will process this point
    processed[i] = true;

    PointXYZRGB p;
    p = (*host_cloud_)[i];
    PointXYZHSV h;
    PointXYZRGBtoXYZHSV(p, h);

    // Create the query queue on the device, point based not indices
    pcl::gpu::Octree::Queries queries_device;
    // Create the query queue on the host
    pcl::PointCloud<pcl::PointXYZ>::VectorType queries_host;
    // Push the starting point in the vector
    queries_host.push_back((*host_cloud_)[i]);

    unsigned int found_points = queries_host.size();
    unsigned int previous_found_points = 0;

    pcl::gpu::NeighborIndices result_device;

    // Host buffer for results
    std::vector<int> sizes, data;

    // once the area stop growing, stop also iterating.
    while (previous_found_points < found_points) {
      // Move queries to GPU
      queries_device.upload(queries_host);
      // Execute search
      tree->radiusSearch(queries_device, tolerance, max_answers, result_device);

      // Store the previously found number of points
      previous_found_points = found_points;

      // Clear the Host vectors
      sizes.clear();
      data.clear();

      // Copy results from GPU to Host
      result_device.sizes.download(sizes);
      result_device.data.download(data);

      for (std::size_t qp = 0; qp < sizes.size(); qp++) {
        for (int qp_r = 0; qp_r < sizes[qp]; qp_r++) {
          if (processed[data[qp_r + qp * max_answers]])
            continue;

          PointXYZRGB p_l;
          p_l = (*host_cloud_)[data[qp_r + qp * max_answers]];
          PointXYZHSV h_l;
          PointXYZRGBtoXYZHSV(p_l, h_l);

          if (std::abs(h_l.h - h.h) < delta_hue) {
            processed[data[qp_r + qp * max_answers]] = true;
            queries_host.push_back((*host_cloud_)[data[qp_r + qp * max_answers]]);
            found_points++;
          }
        }
      }
    }
    for (std::size_t qp = 0; qp < sizes.size(); qp++) {
      for (int qp_r = 0; qp_r < sizes[qp]; qp_r++) {
        indices_out.indices.push_back(data[qp_r + qp * max_answers]);
      }
    }
  }
  // @todo: do we need to sort here and remove double points?
}

void
pcl::gpu::SeededHueSegmentation::segment(PointIndices& indices_in,
                                         PointIndices& indices_out)
{
  // Initialize the GPU search tree
  if (!tree_) {
    tree_.reset(new pcl::gpu::Octree());
    ///@todo what do we do if input isn't a PointXYZ cloud?
    tree_->setCloud(input_);
  }
  if (!tree_->isBuild()) {
    tree_->build();
  }
  /*
    if(tree_->cloud_.size() != host_cloud.size ())
    {
      PCL_ERROR("[pcl::gpu::SeededHueSegmentation] size of host cloud and device cloud
    don't match!\n"); return;
    }
  */
  // Extract the actual clusters
  seededHueSegmentation(
      host_cloud_, tree_, cluster_tolerance_, indices_in, indices_out, delta_hue_);
}

#endif // PCL_GPU_SEGMENTATION_IMPL_SEEDED_HUE_SEGMENTATION_H_

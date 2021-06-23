/*
 * Software License Agreement (BSD License)
 *
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
 * $Id: extract_clusters.cpp 2993 2011-10-31 17:17:53Z koenbuys $
 *
 */

#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>
#include <pcl/gpu/segmentation/impl/gpu_extract_labeled_clusters.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(extractEuclideanClusters, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(EuclideanClusterExtraction, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(extractLabeledEuclideanClusters, PCL_XYZL_POINT_TYPES);
PCL_INSTANTIATE(EuclideanLabeledClusterExtraction, PCL_XYZL_POINT_TYPES);

void
pcl::detail::economical_download(const pcl::gpu::NeighborIndices& source_indices,
                                 const pcl::Indices& buffer_indices,
                                 std::size_t buffer_size,
                                 pcl::Indices& downloaded_indices)
{
  std::vector<int> tmp;
  for (std::size_t qp = 0; qp < buffer_indices.size(); qp++) {
    std::size_t begin = qp * buffer_size;
    tmp.resize(buffer_indices[qp]);
    source_indices.data.download(&tmp[0], begin, buffer_indices[qp]);
    downloaded_indices.insert(downloaded_indices.end(), tmp.begin(), tmp.end());
  }
}

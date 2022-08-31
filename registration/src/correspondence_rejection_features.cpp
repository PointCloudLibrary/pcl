/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/registration/correspondence_rejection_features.h>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::CorrespondenceRejectorFeatures::getRemainingCorrespondences(
    const pcl::Correspondences& original_correspondences,
    pcl::Correspondences& remaining_correspondences)
{
  unsigned int number_valid_correspondences = 0;
  remaining_correspondences.resize(original_correspondences.size());
  // For each set of features, go over each correspondence from input_correspondences_
  for (std::size_t i = 0; i < input_correspondences_->size(); ++i) {
    // Go over the map of features
    for (const auto& feature : features_map_) {
      // Check if the score in feature space is above the given threshold
      // (assume that the number of feature correspondenecs is the same as the number of
      // point correspondences)
      if (!feature.second->isCorrespondenceValid(static_cast<int>(i)))
        break;

      remaining_correspondences[number_valid_correspondences] =
          original_correspondences[i];
      ++number_valid_correspondences;
    }
  }
  remaining_correspondences.resize(number_valid_correspondences);
}

//////////////////////////////////////////////////////////////////////////////////////////////
inline bool
pcl::registration::CorrespondenceRejectorFeatures::hasValidFeatures()
{
  if (features_map_.empty())
    return (false);
  for (const auto& feature : features_map_)
    if (!feature.second->isValid())
      return (false);
  return (true);
}

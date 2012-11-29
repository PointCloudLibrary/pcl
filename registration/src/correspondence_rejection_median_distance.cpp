/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Open Perception, Inc.
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
 *   * Neither the name of Open Perception, Inc. nor the names of its
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

#include <pcl/registration/correspondence_rejection_median_distance.h>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::CorrespondenceRejectorMedianDistance::getRemainingCorrespondences (
    const pcl::Correspondences& original_correspondences,
    pcl::Correspondences& remaining_correspondences)
{
  std::vector <double> dists;
  dists.resize (original_correspondences.size ());

  for (size_t i = 0; i < original_correspondences.size (); ++i)
  {
    if (data_container_)
      dists[i] = data_container_->getCorrespondenceScore (original_correspondences[i]);
    else
      dists[i] = original_correspondences[i].distance;
  }

  std::vector <double> nth (dists);
  nth_element (nth.begin (), nth.begin () + (nth.size () / 2), nth.end ());
  median_distance_ = nth [nth.size () / 2];

  unsigned int number_valid_correspondences = 0;
  remaining_correspondences.resize (original_correspondences.size ());

  for (size_t i = 0; i < original_correspondences.size (); ++i)
    if (dists[i] <= median_distance_ * factor_)
      remaining_correspondences[number_valid_correspondences++] = original_correspondences[i];
  remaining_correspondences.resize (number_valid_correspondences);
}

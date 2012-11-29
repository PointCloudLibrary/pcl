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

#include <pcl/registration/correspondence_rejection_var_trimmed.h>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::CorrespondenceRejectorVarTrimmed::getRemainingCorrespondences (
    const pcl::Correspondences& original_correspondences,
    pcl::Correspondences& remaining_correspondences)
{
  std::vector <double> dists;
  dists.resize (original_correspondences.size ());

  for (size_t i = 0; i < original_correspondences.size (); ++i)
  {
    if (data_container_)
    {
      dists[i] = data_container_->getCorrespondenceScore (original_correspondences[i]);
    }
    else
    {
      dists[i] = original_correspondences[i].distance;
    }
  }
  factor_ = optimizeInlierRatio (dists);
  nth_element (dists.begin (), dists.begin () + int (double (dists.size ()) * factor_), dists.end ());
  trimmed_distance_ = dists [int (double (dists.size ()) * factor_)];

  unsigned int number_valid_correspondences = 0;
  remaining_correspondences.resize (original_correspondences.size ());

  for (size_t i = 0; i < original_correspondences.size (); ++i)
  {
    if ( dists[i] < trimmed_distance_)
    {
      remaining_correspondences[number_valid_correspondences] = original_correspondences[i];
      ++number_valid_correspondences;
    }
  }
  remaining_correspondences.resize (number_valid_correspondences);
}

//////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::registration::CorrespondenceRejectorVarTrimmed::optimizeInlierRatio (std::vector <double>&  dists)
{
  unsigned int points_nbr = static_cast<unsigned int> (dists.size ());
  std::sort (dists.begin (), dists.end ());

  const int min_el = int (floor (min_ratio_ * points_nbr));
  const int max_el = int (floor (max_ratio_ * points_nbr));

  typedef Eigen::Array <double, Eigen::Dynamic, 1> LineArray;
  Eigen::Map<LineArray> sorted_dist (&dists[0], points_nbr);

  const LineArray trunk_sorted_dist = sorted_dist.segment (min_el, max_el-min_el);
  const double lower_sum = sorted_dist.head (min_el).sum ();
  const LineArray ids = LineArray::LinSpaced (trunk_sorted_dist.rows (), min_el+1, max_el);
  const LineArray ratio = ids / points_nbr;
  const LineArray deno = ratio.pow (lambda_);
  const LineArray FRMS = deno.inverse ().square () * ids.inverse () * (lower_sum + trunk_sorted_dist);
  int min_index (0);
  FRMS.minCoeff (&min_index);

  const float opt_ratio = float (min_index + min_el) / float (points_nbr);
  return (opt_ratio);
}

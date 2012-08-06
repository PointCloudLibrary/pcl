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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include <cstddef>
#include <iostream>
#include <pcl/common/eigen.h>
#include <pcl/common/poses_from_matches.h>
#include <pcl/common/transformation_from_correspondences.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PosesFromMatches::PosesFromMatches () : parameters_ () 
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PosesFromMatches::~PosesFromMatches ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::PosesFromMatches::estimatePosesUsing1Correspondence (const pcl::PointCorrespondences6DVector& correspondences,
                                                          int max_no_of_results,
                                                          pcl::PosesFromMatches::PoseEstimatesVector& pose_estimates) const
{
  if (max_no_of_results < 0)
    max_no_of_results = static_cast<int> (correspondences.size ());
  else
    max_no_of_results = std::min (max_no_of_results, static_cast<int> (correspondences.size ()));
  
  for (int correspondence_idx = 0; correspondence_idx < max_no_of_results; ++correspondence_idx)
  {
    const pcl::PointCorrespondence6D& correspondence = correspondences[correspondence_idx];
    PoseEstimate pose_estimate;
    pose_estimate.transformation = correspondence.transformation;
    pose_estimate.score = correspondence.distance;
    pose_estimate.correspondence_indices.push_back (correspondence_idx);
    pose_estimates.push_back (pose_estimate);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::PosesFromMatches::estimatePosesUsing2Correspondences (const pcl::PointCorrespondences6DVector& correspondences,
                                                           int max_no_of_tested_combinations, int max_no_of_results,
                                                           pcl::PosesFromMatches::PoseEstimatesVector& pose_estimates) const
{
  const Eigen::Vector3f x_direction (1.0f, 0.0f, 0.0f),
                        y_direction (0.0f, 1.0f, 0.0f),
                        z_direction (0.0f, 0.0f, 1.0f);
  
  int max_correspondence_idx = static_cast<int> (correspondences.size ());
  int counter_for_tested_combinations = 0,
      counter_for_added_pose_estimates = 0;
  float max_distance_quotient = 1.0f+parameters_.max_correspondence_distance_error,
        max_distance_quotient_squared=powf (max_distance_quotient, 2),
        min_distance_quotient = 1.0f / (max_distance_quotient),
        min_distance_quotient_squared = std::pow (min_distance_quotient, 2);

  pcl::TransformationFromCorrespondences transformation_from_correspondeces;
  
  // The following loop structure goes through the pairs in the order 12, 13, 23, 14, 24, 34, ...,
  // testing the best correspondences pairs first, without beeing stuck too long with one specific
  // (possibly wrong) correspondence.
  bool done = false;
  for (int correspondence2_idx = 0; correspondence2_idx < max_correspondence_idx && !done; ++correspondence2_idx)
  {
    const pcl::PointCorrespondence6D& correspondence2 = correspondences[correspondence2_idx];
    for (int correspondence1_idx = 0; correspondence1_idx < correspondence2_idx; ++correspondence1_idx)
    {
      if (counter_for_tested_combinations >= max_no_of_tested_combinations)
      {
        done = true;
        break;
      }
      
      const pcl::PointCorrespondence6D& correspondence1 = correspondences[correspondence1_idx];
      ++counter_for_tested_combinations;
      
      const Eigen::Vector3f& point1 = correspondence1.point1, & point2 = correspondence2.point1,
                           & corr1  = correspondence1.point2, & corr2  = correspondence2.point2;
      
      float distance_squared = (point2-point1).squaredNorm (),
            distance_corr_squared = (corr2-corr1).squaredNorm (),
            distance_quotient_squared = distance_squared/distance_corr_squared;
      if (   distance_quotient_squared < min_distance_quotient_squared
          || distance_quotient_squared > max_distance_quotient_squared)
      {
        //std::cout << "Skipping because of mismatching distances "<<sqrtf (distance1_squared)
        //          << " and "<<sqrtf (distance1_corr_squared)<<".\n";
        continue;
      }
      
      float distance = sqrtf (distance_squared);
      
      Eigen::Vector3f corr3=corr1, corr4=corr2;
      corr3[0]+=distance; corr4[0]+=distance;
      Eigen::Vector3f point3=correspondence1.transformation*corr3, point4=correspondence2.transformation*corr4;
      
      distance_squared = (point4-point3).squaredNorm (),
      distance_corr_squared = (corr4-corr3).squaredNorm (),
      distance_quotient_squared = distance_squared/distance_corr_squared;
      if (   distance_quotient_squared < min_distance_quotient_squared
          || distance_quotient_squared > max_distance_quotient_squared)
        continue;
      
      Eigen::Vector3f corr5=corr1, corr6=corr2;
      corr5[1]+=distance; corr6[1]+=distance;
      Eigen::Vector3f point5=correspondence1.transformation*corr5, point6=correspondence2.transformation*corr6;
      
      distance_squared = (point6-point5).squaredNorm (),
      distance_corr_squared = (corr6-corr5).squaredNorm (),
      distance_quotient_squared = distance_squared/distance_corr_squared;
      if (   distance_quotient_squared < min_distance_quotient_squared
          || distance_quotient_squared > max_distance_quotient_squared)
        continue;
      
      Eigen::Vector3f corr7=corr1, corr8=corr2;
      corr7[2]+=distance; corr8[2]+=distance;
      Eigen::Vector3f point7=correspondence1.transformation*corr7, point8=correspondence2.transformation*corr8;
      
      distance_squared = (point8-point7).squaredNorm (),
      distance_corr_squared = (corr8-corr7).squaredNorm (),
      distance_quotient_squared = distance_squared/distance_corr_squared;
      if (   distance_quotient_squared < min_distance_quotient_squared
          || distance_quotient_squared > max_distance_quotient_squared)
        continue;
      
      transformation_from_correspondeces.reset ();
      transformation_from_correspondeces.add (corr1, point1);
      transformation_from_correspondeces.add (corr2, point2);
      transformation_from_correspondeces.add (corr3, point3);
      transformation_from_correspondeces.add (corr4, point4);
      transformation_from_correspondeces.add (corr5, point5);
      transformation_from_correspondeces.add (corr6, point6);
      transformation_from_correspondeces.add (corr7, point7);
      transformation_from_correspondeces.add (corr8, point8);
      
      ++counter_for_added_pose_estimates;
      PoseEstimate pose_estimate;
      pose_estimate.transformation = transformation_from_correspondeces.getTransformation ();
      pose_estimate.score = 0.5f * (correspondence1.distance + correspondence2.distance); // TODO: based
                                                                                // on the measured distance_errors?
      pose_estimate.correspondence_indices.push_back (correspondence1_idx);
      pose_estimate.correspondence_indices.push_back (correspondence2_idx);
      pose_estimates.push_back (pose_estimate);
      if (counter_for_added_pose_estimates >= max_no_of_results)
      {
        done = true;
        break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PosesFromMatches::estimatePosesUsing3Correspondences (const PointCorrespondences6DVector& correspondences,
                                                           int max_no_of_tested_combinations, int max_no_of_results,
                                                           PosesFromMatches::PoseEstimatesVector& pose_estimates) const
{
  const Eigen::Vector3f x_direction (1.0f, 0.0f, 0.0f),
                        y_direction (0.0f, 1.0f, 0.0f),
                        z_direction (0.0f, 0.0f, 1.0f);
  
  int max_correspondence_idx = static_cast<int> (correspondences.size ());
  int counter_for_tested_combinations = 0,
      counter_for_added_pose_estimates = 0;
  float max_distance_quotient = 1.0f+parameters_.max_correspondence_distance_error,
        max_distance_quotient_squared = std::pow (max_distance_quotient, 2),
        min_distance_quotient = 1.0f / (max_distance_quotient),
        min_distance_quotient_squared = std::pow (min_distance_quotient, 2);

  pcl::TransformationFromCorrespondences transformation_from_correspondeces;
  
  // The following loop structure goes through the triples in the order 123, 124, 134, 234, 125, 135, 235, ...,
  // testing the best correspondences triples first, without beeing stuck too long with one specific
  // (possibly wrong) correspondence.
  bool done = false;
  for (int correspondence3_idx = 0; correspondence3_idx < max_correspondence_idx && !done; ++correspondence3_idx)
  {
    const pcl::PointCorrespondence6D& correspondence3 = correspondences[correspondence3_idx];
    const Eigen::Vector3f& point3 = correspondence3.point1,
                  & corr3  = correspondence3.point2;
    for (int correspondence2_idx = 0; correspondence2_idx < correspondence3_idx && !done; ++correspondence2_idx)
    {
      const pcl::PointCorrespondence6D& correspondence2 = correspondences[correspondence2_idx];
      const Eigen::Vector3f& point2 = correspondence2.point1,
                    & corr2  = correspondence2.point2;
      
      float distance23_squared = (point3-point2).squaredNorm (),
            distance23_corr_squared = (corr3-corr2).squaredNorm (),
            distance23_quotient_squared = distance23_squared/distance23_corr_squared;
      if (   distance23_quotient_squared < min_distance_quotient_squared 
          || distance23_quotient_squared > max_distance_quotient_squared)
        continue;
      
      for (int correspondence1_idx = 0; correspondence1_idx < correspondence2_idx; ++correspondence1_idx)
      {
        if (counter_for_tested_combinations >= max_no_of_tested_combinations)
        {
          done = true;
          break;
        }
        ++counter_for_tested_combinations;
        const pcl::PointCorrespondence6D& correspondence1 = correspondences[correspondence1_idx];
        const Eigen::Vector3f& point1 = correspondence1.point1,
                             & corr1  = correspondence1.point2;
        float distance12_squared = (point2-point1).squaredNorm (),
              distance12_corr_squared = (corr2-corr1).squaredNorm (),
              distance12_quotient_squared = distance12_squared/distance12_corr_squared;
        if (   distance12_quotient_squared < min_distance_quotient_squared
            || distance12_quotient_squared > max_distance_quotient_squared)
          continue;
        float distance13_squared = (point3-point1).squaredNorm (),
              distance13_corr_squared = (corr3-corr1).squaredNorm (),
              distance13_quotient_squared = distance13_squared/distance13_corr_squared;
        if (   distance13_quotient_squared < min_distance_quotient_squared
            || distance13_quotient_squared > max_distance_quotient_squared)
          continue;
        
        transformation_from_correspondeces.reset ();
        transformation_from_correspondeces.add (corr1, point1);
        transformation_from_correspondeces.add (corr2, point2);
        transformation_from_correspondeces.add (corr3, point3);
        
        ++counter_for_added_pose_estimates;
        PoseEstimate pose_estimate;
        pose_estimate.transformation = transformation_from_correspondeces.getTransformation ();
        pose_estimate.score = (correspondence1.distance + correspondence2.distance + correspondence3.distance) / 3.0f; // TODO: based
                                                                                  // on the measured distance_errors?
        pose_estimate.correspondence_indices.push_back (correspondence1_idx);
        pose_estimate.correspondence_indices.push_back (correspondence2_idx);
        pose_estimate.correspondence_indices.push_back (correspondence3_idx);
        pose_estimates.push_back (pose_estimate);
        if (counter_for_added_pose_estimates >= max_no_of_results)
        {
          done = true;
          break;
        }
      }
    }
  }
}


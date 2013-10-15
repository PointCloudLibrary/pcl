/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_HPP_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_HPP_

#include <boost/unordered_map.hpp>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::setInputCloud (
    const typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::PointCloudConstPtr &cloud)
{
  setInputSource (cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::PointCloudConstPtr const
pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::getInputCloud ()
{
  return (getInputSource ()); 
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::setTargetCloud (
    const typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::PointCloudConstPtr &cloud)
{
  setInputTarget (cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::setMaxIterations (
    int max_iterations)
{
  setMaximumIterations (max_iterations);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::getMaxIterations ()
{
  return (getMaximumIterations ());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::getRemainingCorrespondences (
    const pcl::Correspondences& original_correspondences, 
    pcl::Correspondences& remaining_correspondences)
{
  if (!input_)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No input cloud dataset was given!\n", getClassName ().c_str ());
    return;
  }

  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No input target dataset was given!\n", getClassName ().c_str ());
    return;
  }

  if (save_inliers_)
     inlier_indices_.clear ();

  int nr_correspondences = static_cast<int> (original_correspondences.size ());
  std::vector<int> source_indices (nr_correspondences);
  std::vector<int> target_indices (nr_correspondences);

  // Copy the query-match indices
  for (size_t i = 0; i < original_correspondences.size (); ++i)
  {
    source_indices[i] = original_correspondences[i].index_query;
    target_indices[i] = original_correspondences[i].index_match;
  }

   // from pcl/registration/icp.hpp:
   std::vector<int> source_indices_good;
   std::vector<int> target_indices_good;
   {
     // From the set of correspondences found, attempt to remove outliers
     // Create the registration model
     typedef typename pcl::SampleConsensusModelRegistration<PointT>::Ptr SampleConsensusModelRegistrationPtr;
     SampleConsensusModelRegistrationPtr model;
     model.reset (new pcl::SampleConsensusModelRegistration<PointT> (input_, source_indices));
     // Pass the target_indices
     model->setInputTarget (target_, target_indices);
     // Create a RANSAC model
     pcl::RandomSampleConsensus<PointT> sac (model, inlier_threshold_);
     sac.setMaxIterations (max_iterations_);

     // Compute the set of inliers
     if (!sac.computeModel ())
     {
       remaining_correspondences = original_correspondences;
       best_transformation_.setIdentity ();
       return;
     }
     else
     {
       if (refine_ && !sac.refineModel ())
       {
         PCL_ERROR ("[pcl::registration::CorrespondenceRejectorSampleConsensus::getRemainingCorrespondences] Could not refine the model! Returning an empty solution.\n");
         return;
       }
       
       std::vector<int> inliers;
       sac.getInliers (inliers);

       if (inliers.size () < 3)
       {
         remaining_correspondences = original_correspondences;
         best_transformation_.setIdentity ();
         return;
       }
       boost::unordered_map<int, int> index_to_correspondence;
       for (int i = 0; i < nr_correspondences; ++i)
         index_to_correspondence[original_correspondences[i].index_query] = i;

       remaining_correspondences.resize (inliers.size ());
       for (size_t i = 0; i < inliers.size (); ++i)
         remaining_correspondences[i] = original_correspondences[index_to_correspondence[inliers[i]]];

       if (save_inliers_)
       {
         inlier_indices_.reserve (inliers.size ());
         for (size_t i = 0; i < inliers.size (); ++i)
           inlier_indices_.push_back (index_to_correspondence[inliers[i]]);
       }

       // get best transformation
       Eigen::VectorXf model_coefficients;
       sac.getModelCoefficients (model_coefficients);
       best_transformation_.row (0) = model_coefficients.segment<4>(0);
       best_transformation_.row (1) = model_coefficients.segment<4>(4);
       best_transformation_.row (2) = model_coefficients.segment<4>(8);
       best_transformation_.row (3) = model_coefficients.segment<4>(12);
     }
   }
}

#endif    // PCL_REGISTRATION_IMPL_CORRESPONDENCE_REJECTION_SAMPLE_CONSENSUS_HPP_

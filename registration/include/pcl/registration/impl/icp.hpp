/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc
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

#include <boost/unordered_map.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::IterativeClosestPoint<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess)
{
  // Allocate enough space to hold the results
  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // Point cloud containing the correspondences of each point in <input, indices>
  PointCloudTarget input_corresp;
  input_corresp.points.resize (indices_->size ());

  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;

  // If the guessed transformation is non identity
  if (guess != Eigen::Matrix4f::Identity ())
  {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
    transformPointCloud (output, output, guess);
  }

  // Resize the vector of distances between correspondences 
  std::vector<float> previous_correspondence_distances (indices_->size ());
  correspondence_distances_.resize (indices_->size ());

  while (!converged_)           // repeat until convergence
  {
    // Save the previously estimated transformation
    previous_transformation_ = transformation_;
    // And the previous set of distances
    previous_correspondence_distances = correspondence_distances_;

    int cnt = 0;
    std::vector<int> source_indices (indices_->size ());
    std::vector<int> target_indices (indices_->size ());

    // Iterating over the entire index vector and  find all correspondences
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (!this->searchForNeighbors (output, (*indices_)[idx], nn_indices, nn_dists))
      {
        PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[idx]);
        return;
      }

      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)
      {
        source_indices[cnt] = (*indices_)[idx];
        target_indices[cnt] = nn_indices[0];
        cnt++;
      }

      // Save the nn_dists[0] to a global vector of distances
      correspondence_distances_[(*indices_)[idx]] = std::min (nn_dists[0], static_cast<float> (dist_threshold));
    }
    if (cnt < min_number_correspondences_)
    {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName ().c_str ());
      converged_ = false;
      return;
    }

    // Resize to the actual number of valid correspondences
    source_indices.resize (cnt); target_indices.resize (cnt);

    std::vector<int> source_indices_good;
    std::vector<int> target_indices_good;
    {
      // From the set of correspondences found, attempt to remove outliers
      // Create the registration model
      typedef typename SampleConsensusModelRegistration<PointSource>::Ptr SampleConsensusModelRegistrationPtr;
      SampleConsensusModelRegistrationPtr model;
      model.reset (new SampleConsensusModelRegistration<PointSource> (output.makeShared (), source_indices));
      // Pass the target_indices
      model->setInputTarget (target_, target_indices);
      // Create a RANSAC model
      RandomSampleConsensus<PointSource> sac (model, inlier_threshold_);
      sac.setMaxIterations (ransac_iterations_);

      // Compute the set of inliers
      if (!sac.computeModel ())
      {
        source_indices_good = source_indices;
        target_indices_good = target_indices;
      }
      else
      {
        std::vector<int> inliers;
        // Get the inliers
        sac.getInliers (inliers);
        source_indices_good.resize (inliers.size ());
        target_indices_good.resize (inliers.size ());

        boost::unordered_map<int, int> source_to_target;
        for (unsigned int i = 0; i < source_indices.size(); ++i)
          source_to_target[source_indices[i]] = target_indices[i];

        // Copy just the inliers
        std::copy(inliers.begin(), inliers.end(), source_indices_good.begin());
        for (size_t i = 0; i < inliers.size (); ++i)
          target_indices_good[i] = source_to_target[inliers[i]];
      }
    }

    // Check whether we have enough correspondences
    cnt = static_cast<int> (source_indices_good.size ());
    if (cnt < min_number_correspondences_)
    {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName ().c_str ());
      converged_ = false;
      return;
    }

    PCL_DEBUG ("[pcl::%s::computeTransformation] Number of correspondences %d [%f%%] out of %zu points [100.0%%], RANSAC rejected: %zu [%f%%].\n", 
        getClassName ().c_str (), 
        cnt, 
        (static_cast<float> (cnt) * 100.0f) / static_cast<float> (indices_->size ()), 
        indices_->size (), 
        source_indices.size () - cnt, 
        static_cast<float> (source_indices.size () - cnt) * 100.0f / static_cast<float> (source_indices.size ()));
  
    // Estimate the transform
    //rigid_transformation_estimation_(output, source_indices_good, *target_, target_indices_good, transformation_);
    transformation_estimation_->estimateRigidTransformation (output, source_indices_good, *target_, target_indices_good, transformation_);

    // Tranform the data
    transformPointCloud (output, output, transformation_);

    // Obtain the final transformation    
    final_transformation_ = transformation_ * final_transformation_;

    nr_iterations_++;

    // Update the vizualization of icp convergence
    if (update_visualizer_ != 0)
      update_visualizer_(output, source_indices_good, *target_, target_indices_good );

    // Various/Different convergence termination criteria
    // 1. Number of iterations has reached the maximum user imposed number of iterations (via 
    //    setMaximumIterations)
    // 2. The epsilon (difference) between the previous transformation and the current estimated transformation 
    //    is smaller than an user imposed value (via setTransformationEpsilon)
    // 3. The sum of Euclidean squared errors is smaller than a user defined threshold (via 
    //    setEuclideanFitnessEpsilon)

    if (nr_iterations_ >= max_iterations_ ||
        (transformation_ - previous_transformation_).array ().abs ().sum () < transformation_epsilon_ ||
        fabs (this->getFitnessScore (correspondence_distances_, previous_correspondence_distances)) <= euclidean_fitness_epsilon_
       )
    {
      converged_ = true;
      PCL_DEBUG ("[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f\n",
                 getClassName ().c_str (), nr_iterations_, max_iterations_, (transformation_ - previous_transformation_).array ().abs ().sum ());

      PCL_DEBUG ("nr_iterations_ (%d) >= max_iterations_ (%d)\n", nr_iterations_, max_iterations_);
      PCL_DEBUG ("(transformation_ - previous_transformation_).array ().abs ().sum () (%f) < transformation_epsilon_ (%f)\n",
                 (transformation_ - previous_transformation_).array ().abs ().sum (), transformation_epsilon_);
      PCL_DEBUG ("fabs (getFitnessScore (correspondence_distances_, previous_correspondence_distances)) (%f) <= euclidean_fitness_epsilon_ (%f)\n",
                 fabs (this->getFitnessScore (correspondence_distances_, previous_correspondence_distances)),
                 euclidean_fitness_epsilon_);

    }
  }
}


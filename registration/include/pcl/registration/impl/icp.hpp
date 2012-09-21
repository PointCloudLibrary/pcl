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

#include <pcl/registration/boost.h>
#include <pcl/correspondence.h>
#include <pcl/registration/default_convergence_criteria.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::computeTransformation (
    PointCloudSource &output, const Matrix4 &guess)
{
  // Allocate enough space to hold the results
  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // Point cloud containing the correspondences of each point in <input, indices>
  PointCloudSourcePtr input_transformed (new PointCloudSource);

  nr_iterations_ = 0;
  converged_ = false;

  // Initialise final transformation to the guessed one
  final_transformation_ = guess;

  // If the guessed transformation is non identity
  if (guess != Matrix4::Identity ())
    // Apply guessed transformation prior to search for neighbours
    transformPointCloud (*input_, *input_transformed, guess);
  else
    *input_transformed = *input_;
 
  transformation_ = Matrix4::Identity ();

  // Pass in the default source and target for the Correspondence Estimation/Rejection code
  correspondence_estimation_->setInputSource (input_transformed);
  correspondence_estimation_->setInputTarget (target_);
  //for (size_t i = 0; i < correspondence_rejectors_.size (); ++i)
  //{
  //  correspondence_rejectors_[i]->setInputCloud (input_transformed);
  //  correspondence_rejectors_[i]->setInputTarget (target_);
  //}

  CorrespondencesPtr correspondences (new Correspondences);

  pcl::registration::DefaultConvergenceCriteria<Scalar> converged (nr_iterations_, transformation_, *correspondences);
  converged.setMaximumIterations (max_iterations_);
  
  // Repeat until convergence
  do
  {
    // Save the previously estimated transformation
    previous_transformation_ = transformation_;

    // And the previous set of distances
    //previous_correspondence_distances = correspondence_distances_;

    // Estimate correspondences
    correspondence_estimation_->updateSource (transformation_);
    correspondence_estimation_->determineCorrespondences (*correspondences);

    //if (correspondence_rejectors_.empty ())
    CorrespondencesPtr temp_correspondences (new Correspondences (*correspondences));
    for (size_t i = 0; i < correspondence_rejectors_.size (); ++i)
    {
      PCL_DEBUG ("Applying a correspondence rejector method: %s.\n", correspondence_rejectors_[i]->getClassName ().c_str ());
      correspondence_rejectors_[i]->setInputCorrespondences (temp_correspondences);
      correspondence_rejectors_[i]->updateSource (transformation_.template cast<double> ());
      correspondence_rejectors_[i]->getCorrespondences (*correspondences);
      // Modify input for the next iteration
      if (i < correspondence_rejectors_.size () - 1)
        *temp_correspondences = *correspondences;
    }
    //}

    size_t cnt = correspondences->size ();
    // Check whether we have enough correspondences
    if (cnt < min_number_correspondences_)
    {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName ().c_str ());
      break;
    }

    PCL_DEBUG ("[pcl::%s::computeTransformation] Number of correspondences %d [%f%%] out of %zu points [100.0%%], RANSAC rejected: %zu [%f%%].\n", 
        getClassName ().c_str (), 
        cnt, 
        (static_cast<float> (cnt) * 100.0f) / static_cast<float> (indices_->size ()), 
        indices_->size (), 
        indices_->size () - cnt, 
        static_cast<float> (indices_->size () - cnt) * 100.0f / static_cast<float> (indices_->size ()));
  
    // Estimate the transform
    transformation_estimation_->estimateRigidTransformation (*input_transformed, *target_, *correspondences, transformation_);

    // Tranform the data
    transformPointCloud (*input_transformed, *input_transformed, transformation_);

    // Obtain the final transformation    
    final_transformation_ = transformation_ * final_transformation_;

    ++nr_iterations_;

    // Update the vizualization of icp convergence
    //if (update_visualizer_ != 0)
    //  update_visualizer_(output, source_indices_good, *target_, target_indices_good );
  }
  while (!converged);

  // Transform the input cloud using the final transformation
  PCL_DEBUG ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n", 
      final_transformation_ (0, 0), final_transformation_ (0, 1), final_transformation_ (0, 2), final_transformation_ (0, 3),
      final_transformation_ (1, 0), final_transformation_ (1, 1), final_transformation_ (1, 2), final_transformation_ (1, 3),
      final_transformation_ (2, 0), final_transformation_ (2, 1), final_transformation_ (2, 2), final_transformation_ (2, 3),
      final_transformation_ (3, 0), final_transformation_ (3, 1), final_transformation_ (3, 2), final_transformation_ (3, 3));
  transformPointCloud (*input_, output, final_transformation_);
}


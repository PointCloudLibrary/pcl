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

#pragma once

#include <pcl/correspondence.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/types.h>

namespace pcl
{
  /**
    * \brief calculate 3D transformation based on point correspondences
    * \author Bastian Steder
    * \ingroup common
    */
  class PCL_EXPORTS PosesFromMatches
  {
    public:
      // =====STRUCTS=====
      //! Parameters used in this class
      struct PCL_EXPORTS Parameters
      {
        float max_correspondence_distance_error = 0.2f;  // As a fraction
      };

      //! A result of the pose estimation process
      struct PoseEstimate
      {
        Eigen::Affine3f transformation = Eigen::Affine3f::Identity ();   //!< The estimated transformation between the two coordinate systems
        float score = 0;                         //!< An estimate in [0,1], how good the estimated pose is 
        Indices correspondence_indices;  //!< The indices of the used correspondences

        struct IsBetter 
        {
          bool operator()(const PoseEstimate& pe1, const PoseEstimate& pe2) const { return pe1.score>pe2.score;}
        };
        public:
          PCL_MAKE_ALIGNED_OPERATOR_NEW
      };
      
      // =====TYPEDEFS=====
      using PoseEstimatesVector = std::vector<PoseEstimate, Eigen::aligned_allocator<PoseEstimate> >;

      
      // =====STATIC METHODS=====
      
      // =====PUBLIC METHODS=====
      /** Use single 6DOF correspondences to estimate transformations between the coordinate systems.
       *  Use max_no_of_results=-1 to use all.
       *  It is assumed, that the correspondences are sorted from good to bad. */
      void 
      estimatePosesUsing1Correspondence (
          const PointCorrespondences6DVector& correspondences,
          int max_no_of_results, PoseEstimatesVector& pose_estimates) const;

      /** Use pairs of 6DOF correspondences to estimate transformations between the coordinate systems.
       *  It is assumed, that the correspondences are sorted from good to bad. */
      void 
      estimatePosesUsing2Correspondences (
          const PointCorrespondences6DVector& correspondences,
          int max_no_of_tested_combinations, int max_no_of_results,
          PoseEstimatesVector& pose_estimates) const;
      
      /** Use triples of 6DOF correspondences to estimate transformations between the coordinate systems.
       *  It is assumed, that the correspondences are sorted from good to bad. */
      void 
      estimatePosesUsing3Correspondences (
          const PointCorrespondences6DVector& correspondences,
          int max_no_of_tested_combinations, int max_no_of_results,
          PoseEstimatesVector& pose_estimates) const;

      /// Get a reference to the parameters struct
      Parameters& 
      getParameters () { return parameters_; }

    protected:
      // =====PROTECTED MEMBER VARIABLES=====
      Parameters parameters_;

  };

}  // end namespace pcl

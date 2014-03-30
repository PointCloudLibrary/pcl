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
 * $Id: sac_model_line.h 2326 2011-08-31 07:48:25Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_STICK_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_STICK_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/eigen.h>

namespace pcl
{
  /** \brief SampleConsensusModelStick defines a model for 3D stick segmentation. 
    * A stick is a line with an user given minimum/maximum width.
    * The model coefficients are defined as:
    *   - \b point_on_line.x  : the X coordinate of a point on the line
    *   - \b point_on_line.y  : the Y coordinate of a point on the line
    *   - \b point_on_line.z  : the Z coordinate of a point on the line
    *   - \b line_direction.x : the X coordinate of a line's direction
    *   - \b line_direction.y : the Y coordinate of a line's direction
    *   - \b line_direction.z : the Z coordinate of a line's direction
    *   - \b line_width       : the width of the line
    * \author Radu B. Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class SampleConsensusModelStick : public SampleConsensusModel<PointT>
  {
    public:
      using SampleConsensusModel<PointT>::input_;
      using SampleConsensusModel<PointT>::indices_;
      using SampleConsensusModel<PointT>::radius_min_;
      using SampleConsensusModel<PointT>::radius_max_;
      using SampleConsensusModel<PointT>::error_sqr_dists_;

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelStick> Ptr;

      /** \brief Constructor for base SampleConsensusModelStick.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelStick (const PointCloudConstPtr &cloud,
                                 bool random = false) 
        : SampleConsensusModel<PointT> (cloud, random) {};

      /** \brief Constructor for base SampleConsensusModelStick.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelStick (const PointCloudConstPtr &cloud, 
                                 const std::vector<int> &indices,
                                 bool random = false) 
        : SampleConsensusModel<PointT> (cloud, indices, random) {};
      
      /** \brief Empty destructor */
      virtual ~SampleConsensusModelStick () {}

      /** \brief Check whether the given index samples can form a valid stick model, compute the model coefficients from
        * these samples and store them internally in model_coefficients_. The stick coefficients are represented by a
        * point and a line direction
        * \param[in] samples the point indices found as possible good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool 
      computeModelCoefficients (const std::vector<int> &samples, 
                                Eigen::VectorXf &model_coefficients);

      /** \brief Compute all squared distances from the cloud data to a given stick model.
        * \param[in] model_coefficients the coefficients of a stick model that we need to compute distances to
        * \param[out] distances the resultant estimated squared distances
        */
      void 
      getDistancesToModel (const Eigen::VectorXf &model_coefficients, 
                           std::vector<double> &distances);

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param[in] model_coefficients the coefficients of a stick model that we need to compute distances to
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param[out] inliers the resultant model inliers
        */
      void 
      selectWithinDistance (const Eigen::VectorXf &model_coefficients, 
                            const double threshold, 
                            std::vector<int> &inliers);

      /** \brief Count all the points which respect the given model coefficients as inliers. 
        * 
        * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
        * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
        * \return the resultant number of inliers
        */
      virtual int
      countWithinDistance (const Eigen::VectorXf &model_coefficients, 
                           const double threshold);

      /** \brief Recompute the stick coefficients using the given inlier set and return them to the user.
        * @note: these are the coefficients of the stick model after refinement (eg. after SVD)
        * \param[in] inliers the data inliers found as supporting the model
        * \param[in] model_coefficients the initial guess for the model coefficients
        * \param[out] optimized_coefficients the resultant recomputed coefficients after optimization
        */
      void 
      optimizeModelCoefficients (const std::vector<int> &inliers, 
                                 const Eigen::VectorXf &model_coefficients, 
                                 Eigen::VectorXf &optimized_coefficients);

      /** \brief Create a new point cloud with inliers projected onto the stick model.
        * \param[in] inliers the data inliers that we want to project on the stick model
        * \param[in] model_coefficients the *normalized* coefficients of a stick model
        * \param[out] projected_points the resultant projected points
        * \param[in] copy_data_fields set to true if we need to copy the other data fields
        */
      void 
      projectPoints (const std::vector<int> &inliers, 
                     const Eigen::VectorXf &model_coefficients, 
                     PointCloud &projected_points, 
                     bool copy_data_fields = true);

      /** \brief Verify whether a subset of indices verifies the given stick model coefficients.
        * \param[in] indices the data indices that need to be tested against the plane model
        * \param[in] model_coefficients the plane model coefficients
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool 
      doSamplesVerifyModel (const std::set<int> &indices, 
                            const Eigen::VectorXf &model_coefficients, 
                            const double threshold);

      /** \brief Return an unique id for this model (SACMODEL_STACK). */
      inline pcl::SacModel 
      getModelType () const { return (SACMODEL_STICK); }

    protected:
      /** \brief Check whether a model is valid given the user constraints.
        * \param[in] model_coefficients the set of model coefficients
        */
      inline bool 
      isModelValid (const Eigen::VectorXf &model_coefficients)
      {
        if (model_coefficients.size () != 7)
        {
          PCL_ERROR ("[pcl::SampleConsensusModelStick::selectWithinDistance] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
          return (false);
        }

        return (true);
      }

      /** \brief Check if a sample of indices results in a good sample of points
        * indices.
        * \param[in] samples the resultant index samples
        */
      bool
      isSampleGood (const std::vector<int> &samples) const;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_stick.hpp>
#endif

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_STICK_H_

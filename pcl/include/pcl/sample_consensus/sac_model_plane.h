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
 * $Id: sac_model_plane.h 36021 2011-02-17 03:44:01Z vrabaud $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_PLANE_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_PLANE_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>

namespace pcl
{
  /** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
    * \param p a point
    * \param a the normalized <i>a</i> coefficient of a plane
    * \param b the normalized <i>b</i> coefficient of a plane
    * \param c the normalized <i>c</i> coefficient of a plane
    * \param d the normalized <i>d</i> coefficient of a plane
    */
  template <typename Point> inline double
    pointToPlaneDistanceSigned (const Point &p, double a, double b, double c, double d)
  {
    return (a * p.x + b * p.y + c * p.z + d);
  }

  /** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
    * \param p a point
    * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
    */
  template <typename Point> inline double
    pointToPlaneDistanceSigned (const Point &p, const Eigen::Vector4f &plane_coefficients)
  {
    return ( plane_coefficients[0] * p.x + plane_coefficients[1] * p.y + plane_coefficients[2] * p.z + plane_coefficients[3] );
  }

  /** \brief Get the distance from a point to a plane (unsigned) defined by ax+by+cz+d=0
    * \param p a point
    * \param a the normalized <i>a</i> coefficient of a plane
    * \param b the normalized <i>b</i> coefficient of a plane
    * \param c the normalized <i>c</i> coefficient of a plane
    * \param d the normalized <i>d</i> coefficient of a plane
    */
  template <typename Point> inline double
    pointToPlaneDistance (const Point &p, double a, double b, double c, double d)
  {
    return (fabs (pointToPlaneDistanceSigned (p, a, b, c, d)) );
  }

  /** \brief Get the distance from a point to a plane (unsigned) defined by ax+by+cz+d=0
    * \param p a point
    * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
    */
  template <typename Point> inline double
    pointToPlaneDistance (const Point &p, const Eigen::Vector4f &plane_coefficients)
  {
    return ( fabs (pointToPlaneDistanceSigned (p, plane_coefficients)) );
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b SampleConsensusModelPlane defines a model for 3D plane segmentation.
    * \author Radu Bogdan Rusu
    */
  template <typename PointT>
  class SampleConsensusModelPlane : public SampleConsensusModel<PointT>
  {
    public:
      using SampleConsensusModel<PointT>::input_;
      using SampleConsensusModel<PointT>::indices_;

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelPlane> Ptr;

      /** \brief Constructor for base SampleConsensusModelPlane.
        * \param cloud the input point cloud dataset
        */
      SampleConsensusModelPlane (const PointCloudConstPtr &cloud) : SampleConsensusModel<PointT> (cloud) {};

      /** \brief Constructor for base SampleConsensusModelPlane.
        * \param cloud the input point cloud dataset
        * \param indices a vector of point indices to be used from \a cloud
        */
      SampleConsensusModelPlane (const PointCloudConstPtr &cloud, const std::vector<int> &indices) : SampleConsensusModel<PointT> (cloud, indices) {};

      /** \brief Get 3 random non-collinear points as data samples and return them as point indices.
        * \param iterations the internal number of iterations used by SAC methods
        * \param samples the resultant model samples
        * \note assumes unique points!
        */
      void getSamples (int &iterations, std::vector<int> &samples);

      /** \brief Check whether the given index samples can form a valid plane model, compute the model coefficients from
        * these samples and store them internally in model_coefficients_. The plane coefficients are:
        * a, b, c, d (ax+by+cz+d=0)
        * \param samples the point indices found as possible good candidates for creating a valid model
        * \param model_coefficients the resultant model coefficients
        */
      bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients);

      /** \brief Compute all distances from the cloud data to a given plane model.
        * \param model_coefficients the coefficients of a plane model that we need to compute distances to
        * \param distances the resultant estimated distances
        */
      void getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances);

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param model_coefficients the coefficients of a plane model that we need to compute distances to
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param inliers the resultant model inliers
        */
      void selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers);

      /** \brief Recompute the plane coefficients using the given inlier set and return them to the user.
        * @note: these are the coefficients of the plane model after refinement (eg. after SVD)
        * \param inliers the data inliers found as supporting the model
        * \param model_coefficients the initial guess for the model coefficients
        * \param optimized_coefficients the resultant recomputed coefficients after non-linear optimization
        */
      void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients);

      /** \brief Create a new point cloud with inliers projected onto the plane model.
        * \param inliers the data inliers that we want to project on the plane model
        * \param model_coefficients the *normalized* coefficients of a plane model
        * \param projected_points the resultant projected points
        * \param copy_data_fields set to true if we need to copy the other data fields
        */
      void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields = true);

      /** \brief Verify whether a subset of indices verifies the given plane model coefficients.
        * \param indices the data indices that need to be tested against the plane model
        * \param model_coefficients the plane model coefficients
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold);

      /** \brief Return an unique id for this model (SACMODEL_PLANE). */
      inline pcl::SacModel getModelType () const { return (SACMODEL_PLANE); }

    protected:
      /** \brief Check whether a model is valid given the user constraints.
        * \param model_coefficients the set of model coefficients
        */
      inline bool 
        isModelValid (const Eigen::VectorXf &model_coefficients)
      {
        // Needs a valid model coefficients
        if (model_coefficients.size () != 4)
        {
          ROS_ERROR ("[pcl::SampleConsensusModelPlane::isModelValid] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
          return (false);
        }
        return (true);
      }

    private:
      /** \brief Define the maximum number of iterations for collinearity checks */
      const static int MAX_ITERATIONS_COLLINEAR = 1000;
  };
}

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_PLANE_H_

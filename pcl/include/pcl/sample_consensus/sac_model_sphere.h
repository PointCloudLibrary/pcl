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
 * $Id: sac_model_sphere.h 36021 2011-02-17 03:44:01Z vrabaud $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_SPHERE_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_SPHERE_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <cminpack.h>
#include <boost/thread/mutex.hpp>

namespace pcl
{
  /** \brief @b SampleConsensusModelSphere defines a model for 3D sphere segmentation.
    * \author Radu Bogdan Rusu
    */
  template <typename PointT>
  class SampleConsensusModelSphere : public SampleConsensusModel<PointT>
  {
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModel<PointT>::radius_min_;
    using SampleConsensusModel<PointT>::radius_max_;

    public:
      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelSphere> Ptr;

      /** \brief Constructor for base SampleConsensusModelSphere.
        * \param cloud the input point cloud dataset
        */
      SampleConsensusModelSphere (const PointCloudConstPtr &cloud) : SampleConsensusModel<PointT> (cloud) { }

      /** \brief Constructor for base SampleConsensusModelSphere.
        * \param cloud the input point cloud dataset
        * \param indices a vector of point indices to be used from \a cloud
        */
      SampleConsensusModelSphere (const PointCloudConstPtr &cloud, const std::vector<int> &indices) : SampleConsensusModel<PointT> (cloud, indices) { }

      /** \brief Get 4 random points (3 non-collinear) as data samples and return them as point indices.
        * \param iterations the internal number of iterations used by SAC methods
        * \param samples the resultant model samples
        * \note assumes unique points!
        * \note Two different points could be enough in theory, to infere some sort of a center and a radius,
        *       but in practice, we might end up with a lot of points which are just 'close' to one another.
        *       Therefore we have two options:
        *       a) use normal information (good but I wouldn't rely on it in extremely noisy point clouds, no matter what)
        *       b) get two more points and uniquely identify a sphere in space (3 unique points define a circle)
        */
      void 
      getSamples (int &iterations, std::vector<int> &samples);

      /** \brief Check whether the given index samples can form a valid sphere model, compute the model coefficients
        * from these samples and store them internally in model_coefficients. The sphere coefficients are: x, y, z, R.
        * \param samples the point indices found as possible good candidates for creating a valid model
        * \param model_coefficients the resultant model coefficients
        */
      bool 
      computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients);

      /** \brief Compute all distances from the cloud data to a given sphere model.
        * \param model_coefficients the coefficients of a sphere model that we need to compute distances to
        * \param distances the resultant estimated distances
        */
      void 
      getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances);

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param model_coefficients the coefficients of a sphere model that we need to compute distances to
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        * \param inliers the resultant model inliers
        */
      void 
      selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers);

      /** \brief Recompute the sphere coefficients using the given inlier set and return them to the user.
        * @note: these are the coefficients of the sphere model after refinement (eg. after SVD)
        * \param inliers the data inliers found as supporting the model
        * \param model_coefficients the initial guess for the optimization
        * \param optimized_coefficients the resultant recomputed coefficients after non-linear optimization
        */
      void 
      optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients);

      /** \brief Create a new point cloud with inliers projected onto the sphere model.
        * \param inliers the data inliers that we want to project on the sphere model
        * \param model_coefficients the coefficients of a sphere model
        * \param projected_points the resultant projected points
        * \param copy_data_fields set to true if we need to copy the other data fields
        * \todo implement this.
        */
      void 
      projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields = true);

      /** \brief Verify whether a subset of indices verifies the given sphere model coefficients.
        * \param indices the data indices that need to be tested against the sphere model
        * \param model_coefficients the sphere model coefficients
        * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool 
      doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold);

      /** \brief Return an unique id for this model (SACMODEL_SPHERE). */
      inline pcl::SacModel getModelType () const { return (SACMODEL_SPHERE); }

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
          ROS_ERROR ("[pcl::SampleConsensusModelSphere::isModelValid] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
          return (false);
        }

        if (radius_min_ != -DBL_MAX && model_coefficients[3] < radius_min_)
          return (false);
        if (radius_max_ != DBL_MAX && model_coefficients[3] > radius_max_)
          return (false);

        return (true);
      }

    private:
      /** \brief Temporary boost mutex for \a tmp_inliers_ */
      boost::mutex tmp_mutex_;

      /** \brief Temporary pointer to a list of given indices for optimizeModelCoefficients () */
      const std::vector<int> *tmp_inliers_;

      /** \brief Define the maximum number of iterations for collinearity checks */
      const static int MAX_ITERATIONS_COLLINEAR = 1000;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////(
      /** \brief Cost function to be minimized
        * \param p a pointer to our data structure array
        * \param m the number of functions
        * \param n the number of variables
        * \param x a pointer to the variables array
        * \param fvec a pointer to the resultant functions evaluations
        * \param iflag set to -1 inside the function to terminate execution
        */
      static int functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag);
  };
}

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_SPHERE_H_

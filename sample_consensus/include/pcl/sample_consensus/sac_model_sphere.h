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

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_SPHERE_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_SPHERE_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>

namespace pcl
{
  /** \brief SampleConsensusModelSphere defines a model for 3D sphere segmentation.
    * The model coefficients are defined as:
    *   - \b center.x : the X coordinate of the sphere's center
    *   - \b center.y : the Y coordinate of the sphere's center
    *   - \b center.z : the Z coordinate of the sphere's center
    *   - \b radius   : the sphere's radius
    *
    * \author Radu B. Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class SampleConsensusModelSphere : public SampleConsensusModel<PointT>
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

      typedef boost::shared_ptr<SampleConsensusModelSphere> Ptr;

      /** \brief Constructor for base SampleConsensusModelSphere.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelSphere (const PointCloudConstPtr &cloud,
                                  bool random = false) 
        : SampleConsensusModel<PointT> (cloud, random), tmp_inliers_ ()
      {}

      /** \brief Constructor for base SampleConsensusModelSphere.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelSphere (const PointCloudConstPtr &cloud, 
                                  const std::vector<int> &indices,
                                  bool random = false) 
        : SampleConsensusModel<PointT> (cloud, indices, random), tmp_inliers_ ()
      {}
      
      /** \brief Empty destructor */
      virtual ~SampleConsensusModelSphere () {}

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      SampleConsensusModelSphere (const SampleConsensusModelSphere &source) :
        SampleConsensusModel<PointT> (), tmp_inliers_ () 
      {
        *this = source;
      }

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      inline SampleConsensusModelSphere&
      operator = (const SampleConsensusModelSphere &source)
      {
        SampleConsensusModel<PointT>::operator=(source);
        tmp_inliers_ = source.tmp_inliers_;
        return (*this);
      }

      /** \brief Check whether the given index samples can form a valid sphere model, compute the model 
        * coefficients from these samples and store them internally in model_coefficients. 
        * The sphere coefficients are: x, y, z, R.
        * \param[in] samples the point indices found as possible good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool 
      computeModelCoefficients (const std::vector<int> &samples, 
                                Eigen::VectorXf &model_coefficients);

      /** \brief Compute all distances from the cloud data to a given sphere model.
        * \param[in] model_coefficients the coefficients of a sphere model that we need to compute distances to
        * \param[out] distances the resultant estimated distances
        */
      void 
      getDistancesToModel (const Eigen::VectorXf &model_coefficients, 
                           std::vector<double> &distances);

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param[in] model_coefficients the coefficients of a sphere model that we need to compute distances to
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

      /** \brief Recompute the sphere coefficients using the given inlier set and return them to the user.
        * @note: these are the coefficients of the sphere model after refinement (eg. after SVD)
        * \param[in] inliers the data inliers found as supporting the model
        * \param[in] model_coefficients the initial guess for the optimization
        * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
        */
      void 
      optimizeModelCoefficients (const std::vector<int> &inliers, 
                                 const Eigen::VectorXf &model_coefficients, 
                                 Eigen::VectorXf &optimized_coefficients);

      /** \brief Create a new point cloud with inliers projected onto the sphere model.
        * \param[in] inliers the data inliers that we want to project on the sphere model
        * \param[in] model_coefficients the coefficients of a sphere model
        * \param[out] projected_points the resultant projected points
        * \param[in] copy_data_fields set to true if we need to copy the other data fields
        * \todo implement this.
        */
      void 
      projectPoints (const std::vector<int> &inliers, 
                     const Eigen::VectorXf &model_coefficients, 
                     PointCloud &projected_points, 
                     bool copy_data_fields = true);

      /** \brief Verify whether a subset of indices verifies the given sphere model coefficients.
        * \param[in] indices the data indices that need to be tested against the sphere model
        * \param[in] model_coefficients the sphere model coefficients
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool 
      doSamplesVerifyModel (const std::set<int> &indices, 
                            const Eigen::VectorXf &model_coefficients, 
                            const double threshold);

      /** \brief Return an unique id for this model (SACMODEL_SPHERE). */
      inline pcl::SacModel getModelType () const { return (SACMODEL_SPHERE); }

    protected:
      /** \brief Check whether a model is valid given the user constraints.
        * \param[in] model_coefficients the set of model coefficients
        */
      inline bool 
      isModelValid (const Eigen::VectorXf &model_coefficients)
      {
        // Needs a valid model coefficients
        if (model_coefficients.size () != 4)
        {
          PCL_ERROR ("[pcl::SampleConsensusModelSphere::isModelValid] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
          return (false);
        }

        if (radius_min_ != -std::numeric_limits<double>::max() && model_coefficients[3] < radius_min_)
          return (false);
        if (radius_max_ != std::numeric_limits<double>::max() && model_coefficients[3] > radius_max_)
          return (false);

        return (true);
      }

      /** \brief Check if a sample of indices results in a good sample of points
        * indices.
        * \param[in] samples the resultant index samples
        */
      bool
      isSampleGood(const std::vector<int> &samples) const;

    private:
      /** \brief Temporary pointer to a list of given indices for optimizeModelCoefficients () */
      const std::vector<int> *tmp_inliers_;

#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic ignored "-Weffc++"
#endif
      struct OptimizationFunctor : pcl::Functor<float>
      {
        /** Functor constructor
          * \param[in] m_data_points the number of data points to evaluate
          * \param[in] estimator pointer to the estimator object
          * \param[in] distance distance computation function pointer
          */
        OptimizationFunctor (int m_data_points, pcl::SampleConsensusModelSphere<PointT> *model) : 
          pcl::Functor<float>(m_data_points), model_ (model) {}

        /** Cost function to be minimized
          * \param[in] x the variables array
          * \param[out] fvec the resultant functions evaluations
          * \return 0
          */
        int 
        operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
        {
          Eigen::Vector4f cen_t;
          cen_t[3] = 0;
          for (int i = 0; i < values (); ++i)
          {
            // Compute the difference between the center of the sphere and the datapoint X_i
            cen_t[0] = model_->input_->points[(*model_->tmp_inliers_)[i]].x - x[0];
            cen_t[1] = model_->input_->points[(*model_->tmp_inliers_)[i]].y - x[1];
            cen_t[2] = model_->input_->points[(*model_->tmp_inliers_)[i]].z - x[2];
            
            // g = sqrt ((x-a)^2 + (y-b)^2 + (z-c)^2) - R
            fvec[i] = sqrtf (cen_t.dot (cen_t)) - x[3];
          }
          return (0);
        }
        
        pcl::SampleConsensusModelSphere<PointT> *model_;
      };
#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic warning "-Weffc++"
#endif
   };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#endif

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_SPHERE_H_

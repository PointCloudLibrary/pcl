/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_CIRCLE3D_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_CIRCLE3D_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>

namespace pcl
{
  /** \brief SampleConsensusModelCircle3D defines a model for 3D circle segmentation.
    *
    * The model coefficients are defined as:
    *   - \b center.x : the X coordinate of the circle's center
    *   - \b center.y : the Y coordinate of the circle's center
    *   - \b center.z : the Z coordinate of the circle's center 
    *   - \b radius   : the circle's radius
    *   - \b normal.x : the X coordinate of the normal's direction 
    *   - \b normal.y : the Y coordinate of the normal's direction 
    *   - \b normal.z : the Z coordinate of the normal's direction 
    *
    * \author Raoul Hoffmann, Karol Hausman, Radu B. Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class SampleConsensusModelCircle3D : public SampleConsensusModel<PointT>
  {
    public:
      using SampleConsensusModel<PointT>::input_;
      using SampleConsensusModel<PointT>::indices_;
      using SampleConsensusModel<PointT>::radius_min_;
      using SampleConsensusModel<PointT>::radius_max_;

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelCircle3D<PointT> > Ptr;
      typedef boost::shared_ptr<const SampleConsensusModelCircle3D<PointT> > ConstPtr;

      /** \brief Constructor for base SampleConsensusModelCircle3D.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelCircle3D (const PointCloudConstPtr &cloud,
                                    bool random = false) 
        : SampleConsensusModel<PointT> (cloud, random) {};

      /** \brief Constructor for base SampleConsensusModelCircle3D.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelCircle3D (const PointCloudConstPtr &cloud, 
                                    const std::vector<int> &indices,
                                    bool random = false) 
        : SampleConsensusModel<PointT> (cloud, indices, random) {};
      
      /** \brief Empty destructor */
      virtual ~SampleConsensusModelCircle3D () {}

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      SampleConsensusModelCircle3D (const SampleConsensusModelCircle3D &source) :
        SampleConsensusModel<PointT> (), tmp_inliers_ () 
      {
        *this = source;
      }

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      inline SampleConsensusModelCircle3D&
      operator = (const SampleConsensusModelCircle3D &source)
      {
        SampleConsensusModel<PointT>::operator=(source);
        tmp_inliers_ = source.tmp_inliers_;
        return (*this);
      }

      /** \brief Check whether the given index samples can form a valid 2D circle model, compute the model coefficients
        * from these samples and store them in model_coefficients. The circle coefficients are: x, y, R.
        * \param[in] samples the point indices found as possible good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool
      computeModelCoefficients (const std::vector<int> &samples,
                                Eigen::VectorXf &model_coefficients);

      /** \brief Compute all distances from the cloud data to a given 3D circle model.
        * \param[in] model_coefficients the coefficients of a 2D circle model that we need to compute distances to
        * \param[out] distances the resultant estimated distances
        */
      void
      getDistancesToModel (const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances);

      /** \brief Compute all distances from the cloud data to a given 3D circle model.
        * \param[in] model_coefficients the coefficients of a 3D circle model that we need to compute distances to
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

       /** \brief Recompute the 3d circle coefficients using the given inlier set and return them to the user.
        * @note: these are the coefficients of the 3d circle model after refinement (eg. after SVD)
        * \param[in] inliers the data inliers found as supporting the model
        * \param[in] model_coefficients the initial guess for the optimization
        * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
        */
      void
      optimizeModelCoefficients (const std::vector<int> &inliers,
                                 const Eigen::VectorXf &model_coefficients,
                                 Eigen::VectorXf &optimized_coefficients);

      /** \brief Create a new point cloud with inliers projected onto the 3d circle model.
        * \param[in] inliers the data inliers that we want to project on the 3d circle model
        * \param[in] model_coefficients the coefficients of a 3d circle model
        * \param[out] projected_points the resultant projected points
        * \param[in] copy_data_fields set to true if we need to copy the other data fields
        */
      void
      projectPoints (const std::vector<int> &inliers,
                     const Eigen::VectorXf &model_coefficients,
                     PointCloud &projected_points,
                     bool copy_data_fields = true);

      /** \brief Verify whether a subset of indices verifies the given 3d circle model coefficients.
        * \param[in] indices the data indices that need to be tested against the 3d circle model
        * \param[in] model_coefficients the 3d circle model coefficients
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool
      doSamplesVerifyModel (const std::set<int> &indices,
                            const Eigen::VectorXf &model_coefficients,
                            const double threshold);

      /** \brief Return an unique id for this model (SACMODEL_CIRCLE3D). */
      inline pcl::SacModel
      getModelType () const { return (SACMODEL_CIRCLE3D); }

    protected:
      /** \brief Check whether a model is valid given the user constraints.
        * \param[in] model_coefficients the set of model coefficients
        */
      bool
      isModelValid (const Eigen::VectorXf &model_coefficients);

      /** \brief Check if a sample of indices results in a good sample of points indices.
        * \param[in] samples the resultant index samples
        */
      bool
      isSampleGood(const std::vector<int> &samples) const;

    private:
      /** \brief Temporary pointer to a list of given indices for optimizeModelCoefficients () */
      const std::vector<int> *tmp_inliers_;

      /** \brief Functor for the optimization function */
      struct OptimizationFunctor : pcl::Functor<double>
      {
        /** Functor constructor
         * \param[in] m_data_points the number of functions
         * \param[in] estimator pointer to the estimator object
         * \param[in] distance distance computation function pointer
         */
        OptimizationFunctor (int m_data_points, pcl::SampleConsensusModelCircle3D<PointT> *model) :
          pcl::Functor<double> (m_data_points), model_ (model) {}

       /** Cost function to be minimized
         * \param[in] x the variables array
         * \param[out] fvec the resultant functions evaluations
         * \return 0
         */
        int operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
        {
          for (int i = 0; i < values (); ++i)
          {
            // what i have:
            // P : Sample Point
            Eigen::Vector3d P (model_->input_->points[(*model_->tmp_inliers_)[i]].x, model_->input_->points[(*model_->tmp_inliers_)[i]].y, model_->input_->points[(*model_->tmp_inliers_)[i]].z);
            // C : Circle Center
            Eigen::Vector3d C (x[0], x[1], x[2]);
            // N : Circle (Plane) Normal
            Eigen::Vector3d N (x[4], x[5], x[6]);
            // r : Radius
            double r = x[3];

            Eigen::Vector3d helperVectorPC = P - C;
            // 1.1. get line parameter
            //float lambda = (helperVectorPC.dot(N)) / N.squaredNorm() ;
            double lambda = (-(helperVectorPC.dot (N))) / N.dot (N);
            // Projected Point on plane
            Eigen::Vector3d P_proj = P + lambda * N;
            Eigen::Vector3d helperVectorP_projC = P_proj - C;

            // K : Point on Circle
            Eigen::Vector3d K = C + r * helperVectorP_projC.normalized ();
            Eigen::Vector3d distanceVector =  P - K;

            fvec[i] = distanceVector.norm ();
          }
          return (0);
        }

        pcl::SampleConsensusModelCircle3D<PointT> *model_;
      };
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_circle3d.hpp>
#endif

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_CIRCLE3D_H_

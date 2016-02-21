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

#ifndef PCL_SAMPLE_CONSENSUS_MODEL_CYLINDER_H_
#define PCL_SAMPLE_CONSENSUS_MODEL_CYLINDER_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

namespace pcl
{
  /** \brief @b SampleConsensusModelCylinder defines a model for 3D cylinder segmentation.
    * The model coefficients are defined as:
    *   - \b point_on_axis.x  : the X coordinate of a point located on the cylinder axis
    *   - \b point_on_axis.y  : the Y coordinate of a point located on the cylinder axis
    *   - \b point_on_axis.z  : the Z coordinate of a point located on the cylinder axis
    *   - \b axis_direction.x : the X coordinate of the cylinder's axis direction
    *   - \b axis_direction.y : the Y coordinate of the cylinder's axis direction
    *   - \b axis_direction.z : the Z coordinate of the cylinder's axis direction
    *   - \b radius           : the cylinder's radius
    * 
    * \author Radu Bogdan Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT, typename PointNT>
  class SampleConsensusModelCylinder : public SampleConsensusModel<PointT>, public SampleConsensusModelFromNormals<PointT, PointNT>
  {
    public:
      using SampleConsensusModel<PointT>::model_name_;
      using SampleConsensusModel<PointT>::input_;
      using SampleConsensusModel<PointT>::indices_;
      using SampleConsensusModel<PointT>::radius_min_;
      using SampleConsensusModel<PointT>::radius_max_;
      using SampleConsensusModelFromNormals<PointT, PointNT>::normals_;
      using SampleConsensusModelFromNormals<PointT, PointNT>::normal_distance_weight_;
      using SampleConsensusModel<PointT>::error_sqr_dists_;

      typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
      typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
      typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<SampleConsensusModelCylinder> Ptr;

      /** \brief Constructor for base SampleConsensusModelCylinder.
        * \param[in] cloud the input point cloud dataset
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelCylinder (const PointCloudConstPtr &cloud, bool random = false) 
        : SampleConsensusModel<PointT> (cloud, random)
        , SampleConsensusModelFromNormals<PointT, PointNT> ()
        , axis_ (Eigen::Vector3f::Zero ())
        , eps_angle_ (0)
        , tmp_inliers_ ()
      {
        model_name_ = "SampleConsensusModelCylinder";
        sample_size_ = 2;
        model_size_ = 7;
      }

      /** \brief Constructor for base SampleConsensusModelCylinder.
        * \param[in] cloud the input point cloud dataset
        * \param[in] indices a vector of point indices to be used from \a cloud
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensusModelCylinder (const PointCloudConstPtr &cloud, 
                                    const std::vector<int> &indices,
                                    bool random = false) 
        : SampleConsensusModel<PointT> (cloud, indices, random)
        , SampleConsensusModelFromNormals<PointT, PointNT> ()
        , axis_ (Eigen::Vector3f::Zero ())
        , eps_angle_ (0)
        , tmp_inliers_ ()
      {
        model_name_ = "SampleConsensusModelCylinder";
        sample_size_ = 2;
        model_size_ = 7;
      }

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      SampleConsensusModelCylinder (const SampleConsensusModelCylinder &source) :
        SampleConsensusModel<PointT> (),
        SampleConsensusModelFromNormals<PointT, PointNT> (), 
        axis_ (Eigen::Vector3f::Zero ()),
        eps_angle_ (0),
        tmp_inliers_ ()
      {
        *this = source;
        model_name_ = "SampleConsensusModelCylinder";
      }
      
      /** \brief Empty destructor */
      virtual ~SampleConsensusModelCylinder () {}

      /** \brief Copy constructor.
        * \param[in] source the model to copy into this
        */
      inline SampleConsensusModelCylinder&
      operator = (const SampleConsensusModelCylinder &source)
      {
        SampleConsensusModel<PointT>::operator=(source);
        SampleConsensusModelFromNormals<PointT, PointNT>::operator=(source);
        axis_ = source.axis_;
        eps_angle_ = source.eps_angle_;
        tmp_inliers_ = source.tmp_inliers_;
        return (*this);
      }

      /** \brief Set the angle epsilon (delta) threshold.
        * \param[in] ea the maximum allowed difference between the cylinder axis and the given axis.
        */
      inline void 
      setEpsAngle (const double ea) { eps_angle_ = ea; }

      /** \brief Get the angle epsilon (delta) threshold. */
      inline double 
      getEpsAngle () { return (eps_angle_); }

      /** \brief Set the axis along which we need to search for a cylinder direction.
        * \param[in] ax the axis along which we need to search for a cylinder direction
        */
      inline void 
      setAxis (const Eigen::Vector3f &ax) { axis_ = ax; }

      /** \brief Get the axis along which we need to search for a cylinder direction. */
      inline Eigen::Vector3f 
      getAxis ()  { return (axis_); }

      /** \brief Check whether the given index samples can form a valid cylinder model, compute the model coefficients
        * from these samples and store them in model_coefficients. The cylinder coefficients are: point_on_axis,
        * axis_direction, cylinder_radius_R
        * \param[in] samples the point indices found as possible good candidates for creating a valid model
        * \param[out] model_coefficients the resultant model coefficients
        */
      bool 
      computeModelCoefficients (const std::vector<int> &samples, 
                                Eigen::VectorXf &model_coefficients);

      /** \brief Compute all distances from the cloud data to a given cylinder model.
        * \param[in] model_coefficients the coefficients of a cylinder model that we need to compute distances to
        * \param[out] distances the resultant estimated distances
        */
      void 
      getDistancesToModel (const Eigen::VectorXf &model_coefficients, 
                           std::vector<double> &distances);

      /** \brief Select all the points which respect the given model coefficients as inliers.
        * \param[in] model_coefficients the coefficients of a cylinder model that we need to compute distances to
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

      /** \brief Recompute the cylinder coefficients using the given inlier set and return them to the user.
        * @note: these are the coefficients of the cylinder model after refinement (e.g. after SVD)
        * \param[in] inliers the data inliers found as supporting the model
        * \param[in] model_coefficients the initial guess for the optimization
        * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
        */
      void 
      optimizeModelCoefficients (const std::vector<int> &inliers, 
                                 const Eigen::VectorXf &model_coefficients, 
                                 Eigen::VectorXf &optimized_coefficients);


      /** \brief Create a new point cloud with inliers projected onto the cylinder model.
        * \param[in] inliers the data inliers that we want to project on the cylinder model
        * \param[in] model_coefficients the coefficients of a cylinder model
        * \param[out] projected_points the resultant projected points
        * \param[in] copy_data_fields set to true if we need to copy the other data fields
        */
      void 
      projectPoints (const std::vector<int> &inliers, 
                     const Eigen::VectorXf &model_coefficients, 
                     PointCloud &projected_points, 
                     bool copy_data_fields = true);

      /** \brief Verify whether a subset of indices verifies the given cylinder model coefficients.
        * \param[in] indices the data indices that need to be tested against the cylinder model
        * \param[in] model_coefficients the cylinder model coefficients
        * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
        */
      bool 
      doSamplesVerifyModel (const std::set<int> &indices, 
                            const Eigen::VectorXf &model_coefficients, 
                            const double threshold);

      /** \brief Return an unique id for this model (SACMODEL_CYLINDER). */
      inline pcl::SacModel 
      getModelType () const { return (SACMODEL_CYLINDER); }

    protected:
      using SampleConsensusModel<PointT>::sample_size_;
      using SampleConsensusModel<PointT>::model_size_;

      /** \brief Get the distance from a point to a line (represented by a point and a direction)
        * \param[in] pt a point
        * \param[in] model_coefficients the line coefficients (a point on the line, line direction)
        */
      double 
      pointToLineDistance (const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients);

      /** \brief Project a point onto a line given by a point and a direction vector
        * \param[in] pt the input point to project
        * \param[in] line_pt the point on the line (make sure that line_pt[3] = 0 as there are no internal checks!)
        * \param[in] line_dir the direction of the line (make sure that line_dir[3] = 0 as there are no internal checks!)
        * \param[out] pt_proj the resultant projected point
        */
      inline void
      projectPointToLine (const Eigen::Vector4f &pt, 
                          const Eigen::Vector4f &line_pt, 
                          const Eigen::Vector4f &line_dir,
                          Eigen::Vector4f &pt_proj)
      {
        float k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);
        // Calculate the projection of the point on the line
        pt_proj = line_pt + k * line_dir;
      }

      /** \brief Project a point onto a cylinder given by its model coefficients (point_on_axis, axis_direction,
        * cylinder_radius_R)
        * \param[in] pt the input point to project
        * \param[in] model_coefficients the coefficients of the cylinder (point_on_axis, axis_direction, cylinder_radius_R)
        * \param[out] pt_proj the resultant projected point
        */
      void 
      projectPointToCylinder (const Eigen::Vector4f &pt, 
                              const Eigen::VectorXf &model_coefficients, 
                              Eigen::Vector4f &pt_proj);

      /** \brief Get a string representation of the name of this class. */
      PCL_DEPRECATED ("[pcl::SampleConsensusModelCylinder::getName] getName is deprecated. Please use getClassName instead.")
      std::string
      getName () const { return (model_name_); }

    protected:
      /** \brief Check whether a model is valid given the user constraints.
        * \param[in] model_coefficients the set of model coefficients
        */
      virtual bool
      isModelValid (const Eigen::VectorXf &model_coefficients);

      /** \brief Check if a sample of indices results in a good sample of points
        * indices. Pure virtual.
        * \param[in] samples the resultant index samples
        */
      bool
      isSampleGood (const std::vector<int> &samples) const;

    private:
      /** \brief The axis along which we need to search for a plane perpendicular to. */
      Eigen::Vector3f axis_;
    
      /** \brief The maximum allowed difference between the plane normal and the given axis. */
      double eps_angle_;

      /** \brief temporary pointer to a list of given indices for optimizeModelCoefficients () */
      const std::vector<int> *tmp_inliers_;

#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic ignored "-Weffc++"
#endif
      /** \brief Functor for the optimization function */
      struct OptimizationFunctor : pcl::Functor<float>
      {
        /** Functor constructor
          * \param[in] m_data_points the number of data points to evaluate
          * \param[in] estimator pointer to the estimator object
          * \param[in] distance distance computation function pointer
          */
        OptimizationFunctor (int m_data_points, pcl::SampleConsensusModelCylinder<PointT, PointNT> *model) : 
          pcl::Functor<float> (m_data_points), model_ (model) {}

        /** Cost function to be minimized
          * \param[in] x variables array
          * \param[out] fvec resultant functions evaluations
          * \return 0
          */
        int 
        operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
        {
          Eigen::Vector4f line_pt  (x[0], x[1], x[2], 0);
          Eigen::Vector4f line_dir (x[3], x[4], x[5], 0);
          
          for (int i = 0; i < values (); ++i)
          {
            // dist = f - r
            Eigen::Vector4f pt (model_->input_->points[(*model_->tmp_inliers_)[i]].x,
                                model_->input_->points[(*model_->tmp_inliers_)[i]].y,
                                model_->input_->points[(*model_->tmp_inliers_)[i]].z, 0);

            fvec[i] = static_cast<float> (pcl::sqrPointToLineDistance (pt, line_pt, line_dir) - x[6]*x[6]);
          }
          return (0);
        }

        pcl::SampleConsensusModelCylinder<PointT, PointNT> *model_;
      };
#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic warning "-Weffc++"
#endif
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#endif

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_MODEL_CYLINDER_H_

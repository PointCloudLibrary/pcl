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
 * $Id$
 *
 */

#pragma once

#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

// Sample Consensus methods
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac.h>
// Sample Consensus models
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>

#include <pcl/search/search.h>

namespace pcl
{
  /** \brief @b SACSegmentation represents the Nodelet segmentation class for
    * Sample Consensus methods and models, in the sense that it just creates a
    * Nodelet wrapper for generic-purpose SAC-based segmentation.
    * \author Radu Bogdan Rusu
    * \ingroup segmentation
    */
  template <typename PointT>
  class SACSegmentation : public PCLBase<PointT>
  {
    using PCLBase<PointT>::initCompute;
    using PCLBase<PointT>::deinitCompute;

     public:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;

      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;
      using SearchPtr = typename pcl::search::Search<PointT>::Ptr;

      using SampleConsensusPtr = typename SampleConsensus<PointT>::Ptr;
      using SampleConsensusModelPtr = typename SampleConsensusModel<PointT>::Ptr;

      /** \brief Empty constructor. 
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SACSegmentation (bool random = false) 
        : model_ ()
        , sac_ ()
        , model_type_ (-1)
        , method_type_ (0)
        , threshold_ (0)
        , optimize_coefficients_ (true)
        , radius_min_ (-std::numeric_limits<double>::max ())
        , radius_max_ (std::numeric_limits<double>::max ())
        , samples_radius_ (0.0)
        , samples_radius_search_ ()
        , eps_angle_ (0.0)
        , axis_ (Eigen::Vector3f::Zero ())
        , max_iterations_ (50)
        , threads_ (-1)
        , probability_ (0.99)
        , random_ (random)
      {
      }

      /** \brief Empty destructor. */
      ~SACSegmentation () { /*srv_.reset ();*/ };

      /** \brief The type of model to use (user given parameter).
        * \param[in] model the model type (check \a model_types.h)
        */
      inline void 
      setModelType (int model) { model_type_ = model; }

      /** \brief Get the type of SAC model used. */
      inline int 
      getModelType () const { return (model_type_); }

      /** \brief Get a pointer to the SAC method used. */
      inline SampleConsensusPtr 
      getMethod () const { return (sac_); }

      /** \brief Get a pointer to the SAC model used. */
      inline SampleConsensusModelPtr 
      getModel () const { return (model_); }

      /** \brief The type of sample consensus method to use (user given parameter).
        * \param[in] method the method type (check \a method_types.h)
        */
      inline void 
      setMethodType (int method) { method_type_ = method; }

      /** \brief Get the type of sample consensus method used. */
      inline int 
      getMethodType () const { return (method_type_); }

      /** \brief Distance to the model threshold (user given parameter).
        * \param[in] threshold the distance threshold to use
        */
      inline void 
      setDistanceThreshold (double threshold) { threshold_ = threshold; }

      /** \brief Get the distance to the model threshold. */
      inline double 
      getDistanceThreshold () const { return (threshold_); }

      /** \brief Set the maximum number of iterations before giving up.
        * \param[in] max_iterations the maximum number of iterations the sample consensus method will run
        */
      inline void 
      setMaxIterations (int max_iterations) { max_iterations_ = max_iterations; }

      /** \brief Get maximum number of iterations before giving up. */
      inline int 
      getMaxIterations () const { return (max_iterations_); }

      /** \brief Set the probability of choosing at least one sample free from outliers.
        * \param[in] probability the model fitting probability
        */
      inline void 
      setProbability (double probability) { probability_ = probability; }

      /** \brief Get the probability of choosing at least one sample free from outliers. */
      inline double 
      getProbability () const { return (probability_); }

      /** \brief Set the number of threads to use or turn off parallelization.
        * \param[in] nr_threads the number of hardware threads to use (0 sets the value automatically, a negative number turns parallelization off)
        * \note Not all SAC methods have a parallel implementation. Some will ignore this setting.
        */
      inline void
      setNumberOfThreads (const int nr_threads = -1) { threads_ = nr_threads; }

      /** \brief Set to true if a coefficient refinement is required.
        * \param[in] optimize true for enabling model coefficient refinement, false otherwise
        */
      inline void 
      setOptimizeCoefficients (bool optimize) { optimize_coefficients_ = optimize; }

      /** \brief Get the coefficient refinement internal flag. */
      inline bool 
      getOptimizeCoefficients () const { return (optimize_coefficients_); }

      /** \brief Set the minimum and maximum allowable radius limits for the model (applicable to models that estimate
        * a radius)
        * \param[in] min_radius the minimum radius model
        * \param[in] max_radius the maximum radius model
        */
      inline void
      setRadiusLimits (const double &min_radius, const double &max_radius)
      {
        radius_min_ = min_radius;
        radius_max_ = max_radius;
      }

      /** \brief Get the minimum and maximum allowable radius limits for the model as set by the user.
        * \param[out] min_radius the resultant minimum radius model
        * \param[out] max_radius the resultant maximum radius model
        */
      inline void
      getRadiusLimits (double &min_radius, double &max_radius)
      {
        min_radius = radius_min_;
        max_radius = radius_max_;
      }

      /** \brief Set the maximum distance allowed when drawing random samples
        * \param[in] radius the maximum distance (L2 norm)
        * \param search
        */
      inline void
      setSamplesMaxDist (const double &radius, SearchPtr search)
      {
        samples_radius_ = radius;
        samples_radius_search_ = search;
      }

      /** \brief Get maximum distance allowed when drawing random samples
        *
        * \param[out] radius the maximum distance (L2 norm)
        */
      inline void
      getSamplesMaxDist (double &radius)
      {
        radius = samples_radius_;
      }

      /** \brief Set the axis along which we need to search for a model perpendicular to.
        * \param[in] ax the axis along which we need to search for a model perpendicular to
        */
      inline void 
      setAxis (const Eigen::Vector3f &ax) { axis_ = ax; }

      /** \brief Get the axis along which we need to search for a model perpendicular to. */
      inline Eigen::Vector3f 
      getAxis () const { return (axis_); }

      /** \brief Set the angle epsilon (delta) threshold.
        * \param[in] ea the maximum allowed difference between the model normal and the given axis in radians.
        */
      inline void 
      setEpsAngle (double ea) { eps_angle_ = ea; }

      /** \brief Get the epsilon (delta) model angle threshold in radians. */
      inline double 
      getEpsAngle () const { return (eps_angle_); }

      /** \brief Base method for segmentation of a model in a PointCloud given by <setInputCloud (), setIndices ()>
        * \param[out] inliers the resultant point indices that support the model found (inliers)
        * \param[out] model_coefficients the resultant model coefficients
        */
      virtual void 
      segment (PointIndices &inliers, ModelCoefficients &model_coefficients);

    protected:
      /** \brief Initialize the Sample Consensus model and set its parameters.
        * \param[in] model_type the type of SAC model that is to be used
        */
      virtual bool 
      initSACModel (const int model_type);

      /** \brief Initialize the Sample Consensus method and set its parameters.
        * \param[in] method_type the type of SAC method to be used
        */
      virtual void 
      initSAC (const int method_type);

      /** \brief The model that needs to be segmented. */
      SampleConsensusModelPtr model_;

      /** \brief The sample consensus segmentation method. */
      SampleConsensusPtr sac_;

      /** \brief The type of model to use (user given parameter). */
      int model_type_;

      /** \brief The type of sample consensus method to use (user given parameter). */
      int method_type_;

      /** \brief Distance to the model threshold (user given parameter). */
      double threshold_;

      /** \brief Set to true if a coefficient refinement is required. */
      bool optimize_coefficients_;

      /** \brief The minimum and maximum radius limits for the model. Applicable to all models that estimate a radius. */
      double radius_min_, radius_max_;

      /** \brief The maximum distance of subsequent samples from the first (radius search) */
      double samples_radius_;

      /** \brief The search object for picking subsequent samples using radius search */
      SearchPtr samples_radius_search_;

      /** \brief The maximum allowed difference between the model normal and the given axis. */
      double eps_angle_;

      /** \brief The axis along which we need to search for a model perpendicular to. */
      Eigen::Vector3f axis_;

      /** \brief Maximum number of iterations before giving up (user given parameter). */
      int max_iterations_;

      /** \brief The number of threads the scheduler should use, or a negative number if no parallelization is wanted. */
      int threads_;

      /** \brief Desired probability of choosing at least one sample free from outliers (user given parameter). */
      double probability_;

      /** \brief Set to true if we need a random seed. */
      bool random_;

      /** \brief Class get name method. */
      virtual std::string 
      getClassName () const { return ("SACSegmentation"); }
  };

  /** \brief @b SACSegmentationFromNormals represents the PCL nodelet segmentation class for Sample Consensus methods and
    * models that require the use of surface normals for estimation.
    * \ingroup segmentation
    */
  template <typename PointT, typename PointNT>
  class SACSegmentationFromNormals: public SACSegmentation<PointT>
  {
    using SACSegmentation<PointT>::model_;
    using SACSegmentation<PointT>::model_type_;
    using SACSegmentation<PointT>::radius_min_;
    using SACSegmentation<PointT>::radius_max_;
    using SACSegmentation<PointT>::eps_angle_;
    using SACSegmentation<PointT>::axis_;
    using SACSegmentation<PointT>::random_;

    public:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;

      using PointCloud = typename SACSegmentation<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      using PointCloudN = pcl::PointCloud<PointNT>;
      using PointCloudNPtr = typename PointCloudN::Ptr;
      using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

      using SampleConsensusPtr = typename SampleConsensus<PointT>::Ptr;
      using SampleConsensusModelPtr = typename SampleConsensusModel<PointT>::Ptr;
      using SampleConsensusModelFromNormalsPtr = typename SampleConsensusModelFromNormals<PointT, PointNT>::Ptr;

      /** \brief Empty constructor.
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SACSegmentationFromNormals (bool random = false) 
        : SACSegmentation<PointT> (random)
        , normals_ ()
        , distance_weight_ (0.1)
        , distance_from_origin_ (0)
        , min_angle_ (0.0)
        , max_angle_ (M_PI_2)
      {};

      /** \brief Provide a pointer to the input dataset that contains the point normals of 
        * the XYZ dataset.
        * \param[in] normals the const shared pointer to a PointCloud message
        */
      inline void 
      setInputNormals (const PointCloudNConstPtr &normals) { normals_ = normals; }

      /** \brief Get a pointer to the normals of the input XYZ point cloud dataset. */
      inline PointCloudNConstPtr 
      getInputNormals () const { return (normals_); }

      /** \brief Set the relative weight (between 0 and 1) to give to the angular 
        * distance (0 to pi/2) between point normals and the plane normal.
        * \param[in] distance_weight the distance/angular weight
        */
      inline void 
      setNormalDistanceWeight (double distance_weight) { distance_weight_ = distance_weight; }

      /** \brief Get the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point
        * normals and the plane normal. */
      inline double 
      getNormalDistanceWeight () const { return (distance_weight_); }

      /** \brief Set the minimum opning angle for a cone model.
        * \param min_angle the opening angle which we need minimum to validate a cone model.
        * \param max_angle the opening angle which we need maximum to validate a cone model.
        */
      inline void
      setMinMaxOpeningAngle (const double &min_angle, const double &max_angle)
      {
        min_angle_ = min_angle;
        max_angle_ = max_angle;
      }
 
      /** \brief Get the opening angle which we need minimum to validate a cone model. */
      inline void
      getMinMaxOpeningAngle (double &min_angle, double &max_angle)
      {
        min_angle = min_angle_;
        max_angle = max_angle_;
      }

      /** \brief Set the distance we expect a plane model to be from the origin
        * \param[in] d distance from the template plane model to the origin
        */
      inline void
      setDistanceFromOrigin (const double d) { distance_from_origin_ = d; }

      /** \brief Get the distance of a plane model from the origin. */
      inline double
      getDistanceFromOrigin () const { return (distance_from_origin_); }

    protected:
      /** \brief A pointer to the input dataset that contains the point normals of the XYZ dataset. */
      PointCloudNConstPtr normals_;

      /** \brief The relative weight (between 0 and 1) to give to the angular
        * distance (0 to pi/2) between point normals and the plane normal. 
        */
      double distance_weight_;

      /** \brief The distance from the template plane to the origin. */
      double distance_from_origin_;

      /** \brief The minimum and maximum allowed opening angle of valid cone model. */
      double min_angle_;
      double max_angle_;

      /** \brief Initialize the Sample Consensus model and set its parameters.
        * \param[in] model_type the type of SAC model that is to be used
        */
      bool 
      initSACModel (const int model_type) override;

      /** \brief Class get name method. */
      std::string 
      getClassName () const override { return ("SACSegmentationFromNormals"); }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#endif

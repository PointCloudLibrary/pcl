/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/filters/filter_indices.h>
#include <pcl/ModelCoefficients.h>

// Sample Consensus models
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>

namespace pcl
{
  /** \brief @b ModelOutlierRemoval filters points in a cloud based on the distance between model and point.
   * \details Iterates through the entire input once, automatically filtering non-finite points and the points outside
   * the model specified by setSampleConsensusModelPointer() and the threshold specified by setThreholdFunctionPointer().
   * <br><br>
   * Usage example:
   * \code
   * pcl::ModelCoefficients model_coeff;
   * model_coeff.values.resize(4);
   * model_coeff.values[0] = 0; model_coeff.values[1] = 0; model_coeff.values[2] = 1.5; model_coeff.values[3] = 0.5;
   * pcl::ModelOutlierRemoval<pcl::PointXYZ> filter;
   * filter.setModelCoefficients (model_coeff);
   * filter.setThreshold (0.1);
   * filter.setModelType (pcl::SACMODEL_PLANE);
   * filter.setInputCloud (*cloud_in);
   * filter.setFilterLimitsNegative (false);
   * filter.filter (*cloud_out);
   * \endcode
   */
  template <typename PointT>
  class ModelOutlierRemoval : public FilterIndices<PointT>
  {
    protected:
      using PointCloud = typename FilterIndices<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;
      using SampleConsensusModelPtr = typename SampleConsensusModel<PointT>::Ptr;

    public:
      using PointCloudNPtr = pcl::PointCloud<pcl::Normal>::Ptr;
      using PointCloudNConstPtr = pcl::PointCloud<pcl::Normal>::ConstPtr;

      /** \brief Constructor.
       * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
       */
      inline
      ModelOutlierRemoval (bool extract_removed_indices = false) :
          FilterIndices<PointT> (extract_removed_indices)
      {
        thresh_ = 0;
        normals_distance_weight_ = 0;
        filter_name_ = "ModelOutlierRemoval";
        setThresholdFunction (&pcl::ModelOutlierRemoval<PointT>::checkSingleThreshold, *this);
      }

      /** \brief sets the models coefficients */
      inline void
      setModelCoefficients (const pcl::ModelCoefficients& model_coefficients)
      {
        model_coefficients_.resize (model_coefficients.values.size ());
        for (std::size_t i = 0; i < model_coefficients.values.size (); i++)
        {
          model_coefficients_[i] = model_coefficients.values[i];
        }
      }

      /** \brief returns the models coefficients
       */
      inline pcl::ModelCoefficients
      getModelCoefficients () const
      {
        pcl::ModelCoefficients mc;
        mc.values.resize (model_coefficients_.size ());
        for (std::size_t i = 0; i < mc.values.size (); i++)
          mc.values[i] = model_coefficients_[i];
        return (mc);
      }

      /** \brief Set the type of SAC model used. */
      inline void
      setModelType (pcl::SacModel model)
      {
        model_type_ = model;
      }

      /** \brief Get the type of SAC model used. */
      inline pcl::SacModel
      getModelType () const
      {
        return (model_type_);
      }

      /** \brief Set the thresholdfunction*/
      inline void
      setThreshold (float thresh)
      {
        thresh_ = thresh;
      }

      /** \brief Get the thresholdfunction*/
      inline float
      getThreshold () const
      {
        return (thresh_);
      }

      /** \brief Set the normals cloud*/
      inline void
      setInputNormals (const PointCloudNConstPtr normals_ptr)
      {
        cloud_normals_ = normals_ptr;
      }

      /** \brief Get the normals cloud*/
      inline PointCloudNConstPtr
      getInputNormals () const
      {
        return (cloud_normals_);
      }

      /** \brief Set the normals distance weight*/
      inline void
      setNormalDistanceWeight (const double weight)
      {
        normals_distance_weight_ = weight;
      }

      /** \brief get the normal distance weight*/
      inline double
      getNormalDistanceWeight () const
      {
        return (normals_distance_weight_);
      }

      /** \brief Register a different threshold function
       * \param[in] thresh pointer to a threshold function
       */
      void
      setThresholdFunction (std::function<bool (double)> thresh)
      {
        threshold_function_ = thresh;
      }

      /** \brief Register a different threshold function
       * \param[in] thresh_function pointer to a threshold function
       * \param[in] instance
       */
      template <typename T> void
      setThresholdFunction (bool (T::*thresh_function) (double), T& instance)
      {
        setThresholdFunction ([=, &instance] (double threshold) { return (instance.*thresh_function) (threshold); });
      }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Filtered results are indexed by an indices array.
       * \param[out] indices The resultant indices.
       */
      void
      applyFilter (Indices &indices) override
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
       * \param[out] indices The resultant indices.
       */
      void
      applyFilterIndices (Indices &indices);

    protected:
      double normals_distance_weight_;
      PointCloudNConstPtr cloud_normals_;

      /** \brief The model used to calculate distances */
      SampleConsensusModelPtr model_;

      /** \brief The threshold used to separate outliers (removed_indices) from inliers (indices) */
      float thresh_;

      /** \brief The model coefficients */
      Eigen::VectorXf model_coefficients_;

      /** \brief The type of model to use (user given parameter). */
      pcl::SacModel model_type_;
      std::function<bool (double)> threshold_function_;

      inline bool
      checkSingleThreshold (double value)
      {
        return (value < thresh_);
      }

    private:
      virtual bool
      initSACModel (pcl::SacModel model_type);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/model_outlier_removal.hpp>
#endif

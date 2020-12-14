/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
// Sample Consensus models
#include <pcl/sample_consensus/sac_model.h>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ProjectInliers uses a model and a set of inlier indices from a PointCloud to project them into a
    * separate PointCloud.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<typename PointT>
  class ProjectInliers : public Filter<PointT>
  {
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;

    using PointCloud = typename Filter<PointT>::PointCloud;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using SampleConsensusModelPtr = typename SampleConsensusModel<PointT>::Ptr;
    public:

      using Ptr = shared_ptr<ProjectInliers<PointT> >;
      using ConstPtr = shared_ptr<const ProjectInliers<PointT> >;


      /** \brief Empty constructor. */
      ProjectInliers () : sacmodel_ (), model_type_ (), copy_all_data_ (false)
      {
        filter_name_ = "ProjectInliers";
      }
      
      /** \brief Empty destructor */
      ~ProjectInliers () {}

      /** \brief The type of model to use (user given parameter).
        * \param model the model type (check \a model_types.h)
        */
      inline void
      setModelType (int model)
      {
        model_type_ = model;
      }

      /** \brief Get the type of SAC model used. */
      inline int
      getModelType ()
      {
        return (model_type_);
      }

      /** \brief Provide a pointer to the model coefficients.
        * \param model a pointer to the model coefficients
        */
      inline void
      setModelCoefficients (const ModelCoefficientsConstPtr &model)
      {
        model_ = model;
      }

      /** \brief Get a pointer to the model coefficients. */
      inline ModelCoefficientsConstPtr
      getModelCoefficients ()
      {
        return (model_);
      }

      /** \brief Set whether all data will be returned, or only the projected inliers.
        * \param val true if all data should be returned, false if only the projected inliers
        */
      inline void
      setCopyAllData (bool val)
      {
        copy_all_data_ = val;
      }

      /** \brief Get whether all data is being copied (true), or only the projected inliers (false). */
      inline bool
      getCopyAllData ()
      {
        return (copy_all_data_);
      }
    protected:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Project point indices into a separate PointCloud
        * \param output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output) override;

    private:
      /** \brief A pointer to the vector of model coefficients. */
      ModelCoefficientsConstPtr model_;

      /** \brief The model that needs to be segmented. */
      SampleConsensusModelPtr sacmodel_;

      /** \brief The type of model to use (user given parameter). */
      int model_type_;

      /** \brief True if all data will be returned, false if only the projected inliers. Default: false. */
      bool copy_all_data_;

      /** \brief Initialize the Sample Consensus model and set its parameters.
        * \param model_type the type of SAC model that is to be used
        */
      virtual bool
      initSACModel (int model_type);
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ProjectInliers uses a model and a set of inlier indices from a PointCloud to project them into a
    * separate PointCloud.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS ProjectInliers<pcl::PCLPointCloud2> : public Filter<pcl::PCLPointCloud2>
  {
    using Filter<pcl::PCLPointCloud2>::filter_name_;
    using Filter<pcl::PCLPointCloud2>::getClassName;

    using PCLPointCloud2 = pcl::PCLPointCloud2;
    using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
    using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

    using SampleConsensusModelPtr = SampleConsensusModel<PointXYZ>::Ptr;

    public:
      /** \brief Empty constructor. */
      ProjectInliers () : model_type_ (), copy_all_data_ (false), copy_all_fields_ (true)
      {
        filter_name_ = "ProjectInliers";
      }
      
      /** \brief Empty destructor */
      ~ProjectInliers () {}

      /** \brief The type of model to use (user given parameter).
        * \param[in] model the model type (check \a model_types.h)
        */
      inline void
      setModelType (int model)
      {
        model_type_ = model;
      }

      /** \brief Get the type of SAC model used. */
      inline int
      getModelType () const
      {
        return (model_type_);
      }

      /** \brief Provide a pointer to the model coefficients.
        * \param[in] model a pointer to the model coefficients
        */
      inline void
      setModelCoefficients (const ModelCoefficientsConstPtr &model)
      {
        model_ = model;
      }

      /** \brief Get a pointer to the model coefficients. */
      inline ModelCoefficientsConstPtr
      getModelCoefficients () const
      {
        return (model_);
      }

      /** \brief Set whether all fields should be copied, or only the XYZ.
        * \param[in] val true if all fields will be returned, false if only XYZ
        */
      inline void
      setCopyAllFields (bool val)
      {
        copy_all_fields_ = val;
      }

      /** \brief Get whether all fields are being copied (true), or only XYZ (false). */
      inline bool
      getCopyAllFields () const
      {
        return (copy_all_fields_);
      }

      /** \brief Set whether all data will be returned, or only the projected inliers.
        * \param[in] val true if all data should be returned, false if only the projected inliers
        */
      inline void
      setCopyAllData (bool val)
      {
        copy_all_data_ = val;
      }

      /** \brief Get whether all data is being copied (true), or only the projected inliers (false). */
      inline bool
      getCopyAllData () const
      {
        return (copy_all_data_);
      }
    protected:
      /** \brief The type of model to use (user given parameter). */
      int model_type_;

      /** \brief True if all data will be returned, false if only the projected inliers. Default: false. */
      bool copy_all_data_;

      /** \brief True if all fields will be returned, false if only XYZ. Default: true. */
      bool copy_all_fields_;

      /** \brief A pointer to the vector of model coefficients. */
      ModelCoefficientsConstPtr model_;

      void
      applyFilter (PCLPointCloud2 &output) override;

    private:
      /** \brief The model that needs to be segmented. */
      SampleConsensusModelPtr sacmodel_;

      virtual bool
      initSACModel (int model_type);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/project_inliers.hpp>
#endif

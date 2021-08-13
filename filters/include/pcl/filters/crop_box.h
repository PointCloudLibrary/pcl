/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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
 * $Id: crop_box.h 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#pragma once

#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief CropBox is a filter that allows the user to filter all the data
    * inside of a given box.
    *
    * \author Justin Rosen
    * \ingroup filters
    */
  template<typename PointT>
  class CropBox : public FilterIndices<PointT>
  {
    using Filter<PointT>::getClassName;

    using PointCloud = typename Filter<PointT>::PointCloud;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    public:

      using Ptr = shared_ptr<CropBox<PointT> >;
      using ConstPtr = shared_ptr<const CropBox<PointT> >;

      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
      CropBox (bool extract_removed_indices = false) :
        FilterIndices<PointT> (extract_removed_indices),
        min_pt_ (Eigen::Vector4f (-1, -1, -1, 1)),
        max_pt_ (Eigen::Vector4f (1, 1, 1, 1)),
        rotation_ (Eigen::Vector3f::Zero ()),
        translation_ (Eigen::Vector3f::Zero ()),
        transform_ (Eigen::Affine3f::Identity ())
      {
        filter_name_ = "CropBox";
      }

      /** \brief Set the minimum point of the box
        * \param[in] min_pt the minimum point of the box
        */
      inline void
      setMin (const Eigen::Vector4f &min_pt)
      {
        min_pt_ = min_pt;
      }

      /** \brief Get the value of the minimum point of the box, as set by the user
        * \return the value of the internal \a min_pt parameter.
        */
      inline Eigen::Vector4f
      getMin () const
      {
        return (min_pt_);
      }

      /** \brief Set the maximum point of the box
        * \param[in] max_pt the maximum point of the box
        */
      inline void
      setMax (const Eigen::Vector4f &max_pt)
      {
        max_pt_ = max_pt;
      }

      /** \brief Get the value of the maximum point of the box, as set by the user
        * \return the value of the internal \a max_pt parameter.
        */
      inline Eigen::Vector4f
      getMax () const
      {
        return (max_pt_);
      }

      /** \brief Set a translation value for the box
        * \param[in] translation the (tx,ty,tz) values that the box should be translated by
        */
      inline void
      setTranslation (const Eigen::Vector3f &translation)
      {
        translation_ = translation;
      }

      /** \brief Get the value of the box translation parameter as set by the user. */
      Eigen::Vector3f
      getTranslation () const
      {
        return (translation_);
      }

      /** \brief Set a rotation value for the box
        * \param[in] rotation the (rx,ry,rz) values that the box should be rotated by
        */
      inline void
      setRotation (const Eigen::Vector3f &rotation)
      {
        rotation_ = rotation;
      }

      /** \brief Get the value of the box rotatation parameter, as set by the user. */
      inline Eigen::Vector3f
      getRotation () const
      {
        return (rotation_);
      }

      /** \brief Set a transformation that should be applied to the cloud before filtering
        * \param[in] transform an affine transformation that needs to be applied to the cloud before filtering
        */
      inline void
      setTransform (const Eigen::Affine3f &transform)
      {
        transform_ = transform;
      }

      /** \brief Get the value of the transformation parameter, as set by the user. */
      inline Eigen::Affine3f
      getTransform () const
      {
        return (transform_);
      }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Sample of point indices
        * \param[out] indices the resultant point cloud indices
        */
      void
      applyFilter (Indices &indices) override;
    private:
      /** \brief The minimum point of the box. */
      Eigen::Vector4f min_pt_;
      /** \brief The maximum point of the box. */
      Eigen::Vector4f max_pt_;
      /** \brief The 3D rotation for the box. */
      Eigen::Vector3f rotation_;
      /** \brief The 3D translation for the box. */
      Eigen::Vector3f translation_;
      /** \brief The affine transform applied to the cloud. */
      Eigen::Affine3f transform_;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief CropBox is a filter that allows the user to filter all the data
    * inside of a given box.
    *
    * \author Justin Rosen
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS CropBox<pcl::PCLPointCloud2> : public FilterIndices<pcl::PCLPointCloud2>
  {
    using Filter<pcl::PCLPointCloud2>::filter_name_;
    using Filter<pcl::PCLPointCloud2>::getClassName;

    using PCLPointCloud2 = pcl::PCLPointCloud2;
    using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
    using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

    public:
      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
       CropBox (bool extract_removed_indices = false) :
        FilterIndices<pcl::PCLPointCloud2>::FilterIndices (extract_removed_indices),
        min_pt_(Eigen::Vector4f (-1, -1, -1, 1)),
        max_pt_(Eigen::Vector4f (1, 1, 1, 1)),
        translation_ (Eigen::Vector3f::Zero ()),
        rotation_ (Eigen::Vector3f::Zero ()),
        transform_(Eigen::Affine3f::Identity ())
      {
        filter_name_ = "CropBox";
      }

      /** \brief Set the minimum point of the box
        * \param[in] min_pt the minimum point of the box
        */
      inline void
      setMin (const Eigen::Vector4f& min_pt)
      {
        min_pt_ = min_pt;
      }

      /** \brief Get the value of the minimum point of the box, as set by the user
        * \return the value of the internal \a min_pt parameter.
        */
      inline Eigen::Vector4f
      getMin () const
      {
        return (min_pt_);
      }

      /** \brief Set the maximum point of the box
        * \param[in] max_pt the maximum point of the box
        */
      inline void
      setMax (const Eigen::Vector4f &max_pt)
      {
        max_pt_ = max_pt;
      }

      /** \brief Get the value of the maxiomum point of the box, as set by the user
        * \return the value of the internal \a max_pt parameter.
        */
      inline Eigen::Vector4f
      getMax () const
      {
        return (max_pt_);
      }

      /** \brief Set a translation value for the box
        * \param[in] translation the (tx,ty,tz) values that the box should be translated by
        */
      inline void
      setTranslation (const Eigen::Vector3f &translation)
      {
        translation_ = translation;
      }

      /** \brief Get the value of the box translation parameter as set by the user. */
      inline Eigen::Vector3f
      getTranslation () const
      {
        return (translation_);
      }

      /** \brief Set a rotation value for the box
        * \param[in] rotation the (rx,ry,rz) values that the box should be rotated by
        */
      inline void
      setRotation (const Eigen::Vector3f &rotation)
      {
        rotation_ = rotation;
      }

      /** \brief Get the value of the box rotatation parameter, as set by the user. */
      inline Eigen::Vector3f
      getRotation () const
      {
        return (rotation_);
      }

      /** \brief Set a transformation that should be applied to the cloud before filtering
        * \param[in] transform an affine transformation that needs to be applied to the cloud before filtering
        */
      inline void
      setTransform (const Eigen::Affine3f &transform)
      {
        transform_ = transform;
      }

      /** \brief Get the value of the transformation parameter, as set by the user. */
      inline Eigen::Affine3f
      getTransform () const
      {
        return (transform_);
      }

    protected:
      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PCLPointCloud2 &output) override;

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (Indices &indices) override;

      /** \brief The minimum point of the box. */
      Eigen::Vector4f min_pt_;
      /** \brief The maximum point of the box. */
      Eigen::Vector4f max_pt_;
      /** \brief The 3D translation for the box. */
      Eigen::Vector3f translation_;
      /** \brief The 3D rotation for the box. */
      Eigen::Vector3f rotation_;
      /** \brief The affine transform applied to the cloud. */
      Eigen::Affine3f transform_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/crop_box.hpp>
#endif

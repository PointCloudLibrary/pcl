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
 * $Id: crop_box.h 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#ifndef PCL_FILTERS_CROP_BOX_H_
#define PCL_FILTERS_CROP_BOX_H_

#include "pcl/point_types.h"
#include "pcl/filters/filter_indices.h"
#include "pcl/common/transforms.h"
#include "pcl/common/eigen.h"
//#include <Eigen/Geometry>

namespace pcl
{
  /** \brief @b CropBox 
    * \author Justin Rosen
    * \ingroup filters
    */
  template<typename PointT>
  class CropBox : public FilterIndices<PointT>
  {
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;
    using Filter<PointT>::indices_;
    using Filter<PointT>::input_;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      CropBox () :
        min_pt_(Eigen::Vector4f (-1, -1, -1, 1)),
        max_pt_(Eigen::Vector4f (1, 1, 1, 1)),
        transform_(Eigen::Affine3f::Identity ())
      {
        filter_name_ = "CropBox";
        rotation_ = Eigen::Vector3f::Zero ();
        translation_ = Eigen::Vector3f::Zero ();
      }

      /** \brief
        * \param min_pt
        */
      inline void
      setMin (const Eigen::Vector4f &min_pt)
      {
        min_pt_ = min_pt;
      }

      /** \brief Get the value of the internal \a min_pt parameter.
        */
      Eigen::Vector4f
      getMin ()
      {
        return (min_pt_);
      }

      /** \brief
        * \param max_pt
        */
      inline void
      setMax (const Eigen::Vector4f &max_pt)
      {
        max_pt_ = max_pt;
      }

      /** \brief Get the value of the internal \a min_pt parameter.
        */
      Eigen::Vector4f
      getMax ()
      {
        return (max_pt_);
      }

      /** \brief
        * \param translation
        */
      inline void
      setTranslation (const Eigen::Vector3f &translation)
      {
        translation_ = translation;
      }

      /** \brief Get the value of the internal \a translate parameter.
        */
      Eigen::Vector3f
      getTranslation ()
      {
        return (translation_);
      }

      /** \brief
        * \param rotation
        */
      inline void
      setRotation (const Eigen::Vector3f &rotation)
      {
        rotation_ = rotation;
      }

      /** \brief Get the value of the internal \a translate parameter.
        */
      Eigen::Vector3f
      getRotation ()
      {
        return (rotation_);
      }

      /** \brief
        * \param transform
        */
      inline void
      setTransform (const Eigen::Affine3f &transform)
      {
        transform_ = transform;
      }

      /** \brief Get the value of the internal \a min_pt parameter.
        */
      Eigen::Affine3f
      getTransform ()
      {
        return (transform_);
      }

    protected:
      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);

    private:
      /** \brief Description*/
      Eigen::Vector4f min_pt_;
      Eigen::Vector4f max_pt_;
      Eigen::Vector3f translation_;
      Eigen::Vector3f rotation_;
      Eigen::Affine3f transform_;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b CropBox
    * \author Justin Rosen
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS CropBox<sensor_msgs::PointCloud2> : public FilterIndices<sensor_msgs::PointCloud2>
  {
    using Filter<sensor_msgs::PointCloud2>::filter_name_;
    using Filter<sensor_msgs::PointCloud2>::getClassName;

    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
    /** \brief Empty constructor. */
      CropBox () :
        min_pt_(Eigen::Vector4f (-1, -1, -1, 1)),
        max_pt_(Eigen::Vector4f (1, 1, 1, 1)),
        transform_(Eigen::Affine3f::Identity ())
      {
        filter_name_ = "CropBox";
        rotation_ = Eigen::Vector3f::Zero ();
        translation_ = Eigen::Vector3f::Zero ();
      }

      /** \brief
        * \param min_pt
        */
      inline void
      setMin (const Eigen::Vector4f& min_pt)
      {
        min_pt_ = min_pt;
      }

      /** \brief Get the value of the internal \a min_pt parameter.
        */
      Eigen::Vector4f
      getMin ()
      {
        return (min_pt_);
      }

      /** \brief
        * \param max_pt
        */
      inline void
      setMax (const Eigen::Vector4f &max_pt)
      {
        max_pt_ = max_pt;
      }

      /** \brief Get the value of the internal \a min_pt parameter.
        */
      Eigen::Vector4f
      getMax ()
      {
        return (max_pt_);
      }

      /** \brief
        * \param translation
        */
      inline void
      setTranslation (const Eigen::Vector3f &translation)
      {
        translation_ = translation;
      }

      /** \brief Get the value of the internal \a translate parameter.
        */
      Eigen::Vector3f
      getTranslation ()
      {
        return (translation_);
      }

      /** \brief
        * \param rotation
        */
      inline void
      setRotation (const Eigen::Vector3f &rotation)
      {
        rotation_ = rotation;
      }

      /** \brief Get the value of the internal \a translate parameter.
        */
      Eigen::Vector3f
      getRotation ()
      {
        return (rotation_);
      }

      /** \brief
        * \param transform
        */
      inline void
      setTransform (const Eigen::Affine3f &transform)
      {
        transform_ = transform;
      }

      /** \brief Get the value of the internal \a min_pt parameter.
        */
      Eigen::Affine3f
      getTransform ()
      {
        return (transform_);
      }

    protected:
      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PointCloud2 &output);

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);

      /** \brief Description*/
      Eigen::Vector4f min_pt_;
      Eigen::Vector4f max_pt_;
      Eigen::Vector3f translation_;
      Eigen::Vector3f rotation_;
      Eigen::Affine3f transform_;
  };
}

#endif  // PCL_FILTERS_CROP_BOX_H_

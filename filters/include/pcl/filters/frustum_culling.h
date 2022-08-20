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

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_config.h> // for PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief FrustumCulling filters points inside a frustum
   *  given by pose and field of view of the camera.
   *
   * Code example:
   *
   * \code
   * pcl::PointCloud <pcl::PointXYZ>::Ptr source; 
   * // .. read or fill the source cloud
   *
   * pcl::FrustumCulling<pcl::PointXYZ> fc;
   * fc.setInputCloud (source);
   * fc.setVerticalFOV (45);
   * fc.setHorizontalFOV (60);
   * fc.setNearPlaneDistance (5.0);
   * fc.setFarPlaneDistance (15);
   *
   * Eigen::Matrix4f camera_pose;
   * // .. read or input the camera pose from a registration algorithm.
   * fc.setCameraPose (camera_pose);
   *
   * pcl::PointCloud <pcl::PointXYZ> target;
   * fc.filter (target);
   * \endcode
   *
   *
   * \author Aravindhan K Krishnan
   * \ingroup filters
   */
  template <typename PointT>
  class FrustumCulling : public FilterIndices<PointT>
  {
    using PointCloud = typename Filter<PointT>::PointCloud;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    public:

      using Ptr = shared_ptr<FrustumCulling<PointT> >;
      using ConstPtr = shared_ptr<const FrustumCulling<PointT> >;


      using Filter<PointT>::getClassName;

      FrustumCulling (bool extract_removed_indices = false) 
        : FilterIndices<PointT> (extract_removed_indices)
        , camera_pose_ (Eigen::Matrix4f::Identity ())
        , hfov_ (60.0f)
        , vfov_ (60.0f)
        , np_dist_ (0.1f)
        , fp_dist_ (5.0f)
        , roi_x_ (0.5f)
        , roi_y_ (0.5f)
        , roi_w_ (1.0f)
        , roi_h_ (1.0f)
      {
        filter_name_ = "FrustumCulling";
      }

      /** \brief Set the pose of the camera w.r.t the origin
        * \param[in] camera_pose the camera pose
        *
        * Note: This assumes a coordinate system where X is forward, 
        * Y is up, and Z is right. To convert from the traditional camera 
        * coordinate system (X right, Y down, Z forward), one can use:
        *
        * \code
        * Eigen::Matrix4f pose_orig = //pose in camera coordinates
        * Eigen::Matrix4f cam2robot;
        * cam2robot << 0, 0, 1, 0
        *              0,-1, 0, 0
        *              1, 0, 0, 0
        *              0, 0, 0, 1;
        * Eigen::Matrix4f pose_new = pose_orig * cam2robot;
        * fc.setCameraPose (pose_new);
        * \endcode
        */
      void 
      setCameraPose (const Eigen::Matrix4f& camera_pose)
      {
        camera_pose_ = camera_pose;
      }

      /** \brief Get the pose of the camera w.r.t the origin */
      Eigen::Matrix4f
      getCameraPose () const
      {
        return (camera_pose_);
      }

      /** \brief Set the horizontal field of view for the camera in degrees
        * \param[in] hfov the field of view
        */
      void 
      setHorizontalFOV (float hfov)
      {
        hfov_ = hfov;
      }

      /** \brief Get the horizontal field of view for the camera in degrees */
      float 
      getHorizontalFOV () const
      {
        return (hfov_);
      }

      /** \brief Set the vertical field of view for the camera in degrees
        * \param[in] vfov the field of view
        */
      void 
      setVerticalFOV (float vfov)
      {
        vfov_ = vfov;
      }

      /** \brief Get the vertical field of view for the camera in degrees */
      float 
      getVerticalFOV () const
      {
        return (vfov_);
      }

      /** \brief Set the near plane distance
        * \param[in] np_dist the near plane distance
        */
      void 
      setNearPlaneDistance (float np_dist)
      {
        np_dist_ = np_dist;
      }

      /** \brief Get the near plane distance. */
      float
      getNearPlaneDistance () const
      {
        return (np_dist_);
      }

      /** \brief Set the far plane distance
        * \param[in] fp_dist the far plane distance
        */
      void 
      setFarPlaneDistance (float fp_dist)
      {
        fp_dist_ = fp_dist;
      }

      /** \brief Get the far plane distance */
      float 
      getFarPlaneDistance () const
      {
        return (fp_dist_);
      }
      
      /** \brief Set the region of interest (ROI) in normalized values
        *  
        * Default value of ROI: roi_{x, y} = 0.5, roi_{w, h} = 1.0
        * This corresponds to maximal FoV and returns all the points in the frustum
        * Can be used to cut out objects based on 2D bounding boxes by object detection.
        * 
        * \param[in] roi_x X center position of ROI
        * \param[in] roi_y Y center position of ROI
        * \param[in] roi_w Width of ROI
        * \param[in] roi_h Height of ROI
        */
      void 
      setRegionOfInterest (float roi_x, float roi_y, float roi_w, float roi_h)
      {
        if ((roi_x > 1.0f) || (roi_x < 0.0f) ||
            (roi_y > 1.0f) || (roi_y < 0.0f) ||
            (roi_w <= 0.0f) || (roi_w > 1.0f) ||
            (roi_h <= 0.0f) || (roi_h > 1.0f))
        {
          throw PCLException ("ROI X-Y values should be between 0 and 1. " 
            "Width and height must not be zero.", 
            "frustum_culling.h", "setRegionOfInterest");
        }
        roi_x_ = roi_x;
        roi_y_ = roi_y;
        roi_w_ = roi_w;
        roi_h_ = roi_h;
      }
      
      /** \brief Get the region of interest (ROI) in normalized values
        * \param[in] roi_x X center position of ROI
        * \param[in] roi_y Y center position of ROI
        * \param[in] roi_w Width of ROI 
        * \param[in] roi_h Height of ROI
        */
      void 
      getRegionOfInterest (float &roi_x, float &roi_y, float &roi_w, float &roi_h) const
      {
        roi_x = roi_x_;
        roi_y = roi_y_;
        roi_w = roi_w_;
        roi_h = roi_h_;
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

      /** \brief The camera pose */
      Eigen::Matrix4f camera_pose_;
      /** \brief Horizontal field of view */
      float hfov_;
      /** \brief Vertical field of view */
      float vfov_;
      /** \brief Near plane distance */
      float np_dist_;
      /** \brief Far plane distance */
      float fp_dist_;
      /** \brief Region of interest x center position (normalized)*/
      float roi_x_;
      /** \brief Region of interest y center position (normalized)*/
      float roi_y_;
      /** \brief Region of interest width (normalized)*/
      float roi_w_;
      /** \brief Region of interest height (normalized)*/
      float roi_h_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/frustum_culling.hpp>
#endif

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


#ifndef PCL_FILTERS_FRUSTUM_CULLING_H_
#define PCL_FILTERS_FRUSTUM_CULLING_H_

#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

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
    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

      typedef boost::shared_ptr< FrustumCulling<PointT> > Ptr;
      typedef boost::shared_ptr< const FrustumCulling<PointT> > ConstPtr;


      using Filter<PointT>::getClassName;

      FrustumCulling (bool extract_removed_indices = false) 
        : FilterIndices<PointT>::FilterIndices (extract_removed_indices)
        , camera_pose_ (Eigen::Matrix4f::Identity ())
        , hfov_ (60.0f)
        , vfov_ (60.0f)
        , np_dist_ (0.1f)
        , fp_dist_ (5.0f)
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

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Sample of point indices into a separate PointCloud
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Sample of point indices
        * \param[out] indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);

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

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/frustum_culling.hpp>
#endif

#endif

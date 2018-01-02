/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_FILTERS_SPHERICAL_VOXEL_GRID_H_
#define PCL_FILTERS_SPHERICAL_VOXEL_GRID_H_

#include <pcl/filters/filter.h>

namespace pcl
{
  /** \brief Get the point at maximum distance from a given point and a given pointcloud subject to filter parameters
    * \param cloud the point cloud data message
    * \param distance_field_name the field to filter the input data
    * \param min_distance the minimum filter value
    * \param max_distance the maximum filter value
    * \param pivot_pt the point from where to compute the distance
    * \param max_pt the point in cloud that is the farthest point away from pivot_pt
    * \param limit_negative use points that are not within the filter limits
    */
  template<typename PointT> inline void
  getMaxDistance (const pcl::PointCloud<PointT> &cloud,
                  const std::string &distance_field_name, float min_distance, float max_distance,
                  const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt, bool limit_negative);

  /** \brief Get the point at maximum distance from a given point and a given pointcloud subject to filter parameters
    * \param cloud the point cloud data message
    * \param indicies the indices used from the point cloud
    * \param distance_field_name the field to filter the input data
    * \param min_distance the minimum filter value
    * \param max_distance the maximum filter value
    * \param pivot_pt the point from where to compute the distance
    * \param max_pt the point in cloud that is the farthest point away from pivot_pt
    * \param limit_negative use points that are not within the filter limits
    */
  template<typename PointT> inline void
  getMaxDistance (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                  const std::string &distance_field_name, float min_distance, float max_distance,
                  const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt, bool limit_negative);

  /** \brief Get the point at maximum distance from a given point and a given pointcloud subject to filter parameters
    * \param cloud the point cloud data message
    * \param x_idx the index of the x field in the point cloud
    * \param y_idx the index of the y field in the point cloud
    * \param z_idx the index of the z field in the point cloud
    * \param distance_field_name the field to filter the input data
    * \param min_distance the minimum filter value
    * \param max_distance the maximum filter value
    * \param pivot_pt the point from where to compute the distance
    * \param max_pt the point in cloud that is the farthest point away from pivot_pt
    * \param limit_negative use points that are not within the filter limits
    */
  PCL_EXPORTS void
  getMaxDistance (const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                  const std::string &distance_field_name, float min_distance, float max_distance,
                  const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt, bool limit_negative);


  /** \brief Get the point at maximum distance from a given point and a given pointcloud
    * \param cloud the point cloud data message
    * \param x_idx the index of the x field in the point cloud
    * \param y_idx the index of the y field in the point cloud
    * \param z_idx the index of the z field in the point cloud
    * \param pivot_pt the point from where to compute the distance
    * \param max_pt the point in cloud that is the farthest point away from pivot_pt
    */
  PCL_EXPORTS void
  getMaxDistance (const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                  const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt);

  /** \brief SphericalVoxelGrid assembles a local, spherically shaped 3D grid over a given PointCloud and downsamples + filters the data.
    *
    * The SphericalVoxelGrid class creates a *spherically* shaped 3D voxel grid. Its
    * functionality is the same as the VoxelGrid class, except that the voxel sizes
    * are specified spherically (i.e. one radial + two angular resolutions). This lends
    * itself well to reducing the noise and modifying the resolutions of point clouds
    * generated by scanners which sample in a spherical manner.
    *
    * \author Spencer Davis, Radu B. Rusu, Bastian Steder
    * \ingroup filters
    */
  template<typename PointT>
  class SphericalVoxelGrid : public Filter<PointT>
  {
    protected:
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using Filter<PointT>::input_;
      using Filter<PointT>::indices_;

      typedef typename Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      typedef boost::shared_ptr< SphericalVoxelGrid<PointT> > Ptr;
      typedef boost::shared_ptr< const SphericalVoxelGrid<PointT> > ConstPtr;

    public:
      /** \brief Empty constructor. */
      SphericalVoxelGrid () :
                leaf_size_r_ (0),
                leaf_r_divisions_(0),
                leaf_theta_divisions_ (0),
                leaf_phi_divisions_ (0),
                leaf_size_theta_ (0),
                leaf_size_phi_ (0),
                downsample_all_data_ (true),
                min_points_per_voxel_ (0),
                filter_field_name_ (""),
                filter_limit_min_ (-FLT_MAX),
                filter_limit_max_ (FLT_MAX),
                filter_limit_negative_ (false),
                filter_origin_ (Eigen::Vector4f::Zero ()),
                max_radius_ (0)

      {
          filter_name_ = "SphericalVoxelGrid";
      }

      virtual ~SphericalVoxelGrid () {}

      /** \brief Set the size of the spherically shaped voxels
        * \param[in] r the radial length of each voxel
        * \param[in] theta_divisions the total number of vertical divisions of a single hemisphere
        * (on one side of z axis) in the spherical coordinate system
        * \param[in] phi_divisions the total number of horizonal divisions of the spherical coordiante system
        */
      inline void
      setLeafSize (float r, int theta_divisions, int phi_divisions)
      {
        if (r <= 0 || theta_divisions < 1 || phi_divisions < 1)
        {
          PCL_WARN ("[pcl::%s::setLeafSize] Invalid leaf size\n", getClassName().c_str());
          return;
        }

        leaf_size_r_ = r;
        leaf_size_theta_ = static_cast<float>(M_PI) /  theta_divisions;
        leaf_size_phi_ = (2.0f * static_cast<float>(M_PI)) / phi_divisions;

        leaf_theta_divisions_ = theta_divisions;
        leaf_phi_divisions_ = phi_divisions;
      }

      /** \brief Get the radial size of the leaves
        */
      inline float
      getLeafSizeR () { return (leaf_size_r_); }

      /** \brief Get the vertical angular size (radians) of the leaves
        */
      inline float
      getLeafSizeTheta () { return (leaf_size_theta_); }

      /** \brief Get the horizontal angular size (radians) of the leaves
        */
      inline float
      getLeafSizePhi () { return (leaf_size_phi_); }

      /** \brief Get the number of radial divisions
        */
      inline int
      getDivisionsR () { return (leaf_r_divisions_); }

      /** \brief Get the number of vertical angular divisions
        */
      inline int
      getDivisionsTheta () { return (leaf_theta_divisions_); }

      /** \brief Get the number of horizontal angular divisions
        */
      inline int
      getDivisionsPhi () { return (leaf_phi_divisions_); }

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
        * \param[in] downsample the new value (true/false)
        */
      inline void
      setDownsampleAllData (bool downsample) { downsample_all_data_ = downsample; }

      /** \brief Get the state of the internal downsampling parameter (true if
        * all fields need to be downsampled, false if just XYZ).
        */
      inline bool
      getDownsampleAllData () { return (downsample_all_data_); }

      /** \brief Set the minimum number of points required for a voxel to be used.
        * \param[in] min_points_per_voxel the minimum number of points for required for a voxel to be used
        */
      inline void
      setMinimumPointsNumberPerVoxel (unsigned int min_points_per_voxel) { min_points_per_voxel_ = min_points_per_voxel; }

      /** \brief Return the minimum number of points required for a voxel to be used.
        */
      inline unsigned int
      getMinimumPointsNumberPerVoxel () { return (min_points_per_voxel_); }

      /** \brief Return the maximum radius of a point from the ceter,
        * i.e. the radius of the spherical voxel grid (after filtering)
        */
      inline float
      getMaximumRadius () { return (max_radius_); }

      /** \brief Set the cartesian origin used to build the spherical grid.
        * \param[in] origin the cartesian origin
        */
      inline void
      setOrigin (const Eigen::Vector4f& origin) { filter_origin_ = origin; }

      /** \brief Get the cartesian origin used to build the spherical grid.
        */
      inline Eigen::Vector3f
      getOrigin () { return (filter_origin_.head<3>()); }

      /** \brief Provide the name of the field to be used for filtering data. In conjunction with  \a setFilterLimits,
        * points having values outside this interval will be discarded.
        * \param[in] field_name the name of the field that contains values used for filtering
        */
      inline void
      setFilterFieldName (const std::string &field_name) { filter_field_name_ = field_name; }

      /** \brief Get the name of the field used for filtering. */
      inline std::string const
      getFilterFieldName () { return (filter_field_name_); }

      /** \brief Set the field filter limits. All points having field values outside this interval will be discarded.
        * \param[in] limit_min the minimum allowed field value
        * \param[in] limit_max the maximum allowed field value
        */
      inline void
      setFilterLimits (const double &limit_min, const double &limit_max)
      {
        filter_limit_min_ = limit_min;
        filter_limit_max_ = limit_max;
      }

      /** \brief Get the field filter limits (min/max) set by the user. The default values are -FLT_MAX, FLT_MAX.
        * \param[out] limit_min the minimum allowed field value
        * \param[out] limit_max the maximum allowed field value
        */
      inline void
      getFilterLimits (double &limit_min, double &limit_max)
      {
        limit_min = filter_limit_min_;
        limit_max = filter_limit_max_;
      }

      /** \brief Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max).
        * Default: false.
        * \param[in] limit_negative return data inside the interval (false) or outside (true)
        */
      inline void
      setFilterLimitsNegative (const bool limit_negative) { filter_limit_negative_ = limit_negative; }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false).
        * \param[out] limit_negative true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline void
      getFilterLimitsNegative (bool &limit_negative) { limit_negative = filter_limit_negative_; }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false).
        * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline bool
      getFilterLimitsNegative () { return (filter_limit_negative_); }

  protected:
      /** \brief The radial size of the voxels. */
      float leaf_size_r_;

      /** \brief The number of radial divisions. */
      int leaf_r_divisions_;

      /** \brief The number of vertical divisions. */
      int leaf_theta_divisions_;

      /** \brief The number of horizontal divisions. */
      int leaf_phi_divisions_;

      /** \brief The size of the vertical divisions (radians). */
      float leaf_size_theta_;

      /** \brief The size of the horizontal divisions (radians). */
      float leaf_size_phi_;

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
      bool downsample_all_data_;

      /** \brief Minimum number of points per voxel for the centroid to be computed */
      unsigned int min_points_per_voxel_;

      /** \brief The desired user filter field name. */
      std::string filter_field_name_;

      /** \brief The minimum allowed filter value a point will be considered from. */
      double filter_limit_min_;

      /** \brief The maximum allowed filter value a point will be considered from. */
      double filter_limit_max_;

      /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
      bool filter_limit_negative_;

      /** Cartesian origin of the spherical coordinate system. */
      Eigen::Vector4f filter_origin_;

      /** Maximum distance from the origin to a point in the cloud. */
      float max_radius_;

      /** \brief Downsample a Point Cloud using a spherical voxelized grid approach
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/spherical_voxel_grid.hpp>
#endif

#endif // PCL_FILTERS_SPHERICAL_VOXEL_GRID_H_
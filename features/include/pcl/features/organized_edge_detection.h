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

#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>

namespace pcl
{
  /** \brief OrganizedEdgeBase, OrganizedEdgeFromRGB, OrganizedEdgeFromNormals,
    * and OrganizedEdgeFromRGBNormals find 3D edges from an organized point
    * cloud data. Given an organized point cloud, they will output a PointCloud
    * of edge labels and a vector of PointIndices.
    * OrganizedEdgeBase accepts PCL_XYZ_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, and EDGELABEL_OCCLUDED.
    * OrganizedEdgeFromRGB accepts PCL_RGB_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, and EDGELABEL_RGB_CANNY.
    * OrganizedEdgeFromNormals accepts PCL_XYZ_POINT_TYPES with PCL_NORMAL_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, and EDGELABEL_HIGH_CURVATURE.
    * OrganizedEdgeFromRGBNormals accepts PCL_RGB_POINT_TYPES with PCL_NORMAL_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, EDGELABEL_HIGH_CURVATURE, and EDGELABEL_RGB_CANNY.
    *
    * \author Changhyun Choi
    */
  template <typename PointT, typename PointLT>
  class OrganizedEdgeBase : public PCLBase<PointT>
  {
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using PointCloudL = pcl::PointCloud<PointLT>;
    using PointCloudLPtr = typename PointCloudL::Ptr;
    using PointCloudLConstPtr = typename PointCloudL::ConstPtr;

    public:
      using Ptr = shared_ptr<OrganizedEdgeBase<PointT, PointLT> >;
      using ConstPtr = shared_ptr<const OrganizedEdgeBase<PointT, PointLT> >;
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using PCLBase<PointT>::initCompute;
      using PCLBase<PointT>::deinitCompute;

      /** \brief Constructor for OrganizedEdgeBase */
      OrganizedEdgeBase ()
        : th_depth_discon_ (0.02f)
        , max_search_neighbors_ (50)
        , detecting_edge_types_ (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED)
      {
      }

      /** \brief Destructor for OrganizedEdgeBase */

      ~OrganizedEdgeBase ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities)
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;

      /** \brief Set the tolerance in meters for the relative difference in depth values between neighboring points.
        * e.g. If a point has a depth (z) value of 2.0 meters, a neighboring point is discontinuous if its depth differs by > 2.0 * th. */
      inline void
      setDepthDisconThreshold (const float th)
      {
        th_depth_discon_ = th;
      }

      /** \brief Get the tolerance in meters for the relative difference in depth values between neighboring points.
        * e.g. If a point has a depth (z) value of 2.0 meters, a neighboring point is discontinuous if its depth differs by > 2.0 * th. */
      inline float
      getDepthDisconThreshold () const
      {
        return (th_depth_discon_);
      }

      /** \brief Set the max search distance for deciding occluding and occluded edges. */
      inline void
      setMaxSearchNeighbors (const int max_dist)
      {
        max_search_neighbors_ = max_dist;
      }

      /** \brief Get the max search distance for deciding occluding and occluded edges. */
      inline int
      getMaxSearchNeighbors () const
      {
        return (max_search_neighbors_);
      }

      /** \brief Set the detecting edge types. */
      inline void
      setEdgeType (int edge_types)
      {
        detecting_edge_types_ = edge_types;
      }

      /** \brief Get the detecting edge types. */
      inline int
      getEdgeType () const
      {
        return detecting_edge_types_;
      }

      enum {EDGELABEL_NAN_BOUNDARY=1, EDGELABEL_OCCLUDING=2, EDGELABEL_OCCLUDED=4, EDGELABEL_HIGH_CURVATURE=8, EDGELABEL_RGB_CANNY=16};
      static const int num_of_edgetype_ = 5;

    protected:
      /** \brief Perform the 3D edge detection (edges from depth discontinuities) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        */
      void
      extractEdges (pcl::PointCloud<PointLT>& labels) const;

      /** \brief Assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      assignLabelIndices (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;

      struct Neighbor
      {
        Neighbor (int dx, int dy, int didx)
        : d_x (dx)
        , d_y (dy)
        , d_index (didx)
        {}

        int d_x;
        int d_y;
        int d_index; // = dy * width + dx: pre-calculated
      };

      /** \brief The tolerance in meters for the relative difference in depth values between neighboring points
        * (The default value is set for .02 meters and is adapted with respect to depth value linearly.
        * e.g. If a point has a depth (z) value of 2.0 meters, a neighboring point is discontinuous if its depth differs by > 2.0 * th. */
      float th_depth_discon_;

      /** \brief The max search distance for deciding occluding and occluded edges */
      int max_search_neighbors_;

      /** \brief The bit encoded value that represents edge types to detect */
      int detecting_edge_types_;
  };

  template <typename PointT, typename PointLT>
  class OrganizedEdgeFromRGB : virtual public OrganizedEdgeBase<PointT, PointLT>
  {
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using PointCloudL = pcl::PointCloud<PointLT>;
    using PointCloudLPtr = typename PointCloudL::Ptr;
    using PointCloudLConstPtr = typename PointCloudL::ConstPtr;

    public:
      using OrganizedEdgeBase<PointT, PointLT>::input_;
      using OrganizedEdgeBase<PointT, PointLT>::indices_;
      using OrganizedEdgeBase<PointT, PointLT>::initCompute;
      using OrganizedEdgeBase<PointT, PointLT>::deinitCompute;
      using OrganizedEdgeBase<PointT, PointLT>::detecting_edge_types_;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_NAN_BOUNDARY;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDING;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDED;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_RGB_CANNY;

      /** \brief Constructor for OrganizedEdgeFromRGB */
      OrganizedEdgeFromRGB ()
        : OrganizedEdgeBase<PointT, PointLT> ()
        , th_rgb_canny_low_ (40.0)
        , th_rgb_canny_high_ (100.0)
      {
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_RGB_CANNY);
      }

      /** \brief Destructor for OrganizedEdgeFromRGB */

      ~OrganizedEdgeFromRGB ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities and RGB Canny edge) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;

      /** \brief Set the low threshold value for RGB Canny edge detection */
      inline void
      setRGBCannyLowThreshold (const float th)
      {
        th_rgb_canny_low_ = th;
      }

      /** \brief Get the low threshold value for RGB Canny edge detection */
      inline float
      getRGBCannyLowThreshold () const
      {
        return (th_rgb_canny_low_);
      }

      /** \brief Set the high threshold value for RGB Canny edge detection */
      inline void
      setRGBCannyHighThreshold (const float th)
      {
        th_rgb_canny_high_ = th;
      }

      /** \brief Get the high threshold value for RGB Canny edge detection */
      inline float
      getRGBCannyHighThreshold () const
      {
        return (th_rgb_canny_high_);
      }

    protected:
      /** \brief Perform the 3D edge detection (edges from depth discontinuities and RGB Canny edge)
        * \param[out] labels a PointCloud of edge labels
        */
      void
      extractEdges (pcl::PointCloud<PointLT>& labels) const;

      /** \brief The low threshold value for RGB Canny edge detection (default: 40.0) */
      float th_rgb_canny_low_;

      /** \brief The high threshold value for RGB Canny edge detection (default: 100.0) */
      float th_rgb_canny_high_;
  };

  template <typename PointT, typename PointNT, typename PointLT>
  class OrganizedEdgeFromNormals : virtual public OrganizedEdgeBase<PointT, PointLT>
  {
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using PointCloudN = pcl::PointCloud<PointNT>;
    using PointCloudNPtr = typename PointCloudN::Ptr;
    using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

    using PointCloudL = pcl::PointCloud<PointLT>;
    using PointCloudLPtr = typename PointCloudL::Ptr;
    using PointCloudLConstPtr = typename PointCloudL::ConstPtr;

    public:
      using OrganizedEdgeBase<PointT, PointLT>::input_;
      using OrganizedEdgeBase<PointT, PointLT>::indices_;
      using OrganizedEdgeBase<PointT, PointLT>::initCompute;
      using OrganizedEdgeBase<PointT, PointLT>::deinitCompute;
      using OrganizedEdgeBase<PointT, PointLT>::detecting_edge_types_;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_NAN_BOUNDARY;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDING;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDED;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_HIGH_CURVATURE;

      /** \brief Constructor for OrganizedEdgeFromNormals */
      OrganizedEdgeFromNormals ()
        : OrganizedEdgeBase<PointT, PointLT> ()
        , normals_ ()
        , th_hc_canny_low_ (0.4f)
        , th_hc_canny_high_ (1.1f)
      {
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE);
      }

      /** \brief Destructor for OrganizedEdgeFromNormals */

      ~OrganizedEdgeFromNormals ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities and high curvature regions) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;

      /** \brief Provide a pointer to the input normals.
        * \param[in] normals the input normal cloud
        */
      inline void
      setInputNormals (const PointCloudNConstPtr &normals)
      {
        normals_ = normals;
      }

      /** \brief Get the input normals. */
      inline PointCloudNConstPtr
      getInputNormals () const
      {
        return (normals_);
      }

      /** \brief Set the low threshold value for high curvature Canny edge detection */
      inline void
      setHCCannyLowThreshold (const float th)
      {
        th_hc_canny_low_ = th;
      }

      /** \brief Get the low threshold value for high curvature Canny edge detection */
      inline float
      getHCCannyLowThreshold () const
      {
        return (th_hc_canny_low_);
      }

      /** \brief Set the high threshold value for high curvature Canny edge detection */
      inline void
      setHCCannyHighThreshold (const float th)
      {
        th_hc_canny_high_ = th;
      }

      /** \brief Get the high threshold value for high curvature Canny edge detection */
      inline float
      getHCCannyHighThreshold () const
      {
        return (th_hc_canny_high_);
      }

    protected:
      /** \brief Perform the 3D edge detection (edges from depth discontinuities and high curvature regions)
        * \param[out] labels a PointCloud of edge labels
        */
      void
      extractEdges (pcl::PointCloud<PointLT>& labels) const;

      /** \brief A pointer to the input normals */
      PointCloudNConstPtr normals_;

      /** \brief The low threshold value for high curvature Canny edge detection (default: 0.4) */
      float th_hc_canny_low_;

      /** \brief The high threshold value for high curvature Canny edge detection (default: 1.1) */
      float th_hc_canny_high_;
  };

  template <typename PointT, typename PointNT, typename PointLT>
  class OrganizedEdgeFromRGBNormals : public OrganizedEdgeFromRGB<PointT, PointLT>, public OrganizedEdgeFromNormals<PointT, PointNT, PointLT>
  {
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using PointCloudN = pcl::PointCloud<PointNT>;
    using PointCloudNPtr = typename PointCloudN::Ptr;
    using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

    using PointCloudL = pcl::PointCloud<PointLT>;
    using PointCloudLPtr = typename PointCloudL::Ptr;
    using PointCloudLConstPtr = typename PointCloudL::ConstPtr;

    public:
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::input_;
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::indices_;
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::initCompute;
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::deinitCompute;
      using OrganizedEdgeFromNormals<PointT, PointNT, PointLT>::detecting_edge_types_;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_NAN_BOUNDARY;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDING;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_OCCLUDED;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_HIGH_CURVATURE;
      using OrganizedEdgeBase<PointT, PointLT>::EDGELABEL_RGB_CANNY;

      /** \brief Constructor for OrganizedEdgeFromRGBNormals */
      OrganizedEdgeFromRGBNormals ()
        : OrganizedEdgeFromRGB<PointT, PointLT> ()
        , OrganizedEdgeFromNormals<PointT, PointNT, PointLT> ()
      {
        this->setEdgeType (EDGELABEL_NAN_BOUNDARY | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_RGB_CANNY | EDGELABEL_HIGH_CURVATURE);
      }

      /** \brief Destructor for OrganizedEdgeFromRGBNormals */

      ~OrganizedEdgeFromRGBNormals ()
      {
      }

      /** \brief Perform the 3D edge detection (edges from depth discontinuities, RGB Canny edge, and high curvature regions) and assign point indices for each edge label
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/organized_edge_detection.hpp>
#endif

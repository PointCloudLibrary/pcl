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
 *
 *
 */

#ifndef PCL_FEATURES_ORGANIZED_EDGE_DETECTION_H_
#define PCL_FEATURES_ORGANIZED_EDGE_DETECTION_H_

#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>

namespace pcl
{
  /** \brief OrganizedEdgeDetection finds 3D edges from organized point cloud data.
    * Given an organized point cloud, it will output a PointCloud of edge labels
    * and a vector of PointIndices.
    *
    * \author Changhyun Choi
    */
  template <typename PointT, typename PointLT>
  class OrganizedEdgeDetection : public PCLBase<PointT>
  {
    using PCLBase<PointT>::input_;
    using PCLBase<PointT>::indices_;
    using PCLBase<PointT>::initCompute;
    using PCLBase<PointT>::deinitCompute;

    public:
      typedef typename pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      
      typedef typename pcl::PointCloud<PointLT> PointCloudL;
      typedef typename PointCloudL::Ptr PointCloudLPtr;
      typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

      /** \brief Constructor for OrganizedEdgeDetection */
      OrganizedEdgeDetection ()
        : th_depth_discon_(0.02), max_search_neighbors_(50)
      {
      }

      /** \brief Destructor for OrganizedEdgeDetection */
      virtual
      ~OrganizedEdgeDetection ()
      {
      }

      /** \brief Perform the 3D edge detection
        * \param[out] labels a PointCloud of edge labels
        * \param[out] label_indices a vector of PointIndices corresponding to each edge label
        */
      void
      compute (pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;
      
      /** \brief Set the tolerance in meters for difference in depth values between neighboring points. */
      inline void
      setDepthDisconThreshold (const float th)
      {
        th_depth_discon_ = th;
      }

      /** \brief Get the tolerance in meters for difference in depth values between neighboring points. */
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

      enum {EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, EDGELABEL_HIGH_CURVATURE, EDGELABEL_RGB_CANNY};

    protected:

    private:
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

      /** \brief The tolerance in meters for difference in depth values between neighboring points 
        * (The value is set for 1 meter and is adapted with respect to depth value linearly. 
        * (e.g. 2.0*th_depth_discon_ in 2 meter depth)) 
        */
      float th_depth_discon_;

      /** \brief The max search distance for deciding occluding and occluded edges */
      int max_search_neighbors_;
  };
}

#endif //#ifndef PCL_FEATURES_ORGANIZED_EDGE_DETECTION_H_

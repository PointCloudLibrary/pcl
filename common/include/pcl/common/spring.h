/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_POINT_CLOUD_SPRING_H_
#define PCL_POINT_CLOUD_SPRING_H_

#include <pcl/point_cloud.h>

namespace pcl
{

  /** Class PointCloudSpring used to expand or shrink a point cloud in  
    * columns and or rows direction using either mirroring or 
    * duplicating edges of the input point cloud or filling the new cells 
    * with some value.
    *
    * \author Nizar Sallem
    */
  template <typename PointT>
  class PointCloudSpring
  {
    public:
      typedef typename pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      typedef typename PointCloud::iterator iterator;

      /// expansion policy
      enum EXPAND_POLICY { MIRROR = 0, DUPLICATE };
      /// Default constructor
      PointCloudSpring ()
        : expand_policy_ (-1)
        , r_amount_ (0)
        , c_amount_ (0)
      {}
      /// \set expansion policy
      inline void
      setExpandPolicy (int policy) { expand_policy_ = policy; }
      /// \return the policy
      inline int
      getExpandPolicy () { return (expand_policy_); } 
      /** set expansion amount for columns and rows direction
        * \param[in] amount expand/shrink amount for both rows and columns
        */
      inline void
      setAmount (int amount) { c_amount_ = r_amount_ = amount; }
      /// reset the amounts to 0
      inline void 
      reset () { c_amount_ = r_amount_ = 0; }
      /** set expansion amount for columns 
        * \param[in] amount expand/shrink amount for columns
        */
      inline void
      setColumnsAmount (int amount) { c_amount_ = amount; }
      /// \return the columns expansion amount
      inline int
      getColumnsAmount () { return (c_amount_); }
      /** set expansion amount for rows
        * \param[in] amount expand/shrink amount for rows
        */
      inline void
      setRowsAmount (int amount) { r_amount_ = amount; }
      /// \return the expansion amount for rows
      inline int
      getRowsAmount () { return (r_amount_); }
      /** set input cloud
        * param[in][out] input pointer to the cloud to be modified
        */ 
      inline void
      setInputCloud (const PointCloudConstPtr& input) { input_ = input; }
      /// \return input cloud
      inline PointCloudConstPtr const
      getInputCloud () { return (input_); }
      
    private:
      /** init computation checking params and copying point cloud if necessary
        * \param[out] output the output point cloud
        */
      inline bool
      initCompute (PointCloud& output);
      
    public:
      /** expand a point cloud in the set direction and policy. 
        * If the policy is set to mirror then the new created rows and/or 
        * columns are symmetrical to the top and bottom rows and/or left 
        * and right columns of the original point cloud.
        * If the policy is set to duplicate then the top and bottom rows 
        * and/or the right and left columns will be duplicated.
        * \param[out] output the output point cloud
        */
      inline void
      expand (PointCloud& output)
      {
        if (!initCompute (output))
          PCL_THROW_EXCEPTION (InitFailedException,
                               "[pcl::PointCloudSpring::expand] init failed");
        if (expand_policy_ == DUPLICATE)
        {
          if (r_amount_ > 0)
            expandRowsDuplicate (output);
          if (c_amount_ > 0)
            expandColumnsDuplicate (output);
        }
        else
        {
          if (r_amount_ > 0)
            expandRowsMirror (output);
          if (c_amount_ > 0)
            expandColumnsMirror (output);
        }
      }

      /** expand a point cloud in the set direction.
        * \param[in] val the point value to be used to fill.
        * \param[out] output the output point cloud
        */
      inline void
      expand (const PointT& val, PointCloud& output)
      {
        if (!initCompute (output))
          PCL_THROW_EXCEPTION (InitFailedException,
                               "[pcl::PointCloudSpring::expand] init failed");
        if (r_amount_ > 0)
          expandRows (val, output);
        if (c_amount_ > 0)
          expandColumns (val, output);
      }

      /** shrink a point cloud in the set direction.
        * - If direction is rows or both then \a h_amount rows are deleted
        *   from top and bottom.
        * - If direction is columns or both then \a v_amount columns are deleted
        *   from left and right.
        * \param[out] output the output point cloud
        */
      inline void
      shrink (PointCloud& output)
      {
        if (!initCompute (output))
          PCL_THROW_EXCEPTION (InitFailedException,
                               "[pcl::PointCloudSpring::shrink] init failed");
        if (r_amount_ > 0)
          deleteRows (output);
        if (c_amount_ > 0)
          deleteCols (output);
      }

    private:
      /** expand point cloud inserting \a r_amount_ rows at the 
        * top and the bottom of a point cloud and filling them with 
        * custom values.
        * \param[in] val the point value to be insterted
        * \param[out] output the output point cloud
        */
      void 
      expandRows (const PointT& val, PointCloud& output);
      /** expand point cloud inserting \a c_amount_ columns at 
        * the right and the left of a point cloud and filling them with 
        * custom values.
        * \param[in] val the point value to be insterted
        * \param[out] output the output point cloud
        */
      void 
      expandColumns (const PointT& val, PointCloud& output);
      /** expand point cloud duplicating the top and bottom rows \a r_amount_ times.
        * \param[out] output the output point cloud
        */
      void 
      expandRowsDuplicate (PointCloud& output);
      /** expand point cloud duplicating the right and left columns \a c_amount_ 
        * times.
        * \param[out] output the output point cloud
        */
      void 
      expandColumnsDuplicate (PointCloud& output);
      /** expand point cloud mirroring \a r_amount_ top and bottom rows. 
        * \param[out] output the output point cloud
        */
      void 
      expandRowsMirror (PointCloud& output);
      /** expand point cloud mirroring \a c_amount_ right and left columns. 
        * \param[out] output the output point cloud
        */
      void 
      expandColumnsMirror (PointCloud& output);
      /** delete \a r_amount_ rows in top and bottom of point cloud 
        * \param[out] output the output point cloud
        */
      inline void
      deleteRows (PointCloud& output);
      /** delete \a c_amount_ columns in top and bottom of point cloud 
        * \param[out] output the output point cloud
        */
      inline void
      deleteCols (PointCloud& output);
      
      /// expansion policy
      int expand_policy_;
      /// expansion amount for rows
      int r_amount_;
      /// expansion amount for columns
      int c_amount_;
      /// pointer to the input point cloud
      PointCloudConstPtr input_;
  };
}

#include <pcl/common/impl/spring.hpp>

#endif

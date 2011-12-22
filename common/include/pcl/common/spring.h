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

#ifndef PCL_POINT_CLOUD_EXPANDER_H_
#define PCL_POINT_CLOUD_EXPANDER_H_

#include <pcl/point_cloud.h>

namespace pcl
{

  /** Class PointCloudSpring used to expand or shrink a point cloud in  
    * horizontal and or vertical direction using either mirroring or 
    * duplicating edges of the input point cloud or filling the new cells 
    * to some value.
    * This class will modify the input point cloud so that no deep copy 
    * are made.
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
      /// direction of the expansion
      enum DIRECTION { HORIZONTAL = 0, VERTICAL, BOTH };
      /// Default constructor
      PointCloudSpring () : expand_policy_(-1), direction_(-1) {}
      /// \set expansion policy
      inline void
      setExpandPolicy (int policy) { expand_policy_ = policy; }
      /// \return the policy
      inline int
      getExpandPolicy () { return (expand_policy_); } 
      /// \set expansion's direction
      inline void
      setDirection (int direction) { direction_ = direction; }
      /// \return direction
      inline int
      getDirection () { return (direction_); } 
      /// \set expansion amount
      inline void
      setAmount (int amount) { amount_ = amount; }
      /// \return the amount
      inline int
      getAmount () { return (amount_); }
      /// \set input cloud
      inline void
      setInputCloud (PointCloudPtr& input) { input_ = input; }
      /// \return input cloud
      inline PointCloudPtr
      getInputCloud () { return (input_); }
      
    private:
      inline bool
      initCompute ();
      
    public:
      /** expand a point cloud in the set direction and policy. 
        * If the policy is set to mirror then the new created rows and/or 
        * columns are symmetrical to the top and bottom rows and/or left 
        * and right columns of the original point cloud.
        * If the policy is set to duplicate then the top and bottom rows 
        * and/or the right and left columns will be duplicated.
        */
      inline void
      expand ()
      {
        if (!initCompute ())
          PCL_THROW_EXCEPTION (InitFailedException,
                               "[pcl::PointCloudSpring::initCompute] init failed");
        if (expand_policy_ == DUPLICATE)
        {
          if ((direction_ == VERTICAL) || (direction_ == BOTH))
            expandVerticalDuplicate ();
          if ((direction_ == HORIZONTAL) || (direction_ == BOTH))
            expandHorizontalDuplicate ();
        }
        else
        {
          if ((direction_ == VERTICAL) || (direction_ == BOTH))
            expandVerticalMirror ();
          if ((direction_ == HORIZONTAL) || (direction_ == BOTH))
            expandHorizontalMirror ();
        }
      }

      /** expand a point cloud in the set direction.
        * \input val the point value to be used to fill.
        */
      inline void
      expand (const PointT& val)
      {
        if (!initCompute ())
          PCL_THROW_EXCEPTION (InitFailedException,
                               "[pcl::PointCloudSpring::initCompute] init failed");
        if ((direction_ == VERTICAL) || (direction_ == BOTH))
        {
          expandVertical (val);
        }
        if ((direction_ == HORIZONTAL) || (direction_ == BOTH))
        {
          expandHorizontal (val);
        }
      }

      /** shrink a point cloud in the set direction.
        * - If direction is vertical or both then the specified amount of
        *   top and bottom rows are deleted.
        * - If direction is horizontal or both then the specified amount
        *   of left and right columns are deleted.
        */
      inline void
      shrink ()
      {
        if (!initCompute ())
          PCL_THROW_EXCEPTION (InitFailedException,
                               "[pcl::PointCloudShrinker::initCompute] init failed");
        if ((direction_ == VERTICAL) || (direction_ == BOTH))
          deleteRows ();
        if ((direction_ == HORIZONTAL) || (direction_ == BOTH))
          deleteCols ();
      }

    private:
      /** expand point cloud vertically inserting \a amount_ rows at the 
        * top and the bottom of a point cloud and filling them with 
        * custom values.
        * \param val the point value to be insterted
        */
      void 
      expandVertical(const PointT& val);
      /** expand point cloud vertically inserting \a amount_ columns at 
        * the right and the left of a point cloud and filling them with 
        * custom values.
        * \param val the point value to be insterted
        */
      void 
      expandHorizontal(const PointT& val);
      /** expand point cloud vertically duplicating the top and bottom
        * rows \a amount_ times.
        */
      void 
      expandVerticalDuplicate();
      /** expand point cloud vertically duplicating the right and left
        * columns \a amount_ times.
        */
      void 
      expandHorizontalDuplicate();
      /** expand point cloud vertically mirroring \a amount_ top and 
        * bottom rows.
        */
      void 
      expandVerticalMirror();
      /** expand point cloud vertically mirroring \a amount_ right and 
        * left columns.
        */
      void 
      expandHorizontalMirror();
      /** delete \a amount_ rows in top and bottom of point cloud */
      inline void
      deleteRows ();
      /** delete \a amount_ columns in top and bottom of point cloud */
      inline void
      deleteCols ();
      
      /// expansion policy
      int expand_policy_;
      /// expansion direction
      int direction_;
      /// expansion amount
      int amount_;
      /// pointer to the input point cloud
      PointCloudPtr input_;
  };
}

#include <pcl/common/impl/spring.hpp>

#endif

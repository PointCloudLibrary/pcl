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

#ifndef PCL_SEGMENTATION_COMPARATOR_H_
#define PCL_SEGMENTATION_COMPARATOR_H_

#include <pcl/point_cloud.h>

namespace pcl
{
  /** \brief Comparator is the base class for comparators that compare two points given some function.
    * Currently intended for use with OrganizedConnectedComponentSegmentation
    *
    * \author Alex Trevor
    */
  template <typename PointT>
  class Comparator
  {
    public:
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<Comparator<PointT> > Ptr;
      typedef boost::shared_ptr<const Comparator<PointT> > ConstPtr;

      /** \brief Empty constructor for comparator. */
      Comparator () : input_ ()
      {
      }
      
      /** \brief Empty destructor for comparator. */
      virtual
      ~Comparator ()
      {
      }
      
      /** \brief Set the input cloud for the comparator.
        * \param[in] cloud the point cloud this comparator will operate on
        */
      virtual void 
      setInputCloud (const PointCloudConstPtr& cloud)
      {
        input_ = cloud;
      }
      
      /** \brief Get the input cloud this comparator operates on. */
      virtual PointCloudConstPtr
      getInputCloud () const
      {
        return (input_);
      }

      /** \brief Compares the two points in the input cloud designated by these two indices.
        * This is pure virtual and must be implemented by subclasses with some comparison function.
        * \param[in] idx1 the index of the first point.
        * \param[in] idx2 the index of the second point.
        */
      virtual bool
      compare (int idx1, int idx2) const = 0;
      
    protected:
      PointCloudConstPtr input_;
  };
}

#endif //#ifndef _PCL_SEGMENTATION_COMPARATOR_H_

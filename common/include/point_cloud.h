/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: point_cloud.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */

#ifndef PCL_POINT_CLOUD_H_
#define PCL_POINT_CLOUD_H_

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include "pcl/ros_macros.h"
#if ROS_VERSION_MINIMUM(1, 3, 0)
#include <std_msgs/Header.h>
#else
#include <roslib/Header.h>
#endif
#include <pcl/exceptions.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace pcl
{
  namespace detail 
  {
    struct FieldMapping
    {
      size_t serialized_offset;
      size_t struct_offset;
      size_t size;
    };
  } // namespace detail

  typedef std::vector<detail::FieldMapping> MsgFieldMap;

  // Forward declarations
  template <typename PointT> class PointCloud;

  namespace detail 
  {
    template <typename PointT>
    boost::shared_ptr<pcl::MsgFieldMap>& getMapping(pcl::PointCloud<PointT>& p);
  } // namespace detail

  /** @b PointCloud represents a templated PointCloud implementation. 
    * \author Patrick Mihelich, Radu Bogdan Rusu
    */
  template <typename PointT>
  class PointCloud
  {
    public:
      PointCloud () : width (0), height (0), is_dense (true), 
                      sensor_origin_ (Eigen::Vector4f::Zero ()), sensor_orientation_ (Eigen::Quaternionf::Identity ())
      {}
      
      ////////////////////////////////////////////////////////////////////////////////////////
      inline PointCloud& operator = (const PointCloud& rhs)
      {
        header   = rhs.header;
        points   = rhs.points;
        width    = rhs.width;
        height   = rhs.height;
        sensor_origin_ = rhs.sensor_origin_;
        sensor_orientation_ = rhs.sensor_orientation_;
        is_dense = rhs.is_dense;
        return (*this);
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      inline PointCloud& operator += (const PointCloud& rhs)
      {
        if (rhs.header.frame_id != header.frame_id)
        {
          ROS_ERROR ("PointCloud frame IDs do not match (%s != %s) for += . Cancelling operation...", 
                     rhs.header.frame_id.c_str (), header.frame_id.c_str ());
          return (*this);
        }

        // Make the resultant point cloud take the newest stamp
        if (rhs.header.stamp > header.stamp)
          header.stamp = rhs.header.stamp;

        size_t nr_points = points.size ();
        points.resize (nr_points + rhs.points.size ());
        for (size_t i = nr_points; i < points.size (); ++i)
          points[i] = rhs.points[i - nr_points];

        width    = points.size ();
        height   = 1;
        if (rhs.is_dense && is_dense)
          is_dense = true;
        else
          is_dense = false;
        return (*this);
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      inline PointT at (int u, int v) const
      {
        if (this->height > 1)
          return (points.at (v * this->width + u));
        else
          throw IsNotDenseException ("Can't use 2D indexing with a sparse point cloud");
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      inline const PointT& operator () (int u, int v) const
      {
        return (points[v * this->width + u]);
      }
      
      inline PointT& operator () (int u, int v)
      {
        return (points[v * this->width + u]);
      }

      /** \brief The point cloud header. It contains information about the acquisition time, as well as a transform 
        * frame (see \a tf). */
#if ROS_VERSION_MINIMUM(1, 3, 0)
      std_msgs::Header header;
#else
      roslib::Header header;
#endif

      /** \brief The point data. */
      std::vector<PointT, Eigen::aligned_allocator<PointT> > points;

      /** \brief The point cloud width (if organized as an image-structure). */
      uint32_t width;
      /** \brief The point cloud height (if organized as an image-structure). */
      uint32_t height;

      /** \brief True if no points are invalid (e.g., have NaN or Inf values). */
      bool is_dense;

      /** \brief Sensor acquisition pose (origin/translation). */
      Eigen::Vector4f    sensor_origin_;
      /** \brief Sensor acquisition pose (rotation). */
      Eigen::Quaternionf sensor_orientation_;

      typedef PointT PointType;  // Make the template class available from the outside
      typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > VectorType;
      typedef boost::shared_ptr<PointCloud<PointT> > Ptr;
      typedef boost::shared_ptr<const PointCloud<PointT> > ConstPtr;

      // iterators 
      typedef typename VectorType::iterator iterator;
      typedef typename VectorType::const_iterator const_iterator;
      inline iterator begin () { return (points.begin ()); }
      inline iterator end ()   { return (points.end ()); }
      inline const_iterator begin () const { return (points.begin ()); }
      inline const_iterator end () const  { return (points.end ()); }

      inline size_t size () const { return (points.size ()); }
      inline void push_back (const PointT& p) { points.push_back (p); }

      //inline Ptr makeShared () const { return (boost::make_shared<PointCloud<PointT> >) (*this); } 
      inline Ptr makeShared () const { return Ptr (new PointCloud<PointT> (*this)); } 

    protected:
      // This is motivated by ROS integration. Users should not need to access mapping_.
      boost::shared_ptr<MsgFieldMap> mapping_;

      friend boost::shared_ptr<MsgFieldMap>& detail::getMapping<PointT>(PointCloud& p);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  namespace detail 
  {
    template <typename PointT>
    boost::shared_ptr<pcl::MsgFieldMap>& getMapping(pcl::PointCloud<PointT>& p) { return p.mapping_; }
  } // namespace detail

  template <typename PointT>
  std::ostream& operator << (std::ostream& s, const pcl::PointCloud<PointT> &p)
  {
    s << "header: " << std::endl;
    s << p.header;
    s << "points[]: " << p.points.size () << std::endl;
    s << "width: " << p.width << std::endl;
    s << "height: " << p.height << std::endl;
    s << "sensor_origin_: " 
      << p.sensor_origin_[0] << ' '
      << p.sensor_origin_[1] << ' '
      << p.sensor_origin_[2] << std::endl;
    s << "sensor_orientation_: " 
      << p.sensor_orientation_.x() << ' '
      << p.sensor_orientation_.y() << ' '
      << p.sensor_orientation_.z() << ' '
      << p.sensor_orientation_.w() << std::endl;
    s << "is_dense: " << p.is_dense << std::endl;
    return (s);
  }
}

#endif  //#ifndef PCL_POINT_CLOUD_H_

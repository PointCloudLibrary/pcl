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

#ifndef PCL_POINT_CLOUD_H_
#define PCL_POINT_CLOUD_H_

#include <cstddef>
#include "pcl/pcl_macros.h"
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <std_msgs/Header.h>
#include <pcl/exceptions.h>
#include <pcl/console/print.h>
#include <boost/shared_ptr.hpp>

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
    template <typename PointT> boost::shared_ptr<pcl::MsgFieldMap>&
    getMapping (pcl::PointCloud<PointT>& p);
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

      /** \brief Copy constructor (needed by compilers such as Intel C++)
        * \param pc the cloud to copy into this
        */
      inline PointCloud (PointCloud<PointT> &pc)
      {
        *this = pc;
      }

      /** \brief Copy constructor (needed by compilers such as Intel C++)
        * \param pc the cloud to copy into this
        */
      inline PointCloud (const PointCloud<PointT> &pc)
      {
        *this = pc;
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      inline PointCloud&
      operator += (const PointCloud& rhs)
      {
        if (rhs.header.frame_id != header.frame_id)
        {
          PCL_ERROR ("PointCloud frame IDs do not match (%s != %s) for += . Cancelling operation...\n",
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

        width    = (uint32_t) points.size ();
        height   = 1;
        if (rhs.is_dense && is_dense)
          is_dense = true;
        else
          is_dense = false;
        return (*this);
      }
      
      ////////////////////////////////////////////////////////////////////////////////////////
      inline PointT
      at (int u, int v) const
      {
        if (this->height > 1)
          return (points.at (v * this->width + u));
        else
          throw IsNotDenseException ("Can't use 2D indexing with a sparse point cloud");
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      inline const PointT&
      operator () (int u, int v) const
      {
        return (points[v * this->width + u]);
      }

      inline PointT&
      operator () (int u, int v)
      {
        return (points[v * this->width + u]);
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return whether a dataset is organized (e.g., arranged in a structured grid).
        */
      inline bool
      isOrganized () const
      {
        return (height != 1);
      }
      
      ////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an Eigen MatrixXf (assumes float values) mapped to the specified dimensions of the PointCloud.
        * \param dim the number of dimensions to consider for each point (will become the number of rows)
        * \param stride the number of values in each point (will be the number of values that separate two of the columns)
        * \param offset the number of dimensions to skip from the beginning of each point
            (note stride = offset + dim + x, where x is the number of dimensions to skip from the end of each point)
        * \note for getting only XYZ coordinates out of PointXYZ use dim=3, stride=4 and offset=0 due to the alignment.
        */
      inline Eigen::MatrixXf
      getMatrixXfMap (int dim, int stride, int offset)
      {
        return Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >((float*)(&points[0])+offset, dim, points.size(), Eigen::OuterStride<>(stride));
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an Eigen MatrixXf (assumes float values) mapped to the PointCloud.
        */
      inline Eigen::MatrixXf
      getMatrixXfMap ()
      {
        return getMatrixXfMap (sizeof (PointT) / sizeof (float),  sizeof (PointT) / sizeof (float), 0);
      }

      /** \brief The point cloud header. It contains information about the acquisition time, as well as a transform
        * frame (see \a tf). */
      std_msgs::Header header;

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

      //capacity
      inline size_t size () const { return (points.size ()); }
      inline void reserve(size_t n) { points.reserve (n); }
      inline void resize(size_t n) { points.resize (n); }
      inline bool empty() const { return points.empty (); }

      //element access
      inline const PointT& operator[] (size_t n) const { return points[n]; }
      inline PointT& operator[] (size_t n) { return points[n]; }
      inline const PointT& at (size_t n) const { return points.at (n); }
      inline PointT& at (size_t n) { return points.at (n); }
      inline const PointT& front () const { return points.front (); }
      inline PointT& front () { return points.front (); }
      inline const PointT& back () const { return points.back (); }
      inline PointT& back () { return points.back (); }

      //modifiers
      inline void push_back (const PointT& p) { points.push_back (p); }
      inline iterator insert ( iterator position, const PointT& x )
      {
        iterator it = points.insert (position, x);
        width = points.size ();
        height = 1;
        return it;
      }
      inline void insert ( iterator position, size_t n, const PointT& x )
      {
        points.insert (position, n, x);
        width = points.size ();
        height = 1;
      }
      template <class InputIterator>
      inline void insert ( iterator position, InputIterator first, InputIterator last )
      {
        points.insert(position, first, last);
      }
      inline iterator erase ( iterator position )
      {
        iterator it = points.erase (position); 
        width = points.size ();
        height = 1;
        return it;
      }
      inline iterator erase ( iterator first, iterator last )
      {
        iterator it = points.erase (first, last);
        width = points.size ();
        height = 1;
        return it;
      }
      inline void swap (PointCloud<PointT> &rhs)
      {
        this->points.swap (rhs.points);
        width = points.size ();
        height = 1;
      }
      inline void clear ()
      {
        points.clear ();
        width = 0;
        height = 0;
      }

      /** \brief Copy the cloud to the heap and return a smart pointer
        * Note that deep copy is performed, so avoid using this function on non-empty clouds.
        * The changes of the returned cloud are not mirrored back to this one.
        * \return shared pointer to the copy of the cloud
        */
      inline Ptr makeShared () { return Ptr (new PointCloud<PointT> (*this)); }

      /** \brief Copy the cloud to the heap and return a constant smart pointer
        * Note that deep copy is performed, so avoid using this function on non-empty clouds.
        * \return const shared pointer to the copy of the cloud
        */
      inline ConstPtr makeShared () const { return ConstPtr (new PointCloud<PointT> (*this)); }

    protected:
      // This is motivated by ROS integration. Users should not need to access mapping_.
      boost::shared_ptr<MsgFieldMap> mapping_;

      friend boost::shared_ptr<MsgFieldMap>& detail::getMapping<PointT>(pcl::PointCloud<PointT> &p);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  namespace detail
  {
    template <typename PointT> boost::shared_ptr<pcl::MsgFieldMap>&
    getMapping (pcl::PointCloud<PointT>& p)
    {
      return (p.mapping_);
    }
  } // namespace detail

  template <typename PointT> std::ostream&
  operator << (std::ostream& s, const pcl::PointCloud<PointT> &p)
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

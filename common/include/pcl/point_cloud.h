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

  /** \brief PointCloud represents the base class in PCL for storing collections of 3D points.
    *
    * The class is templated, which means you need to specify the type of data
    * that it should contain. For example, to create a point cloud that holds 4
    * random XYZ data points, use:
    *
    * \code
    * pcl::PointCloud<pcl::PointXYZ> cloud;
    * cloud.push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
    * cloud.push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
    * cloud.push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
    * cloud.push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
    * \endcode
    *
    * The PointCloud class contains the following elements:
    * <ul>
    * <li><b>width</b> - specifies the width of the point cloud dataset in the number of points. WIDTH has two meanings:
    *   <ul><li>it can specify the total number of points in the cloud (equal with POINTS see below) for unorganized datasets;</li>
    *       <li>it can specify the width (total number of points in a row) of an organized point cloud dataset.</li>
    *   </ul>
    *   <i>Mandatory</i>.
    * </li>
    * <li><b>height</b> - specifies the height of the point cloud dataset in the number of points. HEIGHT has two meanings:
    *   <ul><li>it can specify the height (total number of rows) of an organized point cloud dataset;</li>
    *       <li>it is set to 1 for unorganized datasets (thus used to check whether a dataset is organized or not).</li>
    *   </ul>
    *   <i>Mandatory</i>.
    * </li>
    * <li><b>points</b> - the data array where all points of type <b>PointT</b> are stored. <i>Mandatory</i>.</li>
    *
    * <li><b>is_dense</b> - specifies if all the data in <b>points</b> is finite (true), or whether it might contain Inf/NaN values
    * (false). <i>Mandatory</i>.</li>
    *
    * <li><b>sensor_origin_</b> - specifies the sensor acquisition pose (origin/translation). <i>Optional</i>.</li>
    * <li><b>sensor_orientation_</b> - specifies the sensor acquisition pose (rotation). <i>Optional</i>.</li>
    * </ul>
    *
    * \author Patrick Mihelich, Radu B. Rusu
    */
  template <typename PointT>
  class PointCloud
  {
    public:
      /** \brief Default constructor. Sets \ref is_dense to true, \ref width
        * and \ref height to 0, and the \ref sensor_origin_ and \ref
        * sensor_orientation_ to identity.
        */
      PointCloud () : width (0), height (0), is_dense (true),
                      sensor_origin_ (Eigen::Vector4f::Zero ()), sensor_orientation_ (Eigen::Quaternionf::Identity ())
      {}

      /** \brief Copy constructor (needed by compilers such as Intel C++)
        * \param[in] pc the cloud to copy into this
        */
      inline PointCloud (PointCloud<PointT> &pc)
      {
        *this = pc;
      }

      /** \brief Copy constructor (needed by compilers such as Intel C++)
        * \param[in] pc the cloud to copy into this
        */
      inline PointCloud (const PointCloud<PointT> &pc)
      {
        *this = pc;
      }

      /** \brief Copy constructor from point cloud subset
        * \param[in] pc the cloud to copy into this
        * \param[in] indices the subset to copy
        */
      inline PointCloud (const PointCloud<PointT> &pc, 
                         const std::vector<size_t> &indices)
      {
        assert(indices.size () <= pc.size ());
        this->resize (indices.size ());
        for(size_t i = 0; i < indices.size (); i++)
        {
          this->push_back (pc[indices[i]]);
        }
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Add a point cloud to the current cloud.
        * \param[in] rhs the cloud to add to the current cloud
        * \return the new cloud as a concatenation of the current cloud and the new given cloud
        */ 
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
      /** \brief Obtain the point given by the (u, v) coordinates. Only works on organized 
        * datasets (those that have height != 1).
        * \param[in] u the u coordinate
        * \param[in] v the v coordinate
        */
      inline const PointT&
      at (int u, int v) const
      {
        if (this->height > 1)
          return (points.at (v * this->width + u));
        else
          throw IsNotDenseException ("Can't use 2D indexing with a sparse point cloud");
      }

      /** \brief Obtain the point given by the (u, v) coordinates. Only works on organized 
        * datasets (those that have height != 1).
        * \param[in] u the u coordinate
        * \param[in] v the v coordinate
        */
      inline PointT&
      at (int u, int v)
      {
        if (this->height > 1)
          return (points.at (v * this->width + u));
        else
          throw IsNotDenseException ("Can't use 2D indexing with a sparse point cloud");
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Obtain the point given by the (u, v) coordinates. Only works on organized 
        * datasets (those that have height != 1).
        * \param[in] u the u coordinate
        * \param[in] v the v coordinate
        */
      inline const PointT&
      operator () (int u, int v) const
      {
        return (points[v * this->width + u]);
      }

      /** \brief Obtain the point given by the (u, v) coordinates. Only works on organized 
        * datasets (those that have height != 1).
        * \param[in] u the u coordinate
        * \param[in] v the v coordinate
        */
      inline PointT&
      operator () (int u, int v)
      {
        return (points[v * this->width + u]);
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return whether a dataset is organized (e.g., arranged in a structured grid).
        * \note The height value must be different than 1 for a dataset to be organized.
        */
      inline bool
      isOrganized () const
      {
        return (height != 1);
      }
      
      ////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an Eigen MatrixXf (assumes float values) mapped to the specified dimensions of the PointCloud.
        * \anchor getMatrixXfMap
        * \note This method is for advanced users only! Use with care!
        * \param[in] dim the number of dimensions to consider for each point (will become the number of rows)
        * \param[in] stride the number of values in each point (will be the number of values that separate two of the columns)
        * \param[in] offset the number of dimensions to skip from the beginning of each point
        *            (stride = offset + dim + x, where x is the number of dimensions to skip from the end of each point)
        * \note for getting only XYZ coordinates out of PointXYZ use dim=3, stride=4 and offset=0 due to the alignment.
        * \attention PointT types are most of the time aligned, so the offsets are not continuous! 
        */
      inline Eigen::MatrixXf
      getMatrixXfMap (int dim, int stride, int offset) const
      {
        return Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >((float*)(&points[0])+offset, dim, points.size(), Eigen::OuterStride<>(stride));
      }

      ////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an Eigen MatrixXf (assumes float values) mapped to the PointCloud.
        * \note This method is for advanced users only! Use with care!
        * \attention PointT types are most of the time aligned, so the offsets are not continuous! 
        * See \ref getMatrixXfMap for more information.
        */
      inline Eigen::MatrixXf
      getMatrixXfMap () const
      {
        return getMatrixXfMap (sizeof (PointT) / sizeof (float),  sizeof (PointT) / sizeof (float), 0);
      }

      /** \brief The point cloud header. It contains information about the acquisition time. */
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
      inline void reserve (size_t n) { points.reserve (n); }
      inline void resize (size_t n) { points.resize (n); }
      inline bool empty () const { return points.empty (); }

      //element access
      inline const PointT& operator[] (size_t n) const { return points[n]; }
      inline PointT& operator[] (size_t n) { return points[n]; }
      inline const PointT& at (size_t n) const { return points.at (n); }
      inline PointT& at (size_t n) { return points.at (n); }
      inline const PointT& front () const { return points.front (); }
      inline PointT& front () { return points.front (); }
      inline const PointT& back () const { return points.back (); }
      inline PointT& back () { return points.back (); }

      /** \brief Insert a new point in the cloud, at the end of the container.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] pt the point to insert
        */
      inline void 
      push_back (const PointT& pt)
      {
        points.push_back (pt);
        width = points.size ();
        height = 1;
      }

      /** \brief Insert a new point in the cloud, given an iterator.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position where to insert the point
        * \param[in] pt the point to insert
        * \return returns the new position iterator
        */
      inline iterator 
      insert (iterator position, const PointT& pt)
      {
        iterator it = points.insert (position, pt);
        width = points.size ();
        height = 1;
        return (it);
      }

      /** \brief Insert a new point in the cloud N times, given an iterator.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position where to insert the point
        * \param[in] n the number of times to insert the point
        * \param[in] pt the point to insert
        */
      inline void 
      insert (iterator position, size_t n, const PointT& pt)
      {
        points.insert (position, n, pt);
        width = points.size ();
        height = 1;
      }

      /** \brief Insert a new range of points in the cloud, at a certain position.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position where to insert the data
        * \param[in] first where to start inserting the points from
        * \param[in] last where to stop inserting the points from
        */
      template <class InputIterator> inline void 
      insert (iterator position, InputIterator first, InputIterator last)
      {
        points.insert (position, first, last);
      }

      /** \brief Erase a point in the cloud. 
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position what data point to erase
        * \return returns the new position iterator
        */
      inline iterator 
      erase (iterator position)
      {
        iterator it = points.erase (position); 
        width = points.size ();
        height = 1;
        return (it);
      }

      /** \brief Erase a set of points given by a (first, last) iterator pair
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] first where to start erasing points from
        * \param[in] last where to stop erasing points from
        * \return returns the new position iterator
        */
      inline iterator 
      erase (iterator first, iterator last)
      {
        iterator it = points.erase (first, last);
        width = points.size ();
        height = 1;
        return (it);
      }

      /** \brief Swap a point cloud with another cloud.
        * \param rhs point cloud to swap this with
        */ 
      inline void 
      swap (PointCloud<PointT> &rhs)
      {
        this->points.swap (rhs.points);
        std::swap (width, rhs.width);
        std::swap (height, rhs.height);
        std::swap (is_dense, rhs.is_dense);
        std::swap (sensor_origin_, rhs.sensor_origin_);
        std::swap (sensor_orientation_, rhs.sensor_orientation_);
      }

      /** \brief Removes all points in a cloud and sets the width and height to 0. */
      inline void 
      clear ()
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

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifdef __GNUC__
#pragma GCC system_header
#endif

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/PCLHeader.h>
#include <pcl/exceptions.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_traits.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace pcl
{
  namespace detail
  {
    struct FieldMapping
    {
      std::size_t serialized_offset;
      std::size_t struct_offset;
      std::size_t size;
    };
  } // namespace detail

  // Forward declarations
  template <typename PointT> class PointCloud;
  using MsgFieldMap = std::vector<detail::FieldMapping>;

  /** \brief Helper functor structure for copying data between an Eigen type and a PointT. */
  template <typename PointOutT>
  struct NdCopyEigenPointFunctor
  {
    using Pod = typename traits::POD<PointOutT>::type;

    /** \brief Constructor
      * \param[in] p1 the input Eigen type
      * \param[out] p2 the output Point type
      */
    NdCopyEigenPointFunctor (const Eigen::VectorXf &p1, PointOutT &p2)
      : p1_ (p1),
        p2_ (reinterpret_cast<Pod&>(p2)),
        f_idx_ (0) { }

    /** \brief Operator. Data copy happens here. */
    template<typename Key> inline void
    operator() ()
    {
      //boost::fusion::at_key<Key> (p2_) = p1_[f_idx_++];
      using T = typename pcl::traits::datatype<PointOutT, Key>::type;
      std::uint8_t* data_ptr = reinterpret_cast<std::uint8_t*>(&p2_) + pcl::traits::offset<PointOutT, Key>::value;
      *reinterpret_cast<T*>(data_ptr) = static_cast<T> (p1_[f_idx_++]);
    }

    private:
      const Eigen::VectorXf &p1_;
      Pod &p2_;
      int f_idx_;
    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
   };

  /** \brief Helper functor structure for copying data between an Eigen type and a PointT. */
  template <typename PointInT>
  struct NdCopyPointEigenFunctor
  {
    using Pod = typename traits::POD<PointInT>::type;

    /** \brief Constructor
      * \param[in] p1 the input Point type
      * \param[out] p2 the output Eigen type
      */
     NdCopyPointEigenFunctor (const PointInT &p1, Eigen::VectorXf &p2)
      : p1_ (reinterpret_cast<const Pod&>(p1)), p2_ (p2), f_idx_ (0) { }

    /** \brief Operator. Data copy happens here. */
    template<typename Key> inline void
    operator() ()
    {
      //p2_[f_idx_++] = boost::fusion::at_key<Key> (p1_);
      using T = typename pcl::traits::datatype<PointInT, Key>::type;
      const std::uint8_t* data_ptr = reinterpret_cast<const std::uint8_t*>(&p1_) + pcl::traits::offset<PointInT, Key>::value;
      p2_[f_idx_++] = static_cast<float> (*reinterpret_cast<const T*>(data_ptr));
    }

    private:
      const Pod &p1_;
      Eigen::VectorXf &p2_;
      int f_idx_;
    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

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
    *   - \b width - specifies the width of the point cloud dataset in the number of points. WIDTH has two meanings:
    *     - it can specify the total number of points in the cloud (equal with POINTS see below) for unorganized datasets;
    *     - it can specify the width (total number of points in a row) of an organized point cloud dataset.
    *   \a Mandatory.
    *   - \b height - specifies the height of the point cloud dataset in the number of points. HEIGHT has two meanings:
    *     - it can specify the height (total number of rows) of an organized point cloud dataset;
    *     - it is set to 1 for unorganized datasets (thus used to check whether a dataset is organized or not).
    *   \a Mandatory.
    *   - \b points - the data array where all points of type <b>PointT</b> are stored. \a Mandatory.
    *
    *   - \b is_dense - specifies if all the data in <b>points</b> is finite (true), or whether it might contain Inf/NaN values
    * (false). \a Mandatory.
    *
    *   - \b sensor_origin_ - specifies the sensor acquisition pose (origin/translation). \a Optional.
    *   - \b sensor_orientation_ - specifies the sensor acquisition pose (rotation). \a Optional.
    *
    * \author Patrick Mihelich, Radu B. Rusu
    */
  template <typename PointT>
  class PCL_EXPORTS PointCloud
  {
    public:
      /** \brief Default constructor. Sets \ref is_dense to true, \ref width
        * and \ref height to 0, and the \ref sensor_origin_ and \ref
        * sensor_orientation_ to identity.
        */
      PointCloud () = default;

      /** \brief Copy constructor from point cloud subset
        * \param[in] pc the cloud to copy into this
        * \param[in] indices the subset to copy
        */
      PointCloud (const PointCloud<PointT> &pc,
                  const std::vector<int> &indices) :
        header (pc.header), points (indices.size ()), width (indices.size ()), height (1), is_dense (pc.is_dense),
        sensor_origin_ (pc.sensor_origin_), sensor_orientation_ (pc.sensor_orientation_)
      {
        // Copy the obvious
        assert (indices.size () <= pc.size ());
        for (std::size_t i = 0; i < indices.size (); i++)
          points[i] = pc.points[indices[i]];
      }

      /** \brief Allocate constructor from point cloud subset
        * \param[in] width_ the cloud width
        * \param[in] height_ the cloud height
        * \param[in] value_ default value
        */
      PointCloud (std::uint32_t width_, std::uint32_t height_, const PointT& value_ = PointT ())
        : points (width_ * height_, value_)
        , width (width_)
        , height (height_)
      {}

      //TODO: check if copy/move contructors/assignment operators are needed

      /** \brief Add a point cloud to the current cloud.
        * \param[in] rhs the cloud to add to the current cloud
        * \return the new cloud as a concatenation of the current cloud and the new given cloud
        */
      inline PointCloud&
      operator += (const PointCloud& rhs)
      {
        concatenate((*this), rhs);
        return (*this);
      }

      /** \brief Add a point cloud to another cloud.
        * \param[in] rhs the cloud to add to the current cloud
        * \return the new cloud as a concatenation of the current cloud and the new given cloud
        */
      inline PointCloud
      operator + (const PointCloud& rhs)
      {
        return (PointCloud (*this) += rhs);
      }

      inline static bool
      concatenate(pcl::PointCloud<PointT> &cloud1,
                  const pcl::PointCloud<PointT> &cloud2)
      {
        // Make the resultant point cloud take the newest stamp
        cloud1.header.stamp = std::max (cloud1.header.stamp, cloud2.header.stamp);

        // libstdc++ (GCC) on calling reserve allocates new memory, copies and deallocates old vector
        // This causes a drastic performance hit. Prefer not to use reserve with libstdc++ (default on clang)
        cloud1.points.insert (cloud1.points.end (), cloud2.points.begin (), cloud2.points.end ());

        cloud1.width    = static_cast<std::uint32_t>(cloud1.points.size ());
        cloud1.height   = 1;
        cloud1.is_dense = cloud1.is_dense && cloud2.is_dense;
        return true;
      }

      inline static bool
      concatenate(const pcl::PointCloud<PointT> &cloud1,
               const pcl::PointCloud<PointT> &cloud2,
               pcl::PointCloud<PointT> &cloud_out)
      {
        cloud_out = cloud1;
        return concatenate(cloud_out, cloud2);
      }

      /** \brief Obtain the point given by the (column, row) coordinates. Only works on organized
        * datasets (those that have height != 1).
        * \param[in] column the column coordinate
        * \param[in] row the row coordinate
        */
      inline const PointT&
      at (int column, int row) const
      {
        if (this->height > 1)
          return (points.at (row * this->width + column));
        else
          throw UnorganizedPointCloudException ("Can't use 2D indexing with an unorganized point cloud");
      }

      /** \brief Obtain the point given by the (column, row) coordinates. Only works on organized
        * datasets (those that have height != 1).
        * \param[in] column the column coordinate
        * \param[in] row the row coordinate
        */
      inline PointT&
      at (int column, int row)
      {
        if (this->height > 1)
          return (points.at (row * this->width + column));
        else
          throw UnorganizedPointCloudException ("Can't use 2D indexing with an unorganized point cloud");
      }

      /** \brief Obtain the point given by the (column, row) coordinates. Only works on organized
        * datasets (those that have height != 1).
        * \param[in] column the column coordinate
        * \param[in] row the row coordinate
        */
      inline const PointT&
      operator () (std::size_t column, std::size_t row) const
      {
        return (points[row * this->width + column]);
      }

      /** \brief Obtain the point given by the (column, row) coordinates. Only works on organized
        * datasets (those that have height != 1).
        * \param[in] column the column coordinate
        * \param[in] row the row coordinate
        */
      inline PointT&
      operator () (std::size_t column, std::size_t row)
      {
        return (points[row * this->width + column]);
      }

      /** \brief Return whether a dataset is organized (e.g., arranged in a structured grid).
        * \note The height value must be different than 1 for a dataset to be organized.
        */
      inline bool
      isOrganized () const
      {
        return (height > 1);
      }

      /** \brief Return an Eigen MatrixXf (assumes float values) mapped to the specified dimensions of the PointCloud.
        * \anchor getMatrixXfMap
        * \note This method is for advanced users only! Use with care!
        *
        * \attention Since 1.4.0, Eigen matrices are forced to Row Major to increase the efficiency of the algorithms in PCL
        *   This means that the behavior of getMatrixXfMap changed, and is now correctly mapping 1-1 with a PointCloud structure,
        *   that is: number of points in a cloud = rows in a matrix, number of point dimensions = columns in a matrix
        *
        * \param[in] dim the number of dimensions to consider for each point
        * \param[in] stride the number of values in each point (will be the number of values that separate two of the columns)
        * \param[in] offset the number of dimensions to skip from the beginning of each point
        *            (stride = offset + dim + x, where x is the number of dimensions to skip from the end of each point)
        * \note for getting only XYZ coordinates out of PointXYZ use dim=3, stride=4 and offset=0 due to the alignment.
        * \attention PointT types are most of the time aligned, so the offsets are not continuous!
        */
      inline Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >
      getMatrixXfMap (int dim, int stride, int offset)
      {
        if (Eigen::MatrixXf::Flags & Eigen::RowMajorBit)
          return (Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >(reinterpret_cast<float*>(&points[0])+offset, points.size (), dim, Eigen::OuterStride<> (stride)));
        else
          return (Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >(reinterpret_cast<float*>(&points[0])+offset, dim, points.size (), Eigen::OuterStride<> (stride)));
      }

      /** \brief Return an Eigen MatrixXf (assumes float values) mapped to the specified dimensions of the PointCloud.
        * \anchor getMatrixXfMap
        * \note This method is for advanced users only! Use with care!
        *
        * \attention Since 1.4.0, Eigen matrices are forced to Row Major to increase the efficiency of the algorithms in PCL
        *   This means that the behavior of getMatrixXfMap changed, and is now correctly mapping 1-1 with a PointCloud structure,
        *   that is: number of points in a cloud = rows in a matrix, number of point dimensions = columns in a matrix
        *
        * \param[in] dim the number of dimensions to consider for each point
        * \param[in] stride the number of values in each point (will be the number of values that separate two of the columns)
        * \param[in] offset the number of dimensions to skip from the beginning of each point
        *            (stride = offset + dim + x, where x is the number of dimensions to skip from the end of each point)
        * \note for getting only XYZ coordinates out of PointXYZ use dim=3, stride=4 and offset=0 due to the alignment.
        * \attention PointT types are most of the time aligned, so the offsets are not continuous!
        */
      inline const Eigen::Map<const Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >
      getMatrixXfMap (int dim, int stride, int offset) const
      {
        if (Eigen::MatrixXf::Flags & Eigen::RowMajorBit)
          return (Eigen::Map<const Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >(reinterpret_cast<float*>(const_cast<PointT*>(&points[0]))+offset, points.size (), dim, Eigen::OuterStride<> (stride)));
        else
          return (Eigen::Map<const Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >(reinterpret_cast<float*>(const_cast<PointT*>(&points[0]))+offset, dim, points.size (), Eigen::OuterStride<> (stride)));
      }

      /** \brief Return an Eigen MatrixXf (assumes float values) mapped to the PointCloud.
        * \note This method is for advanced users only! Use with care!
        * \attention PointT types are most of the time aligned, so the offsets are not continuous!
        * See \ref getMatrixXfMap for more information.
        */
      inline Eigen::Map<Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >
      getMatrixXfMap ()
      {
        return (getMatrixXfMap (sizeof (PointT) / sizeof (float),  sizeof (PointT) / sizeof (float), 0));
      }

      /** \brief Return an Eigen MatrixXf (assumes float values) mapped to the PointCloud.
        * \note This method is for advanced users only! Use with care!
        * \attention PointT types are most of the time aligned, so the offsets are not continuous!
        * See \ref getMatrixXfMap for more information.
        */
      inline const Eigen::Map<const Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> >
      getMatrixXfMap () const
      {
        return (getMatrixXfMap (sizeof (PointT) / sizeof (float),  sizeof (PointT) / sizeof (float), 0));
      }

      /** \brief The point cloud header. It contains information about the acquisition time. */
      pcl::PCLHeader header;

      /** \brief The point data. */
      std::vector<PointT, Eigen::aligned_allocator<PointT> > points;

      /** \brief The point cloud width (if organized as an image-structure). */
      std::uint32_t width = 0;
      /** \brief The point cloud height (if organized as an image-structure). */
      std::uint32_t height = 0;

      /** \brief True if no points are invalid (e.g., have NaN or Inf values in any of their floating point fields). */
      bool is_dense = true;

      /** \brief Sensor acquisition pose (origin/translation). */
      Eigen::Vector4f    sensor_origin_ = Eigen::Vector4f::Zero ();
      /** \brief Sensor acquisition pose (rotation). */
      Eigen::Quaternionf sensor_orientation_ = Eigen::Quaternionf::Identity ();

      using PointType = PointT;  // Make the template class available from the outside
      using VectorType = std::vector<PointT, Eigen::aligned_allocator<PointT> >;
      using CloudVectorType = std::vector<PointCloud<PointT>, Eigen::aligned_allocator<PointCloud<PointT> > >;
      using Ptr = boost::shared_ptr<PointCloud<PointT> >;
      using ConstPtr = boost::shared_ptr<const PointCloud<PointT> >;

      // std container compatibility typedefs according to
      // http://en.cppreference.com/w/cpp/concept/Container
      using value_type = PointT;
      using reference = PointT&;
      using const_reference = const PointT&;
      using difference_type = typename VectorType::difference_type;
      using size_type = typename VectorType::size_type;

      // iterators
      using iterator = typename VectorType::iterator;
      using const_iterator = typename VectorType::const_iterator;
      inline iterator begin () { return (points.begin ()); }
      inline iterator end ()   { return (points.end ()); }
      inline const_iterator begin () const { return (points.begin ()); }
      inline const_iterator end () const  { return (points.end ()); }

      //capacity
      inline std::size_t size () const { return (points.size ()); }
      inline void reserve (std::size_t n) { points.reserve (n); }
      inline bool empty () const { return points.empty (); }

      /** \brief Resize the cloud
        * \param[in] n the new cloud size
        */
      inline void resize (std::size_t n)
      {
        points.resize (n);
        if (width * height != n)
        {
          width = static_cast<std::uint32_t> (n);
          height = 1;
        }
      }

      //element access
      inline const PointT& operator[] (std::size_t n) const { return (points[n]); }
      inline PointT& operator[] (std::size_t n) { return (points[n]); }
      inline const PointT& at (std::size_t n) const { return (points.at (n)); }
      inline PointT& at (std::size_t n) { return (points.at (n)); }
      inline const PointT& front () const { return (points.front ()); }
      inline PointT& front () { return (points.front ()); }
      inline const PointT& back () const { return (points.back ()); }
      inline PointT& back () { return (points.back ()); }

      /** \brief Insert a new point in the cloud, at the end of the container.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] pt the point to insert
        */
      inline void
      push_back (const PointT& pt)
      {
        points.push_back (pt);
        width = static_cast<std::uint32_t> (points.size ());
        height = 1;
      }

      /** \brief Emplace a new point in the cloud, at the end of the container.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] args the parameters to forward to the point to construct
        * \return reference to the emplaced point
        */
      template <class... Args> inline reference
      emplace_back (Args&& ...args)
      {
        points.emplace_back (std::forward<Args> (args)...);
        width = static_cast<std::uint32_t> (points.size ());
        height = 1;
        return points.back();
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
        width = static_cast<std::uint32_t> (points.size ());
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
      insert (iterator position, std::size_t n, const PointT& pt)
      {
        points.insert (position, n, pt);
        width = static_cast<std::uint32_t> (points.size ());
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
        width = static_cast<std::uint32_t> (points.size ());
        height = 1;
      }

      /** \brief Emplace a new point in the cloud, given an iterator.
        * \note This breaks the organized structure of the cloud by setting the height to 1!
        * \param[in] position iterator before which the point will be emplaced
        * \param[in] args the parameters to forward to the point to construct
        * \return returns the new position iterator
        */
      template <class... Args> inline iterator
      emplace (iterator position, Args&& ...args)
      {
        iterator it = points.emplace (position, std::forward<Args> (args)...);
        width = static_cast<std::uint32_t> (points.size ());
        height = 1;
        return (it);
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
        width = static_cast<std::uint32_t> (points.size ());
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
        width = static_cast<std::uint32_t> (points.size ());
        height = 1;
        return (it);
      }

      /** \brief Swap a point cloud with another cloud.
        * \param[in,out] rhs point cloud to swap this with
        */
      inline void
      swap (PointCloud<PointT> &rhs)
      {
        std::swap (header, rhs.header);
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
      inline Ptr
      makeShared () const { return Ptr (new PointCloud<PointT> (*this)); }

    protected:
      // This is motivated by ROS integration. Users should not need to access mapping_.
      boost::shared_ptr<MsgFieldMap> mapping_;

      friend boost::shared_ptr<MsgFieldMap>& detail::getMapping<PointT>(pcl::PointCloud<PointT> &p);

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
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
    s << "header: " << p.header << std::endl;
    s << "points[]: " << p.points.size () << std::endl;
    s << "width: " << p.width << std::endl;
    s << "height: " << p.height << std::endl;
    s << "is_dense: " << p.is_dense << std::endl;
    s << "sensor origin (xyz): [" <<
      p.sensor_origin_.x () << ", " <<
      p.sensor_origin_.y () << ", " <<
      p.sensor_origin_.z () << "] / orientation (xyzw): [" <<
      p.sensor_orientation_.x () << ", " <<
      p.sensor_orientation_.y () << ", " <<
      p.sensor_orientation_.z () << ", " <<
      p.sensor_orientation_.w () << "]" <<
      std::endl;
    return (s);
  }
}

#define PCL_INSTANTIATE_PointCloud(T) template class PCL_EXPORTS pcl::PointCloud<T>;

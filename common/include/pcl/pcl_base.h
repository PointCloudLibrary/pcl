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
 * $Id$
 *
 */
#ifndef PCL_PCL_BASE_H_
#define PCL_PCL_BASE_H_

#include <cstddef>
// Eigen includes
#include <pcl/common/eigen.h>
// STD includes
#include <vector>

// Include PCL macros such as PCL_ERROR, etc
#include <pcl/pcl_macros.h>

// Boost includes. Needed everywhere.
#include <boost/shared_ptr.hpp>

// Point Cloud message includes. Needed everywhere.
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/console/print.h>

namespace pcl
{
  // definitions used everywhere
  typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
  typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief PCL base class. Implements methods that are used by all PCL objects. 
    * \ingroup common 
    */
  template <typename PointT>
  class PCLBase
  {
    public:
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      /** \brief Empty constructor. */
      PCLBase () : input_ (), indices_ (), use_indices_ (false), fake_indices_ (false) {}
      
      /** \brief Copy constructor. */
      PCLBase (const PCLBase& base)
        : input_ (base.input_)
        , indices_ (base.indices_)
        , use_indices_ (base.use_indices_)
        , fake_indices_ (base.fake_indices_)
      {}

      /** \brief destructor. */
      virtual ~PCLBase() 
      {
        input_.reset ();
        indices_.reset ();
      }
      
      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      virtual inline void 
      setInputCloud (const PointCloudConstPtr &cloud) { input_ = cloud; }

      /** \brief Get a pointer to the input point cloud dataset. */
      inline PointCloudConstPtr const 
      getInputCloud () { return (input_); }

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param indices a pointer to the vector of indices that represents the input data.
        */
      inline void
      setIndices (const IndicesPtr &indices)
      {
        indices_ = indices;
        fake_indices_ = false;
        use_indices_  = true;
      }

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param indices a pointer to the vector of indices that represents the input data.
        */
      inline void
      setIndices (const IndicesConstPtr &indices)
      {
        indices_.reset (new std::vector<int> (*indices));
        fake_indices_ = false;
        use_indices_  = true;
      }

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param indices a pointer to the vector of indices that represents the input data.
        */
      inline void
      setIndices (const PointIndicesConstPtr &indices)
      {
        indices_.reset (new std::vector<int> (indices->indices));
        fake_indices_ = false;
        use_indices_  = true;
      }

      /** \brief Set the indices for the points laying within an interest region of 
        * the point cloud.
        * \note you shouldn't call this method on unorganized point clouds!
        * \param row_start the offset on rows
        * \param col_start the offset on columns
        * \param nb_rows the number of rows to be considered row_start included
        * \param nb_cols the number of columns to be considered col_start included
        */
      inline void 
      setIndices (size_t row_start, size_t col_start, size_t nb_rows, size_t nb_cols)
      {
        if ((nb_rows > input_->height) || (row_start > input_->height))
        {
          PCL_ERROR ("[PCLBase::setIndices] cloud is only %d height", input_->height);
          return;
        }

        if ((nb_cols > input_->width) || (col_start > input_->width))
        {
          PCL_ERROR ("[PCLBase::setIndices] cloud is only %d width", input_->width);
          return;
        }

        size_t row_end = row_start + nb_rows;
        if (row_end > input_->height)
        {
          PCL_ERROR ("[PCLBase::setIndices] %d is out of rows range %d", row_end, input_->height);
          return;
        }

        size_t col_end = col_start + nb_cols;
        if (col_end > input_->width)
        {
          PCL_ERROR ("[PCLBase::setIndices] %d is out of columns range %d", col_end, input_->width);
          return;
        }

        indices_.reset (new std::vector<int>);
        indices_->reserve (nb_cols * nb_rows);
        for(size_t i = row_start; i < row_end; i++)
          for(size_t j = col_start; j < col_end; j++)
            indices_->push_back (static_cast<int> ((i * input_->width) + j));
        fake_indices_ = false;
        use_indices_  = true;
      }

      /** \brief Get a pointer to the vector of indices used. */
      inline IndicesPtr const 
      getIndices () { return (indices_); }

      /** \brief Override PointCloud operator[] to shorten code
        * \note this method can be called instead of (*input_)[(*indices_)[pos]]
        * or input_->points[(*indices_)[pos]]
        * \param pos position in indices_ vector
        */
      const PointT& operator[] (size_t pos)
      {
        return ((*input_)[(*indices_)[pos]]);
      }

    protected:
      /** \brief The input point cloud dataset. */
      PointCloudConstPtr input_;

      /** \brief A pointer to the vector of point indices to use. */
      IndicesPtr indices_;

      /** \brief Set to true if point indices are used. */
      bool use_indices_;

      /** \brief If no set of indices are given, we construct a set of fake indices that mimic the input PointCloud. */
      bool fake_indices_;

      /** \brief This method should get called before starting the actual computation. 
        *
        * Internally, initCompute() does the following:
        *   - checks if an input dataset is given, and returns false otherwise
        *   - checks whether a set of input indices has been given. Returns true if yes.
        *   - if no input indices have been given, a fake set is created, which will be used until:
        *     - either a new set is given via setIndices(), or 
        *     - a new cloud is given that has a different set of points. This will trigger an update on the set of fake indices
        */
      inline bool
      initCompute ()
      {
        // Check if input was set
        if (!input_)
          return (false);

        // If no point indices have been given, construct a set of indices for the entire input point cloud
        if (!indices_)
        {
          fake_indices_ = true;
          indices_.reset (new std::vector<int>);
          try
          {
            indices_->resize (input_->points.size ());
          }
          catch (std::bad_alloc)
          {
            PCL_ERROR ("[initCompute] Failed to allocate %zu indices.\n", input_->points.size ());
          }
          for (size_t i = 0; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
        }

        // If we have a set of fake indices, but they do not match the number of points in the cloud, update them
        if (fake_indices_ && indices_->size () != input_->points.size ())
        {
          size_t indices_size = indices_->size ();
          indices_->resize (input_->points.size ());
          for (size_t i = indices_size; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
        }

        return (true);
      }

      /** \brief This method should get called after finishing the actual computation. 
        *
        */
      inline bool
      deinitCompute ()
      {
        return (true);
      }
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  ////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class PCL_EXPORTS PCLBase<sensor_msgs::PointCloud2>
  {
    public:
      typedef sensor_msgs::PointCloud2 PointCloud2;
      typedef PointCloud2::Ptr PointCloud2Ptr;
      typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      /** \brief Empty constructor. */
      PCLBase () : input_ (), indices_ (), use_indices_ (false), fake_indices_ (false),
                   field_sizes_ (0), x_idx_ (-1), y_idx_ (-1), z_idx_ (-1),
                   x_field_name_ ("x"), y_field_name_ ("y"), z_field_name_ ("z")
      {};

      /** \brief destructor. */
      virtual ~PCLBase() 
      {
        input_.reset ();
        indices_.reset ();
      }

      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      void 
      setInputCloud (const PointCloud2ConstPtr &cloud);

      /** \brief Get a pointer to the input point cloud dataset. */
      inline PointCloud2ConstPtr const 
      getInputCloud () { return (input_); }

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param indices a pointer to the vector of indices that represents the input data.
        */
      inline void
      setIndices (const IndicesPtr &indices)
      {
        indices_ = indices;
        fake_indices_ = false;
        use_indices_  = true;
      }

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param indices a pointer to the vector of indices that represents the input data.
        */
      inline void
      setIndices (const PointIndicesConstPtr &indices)
      {
        indices_.reset (new std::vector<int> (indices->indices));
        fake_indices_ = false;
        use_indices_  = true;
      }

      /** \brief Get a pointer to the vector of indices used. */
      inline IndicesPtr const 
      getIndices () { return (indices_); }

    protected:
      /** \brief The input point cloud dataset. */
      PointCloud2ConstPtr input_;

      /** \brief A pointer to the vector of point indices to use. */
      IndicesPtr indices_;

      /** \brief Set to true if point indices are used. */
      bool use_indices_;

      /** \brief If no set of indices are given, we construct a set of fake indices that mimic the input PointCloud. */
      bool fake_indices_;

      /** \brief The size of each individual field. */
      std::vector<int> field_sizes_;

      /** \brief The x-y-z fields indices. */
      int x_idx_, y_idx_, z_idx_;

      /** \brief The desired x-y-z field names. */
      std::string x_field_name_, y_field_name_, z_field_name_;

      bool initCompute ();
      bool deinitCompute ();
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_PCL_BASE_H_

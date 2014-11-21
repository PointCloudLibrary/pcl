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
#ifndef PCL_PCL_BASE_H_
#define PCL_PCL_BASE_H_

#if defined __GNUC__
#  pragma GCC system_header
#endif

// Include PCL macros such as PCL_ERROR, etc
#include <pcl/pcl_macros.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/StdVector>
#include <Eigen/Core>

// Point Cloud message includes. Needed everywhere.
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/PCLPointCloud2.h>

namespace pcl
{
  // definitions used everywhere
  typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
  typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;

  /////////////////////////////////////////////////////////////////////////////////////////
  /** \brief PCL base class. Implements methods that are used by most PCL algorithms. 
    * \ingroup common 
    */
  template <typename PointT>
  class PCLBase
  {
    public:
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<PointIndices> PointIndicesPtr;
      typedef boost::shared_ptr<PointIndices const> PointIndicesConstPtr;

      /** \brief Empty constructor. */
      PCLBase ();
      
      /** \brief Copy constructor. */
      PCLBase (const PCLBase& base);

      /** \brief Destructor. */
      virtual ~PCLBase ()
      {
        input_.reset ();
        indices_.reset ();
      }
      
      /** \brief Provide a pointer to the input dataset
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      virtual void 
      setInputCloud (const PointCloudConstPtr &cloud);

      /** \brief Get a pointer to the input point cloud dataset. */
      inline PointCloudConstPtr const 
      getInputCloud () const { return (input_); }

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param[in] indices a pointer to the indices that represent the input data.
        */
      virtual void
      setIndices (const IndicesPtr &indices);

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param[in] indices a pointer to the indices that represent the input data.
        */
      virtual void
      setIndices (const IndicesConstPtr &indices);

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param[in] indices a pointer to the indices that represent the input data.
        */
      virtual void
      setIndices (const PointIndicesConstPtr &indices);

      /** \brief Set the indices for the points laying within an interest region of 
        * the point cloud.
        * \note you shouldn't call this method on unorganized point clouds!
        * \param[in] row_start the offset on rows
        * \param[in] col_start the offset on columns
        * \param[in] nb_rows the number of rows to be considered row_start included
        * \param[in] nb_cols the number of columns to be considered col_start included
        */
      virtual void 
      setIndices (size_t row_start, size_t col_start, size_t nb_rows, size_t nb_cols);

      /** \brief Get a pointer to the vector of indices used. */
      inline IndicesPtr const 
      getIndices () { return (indices_); }

      /** \brief Get a pointer to the vector of indices used. */
      inline IndicesConstPtr const 
      getIndices () const { return (indices_); }

      /** \brief Override PointCloud operator[] to shorten code
        * \note this method can be called instead of (*input_)[(*indices_)[pos]]
        * or input_->points[(*indices_)[pos]]
        * \param[in] pos position in indices_ vector
        */
      inline const PointT& operator[] (size_t pos) const 
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
      bool
      initCompute ();

      /** \brief This method should get called after finishing the actual computation. 
        */
      bool
      deinitCompute ();

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class PCL_EXPORTS PCLBase<pcl::PCLPointCloud2>
  {
    public:
      typedef pcl::PCLPointCloud2 PCLPointCloud2;
      typedef boost::shared_ptr<PCLPointCloud2> PCLPointCloud2Ptr;
      typedef boost::shared_ptr<PCLPointCloud2 const> PCLPointCloud2ConstPtr;

      typedef boost::shared_ptr<PointIndices> PointIndicesPtr;
      typedef boost::shared_ptr<PointIndices const> PointIndicesConstPtr;

      /** \brief Empty constructor. */
      PCLBase ();
     
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
      setInputCloud (const PCLPointCloud2ConstPtr &cloud);

      /** \brief Get a pointer to the input point cloud dataset. */
      inline PCLPointCloud2ConstPtr const 
      getInputCloud () { return (input_); }

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param[in] indices a pointer to the indices that represent the input data.
        */
      void
      setIndices (const IndicesPtr &indices);

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param[in] indices a pointer to the indices that represent the input data.
        */
      void
      setIndices (const PointIndicesConstPtr &indices);

      /** \brief Get a pointer to the vector of indices used. */
      inline IndicesPtr const 
      getIndices () { return (indices_); }

    protected:
      /** \brief The input point cloud dataset. */
      PCLPointCloud2ConstPtr input_;

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

#ifdef PCL_NO_PRECOMPILE
#include <pcl/impl/pcl_base.hpp>
#endif

#endif  //#ifndef PCL_PCL_BASE_H_

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: voxel_grid.h 1374 2011-06-19 02:29:56Z bouffa $
 *
 */

#ifndef PCL_FILTERS_FAST_VOXEL_GRID_MAP_H_
#define PCL_FILTERS_FAST_VOXEL_GRID_MAP_H_

#include "pcl/filters/filter.h"
#include <boost/mpl/size.hpp>

namespace pcl
{
  struct he {
    int ix, iy, iz;
    int count;
    Eigen::VectorXf centroid;
  };

  /** \brief Helper functor structure for copying data between an Eigen::VectorXf and a PointT. */
  template <typename PointT>
  struct xNdCopyEigenPointFunctor
  {
    typedef typename traits::POD<PointT>::type Pod;
    
    xNdCopyEigenPointFunctor (const Eigen::VectorXf &p1, PointT &p2)
      : p1_ (p1),
        p2_ (reinterpret_cast<Pod&>(p2)),
        f_idx_ (0) { }

    template<typename Key> inline void operator() ()
    {
      //boost::fusion::at_key<Key> (p2_) = p1_[f_idx_++];
      typedef typename pcl::traits::datatype<PointT, Key>::type T;
      uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&p2_) + pcl::traits::offset<PointT, Key>::value;
      *reinterpret_cast<T*>(data_ptr) = p1_[f_idx_++];
    }

    private:
      const Eigen::VectorXf &p1_;
      Pod &p2_;
      int f_idx_;
  };

  /** \brief Helper functor structure for copying data between an Eigen::VectorXf and a PointT. */
  template <typename PointT>
  struct xNdCopyPointEigenFunctor
  {
    typedef typename traits::POD<PointT>::type Pod;
    
    xNdCopyPointEigenFunctor (const PointT &p1, Eigen::VectorXf &p2)
      : p1_ (reinterpret_cast<const Pod&>(p1)), p2_ (p2), f_idx_ (0) { }

    template<typename Key> inline void operator() ()
    {
      //p2_[f_idx_++] = boost::fusion::at_key<Key> (p1_);
      typedef typename pcl::traits::datatype<PointT, Key>::type T;
      const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&p1_) + pcl::traits::offset<PointT, Key>::value;
      p2_[f_idx_++] = *reinterpret_cast<const T*>(data_ptr);
    }

    private:
      const Pod &p1_;
      Eigen::VectorXf &p2_;
      int f_idx_;
  };

  /** \brief @b FastVoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
    *
    * The @b FastVoxelGrid class creates a *3D voxel grid* (think about a voxel
    * grid as a set of tiny 3D boxes in space) over the input point cloud data.
    * Then, in each *voxel* (i.e., 3D box), all the points present will be
    * approximated (i.e., *downsampled*) with their centroid. This approach is
    * a bit slower than approximating them with the center of the voxel, but it
    * represents the underlying surface more accurately.
    *
    * \author Radu Bogdan Rusu, Bastian Steder
    * \ingroup filters
    */
  template <typename PointT>
  class FastVoxelGrid: public Filter<PointT>
  {
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::filter_limit_negative_;
    using Filter<PointT>::filter_limit_min_;
    using Filter<PointT>::filter_limit_max_;
    using Filter<PointT>::filter_field_name_;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      FastVoxelGrid () : downsample_all_data_ (true), histsize(512)
      {
        setLeafSize(1, 1, 1);
        filter_name_ = "FastVoxelGrid";
        history = new he[histsize];
      }

      /** \brief Destructor. */
      virtual ~FastVoxelGrid ()
      {
      }

      /** \brief Set the voxel grid leaf size.
        * \param leaf_size the voxel grid leaf size
        */
      inline void 
      setLeafSize (const Eigen::Vector3f &leaf_size) 
      { 
        leaf_size_ = leaf_size; 
        inverse_leaf_size_ = Eigen::Array3f::Ones () / leaf_size_.array ();
      }

      /** \brief Set the voxel grid leaf size.
        * \param lx the leaf size for X
        * \param ly the leaf size for Y
        * \param lz the leaf size for Z
        */
      inline void
      setLeafSize (float lx, float ly, float lz)
      {
        setLeafSize(Eigen::Vector3f(lx, ly, lz));
      }

      /** \brief Get the voxel grid leaf size. */
      inline Eigen::Vector3f 
      getLeafSize () { return leaf_size_; }

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
        * \param downsample the new value (true/false)
        */
      inline void 
      setDownsampleAllData (bool downsample) { downsample_all_data_ = downsample; }

      /** \brief Get the state of the internal downsampling parameter (true if
        * all fields need to be downsampled, false if just XYZ). 
        */
      inline bool 
      getDownsampleAllData () { return (downsample_all_data_); }

    protected:
      /** \brief The size of a leaf. */
      Eigen::Vector3f leaf_size_;

      /** \brief Compute 1/leaf_size_ to avoid division later */ 
      Eigen::Array3f inverse_leaf_size_;

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
      bool downsample_all_data_;

      /** \brief history buffer size, power of 2 */
      size_t histsize;

      /** \brief history buffer */
      struct he *history;

      typedef typename pcl::traits::fieldList<PointT>::type FieldList;

      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param output the resultant point cloud message
        */
      void 
      applyFilter (PointCloud &output);

      /** \brief Write a single point from the hash to the output cloud
        */
      void flush(PointCloud &output, size_t op, he *hhe, int rgba_index, int centroid_size);
  };
}

#endif  //#ifndef PCL_FILTERS_VOXEL_GRID_MAP_H_

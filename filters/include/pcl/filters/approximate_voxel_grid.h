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
 * $Id: voxel_grid.h 1374 2011-06-19 02:29:56Z bouffa $
 *
 */

#pragma once

#include <pcl/filters/filter.h>

namespace pcl
{
  /** \brief Helper functor structure for copying data between an Eigen::VectorXf and a PointT. */
  template <typename PointT>
  struct xNdCopyEigenPointFunctor
  {
    using Pod = typename traits::POD<PointT>::type;
    
    xNdCopyEigenPointFunctor (const Eigen::VectorXf &p1, PointT &p2)
      : p1_ (p1),
        p2_ (reinterpret_cast<Pod&>(p2)),
        f_idx_ (0) { }

    template<typename Key> inline void operator() ()
    {
      //boost::fusion::at_key<Key> (p2_) = p1_[f_idx_++];
      using T = typename pcl::traits::datatype<PointT, Key>::type;
      std::uint8_t* data_ptr = reinterpret_cast<std::uint8_t*>(&p2_) + pcl::traits::offset<PointT, Key>::value;
      *reinterpret_cast<T*>(data_ptr) = static_cast<T> (p1_[f_idx_++]);
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
    using Pod = typename traits::POD<PointT>::type;
    
    xNdCopyPointEigenFunctor (const PointT &p1, Eigen::VectorXf &p2)
      : p1_ (reinterpret_cast<const Pod&>(p1)), p2_ (p2), f_idx_ (0) { }

    template<typename Key> inline void operator() ()
    {
      //p2_[f_idx_++] = boost::fusion::at_key<Key> (p1_);
      using T = typename pcl::traits::datatype<PointT, Key>::type;
      const std::uint8_t* data_ptr = reinterpret_cast<const std::uint8_t*>(&p1_) + pcl::traits::offset<PointT, Key>::value;
      p2_[f_idx_++] = static_cast<float> (*reinterpret_cast<const T*>(data_ptr));
    }

    private:
      const Pod &p1_;
      Eigen::VectorXf &p2_;
      int f_idx_;
  };

  /** \brief ApproximateVoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
    *
    * \author James Bowman, Radu B. Rusu
    * \ingroup filters
    */
  template <typename PointT>
  class ApproximateVoxelGrid: public Filter<PointT>
  {
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;

    using PointCloud = typename Filter<PointT>::PointCloud;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    private:
      struct he
      {
        he () : ix (), iy (), iz (), count (0) {}
        int ix, iy, iz;
        int count;
        Eigen::VectorXf centroid;
      };

    public:

      using Ptr = shared_ptr<ApproximateVoxelGrid<PointT> >;
      using ConstPtr = shared_ptr<const ApproximateVoxelGrid<PointT> >;


      /** \brief Empty constructor. */
      ApproximateVoxelGrid () : 
        pcl::Filter<PointT> (),
        leaf_size_ (Eigen::Vector3f::Ones ()),
        inverse_leaf_size_ (Eigen::Array3f::Ones ()),
        downsample_all_data_ (true), histsize_ (512),
        history_ (new he[histsize_])
      {
        filter_name_ = "ApproximateVoxelGrid";
      }

      /** \brief Copy constructor. 
        * \param[in] src the approximate voxel grid to copy into this. 
        */
      ApproximateVoxelGrid (const ApproximateVoxelGrid &src) : 
        pcl::Filter<PointT> (),
        leaf_size_ (src.leaf_size_),
        inverse_leaf_size_ (src.inverse_leaf_size_),
        downsample_all_data_ (src.downsample_all_data_), 
        histsize_ (src.histsize_),
        history_ ()
      {
        history_ = new he[histsize_];
        for (std::size_t i = 0; i < histsize_; i++)
          history_[i] = src.history_[i];
      }


      /** \brief Destructor.
        */
      ~ApproximateVoxelGrid ()
      {
        delete [] history_;
      }


      /** \brief Copy operator. 
        * \param[in] src the approximate voxel grid to copy into this. 
        */
      inline ApproximateVoxelGrid& 
      operator = (const ApproximateVoxelGrid &src)
      {
        leaf_size_ = src.leaf_size_;
        inverse_leaf_size_ = src.inverse_leaf_size_;
        downsample_all_data_ = src.downsample_all_data_;
        histsize_ = src.histsize_;
        history_ = new he[histsize_];
        for (std::size_t i = 0; i < histsize_; i++)
          history_[i] = src.history_[i];
        return (*this);
      }

      /** \brief Set the voxel grid leaf size.
        * \param[in] leaf_size the voxel grid leaf size
        */
      inline void 
      setLeafSize (const Eigen::Vector3f &leaf_size) 
      { 
        leaf_size_ = leaf_size; 
        inverse_leaf_size_ = Eigen::Array3f::Ones () / leaf_size_.array ();
      }

      /** \brief Set the voxel grid leaf size.
        * \param[in] lx the leaf size for X
        * \param[in] ly the leaf size for Y
        * \param[in] lz the leaf size for Z
        */
      inline void
      setLeafSize (float lx, float ly, float lz)
      {
        setLeafSize (Eigen::Vector3f (lx, ly, lz));
      }

      /** \brief Get the voxel grid leaf size. */
      inline Eigen::Vector3f 
      getLeafSize () const { return (leaf_size_); }

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ.
        * \param downsample the new value (true/false)
        */
      inline void 
      setDownsampleAllData (bool downsample) { downsample_all_data_ = downsample; }

      /** \brief Get the state of the internal downsampling parameter (true if
        * all fields need to be downsampled, false if just XYZ). 
        */
      inline bool 
      getDownsampleAllData () const { return (downsample_all_data_); }

    protected:
      /** \brief The size of a leaf. */
      Eigen::Vector3f leaf_size_;

      /** \brief Compute 1/leaf_size_ to avoid division later */ 
      Eigen::Array3f inverse_leaf_size_;

      /** \brief Set to true if all fields need to be downsampled, or false if just XYZ. */
      bool downsample_all_data_;

      /** \brief history buffer size, power of 2 */
      std::size_t histsize_;

      /** \brief history buffer */
      struct he* history_;

      using FieldList = typename pcl::traits::fieldList<PointT>::type;

      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param output the resultant point cloud message
        */
      void 
      applyFilter (PointCloud &output) override;

      /** \brief Write a single point from the hash to the output cloud
        */
      void 
      flush (PointCloud &output, std::size_t op, he *hhe, int rgba_index, int centroid_size);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/approximate_voxel_grid.hpp>
#endif

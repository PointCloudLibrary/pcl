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

#ifndef PCL_FILTERS_UNIFORM_SAMPLING_H_
#define PCL_FILTERS_UNIFORM_SAMPLING_H_

#include <pcl/filters/filter.h>
#include <boost/unordered_map.hpp>

namespace pcl
{
  /** \brief @b UniformSampling assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
    *
    * The @b UniformSampling class creates a *3D voxel grid* (think about a voxel
    * grid as a set of tiny 3D boxes in space) over the input point cloud data.
    * Then, in each *voxel* (i.e., 3D box), all the points present will be
    * approximated (i.e., *downsampled*) with their centroid. This approach is
    * a bit slower than approximating them with the center of the voxel, but it
    * represents the underlying surface more accurately.
    *
    * \author Radu Bogdan Rusu
    * \ingroup keypoints
    */
  template <typename PointT>
  class UniformSampling: public Filter<PointT>
  {
    typedef typename Filter<PointT>::PointCloud PointCloud;

    using Filter<PointT>::filter_name_;
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::getClassName;

    public:
      typedef boost::shared_ptr<UniformSampling<PointT> > Ptr;
      typedef boost::shared_ptr<const UniformSampling<PointT> > ConstPtr;

      /** \brief Empty constructor. */
      UniformSampling () :
        leaves_ (),
        leaf_size_ (Eigen::Vector4f::Zero ()),
        inverse_leaf_size_ (Eigen::Vector4f::Zero ()),
        min_b_ (Eigen::Vector4i::Zero ()),
        max_b_ (Eigen::Vector4i::Zero ()),
        div_b_ (Eigen::Vector4i::Zero ()),
        divb_mul_ (Eigen::Vector4i::Zero ()),
        search_radius_ (0)
      {
        filter_name_ = "UniformSampling";
      }

      /** \brief Destructor. */
      virtual ~UniformSampling ()
      {
        leaves_.clear();
      }

      /** \brief Set the 3D grid leaf size.
        * \param radius the 3D grid leaf size
        */
      virtual inline void 
      setRadiusSearch (double radius) 
      { 
        leaf_size_[0] = leaf_size_[1] = leaf_size_[2] = static_cast<float> (radius);
        // Avoid division errors
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
        search_radius_ = radius;
      }

    protected:
      /** \brief Simple structure to hold an nD centroid and the number of points in a leaf. */
      struct Leaf
      {
        Leaf () : idx (-1) { }
        int idx;
      };

      /** \brief The 3D grid leaves. */
      boost::unordered_map<size_t, Leaf> leaves_;

      /** \brief The size of a leaf. */
      Eigen::Vector4f leaf_size_;

      /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */ 
      Eigen::Array4f inverse_leaf_size_;

      /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
      Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param[out] output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/uniform_sampling.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_UNIFORM_SAMPLING_H_


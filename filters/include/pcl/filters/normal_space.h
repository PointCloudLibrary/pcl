/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2012-, Open Perception, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <pcl/filters/filter_indices.h>
#include <boost/dynamic_bitset.hpp> // for dynamic_bitset
#include <ctime>
#include <random> // std::mt19937

namespace pcl
{
  /** \brief @b NormalSpaceSampling samples the input point cloud in the space of normal directions computed at every point.
    * \ingroup filters
    */
  template<typename PointT, typename NormalT>
  class NormalSpaceSampling : public FilterIndices<PointT>
  {
    using FilterIndices<PointT>::filter_name_;
    using FilterIndices<PointT>::getClassName;
    using FilterIndices<PointT>::indices_;
    using FilterIndices<PointT>::input_;
    using FilterIndices<PointT>::keep_organized_;
    using FilterIndices<PointT>::extract_removed_indices_;
    using FilterIndices<PointT>::removed_indices_;
    using FilterIndices<PointT>::user_filter_value_;

    using PointCloud = typename FilterIndices<PointT>::PointCloud;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;
    using NormalsConstPtr = typename pcl::PointCloud<NormalT>::ConstPtr;

    public:
      
      using Ptr = shared_ptr<NormalSpaceSampling<PointT, NormalT> >;
      using ConstPtr = shared_ptr<const NormalSpaceSampling<PointT, NormalT> >;

      /** \brief Empty constructor. */
      NormalSpaceSampling () : NormalSpaceSampling (false) {}

      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed.
        */
      explicit NormalSpaceSampling (bool extract_removed_indices)
        : FilterIndices<PointT> (extract_removed_indices)
        , sample_ (std::numeric_limits<unsigned int>::max ())
        , seed_ (static_cast<unsigned int> (time (nullptr)))
        , binsx_ ()
        , binsy_ ()
        , binsz_ ()
        , input_normals_ ()
      {
        filter_name_ = "NormalSpaceSampling";
      }

      /** \brief Set number of indices to be sampled.
        * \param[in] sample the number of sample indices
        */
      inline void
      setSample (unsigned int sample)
      { sample_ = sample; }

      /** \brief Get the value of the internal \a sample parameter. */
      inline unsigned int
      getSample () const
      { return (sample_); }

      /** \brief Set seed of random function.
        * \param[in] seed the input seed
        */
      inline void
      setSeed (unsigned int seed)
      { seed_ = seed; }

      /** \brief Get the value of the internal \a seed parameter. */
      inline unsigned int
      getSeed () const
      { return (seed_); }

      /** \brief Set the number of bins in x, y and z direction
        * \param[in] binsx number of bins in x direction
        * \param[in] binsy number of bins in y direction
        * \param[in] binsz number of bins in z direction
        */
      inline void 
      setBins (unsigned int binsx, unsigned int binsy, unsigned int binsz)
      {
        binsx_ = binsx;
        binsy_ = binsy;
        binsz_ = binsz;
      }

      /** \brief Get the number of bins in x, y and z direction
        * \param[out] binsx number of bins in x direction
        * \param[out] binsy number of bins in y direction
        * \param[out] binsz number of bins in z direction
        */
      inline void 
      getBins (unsigned int& binsx, unsigned int& binsy, unsigned int& binsz) const
      {
        binsx = binsx_;
        binsy = binsy_;
        binsz = binsz_;
      }

      /** \brief Set the normals computed on the input point cloud
        * \param[in] normals the normals computed for the input cloud
        */
      inline void 
      setNormals (const NormalsConstPtr &normals) { input_normals_ = normals; }

      /** \brief Get the normals computed on the input point cloud */
      inline NormalsConstPtr
      getNormals () const { return (input_normals_); }

    protected:
      /** \brief Number of indices that will be returned. */
      unsigned int sample_;
      /** \brief Random number seed. */
      unsigned int seed_;

      /** \brief Number of bins in x direction. */
      unsigned int binsx_;
      /** \brief Number of bins in y direction. */
      unsigned int binsy_;
      /** \brief Number of bins in z direction. */
      unsigned int binsz_;
     
      /** \brief The normals computed at each point in the input cloud */
      NormalsConstPtr input_normals_;

      /** \brief Sample of point indices
        * \param[out] indices the resultant point cloud indices
        */
      void
      applyFilter (Indices &indices) override;

      bool
      initCompute ();

    private:
      /** \brief Finds the bin number of the input normal, returns the bin number
        * \param[in] normal the input normal 
        */
      unsigned int 
      findBin (const float *normal);

      /** \brief Checks of the entire bin is sampled, returns true or false
        * \param[out] array flag which says whether a point is sampled or not
        * \param[in] start_index the index to the first point of the bin in array.
        * \param[in] length number of points in the bin
        */
      bool
      isEntireBinSampled (boost::dynamic_bitset<> &array, unsigned int start_index, unsigned int length);

      /** \brief Random engine */
      std::mt19937 rng_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/normal_space.hpp>
#endif

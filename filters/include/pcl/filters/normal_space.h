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

#ifndef PCL_FILTERS_NORMAL_SUBSAMPLE_H_
#define PCL_FILTERS_NORMAL_SUBSAMPLE_H_

#include <pcl/filters/boost.h>
#include <pcl/filters/filter_indices.h>
#include <time.h>
#include <limits.h>

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

    typedef typename FilterIndices<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef typename pcl::PointCloud<NormalT>::ConstPtr NormalsConstPtr;

    public:
      
      typedef boost::shared_ptr<NormalSpaceSampling<PointT, NormalT> > Ptr;
      typedef boost::shared_ptr<const NormalSpaceSampling<PointT, NormalT> > ConstPtr;

      /** \brief Empty constructor. */
      NormalSpaceSampling ()
        : sample_ (std::numeric_limits<unsigned int>::max ())
        , seed_ (static_cast<unsigned int> (time (NULL)))
        , binsx_ ()
        , binsy_ ()
        , binsz_ ()
        , input_normals_ ()
        , rng_uniform_distribution_ (NULL)
      {
        filter_name_ = "NormalSpaceSampling";
      }

      /** \brief Destructor. */
      ~NormalSpaceSampling ()
      {
        if (rng_uniform_distribution_ != NULL)
          delete rng_uniform_distribution_;
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

      /** \brief Sample of point indices into a separate PointCloud
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Sample of point indices
        * \param[out] indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);

      bool
      initCompute ();

    private:
      /** \brief Finds the bin number of the input normal, returns the bin number
        * \param[in] normal the input normal 
        * \param[in] nbins total number of bins
        */
      unsigned int 
      findBin (const float *normal, unsigned int nbins);

      /** \brief Checks of the entire bin is sampled, returns true or false
        * \param[out] array flag which says whether a point is sampled or not
        * \param[in] start_index the index to the first point of the bin in array.
        * \param[in] length number of points in the bin
        */
      bool
      isEntireBinSampled (boost::dynamic_bitset<> &array, unsigned int start_index, unsigned int length);

      /** \brief Uniform random distribution. */
      boost::variate_generator<boost::mt19937, boost::uniform_int<uint32_t> > *rng_uniform_distribution_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/normal_space.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_NORMAL_SPACE_SUBSAMPLE_H_

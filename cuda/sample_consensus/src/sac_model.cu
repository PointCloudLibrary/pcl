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
 * $Id$
 *
 */

#ifdef _WIN32
# define WIN32_LEAN_AND_MEAN
#endif

#include <pcl/pcl_exports.h>

#include "pcl/cuda/sample_consensus/sac_model.h"
#include "pcl/cuda/time_gpu.h"

#include <thrust/replace.h>
#include <thrust/copy.h>
#include <thrust/sequence.h>
#include <thrust/gather.h>


namespace pcl
{
  namespace cuda 
  {

    template <template <typename> class Storage>
    __inline__ __host__
    void create_scatter_stencil (int new_w, int new_h, int skip, int width, int height, typename Storage<int>::type &stencil)
    { 
      for (unsigned int i = 0; i < new_w * new_h; ++i)
      { 
        int xIdx = i % new_w;
        int yIdx = i / new_w; 
        stencil [i] = (xIdx * skip) + width * (yIdx * skip);
      }
    }

    template <template <typename> class Storage> void
    SampleConsensusModel<Storage>::setInputCloud (const PointCloudConstPtr &cloud)
    {
      //input_.reset (new PointCloud);
      //int skip = 2;
      //int new_w = cloud->width/skip;
      //int new_h = cloud->height/skip;

      //input_->width    = new_w;
      //input_->height   = new_h;
      //input_->is_dense = cloud->is_dense;
      //
      //typename Storage<int>::type stencil (new_w * new_h);
      //create_scatter_stencil<Storage> (new_w, new_h, skip, cloud->width, cloud->height, stencil);
      //
      //input_->points.resize (stencil.size ());
      //thrust::gather (stencil.begin (), stencil.end (), cloud->points.begin (), input_->points.begin ());
      // TODO pcl_cuda::ScopeTime t ("setInputCloud"); 
      input_ = cloud;

      //// first, we create an inlier stencil that has -1 set for all nan points
      if (!indices_stencil_)
        indices_stencil_.reset (new Indices);

      indices_stencil_->resize (input_->points.size());
      thrust::sequence (indices_stencil_->begin (), indices_stencil_->end ());

      thrust::replace_if (indices_stencil_->begin (), indices_stencil_->end (),
          input_->points.begin (), isNaNPoint (),
          -1);

      // copy all non-(-1) values from indices_stencil_ to indices_
      if (!indices_)
        indices_.reset (new Indices);

      indices_->resize (input_->points.size());
      typename Indices::iterator it;
      it = thrust::copy_if (indices_stencil_->begin (), indices_stencil_->end (), indices_->begin (), isInlier ());
      indices_->erase (it, indices_->end ());

      std::cerr << "setInputCloud :" << indices_->size () << " valid points ( " << indices_->size () / (float) input_->points.size () << " %)" << std::endl;

    }


    // ------ got moved to inline function def in sac_model.h
    //template <template <typename> class Storage> typename SampleConsensusModel<Storage>::IndicesPtr 
    //SampleConsensusModel<Storage>::getIndices () const
    //{
    //  if (nr_indices_in_stencil_ != indices_->size())
    //  {
    //    typename Indices::iterator last = thrust::remove_copy (indices_stencil_->begin (), indices_stencil_->end (), indices_->begin (), -1);
    //    indices_->resize (last - indices_->begin ());
    //  }
    //
    //  return (indices_);
    //}

    template <template <typename> class Storage> int
    SampleConsensusModel<Storage>::deleteIndices (const IndicesPtr &indices_stencil )
    {
      //assert (indices_->size() ==  indices_stencil->size());
      //thrust::transform (thrust::make_zip_iterator (thrust::make_tuple (indices_stencil_->begin(), indices_stencil->begin ())),
      //                   thrust::make_zip_iterator (thrust::make_tuple (indices_stencil_->begin(), indices_stencil->begin ())) + indices_stencil_->size(),
      //                   indices_stencil_->begin (),
      //                   DeleteIndices ());
      //typename Indices::iterator last = thrust::remove_copy (indices_stencil_->begin (), indices_stencil_->end (), indices_->begin (), -1);
      //indices_->resize (last - indices_->begin());
      //return (int)indices_->size();

      assert (indices_stencil_->size() ==  indices_stencil->size());
      thrust::transform (thrust::make_zip_iterator (thrust::make_tuple (indices_stencil_->begin(), indices_stencil->begin ())),
                         thrust::make_zip_iterator (thrust::make_tuple (indices_stencil_->begin(), indices_stencil->begin ())) + indices_stencil_->size(),
                         indices_stencil_->begin (),
                         DeleteIndices ());
      int pts_deleted = (int) thrust::count (indices_stencil_->begin (), indices_stencil_->end (), -1);

      return (int) indices_stencil_->size ()- pts_deleted;
    }

    template <template <typename> class Storage> int
      SampleConsensusModel<Storage>::deleteIndices (const Hypotheses &h, int idx, IndicesPtr &inliers, const IndicesPtr &inliers_delete)
    {
      if (inliers->size() !=  inliers_delete->size())
        std::cerr << "assert (inliers->size() ==  inliers_delete->size()); ---> " << inliers->size () << " != " << inliers_delete->size () << std::endl;
      //assert (inliers->size() ==  inliers_delete->size());
      thrust::transform (thrust::make_zip_iterator (thrust::make_tuple (inliers->begin(), inliers_delete->begin ())),
                         thrust::make_zip_iterator (thrust::make_tuple (inliers->begin(), inliers_delete->begin ())) + inliers_delete->size(),
                         inliers->begin (),
                         DeleteIndices ());
      int i = (int) thrust::count (inliers->begin (), inliers->end (), -1);
      return (int)inliers->size() - i;
    }


    //////////////////////////////////////////////////////////////////////////
    // set all points which are inliers in the current model to -1,
    // points which are outliers (-1) to the current model are copied
    template <typename Tuple> int
    DeleteIndices::operator () (const Tuple &t)
    {
      if (thrust::get<1>(t) == -1)
        return thrust::get<0>(t);
      else
        return -1;
    }


    template class PCL_EXPORTS SampleConsensusModel<Device>;
    template class PCL_EXPORTS SampleConsensusModel<Host>;

  } // namespace
} // namespace


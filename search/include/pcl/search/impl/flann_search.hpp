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
 * $Id$
 *
 */

#ifndef PCL_SEARCH_IMPL_FLANN_SEARCH_H_
#define PCL_SEARCH_IMPL_FLANN_SEARCH_H_

#include <pcl/search/flann_search.h>
#include <pcl/kdtree/flann.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance>
typename pcl::search::FlannSearch<PointT, FlannDistance>::IndexPtr
pcl::search::FlannSearch<PointT, FlannDistance>::KdTreeIndexCreator::createIndex (MatrixConstPtr data)
{
  return (IndexPtr (new flann::KDTreeSingleIndex<FlannDistance> (*data,flann::KDTreeSingleIndexParams (max_leaf_size_))));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance>
typename pcl::search::FlannSearch<PointT, FlannDistance>::IndexPtr
pcl::search::FlannSearch<PointT, FlannDistance>::KMeansIndexCreator::createIndex (MatrixConstPtr data)
{
  return (IndexPtr (new flann::KMeansIndex<FlannDistance> (*data,flann::KMeansIndexParams ())));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance>
typename pcl::search::FlannSearch<PointT, FlannDistance>::IndexPtr
pcl::search::FlannSearch<PointT, FlannDistance>::KdTreeMultiIndexCreator::createIndex (MatrixConstPtr data)
{
  return (IndexPtr (new flann::KDTreeIndex<FlannDistance> (*data, flann::KDTreeIndexParams (trees_))));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance>
pcl::search::FlannSearch<PointT, FlannDistance>::FlannSearch(bool sorted, FlannIndexCreatorPtr creator) : pcl::search::Search<PointT> ("FlannSearch",sorted),
  index_(), creator_ (creator), input_flann_(), eps_ (0), checks_ (32), input_copied_for_flann_ (false), point_representation_ (new DefaultPointRepresentation<PointT>),
  dim_ (0), index_mapping_(), identity_mapping_()
{
  dim_ = point_representation_->getNumberOfDimensions ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance>
pcl::search::FlannSearch<PointT, FlannDistance>::~FlannSearch()
{
  if (input_copied_for_flann_)
    delete [] input_flann_->ptr();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance> void
pcl::search::FlannSearch<PointT, FlannDistance>::setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices)
{
  input_ = cloud;
  indices_ = indices;
  convertInputToFlannMatrix ();
  index_ = creator_->createIndex (input_flann_);
  index_->buildIndex ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance> int
pcl::search::FlannSearch<PointT, FlannDistance>::nearestKSearch (const PointT &point, int k, std::vector<int> &indices, std::vector<float> &dists) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!"); // remove this check as soon as FLANN does NaN checks internally
  bool can_cast = point_representation_->isTrivial ();

  float* data = 0;
  if (!can_cast)
  {
    data = new float [point_representation_->getNumberOfDimensions ()];
    point_representation_->vectorize (point,data);
  }

  float* cdata = can_cast ? const_cast<float*> (reinterpret_cast<const float*> (&point)): data;
  const flann::Matrix<float> m (cdata ,1, point_representation_->getNumberOfDimensions ());

  flann::SearchParams p;
  p.eps = eps_;
  p.sorted = sorted_results_;
  p.checks = checks_;
  if (indices.size() != static_cast<unsigned int> (k))
    indices.resize (k,-1);
  if (dists.size() != static_cast<unsigned int> (k))
    dists.resize (k);
  flann::Matrix<int> i (&indices[0],1,k);
  flann::Matrix<float> d (&dists[0],1,k);
  int result = index_->knnSearch (m,i,d,k, p);

  delete [] data;

  if (!identity_mapping_)
  {
    for (size_t i = 0; i < static_cast<unsigned int> (k); ++i)
    {
      int& neighbor_index = indices[i];
      neighbor_index = index_mapping_[neighbor_index];
    }
  }
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance> void
pcl::search::FlannSearch<PointT, FlannDistance>::nearestKSearch (
    const PointCloud& cloud, const std::vector<int>& indices, int k, std::vector< std::vector<int> >& k_indices,
    std::vector< std::vector<float> >& k_sqr_distances) const
{
  if (indices.empty ())
  {
    k_indices.resize (cloud.size ());
    k_sqr_distances.resize (cloud.size ());

    if (! cloud.is_dense) // remove this check as soon as FLANN does NaN checks internally
    {
      for (size_t i = 0; i < cloud.size(); i++)
      {
        assert (point_representation_->isValid (cloud[i]) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
      }
    }

    bool can_cast = point_representation_->isTrivial ();

    // full point cloud + trivial copy operation = no need to do any conversion/copying to the flann matrix!
    float* data=0;
    if (!can_cast)
    {
      data = new float[dim_*cloud.size ()];
      for (size_t i = 0; i < cloud.size (); ++i)
      {
        float* out = data+i*dim_;
        point_representation_->vectorize (cloud[i],out);
      }
    }

    // const cast is evil, but the matrix constructor won't change the data, and the
    // search won't change the matrix
    float* cdata = can_cast ? const_cast<float*> (reinterpret_cast<const float*> (&cloud[0])): data;
    const flann::Matrix<float> m (cdata ,cloud.size (), dim_, can_cast ? sizeof (PointT) : dim_ * sizeof (float) );

    flann::SearchParams p;
    p.sorted = sorted_results_;
    p.eps = eps_;
    p.checks = checks_;
    index_->knnSearch (m,k_indices,k_sqr_distances,k, p);

    delete [] data;
  }
  else // if indices are present, the cloud has to be copied anyway. Only copy the relevant parts of the points here.
  {
    k_indices.resize (indices.size ());
    k_sqr_distances.resize (indices.size ());

    if (! cloud.is_dense) // remove this check as soon as FLANN does NaN checks internally
    {
      for (size_t i = 0; i < indices.size(); i++)
      {
        assert (point_representation_->isValid (cloud [indices[i]]) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
      }
    }

    float* data=new float [dim_*indices.size ()];
    for (size_t i = 0; i < indices.size (); ++i)
    {
      float* out = data+i*dim_;
      point_representation_->vectorize (cloud[indices[i]],out);
    }
    const flann::Matrix<float> m (data ,indices.size (), point_representation_->getNumberOfDimensions ());

    flann::SearchParams p;
    p.sorted = sorted_results_;
    p.eps = eps_;
    p.checks = checks_;
    index_->knnSearch (m,k_indices,k_sqr_distances,k, p);

    delete[] data;
  }
  if (!identity_mapping_)
  {
    for (size_t j = 0; j < k_indices.size (); ++j)
    {
      for (size_t i = 0; i < static_cast<unsigned int> (k); ++i)
      {
        int& neighbor_index = k_indices[j][i];
        neighbor_index = index_mapping_[neighbor_index];
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance> int
pcl::search::FlannSearch<PointT, FlannDistance>::radiusSearch (const PointT& point, double radius,
    std::vector<int> &indices, std::vector<float> &distances,
    unsigned int max_nn) const
{
  assert (point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!"); // remove this check as soon as FLANN does NaN checks internally
  bool can_cast = point_representation_->isTrivial ();

  float* data = 0;
  if (!can_cast)
  {
    data = new float [point_representation_->getNumberOfDimensions ()];
    point_representation_->vectorize (point,data);
  }

  float* cdata = can_cast ? const_cast<float*> (reinterpret_cast<const float*> (&point)) : data;
  const flann::Matrix<float> m (cdata ,1, point_representation_->getNumberOfDimensions ());

  flann::SearchParams p;
  p.sorted = sorted_results_;
  p.eps = eps_;
  p.max_neighbors = max_nn > 0 ? max_nn : -1;
  p.checks = checks_;
  std::vector<std::vector<int> > i (1);
  std::vector<std::vector<float> > d (1);
  int result = index_->radiusSearch (m,i,d,static_cast<float> (radius * radius), p);

  delete [] data;
  indices = i [0];
  distances = d [0];

  if (!identity_mapping_)
  {
    for (size_t i = 0; i < indices.size (); ++i)
    {
      int& neighbor_index = indices [i];
      neighbor_index = index_mapping_ [neighbor_index];
    }
  }
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance> void
pcl::search::FlannSearch<PointT, FlannDistance>::radiusSearch (
    const PointCloud& cloud, const std::vector<int>& indices, double radius, std::vector< std::vector<int> >& k_indices,
    std::vector< std::vector<float> >& k_sqr_distances, unsigned int max_nn) const
{
  if (indices.empty ()) // full point cloud + trivial copy operation = no need to do any conversion/copying to the flann matrix!
  {
    k_indices.resize (cloud.size ());
    k_sqr_distances.resize (cloud.size ());

    if (! cloud.is_dense) // remove this check as soon as FLANN does NaN checks internally
    {
      for (size_t i = 0; i < cloud.size(); i++)
      {
        assert (point_representation_->isValid (cloud[i]) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");
      }
    }

    bool can_cast = point_representation_->isTrivial ();

    float* data = 0;
    if (!can_cast)
    {
      data = new float[dim_*cloud.size ()];
      for (size_t i = 0; i < cloud.size (); ++i)
      {
        float* out = data+i*dim_;
        point_representation_->vectorize (cloud[i],out);
      }
    }

    float* cdata = can_cast ? const_cast<float*> (reinterpret_cast<const float*> (&cloud[0])) : data;
    const flann::Matrix<float> m (cdata ,cloud.size (), dim_, can_cast ? sizeof (PointT) : dim_ * sizeof (float));

    flann::SearchParams p;
    p.sorted = sorted_results_;
    p.eps = eps_;
    p.checks = checks_;
    // here: max_nn==0: take all neighbors. flann: max_nn==0: return no neighbors, only count them. max_nn==-1: return all neighbors
    p.max_neighbors = max_nn != 0 ? max_nn : -1;
    index_->radiusSearch (m,k_indices,k_sqr_distances,static_cast<float> (radius * radius), p);

    delete [] data;
  }
  else // if indices are present, the cloud has to be copied anyway. Only copy the relevant parts of the points here.
  {
    k_indices.resize (indices.size ());
    k_sqr_distances.resize (indices.size ());

    if (! cloud.is_dense)  // remove this check as soon as FLANN does NaN checks internally
    {
      for (size_t i = 0; i < indices.size(); i++)
      {
        assert (point_representation_->isValid (cloud [indices[i]]) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");
      }
    }

    float* data = new float [dim_ * indices.size ()];
    for (size_t i = 0; i < indices.size (); ++i)
    {
      float* out = data+i*dim_;
      point_representation_->vectorize (cloud[indices[i]], out);
    }
    const flann::Matrix<float> m (data, cloud.size (), point_representation_->getNumberOfDimensions ());

    flann::SearchParams p;
    p.sorted = sorted_results_;
    p.eps = eps_;
    p.checks = checks_;
    // here: max_nn==0: take all neighbors. flann: max_nn==0: return no neighbors, only count them. max_nn==-1: return all neighbors
    p.max_neighbors = max_nn != 0 ? max_nn : -1;
    index_->radiusSearch (m, k_indices, k_sqr_distances, static_cast<float> (radius * radius), p);

    delete[] data;
  }
  if (!identity_mapping_)
  {
    for (size_t j = 0; j < k_indices.size (); ++j )
    {
      for (size_t i = 0; i < k_indices[j].size (); ++i)
      {
        int& neighbor_index = k_indices[j][i];
        neighbor_index = index_mapping_[neighbor_index];
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename FlannDistance> void
pcl::search::FlannSearch<PointT, FlannDistance>::convertInputToFlannMatrix ()
{
  size_t original_no_of_points = indices_ && !indices_->empty () ? indices_->size () : input_->size ();

  if (input_copied_for_flann_)
    delete input_flann_->ptr();
  input_copied_for_flann_ = true;
  index_mapping_.clear();
  identity_mapping_ = true;

  //cloud_ = (float*)malloc (original_no_of_points * dim_ * sizeof (float));
  //index_mapping_.reserve(original_no_of_points);
  //identity_mapping_ = true;

  if (!indices_ || indices_->empty ())
  {
    // best case: all points can be passed to flann without any conversions
    if (input_->is_dense && point_representation_->isTrivial ())
    {
      // const cast is evil, but flann won't change the data
      input_flann_ = MatrixPtr (new flann::Matrix<float> (const_cast<float*>(reinterpret_cast<const float*>(&(*input_) [0])), original_no_of_points, point_representation_->getNumberOfDimensions (),sizeof (PointT)));
      input_copied_for_flann_ = false;
    }
    else
    {
      input_flann_ = MatrixPtr (new flann::Matrix<float> (new float[original_no_of_points*point_representation_->getNumberOfDimensions ()], original_no_of_points, point_representation_->getNumberOfDimensions ()));
      float* cloud_ptr = input_flann_->ptr();
      for (size_t i = 0; i < original_no_of_points; ++i)
      {
        const PointT& point = (*input_)[i];
        // Check if the point is invalid
        if (!point_representation_->isValid (point))
        {
          identity_mapping_ = false;
          continue;
        }

        index_mapping_.push_back (static_cast<int> (i));  // If the returned index should be for the indices vector

        point_representation_->vectorize (point, cloud_ptr);
        cloud_ptr += dim_;
      }
    }

  }
  else
  {
    input_flann_ = MatrixPtr (new flann::Matrix<float> (new float[original_no_of_points*point_representation_->getNumberOfDimensions ()], original_no_of_points, point_representation_->getNumberOfDimensions ()));
    float* cloud_ptr = input_flann_->ptr();
    for (size_t indices_index = 0; indices_index < original_no_of_points; ++indices_index)
    {
      int cloud_index = (*indices_)[indices_index];
      const PointT&  point = (*input_)[cloud_index];
      // Check if the point is invalid
      if (!point_representation_->isValid (point))
      {
        identity_mapping_ = false;
        continue;
      }

      index_mapping_.push_back (static_cast<int> (indices_index));  // If the returned index should be for the indices vector

      point_representation_->vectorize (point, cloud_ptr);
      cloud_ptr += dim_;
    }
  }
  if (input_copied_for_flann_)
    input_flann_->rows = index_mapping_.size ();
}

#define PCL_INSTANTIATE_FlannSearch(T) template class PCL_EXPORTS pcl::search::FlannSearch<T>;

#endif

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#ifndef PCL_SEGMENTATION_IMPL_GRABCUT_HPP
#define PCL_SEGMENTATION_IMPL_GRABCUT_HPP

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>

namespace pcl
{
  template <>
  float squaredEuclideanDistance (const pcl::segmentation::grabcut::Color &c1,
                                  const pcl::segmentation::grabcut::Color &c2)
  {
    return ((c1.r-c2.r)*(c1.r-c2.r)+(c1.g-c2.g)*(c1.g-c2.g)+(c1.b-c2.b)*(c1.b-c2.b));
  }
}

template <typename PointT>
pcl::segmentation::grabcut::Color::Color (const PointT& p)
{
  r = static_cast<float> (p.r) / 255.0;
  g = static_cast<float> (p.g) / 255.0;
  b = static_cast<float> (p.b) / 255.0;
}

template <typename PointT>
pcl::segmentation::grabcut::Color::operator PointT () const
{
  PointT p;
  p.r = static_cast<uint32_t> (r * 255);
  p.g = static_cast<uint32_t> (g * 255);
  p.b = static_cast<uint32_t> (b * 255);
  return (p);
}

template <typename PointT> void
pcl::GrabCut<PointT>::setInputCloud (const PointCloudConstPtr &cloud)
{
  input_ = cloud;
}

template <typename PointT> bool
pcl::GrabCut<PointT>::initCompute ()
{
  using namespace pcl::segmentation::grabcut;
  if (!pcl::PCLBase<PointT>::initCompute ())
  {
    PCL_ERROR ("[pcl::GrabCut::initCompute ()] Init failed!");
    return (false);
  }

  std::vector<pcl::PCLPointField> in_fields_;
  if ((pcl::getFieldIndex<PointT> (*input_, "rgb", in_fields_) == -1) &&
      (pcl::getFieldIndex<PointT> (*input_, "rgba", in_fields_) == -1))
  {
    PCL_ERROR ("[pcl::GrabCut::initCompute ()] No RGB data available, aborting!");
    return (false);
  }

  // Initialize the working image
  image_.reset (new Image (input_->width, input_->height));
  for (std::size_t i = 0; i < input_->size (); ++i)
  {
    (*image_) [i] = Color (input_->points[i]);
  }
  width_ = image_->width;
  height_ = image_->height;

  // Initialize the spatial locator
  if (!tree_ && !input_->isOrganized ())
  {
    tree_.reset (new pcl::search::KdTree<PointT> (true));
    tree_->setInputCloud (input_);
  }

  const std::size_t indices_size = indices_->size ();
  trimap_ = std::vector<segmentation::grabcut::TrimapValue> (indices_size, TrimapUnknown);
  hard_segmentation_ = std::vector<segmentation::grabcut::SegmentationValue> (indices_size,
                                                                              SegmentationBackground);
  GMM_component_.resize (indices_size);
  n_links_.resize (indices_size);

  //  soft_segmentation_ = 0;		// Not yet implemented
  foreground_GMM_.resize (K_);
  background_GMM_.resize (K_);

  //set some constants
  computeL ();

  if (image_->isOrganized ())
  {
    computeBetaOrganized ();
    computeNLinksOrganized ();
  }
  else
  {
    computeBetaNonOrganized ();
    computeNLinksNonOrganized ();
  }

  initialized_ = false;
  return (true);
}

template <typename PointT> void
pcl::GrabCut<PointT>::addEdge (vertex_descriptor v1, vertex_descriptor v2, float capacity, float rev_capacity)
{
  graph_.addEdge (v1, v2, capacity, rev_capacity);
}

template <typename PointT> void
pcl::GrabCut<PointT>::setTerminalWeights (vertex_descriptor v, float source_capacity, float sink_capacity)
{
  graph_.addSourceEdge (v, source_capacity);
  graph_.addTargetEdge (v, sink_capacity);
}

template <typename PointT> void
pcl::GrabCut<PointT>::setBackgroundPointsIndices (const PointIndicesConstPtr &indices)
{
  using namespace pcl::segmentation::grabcut;
  if (!initCompute ())
    return;

  std::fill (trimap_.begin (), trimap_.end (), TrimapBackground);
  std::fill (hard_segmentation_.begin (), hard_segmentation_.end (), SegmentationBackground);
  for (std::vector<int>::const_iterator idx = indices->indices.begin (); idx != indices->indices.end (); ++idx)
  {
    trimap_[*idx] = TrimapUnknown;
    hard_segmentation_[*idx] = SegmentationForeground;
  }

  if (!initialized_)
  {
    fitGMMs ();
    initialized_ = true;
  }
}

template <typename PointT> void
pcl::GrabCut<PointT>::fitGMMs ()
{
  // Step 3: Build GMMs using Orchard-Bouman clustering algorithm
  buildGMMs (*image_, *indices_, hard_segmentation_, GMM_component_, background_GMM_, foreground_GMM_);

  // Initialize the graph for graphcut (do this here so that the T-Link debugging image will be initialized)
  initGraph ();
}

template <typename PointT> int
pcl::GrabCut<PointT>::refineOnce ()
{
  // Steps 4 and 5: Learn new GMMs from current segmentation
  learnGMMs (*image_, *indices_, hard_segmentation_, GMM_component_, background_GMM_, foreground_GMM_);

  // Step 6: Run GraphCut and update segmentation
  initGraph ();

  float flow = graph_.solve ();

  int changed = updateHardSegmentation ();
  PCL_INFO ("%d pixels changed segmentation (max flow = %f)\n", changed, flow);

  return (changed);
}

template <typename PointT> void
pcl::GrabCut<PointT>::refine ()
{
  std::size_t changed = indices_->size ();

  while (changed)
    changed = refineOnce ();
}

template <typename PointT> int
pcl::GrabCut<PointT>::updateHardSegmentation ()
{
  using namespace pcl::segmentation::grabcut;

  int changed = 0;

  const int number_of_indices = static_cast<int> (indices_->size ());
  for (int i_point = 0; i_point < number_of_indices; ++i_point)
  {
    SegmentationValue old_value = hard_segmentation_ [i_point];

    if (trimap_ [i_point] == TrimapBackground)
      hard_segmentation_ [i_point] = SegmentationBackground;
    else
      if (trimap_ [i_point] == TrimapForeground)
        hard_segmentation_ [i_point] = SegmentationForeground;
      else	// TrimapUnknown
      {
        if (isSource (graph_nodes_[i_point]))
          hard_segmentation_ [i_point] = SegmentationForeground;
        else
          hard_segmentation_ [i_point] = SegmentationBackground;
      }

    if (old_value != hard_segmentation_ [i_point])
      ++changed;
  }
  return (changed);
}

template <typename PointT> void
pcl::GrabCut<PointT>::setTrimap (const PointIndicesConstPtr &indices, segmentation::grabcut::TrimapValue t)
{
  using namespace pcl::segmentation::grabcut;
  std::vector<int>::const_iterator idx = indices->indices.begin ();
  for (; idx != indices->indices.end (); ++idx)
    trimap_[*idx] = t;

  // Immediately set the hard segmentation as well so that the display will update.
  if (t == TrimapForeground)
    for (idx = indices->indices.begin (); idx != indices->indices.end (); ++idx)
      hard_segmentation_[*idx] = SegmentationForeground;
  else
    if (t == TrimapBackground)
      for (idx = indices->indices.begin (); idx != indices->indices.end (); ++idx)
        hard_segmentation_[*idx] = SegmentationBackground;
}

template <typename PointT> void
pcl::GrabCut<PointT>::initGraph ()
{
  using namespace pcl::segmentation::grabcut;
  const int number_of_indices = static_cast<int> (indices_->size ());
  // Set up the graph (it can only be used once, so we have to recreate it each time the graph is updated)
  graph_.clear ();
  graph_nodes_.clear ();
  graph_nodes_.resize (indices_->size ());
  int start = graph_.addNodes (indices_->size ());
  for (int idx = 0; idx < indices_->size (); ++idx)
  {
    graph_nodes_[idx] = start;
    ++start;
  }

  // Set T-Link weights
  for (int i_point = 0; i_point < number_of_indices; ++i_point)
  {
    int point_index = (*indices_) [i_point];
    float back, fore;

    switch (trimap_[point_index])
    {
      case TrimapUnknown :
      {
        fore = static_cast<float> (-log (background_GMM_.probabilityDensity (image_->points[point_index])));
        back = static_cast<float> (-log (foreground_GMM_.probabilityDensity (image_->points[point_index])));
        break;
      }
      case TrimapBackground :
      {
        fore = 0;
        back = L_;
        break;
      }
      default :
      {
        fore = L_;
        back = 0;
      }
    }

    setTerminalWeights (graph_nodes_[i_point], fore, back);
  }

  // Set N-Link weights from precomputed values
  for (int i_point = 0; i_point < number_of_indices; ++i_point)
  {
    const NLinks &n_link = n_links_[i_point];
    if (n_link.nb_links > 0)
    {
      int point_index = (*indices_) [i_point];
      std::vector<int>::const_iterator indices_it    = n_link.indices.begin ();
      std::vector<float>::const_iterator weights_it  = n_link.weights.begin ();
      for (; indices_it != n_link.indices.end (); ++indices_it, ++weights_it)
      {
        if ((*indices_it != point_index) && (*indices_it > -1))
        {
          addEdge (graph_nodes_[i_point], graph_nodes_[*indices_it], *weights_it, *weights_it);
        }
      }
    }
  }
}

template <typename PointT> void
pcl::GrabCut<PointT>::computeNLinksNonOrganized ()
{
  const int number_of_indices = static_cast<int> (indices_->size ());
  for (int i_point = 0; i_point < number_of_indices; ++i_point)
  {
    NLinks &n_link = n_links_[i_point];
    if (n_link.nb_links > 0)
    {
      int point_index = (*indices_) [i_point];
      std::vector<int>::const_iterator indices_it = n_link.indices.begin ();
      std::vector<float>::const_iterator dists_it = n_link.dists.begin   ();
      std::vector<float>::iterator weights_it     = n_link.weights.begin ();
      for (; indices_it != n_link.indices.end (); ++indices_it, ++dists_it, ++weights_it)
      {
        if (*indices_it != point_index)
        {
          // We saved the color distance previously at the computeBeta stage for optimization purpose
          float color_distance = *weights_it;
          // Set the real weight
          *weights_it = static_cast<float> (lambda_ * exp (-beta_ * color_distance) / sqrt (*dists_it));
        }
      }
    }
  }
}

template <typename PointT> void
pcl::GrabCut<PointT>::computeNLinksOrganized ()
{
	for( unsigned int y = 0; y < image_->height; ++y )
	{
    for( unsigned int x = 0; x < image_->width; ++x )
    {
      // We saved the color and euclidean distance previously at the computeBeta stage for
      // optimization purpose but here we compute the real weight
      std::size_t point_index = y * input_->width + x;
      NLinks &links = n_links_[point_index];

      if( x > 0 && y < image_->height-1 )
        links.weights[0] = lambda_ * exp (-beta_ * links.weights[0]) / links.dists[0];

      if( y < image_->height-1 )
        links.weights[1] = lambda_ * exp (-beta_ * links.weights[1]) / links.dists[1];

      if( x < image_->width-1 && y < image_->height-1 )
        links.weights[2] = lambda_ * exp (-beta_ * links.weights[2]) / links.dists[2];

      if( x < image_->width-1 )
        links.weights[3] = lambda_ * exp (-beta_ * links.weights[3]) / links.dists[3];
    }
	}
}

template <typename PointT> void
pcl::GrabCut<PointT>::computeBetaNonOrganized ()
{
  float result = 0;
  std::size_t edges = 0;

  const int number_of_indices = static_cast<int> (indices_->size ());

  for (int i_point = 0; i_point < number_of_indices; i_point++)
  {
    int point_index = (*indices_)[i_point];
    const PointT& point = input_->points [point_index];
    if (pcl::isFinite (point))
    {
      NLinks &links = n_links_[i_point];
      int found = tree_->nearestKSearch (point, nb_neighbours_, links.indices, links.dists);
      if (found > 1)
      {
        links.nb_links = found - 1;
        links.weights.reserve (links.nb_links);
        for (std::vector<int>::const_iterator nn_it = links.indices.begin (); nn_it != links.indices.end (); ++nn_it)
        {
          if (*nn_it != point_index)
          {
            float color_distance = squaredEuclideanDistance (image_->points[point_index], image_->points[*nn_it]);
            links.weights.push_back (color_distance);
            result+= color_distance;
            ++edges;
          }
          else
            links.weights.push_back (0.f);
        }
      }
    }
  }

  beta_ = 1e5 / (2*result / edges);
}

template <typename PointT> void
pcl::GrabCut<PointT>::computeBetaOrganized ()
{
  float result = 0;
  std::size_t edges = 0;

  for (unsigned int y = 0; y < input_->height; ++y)
  {
    for (unsigned int x = 0; x < input_->width; ++x)
    {
      std::size_t point_index = y * input_->width + x;
      NLinks &links = n_links_[point_index];
      links.nb_links = 4;
      links.weights.resize (links.nb_links, 0);
      links.dists.resize (links.nb_links, 0);
      links.indices.resize (links.nb_links, -1);

      if (x > 0 && y < input_->height-1)
      {
        std::size_t upleft = (y+1)  * input_->width + x - 1;
        links.indices[0] = upleft;
        links.dists[0] = sqrt (2.f);
        float color_dist =  squaredEuclideanDistance (image_->points[point_index],
                                                      image_->points[upleft]);
        links.weights[0] = color_dist;
        result+= color_dist;
        edges++;
      }

      if (y < input_->height-1)
      {
        std::size_t up = (y+1) * input_->width + x;
        links.indices[1] = up;
        links.dists[1] = 1;
        float color_dist =  squaredEuclideanDistance (image_->points[point_index],
                                                      image_->points[up]);
        links.weights[1] = color_dist;
        result+= color_dist;
        edges++;
      }

      if (x < input_->width-1 && y < input_->height-1)
      {
        std::size_t upright = (y+1) * input_->width + x + 1;
        links.indices[2] = upright;
        links.dists[2] = sqrt (2.f);
        float color_dist =  squaredEuclideanDistance (image_->points[point_index],
                                                      image_->points [upright]);
        links.weights[2] = color_dist;
        result+= color_dist;
        edges++;
      }

      if (x < input_->width-1)
      {
        std::size_t right = y * input_->width + x + 1;
        links.indices[3] = right;
        links.dists[3] = 1;
        float color_dist =  squaredEuclideanDistance (image_->points[point_index],
                                                      image_->points[right]);
        links.weights[3] = color_dist;
        result+= color_dist;
        edges++;
      }
    }
  }

  beta_ = 1e5 / (2*result / edges);
}

template <typename PointT> void
pcl::GrabCut<PointT>::computeL ()
{
  L_ = 8*lambda_ + 1;
}

template <typename PointT> void
pcl::GrabCut<PointT>::extract (std::vector<pcl::PointIndices>& clusters)
{
  using namespace pcl::segmentation::grabcut;
  clusters.clear ();
  clusters.resize (2);
  clusters[0].indices.reserve (indices_->size ());
  clusters[1].indices.reserve (indices_->size ());
  refine ();
  assert (hard_segmentation_.size () == indices_->size ());
  const int indices_size = static_cast<int> (indices_->size ());
  for (int i = 0; i < indices_size; ++i)
    if (hard_segmentation_[i] == SegmentationForeground)
      clusters[1].indices.push_back (i);
    else
      clusters[0].indices.push_back (i);
}

#endif

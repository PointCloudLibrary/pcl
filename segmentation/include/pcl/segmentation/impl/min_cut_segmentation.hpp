/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * $Id:$
 *
 */

#ifndef PCL_SEGMENTATION_MIN_CUT_SEGMENTATION_HPP_
#define PCL_SEGMENTATION_MIN_CUT_SEGMENTATION_HPP_

#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <stdlib.h>
#include <cmath>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::MinCutSegmentation<PointT>::MinCutSegmentation () :
  input_cloud_ (),
  search_ (),
  number_of_neighbours_ (14),
  number_of_points_ (0),
  graph_is_valid_ (false),
  unary_potentials_are_valid_ (false),
  binary_potentials_are_valid_ (false),
  graph_ (),
  source_ (),/////////////////////////////////
  sink_ (),///////////////////////////////////
  vertices_ (0),
  edge_marker_ (0),
  capacity_ (),
  reverse_edges_ (),
  inverse_sigma_ (16.0),
  radius_ (16.0),
  source_weight_ (0.8),
  max_flow_ (0.0),
  epsilon_ (0.0001),
  labels_ (0),
  foreground_points_ (0),
  background_points_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::MinCutSegmentation<PointT>::~MinCutSegmentation ()
{
  if (input_cloud_ != 0)
    input_cloud_.reset ();
  if (search_ != 0)
    search_.reset ();
  if (graph_ != 0)
    graph_.reset ();
  if (capacity_ != 0)
    capacity_.reset ();
  if (reverse_edges_ != 0)
    reverse_edges_.reset ();

  vertices_.clear ();
  edge_marker_.clear ();
  labels_.clear ();
  foreground_points_.clear ();
  background_points_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setInputCloud (typename pcl::PointCloud<PointT>::Ptr input_cloud)
{
  if (input_cloud_ != 0)
    input_cloud_.reset ();

  input_cloud_ = input_cloud;
  number_of_points_ = static_cast<int> (input_cloud->points.size ());
  graph_is_valid_ = false;
  unary_potentials_are_valid_ = false;
  binary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setSigma (double sigma)
{
  if (sigma > epsilon_)
  {
    inverse_sigma_ = 1.0 / (sigma * sigma);
    binary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setRadius (double radius)
{
  if (radius > epsilon_)
  {
    radius_ = radius * radius;
    unary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setSourceWeight (double weight)
{
  if (weight > epsilon_)
  {
    source_weight_ = weight;
    unary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setSearchMethod (typename pcl::search::Search<PointT>::Ptr search)
{
  if (search_ != 0)
    search_.reset ();

  search_ = search;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setNeighbourNumber (unsigned int number_of_neighbours)
{
  if (number_of_neighbours_ != number_of_neighbours && number_of_neighbours != 0)
  {
    number_of_neighbours_ = number_of_neighbours;
    graph_is_valid_ = false;
    unary_potentials_are_valid_ = false;
    binary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setForegroundPoints (typename pcl::PointCloud<PointT>::Ptr foreground_points)
{
  foreground_points_.clear ();
  foreground_points_.reserve (foreground_points->points.size ());
  for (size_t i_point = 0; i_point < foreground_points->points.size (); i_point++)
    foreground_points_.push_back (foreground_points->points[i_point]);

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setForegroundPoints (std::vector<int>& foreground_points_indices)
{
  foreground_points_.clear ();
  foreground_points_.reserve (foreground_points_indices.size ());

  for (size_t i_point = 0; i_point < foreground_points_indices.size (); i_point++)
    if (foreground_points_indices[i_point] < number_of_points_)
      foreground_points_.push_back ( input_cloud_->points[ foreground_points_indices[i_point] ] );

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setBackgroundPoints (typename pcl::PointCloud<PointT>::Ptr background_points)
{
  background_points_.clear ();
  background_points_.reserve (background_points->points.size ());
  for (size_t i_point = 0; i_point < background_points->points.size (); i_point++)
    background_points_.push_back (background_points->points[i_point]);

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setBackgroundPoints (std::vector<int>& background_points_indices)
{
  background_points_.clear ();
  background_points_.reserve (background_points_indices.size ());

  for (size_t i_point = 0; i_point < background_points_indices.size (); i_point++)
    if (background_points_indices[i_point] < number_of_points_)
      background_points_.push_back ( input_cloud_->points[ background_points_indices[i_point] ] );

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
pcl::MinCutSegmentation<PointT>::getInputCloud () const
{
  return (input_cloud_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::getSigma () const
{
  return (pow (1.0 / inverse_sigma_, 0.5));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::getRadius () const
{
  return (pow (radius_, 0.5));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::getSourceWeight () const
{
  return (source_weight_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::search::Search<PointT>::Ptr
pcl::MinCutSegmentation<PointT>::getSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::MinCutSegmentation<PointT>::getNeighbourNumber () const
{
  return (number_of_neighbours_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<PointT, Eigen::aligned_allocator<PointT> >
pcl::MinCutSegmentation<PointT>::getForegroundPoints () const
{
  return (foreground_points_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<PointT, Eigen::aligned_allocator<PointT> >
pcl::MinCutSegmentation<PointT>::getBackgroundPoints () const
{
  return (background_points_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::getMaxFlow () const
{
  return (max_flow_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MinCutSegmentation<PointT>::segmentPoints ()
{
  if ( graph_is_valid_ && unary_potentials_are_valid_ && binary_potentials_are_valid_ )
    return (true);

  labels_.clear ();
  bool success = true;

  if ( !graph_is_valid_ )
  {
    success = buildGraph ();
    if (success == false)
      return (false);
    graph_is_valid_ = true;
    unary_potentials_are_valid_ = true;
    binary_potentials_are_valid_ = true;
  }

  if ( !unary_potentials_are_valid_ )
  {
    success = recalculateUnaryPotentials ();
    if (success == false)
      return (false);
    unary_potentials_are_valid_ = true;
  }

  if ( !binary_potentials_are_valid_ )
  {
    success = recalculateBinaryPotentials ();
    if (success == false)
      return (false);
    binary_potentials_are_valid_ = true;
  }

  IndexMap index_map = boost::get (boost::vertex_index, *graph_);
  ResidualCapacityMap residual_capacity = boost::get (boost::edge_residual_capacity, *graph_);

  max_flow_ = boost::boykov_kolmogorov_max_flow (*graph_, source_, sink_);

  assembleLabels (residual_capacity);

  return (success);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MinCutSegmentation<PointT>::buildGraph ()
{
  if (input_cloud_ == 0 || number_of_points_ == 0 || foreground_points_.empty () == true )
    return (false);

  if (search_ == 0)
    search_ = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);

  graph_.reset ();
  graph_ = boost::shared_ptr< mGraph > (new mGraph ());

  capacity_.reset ();
  capacity_ = boost::shared_ptr<CapacityMap> (new CapacityMap ());
  *capacity_ = boost::get (boost::edge_capacity, *graph_);

  reverse_edges_.reset ();
  reverse_edges_ = boost::shared_ptr<ReverseEdgeMap> (new ReverseEdgeMap ());
  *reverse_edges_ = boost::get (boost::edge_reverse, *graph_);

  VertexDescriptor vertex_descriptor(0);
  vertices_.clear ();
  vertices_.resize (number_of_points_ + 2, vertex_descriptor);

  std::set<int> out_edges_marker;
  edge_marker_.clear ();
  edge_marker_.resize (number_of_points_ + 2, out_edges_marker);

  for (int i_point = 0; i_point < number_of_points_ + 2; i_point++)
    vertices_[i_point] = boost::add_vertex (*graph_);

  source_ = vertices_[number_of_points_];
  sink_ = vertices_[number_of_points_ + 1];

  for (int i_point = 0; i_point < number_of_points_; i_point++)
  {
    double source_weight = 0.0;
    double sink_weight = 0.0;
    calculateUnaryPotential (i_point, source_weight, sink_weight);
    addEdge (static_cast<int> (source_), i_point, source_weight);
    addEdge (i_point, static_cast<int> (sink_), sink_weight);
  }

  std::vector<int> neighbours;
  std::vector<float> distances;
  search_->setInputCloud (input_cloud_);
  for (int i_point = 0; i_point < number_of_points_; i_point++)
  {
    search_->nearestKSearch (i_point, number_of_neighbours_, neighbours, distances);
    for (size_t i_nghbr = 1; i_nghbr < neighbours.size (); i_nghbr++)
    {
      double weight = calculateBinaryPotential (i_point, neighbours[i_nghbr]);
      addEdge (i_point, neighbours[i_nghbr], weight);
      addEdge (neighbours[i_nghbr], i_point, weight);
    }
    neighbours.clear ();
    distances.clear ();
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::calculateBinaryPotential (int source, int target) const
{
  double weight = 0.0;
  double distance = 0.0;
  distance += (input_cloud_->points[source].x - input_cloud_->points[target].x) * (input_cloud_->points[source].x - input_cloud_->points[target].x);
  distance += (input_cloud_->points[source].y - input_cloud_->points[target].y) * (input_cloud_->points[source].y - input_cloud_->points[target].y);
  distance += (input_cloud_->points[source].z - input_cloud_->points[target].z) * (input_cloud_->points[source].z - input_cloud_->points[target].z);
  distance *= inverse_sigma_;
  weight = exp (-distance);

  return (weight);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::calculateUnaryPotential (int point, double& source_weight, double& sink_weight) const
{
  double min_dist_to_foreground = std::numeric_limits<double>::max ();
  //double min_dist_to_background = std::numeric_limits<double>::max ();
  double closest_foreground_point[] = {0.0, 0.0};
  //double closest_background_point[] = {0.0, 0.0};
  double initial_point[] = {0.0, 0.0};

  initial_point[0] = input_cloud_->points[point].x;
  initial_point[1] = input_cloud_->points[point].y;

  for (size_t i_point = 0; i_point < foreground_points_.size (); i_point++)
  {
    double dist = 0.0;
    dist += (foreground_points_[i_point].x - initial_point[0]) * (foreground_points_[i_point].x - initial_point[0]);
    dist += (foreground_points_[i_point].y - initial_point[1]) * (foreground_points_[i_point].y - initial_point[1]);
    if (min_dist_to_foreground > dist)
    {
      min_dist_to_foreground = dist;
      closest_foreground_point[0] = foreground_points_[i_point].x;
      closest_foreground_point[1] = foreground_points_[i_point].y;
    }
  }

  sink_weight = pow (min_dist_to_foreground / radius_, 0.5);

  source_weight = source_weight_;
  return;
/*
  if (background_points_.size () == 0)
    return;

  for (int i_point = 0; i_point < background_points_.size (); i_point++)
  {
    double dist = 0.0;
    dist += (background_points_[i_point].x - initial_point[0]) * (background_points_[i_point].x - initial_point[0]);
    dist += (background_points_[i_point].y - initial_point[1]) * (background_points_[i_point].y - initial_point[1]);
    if (min_dist_to_background > dist)
    {
      min_dist_to_background = dist;
      closest_background_point[0] = background_points_[i_point].x;
      closest_background_point[1] = background_points_[i_point].y;
    }
  }

  if (min_dist_to_background <= epsilon_)
  {
    source_weight = 0.0;
    sink_weight = 1.0;
    return;
  }

  source_weight = 1.0 / (1.0 + pow (min_dist_to_background / min_dist_to_foreground, 0.5));
  sink_weight = 1 - source_weight;
*/
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MinCutSegmentation<PointT>::addEdge (int source, int target, double weight)
{
  std::set<int>::iterator iter_out = edge_marker_[source].find (target);
  if ( iter_out != edge_marker_[source].end () )
    return (false);

  EdgeDescriptor edge;
  EdgeDescriptor reverse_edge;
  bool edge_was_added, reverse_edge_was_added;

  boost::tie (edge, edge_was_added) = boost::add_edge ( vertices_[source], vertices_[target], *graph_ );
  boost::tie (reverse_edge, reverse_edge_was_added) = boost::add_edge ( vertices_[target], vertices_[source], *graph_ );
  if ( !edge_was_added || !reverse_edge_was_added )
    return (false);

  (*capacity_)[edge] = weight;
  (*capacity_)[reverse_edge] = 0.0;
  (*reverse_edges_)[edge] = reverse_edge;
  (*reverse_edges_)[reverse_edge] = edge;
  edge_marker_[source].insert (target);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MinCutSegmentation<PointT>::recalculateUnaryPotentials ()
{
  OutEdgeIterator src_edge_iter;
  OutEdgeIterator src_edge_end;
  std::pair<EdgeDescriptor, bool> sink_edge;

  for (boost::tie (src_edge_iter, src_edge_end) = boost::out_edges (source_, *graph_); src_edge_iter != src_edge_end; src_edge_iter++)
  {
    double source_weight = 0.0;
    double sink_weight = 0.0;
    sink_edge.second = false;
    calculateUnaryPotential (static_cast<int> (boost::target (*src_edge_iter, *graph_)), source_weight, sink_weight);
    sink_edge = boost::lookup_edge (boost::target (*src_edge_iter, *graph_), sink_, *graph_);
    if (!sink_edge.second)
      return (false);

    (*capacity_)[*src_edge_iter] = source_weight;
    (*capacity_)[sink_edge.first] = sink_weight;
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MinCutSegmentation<PointT>::recalculateBinaryPotentials ()
{
  VertexIterator vertex_iter;
  VertexIterator vertex_end;
  OutEdgeIterator edge_iter;
  OutEdgeIterator edge_end;

  std::vector< std::set<VertexDescriptor> > edge_marker;
  std::set<VertexDescriptor> out_edges_marker;
  edge_marker.clear ();
  edge_marker.resize (number_of_points_ + 2, out_edges_marker);

  for (boost::tie (vertex_iter, vertex_end) = boost::vertices (*graph_); vertex_iter != vertex_end; vertex_iter++)
  {
    VertexDescriptor source_vertex = *vertex_iter;
    if (source_vertex == source_ || source_vertex == sink_)
      continue;
    for (boost::tie (edge_iter, edge_end) = boost::out_edges (source_vertex, *graph_); edge_iter != edge_end; edge_iter++)
    {
      //If this is not the edge of the graph, but the reverse fictitious edge that is needed for the algorithm then continue
      EdgeDescriptor reverse_edge = (*reverse_edges_)[*edge_iter];
      if ((*capacity_)[reverse_edge] != 0.0)
        continue;

      //If we already changed weight for this edge then continue
      VertexDescriptor target_vertex = boost::target (*edge_iter, *graph_);
      std::set<VertexDescriptor>::iterator iter_out = edge_marker[static_cast<int> (source_vertex)].find (target_vertex);
      if ( iter_out != edge_marker[static_cast<int> (source_vertex)].end () )
        continue;

      if (target_vertex != source_ && target_vertex != sink_)
      {
        //Change weight and remember that this edges were updated
        double weight = calculateBinaryPotential (static_cast<int> (target_vertex), static_cast<int> (source_vertex));
        (*capacity_)[*edge_iter] = weight;
        edge_marker[static_cast<int> (source_vertex)].insert (target_vertex);
      }
    }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::assembleLabels (ResidualCapacityMap& residual_capacity)
{
  labels_.clear ();
  labels_.resize (number_of_points_, 0);

  OutEdgeIterator edge_iter, edge_end;
  for ( boost::tie (edge_iter, edge_end) = boost::out_edges (source_, *graph_); edge_iter != edge_end; edge_iter++ )
    if (residual_capacity[*edge_iter] > epsilon_)
      labels_[edge_iter->m_target] = 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::MinCutSegmentation<PointT>::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!labels_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
    unsigned char foreground_color[3] = {255, 255, 255};
    unsigned char background_color[3] = {255, 0, 0};
    colored_cloud->width = input_cloud_->width;
    colored_cloud->height = input_cloud_->height;
    colored_cloud->is_dense = input_cloud_->is_dense;
    for (size_t i_point = 0; i_point < size_t (number_of_points_); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(input_cloud_->points[i_point].data);
      point.y = *(input_cloud_->points[i_point].data + 1);
      point.z = *(input_cloud_->points[i_point].data + 2);
      if (labels_[i_point] == 0)
      {
        point.r = background_color[0];
        point.g = background_color[1];
        point.b = background_color[2];
      }
      else
      {
        point.r = foreground_color[0];
        point.g = foreground_color[1];
        point.b = foreground_color[2];
      }
      colored_cloud->points.push_back (point);
    }
  }

  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename boost::shared_ptr<typename pcl::MinCutSegmentation<PointT>::mGraph>
pcl::MinCutSegmentation<PointT>::getGraph () const
{
  return (graph_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int>
pcl::MinCutSegmentation<PointT>::getSegments () const
{
  return (labels_);
}

#define PCL_INSTANTIATE_MinCutSegmentation(T) template class pcl::MinCutSegmentation<T>;

#endif    // PCL_SEGMENTATION_MIN_CUT_SEGMENTATION_HPP_

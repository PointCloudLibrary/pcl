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
 * $Id:$
 *
 */

#ifndef PCL_SEGMENTATION_MIN_CUT_SEGMENTATION_HPP_
#define PCL_SEGMENTATION_MIN_CUT_SEGMENTATION_HPP_

#include <boost/graph/boykov_kolmogorov_max_flow.hpp> // for boykov_kolmogorov_max_flow
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <cmath>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::MinCutSegmentation<PointT>::MinCutSegmentation () :
  inverse_sigma_ (16.0),
  binary_potentials_are_valid_ (false),
  epsilon_ (0.0001),
  radius_ (16.0),
  unary_potentials_are_valid_ (false),
  source_weight_ (0.8),
  search_ (),
  number_of_neighbours_ (14),
  graph_is_valid_ (false),
  foreground_points_ (0),
  background_points_ (0),
  clusters_ (0),
  vertices_ (0),
  edge_marker_ (0),
  source_ (),/////////////////////////////////
  sink_ (),///////////////////////////////////
  max_flow_ (0.0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::MinCutSegmentation<PointT>::~MinCutSegmentation ()
{
  foreground_points_.clear ();
  background_points_.clear ();
  clusters_.clear ();
  vertices_.clear ();
  edge_marker_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setInputCloud (const PointCloudConstPtr &cloud)
{
  input_ = cloud;
  graph_is_valid_ = false;
  unary_potentials_are_valid_ = false;
  binary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::getSigma () const
{
  return (pow (1.0 / inverse_sigma_, 0.5));
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
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::getRadius () const
{
  return (pow (radius_, 0.5));
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
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::getSourceWeight () const
{
  return (source_weight_);
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
template <typename PointT> typename pcl::MinCutSegmentation<PointT>::KdTreePtr
pcl::MinCutSegmentation<PointT>::getSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setSearchMethod (const KdTreePtr& tree)
{
  search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
pcl::MinCutSegmentation<PointT>::getNumberOfNeighbours () const
{
  return (number_of_neighbours_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setNumberOfNeighbours (unsigned int neighbour_number)
{
  if (number_of_neighbours_ != neighbour_number && neighbour_number != 0)
  {
    number_of_neighbours_ = neighbour_number;
    graph_is_valid_ = false;
    unary_potentials_are_valid_ = false;
    binary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<PointT, Eigen::aligned_allocator<PointT> >
pcl::MinCutSegmentation<PointT>::getForegroundPoints () const
{
  return (foreground_points_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setForegroundPoints (typename pcl::PointCloud<PointT>::Ptr foreground_points)
{
  foreground_points_.clear ();
  foreground_points_.insert(
      foreground_points_.end(), foreground_points->cbegin(), foreground_points->cend());

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<PointT, Eigen::aligned_allocator<PointT> >
pcl::MinCutSegmentation<PointT>::getBackgroundPoints () const
{
  return (background_points_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::setBackgroundPoints (typename pcl::PointCloud<PointT>::Ptr background_points)
{
  background_points_.clear ();
  background_points_.insert(
      background_points_.end(), background_points->cbegin(), background_points->cend());

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::extract (std::vector <pcl::PointIndices>& clusters)
{
  clusters.clear ();

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  if ( graph_is_valid_ && unary_potentials_are_valid_ && binary_potentials_are_valid_ )
  {
    clusters.reserve (clusters_.size ());
    std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));
    deinitCompute ();
    return;
  }

  clusters_.clear ();

  if ( !graph_is_valid_ )
  {
    bool success = buildGraph ();
    if (!success)
    {
      deinitCompute ();
      return;
    }
    graph_is_valid_ = true;
    unary_potentials_are_valid_ = true;
    binary_potentials_are_valid_ = true;
  }

  if ( !unary_potentials_are_valid_ )
  {
    bool success = recalculateUnaryPotentials ();
    if (!success)
    {
      deinitCompute ();
      return;
    }
    unary_potentials_are_valid_ = true;
  }

  if ( !binary_potentials_are_valid_ )
  {
    bool success = recalculateBinaryPotentials ();
    if (!success)
    {
      deinitCompute ();
      return;
    }
    binary_potentials_are_valid_ = true;
  }

  //IndexMap index_map = boost::get (boost::vertex_index, *graph_);
  ResidualCapacityMap residual_capacity = boost::get (boost::edge_residual_capacity, *graph_);

  max_flow_ = boost::boykov_kolmogorov_max_flow (*graph_, source_, sink_);

  assembleLabels (residual_capacity);

  clusters.reserve (clusters_.size ());
  std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::getMaxFlow () const
{
  return (max_flow_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::MinCutSegmentation<PointT>::mGraphPtr
pcl::MinCutSegmentation<PointT>::getGraph () const
{
  return (graph_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::MinCutSegmentation<PointT>::buildGraph ()
{
  const auto number_of_points = input_->size ();
  const auto number_of_indices = indices_->size ();

  if (input_->points.empty () || number_of_points == 0 || foreground_points_.empty () == true )
    return (false);

  if (!search_)
    search_.reset (new pcl::search::KdTree<PointT>);

  graph_.reset (new mGraph);

  capacity_.reset (new CapacityMap);
  *capacity_ = boost::get (boost::edge_capacity, *graph_);

  reverse_edges_.reset (new ReverseEdgeMap);
  *reverse_edges_ = boost::get (boost::edge_reverse, *graph_);

  VertexDescriptor vertex_descriptor(0);
  vertices_.clear ();
  vertices_.resize (number_of_points + 2, vertex_descriptor);

  std::set<int> out_edges_marker;
  edge_marker_.clear ();
  edge_marker_.resize (number_of_points + 2, out_edges_marker);

  for (std::size_t i_point = 0; i_point < number_of_points + 2; i_point++)
    vertices_[i_point] = boost::add_vertex (*graph_);

  source_ = vertices_[number_of_points];
  sink_ = vertices_[number_of_points + 1];

  for (const auto& point_index : (*indices_))
  {
    double source_weight = 0.0;
    double sink_weight = 0.0;
    calculateUnaryPotential (point_index, source_weight, sink_weight);
    addEdge (static_cast<int> (source_), point_index, source_weight);
    addEdge (point_index, static_cast<int> (sink_), sink_weight);
  }

  pcl::Indices neighbours;
  std::vector<float> distances;
  search_->setInputCloud (input_, indices_);
  for (std::size_t i_point = 0; i_point < number_of_indices; i_point++)
  {
    index_t point_index = (*indices_)[i_point];
    search_->nearestKSearch (i_point, number_of_neighbours_, neighbours, distances);
    for (std::size_t i_nghbr = 1; i_nghbr < neighbours.size (); i_nghbr++)
    {
      double weight = calculateBinaryPotential (point_index, neighbours[i_nghbr]);
      addEdge (point_index, neighbours[i_nghbr], weight);
      addEdge (neighbours[i_nghbr], point_index, weight);
    }
    neighbours.clear ();
    distances.clear ();
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::MinCutSegmentation<PointT>::calculateUnaryPotential (int point, double& source_weight, double& sink_weight) const
{
  double min_dist_to_foreground = std::numeric_limits<double>::max ();
  //double min_dist_to_background = std::numeric_limits<double>::max ();
  //double closest_background_point[] = {0.0, 0.0};
  double initial_point[] = {0.0, 0.0};

  initial_point[0] = (*input_)[point].x;
  initial_point[1] = (*input_)[point].y;

  for (const auto& fg_point : foreground_points_)
  {
    double dist = 0.0;
    dist += (fg_point.x - initial_point[0]) * (fg_point.x - initial_point[0]);
    dist += (fg_point.y - initial_point[1]) * (fg_point.y - initial_point[1]);
    if (min_dist_to_foreground > dist)
    {
      min_dist_to_foreground = dist;
    }
  }

  sink_weight = pow (min_dist_to_foreground / radius_, 0.5);

  source_weight = source_weight_;
  return;
/*
  if (background_points_.size () == 0)
    return;

  for (const auto& bg_point : background_points_)
  {
    double dist = 0.0;
    dist += (bg_point.x - initial_point[0]) * (bg_point.x - initial_point[0]);
    dist += (bg_point.y - initial_point[1]) * (bg_point.y - initial_point[1]);
    if (min_dist_to_background > dist)
    {
      min_dist_to_background = dist;
      closest_background_point[0] = bg_point.x;
      closest_background_point[1] = bg_point.y;
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
template <typename PointT> double
pcl::MinCutSegmentation<PointT>::calculateBinaryPotential (int source, int target) const
{
  double weight = 0.0;
  double distance = 0.0;
  distance += ((*input_)[source].x - (*input_)[target].x) * ((*input_)[source].x - (*input_)[target].x);
  distance += ((*input_)[source].y - (*input_)[target].y) * ((*input_)[source].y - (*input_)[target].y);
  distance += ((*input_)[source].z - (*input_)[target].z) * ((*input_)[source].z - (*input_)[target].z);
  distance *= inverse_sigma_;
  weight = std::exp (-distance);

  return (weight);
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
  edge_marker.resize (indices_->size () + 2, out_edges_marker);

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
  std::vector<int> labels;
  labels.resize (input_->size (), 0);
  for (const auto& i_point : (*indices_))
    labels[i_point] = 1;

  clusters_.clear ();

  pcl::PointIndices segment;
  clusters_.resize (2, segment);

  OutEdgeIterator edge_iter, edge_end;
  for ( boost::tie (edge_iter, edge_end) = boost::out_edges (source_, *graph_); edge_iter != edge_end; edge_iter++ )
  {
    if (labels[edge_iter->m_target] == 1)
    {
      if (residual_capacity[*edge_iter] > epsilon_)
        clusters_[1].indices.push_back (static_cast<int> (edge_iter->m_target));
      else
        clusters_[0].indices.push_back (static_cast<int> (edge_iter->m_target));
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::MinCutSegmentation<PointT>::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
    unsigned char foreground_color[3] = {255, 255, 255};
    unsigned char background_color[3] = {255, 0, 0};
    colored_cloud->width = (clusters_[0].indices.size () + clusters_[1].indices.size ());
    colored_cloud->height = 1;
    colored_cloud->is_dense = input_->is_dense;

    pcl::PointXYZRGB point;
    for (const auto& point_index : (clusters_[0].indices))
    {
      point.x = *((*input_)[point_index].data);
      point.y = *((*input_)[point_index].data + 1);
      point.z = *((*input_)[point_index].data + 2);
      point.r = background_color[0];
      point.g = background_color[1];
      point.b = background_color[2];
      colored_cloud->points.push_back (point);
    }

    for (const auto& point_index : (clusters_[1].indices))
    {
      point.x = *((*input_)[point_index].data);
      point.y = *((*input_)[point_index].data + 1);
      point.z = *((*input_)[point_index].data + 2);
      point.r = foreground_color[0];
      point.g = foreground_color[1];
      point.b = foreground_color[2];
      colored_cloud->points.push_back (point);
    }
  }

  return (colored_cloud);
}

#define PCL_INSTANTIATE_MinCutSegmentation(T) template class pcl::MinCutSegmentation<T>;

#endif    // PCL_SEGMENTATION_MIN_CUT_SEGMENTATION_HPP_

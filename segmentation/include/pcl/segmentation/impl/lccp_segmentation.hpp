/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */

#ifndef PCL_SEGMENTATION_IMPL_LCCP_SEGMENTATION_HPP_
#define PCL_SEGMENTATION_IMPL_LCCP_SEGMENTATION_HPP_

#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/common/common.h>


//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////////////////// Public Functions /////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////



template <typename PointT>
pcl::LCCPSegmentation<PointT>::LCCPSegmentation () :
  concavity_tolerance_threshold_ (10),
  grouping_data_valid_ (false),
  supervoxels_set_ (false),
  use_smoothness_check_ (false),
  smoothness_threshold_ (0.1),
  use_sanity_check_ (false),  
  seed_resolution_ (0),
  voxel_resolution_ (0),
  k_factor_ (0),
  min_segment_size_ (0)
{
}

template <typename PointT>
pcl::LCCPSegmentation<PointT>::~LCCPSegmentation ()
{
}

template <typename PointT> void
pcl::LCCPSegmentation<PointT>::reset ()
{
  sv_adjacency_list_.clear ();
  processed_.clear ();
  sv_label_to_supervoxel_map_.clear ();
  sv_label_to_seg_label_map_.clear ();
  seg_label_to_sv_list_map_.clear ();
  seg_label_to_neighbor_set_map_.clear ();
  grouping_data_valid_ = false;
  supervoxels_set_ = false;
}

template <typename PointT> void
pcl::LCCPSegmentation<PointT>::segment ()
{
  if (supervoxels_set_)
  {
    // Calculate for every Edge if the connection is convex or invalid
    // This effectively performs the segmentation.
    calculateConvexConnections (sv_adjacency_list_);

    // Correct edge relations using extended convexity definition if k>0
    applyKconvexity (k_factor_);

    // group supervoxels
    doGrouping ();
    
    grouping_data_valid_ = true;
    
    // merge small segments
    mergeSmallSegments ();
  }
  else
    PCL_WARN ("[pcl::LCCPSegmentation::segment] WARNING: Call function setInputSupervoxels first. Nothing has been done. \n");
}


template <typename PointT> void
pcl::LCCPSegmentation<PointT>::relabelCloud (pcl::PointCloud<pcl::PointXYZL> &labeled_cloud_arg)
{
  if (grouping_data_valid_)
  {
    // Relabel all Points in cloud with new labels
    for (auto &voxel : labeled_cloud_arg)
    {
      voxel.label = sv_label_to_seg_label_map_[voxel.label];
    }
  }
  else
  {
    PCL_WARN ("[pcl::LCCPSegmentation::relabelCloud] WARNING: Call function segment first. Nothing has been done. \n");
  }
}



//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////////////////// Protected Functions //////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

template <typename PointT> void
pcl::LCCPSegmentation<PointT>::computeSegmentAdjacency ()
{  
  seg_label_to_neighbor_set_map_.clear ();

  std::uint32_t current_segLabel;
  std::uint32_t neigh_segLabel;

  VertexIterator sv_itr, sv_itr_end;
  //The vertices in the supervoxel adjacency list are the supervoxel centroids
  // For every Supervoxel..
  for(std::tie(sv_itr, sv_itr_end) = boost::vertices(sv_adjacency_list_); sv_itr != sv_itr_end; ++sv_itr)  // For all supervoxels
  {
    const std::uint32_t& sv_label = sv_adjacency_list_[*sv_itr];
    current_segLabel = sv_label_to_seg_label_map_[sv_label];

    AdjacencyIterator itr_neighbor, itr_neighbor_end;
    // ..look at all neighbors and insert their labels into the neighbor set
    for (std::tie(itr_neighbor, itr_neighbor_end) = boost::adjacent_vertices (*sv_itr, sv_adjacency_list_); itr_neighbor != itr_neighbor_end; ++itr_neighbor)
    {
      const std::uint32_t& neigh_label = sv_adjacency_list_[*itr_neighbor];
      neigh_segLabel = sv_label_to_seg_label_map_[neigh_label];

      if (current_segLabel != neigh_segLabel)
      {
        seg_label_to_neighbor_set_map_[current_segLabel].insert (neigh_segLabel);
      }
    }
  }
}

template <typename PointT> void
pcl::LCCPSegmentation<PointT>::mergeSmallSegments ()
{
  if (min_segment_size_ == 0)
    return;

  computeSegmentAdjacency ();

  std::set<std::uint32_t> filteredSegLabels;

  bool continue_filtering = true;

  while (continue_filtering)
  {
    continue_filtering = false;
    unsigned int nr_filtered = 0;

    VertexIterator sv_itr, sv_itr_end;
    // Iterate through all supervoxels, check if they are in a "small" segment -> change label to largest neighborID
    for (std::tie(sv_itr, sv_itr_end) = boost::vertices (sv_adjacency_list_); sv_itr != sv_itr_end; ++sv_itr)  // For all supervoxels
    {
      const std::uint32_t& sv_label = sv_adjacency_list_[*sv_itr];
      std::uint32_t current_seg_label = sv_label_to_seg_label_map_[sv_label];
      std::uint32_t largest_neigh_seg_label = current_seg_label;
      std::uint32_t largest_neigh_size = seg_label_to_sv_list_map_[current_seg_label].size ();

      const std::uint32_t& nr_neighbors = seg_label_to_neighbor_set_map_[current_seg_label].size ();
      if (nr_neighbors == 0)
        continue;

      if (seg_label_to_sv_list_map_[current_seg_label].size () <= min_segment_size_)
      {
        continue_filtering = true;
        nr_filtered++;

        // Find largest neighbor
        for (auto neighbors_itr = seg_label_to_neighbor_set_map_[current_seg_label].cbegin (); neighbors_itr != seg_label_to_neighbor_set_map_[current_seg_label].cend (); ++neighbors_itr)
        {
          if (seg_label_to_sv_list_map_[*neighbors_itr].size () >= largest_neigh_size)
          {
            largest_neigh_seg_label = *neighbors_itr;
            largest_neigh_size = seg_label_to_sv_list_map_[*neighbors_itr].size ();
          }
        }

        // Add to largest neighbor
        if (largest_neigh_seg_label != current_seg_label)
        {
          if (filteredSegLabels.count (largest_neigh_seg_label) > 0)
            continue;  // If neighbor was already assigned to someone else

          sv_label_to_seg_label_map_[sv_label] = largest_neigh_seg_label;
          filteredSegLabels.insert (current_seg_label);

          // Assign supervoxel labels of filtered segment to new owner
          for (auto sv_ID_itr = seg_label_to_sv_list_map_[current_seg_label].cbegin (); sv_ID_itr != seg_label_to_sv_list_map_[current_seg_label].cend (); ++sv_ID_itr)
          {
            seg_label_to_sv_list_map_[largest_neigh_seg_label].insert (*sv_ID_itr);
          }
        }
      }
    }

    // Erase filtered Segments from segment map
    for (const unsigned int &filteredSegLabel : filteredSegLabels)
    {
      seg_label_to_sv_list_map_.erase (filteredSegLabel);
    }

    // After filtered Segments are deleted, compute completely new adjacency map
    // NOTE Recomputing the adjacency of every segment in every iteration is an easy but inefficient solution.
    // Because the number of segments in an average scene is usually well below 1000, the time spend for noise filtering is still negligible in most cases
    computeSegmentAdjacency ();
  }  // End while (Filtering)
}

template <typename PointT> void
pcl::LCCPSegmentation<PointT>::prepareSegmentation (const std::map<std::uint32_t, typename pcl::Supervoxel<PointT>::Ptr>& supervoxel_clusters_arg,
                                                    const std::multimap<std::uint32_t, std::uint32_t>& label_adjaceny_arg)
{
  // Clear internal data
  reset ();

  // Copy map with supervoxel pointers
  sv_label_to_supervoxel_map_ = supervoxel_clusters_arg;

  //    Build a boost adjacency list from the adjacency multimap
  std::map<std::uint32_t, VertexID> label_ID_map;

  // Add all supervoxel labels as vertices
  for (typename std::map<std::uint32_t, typename pcl::Supervoxel<PointT>::Ptr>::iterator svlabel_itr = sv_label_to_supervoxel_map_.begin ();
      svlabel_itr != sv_label_to_supervoxel_map_.end (); ++svlabel_itr)
  {
    const std::uint32_t& sv_label = svlabel_itr->first;
    VertexID node_id = boost::add_vertex (sv_adjacency_list_);
    sv_adjacency_list_[node_id] = sv_label;
    label_ID_map[sv_label] = node_id;
  }

  // Add all edges
  for (const auto &sv_neighbors_itr : label_adjaceny_arg)
  {
    const std::uint32_t& sv_label = sv_neighbors_itr.first;
    const std::uint32_t& neighbor_label = sv_neighbors_itr.second;

    VertexID u = label_ID_map[sv_label];
    VertexID v = label_ID_map[neighbor_label];
    
    boost::add_edge (u, v, sv_adjacency_list_);
  }

  // Initialization
  // clear the processed_ map
  seg_label_to_sv_list_map_.clear ();
  for (typename std::map<std::uint32_t, typename pcl::Supervoxel<PointT>::Ptr>::iterator svlabel_itr = sv_label_to_supervoxel_map_.begin ();
      svlabel_itr != sv_label_to_supervoxel_map_.end (); ++svlabel_itr)
  {
    const std::uint32_t& sv_label = svlabel_itr->first;
    processed_[sv_label] = false;
    sv_label_to_seg_label_map_[sv_label] = 0;
  }
}




template <typename PointT> void
pcl::LCCPSegmentation<PointT>::doGrouping ()
{
  // clear the processed_ map
  seg_label_to_sv_list_map_.clear ();
  for (typename std::map<std::uint32_t, typename pcl::Supervoxel<PointT>::Ptr>::iterator svlabel_itr = sv_label_to_supervoxel_map_.begin ();
      svlabel_itr != sv_label_to_supervoxel_map_.end (); ++svlabel_itr)
  {
    const std::uint32_t& sv_label = svlabel_itr->first;
    processed_[sv_label] = false;
    sv_label_to_seg_label_map_[sv_label] = 0;
  }
  
  VertexIterator sv_itr, sv_itr_end;
  // Perform depth search on the graph and recursively group all supervoxels with convex connections
  //The vertices in the supervoxel adjacency list are the supervoxel centroids
  // Note: *sv_itr is of type " boost::graph_traits<VoxelAdjacencyList>::vertex_descriptor " which it nothing but a typedef of std::size_t..
  unsigned int segment_label = 1;  // This starts at 1, because 0 is reserved for errors
  for (std::tie(sv_itr, sv_itr_end) = boost::vertices (sv_adjacency_list_); sv_itr != sv_itr_end; ++sv_itr)  // For all supervoxels
  {
    const VertexID sv_vertex_id = *sv_itr;
    const std::uint32_t& sv_label = sv_adjacency_list_[sv_vertex_id];
    if (!processed_[sv_label])
    {
      // Add neighbors (and their neighbors etc.) to group if similarity constraint is met
      recursiveSegmentGrowing (sv_vertex_id, segment_label);
      ++segment_label;  // After recursive grouping ended (no more neighbors to consider) -> go to next group
    }
  }
}

template <typename PointT> void
pcl::LCCPSegmentation<PointT>::recursiveSegmentGrowing (const VertexID &query_point_id,
                                                        const unsigned int segment_label)
{
  const std::uint32_t& sv_label = sv_adjacency_list_[query_point_id];

  processed_[sv_label] = true;

  // The next two lines add the supervoxel to the segment
  sv_label_to_seg_label_map_[sv_label] = segment_label;
  seg_label_to_sv_list_map_[segment_label].insert (sv_label);

  OutEdgeIterator out_Edge_itr, out_Edge_itr_end;
  // Iterate through all neighbors of this supervoxel and check whether they should be merged with the current supervoxel
  // boost::out_edges (query_point_id, sv_adjacency_list_): adjacent vertices to node (*itr) in graph sv_adjacency_list_
  for (std::tie(out_Edge_itr, out_Edge_itr_end) = boost::out_edges (query_point_id, sv_adjacency_list_); out_Edge_itr != out_Edge_itr_end; ++out_Edge_itr)
  {
    const VertexID neighbor_ID = boost::target (*out_Edge_itr, sv_adjacency_list_);
    const std::uint32_t& neighbor_label = sv_adjacency_list_[neighbor_ID];

    if (!processed_[neighbor_label])  // If neighbor was not already processed
    {
      if (sv_adjacency_list_[*out_Edge_itr].is_valid)
      {
        recursiveSegmentGrowing (neighbor_ID, segment_label);
      }
    }
  }  // End neighbor loop
}

template <typename PointT> void
pcl::LCCPSegmentation<PointT>::applyKconvexity (const unsigned int k_arg)
{
  if (k_arg == 0)
    return;

  EdgeIterator edge_itr, edge_itr_end, next_edge;
  // Check all edges in the graph for k-convexity
  for (std::tie (edge_itr, edge_itr_end) = boost::edges (sv_adjacency_list_), next_edge = edge_itr; edge_itr != edge_itr_end; edge_itr = next_edge)
  {
    ++next_edge;  // next_edge iterator is necessary, because removing an edge invalidates the iterator to the current edge

    bool is_convex = sv_adjacency_list_[*edge_itr].is_convex;

    if (is_convex)  // If edge is (0-)convex
    {
      unsigned int kcount = 0;

      const VertexID source = boost::source (*edge_itr, sv_adjacency_list_);
      const VertexID target = boost::target (*edge_itr, sv_adjacency_list_);

      OutEdgeIterator source_neighbors_itr, source_neighbors_itr_end;
      // Find common neighbors, check their connection
      for (std::tie(source_neighbors_itr, source_neighbors_itr_end) = boost::out_edges (source, sv_adjacency_list_); source_neighbors_itr != source_neighbors_itr_end; ++source_neighbors_itr)  // For all supervoxels
      {
        VertexID source_neighbor_ID = boost::target (*source_neighbors_itr, sv_adjacency_list_);

        OutEdgeIterator target_neighbors_itr, target_neighbors_itr_end;
        for (std::tie(target_neighbors_itr, target_neighbors_itr_end) = boost::out_edges (target, sv_adjacency_list_); target_neighbors_itr != target_neighbors_itr_end; ++target_neighbors_itr)  // For all supervoxels
        {
          VertexID target_neighbor_ID = boost::target (*target_neighbors_itr, sv_adjacency_list_);
          if (source_neighbor_ID == target_neighbor_ID)  // Common neighbor
          {
            EdgeID src_edge = boost::edge (source, source_neighbor_ID, sv_adjacency_list_).first;
            EdgeID tar_edge = boost::edge (target, source_neighbor_ID, sv_adjacency_list_).first;

            bool src_is_convex = (sv_adjacency_list_)[src_edge].is_convex;
            bool tar_is_convex = (sv_adjacency_list_)[tar_edge].is_convex;

            if (src_is_convex && tar_is_convex)
              ++kcount;

            break;
          }
        }

        if (kcount >= k_arg)  // Connection is k-convex, stop search
          break;
      }

      // Check k convexity
      if (kcount < k_arg)
        (sv_adjacency_list_)[*edge_itr].is_valid = false;
    }
  }
}

template <typename PointT> void
pcl::LCCPSegmentation<PointT>::calculateConvexConnections (SupervoxelAdjacencyList& adjacency_list_arg)
{

  EdgeIterator edge_itr, edge_itr_end, next_edge;
  for (std::tie(edge_itr, edge_itr_end) = boost::edges (adjacency_list_arg), next_edge = edge_itr; edge_itr != edge_itr_end; edge_itr = next_edge)
  {
    ++next_edge;  // next_edge iterator is necessary, because removing an edge invalidates the iterator to the current edge

    std::uint32_t source_sv_label = adjacency_list_arg[boost::source (*edge_itr, adjacency_list_arg)];
    std::uint32_t target_sv_label = adjacency_list_arg[boost::target (*edge_itr, adjacency_list_arg)];

    float normal_difference;
    bool is_convex = connIsConvex (source_sv_label, target_sv_label, normal_difference);
    adjacency_list_arg[*edge_itr].is_convex = is_convex;
    adjacency_list_arg[*edge_itr].is_valid = is_convex;
    adjacency_list_arg[*edge_itr].normal_difference = normal_difference;
  }
}

template <typename PointT> bool
pcl::LCCPSegmentation<PointT>::connIsConvex (const std::uint32_t source_label_arg,
                                             const std::uint32_t target_label_arg,
                                             float &normal_angle)
{
  typename pcl::Supervoxel<PointT>::Ptr& sv_source = sv_label_to_supervoxel_map_[source_label_arg];
  typename pcl::Supervoxel<PointT>::Ptr& sv_target = sv_label_to_supervoxel_map_[target_label_arg];

  const Eigen::Vector3f& source_centroid = sv_source->centroid_.getVector3fMap ();
  const Eigen::Vector3f& target_centroid = sv_target->centroid_.getVector3fMap ();

  const Eigen::Vector3f& source_normal = sv_source->normal_.getNormalVector3fMap (). normalized ();
  const Eigen::Vector3f& target_normal = sv_target->normal_.getNormalVector3fMap (). normalized ();

  //NOTE For angles below 0 nothing will be merged
  if (concavity_tolerance_threshold_ < 0)
  {
    return (false);
  }

  bool is_convex = true;
  bool is_smooth = true;

  normal_angle = getAngle3D (source_normal, target_normal, true);
  //  Geometric comparisons
  Eigen::Vector3f vec_t_to_s, vec_s_to_t;
  
  vec_t_to_s = source_centroid - target_centroid;
  vec_s_to_t = -vec_t_to_s;

  Eigen::Vector3f ncross;
  ncross = source_normal.cross (target_normal);

  // Smoothness Check: Check if there is a step between adjacent patches
  if (use_smoothness_check_)
  {
    float expected_distance = ncross.norm () * seed_resolution_;
    float dot_p_1 = vec_t_to_s.dot (source_normal);
    float dot_p_2 = vec_s_to_t.dot (target_normal);
    float point_dist = (std::fabs (dot_p_1) < std::fabs (dot_p_2)) ? std::fabs (dot_p_1) : std::fabs (dot_p_2);
    const float dist_smoothing = smoothness_threshold_ * voxel_resolution_;  // This is a slacking variable especially important for patches with very similar normals

    if (point_dist > (expected_distance + dist_smoothing))
    {
      is_smooth &= false;
    }
  }
  // ----------------

  // Sanity Criterion: Check if definition convexity/concavity makes sense for connection of given patches
  float intersection_angle =  getAngle3D (ncross, vec_t_to_s, true);
  float min_intersect_angle = (intersection_angle < 90.) ? intersection_angle : 180. - intersection_angle;

  float intersect_thresh = 60. * 1. / (1. + std::exp (-0.25 * (normal_angle - 25.)));
  if (min_intersect_angle < intersect_thresh && use_sanity_check_)
  {
    // std::cout << "Concave/Convex not defined for given case!" << std::endl;
    is_convex &= false;
  }


  // vec_t_to_s is the reference direction for angle measurements
  // Convexity Criterion: Check if connection of patches is convex. If this is the case the two supervoxels should be merged.
  if ((getAngle3D (vec_t_to_s, source_normal) - getAngle3D (vec_t_to_s, target_normal)) <= 0)
  {
    is_convex &= true;  // connection convex
  }
  else
  {
    is_convex &= (normal_angle < concavity_tolerance_threshold_);  // concave connections will be accepted  if difference of normals is small
  }
  return (is_convex && is_smooth);
}

#endif // PCL_SEGMENTATION_IMPL_LCCP_SEGMENTATION_HPP_

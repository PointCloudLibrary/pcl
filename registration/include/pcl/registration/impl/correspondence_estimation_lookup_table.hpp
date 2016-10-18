/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_LOOKUP_TABLE_H_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_LOOKUP_TABLE_H_

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::registration::CorrespondenceLookupTable<PointT>::computeLookupTableBounds (const pcl::PointCloud<PointT>& pointcloud)
{
  if (pointcloud.size () < 2)
    return false;

  pcl::getMinMax3D (pointcloud, minimum_bounds_, maximum_bounds_);
  for (size_t i = 0; i < 3; ++i) {
    minimum_bounds_ (i) -= lookup_table_margin_ (i);
    maximum_bounds_ (i) += lookup_table_margin_ (i);
  }

  number_cells_x_ = std::max ((size_t)(((maximum_bounds_ (0) - minimum_bounds_ (0)) * cell_resolution_inverse_) + 0.5), (size_t)1);
  number_cells_y_ = std::max ((size_t)(((maximum_bounds_ (1) - minimum_bounds_ (1)) * cell_resolution_inverse_) + 0.5), (size_t)1);
  number_cells_z_ = std::max ((size_t)(((maximum_bounds_ (2) - minimum_bounds_ (2)) * cell_resolution_inverse_) + 0.5), (size_t)1);

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::registration::CorrespondenceLookupTable<PointT>::initLookupTable (const typename pcl::search::Search<PointT>::Ptr& tree)
{
  if (!computeLookupTableBounds (*(tree->getInputCloud ())))
    return false;

  search_tree_ = tree;
  size_t number_cells = number_cells_z_ * number_cells_y_ * number_cells_x_;
  lookup_table_.resize (number_cells);

  std::vector<int> index (1);
  std::vector<float> distance (1);
  int correspondence_number = 0;
  PointT query_point;
  query_point.z = minimum_bounds_(2);

  for (size_t z = 0; z < number_cells_z_; ++z) {
    query_point.y = minimum_bounds_(1);
    for (size_t y = 0; y < number_cells_y_; ++y) {
      query_point.x = minimum_bounds_(0);
      for (size_t x = 0; x < number_cells_x_; ++x) {
        search_tree_->nearestKSearch (query_point, 1, index, distance);
        pcl::registration::CorrespondenceLookupTableCell& correspondence = lookup_table_[correspondence_number++];
        correspondence.closest_point_index = index[0];
        correspondence.distance_squared_to_closest_point = distance[0];
        query_point.x += cell_resolution_;
      }
      query_point.y += cell_resolution_;
    }
    query_point.z += cell_resolution_;
  }

  PCL_DEBUG ("[pcl::registration::LookupTable::initLookupTable] Initialized correspondence estimation LookupTable with %f resolution containing [x:%u|y:%u|z:%u]=%u cells from a point cloud with %u points\n",
             cell_resolution_, number_cells_x_, number_cells_y_, number_cells_z_, number_cells, tree->getInputCloud ()->size ());
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::registration::CorrespondenceLookupTable<PointT>::getCorrespondence (const PointT& query_point, double maximum_correspondence_distance_squared, pcl::registration::CorrespondenceLookupTableCell& correspondance)
{
  int x_index = (int)(((query_point.x - minimum_bounds_ (0)) * cell_resolution_inverse_) + 0.5);
  int y_index = (int)(((query_point.y - minimum_bounds_ (1)) * cell_resolution_inverse_) + 0.5);
  int z_index = (int)(((query_point.z - minimum_bounds_ (2)) * cell_resolution_inverse_) + 0.5);

  size_t correspondence_index = x_index + y_index * number_cells_x_ + z_index * number_cells_x_ * number_cells_y_;
  if (x_index >= 0 && x_index < (int)number_cells_x_ &&
      y_index >= 0 && y_index < (int)number_cells_y_ &&
      z_index >= 0 && z_index < (int)number_cells_z_ &&
      correspondence_index < lookup_table_.size())
  {
    correspondance = lookup_table_[correspondence_index];
    ++number_of_queries_on_lookup_table_;
  }
  else if (search_tree_)
  {
    std::vector<int> index (1);
    std::vector<float> distance (1);
    search_tree_->nearestKSearch (query_point, 1, index, distance);
    correspondance.closest_point_index = index[0];
    correspondance.distance_squared_to_closest_point = distance[0];
    ++number_of_queries_on_search_tree_;
  }
  else
  	return false;

  if (correspondance.distance_squared_to_closest_point <= maximum_correspondence_distance_squared)
    return true;
  else
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl::registration::CorrespondenceEstimationLookupTable<PointSource, PointTarget, Scalar>::initComputeReciprocal ()
{
  if (CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal ())
  {
    if (!force_no_recompute_reciprocal_)
      return source_correspondences_lookup_table_.initLookupTable(tree_reciprocal_);
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl::registration::CorrespondenceEstimationLookupTable<PointSource, PointTarget, Scalar>::initCompute ()
{
  if (CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute ())
  {
    if (!force_no_recompute_)
      return target_correspondences_lookup_table_.initLookupTable(tree_);
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimationLookupTable<PointSource, PointTarget, Scalar>::determineCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
  if (!initCompute ())
    return;

  double max_distance_squared = max_distance * max_distance;
  correspondences.resize (indices_->size ());
  pcl::registration::CorrespondenceLookupTableCell correspondence_cell;
  size_t number_valid_correspondences = 0;

  if (pcl::isSamePointType<PointSource, PointTarget> ())
  {
    for (std::vector<int>::const_iterator input_index = indices_->begin (); input_index != indices_->end (); ++input_index)
    {
      if (target_correspondences_lookup_table_.getCorrespondence(input_->points[*input_index], max_distance_squared, correspondence_cell))
      {
        pcl::Correspondence& correspondence = correspondences[number_valid_correspondences++];
        correspondence.index_query = *input_index;
        correspondence.index_match = correspondence_cell.closest_point_index;
        correspondence.distance = correspondence_cell.distance_squared_to_closest_point;
      }
    }
  }
  else
  {
    PointTarget pt;
    for (std::vector<int>::const_iterator input_index = indices_->begin (); input_index != indices_->end (); ++input_index)
    {
      copyPoint (input_->points[*input_index], pt);
      if (target_correspondences_lookup_table_.getCorrespondence(input_->points[*input_index], max_distance_squared, correspondence_cell))
      {
      	pcl::Correspondence& correspondence = correspondences[number_valid_correspondences++];
        correspondence.index_query = *input_index;
        correspondence.index_match = correspondence_cell.closest_point_index;
        correspondence.distance = correspondence_cell.distance_squared_to_closest_point;
      }
    }
  }
  correspondences.resize (number_valid_correspondences);
  deinitCompute ();

  PCL_DEBUG ("[pcl::registration::CorrespondenceEstimationLookupTable::determineCorrespondences] Computed %u correspondences (%u using the LookupTable and %u using the search tree)\n",
             number_valid_correspondences, target_correspondences_lookup_table_.getNumberOfQueriesOnLookupTable(), target_correspondences_lookup_table_.getNumberOfQueriesOnSearchTree());
  target_correspondences_lookup_table_.resetNumberOfQueriesOnLookupTable();
  target_correspondences_lookup_table_.resetNumberOfQueriesOnSearchTree();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimationLookupTable<PointSource, PointTarget, Scalar>::determineReciprocalCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
  if (!initCompute ())
    return;

  if (!initComputeReciprocal())
    return;

  double max_distance_squared = max_distance * max_distance;
  correspondences.resize (indices_->size());
  pcl::registration::CorrespondenceLookupTableCell correspondence_cell;
  pcl::registration::CorrespondenceLookupTableCell reciprocal_correspondence_cell;
  unsigned int number_valid_correspondences = 0;

  if (pcl::isSamePointType<PointSource, PointTarget> ())
  {
    for (std::vector<int>::const_iterator input_index = indices_->begin (); input_index != indices_->end (); ++input_index)
    {
      if (target_correspondences_lookup_table_.getCorrespondence(input_->points[*input_index], max_distance_squared, correspondence_cell) &&
          source_correspondences_lookup_table_.getCorrespondence(target_->points[correspondence_cell.closest_point_index], max_distance_squared, reciprocal_correspondence_cell) &&
          *input_index == reciprocal_correspondence_cell.closest_point_index)
      {
      	pcl::Correspondence& correspondence = correspondences[number_valid_correspondences++];
        correspondence.index_query = *input_index;
        correspondence.index_match = correspondence_cell.closest_point_index;
        correspondence.distance = correspondence_cell.distance_squared_to_closest_point;
      }
    }
  }
  else
  {
    PointTarget pt_src;
    for (std::vector<int>::const_iterator input_index = indices_->begin (); input_index != indices_->end (); ++input_index)
    {
      pcl::copyPoint (input_->points[*input_index], pt_src);
      if (target_correspondences_lookup_table_.getCorrespondence(input_->points[*input_index], max_distance_squared, correspondence_cell))
      {
        PointSource pt_tgt;
        pcl::copyPoint (target_->points[correspondence_cell.closest_point_index], pt_tgt);
        if (source_correspondences_lookup_table_.getCorrespondence(target_->points[correspondence_cell.closest_point_index], max_distance_squared, reciprocal_correspondence_cell) &&
            *input_index == reciprocal_correspondence_cell.closest_point_index)
        {
         pcl::Correspondence& correspondence = correspondences[number_valid_correspondences++];
          correspondence.index_query = *input_index;
          correspondence.index_match = correspondence_cell.closest_point_index;
          correspondence.distance = correspondence_cell.distance_squared_to_closest_point;
        }
      }
    }
  }
  correspondences.resize (number_valid_correspondences);
  deinitCompute ();

  PCL_DEBUG ("[pcl::registration::CorrespondenceEstimationLookupTable::determineReciprocalCorrespondences] Computed %u correspondences (%u using the LookupTable, %u using the search tree, %u using the reciprocal LookupTable and %u using the reciprocal search tree)\n",
               number_valid_correspondences, target_correspondences_lookup_table_.getNumberOfQueriesOnLookupTable(), target_correspondences_lookup_table_.getNumberOfQueriesOnSearchTree(),
               source_correspondences_lookup_table_.getNumberOfQueriesOnLookupTable(), source_correspondences_lookup_table_.getNumberOfQueriesOnSearchTree());
  target_correspondences_lookup_table_.resetNumberOfQueriesOnLookupTable();
  target_correspondences_lookup_table_.resetNumberOfQueriesOnSearchTree();
  source_correspondences_lookup_table_.resetNumberOfQueriesOnLookupTable();
  source_correspondences_lookup_table_.resetNumberOfQueriesOnSearchTree();
}

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_LOOKUP_TABLE_H_ */

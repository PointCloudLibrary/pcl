/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_SURFACE_IMPL_GP3_H_
#define PCL_SURFACE_IMPL_GP3_H_

#include <pcl/surface/gp3.h>

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::GreedyProjectionTriangulation<PointInT>::performReconstruction (pcl::PolygonMesh &output)
{
  output.polygons.clear ();
  output.polygons.reserve (2 * indices_->size ()); /// NOTE: usually the number of triangles is around twice the number of vertices
 if (!reconstructPolygons (output.polygons))
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Reconstruction failed. Check parameters: search radius (%f) or mu (%f) before continuing.\n", getClassName ().c_str (), search_radius_, mu_);
    output.cloud.width = output.cloud.height = 0;
    output.cloud.data.clear ();
    return;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::GreedyProjectionTriangulation<PointInT>::performReconstruction (std::vector<pcl::Vertices> &polygons)
{
  polygons.clear ();
  polygons.reserve (2 * indices_->size ()); /// NOTE: usually the number of triangles is around twice the number of vertices
  if (!reconstructPolygons (polygons))
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Reconstruction failed. Check parameters: search radius (%f) or mu (%f) before continuing.\n", getClassName ().c_str (), search_radius_, mu_);
    return;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> bool
pcl::GreedyProjectionTriangulation<PointInT>::reconstructPolygons (std::vector<pcl::Vertices> &polygons)
{
  if (search_radius_ <= 0 || mu_ <= 0)
  {
    polygons.clear ();
    return (false);
  }
  const double sqr_mu = mu_*mu_;
  const double sqr_max_edge = search_radius_*search_radius_;
  if (nnn_ > static_cast<int> (indices_->size ()))
    nnn_ = static_cast<int> (indices_->size ());

  // Variables to hold the results of nearest neighbor searches
  pcl::Indices nnIdx (nnn_);
  std::vector<float> sqrDists (nnn_);

  // current number of connected components
  int part_index = 0;

  // 2D coordinates of points
  const Eigen::Vector2f uvn_nn_qp_zero = Eigen::Vector2f::Zero();
  Eigen::Vector2f uvn_current;
  Eigen::Vector2f uvn_prev;
  Eigen::Vector2f uvn_next;

  // initializing fields
  already_connected_ = false; // see declaration for comments :P

  // initializing states and fringe neighbors
  part_.clear ();
  state_.clear ();
  source_.clear ();
  ffn_.clear ();
  sfn_.clear ();
  part_.resize(indices_->size (), -1); // indices of point's part
  state_.resize(indices_->size (), FREE);
  source_.resize(indices_->size (), NONE);
  ffn_.resize(indices_->size (), NONE);
  sfn_.resize(indices_->size (), NONE);
  fringe_queue_.clear ();
  int fqIdx = 0; // current fringe's index in the queue to be processed

  // Avoiding NaN coordinates if needed
  if (!input_->is_dense)
  {
    // Skip invalid points from the indices list
    for (const auto& idx : (*indices_))
      if (!std::isfinite ((*input_)[idx].x) ||
          !std::isfinite ((*input_)[idx].y) ||
          !std::isfinite ((*input_)[idx].z))
        state_[idx] = NONE;
  }

  // Saving coordinates and point to index mapping
  coords_.clear ();
  coords_.reserve (indices_->size ());
  std::vector<int> point2index (input_->size (), -1);
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    coords_.push_back((*input_)[(*indices_)[cp]].getVector3fMap());
    point2index[(*indices_)[cp]] = cp;
  }

  // Initializing
  int is_free=0, nr_parts=0, increase_nnn4fn=0, increase_nnn4s=0, increase_dist=0, nr_touched = 0;
  angles_.resize(nnn_);
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > uvn_nn (nnn_);
  Eigen::Vector2f uvn_s;

  // iterating through fringe points and finishing them until everything is done
  while (is_free != NONE)
  {
    R_ = is_free;
    if (state_[R_] == FREE)
    {
      state_[R_] = NONE;
      part_[R_] = part_index++;

      // creating starting triangle
      //searchForNeighbors ((*indices_)[R_], nnIdx, sqrDists);
      //tree_->nearestKSearch ((*input_)[(*indices_)[R_]], nnn_, nnIdx, sqrDists);
      tree_->nearestKSearch (indices_->at (R_), nnn_, nnIdx, sqrDists);
      double sqr_dist_threshold = (std::min)(sqr_max_edge, sqr_mu * sqrDists[1]);

      // Search tree returns indices into the original cloud, but we are working with indices. TODO: make that optional!
      for (int i = 1; i < nnn_; i++)
      {
        //if (point2index[nnIdx[i]] == -1)
        //  std::cerr << R_ << " [" << indices_->at (R_) << "] " << i << ": " << nnIdx[i] << " / " << point2index[nnIdx[i]] << std::endl;
        nnIdx[i] = point2index[nnIdx[i]];
      }

      // Get the normal estimate at the current point 
      const Eigen::Vector3f nc = (*input_)[(*indices_)[R_]].getNormalVector3fMap ();

      // Get a coordinate system that lies on a plane defined by its normal
      v_ = nc.unitOrthogonal ();
      u_ = nc.cross (v_);

      // Projecting point onto the surface 
      float dist = nc.dot (coords_[R_]);
      proj_qp_ = coords_[R_] - dist * nc;

      // Converting coords, calculating angles and saving the projected near boundary edges
      int nr_edge = 0;
      std::vector<doubleEdge> doubleEdges;
      for (int i = 1; i < nnn_; i++) // nearest neighbor with index 0 is the query point R_ itself
      {
        // Transforming coordinates
        tmp_ = coords_[nnIdx[i]] - proj_qp_;
        uvn_nn[i][0] = tmp_.dot(u_);
        uvn_nn[i][1] = tmp_.dot(v_);
        // Computing the angle between each neighboring point and the query point itself
        angles_[i].angle = std::atan2(uvn_nn[i][1], uvn_nn[i][0]);
        // initializing angle descriptors
        angles_[i].index = nnIdx[i];
        if (
            (state_[nnIdx[i]] == COMPLETED) || (state_[nnIdx[i]] == BOUNDARY)
            || (state_[nnIdx[i]] == NONE) || (nnIdx[i] == UNAVAILABLE) /// NOTE: discarding NaN points and those that are not in indices_
            || (sqrDists[i] > sqr_dist_threshold)
           )
          angles_[i].visible = false;
        else
          angles_[i].visible = true;
        // Saving the edges between nearby boundary points
        if ((state_[nnIdx[i]] == FRINGE) || (state_[nnIdx[i]] == BOUNDARY))
        {
          doubleEdge e;
          e.index = i;
          nr_edge++;
          tmp_ = coords_[ffn_[nnIdx[i]]] - proj_qp_;
          e.first[0] = tmp_.dot(u_);
          e.first[1] = tmp_.dot(v_);
          tmp_ = coords_[sfn_[nnIdx[i]]] - proj_qp_;
          e.second[0] = tmp_.dot(u_);
          e.second[1] = tmp_.dot(v_);
          doubleEdges.push_back(e);
        }
      }
      angles_[0].visible = false;

      // Verify the visibility of each potential new vertex 
      for (int i = 1; i < nnn_; i++) // nearest neighbor with index 0 is the query point R_ itself
        if ((angles_[i].visible) && (ffn_[R_] != nnIdx[i]) && (sfn_[R_] != nnIdx[i]))
        {
          bool visibility = true;
          for (int j = 0; j < nr_edge; j++)
          {
            if (ffn_[nnIdx[doubleEdges[j].index]] != nnIdx[i])
              visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index], doubleEdges[j].first, Eigen::Vector2f::Zero());
            if (!visibility)
              break;
            if (sfn_[nnIdx[doubleEdges[j].index]] != nnIdx[i])
              visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index], doubleEdges[j].second, Eigen::Vector2f::Zero());
            if (!visibility)
              break;
          }
          angles_[i].visible = visibility;
        }

      // Selecting first two visible free neighbors
      bool not_found = true;
      int left = 1;
      do
      {
        while ((left < nnn_) && ((!angles_[left].visible) || (state_[nnIdx[left]] > FREE))) left++;
        if (left >= nnn_)
          break;
        int right = left+1;
        do
        {
          while ((right < nnn_) && ((!angles_[right].visible) || (state_[nnIdx[right]] > FREE))) right++;
          if (right >= nnn_)
            break;
          if ((coords_[nnIdx[left]] - coords_[nnIdx[right]]).squaredNorm () > sqr_max_edge)
            right++;
          else
          {
            addFringePoint (nnIdx[right], R_);
            addFringePoint (nnIdx[left], nnIdx[right]);
            addFringePoint (R_, nnIdx[left]);
            state_[R_] = state_[nnIdx[left]] = state_[nnIdx[right]] = FRINGE;
            ffn_[R_] = nnIdx[left];
            sfn_[R_] = nnIdx[right];
            ffn_[nnIdx[left]] = nnIdx[right];
            sfn_[nnIdx[left]] = R_;
            ffn_[nnIdx[right]] = R_;
            sfn_[nnIdx[right]] = nnIdx[left];
            addTriangle (R_, nnIdx[left], nnIdx[right], polygons);
            nr_parts++;
            not_found = false;
            break;
          }
        }
        while (true);
        left++;
      }
      while (not_found);
    }

    is_free = NONE;
    for (std::size_t temp = 0; temp < indices_->size (); temp++)
    {
      if (state_[temp] == FREE)
      {
        is_free = temp;
        break;
      }
    }

    bool is_fringe = true;
    while (is_fringe)
    {
      is_fringe = false;

      int fqSize = static_cast<int> (fringe_queue_.size ());
      while ((fqIdx < fqSize) && (state_[fringe_queue_[fqIdx]] != FRINGE))
        fqIdx++;

      // an unfinished fringe point is found
      if (fqIdx >= fqSize)
      {
        continue;
      }

      R_ = fringe_queue_[fqIdx];
      is_fringe = true;

      if (ffn_[R_] == sfn_[R_])
      {
        state_[R_] = COMPLETED;
        continue;
      }
      //searchForNeighbors ((*indices_)[R_], nnIdx, sqrDists);
      //tree_->nearestKSearch ((*input_)[(*indices_)[R_]], nnn_, nnIdx, sqrDists);
      tree_->nearestKSearch (indices_->at (R_), nnn_, nnIdx, sqrDists);

      // Search tree returns indices into the original cloud, but we are working with indices TODO: make that optional!
      for (int i = 1; i < nnn_; i++)
      {
        //if (point2index[nnIdx[i]] == -1)
        //  std::cerr << R_ << " [" << indices_->at (R_) << "] " << i << ": " << nnIdx[i] << " / " << point2index[nnIdx[i]] << std::endl;
        nnIdx[i] = point2index[nnIdx[i]];
      }

      // Locating FFN and SFN to adapt distance threshold
      double sqr_source_dist = (coords_[R_] - coords_[source_[R_]]).squaredNorm ();
      double sqr_ffn_dist = (coords_[R_] - coords_[ffn_[R_]]).squaredNorm ();
      double sqr_sfn_dist = (coords_[R_] - coords_[sfn_[R_]]).squaredNorm ();
      double max_sqr_fn_dist = (std::max)(sqr_ffn_dist, sqr_sfn_dist);
      double sqr_dist_threshold = (std::min)(sqr_max_edge, sqr_mu * sqrDists[1]); //sqr_mu * sqr_avg_conn_dist);
      if (max_sqr_fn_dist > sqrDists[nnn_-1])
      {
        if (0 == increase_nnn4fn)
          PCL_WARN("Not enough neighbors are considered: ffn or sfn out of range! Consider increasing nnn_... Setting R=%d to be BOUNDARY!\n", R_);
        increase_nnn4fn++;
        state_[R_] = BOUNDARY;
        continue;
      }
      double max_sqr_fns_dist = (std::max)(sqr_source_dist, max_sqr_fn_dist);
      if (max_sqr_fns_dist > sqrDists[nnn_-1])
      {
        if (0 == increase_nnn4s)
          PCL_WARN("Not enough neighbors are considered: source of R=%d is out of range! Consider increasing nnn_...\n", R_);
        increase_nnn4s++;
      }

      // Get the normal estimate at the current point 
      const Eigen::Vector3f nc = (*input_)[(*indices_)[R_]].getNormalVector3fMap ();

      // Get a coordinate system that lies on a plane defined by its normal
      v_ = nc.unitOrthogonal ();
      u_ = nc.cross (v_);

      // Projecting point onto the surface
      float dist = nc.dot (coords_[R_]);
      proj_qp_ = coords_[R_] - dist * nc;

      // Converting coords, calculating angles and saving the projected near boundary edges
      int nr_edge = 0;
      std::vector<doubleEdge> doubleEdges;
      for (int i = 1; i < nnn_; i++) // nearest neighbor with index 0 is the query point R_ itself
      {
        tmp_ = coords_[nnIdx[i]] - proj_qp_;
        uvn_nn[i][0] = tmp_.dot(u_);
        uvn_nn[i][1] = tmp_.dot(v_);
  
        // Computing the angle between each neighboring point and the query point itself 
        angles_[i].angle = std::atan2(uvn_nn[i][1], uvn_nn[i][0]);
        // initializing angle descriptors
        angles_[i].index = nnIdx[i];
        angles_[i].nnIndex = i;
        if (
            (state_[nnIdx[i]] == COMPLETED) || (state_[nnIdx[i]] == BOUNDARY)
            || (state_[nnIdx[i]] == NONE) || (nnIdx[i] == UNAVAILABLE) /// NOTE: discarding NaN points and those that are not in indices_
            || (sqrDists[i] > sqr_dist_threshold)
           )
          angles_[i].visible = false;
        else
          angles_[i].visible = true;
        if ((ffn_[R_] == nnIdx[i]) || (sfn_[R_] == nnIdx[i]))
          angles_[i].visible = true;
        bool same_side = true;
        const Eigen::Vector3f neighbor_normal = (*input_)[(*indices_)[nnIdx[i]]].getNormalVector3fMap (); /// NOTE: nnIdx was reset
        double cosine = nc.dot (neighbor_normal);
        if (cosine > 1) cosine = 1;
        if (cosine < -1) cosine = -1;
        double angle = std::acos (cosine);
        if ((!consistent_) && (angle > M_PI/2))
          angle = M_PI - angle;
        if (angle > eps_angle_)
        {
          angles_[i].visible = false;
          same_side = false;
        }
        // Saving the edges between nearby boundary points 
        if ((i!=0) && (same_side) && ((state_[nnIdx[i]] == FRINGE) || (state_[nnIdx[i]] == BOUNDARY)))
        {
          doubleEdge e;
          e.index = i;
          nr_edge++;
          tmp_ = coords_[ffn_[nnIdx[i]]] - proj_qp_;
          e.first[0] = tmp_.dot(u_);
          e.first[1] = tmp_.dot(v_);
          tmp_ = coords_[sfn_[nnIdx[i]]] - proj_qp_;
          e.second[0] = tmp_.dot(u_);
          e.second[1] = tmp_.dot(v_);
          doubleEdges.push_back(e);
          // Pruning by visibility criterion 
          if ((state_[nnIdx[i]] == FRINGE) && (ffn_[R_] != nnIdx[i]) && (sfn_[R_] != nnIdx[i]))
          {
            double angle1 = std::atan2(e.first[1] - uvn_nn[i][1], e.first[0] - uvn_nn[i][0]);
            double angle2 = std::atan2(e.second[1] - uvn_nn[i][1], e.second[0] - uvn_nn[i][0]);
            double angleMin, angleMax;
            if (angle1 < angle2)
            {
              angleMin = angle1;
              angleMax = angle2;
            }
            else
            {
              angleMin = angle2;
              angleMax = angle1;
            }
            double angleR = angles_[i].angle + M_PI;
            if (angleR >= M_PI)
              angleR -= 2*M_PI;
            if ((source_[nnIdx[i]] == ffn_[nnIdx[i]]) || (source_[nnIdx[i]] == sfn_[nnIdx[i]]))
            {
              if ((angleMax - angleMin) < M_PI)
              {
                if ((angleMin < angleR) && (angleR < angleMax))
                  angles_[i].visible = false;
              }
              else
              {
                if ((angleR < angleMin) || (angleMax < angleR))
                  angles_[i].visible = false;
              }
            }
            else
            {
              tmp_ = coords_[source_[nnIdx[i]]] - proj_qp_;
              uvn_s[0] = tmp_.dot(u_);
              uvn_s[1] = tmp_.dot(v_);
              double angleS = std::atan2(uvn_s[1] - uvn_nn[i][1], uvn_s[0] - uvn_nn[i][0]);
              if ((angleMin < angleS) && (angleS < angleMax))
              {
                if ((angleMin < angleR) && (angleR < angleMax))
                  angles_[i].visible = false;
              }
              else
              {
                if ((angleR < angleMin) || (angleMax < angleR))
                  angles_[i].visible = false;
              }
            }
          }
        }
      }
      angles_[0].visible = false;

      // Verify the visibility of each potential new vertex
      for (int i = 1; i < nnn_; i++) // nearest neighbor with index 0 is the query point R_ itself
        if ((angles_[i].visible) && (ffn_[R_] != nnIdx[i]) && (sfn_[R_] != nnIdx[i]))
        {
          bool visibility = true;
          for (int j = 0; j < nr_edge; j++)
          {
            if (doubleEdges[j].index != i)
            {
              const auto& f = ffn_[nnIdx[doubleEdges[j].index]];
              if ((f != nnIdx[i]) && (f != R_))
                visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index], doubleEdges[j].first, Eigen::Vector2f::Zero());
              if (!visibility)
                break;

              const auto& s = sfn_[nnIdx[doubleEdges[j].index]];
              if ((s != nnIdx[i]) && (s != R_))
                visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index], doubleEdges[j].second, Eigen::Vector2f::Zero());
              if (!visibility)
                break;
            }
          }
          angles_[i].visible = visibility;
        }

      // Sorting angles
      std::sort (angles_.begin (), angles_.end (), GreedyProjectionTriangulation<PointInT>::nnAngleSortAsc);

      // Triangulating
      if (angles_[2].visible == false)
      {
        if ( !( (angles_[0].index == ffn_[R_] && angles_[1].index == sfn_[R_]) || (angles_[0].index == sfn_[R_] && angles_[1].index == ffn_[R_]) ) )
        {
          state_[R_] = BOUNDARY;
        }
        else
        {
          if ((source_[R_] == angles_[0].index) || (source_[R_] == angles_[1].index))
            state_[R_] = BOUNDARY;
          else
          {
            if (sqr_max_edge < (coords_[ffn_[R_]] - coords_[sfn_[R_]]).squaredNorm ())
            {
              state_[R_] = BOUNDARY;
            }
            else
            {
              tmp_ = coords_[source_[R_]] - proj_qp_;
              uvn_s[0] = tmp_.dot(u_);
              uvn_s[1] = tmp_.dot(v_);
              double angleS = std::atan2(uvn_s[1], uvn_s[0]);
              double dif = angles_[1].angle - angles_[0].angle;
              if ((angles_[0].angle < angleS) && (angleS < angles_[1].angle))
              {
                if (dif < 2*M_PI - maximum_angle_)
                  state_[R_] = BOUNDARY;
                else
                  closeTriangle (polygons);
              }
              else
              {
                if (dif >= maximum_angle_)
                  state_[R_] = BOUNDARY;
                else
                  closeTriangle (polygons);
              }
            }
          }
        }
        continue;
      }

      // Finding the FFN and SFN
      int start = -1, end = -1;
      for (int i=0; i<nnn_; i++)
      {
        if (ffn_[R_] == angles_[i].index)
        {
          start = i;
          if (sfn_[R_] == angles_[i+1].index)
            end = i+1;
          else
            if (i==0)
            {
              for (i = i+2; i < nnn_; i++)
                if (sfn_[R_] == angles_[i].index)
                  break;
              end = i;
            }
            else
            {
              for (i = i+2; i < nnn_; i++)
                if (sfn_[R_] == angles_[i].index)
                  break;
              end = i;
            }
          break;
        }
        if (sfn_[R_] == angles_[i].index)
        {
          start = i;
          if (ffn_[R_] == angles_[i+1].index)
            end = i+1;
          else
            if (i==0)
            {
              for (i = i+2; i < nnn_; i++)
                if (ffn_[R_] == angles_[i].index)
                  break;
              end = i;
            }
            else
            {
              for (i = i+2; i < nnn_; i++)
                if (ffn_[R_] == angles_[i].index)
                  break;
              end = i;
            }
          break;
        }
      }

      // start and end are always set, as we checked if ffn or sfn are out of range before, but let's check anyways if < 0
      if ((start < 0) || (end < 0) || (end == nnn_) || (!angles_[start].visible) || (!angles_[end].visible))
      {
        state_[R_] = BOUNDARY;
        continue;
      }

      // Finding last visible nn 
      int last_visible = end;
      while ((last_visible+1<nnn_) && (angles_[last_visible+1].visible)) last_visible++;

      // Finding visibility region of R
      bool need_invert = false;
      if ((source_[R_] == ffn_[R_]) || (source_[R_] == sfn_[R_]))
      {
        if ((angles_[end].angle - angles_[start].angle) < M_PI)
          need_invert = true;
      }
      else
      {
        int sourceIdx;
        for (sourceIdx=0; sourceIdx<nnn_; sourceIdx++)
          if (angles_[sourceIdx].index == source_[R_])
            break;
        if (sourceIdx == nnn_)
        {
          int vis_free = NONE, nnCB = NONE; // any free visible and nearest completed or boundary neighbor of R
          for (int i = 1; i < nnn_; i++) // nearest neighbor with index 0 is the query point R_ itself
          {
            // NOTE: nnCB is an index in nnIdx
            if ((state_[nnIdx[i]] == COMPLETED) || (state_[nnIdx[i]] == BOUNDARY))
            {
              if (nnCB == NONE)
              {
                nnCB = i;
                if (vis_free != NONE)
                  break;
              }
            }
            // NOTE: vis_free is an index in angles
            if (state_[angles_[i].index] <= FREE)
            {
              if (i <= last_visible)
              {
                vis_free = i;
                if (nnCB != NONE)
                  break;
              }
            }
          }
          // NOTE: nCB is an index in angles
          int nCB = 0;
          if (nnCB != NONE)
            while (angles_[nCB].index != nnIdx[nnCB]) nCB++;
          else
            nCB = NONE;

          if (vis_free != NONE)
          {
            if ((vis_free < start) || (vis_free > end))
              need_invert = true;
          }
          else
          {
            if (nCB != NONE)
            {
              if ((nCB == start) || (nCB == end))
              {
                bool inside_CB = false;
                bool outside_CB = false;
                for (int i=0; i<nnn_; i++)
                {
                  if (
                      ((state_[angles_[i].index] == COMPLETED) || (state_[angles_[i].index] == BOUNDARY))
                      && (i != start) && (i != end)
                     )
                    {
                      if ((angles_[start].angle <= angles_[i].angle) && (angles_[i].angle <= angles_[end].angle))
                      {
                        inside_CB = true;
                        if (outside_CB)
                          break;
                      }
                      else
                      {
                        outside_CB = true;
                        if (inside_CB)
                          break;
                      }
                    }
                }
                if (inside_CB && !outside_CB)
                  need_invert = true;
                else if (!(!inside_CB && outside_CB))
                {
                  if ((angles_[end].angle - angles_[start].angle) < M_PI)
                    need_invert = true;
                }
              }
              else
              {
                if ((angles_[nCB].angle > angles_[start].angle) && (angles_[nCB].angle < angles_[end].angle))
                  need_invert = true;
              }
            }
            else
            {
              if (start == end-1)
                need_invert = true;
            }
          }
        }
        else if ((angles_[start].angle < angles_[sourceIdx].angle) && (angles_[sourceIdx].angle < angles_[end].angle))
          need_invert = true;
      }

      // switching start and end if necessary
      if (need_invert)
      {
        int tmp = start;
        start = end;
        end = tmp;
      }

      // Arranging visible nnAngles in the order they need to be connected and
      // compute the maximal angle difference between two consecutive visible angles
      bool is_boundary = false, is_skinny = false;
      std::vector<bool> gaps (nnn_, false);
      std::vector<bool> skinny (nnn_, false);
      std::vector<double> dif (nnn_);
      std::vector<int> angleIdx; angleIdx.reserve (nnn_);
      if (start > end)
      {
        for (int j=start; j<last_visible; j++)
        {
          dif[j] = (angles_[j+1].angle - angles_[j].angle);
          if (dif[j] < minimum_angle_)
          {
            skinny[j] = is_skinny = true;
          }
          else if (maximum_angle_ <= dif[j])
          {
            gaps[j] = is_boundary = true;
          }
          if ((!gaps[j]) && (sqr_max_edge < (coords_[angles_[j+1].index] - coords_[angles_[j].index]).squaredNorm ()))
          {
            gaps[j] = is_boundary = true;
          }
          angleIdx.push_back(j);
        }

        dif[last_visible] = (2*M_PI + angles_[0].angle - angles_[last_visible].angle);
        if (dif[last_visible] < minimum_angle_)
        {
          skinny[last_visible] = is_skinny = true;
        }
        else if (maximum_angle_ <= dif[last_visible])
        {
          gaps[last_visible] = is_boundary = true;
        }
        if ((!gaps[last_visible]) && (sqr_max_edge < (coords_[angles_[0].index] - coords_[angles_[last_visible].index]).squaredNorm ()))
        {
          gaps[last_visible] = is_boundary = true;
        }
        angleIdx.push_back(last_visible);

        for (int j=0; j<end; j++)
        {
          dif[j] = (angles_[j+1].angle - angles_[j].angle);
          if (dif[j] < minimum_angle_)
          {
            skinny[j] = is_skinny = true;
          }
          else if (maximum_angle_ <= dif[j])
          {
            gaps[j] = is_boundary = true;
          }
          if ((!gaps[j]) && (sqr_max_edge < (coords_[angles_[j+1].index] - coords_[angles_[j].index]).squaredNorm ()))
          {
            gaps[j] = is_boundary = true;
          }
          angleIdx.push_back(j);
        }
        angleIdx.push_back(end);
      }
      // start < end
      else
      {
        for (int j=start; j<end; j++)
        {
          dif[j] = (angles_[j+1].angle - angles_[j].angle);
          if (dif[j] < minimum_angle_)
          {
            skinny[j] = is_skinny = true;
          }
          else if (maximum_angle_ <= dif[j])
          {
            gaps[j] = is_boundary = true;
          }
          if ((!gaps[j]) && (sqr_max_edge < (coords_[angles_[j+1].index] - coords_[angles_[j].index]).squaredNorm ()))
          {
            gaps[j] = is_boundary = true;
          }
          angleIdx.push_back(j);
        }
        angleIdx.push_back(end);
      }

      // Set the state of the point
      state_[R_] = is_boundary ? BOUNDARY : COMPLETED;

      auto first_gap_after = angleIdx.end ();
      auto last_gap_before = angleIdx.begin ();
      int nr_gaps = 0;
      for (auto it = angleIdx.begin (); it != angleIdx.end () - 1; ++it)
      {
        if (gaps[*it])
        {
          nr_gaps++;
          if (first_gap_after == angleIdx.end())
            first_gap_after = it;
          last_gap_before = it+1;
        }
      }
      if (nr_gaps > 1)
      {
        angleIdx.erase(first_gap_after+1, last_gap_before);
      }

      // Neglecting points that would form skinny triangles (if possible)
      if (is_skinny)
      {
        double angle_so_far = 0, angle_would_be;
        double max_combined_angle = (std::min)(maximum_angle_, M_PI-2*minimum_angle_);
        Eigen::Vector2f X;
        Eigen::Vector2f S1;
        Eigen::Vector2f S2;
        std::vector<int> to_erase;
        for (auto it = angleIdx.begin()+1; it != angleIdx.end()-1; ++it)
        {
          if (gaps[*(it-1)])
            angle_so_far = 0;
          else
            angle_so_far += dif[*(it-1)];
          if (gaps[*it])
            angle_would_be = angle_so_far;
          else
            angle_would_be = angle_so_far + dif[*it];
          if (
              (skinny[*it] || skinny[*(it-1)]) &&
              ((state_[angles_[*it].index] <= FREE) || (state_[angles_[*(it-1)].index] <= FREE)) &&
              ((!gaps[*it]) || (angles_[*it].nnIndex > angles_[*(it-1)].nnIndex)) &&
              ((!gaps[*(it-1)]) || (angles_[*it].nnIndex > angles_[*(it+1)].nnIndex)) &&
              (angle_would_be < max_combined_angle)
             )
          {
            if (gaps[*(it-1)])
            {
              gaps[*it] = true;
              to_erase.push_back(*it);
            }
            else if (gaps[*it])
            {
              gaps[*(it-1)] = true;
              to_erase.push_back(*it);
            }
            else
            {
              std::vector<int>::iterator prev_it;
              int erased_idx = static_cast<int> (to_erase.size ()) -1;
              for (prev_it = it-1; (erased_idx != -1) && (it != angleIdx.begin()); --it)
                if (*it == to_erase[erased_idx])
                  erased_idx--;
                else
                  break;
              bool can_delete = true;
              for (auto curr_it = prev_it+1; curr_it != it+1; ++curr_it)
              {
                tmp_ = coords_[angles_[*curr_it].index] - proj_qp_;
                X[0] = tmp_.dot(u_);
                X[1] = tmp_.dot(v_);
                tmp_ = coords_[angles_[*prev_it].index] - proj_qp_;
                S1[0] = tmp_.dot(u_);
                S1[1] = tmp_.dot(v_);
                tmp_ = coords_[angles_[*(it+1)].index] - proj_qp_;
                S2[0] = tmp_.dot(u_);
                S2[1] = tmp_.dot(v_);
                // check for inclusions 
                if (isVisible(X,S1,S2))
                {
                  can_delete = false;
                  angle_so_far = 0;
                  break;
                }
              }
              if (can_delete)
              {
                to_erase.push_back(*it);
              }
            }
          }
          else
            angle_so_far = 0;
        }
        for (const auto &idx : to_erase)
        {
          for (auto iter = angleIdx.begin(); iter != angleIdx.end(); ++iter)
            if (idx == *iter)
            {
              angleIdx.erase(iter);
              break;
            }
        }
      }

      // Writing edges and updating edge-front 
      changed_1st_fn_ = false;
      changed_2nd_fn_ = false;
      new2boundary_ = NONE;
      for (auto it = angleIdx.begin()+1; it != angleIdx.end()-1; ++it)
      {
        current_index_ = angles_[*it].index;

        is_current_free_ = false;
        if (state_[current_index_] <= FREE)
        {
          state_[current_index_] = FRINGE;
          is_current_free_ = true;
        }
        else if (!already_connected_)
        {
          prev_is_ffn_ = (ffn_[current_index_] == angles_[*(it-1)].index) && (!gaps[*(it-1)]);
          prev_is_sfn_ = (sfn_[current_index_] == angles_[*(it-1)].index) && (!gaps[*(it-1)]);
          next_is_ffn_ = (ffn_[current_index_] == angles_[*(it+1)].index) && (!gaps[*it]);
          next_is_sfn_ = (sfn_[current_index_] == angles_[*(it+1)].index) && (!gaps[*it]);
          if (!prev_is_ffn_ && !next_is_sfn_ && !prev_is_sfn_ && !next_is_ffn_)
          {
            nr_touched++;
          }
        }
                                   
        if (gaps[*it])
          if (gaps[*(it-1)])
          {
            if (is_current_free_)
              state_[current_index_] = NONE; /// TODO: document!
          }

          else // (gaps[*it]) && ^(gaps[*(it-1)])
          {
            addTriangle (current_index_, angles_[*(it-1)].index, R_, polygons);
            addFringePoint (current_index_, R_);
            new2boundary_ = current_index_;
            if (!already_connected_) 
              connectPoint (polygons, angles_[*(it-1)].index, R_,
                            angles_[*(it+1)].index,
                            uvn_nn[angles_[*it].nnIndex], uvn_nn[angles_[*(it-1)].nnIndex], uvn_nn_qp_zero);
            else already_connected_ = false;
            if (ffn_[R_] == angles_[*(angleIdx.begin())].index)
            {
              ffn_[R_] = new2boundary_;
            }
            else if (sfn_[R_] == angles_[*(angleIdx.begin())].index)
            {
              sfn_[R_] = new2boundary_;
            }
          }

        else // ^(gaps[*it])
          if (gaps[*(it-1)])
          {
            addFringePoint (current_index_, R_);
            new2boundary_ = current_index_;
            if (!already_connected_) connectPoint (polygons, R_, angles_[*(it+1)].index,
                                                   (it+2) == angleIdx.end() ? -1 : angles_[*(it+2)].index,
                                                   uvn_nn[angles_[*it].nnIndex], uvn_nn_qp_zero, 
                                                   uvn_nn[angles_[*(it+1)].nnIndex]);
            else already_connected_ = false;
            if (ffn_[R_] == angles_[*(angleIdx.end()-1)].index)
            {
              ffn_[R_] = new2boundary_;
            }
            else if (sfn_[R_] == angles_[*(angleIdx.end()-1)].index)
            {
              sfn_[R_] = new2boundary_;
            }
          }

          else // ^(gaps[*it]) && ^(gaps[*(it-1)]) 
          {
            addTriangle (current_index_, angles_[*(it-1)].index, R_, polygons);
            addFringePoint (current_index_, R_);
            if (!already_connected_) connectPoint (polygons, angles_[*(it-1)].index, angles_[*(it+1)].index,
                                                   (it+2) == angleIdx.end() ? -1 : gaps[*(it+1)] ? R_ : angles_[*(it+2)].index,
                                                   uvn_nn[angles_[*it].nnIndex], 
                                                   uvn_nn[angles_[*(it-1)].nnIndex], 
                                                   uvn_nn[angles_[*(it+1)].nnIndex]);
            else already_connected_ = false;
          }
      }
      
      // Finishing up R_
      if (ffn_[R_] == sfn_[R_])
      {
        state_[R_] = COMPLETED;
      }
      if (!gaps[*(angleIdx.end()-2)])
      {
        addTriangle (angles_[*(angleIdx.end()-2)].index, angles_[*(angleIdx.end()-1)].index, R_, polygons);
        addFringePoint (angles_[*(angleIdx.end()-2)].index, R_);
        if (R_ == ffn_[angles_[*(angleIdx.end()-1)].index])
        {
          if (angles_[*(angleIdx.end()-2)].index == sfn_[angles_[*(angleIdx.end()-1)].index])
          {
            state_[angles_[*(angleIdx.end()-1)].index] = COMPLETED;
          }
          else
          {
            ffn_[angles_[*(angleIdx.end()-1)].index] = angles_[*(angleIdx.end()-2)].index;
          }
        }
        else if (R_ == sfn_[angles_[*(angleIdx.end()-1)].index])
        {
          if (angles_[*(angleIdx.end()-2)].index == ffn_[angles_[*(angleIdx.end()-1)].index])
          {
            state_[angles_[*(angleIdx.end()-1)].index] = COMPLETED;
          }
          else
          {
            sfn_[angles_[*(angleIdx.end()-1)].index] = angles_[*(angleIdx.end()-2)].index;
          }
        }
      }
      if (!gaps[*(angleIdx.begin())])
      {
        if (R_ == ffn_[angles_[*(angleIdx.begin())].index])
        {
          if (angles_[*(angleIdx.begin()+1)].index == sfn_[angles_[*(angleIdx.begin())].index])
          {
            state_[angles_[*(angleIdx.begin())].index] = COMPLETED;
          }
          else
          {
            ffn_[angles_[*(angleIdx.begin())].index] = angles_[*(angleIdx.begin()+1)].index;
          }
        }
        else if (R_ == sfn_[angles_[*(angleIdx.begin())].index])
        {
          if (angles_[*(angleIdx.begin()+1)].index == ffn_[angles_[*(angleIdx.begin())].index])
          {
            state_[angles_[*(angleIdx.begin())].index] = COMPLETED;
          }
          else
          {
            sfn_[angles_[*(angleIdx.begin())].index] = angles_[*(angleIdx.begin()+1)].index;
          }
        }
      }
    }
  }
  PCL_DEBUG ("Number of triangles: %lu\n", polygons.size());
  PCL_DEBUG ("Number of unconnected parts: %d\n", nr_parts);
  if (increase_nnn4fn > 0)
    PCL_WARN ("Number of neighborhood size increase requests for fringe neighbors: %d\n", increase_nnn4fn);
  if (increase_nnn4s > 0)
    PCL_WARN ("Number of neighborhood size increase requests for source: %d\n", increase_nnn4s);
  if (increase_dist > 0)
    PCL_WARN ("Number of automatic maximum distance increases: %d\n", increase_dist);

  // sorting and removing doubles from fringe queue 
  std::sort (fringe_queue_.begin (), fringe_queue_.end ());
  fringe_queue_.erase (std::unique (fringe_queue_.begin (), fringe_queue_.end ()), fringe_queue_.end ());
  PCL_DEBUG ("Number of processed points: %lu / %lu\n", fringe_queue_.size(), indices_->size ());
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::GreedyProjectionTriangulation<PointInT>::closeTriangle (std::vector<pcl::Vertices> &polygons)
{
  state_[R_] = COMPLETED;
  addTriangle (angles_[0].index, angles_[1].index, R_, polygons);
  for (int aIdx=0; aIdx<2; aIdx++)
  {
    if (ffn_[angles_[aIdx].index] == R_)
    {
      if (sfn_[angles_[aIdx].index] == angles_[(aIdx+1)%2].index)
      {
        state_[angles_[aIdx].index] = COMPLETED;
      }
      else
      {
        ffn_[angles_[aIdx].index] = angles_[(aIdx+1)%2].index;
      }
    }
    else if (sfn_[angles_[aIdx].index] == R_)
    {
      if (ffn_[angles_[aIdx].index] == angles_[(aIdx+1)%2].index)
      {
        state_[angles_[aIdx].index] = COMPLETED;
      }
      else
      {
        sfn_[angles_[aIdx].index] = angles_[(aIdx+1)%2].index;
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::GreedyProjectionTriangulation<PointInT>::connectPoint (
    std::vector<pcl::Vertices> &polygons, 
    const pcl::index_t prev_index, const pcl::index_t next_index, const pcl::index_t next_next_index, 
    const Eigen::Vector2f &uvn_current, 
    const Eigen::Vector2f &uvn_prev, 
    const Eigen::Vector2f &uvn_next)
{
  if (is_current_free_)
  {
    ffn_[current_index_] = prev_index;
    sfn_[current_index_] = next_index;
  }
  else
  {
    if ((prev_is_ffn_ && next_is_sfn_) || (prev_is_sfn_ && next_is_ffn_))
      state_[current_index_] = COMPLETED;
    else if (prev_is_ffn_ && !next_is_sfn_)
      ffn_[current_index_] = next_index;
    else if (next_is_ffn_ && !prev_is_sfn_)
      ffn_[current_index_] = prev_index;
    else if (prev_is_sfn_ && !next_is_ffn_)
      sfn_[current_index_] = next_index;
    else if (next_is_sfn_ && !prev_is_ffn_)
      sfn_[current_index_] = prev_index;
    else
    {
      bool found_triangle = false;
      if ((prev_index != R_) && ((ffn_[current_index_] == ffn_[prev_index]) || (ffn_[current_index_] == sfn_[prev_index])))
      {
        found_triangle = true;
        addTriangle (current_index_, ffn_[current_index_], prev_index, polygons);
        state_[prev_index] = COMPLETED;
        state_[ffn_[current_index_]] = COMPLETED;
        ffn_[current_index_] = next_index;
      }
      else if ((prev_index != R_) && ((sfn_[current_index_] == ffn_[prev_index]) || (sfn_[current_index_] == sfn_[prev_index])))
      {
        found_triangle = true;
        addTriangle (current_index_, sfn_[current_index_], prev_index, polygons);
        state_[prev_index] = COMPLETED;
        state_[sfn_[current_index_]] = COMPLETED;
        sfn_[current_index_] = next_index;
      }
      else if (state_[next_index] > FREE)
      {
        if ((ffn_[current_index_] == ffn_[next_index]) || (ffn_[current_index_] == sfn_[next_index]))
        {
          found_triangle = true;
          addTriangle (current_index_, ffn_[current_index_], next_index, polygons);

          if (ffn_[current_index_] == ffn_[next_index])
          {
            ffn_[next_index] = current_index_;
          }
          else
          {
            sfn_[next_index] = current_index_;
          }
          state_[ffn_[current_index_]] = COMPLETED;
          ffn_[current_index_] = prev_index;
        }
        else if ((sfn_[current_index_] == ffn_[next_index]) || (sfn_[current_index_] == sfn_[next_index]))
        {
          found_triangle = true;
          addTriangle (current_index_, sfn_[current_index_], next_index, polygons);

          if (sfn_[current_index_] == ffn_[next_index])
          {
            ffn_[next_index] = current_index_;
          }
          else
          {
            sfn_[next_index] = current_index_;
          }
          state_[sfn_[current_index_]] = COMPLETED;
          sfn_[current_index_] = prev_index;
        }
      }

      if (found_triangle)
      {
      }
      else
      {
        tmp_ = coords_[ffn_[current_index_]] - proj_qp_;
        uvn_ffn_[0] = tmp_.dot(u_);
        uvn_ffn_[1] = tmp_.dot(v_);
        tmp_ = coords_[sfn_[current_index_]] - proj_qp_;
        uvn_sfn_[0] = tmp_.dot(u_);
        uvn_sfn_[1] = tmp_.dot(v_);
        bool prev_ffn = isVisible(uvn_prev, uvn_next, uvn_current, uvn_ffn_) && isVisible(uvn_prev, uvn_sfn_, uvn_current, uvn_ffn_);
        bool prev_sfn = isVisible(uvn_prev, uvn_next, uvn_current, uvn_sfn_) && isVisible(uvn_prev, uvn_ffn_, uvn_current, uvn_sfn_);
        bool next_ffn = isVisible(uvn_next, uvn_prev, uvn_current, uvn_ffn_) && isVisible(uvn_next, uvn_sfn_, uvn_current, uvn_ffn_);
        bool next_sfn = isVisible(uvn_next, uvn_prev, uvn_current, uvn_sfn_) && isVisible(uvn_next, uvn_ffn_, uvn_current, uvn_sfn_);
        int min_dist = -1;
        if (prev_ffn && next_sfn && prev_sfn && next_ffn)
        {
          /* should be never the case */
          double prev2f = (coords_[ffn_[current_index_]] - coords_[prev_index]).squaredNorm ();
          double next2s = (coords_[sfn_[current_index_]] - coords_[next_index]).squaredNorm ();
          double prev2s = (coords_[sfn_[current_index_]] - coords_[prev_index]).squaredNorm ();
          double next2f = (coords_[ffn_[current_index_]] - coords_[next_index]).squaredNorm ();
          if (prev2f < prev2s)
          {
            if (prev2f < next2f)
            {
              if (prev2f < next2s)
                min_dist = 0;
              else
                min_dist = 3;
            }
            else
            {
              if (next2f < next2s)
                min_dist = 2;
              else
                min_dist = 3;
            }
          }
          else
          {
            if (prev2s < next2f)
            {
              if (prev2s < next2s)
                min_dist = 1;
              else
                min_dist = 3;
            }
            else
            {
              if (next2f < next2s)
                min_dist = 2;
              else
                min_dist = 3;
            }
          }
        }
        else if (prev_ffn && next_sfn)
        {
          /* a clear case */
          double prev2f = (coords_[ffn_[current_index_]] - coords_[prev_index]).squaredNorm ();
          double next2s = (coords_[sfn_[current_index_]] - coords_[next_index]).squaredNorm ();
          if (prev2f < next2s)
            min_dist = 0;
          else
            min_dist = 3;
        }
        else if (prev_sfn && next_ffn)
        {
          /* a clear case */
          double prev2s = (coords_[sfn_[current_index_]] - coords_[prev_index]).squaredNorm ();
          double next2f = (coords_[ffn_[current_index_]] - coords_[next_index]).squaredNorm ();
          if (prev2s < next2f)
            min_dist = 1;
          else
            min_dist = 2;
        }
        /* straightforward cases */
        else if (prev_ffn && !next_sfn && !prev_sfn && !next_ffn)
          min_dist = 0;
        else if (!prev_ffn && !next_sfn && prev_sfn && !next_ffn)
          min_dist = 1;
        else if (!prev_ffn && !next_sfn && !prev_sfn && next_ffn)
          min_dist = 2;
        else if (!prev_ffn && next_sfn && !prev_sfn && !next_ffn)
          min_dist = 3;
        /* messed up cases */
        else if (prev_ffn)
        {
          double prev2f = (coords_[ffn_[current_index_]] - coords_[prev_index]).squaredNorm ();
          if (prev_sfn)
          {
            double prev2s = (coords_[sfn_[current_index_]] - coords_[prev_index]).squaredNorm ();
            if (prev2s < prev2f)
              min_dist = 1;
            else
              min_dist = 0;
          }
          else if (next_ffn)
          {
            double next2f = (coords_[ffn_[current_index_]] - coords_[next_index]).squaredNorm ();
            if (next2f < prev2f)
              min_dist = 2;
            else
              min_dist = 0;
          }
        }
        else if (next_sfn)
        {
          double next2s = (coords_[sfn_[current_index_]] - coords_[next_index]).squaredNorm ();
          if (prev_sfn)
          {
            double prev2s = (coords_[sfn_[current_index_]] - coords_[prev_index]).squaredNorm ();
            if (prev2s < next2s)
              min_dist = 1;
            else
              min_dist = 3;
          }
          else if (next_ffn)
          {
            double next2f = (coords_[ffn_[current_index_]] - coords_[next_index]).squaredNorm ();
            if (next2f < next2s)
              min_dist = 2;
            else
              min_dist = 3;
          }
        }
        switch (min_dist)
        {
          case 0://prev2f:
          {
            addTriangle (current_index_, ffn_[current_index_], prev_index, polygons);

            /* updating prev_index */
            if (ffn_[prev_index] == current_index_)
            {
              ffn_[prev_index] = ffn_[current_index_];
            }
            else if (sfn_[prev_index] == current_index_)
            {
              sfn_[prev_index] = ffn_[current_index_];
            }
            else if (ffn_[prev_index] == R_)
            {
              changed_1st_fn_ = true;
              ffn_[prev_index] = ffn_[current_index_];
            }
            else if (sfn_[prev_index] == R_)
            {
              changed_1st_fn_ = true;
              sfn_[prev_index] = ffn_[current_index_];
            }
            else if (prev_index == R_)
            {
              new2boundary_ = ffn_[current_index_];
            }

            /* updating ffn */
            if (ffn_[ffn_[current_index_]] == current_index_)
            {
              ffn_[ffn_[current_index_]] = prev_index;
            }
            else if (sfn_[ffn_[current_index_]] == current_index_)
            {
              sfn_[ffn_[current_index_]] = prev_index;
            }

            /* updating current */
            ffn_[current_index_] = next_index;

            break;
          }
          case 1://prev2s:
          {
            addTriangle (current_index_, sfn_[current_index_], prev_index, polygons);

            /* updating prev_index */
            if (ffn_[prev_index] == current_index_)
            {
              ffn_[prev_index] = sfn_[current_index_];
            }
            else if (sfn_[prev_index] == current_index_)
            {
              sfn_[prev_index] = sfn_[current_index_];
            }
            else if (ffn_[prev_index] == R_)
            {
              changed_1st_fn_ = true;
              ffn_[prev_index] = sfn_[current_index_];
            }
            else if (sfn_[prev_index] == R_)
            {
              changed_1st_fn_ = true;
              sfn_[prev_index] = sfn_[current_index_];
            }
            else if (prev_index == R_)
            {
              new2boundary_ = sfn_[current_index_];
            }

            /* updating sfn */
            if (ffn_[sfn_[current_index_]] == current_index_)
            {
              ffn_[sfn_[current_index_]] = prev_index;
            }
            else if (sfn_[sfn_[current_index_]] == current_index_)
            {
              sfn_[sfn_[current_index_]] = prev_index;
            }

            /* updating current */
            sfn_[current_index_] = next_index;

            break;
          }
          case 2://next2f:
          {
            addTriangle (current_index_, ffn_[current_index_], next_index, polygons);
            auto neighbor_update = next_index;

            /* updating next_index */
            if (state_[next_index] <= FREE)
            {
              state_[next_index] = FRINGE;
              ffn_[next_index] = current_index_;
              sfn_[next_index] = ffn_[current_index_];
            }
            else
            {
              if (ffn_[next_index] == R_)
              {
                changed_2nd_fn_ = true;
                ffn_[next_index] = ffn_[current_index_];
              }
              else if (sfn_[next_index] == R_)
              {
                changed_2nd_fn_ = true;
                sfn_[next_index] = ffn_[current_index_];
              }
              else if (next_index == R_)
              {
                new2boundary_ = ffn_[current_index_];
                if (next_next_index == new2boundary_)
                  already_connected_ = true;
              }
              else if (ffn_[next_index] == next_next_index)
              {
                already_connected_ = true;
                ffn_[next_index] = ffn_[current_index_];
              }
              else if (sfn_[next_index] == next_next_index)
              {
                already_connected_ = true;
                sfn_[next_index] = ffn_[current_index_];
              }
              else
              {
                tmp_ = coords_[ffn_[next_index]] - proj_qp_;
                uvn_next_ffn_[0] = tmp_.dot(u_);
                uvn_next_ffn_[1] = tmp_.dot(v_);
                tmp_ = coords_[sfn_[next_index]] - proj_qp_;
                uvn_next_sfn_[0] = tmp_.dot(u_);
                uvn_next_sfn_[1] = tmp_.dot(v_);

                bool ffn_next_ffn = isVisible(uvn_next_ffn_, uvn_next, uvn_current, uvn_ffn_) && isVisible(uvn_next_ffn_, uvn_next, uvn_next_sfn_, uvn_ffn_);
                bool sfn_next_ffn = isVisible(uvn_next_sfn_, uvn_next, uvn_current, uvn_ffn_) && isVisible(uvn_next_sfn_, uvn_next, uvn_next_ffn_, uvn_ffn_);

                int connect2ffn = -1;
                if (ffn_next_ffn && sfn_next_ffn)
                {
                  double fn2f = (coords_[ffn_[current_index_]] - coords_[ffn_[next_index]]).squaredNorm ();
                  double sn2f = (coords_[ffn_[current_index_]] - coords_[sfn_[next_index]]).squaredNorm ();
                  if (fn2f < sn2f) connect2ffn = 0;
                  else connect2ffn = 1;
                }
                else if (ffn_next_ffn) connect2ffn = 0;
                else if (sfn_next_ffn) connect2ffn = 1;

                switch (connect2ffn)
                {
                  case 0: // ffn[next]
                  {
                    addTriangle (next_index, ffn_[current_index_], ffn_[next_index], polygons);
                    neighbor_update = ffn_[next_index];

                    /* ffn[next_index] */
                    if ((ffn_[ffn_[next_index]] == ffn_[current_index_]) || (sfn_[ffn_[next_index]] == ffn_[current_index_]))
                    {
                      state_[ffn_[next_index]] = COMPLETED;
                    }
                    else if (ffn_[ffn_[next_index]] == next_index)
                    {
                      ffn_[ffn_[next_index]] = ffn_[current_index_];
                    }
                    else if (sfn_[ffn_[next_index]] == next_index)
                    {
                      sfn_[ffn_[next_index]] = ffn_[current_index_];
                    }

                    ffn_[next_index] = current_index_;

                    break;
                  }
                  case 1: // sfn[next]
                  {
                    addTriangle (next_index, ffn_[current_index_], sfn_[next_index], polygons);
                    neighbor_update = sfn_[next_index];

                    /* sfn[next_index] */
                    if ((ffn_[sfn_[next_index]] == ffn_[current_index_]) || (sfn_[sfn_[next_index]] == ffn_[current_index_]))
                    {
                      state_[sfn_[next_index]] = COMPLETED;
                    }
                    else if (ffn_[sfn_[next_index]] == next_index)
                    {
                      ffn_[sfn_[next_index]] = ffn_[current_index_];
                    }
                    else if (sfn_[sfn_[next_index]] == next_index)
                    {
                      sfn_[sfn_[next_index]] = ffn_[current_index_];
                    }

                    sfn_[next_index] = current_index_;

                    break;
                  }
                  default:;
                }
              }
            }

            /* updating ffn */
            if ((ffn_[ffn_[current_index_]] == neighbor_update) || (sfn_[ffn_[current_index_]] == neighbor_update))
            {
              state_[ffn_[current_index_]] = COMPLETED;
            }
            else if (ffn_[ffn_[current_index_]] == current_index_)
            {
              ffn_[ffn_[current_index_]] = neighbor_update;
            }
            else if (sfn_[ffn_[current_index_]] == current_index_)
            {
              sfn_[ffn_[current_index_]] = neighbor_update;
            }

            /* updating current */
            ffn_[current_index_] = prev_index;

            break;
          }
          case 3://next2s:
          {
            addTriangle (current_index_, sfn_[current_index_], next_index, polygons);
            auto neighbor_update = next_index;

            /* updating next_index */
            if (state_[next_index] <= FREE)
            {
              state_[next_index] = FRINGE;
              ffn_[next_index] = current_index_;
              sfn_[next_index] = sfn_[current_index_];
            }
            else
            {
              if (ffn_[next_index] == R_)
              {
                changed_2nd_fn_ = true;
                ffn_[next_index] = sfn_[current_index_];
              }
              else if (sfn_[next_index] == R_)
              {
                changed_2nd_fn_ = true;
                sfn_[next_index] = sfn_[current_index_];
              }
              else if (next_index == R_)
              {
                new2boundary_ = sfn_[current_index_];
                if (next_next_index == new2boundary_)
                  already_connected_ = true;
              }
              else if (ffn_[next_index] == next_next_index)
              {
                already_connected_ = true;
                ffn_[next_index] = sfn_[current_index_];
              }
              else if (sfn_[next_index] == next_next_index)
              {
                already_connected_ = true;
                sfn_[next_index] = sfn_[current_index_];
              }
              else
              {
                tmp_ = coords_[ffn_[next_index]] - proj_qp_;
                uvn_next_ffn_[0] = tmp_.dot(u_);
                uvn_next_ffn_[1] = tmp_.dot(v_);
                tmp_ = coords_[sfn_[next_index]] - proj_qp_;
                uvn_next_sfn_[0] = tmp_.dot(u_);
                uvn_next_sfn_[1] = tmp_.dot(v_);

                bool ffn_next_sfn = isVisible(uvn_next_ffn_, uvn_next, uvn_current, uvn_sfn_) && isVisible(uvn_next_ffn_, uvn_next, uvn_next_sfn_, uvn_sfn_);
                bool sfn_next_sfn = isVisible(uvn_next_sfn_, uvn_next, uvn_current, uvn_sfn_) && isVisible(uvn_next_sfn_, uvn_next, uvn_next_ffn_, uvn_sfn_);

                int connect2sfn = -1;
                if (ffn_next_sfn && sfn_next_sfn)
                {
                  double fn2s = (coords_[sfn_[current_index_]] - coords_[ffn_[next_index]]).squaredNorm ();
                  double sn2s = (coords_[sfn_[current_index_]] - coords_[sfn_[next_index]]).squaredNorm ();
                  if (fn2s < sn2s) connect2sfn = 0;
                  else connect2sfn = 1;
                }
                else if (ffn_next_sfn) connect2sfn = 0;
                else if (sfn_next_sfn) connect2sfn = 1;

                switch (connect2sfn)
                {
                  case 0: // ffn[next]
                  {
                    addTriangle (next_index, sfn_[current_index_], ffn_[next_index], polygons);
                    neighbor_update = ffn_[next_index];

                    /* ffn[next_index] */
                    if ((ffn_[ffn_[next_index]] == sfn_[current_index_]) || (sfn_[ffn_[next_index]] == sfn_[current_index_]))
                    {
                      state_[ffn_[next_index]] = COMPLETED;
                    }
                    else if (ffn_[ffn_[next_index]] == next_index)
                    {
                      ffn_[ffn_[next_index]] = sfn_[current_index_];
                    }
                    else if (sfn_[ffn_[next_index]] == next_index)
                    {
                      sfn_[ffn_[next_index]] = sfn_[current_index_];
                    }

                    ffn_[next_index] = current_index_;

                    break;
                  }
                  case 1: // sfn[next]
                  {
                    addTriangle (next_index, sfn_[current_index_], sfn_[next_index], polygons);
                    neighbor_update = sfn_[next_index];

                    /* sfn[next_index] */
                    if ((ffn_[sfn_[next_index]] == sfn_[current_index_]) || (sfn_[sfn_[next_index]] == sfn_[current_index_]))
                    {
                      state_[sfn_[next_index]] = COMPLETED;
                    }
                    else if (ffn_[sfn_[next_index]] == next_index)
                    {
                      ffn_[sfn_[next_index]] = sfn_[current_index_];
                    }
                    else if (sfn_[sfn_[next_index]] == next_index)
                    {
                      sfn_[sfn_[next_index]] = sfn_[current_index_];
                    }

                    sfn_[next_index] = current_index_;

                    break;
                  }
                  default:;
                }
              }
            }

            /* updating sfn */
            if ((ffn_[sfn_[current_index_]] == neighbor_update) || (sfn_[sfn_[current_index_]] == neighbor_update))
            {
              state_[sfn_[current_index_]] = COMPLETED;
            }
            else if (ffn_[sfn_[current_index_]] == current_index_)
            {
              ffn_[sfn_[current_index_]] = neighbor_update;
            }
            else if (sfn_[sfn_[current_index_]] == current_index_)
            {
              sfn_[sfn_[current_index_]] = neighbor_update;
            }

            sfn_[current_index_] = prev_index;

            break;
          }
          default:;
        }
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> std::vector<std::vector<std::size_t> >
pcl::GreedyProjectionTriangulation<PointInT>::getTriangleList (const pcl::PolygonMesh &input)
{
  std::vector<std::vector<std::size_t> > triangleList (input.cloud.width * input.cloud.height);

  for (std::size_t i=0; i < input.polygons.size (); ++i)
    for (std::size_t j=0; j < input.polygons[i].vertices.size (); ++j)
      triangleList[input.polygons[i].vertices[j]].push_back (i);
  return (triangleList);
}

#define PCL_INSTANTIATE_GreedyProjectionTriangulation(T)                \
  template class PCL_EXPORTS pcl::GreedyProjectionTriangulation<T>;

#endif    // PCL_SURFACE_IMPL_GP3_H_



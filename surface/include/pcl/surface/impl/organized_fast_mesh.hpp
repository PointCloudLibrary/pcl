/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Dirk Holz (University of Bonn)
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

#ifndef PCL_SURFACE_ORGANIZED_FAST_MESH_HPP_
#define PCL_SURFACE_ORGANIZED_FAST_MESH_HPP_

#include <pcl/surface/organized_fast_mesh.h>

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::OrganizedFastMesh<PointInT>::performReconstruction (pcl::PolygonMesh &output)
{
  reconstructPolygons (output.polygons);

  // Get the field names
  int x_idx = pcl::getFieldIndex (output.cloud, "x");
  int y_idx = pcl::getFieldIndex (output.cloud, "y");
  int z_idx = pcl::getFieldIndex (output.cloud, "z");
  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
    return;
  // correct all measurements,
  // (running over complete image since some rows and columns are left out
  // depending on triangle_pixel_size)
  // avoid to do that here (only needed for ASCII mesh file output, e.g., in vtk files
  for (unsigned int i = 0; i < input_->points.size (); ++i)
    if (!isFinite (input_->points[i]))
      resetPointData (i, output, 0.0f, x_idx, y_idx, z_idx);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::OrganizedFastMesh<PointInT>::performReconstruction (std::vector<pcl::Vertices> &polygons)
{
  reconstructPolygons (polygons);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::OrganizedFastMesh<PointInT>::reconstructPolygons (std::vector<pcl::Vertices> &polygons)
{
  if (triangulation_type_ == TRIANGLE_RIGHT_CUT)
    makeRightCutMesh (polygons);
  else if (triangulation_type_ == TRIANGLE_LEFT_CUT)
    makeLeftCutMesh (polygons);
  else if (triangulation_type_ == TRIANGLE_ADAPTIVE_CUT)
    makeAdaptiveCutMesh (polygons);
  else if (triangulation_type_ == QUAD_MESH)
    makeQuadMesh (polygons);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::OrganizedFastMesh<PointInT>::makeQuadMesh (std::vector<pcl::Vertices>& polygons)
{
  int last_column = input_->width - triangle_pixel_size_columns_;
  int last_row = input_->height - triangle_pixel_size_rows_;

  int i = 0, index_down = 0, index_right = 0, index_down_right = 0, idx = 0;
  int y_big_incr = triangle_pixel_size_rows_ * input_->width,
      x_big_incr = y_big_incr + triangle_pixel_size_columns_;
  // Reserve enough space
  polygons.resize (input_->width * input_->height);

  // Go over the rows first
  for (int y = 0; y < last_row; y += triangle_pixel_size_rows_)
  {
    // Initialize a new row
    i = y * input_->width;
    index_right = i + triangle_pixel_size_columns_;
    index_down = i + y_big_incr;
    index_down_right = i + x_big_incr;

    // Go over the columns
    for (int x = 0; x < last_column; x += triangle_pixel_size_columns_,
                                     i += triangle_pixel_size_columns_,
                                     index_right += triangle_pixel_size_columns_,
                                     index_down += triangle_pixel_size_columns_,
                                     index_down_right += triangle_pixel_size_columns_)
    {
      if (isValidQuad (i, index_right, index_down_right, index_down))
        if (store_shadowed_faces_ || !isShadowedQuad (i, index_right, index_down_right, index_down))
          addQuad (i, index_right, index_down_right, index_down, idx++, polygons);
    }
  }
  polygons.resize (idx);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::OrganizedFastMesh<PointInT>::makeRightCutMesh (std::vector<pcl::Vertices>& polygons)
{
  int last_column = input_->width - triangle_pixel_size_columns_;
  int last_row = input_->height - triangle_pixel_size_rows_;

  int i = 0, index_down = 0, index_right = 0, index_down_right = 0, idx = 0;
  int y_big_incr = triangle_pixel_size_rows_ * input_->width,
      x_big_incr = y_big_incr + triangle_pixel_size_columns_;
  // Reserve enough space
  polygons.resize (input_->width * input_->height * 2);

  // Go over the rows first
  for (int y = 0; y < last_row; y += triangle_pixel_size_rows_)
  {
    // Initialize a new row
    i = y * input_->width;
    index_right = i + triangle_pixel_size_columns_;
    index_down = i + y_big_incr;
    index_down_right = i + x_big_incr;

    // Go over the columns
    for (int x = 0; x < last_column; x += triangle_pixel_size_columns_,
                                     i += triangle_pixel_size_columns_,
                                     index_right += triangle_pixel_size_columns_,
                                     index_down += triangle_pixel_size_columns_,
                                     index_down_right += triangle_pixel_size_columns_)
    {
      if (isValidTriangle (i, index_down_right, index_right))
        if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down_right, index_right))
          addTriangle (i, index_down_right, index_right, idx++, polygons);

      if (isValidTriangle (i, index_down, index_down_right))
        if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down, index_down_right))
          addTriangle (i, index_down, index_down_right, idx++, polygons);
    }
  }
  polygons.resize (idx);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::OrganizedFastMesh<PointInT>::makeLeftCutMesh (std::vector<pcl::Vertices>& polygons)
{
  int last_column = input_->width - triangle_pixel_size_columns_;
  int last_row = input_->height - triangle_pixel_size_rows_;

  int i = 0, index_down = 0, index_right = 0, index_down_right = 0, idx = 0;
  int y_big_incr = triangle_pixel_size_rows_ * input_->width,
      x_big_incr = y_big_incr + triangle_pixel_size_columns_;
  // Reserve enough space
  polygons.resize (input_->width * input_->height * 2);

  // Go over the rows first
  for (int y = 0; y < last_row; y += triangle_pixel_size_rows_)
  {
    // Initialize a new row
    i = y * input_->width;
    index_right = i + triangle_pixel_size_columns_;
    index_down = i + y_big_incr;
    index_down_right = i + x_big_incr;

    // Go over the columns
    for (int x = 0; x < last_column; x += triangle_pixel_size_columns_,
                                     i += triangle_pixel_size_columns_,
                                     index_right += triangle_pixel_size_columns_,
                                     index_down += triangle_pixel_size_columns_,
                                     index_down_right += triangle_pixel_size_columns_)
    {
      if (isValidTriangle (i, index_down, index_right))
        if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down, index_right))
          addTriangle (i, index_down, index_right, idx++, polygons);

      if (isValidTriangle (index_right, index_down, index_down_right))
        if (store_shadowed_faces_ || !isShadowedTriangle (index_right, index_down, index_down_right))
          addTriangle (index_right, index_down, index_down_right, idx++, polygons);
    }
  }
  polygons.resize (idx);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::OrganizedFastMesh<PointInT>::makeAdaptiveCutMesh (std::vector<pcl::Vertices>& polygons)
{
  int last_column = input_->width - triangle_pixel_size_columns_;
  int last_row = input_->height - triangle_pixel_size_rows_;

  int i = 0, index_down = 0, index_right = 0, index_down_right = 0, idx = 0;
  int y_big_incr = triangle_pixel_size_rows_ * input_->width,
      x_big_incr = y_big_incr + triangle_pixel_size_columns_;
  // Reserve enough space
  polygons.resize (input_->width * input_->height * 2);

  // Go over the rows first
  for (int y = 0; y < last_row; y += triangle_pixel_size_rows_)
  {
    // Initialize a new row
    i = y * input_->width;
    index_right = i + triangle_pixel_size_columns_;
    index_down = i + y_big_incr;
    index_down_right = i + x_big_incr;

    // Go over the columns
    for (int x = 0; x < last_column; x += triangle_pixel_size_columns_,
                                     i += triangle_pixel_size_columns_,
                                     index_right += triangle_pixel_size_columns_,
                                     index_down += triangle_pixel_size_columns_,
                                     index_down_right += triangle_pixel_size_columns_)
    {
      const bool right_cut_upper = isValidTriangle (i, index_down_right, index_right);
      const bool right_cut_lower = isValidTriangle (i, index_down, index_down_right);
      const bool left_cut_upper = isValidTriangle (i, index_down, index_right);
      const bool left_cut_lower = isValidTriangle (index_right, index_down, index_down_right);

      if (right_cut_upper && right_cut_lower && left_cut_upper && left_cut_lower)
      {
        float dist_right_cut = fabsf (input_->points[index_down].z - input_->points[index_right].z);
        float dist_left_cut = fabsf (input_->points[i].z - input_->points[index_down_right].z);
        if (dist_right_cut >= dist_left_cut)
        {
          if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down_right, index_right))
            addTriangle (i, index_down_right, index_right, idx++, polygons);
          if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down, index_down_right))
            addTriangle (i, index_down, index_down_right, idx++, polygons);
        }
        else
        {
          if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down, index_right))
            addTriangle (i, index_down, index_right, idx++, polygons);
          if (store_shadowed_faces_ || !isShadowedTriangle (index_right, index_down, index_down_right))
            addTriangle (index_right, index_down, index_down_right, idx++, polygons);
        }
      }
      else
      {
        if (right_cut_upper)
          if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down_right, index_right))
            addTriangle (i, index_down_right, index_right, idx++, polygons);
        if (right_cut_lower)
          if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down, index_down_right))
            addTriangle (i, index_down, index_down_right, idx++, polygons);
        if (left_cut_upper)
          if (store_shadowed_faces_ || !isShadowedTriangle (i, index_down, index_right))
            addTriangle (i, index_down, index_right, idx++, polygons);
        if (left_cut_lower)
          if (store_shadowed_faces_ || !isShadowedTriangle (index_right, index_down, index_down_right))
            addTriangle (index_right, index_down, index_down_right, idx++, polygons);
      }
    }
  }
  polygons.resize (idx);
}

#define PCL_INSTANTIATE_OrganizedFastMesh(T)                \
  template class PCL_EXPORTS pcl::OrganizedFastMesh<T>;

#endif  // PCL_SURFACE_ORGANIZED_FAST_MESH_HPP_

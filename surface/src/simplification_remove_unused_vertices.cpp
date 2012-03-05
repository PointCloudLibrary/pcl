/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
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

#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <cstring>
#include <vector>
#include <iostream>
#include <stdio.h>

void
pcl::surface::SimplificationRemoveUnusedVertices::simplify(const pcl::PolygonMesh& input, pcl::PolygonMesh& output, std::vector<int>& indices)
{
  if (input.polygons.size () == 0)
    return;

  unsigned int nr_points = input.cloud.width * input.cloud.height;

  std::vector<int> new_indices (nr_points, -1);
  indices.clear ();
  indices.reserve (nr_points);

  // mark all points in triangles as being used
  for (size_t polygon = 0; polygon < input.polygons.size (); ++polygon)
    for (size_t point = 0; point < input.polygons[polygon].vertices.size (); ++point)
      if (new_indices[ input.polygons[polygon].vertices[point] ] == -1 )
      {
        new_indices[input.polygons[polygon].vertices[point]] = static_cast<int> (indices.size ());
        indices.push_back (input.polygons[polygon].vertices[point]);
      }

  // in case all points are used , do nothing and return input mesh
  if (indices.size () == nr_points)
  {
    output = input;
    return;
  }

  // copy cloud information
  output.header = input.header;
  output.cloud.data.clear ();
  output.cloud.header = input.cloud.header;
  output.cloud.fields = input.cloud.fields;
  output.cloud.row_step = input.cloud.row_step;
  output.cloud.point_step = input.cloud.point_step;
  output.cloud.is_bigendian = input.cloud.is_bigendian;
  output.cloud.height = 1; // cloud is no longer organized
  output.cloud.width = static_cast<int> (indices.size ());
  output.cloud.row_step = output.cloud.point_step * output.cloud.width;
  output.cloud.data.resize (output.cloud.width * output.cloud.height * output.cloud.point_step);
  output.cloud.is_dense = false;
  output.polygons.clear ();

  // copy (only!) used points
  for (size_t i = 0; i < indices.size (); ++i)
    memcpy (&output.cloud.data[i * output.cloud.point_step], &input.cloud.data[indices[i] * output.cloud.point_step], output.cloud.point_step);

  // copy mesh information (and update indices)
  output.polygons.reserve (input.polygons.size ());
  for (size_t polygon = 0; polygon < input.polygons.size (); ++polygon)
  {
    pcl::Vertices corrected_polygon;
    corrected_polygon.vertices.resize (input.polygons[polygon].vertices.size ());
    for (size_t point = 0; point < input.polygons[polygon].vertices.size(); ++point)
      corrected_polygon.vertices[point] = new_indices[input.polygons[polygon].vertices[point]];
    output.polygons.push_back (corrected_polygon);
  }
}


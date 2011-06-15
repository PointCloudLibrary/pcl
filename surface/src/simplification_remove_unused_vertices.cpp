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

void
pcl::surface::SimplificationRemoveUnusedVertices::simplify(const pcl::PolygonMesh& input, pcl::PolygonMesh& output)
{
  if ( input.polygons.size() == 0)
    return;

  unsigned int nr_points = input.cloud.width * input.cloud.height;

  // mark all points as being unused
  std::vector<bool> point_used ((size_t)(nr_points), false);

  // mark all points in triangles as being used
  unsigned int nr_used_points = 0;
  for ( size_t polygon = 0; polygon < input.polygons.size(); ++polygon)
    for ( size_t point = 0; point < input.polygons[polygon].vertices.size(); ++point )
      if ( !point_used[ input.polygons[polygon].vertices[point] ] )
      {
        ++nr_used_points;
        point_used[ input.polygons[polygon].vertices[point] ] = true;
      }

  // in case all points are, do nothing and return input mesh
  if ( nr_used_points == nr_points )
  {
    output = input;
    return;
  }

  // copy cloud information
  output.header = input.header;
  output.cloud.data.clear();
  output.polygons.clear();
  output.cloud.header = input.cloud.header;
  output.cloud.fields = input.cloud.fields;
  output.cloud.row_step = input.cloud.row_step;
  output.cloud.point_step = input.cloud.point_step;
  output.cloud.is_bigendian = input.cloud.is_bigendian;
  output.cloud.height = 1; // cloud is no longer organized
  output.cloud.width = nr_used_points;
  output.cloud.row_step = output.cloud.point_step * output.cloud.width;
  output.cloud.data.resize (output.cloud.width * output.cloud.height * output.cloud.point_step);

  // copy (only!) used points
  std::vector<int> point_indices ((size_t)(nr_points), 0);
  nr_used_points = 0;
  for ( size_t i = 0; i < point_used.size(); ++i )
  {
    if ( point_used[i] )
    {
      memcpy (&output.cloud.data[nr_used_points * output.cloud.point_step], &input.cloud.data[i * output.cloud.point_step], output.cloud.point_step);
      point_indices[i] = nr_used_points;
      nr_used_points++;
    }
  }

  // copy mesh information (and update indices)
  output.polygons.reserve( input.polygons.size() );
  for ( size_t polygon = 0; polygon < input.polygons.size(); ++polygon)
  {
    pcl::Vertices corrected_polygon;
    corrected_polygon.vertices.resize(input.polygons[polygon].vertices.size());

    for ( size_t point = 0; point < input.polygons[polygon].vertices.size(); ++point )
      corrected_polygon.vertices[point] = point_indices[ input.polygons[polygon].vertices[point] ];

    output.polygons.push_back(corrected_polygon);
  }
}

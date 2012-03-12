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
 * $Id: texture_mapping.hpp 3387 2011-12-05 05:17:22Z rusu $
 *
 */
/** \author Khai Tran */
#ifndef PCL_SURFACE_IMPL_TEXTURE_MAPPING_HPP_
#define PCL_SURFACE_IMPL_TEXTURE_MAPPING_HPP_

#include <pcl/surface/texture_mapping.h>

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> float
pcl::TextureMapping<PointInT>::getDistance (Eigen::Vector3f &p1, Eigen::Vector3f &p2)
{
  return std::sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]));
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> std::vector<Eigen::Vector2f>
pcl::TextureMapping<PointInT>::mapTexture2Face (Eigen::Vector3f  &p1, Eigen::Vector3f  &p2, Eigen::Vector3f &p3)
{
  std::vector<Eigen::Vector2f> tex_coordinates;
  // process for each face
  Eigen::Vector3f p1p2(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
  Eigen::Vector3f p1p3(p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]);
  Eigen::Vector3f p2p3(p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2]);

  // Normalize
  p1p2 = p1p2/std::sqrt(p1p2.dot(p1p2));
  p1p3 = p1p3/std::sqrt(p1p3.dot(p1p3));
  p2p3 = p2p3/std::sqrt(p2p3.dot(p2p3));

  // compute vector normal of a face
  Eigen::Vector3f f_normal = p1p2.cross(p1p3);
  f_normal = f_normal/std::sqrt(f_normal.dot(f_normal));

  // project vector field onto the face: vector v1_projected = v1 - Dot(v1, n) * n;
  Eigen::Vector3f f_vector_field =  vector_field_ - vector_field_.dot(f_normal) * f_normal;

  // Normalize
  f_vector_field = f_vector_field/std::sqrt(f_vector_field.dot(f_vector_field));

  // texture coordinates
  Eigen::Vector2f tp1, tp2, tp3;

  double alpha = std::acos(f_vector_field.dot(p1p2));

  // distance between 3 vertices of triangles
  double e1 = getDistance(p2, p3)/f_;
  double e2 = getDistance(p1, p3)/f_;
  double e3 = getDistance(p1, p2)/f_;

  // initialize
  tp1[0] = 0.0;
  tp1[1] = 0.0;

  tp2[0] = e3;
  tp2[1] = 0.0;

  // determine texture coordinate tp3;
  double cos_p1 = (e2*e2+e3*e3-e1*e1)/(2*e2*e3);
  double sin_p1 = sqrt(1-(cos_p1*cos_p1));

  tp3[0] = cos_p1*e2;
  tp3[1] = sin_p1*e2;

  // rotating by alpha (angle between V and pp1 & pp2)
  Eigen::Vector2f r_tp2, r_tp3;
  r_tp2[0] = tp2[0]*std::cos(alpha) - tp2[1]*std::sin(alpha);
  r_tp2[1] = tp2[0]*std::sin(alpha) + tp2[1]*std::cos(alpha);

  r_tp3[0] = tp3[0]*std::cos(alpha) - tp3[1]*std::sin(alpha);
  r_tp3[1] = tp3[0]*std::sin(alpha) + tp3[1]*std::cos(alpha);

  // shifting
  tp1[0] = tp1[0];
  tp2[0] = r_tp2[0];
  tp3[0] = r_tp3[0];
  tp1[1] = tp1[1];
  tp2[1] = r_tp2[1];
  tp3[1] = r_tp3[1];

  float min_x = tp1[0];
  float min_y = tp1[1];
  if (min_x > tp2[0]) min_x = tp2[0];
  if (min_x > tp3[0]) min_x = tp3[0];
  if (min_y > tp2[1]) min_y = tp2[1];
  if (min_y > tp3[1]) min_y = tp3[1];

  if(min_x < 0)
  {
    tp1[0] = tp1[0] - min_x;
    tp2[0] = tp2[0] - min_x;
    tp3[0] = tp3[0] - min_x;
  }
  if(min_y < 0)
  {
    tp1[1] = tp1[1] - min_y;
    tp2[1] = tp2[1] - min_y;
    tp3[1] = tp3[1] - min_y;
  }

  tex_coordinates.push_back(tp1);
  tex_coordinates.push_back(tp2);
  tex_coordinates.push_back(tp3);
  return tex_coordinates;
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::TextureMapping<PointInT>::mapTexture2Mesh (pcl::TextureMesh &tex_mesh)
{
  // mesh information
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // temporary PointXYZ
  float x, y, z;
  // temporary face
  Eigen::Vector3f facet[3];

  // texture coordinates for each mesh
  std::vector< std::vector<Eigen::Vector2f> > texture_map;

  for (size_t m = 0; m < tex_mesh.tex_polygons.size(); ++m)
  {
    // texture coordinates for each mesh
    std::vector<Eigen::Vector2f> texture_map_tmp;

    // processing for each face
    for (size_t i=0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      size_t idx;

      // get facet information
      for (size_t j=0; j < tex_mesh.tex_polygons[m][i].vertices.size(); ++j)
      {
        idx =  tex_mesh.tex_polygons[m][i].vertices[j];
        memcpy (&x, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[0].offset], sizeof (float));
        memcpy (&y, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[1].offset], sizeof (float));
        memcpy (&z, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[2].offset], sizeof (float));
        facet[j][0] = x;
        facet[j][1] = y;
        facet[j][2] = z;
      }

      // get texture coordinates of each face
      std::vector<Eigen::Vector2f> tex_coordinates = mapTexture2Face(facet[0], facet[1], facet[2]);
      for(size_t n = 0; n < tex_coordinates.size(); ++n)
        texture_map_tmp.push_back(tex_coordinates[n]);
    }// end faces

    // texture materials
    std::stringstream tex_name;
    tex_name << "material_"<< m;
    tex_name >> tex_material_.tex_name;
    tex_material_.tex_file = tex_files_[m];
    tex_mesh.tex_materials.push_back(tex_material_);

    // texture coordinates
    tex_mesh.tex_coordinates.push_back(texture_map_tmp);
  }// end meshes
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::TextureMapping<PointInT>::mapTexture2MeshUV (pcl::TextureMesh &tex_mesh){

  // mesh information
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = static_cast<int> (tex_mesh.cloud.data.size ()) / nr_points;

  float x_lowest = 100000;
  float x_highest = 0 ;
  float y_lowest = 100000;
  //float y_highest = 0 ;
  float z_lowest = 100000;
  float z_highest = 0;
  float x_, y_, z_;

  for (int i =0; i < nr_points; ++i)
  {
    memcpy (&x_, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[0].offset], sizeof (float));
    memcpy (&y_, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[1].offset], sizeof (float));
    memcpy (&z_, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[2].offset], sizeof (float));
    // x
    if (x_ <= x_lowest) x_lowest = x_;
    if (x_ > x_lowest) x_highest = x_;

    // y
    if (y_ <= y_lowest) y_lowest = y_;
    //if (y_ > y_lowest) y_highest = y_;

    // z
    if (z_ <= z_lowest) z_lowest = z_;
    if (z_ > z_lowest) z_highest = z_;
  }
  // x
  float x_range = (x_lowest - x_highest)*-1;
  float x_offset = 0 - x_lowest;
  // x
  //float y_range = (y_lowest - y_highest)*-1;
  //float y_offset = 0 - y_lowest;
  // z
  float z_range = (z_lowest - z_highest)*-1;
  float z_offset = 0 - z_lowest;


  // texture coordinates for each mesh
  std::vector< std::vector<Eigen::Vector2f> > texture_map;

  for (size_t m = 0; m < tex_mesh.tex_polygons.size(); ++m)
  {
    // texture coordinates for each mesh
    std::vector<Eigen::Vector2f> texture_map_tmp;

    // processing for each face
    for (size_t i=0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      size_t idx;
      Eigen::Vector2f tmp_VT;
      for (size_t j=0; j < tex_mesh.tex_polygons[m][i].vertices.size(); ++j)
      {
        idx =  tex_mesh.tex_polygons[m][i].vertices[j];
        memcpy (&x_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[0].offset], sizeof (float));
        memcpy (&y_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[1].offset], sizeof (float));
        memcpy (&z_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[2].offset], sizeof (float));

        // calculate uv coordinates
        tmp_VT[0] = (x_ + x_offset)/x_range;
        tmp_VT[1] = (z_ + z_offset)/z_range;
        texture_map_tmp.push_back(tmp_VT);
      }
    }// end faces

    // texture materials
    std::stringstream tex_name;
    tex_name << "material_"<< m;
    tex_name >> tex_material_.tex_name;
    tex_material_.tex_file = tex_files_[m];
    tex_mesh.tex_materials.push_back(tex_material_);

    // texture coordinates
    tex_mesh.tex_coordinates.push_back(texture_map_tmp);
  }// end meshes
}

#define PCL_INSTANTIATE_TextureMapping(T)                \
  template class PCL_EXPORTS pcl::TextureMapping<T>;

#endif /* TEXTURE_MAPPING_HPP_ */


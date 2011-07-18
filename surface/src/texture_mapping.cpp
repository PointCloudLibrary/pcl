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
 * $Id: texture_mapping.cpp 1006 2011-07-13 13:07:00 ktran $
 *
 */

/** \author Khai Tran */
#include <pcl/surface/texture_mapping.h>

///////////////////////////////////////////////////////////////////////////////////////////////
pcl::TextureMapping::TextureMapping(){}
///////////////////////////////////////////////////////////////////////////////////////////////
pcl::TextureMapping::~TextureMapping(){}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setF(float f){
  this->f = f;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setVectorField(float x, float y, float z){
  this->vector_field =  Eigen::Vector3f(x, y, z);
  // normalize vector field
   this->vector_field = this->vector_field/std::sqrt(this->vector_field.dot(this->vector_field));
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setTextureFiles( std::vector<std::string> tex_files){
  this->tex_files = tex_files;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setTextureMaterials(TexMaterial tex_material){
  this->tex_material = tex_material;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setTextureMesh(TextureMesh tex_mesh){
  this->tex_mesh = tex_mesh;
}
///////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::TextureMapping::getDistance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
  return std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}
///////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::PointXY>
pcl::TextureMapping::mapTexture2Face(pcl::PointXYZ pp1, pcl::PointXYZ pp2, pcl::PointXYZ pp3){
  std::vector<pcl::PointXY> tex_coordinates;
  // process for each face
  Eigen::Vector3f p1p2(pp2.x - pp1.x, pp2.y - pp1.y, pp2.z - pp1.z);
  Eigen::Vector3f p1p3(pp3.x - pp1.x, pp3.y - pp1.y, pp3.z - pp1.z);
  Eigen::Vector3f p2p3(pp3.x - pp2.x, pp3.y - pp2.y, pp3.z - pp2.z);

  // Normalize
  p1p2 = p1p2/std::sqrt(p1p2.dot(p1p2));
  p1p3 = p1p3/std::sqrt(p1p3.dot(p1p3));
  p2p3 = p2p3/std::sqrt(p2p3.dot(p2p3));

  // compute vector normal of a face
  Eigen::Vector3f f_normal = p1p2.cross(p1p3);
  f_normal = f_normal/std::sqrt(f_normal.dot(f_normal));
  // project vector field onto the face:
  // vector v1_projected = v1 - Dot(v1, n) * n;

  Eigen::Vector3f f_vector_field =  vector_field - vector_field.dot(f_normal) * f_normal;
  // Normalize
  f_vector_field = f_vector_field/std::sqrt(f_vector_field.dot(f_vector_field));
  PointXY tp1, tp2, tp3;

  double alpha = std::acos(f_vector_field.dot(p1p2));

  // distance between 3 vertices of triangles
  double e1 = getDistance(pp2, pp3)/f;
  double e2 = getDistance(pp1, pp3)/f;
  double e3 = getDistance(pp1, pp2)/f;

  // initialize
  tp1.x = 0.0;
  tp1.y = 0.0;

  tp2.x = e3;
  tp2.y = 0.0;

  // determine texture coordinate tp3;

  double cos_p1 = (e2*e2+e3*e3-e1*e1)/(2*e2*e3);
  double sin_p1 = sqrt(1-(cos_p1*cos_p1));

  tp3.x = cos_p1*e2;
  tp3.y = sin_p1*e2;

  // rotating by alpha (angle between V and pp1 & pp2)
  PointXY r_tp2, r_tp3;
  r_tp2.x = tp2.x*std::cos(alpha) - tp2.y*std::sin(alpha);
  r_tp2.y = tp2.x*std::sin(alpha) + tp2.y*std::cos(alpha);

  r_tp3.x = tp3.x*std::cos(alpha) - tp3.y*std::sin(alpha);
  r_tp3.y = tp3.x*std::sin(alpha) + tp3.y*std::cos(alpha);

  // shifting

  tp1.x = tp1.x;
  tp2.x = r_tp2.x;
  tp3.x = r_tp3.x;
  tp1.y = tp1.y;
  tp2.y = r_tp2.y;
  tp3.y = r_tp3.y;

  float min_x = tp1.x;
  float min_y = tp1.y;
  if (min_x > tp2.x) min_x = tp2.x;
  if (min_x > tp3.x) min_x = tp3.x;
  if (min_y > tp2.y) min_y = tp2.y;
  if (min_y > tp3.y) min_y = tp3.y;

  if(min_x < 0){
          tp1.x = tp1.x -min_x;
          tp2.x = tp2.x -min_x;
          tp3.x = tp3.x -min_x;
  }
  if(min_y < 0){
          tp1.y = tp1.y -min_y;
          tp2.y = tp2.y -min_y;
          tp3.y = tp3.y -min_y;
  }

  tex_coordinates.push_back(tp1);
  tex_coordinates.push_back(tp2);
  tex_coordinates.push_back(tp3);
  return tex_coordinates;
}
///////////////////////////////////////////////////////////////////////////////////////////////
pcl::TextureMesh
pcl::TextureMapping::mapTexture2Mesh(){
  // mesh information
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // temporary PointXYZ
  float x_, y_, z_;
  // temporary face
  PointXYZ facet[3];

  // texture coordinates for each mesh
  std::vector< std::vector<PointXY> > texture_map;

  for (size_t m = 0; m < tex_mesh.tex_polygons.size(); ++m){

    // texture coordinates for each mesh
    std::vector<PointXY> texture_map_tmp;

    // processing for each face
    for (size_t i=0; i < tex_mesh.tex_polygons[m].size(); ++i){
      size_t idx;

      // get facet information
      for (size_t j=0; j < tex_mesh.tex_polygons[m][i].vertices.size(); ++j){
        idx =  tex_mesh.tex_polygons[m][i].vertices[j];
        memcpy (&x_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[0].offset], sizeof (float));
        memcpy (&y_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[1].offset], sizeof (float));
        memcpy (&z_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[2].offset], sizeof (float));
        facet[j].x = x_;
        facet[j].y = y_;
        facet[j].z = z_;
      }

      // get texture coordinates of each face
      std::vector<PointXY> tex_coordinates = mapTexture2Face(facet[0], facet[1], facet[2]);
      for(size_t n = 0; n < tex_coordinates.size(); ++n) texture_map_tmp.push_back(tex_coordinates[n]);
    }// end faces

    // texture materials
    std::stringstream tex_name;
    tex_name << "material_"<< m;
    tex_name >> this->tex_material.tex_name;
    this->tex_material.tex_file = this->tex_files[m];
    tex_mesh.tex_materials.push_back(this->tex_material);

    // texture coordinates
    tex_mesh.tex_coordinates.push_back(texture_map_tmp);
  }// end meshes

  // return mesh with texture
  return tex_mesh;
}

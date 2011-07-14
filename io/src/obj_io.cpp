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
 * $Id: obj_io.cpp 1002 2011-07-13 13:07:00 ktran $
 *
 */
#include <pcl/io/obj_io.h>
#include <fstream>
#include <iostream>
#include <pcl/io/io.h>

int
pcl::io::saveOBJFile (const std::string &file_name,
	    const pcl::TextureMesh &tex_mesh, unsigned precision)
{
  if (tex_mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }
  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  // Define material file
  std::string mtl_file_name = file_name.substr(0, file_name.find_last_of("."))+".mtl";

  /* Write 3D information */
  // number of points
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // mesh size
  int nr_meshes = tex_mesh.tex_polygons.size();
  // number of facets for each sub mesh
  int *nr_faces = new int[nr_meshes];
  for(int m=0; m < nr_meshes; ++m){
	  nr_faces[m] =  tex_mesh.tex_polygons[m].size();
  }

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs <<"mtllib " <<mtl_file_name << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
          tex_mesh.cloud.fields[d].name == "x" ||
          tex_mesh.cloud.fields[d].name == "y" ||
          tex_mesh.cloud.fields[d].name == "z"))
      {
    	if(!v_written)
    	{
           // write vertices beginning with v
          fs << "v ";
          v_written = true;
    	}
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
      }
      fs << " ";
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

    // Write vertex texture with "vt" (adding latter)


  for(int m = 0; m < nr_meshes; ++m){
    fs << "# "<< tex_mesh.tex_coordinates[m].size() <<" vertex textures in submesh " << m <<  std::endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size(); ++i){
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i].x << " " << tex_mesh.tex_coordinates[m][i].y << std::endl;
    }
  }

  int f_idx = 0;

  // int idx_vt =0;
  for(int m = 0; m < nr_meshes; ++m){
    if( m > 0) f_idx += nr_faces[m-1];
    int m_idx = 0;
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      if(i == tex_mesh.tex_material_idx[m][m_idx] )  {
        // Specify the material will be used
        fs << "# The material will be used for mesh " << m << std::endl;
        fs << "usemtl " <<  tex_mesh.tex_materials[m][m_idx].tex_name << std::endl;
        // Write faces with "f"
        fs << "# Faces" << std::endl;
        m_idx++;
      }
      fs << "f ";
      size_t j = 0;
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size () - 1; ++j)
        fs << tex_mesh.tex_polygons[m][i].vertices[j] +1  <<"/" << 3*(i+f_idx) +j+1 << " "; // vertex index in obj file format starting with 1
      fs << tex_mesh.tex_polygons[m][i].vertices[j]+1 <<"/" << 3*(i+f_idx)+2+1 << std::endl;
    }
    fs << "# "<< nr_faces[m] <<" faces in mesh " << m << std::endl;
  }
  fs << "# End of File";

  // Close obj file
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;
  for(int m = 0; m < nr_meshes; ++m){
    for(size_t i=0; i< tex_mesh.tex_materials[m].size(); ++i){
      m_fs << "newmtl " << tex_mesh.tex_materials[m][i].tex_name << std::endl;
      m_fs << "Ka "<< tex_mesh.tex_materials[m][i].tex_Ka.r << " " << tex_mesh.tex_materials[m][i].tex_Ka.g << " " << tex_mesh.tex_materials[m][i].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
      m_fs << "Kd "<< tex_mesh.tex_materials[m][i].tex_Kd.r << " " << tex_mesh.tex_materials[m][i].tex_Kd.g << " " << tex_mesh.tex_materials[m][i].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
      m_fs << "Ks "<< tex_mesh.tex_materials[m][i].tex_Ks.r << " " << tex_mesh.tex_materials[m][i].tex_Ks.g << " " << tex_mesh.tex_materials[m][i].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
      m_fs << "d " << tex_mesh.tex_materials[m][i].tex_d << std::endl; // defines the transparency of the material to be alpha.
      m_fs << "Ns "<< tex_mesh.tex_materials[m][i].tex_Ns  << std::endl; // defines the shininess of the material to be s.
      m_fs << "illum "<< tex_mesh.tex_materials[m][i].tex_illum << std::endl; // denotes the illumination model used by the material.
                                              // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
                                              // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
      m_fs << "map_Kd " << tex_mesh.tex_materials[m][i].tex_file << std::endl;
      m_fs << "###" << std::endl;
    }
  }
  m_fs.close();
  return (0);
}

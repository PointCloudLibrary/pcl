/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: obj_io.cpp 1002 2011-07-13 13:07:00 ktran $
 *
 */
#include <pcl/io/obj_io.h>
#include <fstream>
#include <iostream>
#include <pcl/common/io.h>

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
  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  std::string mtl_file_name_nopath = mtl_file_name;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  unsigned nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  unsigned point_size = static_cast<unsigned> (tex_mesh.cloud.data.size () / nr_points);

  // mesh size
  unsigned nr_meshes = static_cast<unsigned> (tex_mesh.tex_polygons.size ());
  // number of faces for header
  unsigned nr_faces = 0;
  for (unsigned m = 0; m < nr_meshes; ++m)
    nr_faces += static_cast<unsigned> (tex_mesh.tex_polygons[m].size ());

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs << "mtllib " << mtl_file_name_nopath << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (unsigned i = 0; i < nr_points; ++i)
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
        if (!v_written)
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
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

  // Write vertex normals
  for (unsigned i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "vn" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
          tex_mesh.cloud.fields[d].name == "normal_x" ||
          tex_mesh.cloud.fields[d].name == "normal_y" ||
          tex_mesh.cloud.fields[d].name == "normal_z"))
      {
    	  if (!v_written)
    	  {
          // write vertices beginning with vn
          fs << "vn ";
          v_written = true;
    	  }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
      return (-2);
    }
    fs << std::endl;
  }
  // Write vertex texture with "vt" (adding latter)

  for (unsigned m = 0; m < nr_meshes; ++m)
  {
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  std::endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
    }
  }

  unsigned f_idx = 0;

  // int idx_vt =0;
  for (unsigned m = 0; m < nr_meshes; ++m)
  {
    if (m > 0) f_idx += static_cast<unsigned> (tex_mesh.tex_polygons[m-1].size ());

    fs << "# The material will be used for mesh " << m << std::endl;
    fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << std::endl;
    fs << "# Faces" << std::endl;

    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      // Write faces with "f"
      fs << "f";
      size_t j = 0;
      // There's one UV per vertex per face, i.e., the same vertex can have
      // different UV depending on the face.
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j) 
      {
        uint32_t idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
        fs << " " << idx
           << "/" << tex_mesh.tex_polygons[m][i].vertices.size () * (i+f_idx) +j+1
           << "/" << idx; // vertex index in obj file format starting with 1
      }
      fs << std::endl;
    }
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
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
  for(unsigned m = 0; m < nr_meshes; ++m)
  {
    m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
    m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
    m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
                                            // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
                                            // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
    m_fs << "###" << std::endl;
  }
  m_fs.close ();
  return (0);
}

int
pcl::io::saveOBJFile (const std::string &file_name,
                      const pcl::PolygonMesh &mesh, unsigned precision)
{
  if (mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }
  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  /* Write 3D information */
  // number of points
  int nr_points  = mesh.cloud.width * mesh.cloud.height;
  // point size
  unsigned point_size = static_cast<unsigned> (mesh.cloud.data.size () / nr_points);
  // number of faces for header
  unsigned nr_faces = static_cast<unsigned> (mesh.polygons.size ());
  // Do we have vertices normals?
  int normal_index = getFieldIndex (mesh.cloud, "normal");

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  if (normal_index != -1)
    fs << "# Vertices normals : " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# List of Vertices, with (x,y,z) coordinates, w is optional." << std::endl;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
    {
      int c = 0;
      // adding vertex
      if ((mesh.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
          mesh.cloud.fields[d].name == "x" ||
          mesh.cloud.fields[d].name == "y" ||
          mesh.cloud.fields[d].name == "z"))
      {
        if (mesh.cloud.fields[d].name == "x")
           // write vertices beginning with v
          fs << "v ";

        float value;
        memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }

  fs << "# "<< nr_points <<" vertices" << std::endl;

  if(normal_index != -1)
  {    
    fs << "# Normals in (x,y,z) form; normals might not be unit." <<  std::endl;
    // Write vertex normals
    for (int i = 0; i < nr_points; ++i)
    {
      int nxyz = 0;
      for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
      {
        int c = 0;
        // adding vertex
        if ((mesh.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
              mesh.cloud.fields[d].name == "normal_x" ||
              mesh.cloud.fields[d].name == "normal_y" ||
              mesh.cloud.fields[d].name == "normal_z"))
        {
          if (mesh.cloud.fields[d].name == "normal_x")
            // write vertices beginning with vn
            fs << "vn ";
          
          float value;
          memcpy (&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
          fs << value;
          if (++nxyz == 3)
            break;
          fs << " ";
        }
      }
      if (nxyz != 3)
      {
        PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
        return (-2);
      }
      fs << std::endl;
    }

    fs << "# "<< nr_points <<" vertices normals" << std::endl;
  }

  fs << "# Face Definitions" << std::endl;
  // Write down faces
  if(normal_index == -1)
  {
    for(unsigned i = 0; i < nr_faces; i++)
    {
      fs << "f ";
      size_t j = 0;    
      for (; j < mesh.polygons[i].vertices.size () - 1; ++j)
        fs << mesh.polygons[i].vertices[j] + 1 << " ";
      fs << mesh.polygons[i].vertices[j] + 1 << std::endl;
    }
  }
  else
  {
    for(unsigned i = 0; i < nr_faces; i++)
    {
      fs << "f ";
      size_t j = 0;    
      for (; j < mesh.polygons[i].vertices.size () - 1; ++j)
        fs << mesh.polygons[i].vertices[j] + 1 << "//" << mesh.polygons[i].vertices[j] + 1;
      fs << mesh.polygons[i].vertices[j] + 1 << "//" << mesh.polygons[i].vertices[j] + 1 << std::endl;
    }
  }
  fs << "# End of File" << std::endl;

  // Close obj file
  fs.close ();  
  return 0;
}

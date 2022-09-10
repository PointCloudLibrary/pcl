/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <fstream>
#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::saveVTKFile (const std::string &file_name, 
                      const pcl::PolygonMesh &triangles, unsigned precision)
{
  if (triangles.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no data!\n");
    return (-1);
  }

  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  unsigned int nr_points  = triangles.cloud.width * triangles.cloud.height;
  auto point_size = static_cast<unsigned int> (triangles.cloud.data.size () / nr_points);

  // Write the header information
  fs << "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS " << nr_points << " float" << '\n';

  // Iterate through the points
  for (unsigned int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (std::size_t d = 0; d < triangles.cloud.fields.size (); ++d)
    {
      if ((triangles.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
           triangles.cloud.fields[d].name == "x" || 
           triangles.cloud.fields[d].name == "y" || 
           triangles.cloud.fields[d].name == "z"))
      {
        float value;
        memcpy (&value, &triangles.cloud.data[i * point_size + triangles.cloud.fields[d].offset], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
      }
      fs << " ";
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << '\n';
  }

  // Write vertices
  fs << "\nVERTICES " << nr_points << " " << 2*nr_points << '\n';
  for (unsigned int i = 0; i < nr_points; ++i)
    fs << "1 " << i << '\n';

  // Write polygons
  // compute the correct number of values:
  std::size_t triangle_size = triangles.polygons.size ();
  std::size_t correct_number = triangle_size;
  for (std::size_t i = 0; i < triangle_size; ++i)
    correct_number += triangles.polygons[i].vertices.size ();
  fs << "\nPOLYGONS " << triangle_size << " " << correct_number << '\n';
  for (std::size_t i = 0; i < triangle_size; ++i)
  {
    fs << triangles.polygons[i].vertices.size () << " ";
    for (std::size_t j = 0; j < triangles.polygons[i].vertices.size () - 1; ++j)
      fs << triangles.polygons[i].vertices[j] << " ";
    fs << triangles.polygons[i].vertices.back() << '\n';
  }

  // Write RGB values
  int field_index = getFieldIndex (triangles.cloud, "rgb");
  if (field_index != -1)
  {
    fs << "\nPOINT_DATA " << nr_points << "\nCOLOR_SCALARS scalars 3\n";
    for (unsigned int i = 0; i < nr_points; ++i)
    {
      if (triangles.cloud.fields[field_index].datatype == pcl::PCLPointField::FLOAT32)
      {
        pcl::RGB color;
        memcpy (&color, &triangles.cloud.data[i * point_size + triangles.cloud.fields[field_index].offset], sizeof (RGB));
        int r = color.r;
        int g = color.g;
        int b = color.b;
        fs << static_cast<float> (r) / 255.0f << " " << static_cast<float> (g) / 255.0f << " " << static_cast<float> (b) / 255.0f;
      }
      fs << '\n';
    }
  }

  // Close file
  fs.close ();
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::saveVTKFile (const std::string &file_name, 
                      const pcl::PCLPointCloud2 &cloud, unsigned precision)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no data!\n");
    return (-1);
  }

  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  unsigned int nr_points  = cloud.width * cloud.height;
  auto point_size = static_cast<unsigned int> (cloud.data.size () / nr_points);

  // Write the header information
  fs << "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS " << nr_points << " float" << '\n';

  // Iterate through the points
  for (unsigned int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (std::size_t d = 0; d < cloud.fields.size (); ++d)
    {
      if ((cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
           cloud.fields[d].name == "x" || 
           cloud.fields[d].name == "y" || 
           cloud.fields[d].name == "z"))
      {
        float value;
        memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
      }
      fs << " ";
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << '\n';
  }

  // Write vertices
  fs << "\nVERTICES " << nr_points << " " << 2*nr_points << '\n';
  for (unsigned int i = 0; i < nr_points; ++i)
    fs << "1 " << i << '\n';

  // Write RGB values
  int field_index = getFieldIndex (cloud, "rgb");
  if (field_index != -1)
  {
    fs << "\nPOINT_DATA " << nr_points << "\nCOLOR_SCALARS scalars 3\n";
    for (unsigned int i = 0; i < nr_points; ++i)
    {
      if (cloud.fields[field_index].datatype == pcl::PCLPointField::FLOAT32)
      {
        pcl::RGB color;
        memcpy (&color, &cloud.data[i * point_size + cloud.fields[field_index].offset], sizeof (RGB));
        int r = color.r;
        int g = color.g;
        int b = color.b;
        fs << static_cast<float> (r) / 255.0f << " " << static_cast<float> (g) / 255.0f << " " << static_cast<float> (b) / 255.0f;
      }
      fs << '\n';
    }
  }

  // Close file
  fs.close ();
  return (0);
}


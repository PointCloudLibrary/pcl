/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2013, Open Perception, Inc.
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
 */
#include <pcl/io/obj_io.h>
#include <fstream>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/boost.h>
#include <boost/lexical_cast.hpp>
#include <pcl/console/time.h>

pcl::MTLReader::MTLReader ()
{
  xyz_to_rgb_matrix_ << 2.3706743, -0.9000405, -0.4706338,
                       -0.5138850,  1.4253036,  0.0885814,
                        0.0052982, -0.0146949,  1.0093968;
}

inline void
pcl::MTLReader::cie2rgb (const Eigen::Vector3f &xyz, pcl::TexMaterial::RGB& rgb) const
{
  Eigen::Vector3f rgb_vec = xyz_to_rgb_matrix_ * xyz;
  rgb.r = rgb_vec[0]; rgb.g = rgb_vec[1]; rgb.b = rgb_vec[2];
}

int
pcl::MTLReader::fillRGBfromXYZ (const std::vector<std::string>& split_line,
                                pcl::TexMaterial::RGB& rgb)
{
  Eigen::Vector3f xyz;
  if (split_line.size () == 5)
  {
    try
    {
      xyz[0] = boost::lexical_cast<float> (split_line[2]);
      xyz[1] = boost::lexical_cast<float> (split_line[3]);
      xyz[2] = boost::lexical_cast<float> (split_line[4]);
    }
    catch (boost::bad_lexical_cast &)
    {
      return (-1);
    }
  }
  else
    if (split_line.size () == 3)
    {
      try
      {
        xyz[0] = xyz[1] = xyz[2] = boost::lexical_cast<float> (split_line[2]);
      }
      catch (boost::bad_lexical_cast &)
      {
        return (-1);
      }
    }
    else
      return (-1);

  cie2rgb (xyz, rgb);
  return (0);
}

int
pcl::MTLReader::fillRGBfromRGB (const std::vector<std::string>& split_line,
                                pcl::TexMaterial::RGB& rgb)
{
  if (split_line.size () == 4)
  {
    try
    {
      rgb.r = boost::lexical_cast<float> (split_line[1]);
      rgb.g = boost::lexical_cast<float> (split_line[2]);
      rgb.b = boost::lexical_cast<float> (split_line[3]);
    }
    catch (boost::bad_lexical_cast &)
    {
      rgb.r = rgb.g = rgb.b = 0;
      return (-1);
    }
  }
  else
    if (split_line.size () == 2)
    {
      try
      {
        rgb.r = rgb.g = rgb.b = boost::lexical_cast<float> (split_line[1]);
      }
      catch (boost::bad_lexical_cast &)
      {
        return (-1);
      }
    }
    else
      return (-1);

  return (0);
}

std::vector<pcl::TexMaterial>::const_iterator
pcl::MTLReader::getMaterial (const std::string& material_name) const
{
  std::vector<pcl::TexMaterial>::const_iterator mat_it = materials_.begin ();
  for (; mat_it != materials_.end (); ++mat_it)
    if (mat_it->tex_name == material_name)
      break;
  return (mat_it);
}

int
pcl::MTLReader::read (const std::string& obj_file_name,
                      const std::string& mtl_file_name)
{
  if (obj_file_name == "" || !boost::filesystem::exists (obj_file_name))
  {
    PCL_ERROR ("[pcl::MTLReader::read] Could not find file '%s'!\n",
               obj_file_name.c_str ());
    return (-1);
  }

  if (mtl_file_name == "")
  {
    PCL_ERROR ("[pcl::MTLReader::read] MTL file name is empty!\n");
    return (-1);
  }

  boost::filesystem::path obj_file_path (obj_file_name.c_str ());
  boost::filesystem::path mtl_file_path = obj_file_path.parent_path ();
  mtl_file_path /=  mtl_file_name;
  return (read (mtl_file_path.string ()));
}

int
pcl::MTLReader::read (const std::string& mtl_file_path)
{
  if (mtl_file_path == "" || !boost::filesystem::exists (mtl_file_path))
  {
    PCL_ERROR ("[pcl::MTLReader::read] Could not find file '%s'.\n", mtl_file_path.c_str ());
    return (-1);
  }

  std::ifstream mtl_file;
  mtl_file.open (mtl_file_path.c_str (), std::ios::binary);
  if (!mtl_file.is_open () || mtl_file.fail ())
  {
    PCL_ERROR ("[pcl::MTLReader::read] Could not open file '%s'! Error : %s\n",
               mtl_file_path.c_str (), strerror(errno));
    mtl_file.close ();
    return (-1);
  }

  std::string line;
  std::vector<std::string> st;
  boost::filesystem::path parent_path = mtl_file_path.c_str ();
  parent_path = parent_path.parent_path ();

  try
  {
    while (!mtl_file.eof ())
    {
      getline (mtl_file, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());
      line = sstream.str ();
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
      // Ignore comments
      if (st[0] == "#")
        continue;

      if (st[0] == "newmtl")
      {
        materials_.push_back (pcl::TexMaterial ());
        materials_.back ().tex_name = st[1];
        continue;
      }

      if (st[0] == "Ka" || st[0] == "Kd" || st[0] == "Ks")
      {
        if (st[1] == "spectral")
        {
          PCL_ERROR ("[pcl::MTLReader::read] Can't handle spectral files!\n");
          mtl_file.close ();
          materials_.clear ();
          return (-1);
        }
        else
        {
          pcl::TexMaterial::RGB *rgb = &materials_.back ().tex_Ka;
          if (st[0] == "Kd")
            rgb = &materials_.back ().tex_Kd;
          else if (st[0] == "Ks")
            rgb = &materials_.back ().tex_Ks;

          if (st[1] == "xyz")
          {
            if (fillRGBfromXYZ (st, *rgb))
            {
              PCL_ERROR ("[pcl::MTLReader::read] Could not convert %s to RGB values",
                         line.c_str ());
              mtl_file.close ();
              materials_.clear ();
              return (-1);
            }
          }
          else
          {
            if (fillRGBfromRGB (st, *rgb))
            {
              PCL_ERROR ("[pcl::MTLReader::read] Could not convert %s to RGB values",
                         line.c_str ());
              mtl_file.close ();
              materials_.clear ();
              return (-1);
            }
          }
        }
        continue;
      }

      if (st[0] == "illum")
      {
        try
        {
          materials_.back ().tex_illum = boost::lexical_cast<int> (st[1]);
        }
        catch (boost::bad_lexical_cast &)
        {
          PCL_ERROR ("[pcl::MTLReader::read] Could not convert %s to illumination model",
                     line.c_str ());
          mtl_file.close ();
          materials_.clear ();
          return (-1);
        }
        continue;
      }

      if (st[0] == "d" || st[0] == "Tr")
      {
        bool reverse = (st[0] == "Tr");
        try
        {
          materials_.back ().tex_d = boost::lexical_cast<float> (st[st.size () > 2 ? 2:1]);
          if (reverse)
            materials_.back ().tex_d = 1.f - materials_.back ().tex_d;
        }
        catch (boost::bad_lexical_cast &)
        {
          PCL_ERROR ("[pcl::MTLReader::read] Could not convert %s to transparency value",
                     line.c_str ());
          mtl_file.close ();
          materials_.clear ();
          return (-1);
        }
        continue;
      }

      if (st[0] == "Ns")
      {
        try
        {
          materials_.back ().tex_Ns = boost::lexical_cast<float> (st[1]);
        }
        catch (boost::bad_lexical_cast &)
        {
          PCL_ERROR ("[pcl::MTLReader::read] Could not convert %s to shininess value",
                     line.c_str ());
          mtl_file.close ();
          materials_.clear ();
          return (-1);
        }
        continue;
      }

      if (st[0] == "map_Kd")
      {
        boost::filesystem::path full_path = parent_path;
        full_path/= st.back ().c_str ();
        materials_.back ().tex_file = full_path.string ();
        continue;
      }
      // other elements? we don't care for now
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::MTLReader::read] %s\n", exception);
    mtl_file.close ();
    materials_.clear ();
    return (-1);
  }

  return (0);
}

int
pcl::OBJReader::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                            int &file_version, int &data_type, unsigned int &data_idx,
                            const int offset)
{
  origin       = Eigen::Vector4f::Zero ();
  orientation  = Eigen::Quaternionf::Identity ();
  file_version = 0;
  cloud.width  = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();
  data_type = 0;
  data_idx = offset;

  std::ifstream fs;
  std::string line;

  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::OBJReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  // Open file in binary mode to avoid problem of
  // std::getline() corrupting the result of ifstream::tellg()
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::OBJReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno));
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (offset, std::ios::beg);

  // Read the header and fill it in with wonderful values
  bool vertex_normal_found = false;
  bool vertex_texture_found = false;
  // Material library, skip for now!
  // bool material_found = false;
  std::vector<std::string> material_files;
  std::size_t nr_point = 0;
  std::vector<std::string> st;

  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());
      line = sstream.str ();
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
      // Ignore comments
      if (st.at (0) == "#")
        continue;

      // Vertex
      if (st.at (0) == "v")
      {
        ++nr_point;
        continue;
      }

      // Vertex texture
      if ((st.at (0) == "vt") && !vertex_texture_found)
      {
        vertex_texture_found = true;
        continue;
      }

      // Vertex normal
      if ((st.at (0) == "vn") && !vertex_normal_found)
      {
        vertex_normal_found = true;
        continue;
      }

      // Material library, skip for now!
      if (st.at (0) == "mtllib")
      {
        material_files.push_back (st.at (1));
        continue;
      }
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::OBJReader::readHeader] %s\n", exception);
    fs.close ();
    return (-1);
  }

  if (!nr_point)
  {
    PCL_ERROR ("[pcl::OBJReader::readHeader] No vertices found!\n");
    fs.close ();
    return (-1);
  }

  int field_offset = 0;
  for (int i = 0; i < 3; ++i, field_offset += 4)
  {
    cloud.fields.push_back (pcl::PCLPointField ());
    cloud.fields[i].offset   = field_offset;
    cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
    cloud.fields[i].count    = 1;
  }

  cloud.fields[0].name = "x";
  cloud.fields[1].name = "y";
  cloud.fields[2].name = "z";

  if (vertex_normal_found)
  {
    std::string normals_names[3] = { "normal_x", "normal_y", "normal_z" };
    for (int i = 0; i < 3; ++i, field_offset += 4)
    {
      cloud.fields.push_back (pcl::PCLPointField ());
      pcl::PCLPointField& last = cloud.fields.back ();
      last.name     = normals_names[i];
      last.offset   = field_offset;
      last.datatype = pcl::PCLPointField::FLOAT32;
      last.count    = 1;
    }
  }

  if (material_files.size () > 0)
  {
    for (std::size_t i = 0; i < material_files.size (); ++i)
    {
      MTLReader companion;
      if (companion.read (file_name, material_files[i]))
        PCL_WARN ("[pcl::OBJReader::readHeader] Problem reading material file %s\n",
                  material_files[i].c_str ());
      companions_.push_back (companion);
    }
  }

  cloud.point_step = field_offset;
  cloud.width      = nr_point;
  cloud.height     = 1;
  cloud.row_step   = cloud.point_step * cloud.width;
  cloud.is_dense   = true;
  cloud.data.resize (cloud.point_step * nr_point);
  fs.close ();
  return (0);
}

int
pcl::OBJReader::read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset)
{
  int file_version;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  return (read (file_name, cloud, origin, orientation, file_version, offset));
}

int
pcl::OBJReader::read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                      int &file_version, const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  int data_type;
  unsigned int data_idx;
  if (readHeader (file_name, cloud, origin, orientation, file_version, data_type, data_idx, offset))
  {
    PCL_ERROR ("[pcl::OBJReader::read] Problem reading header!\n");
    return (-1);
  }

  std::ifstream fs;
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::OBJReader::readHeader] Could not open file '%s'! Error : %s\n",
               file_name.c_str (), strerror(errno));
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (data_idx, std::ios::beg);

  // Get normal_x and rgba fields indices
  int normal_x_field = -1;
  // std::size_t rgba_field = 0;
  for (std::size_t i = 0; i < cloud.fields.size (); ++i)
    if (cloud.fields[i].name == "normal_x")
    {
      normal_x_field = i;
      break;
    }


  // else if (cloud.fields[i].name == "rgba")
  //   rgba_field = i;

  std::size_t point_idx = 0;
  std::size_t normal_idx = 0;
  std::string line;
  std::vector<std::string> st;
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());
      line = sstream.str ();
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      // Ignore comments
      if (st[0] == "#")
        continue;

      // Vertex
      if (st[0] == "v")
      {
        try
        {
          for (int i = 1, f = 0; i < 4; ++i, ++f)
          {
            float value = boost::lexical_cast<float> (st[i]);
            memcpy (&cloud.data[point_idx * cloud.point_step + cloud.fields[f].offset],
                &value,
                sizeof (float));
          }
          ++point_idx;
        }
        catch (const boost::bad_lexical_cast &e)
        {
          PCL_ERROR ("Unable to convert %s to vertex coordinates!", line.c_str ());
          return (-1);
        }
        continue;
      }

      // Vertex normal
      if (st[0] == "vn")
      {
        try
        {
          for (int i = 1, f = normal_x_field; i < 4; ++i, ++f)
          {
            float value = boost::lexical_cast<float> (st[i]);
            memcpy (&cloud.data[normal_idx * cloud.point_step + cloud.fields[f].offset],
                &value,
                sizeof (float));
          }
          ++normal_idx;
        }
        catch (const boost::bad_lexical_cast &e)
        {
          PCL_ERROR ("Unable to convert line %s to vertex normal!", line.c_str ());
          return (-1);
        }
        continue;
      }
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::OBJReader::read] %s\n", exception);
    fs.close ();
    return (-1);
  }

  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::OBJReader::read] Loaded %s as a dense cloud in %g ms with %d points. Available dimensions: %s.\n",
             file_name.c_str (), total_time,
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  fs.close ();
  return (0);
}

int
pcl::OBJReader::read (const std::string &file_name, pcl::TextureMesh &mesh, const int offset)
{
  int file_version;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  return (read (file_name, mesh, origin, orientation, file_version, offset));
}

int
pcl::OBJReader::read (const std::string &file_name, pcl::TextureMesh &mesh,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                      int &file_version, const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  int data_type;
  unsigned int data_idx;
  if (readHeader (file_name, mesh.cloud, origin, orientation, file_version, data_type, data_idx, offset))
  {
    PCL_ERROR ("[pcl::OBJReader::read] Problem reading header!\n");
    return (-1);
  }

  std::ifstream fs;
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::OBJReader::readHeader] Could not open file '%s'! Error : %s\n",
               file_name.c_str (), strerror(errno));
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (data_idx, std::ios::beg);

  // Get normal_x and rgba fields indices
  int normal_x_field = -1;
  // std::size_t rgba_field = 0;
  for (std::size_t i = 0; i < mesh.cloud.fields.size (); ++i)
    if (mesh.cloud.fields[i].name == "normal_x")
    {
      normal_x_field = i;
      break;
    }

  std::size_t v_idx = 0;
  std::size_t vn_idx = 0;
  std::size_t vt_idx = 0;
  std::size_t f_idx = 0;
  std::string line;
  std::vector<std::string> st;
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > coordinates;
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());
      line = sstream.str ();
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      // Ignore comments
      if (st[0] == "#")
        continue;
      // Vertex
      if (st[0] == "v")
      {
        try
        {
          for (int i = 1, f = 0; i < 4; ++i, ++f)
          {
            float value = boost::lexical_cast<float> (st[i]);
            memcpy (&mesh.cloud.data[v_idx * mesh.cloud.point_step + mesh.cloud.fields[f].offset],
                &value,
                sizeof (float));
          }
          ++v_idx;
        }
        catch (const boost::bad_lexical_cast &e)
        {
          PCL_ERROR ("Unable to convert %s to vertex coordinates!", line.c_str ());
          return (-1);
        }
        continue;
      }
      // Vertex normal
      if (st[0] == "vn")
      {
        try
        {
          for (int i = 1, f = normal_x_field; i < 4; ++i, ++f)
          {
            float value = boost::lexical_cast<float> (st[i]);
            memcpy (&mesh.cloud.data[vn_idx * mesh.cloud.point_step + mesh.cloud.fields[f].offset],
                &value,
                sizeof (float));
          }
          ++vn_idx;
        }
        catch (const boost::bad_lexical_cast &e)
        {
          PCL_ERROR ("Unable to convert line %s to vertex normal!", line.c_str ());
          return (-1);
        }
        continue;
      }
      // Texture coordinates
      if (st[0] == "vt")
      {
        try
        {
          Eigen::Vector3f c (0, 0, 0);
          for (std::size_t i = 1; i < st.size (); ++i)
            c[i-1] = boost::lexical_cast<float> (st[i]);
          if (c[2] == 0)
            coordinates.push_back (Eigen::Vector2f (c[0], c[1]));
          else
            coordinates.push_back (Eigen::Vector2f (c[0]/c[2], c[1]/c[2]));
          ++vt_idx;
        }
        catch (const boost::bad_lexical_cast &e)
        {
          PCL_ERROR ("Unable to convert line %s to texture coordinates!", line.c_str ());
          return (-1);
        }
        continue;
      }
      // Material
      if (st[0] == "usemtl")
      {
        mesh.tex_polygons.push_back (std::vector<pcl::Vertices> ());
        mesh.tex_materials.push_back (pcl::TexMaterial ());
        for (std::size_t i = 0; i < companions_.size (); ++i)
        {
          std::vector<pcl::TexMaterial>::const_iterator mat_it = companions_[i].getMaterial (st[1]);
          if (mat_it != companions_[i].materials_.end ())
          {
            mesh.tex_materials.back () = *mat_it;
            break;
          }
        }
        // We didn't find the appropriate material so we create it here with name only.
        if (mesh.tex_materials.back ().tex_name == "")
          mesh.tex_materials.back ().tex_name = st[1];
        mesh.tex_coordinates.push_back (coordinates);
        coordinates.clear ();
        continue;
      }
      // Face
      if (st[0] == "f")
      {
        //We only care for vertices indices
        pcl::Vertices face_v; face_v.vertices.resize (st.size () - 1);
        for (std::size_t i = 1; i < st.size (); ++i)
        {
          int v;
          sscanf (st[i].c_str (), "%d", &v);
          v = (v < 0) ? v_idx + v : v - 1;
          face_v.vertices[i-1] = v;
        }
        mesh.tex_polygons.back ().push_back (face_v);
        ++f_idx;
        continue;
      }
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::OBJReader::read] %s\n", exception);
    fs.close ();
    return (-1);
  }

  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::OBJReader::read] Loaded %s as a TextureMesh in %g ms with %g points, %g texture materials, %g polygons.\n",
             file_name.c_str (), total_time,
             v_idx -1, mesh.tex_materials.size (), f_idx -1);
  fs.close ();
  return (0);
}

int
pcl::OBJReader::read (const std::string &file_name, pcl::PolygonMesh &mesh, const int offset)
{
  int file_version;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  return (read (file_name, mesh, origin, orientation, file_version, offset));
}

int
pcl::OBJReader::read (const std::string &file_name, pcl::PolygonMesh &mesh,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                      int &file_version, const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  int data_type;
  unsigned int data_idx;
  if (readHeader (file_name, mesh.cloud, origin, orientation, file_version, data_type, data_idx, offset))
  {
    PCL_ERROR ("[pcl::OBJReader::read] Problem reading header!\n");
    return (-1);
  }

  std::ifstream fs;
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::OBJReader::readHeader] Could not open file '%s'! Error : %s\n",
               file_name.c_str (), strerror(errno));
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (data_idx, std::ios::beg);

  // Get normal_x and rgba fields indices
  int normal_x_field = -1;
  // std::size_t rgba_field = 0;
  for (std::size_t i = 0; i < mesh.cloud.fields.size (); ++i)
    if (mesh.cloud.fields[i].name == "normal_x")
    {
      normal_x_field = i;
      break;
    }

  std::size_t v_idx = 0;
  std::size_t vn_idx = 0;
  std::string line;
  std::vector<std::string> st;
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());
      line = sstream.str ();
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      // Ignore comments
      if (st[0] == "#")
        continue;

      // Vertex
      if (st[0] == "v")
      {
        try
        {
          for (int i = 1, f = 0; i < 4; ++i, ++f)
          {
            float value = boost::lexical_cast<float> (st[i]);
            memcpy (&mesh.cloud.data[v_idx * mesh.cloud.point_step + mesh.cloud.fields[f].offset],
                &value,
                sizeof (float));
          }
          ++v_idx;
        }
        catch (const boost::bad_lexical_cast &e)
        {
          PCL_ERROR ("Unable to convert %s to vertex coordinates!", line.c_str ());
          return (-1);
        }
        continue;
      }

      // Vertex normal
      if (st[0] == "vn")
      {
        try
        {
          for (int i = 1, f = normal_x_field; i < 4; ++i, ++f)
          {
            float value = boost::lexical_cast<float> (st[i]);
            memcpy (&mesh.cloud.data[vn_idx * mesh.cloud.point_step + mesh.cloud.fields[f].offset],
                &value,
                sizeof (float));
          }
          ++vn_idx;
        }
        catch (const boost::bad_lexical_cast &e)
        {
          PCL_ERROR ("Unable to convert line %s to vertex normal!", line.c_str ());
          return (-1);
        }
        continue;
      }

      // Face
      if (st[0] == "f")
      {
        pcl::Vertices face_vertices; face_vertices.vertices.resize (st.size () - 1);
        for (std::size_t i = 1; i < st.size (); ++i)
        {
          int v;
          sscanf (st[i].c_str (), "%d", &v);
          v = (v < 0) ? v_idx + v : v - 1;
          face_vertices.vertices[i - 1] = v;
        }
        mesh.polygons.push_back (face_vertices);
        continue;
      }
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::OBJReader::read] %s\n", exception);
    fs.close ();
    return (-1);
  }

  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::OBJReader::read] Loaded %s as a PolygonMesh in %g ms with %g points and %g polygons.\n",
             file_name.c_str (), total_time,
             mesh.cloud.width * mesh.cloud.height, mesh.polygons.size ());
  fs.close ();
  return (0);
}

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
  fs << "####" << '\n';
  fs << "# OBJ dataFile simple version. File name: " << file_name << '\n';
  fs << "# Vertices: " << nr_points << '\n';
  fs << "# Faces: " <<nr_faces << '\n';
  fs << "# Material information:" << '\n';
  fs << "mtllib " << mtl_file_name_nopath << '\n';
  fs << "####" << '\n';

  // Write vertex coordinates
  fs << "# Vertices" << '\n';
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
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
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
    fs << '\n';
  }
  fs << "# "<< nr_points <<" vertices" << '\n';

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
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
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
    fs << '\n';
  }
  // Write vertex texture with "vt" (adding latter)

  for (unsigned m = 0; m < nr_meshes; ++m)
  {
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  '\n';
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << '\n';
    }
  }

  unsigned f_idx = 0;

  // int idx_vt =0;
  for (unsigned m = 0; m < nr_meshes; ++m)
  {
    if (m > 0) f_idx += static_cast<unsigned> (tex_mesh.tex_polygons[m-1].size ());

    fs << "# The material will be used for mesh " << m << '\n';
    fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << '\n';
    fs << "# Faces" << '\n';

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
      fs << '\n';
    }
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << '\n';
  }
  fs << "# End of File" << std::flush;

  // Close obj file
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << '\n';
  m_fs << "# Wavefront material file" << '\n';
  m_fs << "#" << '\n';
  for(unsigned m = 0; m < nr_meshes; ++m)
  {
    m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << '\n';
    m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << '\n'; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << '\n'; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << '\n'; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << tex_mesh.tex_materials[m].tex_d << '\n'; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << '\n'; // defines the shininess of the material to be s.
    m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << '\n'; // denotes the illumination model used by the material.
                                            // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
                                            // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << '\n';
    m_fs << "###" << '\n';
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
  int normal_index = getFieldIndex (mesh.cloud, "normal_x");

  // Write the header information
  fs << "####" << '\n';
  fs << "# OBJ dataFile simple version. File name: " << file_name << '\n';
  fs << "# Vertices: " << nr_points << '\n';
  if (normal_index != -1)
    fs << "# Vertices normals : " << nr_points << '\n';
  fs << "# Faces: " <<nr_faces << '\n';
  fs << "####" << '\n';

  // Write vertex coordinates
  fs << "# List of Vertices, with (x,y,z) coordinates, w is optional." << '\n';
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
    {
      int c = 0;
      // adding vertex
      if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
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
    fs << '\n';
  }

  fs << "# "<< nr_points <<" vertices" << '\n';

  if(normal_index != -1)
  {
    fs << "# Normals in (x,y,z) form; normals might not be unit." <<  '\n';
    // Write vertex normals
    for (int i = 0; i < nr_points; ++i)
    {
      int nxyz = 0;
      for (size_t d = 0; d < mesh.cloud.fields.size (); ++d)
      {
        int c = 0;
        // adding vertex
        if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
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
      fs << '\n';
    }

    fs << "# "<< nr_points <<" vertices normals" << '\n';
  }

  fs << "# Face Definitions" << '\n';
  // Write down faces
  if(normal_index == -1)
  {
    for(unsigned i = 0; i < nr_faces; i++)
    {
      fs << "f ";
      size_t j = 0;
      for (; j < mesh.polygons[i].vertices.size () - 1; ++j)
        fs << mesh.polygons[i].vertices[j] + 1 << " ";
      fs << mesh.polygons[i].vertices[j] + 1 << '\n';
    }
  }
  else
  {
    for(unsigned i = 0; i < nr_faces; i++)
    {
      fs << "f ";
      size_t j = 0;
      for (; j < mesh.polygons[i].vertices.size () - 1; ++j)
        fs << mesh.polygons[i].vertices[j] + 1 << "//" << mesh.polygons[i].vertices[j] + 1 << " ";
      fs << mesh.polygons[i].vertices[j] + 1 << "//" << mesh.polygons[i].vertices[j] + 1 << '\n';
    }
  }
  fs << "# End of File" << std::endl;

  // Close obj file
  fs.close ();
  return 0;
}

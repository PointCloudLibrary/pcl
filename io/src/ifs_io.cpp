/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#include <fstream>
#include <pcl/io/boost.h>
#include <pcl/common/io.h>
#include <pcl/io/ifs_io.h>
#include <pcl/console/time.h>

#include <cstring>
#include <cerrno>

///////////////////////////////////////////////////////////////////////////////////////////
int
pcl::IFSReader::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                            int &ifs_version, unsigned int &data_idx)
{
  // Default values
  data_idx = 0;
  ifs_version = IFS_V1_0;
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  uint32_t nr_points = 0;
  std::ifstream fs;
  std::string line;

  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::IFSReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::IFSReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno));
    fs.close ();
    return (-1);
  }

  //Read the magic
  uint32_t length_of_magic;
  fs.read ((char*)&length_of_magic, sizeof (uint32_t));
  char *magic = new char [length_of_magic];
  fs.read (magic, sizeof (char) * length_of_magic);
  if (strcmp (magic, "IFS"))
  {
    PCL_ERROR ("[pcl::IFSReader::readHeader] File %s is not an IFS file!\n", file_name.c_str ());
    fs.close ();
    return (-1);
  }
  delete[] magic;

  //Read IFS version
  float version;
  fs.read ((char*)&version, sizeof (float));
  if (version == 1.0f)
    ifs_version = IFS_V1_0;
  else
    if (version == 1.1f)
      ifs_version = IFS_V1_1;
    else
    {
      PCL_ERROR ("[pcl::IFSReader::readHeader] Bad IFS file %f!\n", version);
      fs.close ();
      return (-1);
    }

  //Read the name
  uint32_t length_of_name;
  fs.read ((char*)&length_of_name, sizeof (uint32_t));
  char *name = new char [length_of_name];
  fs.read (name, sizeof (char) * length_of_name);
  delete[] name;
  int offset = 0;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!fs.eof ())
    {
      //Read the keyword
      uint32_t length_of_keyword;
      fs.read ((char*)&length_of_keyword, sizeof (uint32_t));
      char *keyword = new char [length_of_keyword];
      fs.read (keyword, sizeof (char) * length_of_keyword);

      if (strcmp (keyword, "VERTICES") == 0)
      {
        fs.read ((char*)&nr_points, sizeof (uint32_t));
        if ((nr_points == 0) || (nr_points > 10000000))
        {
          PCL_ERROR ("[pcl::IFSReader::readHeader] Bad number of vertices %zu!\n", nr_points);
          fs.close ();
          return (-1);
        }

        cloud.fields.resize (3);
        cloud.fields[0].name = "x";
        cloud.fields[1].name = "y";
        cloud.fields[2].name = "z";

        for (int i = 0; i < 3; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        cloud.data.resize (nr_points * cloud.point_step);
        data_idx = fs.tellg ();
        break;
      }
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::IFSReader::readHeader] %s\n", exception);
    fs.close ();
    return (-1);
  }

  // Set width and height
  cloud.width  = nr_points;
  cloud.height = 1;
  cloud.row_step = cloud.point_step * cloud.width;

  // Close file
  fs.close ();

  return (0);
}

/////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::IFSReader::read (const std::string &file_name,
                      pcl::PCLPointCloud2 &cloud, int &ifs_version)
{
  pcl::console::TicToc tt;
  tt.tic ();

  unsigned int data_idx;

  int res = readHeader (file_name, cloud, ifs_version, data_idx);

  if (res < 0)
    return (res);

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  boost::iostreams::mapped_file_source mapped_file;

  size_t data_size = data_idx + cloud.data.size ();

  try
  {
    mapped_file.open (file_name, data_size, 0);
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::IFSReader::read] Error : %s!\n", file_name.c_str (), exception);
    mapped_file.close ();
    return (-1);
  }

  if(!mapped_file.is_open ())
  {
    PCL_ERROR ("[pcl::IFSReader::read] File mapping failure\n");
    mapped_file.close ();
    return (-1);
  }

  // Copy the data
  memcpy (&cloud.data[0], mapped_file.data () + data_idx, cloud.data.size ());

  mapped_file.close ();

  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::IFSReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available dimensions: %s.\n",
             file_name.c_str (), cloud.is_dense ? "dense" : "non-dense", total_time,
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  return (0);
}

/////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::IFSReader::read (const std::string &file_name, pcl::PolygonMesh &mesh, int &ifs_version)
{
  pcl::console::TicToc tt;
  tt.tic ();

  unsigned int data_idx;

  int res = readHeader (file_name, mesh.cloud, ifs_version, data_idx);

  if (res < 0)
    return (res);

  // Setting the is_dense property to true by default
  mesh.cloud.is_dense = true;

  boost::iostreams::mapped_file_source mapped_file;

  size_t data_size = data_idx + mesh.cloud.data.size ();

  try
  {
    mapped_file.open (file_name, data_size, 0);
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::IFSReader::read] Error : %s!\n", file_name.c_str (), exception);
    mapped_file.close ();
    return (-1);
  }

  if(!mapped_file.is_open ())
  {
    PCL_ERROR ("[pcl::IFSReader::read] File mapping failure\n");
    mapped_file.close ();
    return (-1);
  }

  // Copy the data
  memcpy (&mesh.cloud.data[0], mapped_file.data () + data_idx, mesh.cloud.data.size ());

  mapped_file.close ();

  // Reopen the file to load the facets
  std::ifstream fs;
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::IFSReader::read] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno));
    fs.close ();
    return (-1);
  }
  // Jump to the end of cloud data
  fs.seekg (data_size);
  // Read the TRIANGLES keyword
  uint32_t length_of_keyword;
  fs.read ((char*)&length_of_keyword, sizeof (uint32_t));
  char *keyword = new char [length_of_keyword];
  fs.read (keyword, sizeof (char) * length_of_keyword);
  if (strcmp (keyword, "TRIANGLES"))
  {
    PCL_ERROR ("[pcl::IFSReader::read] File %s is does not contain facets!\n", file_name.c_str ());
    fs.close ();
    return (-1);
  }
  delete[] keyword;
  // Read the number of facets
  uint32_t nr_facets;
  fs.read ((char*)&nr_facets, sizeof (uint32_t));
  if ((nr_facets == 0) || (nr_facets > 10000000))
  {
    PCL_ERROR ("[pcl::IFSReader::read] Bad number of facets %zu!\n", nr_facets);
    fs.close ();
    return (-1);
  }
  // Resize the mesh polygons
  mesh.polygons.resize (nr_facets);
  // Fill each polygon
  for (uint32_t i = 0; i < nr_facets; ++i)
  {
    pcl::Vertices &facet = mesh.polygons[i];
    facet.vertices.resize (3);
    fs.read ((char*)&(facet.vertices[0]), sizeof (uint32_t));
    fs.read ((char*)&(facet.vertices[1]), sizeof (uint32_t));
    fs.read ((char*)&(facet.vertices[2]), sizeof (uint32_t));
  }
  // We are done, close the file
  fs.close ();
  // Display statistics
  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::IFSReader::read] Loaded %s as a polygon mesh in %g ms with %d points and %d facets.\n",
             file_name.c_str (), total_time, mesh.cloud.width * mesh.cloud.height, mesh.polygons.size ());
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::IFSWriter::write (const std::string &file_name, const pcl::PCLPointCloud2 &cloud, const std::string& cloud_name)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::IFSWriter::write] Input point cloud has no data!\n");
    return (-1);
  }

  if (!cloud.is_dense)
  {
    PCL_ERROR ("[pcl::IFSWriter::write] Non dense cloud are not alowed by IFS format!\n");
    return (-1);
  }

  const std::string magic = "IFS";
  const float version = 1.0f;
  const std::string vertices = "VERTICES";
  std::vector<char> header (sizeof (uint32_t) + magic.size () + 1 +
                            sizeof (float) +
                            sizeof (uint32_t) + cloud_name.size () + 1 +
                            sizeof (uint32_t) + vertices.size () + 1 +
                            sizeof (uint32_t));
  char* addr = &(header[0]);
  const uint32_t magic_size = static_cast<uint32_t> (magic.size ()) + 1;
  memcpy (addr, &magic_size, sizeof (uint32_t));
  addr+= sizeof (uint32_t);
  memcpy (addr, magic.c_str (), magic_size * sizeof (char));
  addr+= magic_size * sizeof (char);
  memcpy (addr, &version, sizeof (float));
  addr+= sizeof (float);
  const uint32_t cloud_name_size = static_cast<uint32_t> (cloud_name.size ()) + 1;
  memcpy (addr, &cloud_name_size, sizeof (uint32_t));
  addr+= sizeof (uint32_t);
  memcpy (addr, cloud_name.c_str (), cloud_name_size * sizeof (char));
  addr+= cloud_name_size * sizeof (char);
  const uint32_t vertices_size = static_cast<uint32_t> (vertices.size ()) + 1;
  memcpy (addr, &vertices_size, sizeof (uint32_t));
  addr+= sizeof (uint32_t);
  memcpy (addr, vertices.c_str (), vertices_size * sizeof (char));
  addr+= vertices_size * sizeof (char);
  const uint32_t nb_vertices = cloud.data.size () / cloud.point_step;
  memcpy (addr, &nb_vertices, sizeof (uint32_t));
  addr+= sizeof (uint32_t);

  std::size_t data_idx = header.size ();

  boost::iostreams::mapped_file_sink sink;
  boost::iostreams::mapped_file_params params;
  params.path = file_name;
  params.flags = boost::iostreams::mapped_file_base::readwrite;
  params.offset = 0;
  params.new_file_size = data_idx + cloud.data.size ();
  params.length = data_idx + cloud.data.size ();

  try
  {
    sink.open (params);
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::IFSWriter::write] Error : %s!\n", file_name.c_str (), exception);
    sink.close ();
    return (-1);
  }

  if (!sink.is_open ())
  {
    PCL_ERROR ("[pcl::IFSWriter::write] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno));
    sink.close ();
    return (-1);
  }

  // copy header
  memcpy (sink.data (), &header[0], data_idx);

  // Copy the data
  memcpy (sink.data () + data_idx, &cloud.data[0], cloud.data.size ());

  sink.close ();

  return (0);
}

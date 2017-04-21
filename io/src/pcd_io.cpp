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

#include <fstream>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <pcl/io/boost.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/lzf.h>
#include <pcl/console/time.h>

#include <cstring>
#include <cerrno>

#ifdef _WIN32
# include <io.h>
# include <windows.h>
# define pcl_open                    _open
# define pcl_close(fd)               _close(fd)
# define pcl_lseek(fd,offset,origin) _lseek(fd,offset,origin)
#else
# include <sys/mman.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)
#endif
#include <boost/version.hpp>

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PCDWriter::setLockingPermissions (const std::string &file_name,
                                       boost::interprocess::file_lock &lock)
{
  (void)file_name;
  (void)lock;
#ifndef WIN32
  // Boost version 1.49 introduced permissions
#if BOOST_VERSION >= 104900
  // Attempt to lock the file. 
  // For mandatory locking, the filesystem must be mounted with the "mand" option in Linux (see http://www.hackinglinuxexposed.com/articles/20030623.html)
  lock = boost::interprocess::file_lock (file_name.c_str ());
  if (lock.try_lock ())
    PCL_DEBUG ("[pcl::PCDWriter::setLockingPermissions] File %s locked successfully.\n", file_name.c_str ());
  else
    PCL_DEBUG ("[pcl::PCDWriter::setLockingPermissions] File %s could not be locked!\n", file_name.c_str ());

  namespace fs = boost::filesystem;
  try
  {
    fs::permissions (fs::path (file_name), fs::add_perms | fs::set_gid_on_exe);
  }
  catch (const std::exception &e)
  {
    PCL_DEBUG ("[pcl::PCDWriter::setLockingPermissions] Permissions on %s could not be set!\n", file_name.c_str ());
  }
#endif
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PCDWriter::resetLockingPermissions (const std::string &file_name,
                                         boost::interprocess::file_lock &lock)
{
  (void)file_name;
  (void)lock;
#ifndef WIN32
  // Boost version 1.49 introduced permissions
#if BOOST_VERSION >= 104900
  (void)file_name;
  namespace fs = boost::filesystem;
  try
  {
    fs::permissions (fs::path (file_name), fs::remove_perms | fs::set_gid_on_exe);
  }
  catch (const std::exception &e)
  {
    PCL_DEBUG ("[pcl::PCDWriter::resetLockingPermissions] Permissions on %s could not be reset!\n", file_name.c_str ());
  }
  lock.unlock ();
#endif
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDReader::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, 
                            int &pcd_version, int &data_type, unsigned int &data_idx, const int offset)
{
  // Default values
  data_idx = 0;
  data_type = 0;
  pcd_version = PCD_V6;
  origin      = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  int nr_points = 0;
  std::ifstream fs;
  std::string line;

  int specified_channel_count = 0;

  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  // Open file in binary mode to avoid problem of 
  // std::getline() corrupting the result of ifstream::tellg()
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno)); 
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (offset, std::ios::beg);

  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;
  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        cloud.fields.resize (specified_channel_count);
        for (int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at (i + 1);
          cloud.fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;                // estimate and save the data offsets
          offset += col_type;
          field_sizes[i] = col_type;                      // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
        continue;
      }

      // Get the field types
      if (line_type.substr (0, 4) == "TYPE")
      {
        if (field_sizes.empty ())
          throw "TYPE of FIELDS specified before SIZE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = static_cast<uint8_t> (getFieldType (field_sizes[i], field_types[i]));
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }
        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
        continue;
      }

      // Get the width of the data (organized point cloud dataset)
      if (line_type.substr (0, 5) == "WIDTH")
      {
        sstream >> cloud.width;
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        sstream >> cloud.height;
        continue;
      }

      // Get the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        pcd_version = PCD_V7;
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";

        float x, y, z, w;
        sstream >> x >> y >> z ;
        origin      = Eigen::Vector4f (x, y, z, 0.0f);
        sstream >> w >> x >> y >> z;
        orientation = Eigen::Quaternionf (w, x, y, z);
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        sstream >> nr_points;
        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }

      // Read the header + comments line by line until we get to <DATA>
      if (line_type.substr (0, 4) == "DATA")
      {
        data_idx = static_cast<int> (fs.tellg ());
        if (st.at (1).substr (0, 17) == "binary_compressed")
         data_type = 2;
        else
          if (st.at (1).substr (0, 6) == "binary")
            data_type = 1;
        continue;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
    fs.close ();
    return (-1);
  }

  // Exit early: if no points have been given, there's no sense to read or check anything anymore
  if (nr_points == 0)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] No points to read\n");
    fs.close ();
    return (-1);
  }
  
  // Compatibility with older PCD file versions
  if (cloud.width == 0 && cloud.height == 0)
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
  }
  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if (cloud.height == 0)
  {
    cloud.height = 1;
    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
    if (cloud.width == 0)
      cloud.width  = nr_points;
  }
  else
  {
    if (cloud.width == 0 && nr_points != 0)
    {
      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      fs.close ();
      return (-1);
    }
  }

  if (int (cloud.width * cloud.height) != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    fs.close ();
    return (-1);
  }

  // Close file
  fs.close ();

  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDReader::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset)
{
  // Default values
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  int nr_points = 0;
  std::ifstream fs;
  std::string line;

  int specified_channel_count = 0;

  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  // Open file in binary mode to avoid problem of 
  // std::getline() corrupting the result of ifstream::tellg()
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno)); 
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (offset, std::ios::beg);

  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;
  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        cloud.fields.resize (specified_channel_count);
        for (int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at (i + 1);
          cloud.fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;                // estimate and save the data offsets
          offset += col_type;
          field_sizes[i] = col_type;                      // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
        continue;
      }

      // Get the field types
      if (line_type.substr (0, 4) == "TYPE")
      {
        if (field_sizes.empty ())
          throw "TYPE of FIELDS specified before SIZE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = static_cast<uint8_t> (getFieldType (field_sizes[i], field_types[i]));
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }
        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
        continue;
      }

      // Get the width of the data (organized point cloud dataset)
      if (line_type.substr (0, 5) == "WIDTH")
      {
        sstream >> cloud.width;
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        sstream >> cloud.height;
        continue;
      }

      // Check the format of the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        sstream >> nr_points;
        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
    fs.close ();
    return (-1);
  }

  // Exit early: if no points have been given, there's no sense to read or check anything anymore
  if (nr_points == 0)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] No points to read\n");
    fs.close ();
    return (-1);
  }
  
  // Compatibility with older PCD file versions
  if (cloud.width == 0 && cloud.height == 0)
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
  }
  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if (cloud.height == 0)
  {
    cloud.height = 1;
    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
    if (cloud.width == 0)
      cloud.width  = nr_points;
  }
  else
  {
    if (cloud.width == 0 && nr_points != 0)
    {
      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      fs.close ();
      return (-1);
    }
  }

  if (int (cloud.width * cloud.height) != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    fs.close ();
    return (-1);
  }

  // Close file
  fs.close ();

  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDReader::read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version, 
                      const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  int data_type;
  unsigned int data_idx;

  int res = readHeader (file_name, cloud, origin, orientation, pcd_version, data_type, data_idx, offset);

  if (res < 0)
    return (res);

  unsigned int idx = 0;

  // Get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PCDReader::read] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }
  
  // if ascii
  if (data_type == 0)
  {
    // Re-open the file (readHeader closes it)
    std::ifstream fs;
    fs.open (file_name.c_str ());
    if (!fs.is_open () || fs.fail ())
    {
      PCL_ERROR ("[pcl::PCDReader::read] Could not open file %s.\n", file_name.c_str ());
      return (-1);
    }

    fs.seekg (data_idx);

    std::string line;
    std::vector<std::string> st;

    // Read the rest of the file
    try
    {
      while (idx < nr_points && !fs.eof ())
      {
        getline (fs, line);
        // Ignore empty lines
        if (line == "")
          continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
        
        if (idx >= nr_points)
        {
          PCL_WARN ("[pcl::PCDReader::read] input file %s has more points (%d) than advertised (%d)!\n", file_name.c_str (), idx, nr_points);
          break;
        }

        size_t total = 0;
        // Copy data
        for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
        {
          // Ignore invalid padded dimensions that are inherited from binary data
          if (cloud.fields[d].name == "_")
          {
            total += cloud.fields[d].count; // jump over this many elements in the string token
            continue;
          }
          for (unsigned int c = 0; c < cloud.fields[d].count; ++c)
          {
            switch (cloud.fields[d].datatype)
            {
              case pcl::PCLPointField::INT8:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT8>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT8:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT8>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::INT16:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT16>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT16:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT16>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::INT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::FLOAT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::FLOAT64:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              default:
                PCL_WARN ("[pcl::PCDReader::read] Incorrect field data type specified (%d)!\n",cloud.fields[d].datatype);
                break;
            }
          }
          total += cloud.fields[d].count; // jump over this many elements in the string token
        }
        idx++;
      }
    }
    catch (const char *exception)
    {
      PCL_ERROR ("[pcl::PCDReader::read] %s\n", exception);
      fs.close ();
      return (-1);
    }

    // Close file
    fs.close ();
  }
  else 
  /// ---[ Binary mode only
  /// We must re-open the file and read with mmap () for binary
  {
    // Open for reading
    int fd = pcl_open (file_name.c_str (), O_RDONLY);
    if (fd == -1)
    {
      PCL_ERROR ("[pcl::PCDReader::read] Failure to open file %s\n", file_name.c_str () );
      return (-1);
    }
    
    // Seek at the given offset
    off_t result = pcl_lseek (fd, offset, SEEK_SET);
    if (result < 0)
    {
      pcl_close (fd);
      PCL_ERROR ("[pcl::PCDReader::read] lseek errno: %d strerror: %s\n", errno, strerror (errno));
      PCL_ERROR ("[pcl::PCDReader::read] Error during lseek ()!\n");
      return (-1);
    }
    
    size_t data_size = data_idx + cloud.data.size ();
    // Prepare the map
#ifdef _WIN32
    // As we don't know the real size of data (compressed or not), 
    // we set dwMaximumSizeHigh = dwMaximumSizeLow = 0 so as to map the whole file
    HANDLE fm = CreateFileMapping ((HANDLE) _get_osfhandle (fd), NULL, PAGE_READONLY, 0, 0, NULL);
    // As we don't know the real size of data (compressed or not), 
    // we set dwNumberOfBytesToMap = 0 so as to map the whole file
    char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ, 0, 0, 0));
    if (map == NULL)
    {
      CloseHandle (fm);
      pcl_close (fd);
      PCL_ERROR ("[pcl::PCDReader::read] Error mapping view of file, %s\n", file_name.c_str ());
      return (-1);
    }
#else
    char *map = static_cast<char*> (mmap (0, data_size, PROT_READ, MAP_SHARED, fd, 0));
    if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
    {
      pcl_close (fd);
      PCL_ERROR ("[pcl::PCDReader::read] Error preparing mmap for binary PCD file.\n");
      return (-1);
    }
#endif

    /// ---[ Binary compressed mode only
    if (data_type == 2)
    {
      // Uncompress the data first
      unsigned int compressed_size, uncompressed_size;
      memcpy (&compressed_size, &map[data_idx + 0], sizeof (unsigned int));
      memcpy (&uncompressed_size, &map[data_idx + 4], sizeof (unsigned int));
      PCL_DEBUG ("[pcl::PCDReader::read] Read a binary compressed file with %u bytes compressed and %u original.\n", compressed_size, uncompressed_size);
      // For all those weird situations where the compressed data is actually LARGER than the uncompressed one
      // (we really ought to check this in the compressor and copy the original data in those cases)
      if (data_size < compressed_size || uncompressed_size < compressed_size)
      {
        PCL_DEBUG ("[pcl::PCDReader::read] Allocated data size (%lu) or uncompressed size (%lu) smaller than compressed size (%u). Need to remap.\n", data_size, uncompressed_size, compressed_size);
#ifdef _WIN32
        UnmapViewOfFile (map);
        data_size = compressed_size + data_idx + 8;
        map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ, 0, 0, data_size));
#else
        munmap (map, data_size);
        data_size = compressed_size + data_idx + 8;
        map = static_cast<char*> (mmap (0, data_size, PROT_READ, MAP_SHARED, fd, 0));
#endif
      }

      if (uncompressed_size != cloud.data.size ())
      {
        PCL_WARN ("[pcl::PCDReader::read] The estimated cloud.data size (%u) is different than the saved uncompressed value (%u)! Data corruption?\n", 
                  cloud.data.size (), uncompressed_size);
        cloud.data.resize (uncompressed_size);
      }

      char *buf = static_cast<char*> (malloc (data_size));
      // The size of the uncompressed data better be the same as what we stored in the header
      unsigned int tmp_size = pcl::lzfDecompress (&map[data_idx + 8], compressed_size, buf, static_cast<unsigned int> (data_size));
      if (tmp_size != uncompressed_size)
      {
        free (buf);
        pcl_close (fd);
        PCL_ERROR ("[pcl::PCDReader::read] Size of decompressed lzf data (%u) does not match value stored in PCD header (%u). Errno: %d\n", tmp_size, uncompressed_size, errno);
        return (-1);
      }

      // Get the fields sizes
      std::vector<pcl::PCLPointField> fields (cloud.fields.size ());
      std::vector<int> fields_sizes (cloud.fields.size ());
      int nri = 0, fsize = 0;
      for (size_t i = 0; i < cloud.fields.size (); ++i)
      {
        if (cloud.fields[i].name == "_")
          continue;
        fields_sizes[nri] = cloud.fields[i].count * pcl::getFieldSize (cloud.fields[i].datatype);
        fsize += fields_sizes[nri];
        fields[nri] = cloud.fields[i];
        ++nri;
      }
      fields.resize (nri);
      fields_sizes.resize (nri);

      // Unpack the xxyyzz to xyz
      std::vector<char*> pters (fields.size ());
      int toff = 0;
      for (size_t i = 0; i < pters.size (); ++i)
      {
        pters[i] = &buf[toff];
        toff += fields_sizes[i] * cloud.width * cloud.height;
      }
      // Copy it to the cloud
      for (size_t i = 0; i < cloud.width * cloud.height; ++i)
      {
        for (size_t j = 0; j < pters.size (); ++j)
        {
          memcpy (&cloud.data[i * fsize + fields[j].offset], pters[j], fields_sizes[j]);
          // Increment the pointer
          pters[j] += fields_sizes[j];
        }
      }
      //memcpy (&cloud.data[0], &buf[0], uncompressed_size);

      free (buf);
    }
    else
      // Copy the data
      memcpy (&cloud.data[0], &map[0] + data_idx, cloud.data.size ());

    // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile (map);
    CloseHandle (fm);
#else
    if (munmap (map, data_size) == -1)
    {
      pcl_close (fd);
      PCL_ERROR ("[pcl::PCDReader::read] Munmap failure\n");
      return (-1);
    }
#endif
    pcl_close (fd);
  }

  if ((idx != nr_points) && (data_type == 0))
  {
    PCL_ERROR ("[pcl::PCDReader::read] Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
    return (-1);
  }

  // No need to do any extra checks if the data type is ASCII
  if (data_type != 0)
  {
    int point_size = static_cast<int> (cloud.data.size () / (cloud.height * cloud.width));
    // Once copied, we need to go over each field and check if it has NaN/Inf values and assign cloud.is_dense to true or false
    for (uint32_t i = 0; i < cloud.width * cloud.height; ++i)
    {
      for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
      {
        for (uint32_t c = 0; c < cloud.fields[d].count; ++c)
        {
          switch (cloud.fields[d].datatype)
          {
            case pcl::PCLPointField::INT8:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT8>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT8:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT8>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::INT16:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT16>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT16:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT16>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::INT32:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT32:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::FLOAT32:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::FLOAT64:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
          }
        }
      }
    }
  }
  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::PCDReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available dimensions: %s.\n", 
             file_name.c_str (), cloud.is_dense ? "dense" : "non-dense", total_time, 
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDReader::read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset)
{
  int pcd_version;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  // Load the data
  int res = read (file_name, cloud, origin, orientation, pcd_version, offset);

  if (res < 0)
    return (res);

  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::PCDWriter::generateHeaderASCII (const pcl::PCLPointCloud2 &cloud,
                                     const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation)
{
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << "# .PCD v0.7 - Point Cloud Data file format"
         "\nVERSION 0.7"
         "\nFIELDS ";

  std::ostringstream stream;
  stream.imbue (std::locale::classic ());
  std::string result;

  for (size_t d = 0; d < cloud.fields.size () - 1; ++d)
  {
    // Ignore invalid padded dimensions that are inherited from binary data
    if (cloud.fields[d].name != "_")
      result += cloud.fields[d].name + " ";
  }
  // Ignore invalid padded dimensions that are inherited from binary data
  if (cloud.fields[cloud.fields.size () - 1].name != "_")
    result += cloud.fields[cloud.fields.size () - 1].name;

  // Remove trailing spaces
  boost::trim (result);
  oss << result << "\nSIZE ";

  stream.str ("");
  // Write the SIZE of each field
  for (size_t d = 0; d < cloud.fields.size () - 1; ++d)
  {
    // Ignore invalid padded dimensions that are inherited from binary data
    if (cloud.fields[d].name != "_")
      stream << pcl::getFieldSize (cloud.fields[d].datatype) << " ";
  }
  // Ignore invalid padded dimensions that are inherited from binary data
  if (cloud.fields[cloud.fields.size () - 1].name != "_")
    stream << pcl::getFieldSize (cloud.fields[cloud.fields.size () - 1].datatype);

  // Remove trailing spaces
  result = stream.str ();
  boost::trim (result);
  oss << result << "\nTYPE ";

  stream.str ("");
  // Write the TYPE of each field
  for (size_t d = 0; d < cloud.fields.size () - 1; ++d)
  {
    // Ignore invalid padded dimensions that are inherited from binary data
    if (cloud.fields[d].name != "_")
    {
      if (cloud.fields[d].name == "rgb")
        stream << "U ";
      else
        stream << pcl::getFieldType (cloud.fields[d].datatype) << " ";
    }
  }
  // Ignore invalid padded dimensions that are inherited from binary data
  if (cloud.fields[cloud.fields.size () - 1].name != "_")
  {
    if (cloud.fields[cloud.fields.size () - 1].name == "rgb")
      stream << "U";
    else
      stream << pcl::getFieldType (cloud.fields[cloud.fields.size () - 1].datatype);
  }

  // Remove trailing spaces
  result = stream.str ();
  boost::trim (result);
  oss << result << "\nCOUNT ";
  
  stream.str ("");
  // Write the TYPE of each field
  for (size_t d = 0; d < cloud.fields.size () - 1; ++d)
  {
    // Ignore invalid padded dimensions that are inherited from binary data
    if (cloud.fields[d].name == "_")
      continue;
    int count = abs (static_cast<int> (cloud.fields[d].count));
    if (count == 0) 
      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
  
    stream << count << " ";
  }
  // Ignore invalid padded dimensions that are inherited from binary data
  if (cloud.fields[cloud.fields.size () - 1].name != "_")
  {
    int count = abs (static_cast<int> (cloud.fields[cloud.fields.size () - 1].count));
    if (count == 0)
      count = 1;

    stream << count;
  }

  // Remove trailing spaces
  result = stream.str ();
  boost::trim (result);
  oss << result << "\nWIDTH " << cloud.width << "\nHEIGHT " << cloud.height << "\n";

  oss << "VIEWPOINT " << origin[0] << " " << origin[1] << " " << origin[2] << " " << orientation.w () << " " << 
                         orientation.x () << " " << orientation.y () << " " << orientation.z () << "\n";
  
  oss << "POINTS " << cloud.width * cloud.height << "\n";

  return (oss.str ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::PCDWriter::generateHeaderBinary (const pcl::PCLPointCloud2 &cloud,
                                      const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation)
{
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << "# .PCD v0.7 - Point Cloud Data file format"
         "\nVERSION 0.7"
         "\nFIELDS";

  // Compute the total size of the fields
  unsigned int fsize = 0;
  for (size_t i = 0; i < cloud.fields.size (); ++i)
    fsize += cloud.fields[i].count * getFieldSize (cloud.fields[i].datatype);
 
  // The size of the fields cannot be larger than point_step
  if (fsize > cloud.point_step)
  {
    PCL_ERROR ("[pcl::PCDWriter::generateHeaderBinary] The size of the fields (%d) is larger than point_step (%d)! Something is wrong here...\n", fsize, cloud.point_step);
    return ("");
  }

  std::stringstream field_names, field_types, field_sizes, field_counts;
  // Check if the size of the fields is smaller than the size of the point step
  unsigned int toffset = 0;
  for (size_t i = 0; i < cloud.fields.size (); ++i)
  {
    // If field offsets do not match, then we need to create fake fields
    if (toffset != cloud.fields[i].offset)
    {
      // If we're at the last "valid" field
      int fake_offset = (i == 0) ? 
        // Use the current_field offset
        (cloud.fields[i].offset)
        :
        // Else, do cur_field.offset - prev_field.offset + sizeof (prev_field)
        (cloud.fields[i].offset - 
        (cloud.fields[i-1].offset + 
         cloud.fields[i-1].count * getFieldSize (cloud.fields[i].datatype)));
      
      toffset += fake_offset;

      field_names << " _";  // By convention, _ is an invalid field name
      field_sizes << " 1";  // Make size = 1
      field_types << " U";  // Field type = unsigned byte (uint8)
      field_counts << " " << fake_offset;
    }

    // Add the regular dimension
    toffset += cloud.fields[i].count * getFieldSize (cloud.fields[i].datatype);
    field_names << " " << cloud.fields[i].name;
    field_sizes << " " << pcl::getFieldSize (cloud.fields[i].datatype);
    field_types << " " << pcl::getFieldType (cloud.fields[i].datatype);
    int count = abs (static_cast<int> (cloud.fields[i].count));
    if (count == 0) count = 1;  // check for 0 counts (coming from older converter code)
    field_counts << " " << count;
  }
  // Check extra padding
  if (toffset < cloud.point_step)
  {
    field_names << " _";  // By convention, _ is an invalid field name
    field_sizes << " 1";  // Make size = 1
    field_types << " U";  // Field type = unsigned byte (uint8)
    field_counts << " " << (cloud.point_step - toffset);
  }
  oss << field_names.str ();
  oss << "\nSIZE" << field_sizes.str () 
      << "\nTYPE" << field_types.str () 
      << "\nCOUNT" << field_counts.str ();
  oss << "\nWIDTH " << cloud.width << "\nHEIGHT " << cloud.height << "\n";

  oss << "VIEWPOINT " << origin[0] << " " << origin[1] << " " << origin[2] << " " << orientation.w () << " " << 
                         orientation.x () << " " << orientation.y () << " " << orientation.z () << "\n";
  
  oss << "POINTS " << cloud.width * cloud.height << "\n";

  return (oss.str ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::PCDWriter::generateHeaderBinaryCompressed (const pcl::PCLPointCloud2 &cloud,
                                                const Eigen::Vector4f &origin, 
                                                const Eigen::Quaternionf &orientation)
{
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << "# .PCD v0.7 - Point Cloud Data file format"
         "\nVERSION 0.7"
         "\nFIELDS";

  // Compute the total size of the fields
  unsigned int fsize = 0;
  for (size_t i = 0; i < cloud.fields.size (); ++i)
    fsize += cloud.fields[i].count * getFieldSize (cloud.fields[i].datatype);
 
  // The size of the fields cannot be larger than point_step
  if (fsize > cloud.point_step)
  {
    PCL_ERROR ("[pcl::PCDWriter::generateHeaderBinaryCompressed] The size of the fields (%d) is larger than point_step (%d)! Something is wrong here...\n", fsize, cloud.point_step);
    return ("");
  }

  std::stringstream field_names, field_types, field_sizes, field_counts;
  // Check if the size of the fields is smaller than the size of the point step
  for (size_t i = 0; i < cloud.fields.size (); ++i)
  {
    if (cloud.fields[i].name == "_")
      continue;
    // Add the regular dimension
    field_names << " " << cloud.fields[i].name;
    field_sizes << " " << pcl::getFieldSize (cloud.fields[i].datatype);
    field_types << " " << pcl::getFieldType (cloud.fields[i].datatype);
    int count = abs (static_cast<int> (cloud.fields[i].count));
    if (count == 0) count = 1;  // check for 0 counts (coming from older converter code)
    field_counts << " " << count;
  }
  oss << field_names.str ();
  oss << "\nSIZE" << field_sizes.str () 
      << "\nTYPE" << field_types.str () 
      << "\nCOUNT" << field_counts.str ();
  oss << "\nWIDTH " << cloud.width << "\nHEIGHT " << cloud.height << "\n";

  oss << "VIEWPOINT " << origin[0] << " " << origin[1] << " " << origin[2] << " " << orientation.w () << " " << 
                         orientation.x () << " " << orientation.y () << " " << orientation.z () << "\n";
  
  oss << "POINTS " << cloud.width * cloud.height << "\n";

  return (oss.str ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDWriter::writeASCII (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                            const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation,
                            const int precision)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PCDWriter::writeASCII] Input point cloud has no data!\n");
    return (-1);
  }

  std::ofstream fs;
  fs.precision (precision);
  fs.imbue (std::locale::classic ());
  fs.open (file_name.c_str ());      // Open file
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR("[pcl::PCDWriter::writeASCII] Could not open file '%s' for writing! Error : %s\n", file_name.c_str (), strerror(errno)); 
    return (-1);
  }
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  int nr_points  = cloud.width * cloud.height;
  int point_size = static_cast<int> (cloud.data.size () / nr_points);

  // Write the header information
  fs << generateHeaderASCII (cloud, origin, orientation) << "DATA ascii\n";

  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());

  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
    {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (cloud.fields[d].name == "_")
        continue;

      int count = cloud.fields[d].count;
      if (count == 0) 
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)

      for (int c = 0; c < count; ++c)
      {
        switch (cloud.fields[d].datatype)
        {
          case pcl::PCLPointField::INT8:
          {
            copyValueString<pcl::traits::asType<pcl::PCLPointField::INT8>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            copyValueString<pcl::traits::asType<pcl::PCLPointField::UINT8>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            copyValueString<pcl::traits::asType<pcl::PCLPointField::INT16>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            copyValueString<pcl::traits::asType<pcl::PCLPointField::UINT16>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            copyValueString<pcl::traits::asType<pcl::PCLPointField::INT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            copyValueString<pcl::traits::asType<pcl::PCLPointField::UINT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            /*
             * Despite the float type, store the rgb field as uint32
             * because several fully opaque color values are mapped to
             * nan.
             */
            if ("rgb" == cloud.fields[d].name)
              copyValueString<pcl::traits::asType<pcl::PCLPointField::UINT32>::type>(cloud, i, point_size, d, c, stream);
            else
              copyValueString<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            copyValueString<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          default:
            PCL_WARN ("[pcl::PCDWriter::writeASCII] Incorrect field data type specified (%d)!\n", cloud.fields[d].datatype);
            break;
        }

        if (d < cloud.fields.size () - 1 || c < static_cast<int> (cloud.fields[d].count) - 1)
          stream << " ";
      }
    }
    // Copy the stream, trim it, and write it to disk
    std::string result = stream.str ();
    boost::trim (result);
    stream.str ("");
    fs << result << "\n";
  }
  fs.close ();              // Close file
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDWriter::writeBinary (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                             const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Input point cloud has no data!\n");
    return (-1);
  }
  std::streamoff data_idx = 0;
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << generateHeaderBinary (cloud, origin, orientation) << "DATA binary\n";
  oss.flush();
  data_idx = static_cast<unsigned int> (oss.tellp ());

#ifdef _WIN32
  HANDLE h_native_file = CreateFile (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during CreateFile (%s)!\n", file_name.c_str());
    return (-1);
  }
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (fd < 0)
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during open (%s)!\n", file_name.c_str());
    return (-1);
  }
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  // Stretch the file size to the size of the data
  off_t result = pcl_lseek (fd, getpagesize () + cloud.data.size () - 1, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during lseek ()!\n");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during write ()!\n");
    return (-1);
  }
#endif
  // Prepare the map
#ifdef _WIN32
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, (DWORD) (data_idx + cloud.data.size ()), NULL);
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + cloud.data.size ()));
  CloseHandle (fm);

#else
  char *map = static_cast<char*> (mmap (0, static_cast<size_t> (data_idx + cloud.data.size ()), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during mmap ()!\n");
    return (-1);
  }
#endif

  // copy header
  memcpy (&map[0], oss.str().c_str (), static_cast<size_t> (data_idx));

  // Copy the data
  memcpy (&map[0] + data_idx, &cloud.data[0], cloud.data.size ());

#ifndef _WIN32
  // If the user set the synchronization flag on, call msync
  if (map_synchronization_)
    msync (map, static_cast<size_t> (data_idx + cloud.data.size ()), MS_SYNC);
#endif

  // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile (map);
#else
  if (munmap (map, static_cast<size_t> (data_idx + cloud.data.size ())) == -1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during munmap ()!\n");
    return (-1);
  }
#endif
  // Close file
#ifdef _WIN32
  CloseHandle(h_native_file);
#else
  pcl_close (fd);
#endif
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDWriter::writeBinaryCompressed (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                                       const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] Input point cloud has no data!\n");
    return (-1);
  }
  std::streamoff data_idx = 0;
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << generateHeaderBinaryCompressed (cloud, origin, orientation) << "DATA binary_compressed\n";
  oss.flush ();
  data_idx = oss.tellp ();

#ifdef _WIN32
  HANDLE h_native_file = CreateFile (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] Error during CreateFile (%s)!\n", file_name.c_str ());
    return (-1);
  }
#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (fd < 0)
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] Error during open (%s)!\n", file_name.c_str());
    return (-1);
  }
#endif
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  size_t fsize = 0;
  size_t data_size = 0;
  size_t nri = 0;
  std::vector<pcl::PCLPointField> fields (cloud.fields.size ());
  std::vector<int> fields_sizes (cloud.fields.size ());
  // Compute the total size of the fields
  for (size_t i = 0; i < cloud.fields.size (); ++i)
  {
    if (cloud.fields[i].name == "_")
      continue;
    
    fields_sizes[nri] = cloud.fields[i].count * pcl::getFieldSize (cloud.fields[i].datatype);
    fsize += fields_sizes[nri];
    fields[nri] = cloud.fields[i];
    ++nri;
  }
  fields_sizes.resize (nri);
  fields.resize (nri);
 
  // Compute the size of data
  data_size = cloud.width * cloud.height * fsize;

  //////////////////////////////////////////////////////////////////////
  // Empty array holding only the valid data
  // data_size = nr_points * point_size 
  //           = nr_points * (sizeof_field_1 + sizeof_field_2 + ... sizeof_field_n)
  //           = sizeof_field_1 * nr_points + sizeof_field_2 * nr_points + ... sizeof_field_n * nr_points
  char *only_valid_data = static_cast<char*> (malloc (data_size));

  // Convert the XYZRGBXYZRGB structure to XXYYZZRGBRGB to aid compression. For
  // this, we need a vector of fields.size () (4 in this case), which points to
  // each individual plane:
  //   pters[0] = &only_valid_data[offset_of_plane_x];
  //   pters[1] = &only_valid_data[offset_of_plane_y];
  //   pters[2] = &only_valid_data[offset_of_plane_z];
  //   pters[3] = &only_valid_data[offset_of_plane_RGB];
  //
  std::vector<char*> pters (fields.size ());
  int toff = 0;
  for (size_t i = 0; i < pters.size (); ++i)
  {
    pters[i] = &only_valid_data[toff];
    toff += fields_sizes[i] * cloud.width * cloud.height;
  }
  
  // Go over all the points, and copy the data in the appropriate places
  for (size_t i = 0; i < cloud.width * cloud.height; ++i)
  {
    for (size_t j = 0; j < pters.size (); ++j)
    {
      memcpy (pters[j], &cloud.data[i * cloud.point_step + fields[j].offset], fields_sizes[j]);
      // Increment the pointer
      pters[j] += fields_sizes[j];
    }
  }

  char* temp_buf = static_cast<char*> (malloc (static_cast<size_t> (static_cast<float> (data_size) * 1.5f + 8.0f)));
  // Compress the valid data
  unsigned int compressed_size = pcl::lzfCompress (only_valid_data, 
                                                   static_cast<unsigned int> (data_size), 
                                                   &temp_buf[8], 
                                                   static_cast<unsigned int> (static_cast<float> (data_size) * 1.5f));
  unsigned int compressed_final_size = 0;
  // Was the compression successful?
  if (compressed_size)
  {
    char *header = &temp_buf[0];
    memcpy (&header[0], &compressed_size, sizeof (unsigned int));
    memcpy (&header[4], &data_size, sizeof (unsigned int));
    data_size = compressed_size + 8;
    compressed_final_size = static_cast<unsigned int> (data_size + data_idx);
  }
  else
  {
#ifndef _WIN32
    pcl_close (fd);
#endif
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during compression!");
    return (-1);
  }

#ifndef _WIN32
  // Stretch the file size to the size of the data
  off_t result = pcl_lseek (fd, getpagesize () + data_size - 1, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] Error during lseek ()!\n");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] Error during write ()!\n");
    return (-1);
  }
#endif

  // Prepare the map
#ifdef _WIN32
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, compressed_final_size, NULL);
  char *map = static_cast<char*> (MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, compressed_final_size));
  CloseHandle (fm);

#else
  char *map = static_cast<char*> (mmap (0, compressed_final_size, PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] Error during mmap ()!\n");
    return (-1);
  }
#endif

  // copy header
  memcpy (&map[0], oss.str ().c_str (), static_cast<size_t> (data_idx));
  // Copy the compressed data
  memcpy (&map[data_idx], temp_buf, data_size);

#ifndef _WIN32
  // If the user set the synchronization flag on, call msync
  if (map_synchronization_)
    msync (map, compressed_final_size, MS_SYNC);
#endif

  // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile (map);
#else
  if (munmap (map, (compressed_final_size)) == -1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] Error during munmap ()!\n");
    return (-1);
  }
#endif
  // Close file
#ifdef _WIN32
  CloseHandle (h_native_file);
#else
  pcl_close (fd);
#endif
  resetLockingPermissions (file_name, file_lock);

  free (only_valid_data);
  free (temp_buf);
  return (0);
}


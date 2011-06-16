/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
#include <boost/algorithm/string.hpp>
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"

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

///////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDReader::readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, 
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, 
                            int &pcd_version, bool &binary_data, int &data_idx)
{
  // Default values
  data_idx = 0;
  binary_data = false;
  pcd_version = PCD_V6;
  origin      = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  cloud.is_dense = true;

  int nr_points = 0;
  std::ifstream fs;
  std::string line;

  int specified_channel_count = 0;

  // Open file in binary mode to avoid problem of 
  // std::getline() corrupting the result of ifstream::tellg()
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not open file %s.\n", file_name.c_str ());
    return (-1);
  }

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

      std::string line_type = st.at (0);

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        specified_channel_count = st.size () - 1;

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
          cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        specified_channel_count = st.size () - 1;

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != (int)cloud.fields.size ())
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type = atoi (st.at (i + 1).c_str ());
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

        specified_channel_count = st.size () - 1;

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != (int)cloud.fields.size ())
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = getFieldType (field_sizes[i], field_types[i]);
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        specified_channel_count = st.size () - 1;

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != (int)cloud.fields.size ())
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count = atoi (st.at (i + 1).c_str ());
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
        cloud.width = atoi (st.at (1).c_str ());
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        cloud.height = atoi (st.at (1).c_str ());
        /* Old, buggy behavior. is_dense does not represent "is_organized"
        if (cloud.height == 1)
          cloud.is_dense = false;
        else
          cloud.is_dense = true;*/
        continue;
      }

      // Get the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        pcd_version = PCD_V7;
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";

        origin      = Eigen::Vector4f (atof (st.at (1).c_str ()), atof (st.at (2).c_str ()), atof (st.at (3).c_str ()), 0);
        orientation = Eigen::Quaternionf (atof (st.at (4).c_str ()), atof (st.at (5).c_str ()), atof (st.at (6).c_str ()), atof (st.at (7).c_str ()));
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        nr_points = atoi (st.at (1).c_str ());
        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }

      // Read the header + comments line by line until we get to <DATA>
      if (line_type.substr (0, 4) == "DATA")
      {
        data_idx = fs.tellg ();
        if (st.at (1).substr (0, 6) == "binary")
        {
           binary_data = true;
        }
        continue;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
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
    if (cloud.width == 0)
    {
      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      return (-1);
    }
  }

  if (int(cloud.width * cloud.height) != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    return (-1);
  }

  // Close file
  fs.close ();

  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDReader::read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version)
{
  bool binary_data;
  int data_idx;

  int res = readHeader (file_name, cloud, origin, orientation, pcd_version, binary_data, data_idx);

  if (res < 0)
    return (res);

  // @todo fixme
  cloud.is_bigendian = false;  

  int idx = 0;

  // Get the number of points the cloud should have
  int nr_points = cloud.width * cloud.height;

  // if ascii
  if (!binary_data)
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
      while (!fs.eof ())
      {
        getline (fs, line);
        // Ignore empty lines
        if (line == "")
          continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

        // We must have points then
        // Convert the first token to float and use it as the first point coordinate
        if (idx >= nr_points)
        {
          PCL_WARN ("[pcl::PCDReader::read] input file %s has more points than advertised (%d)!\n", file_name.c_str (), nr_points);
          break;
        }


        // Copy data
        for (size_t d = 0; d < cloud.fields.size (); ++d)
        {
          // Ignore invalid padded dimensions that are inherited from binary data
          if (cloud.fields[d].name == "_")
            continue;
          for (size_t c = 0; c < cloud.fields[d].count; ++c)
          {
            switch (cloud.fields[d].datatype)
            {
              case sensor_msgs::PointField::INT8:
              {
                copyValue<pcl::traits::asType<sensor_msgs::PointField::INT8>::type> (st.at (d + c), cloud, idx, d, c);
                break;
              }
              case sensor_msgs::PointField::UINT8:
              {
                copyValue<pcl::traits::asType<sensor_msgs::PointField::UINT8>::type> (st.at (d + c), cloud, idx, d, c);
                break;
              }
              case sensor_msgs::PointField::INT16:
              {
                copyValue<pcl::traits::asType<sensor_msgs::PointField::INT16>::type> (st.at (d + c), cloud, idx, d, c);
                break;
              }
              case sensor_msgs::PointField::UINT16:
              {
                copyValue<pcl::traits::asType<sensor_msgs::PointField::UINT16>::type> (st.at (d + c), cloud, idx, d, c);
                break;
              }
              case sensor_msgs::PointField::INT32:
              {
                copyValue<pcl::traits::asType<sensor_msgs::PointField::UINT16>::type> (st.at (d + c), cloud, idx, d, c);
                break;
              }
              case sensor_msgs::PointField::UINT32:
              {
                copyValue<pcl::traits::asType<sensor_msgs::PointField::UINT32>::type> (st.at (d + c), cloud, idx, d, c);
                break;
              }
              case sensor_msgs::PointField::FLOAT32:
              {
                copyValue<pcl::traits::asType<sensor_msgs::PointField::FLOAT32>::type> (st.at (d + c), cloud, idx, d, c);
                break;
              }
              case sensor_msgs::PointField::FLOAT64:
              {
                copyValue<pcl::traits::asType<sensor_msgs::PointField::FLOAT64>::type> (st.at (d + c), cloud, idx, d, c);
                break;
              }
              default:
                PCL_WARN ("[pcl::PCDReader::read] Incorrect field data type specified (%d)!\n",cloud.fields[d].datatype);
                break;
            }
          }
        }

        idx++;
      }
    }
    catch (const char *exception)
    {
      PCL_ERROR ("[pcl::PCDReader::read] %s\n", exception);
      return (-1);
    }

    // Close file
    fs.close ();

  }
  else 
  /// ---[ Binary mode only
  /// We must re-open the file and read with mmap () for binary
  {
    // Set the is_dense mode to false -- otherwise we would have to iterate over all points and check them 1 by 1
    cloud.is_dense = false;
    // Open for reading
    int fd = pcl_open (file_name.c_str (), O_RDONLY);
    if (fd == -1)
      return (-1);

    // Prepare the map
#ifdef _WIN32
    // map te whole file
    HANDLE fm = CreateFileMapping ((HANDLE) _get_osfhandle (fd), NULL, PAGE_READONLY, 0, data_idx + cloud.data.size (), NULL);
    char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ, 0, 0, data_idx + cloud.data.size ()));
    if(map == NULL)
    {
      CloseHandle (fm);
      pcl_close (fd);
      return (-1);
    }
    CloseHandle (fm);
#else
    char *map = (char*)mmap (0, data_idx + cloud.data.size (), PROT_READ, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED)
    {
      pcl_close (fd);
      return (-1);
    }
#endif
    // Copy the data
    memcpy (&cloud.data[0], &map[0] + data_idx, cloud.data.size ());

    // Unmap the pages of memory
#if _WIN32
    UnmapViewOfFile (map);
#else
    if (munmap (map, cloud.data.size ()) == -1)
    {
      pcl_close (fd);
      return (-1);
    }
#endif
    pcl_close (fd);
  }

  if ( (idx != nr_points) && (!binary_data) )
  {
    PCL_ERROR ("[pcl::PCDReader::read] Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
    return (-1);
  }

  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDReader::read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud)
{
  int pcd_version;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  // Load the data
  int res = read (file_name, cloud, origin, orientation, pcd_version);

  if (res < 0)
    return (res);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::PCDWriter::generateHeaderASCII (const sensor_msgs::PointCloud2 &cloud, 
                                     const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation)
{
  std::ostringstream oss;

  oss << "# .PCD v.7 - Point Cloud Data file format"
         "\nVERSION .7"
         "\nFIELDS ";

  std::ostringstream stream;
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
      stream << pcl::getFieldType (cloud.fields[d].datatype) << " ";
  }
  // Ignore invalid padded dimensions that are inherited from binary data
  if (cloud.fields[cloud.fields.size () - 1].name != "_")
    stream << pcl::getFieldType (cloud.fields[cloud.fields.size () - 1].datatype);

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
    int count = abs ((int)cloud.fields[d].count);
    if (count == 0) 
      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
  
    stream << count << " ";
  }
  // Ignore invalid padded dimensions that are inherited from binary data
  if (cloud.fields[cloud.fields.size () - 1].name != "_")
  {
    int count = abs ((int)cloud.fields[cloud.fields.size () - 1].count);
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
pcl::PCDWriter::generateHeaderBinary (const sensor_msgs::PointCloud2 &cloud, 
                                      const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation)
{
  std::ostringstream oss;

  oss << "# .PCD v.7 - Point Cloud Data file format"
         "\nVERSION .7"
         "\nFIELDS";

  // Compute the total size of the fields
  unsigned int fsize = 0;
  for (size_t i = 0; i < cloud.fields.size (); ++i)
    fsize += cloud.fields[i].count * getFieldSize (cloud.fields[i].datatype);
 
  // The size of the fields cannot be larger than point_step
  if (fsize > cloud.point_step)
  {
    PCL_ERROR ("[pcl::PCDWriter::generateHeader] The size of the fields (%d) is larger than point_step (%d)! Something is wrong here...\n", fsize, cloud.point_step);
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
    int count = abs ((int)cloud.fields[i].count);
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
int
pcl::PCDWriter::writeASCII (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
                            const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation,
                            int precision)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PCDWriter::writeASCII] Input point cloud has no data!\n");
    return (-1);
  }

  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());      // Open file

  int nr_points  = cloud.width * cloud.height;
  int point_size = cloud.data.size () / nr_points;

  // Write the header information
  fs << generateHeaderASCII (cloud, origin, orientation) << "DATA ascii\n";

  std::ostringstream stream;

  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    for (size_t d = 0; d < cloud.fields.size (); ++d)
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
          case sensor_msgs::PointField::INT8:
          {
            char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (char)], sizeof (char));
            stream << boost::numeric_cast<int>(value);
            break;
          }
          case sensor_msgs::PointField::UINT8:
          {
            unsigned char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned char)], sizeof (unsigned char));
            stream << boost::numeric_cast<int>(value);
            break;
          }
          case sensor_msgs::PointField::INT16:
          {
            short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (short)], sizeof (short));
            stream << boost::numeric_cast<int>(value);
            break;
          }
          case sensor_msgs::PointField::UINT16:
          {
            unsigned short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned short)], sizeof (unsigned short));
            stream << boost::numeric_cast<int>(value);
            break;
          }
          case sensor_msgs::PointField::INT32:
          {
            int value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (int)], sizeof (int));
            stream << value;
            break;
          }
          case sensor_msgs::PointField::UINT32:
          {
            unsigned int value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned int)], sizeof (unsigned int));
            stream << value;
            break;
          }
          case sensor_msgs::PointField::FLOAT32:
          {
            float value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
            stream << value;
            break;
          }
          case sensor_msgs::PointField::FLOAT64:
          {
            double value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (double)], sizeof (double));
            stream << value;
            break;
          }
          default:
            PCL_WARN ("[pcl::PCDWriter::writeASCII] Incorrect field data type specified (%d)!\n", cloud.fields[d].datatype);
            break;
        }

        if (d < cloud.fields.size () - 1 || c < (int)cloud.fields[d].count - 1)
          stream << " ";
      }
    }
    // Copy the stream, trim it, and write it to disk
    std::string result = stream.str ();
    boost::trim (result);
    stream.str ("");
    fs << result << std::endl;
  }
  fs.close ();              // Close file
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDWriter::writeBinary (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud,
                             const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Input point cloud has no data!\n");
    return (-1);
  }
  int data_idx = 0;
  std::ostringstream oss;
  oss << generateHeaderBinary (cloud, origin, orientation) << "DATA binary\n";
  oss.flush();
  data_idx = oss.tellp ();

#if _WIN32
  HANDLE h_native_file = CreateFile (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if(h_native_file == INVALID_HANDLE_VALUE)
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during CreateFile (%s)!\n", file_name.c_str());
    return (-1);
  }
#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);
  if (fd < 0)
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during open (%s)!\n", file_name.c_str());
    return (-1);
  }
  // Stretch the file size to the size of the data
  int result = pcl_lseek (fd, getpagesize () + cloud.data.size () - 1, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during lseek ()!\n");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = ::write (fd, "", 1);
  if (result != 1)
  {
    pcl_close (fd);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during write ()!\n");
    return (-1);
  }
#endif
  // Prepare the map
#ifdef _WIN32
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, data_idx + cloud.data.size (), NULL);
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + cloud.data.size ()));
  CloseHandle (fm);

#else
  char *map = (char*)mmap (0, data_idx + cloud.data.size (), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (map == MAP_FAILED)
  {
    pcl_close (fd);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during mmap ()!\n");
    return (-1);
  }
#endif

  // copy header
  memcpy (&map[0], oss.str().c_str(), data_idx);

  // Copy the data
  memcpy (&map[0] + data_idx, &cloud.data[0], cloud.data.size ());

  // Unmap the pages of memory
#if _WIN32
    UnmapViewOfFile (map);
#else
  if (munmap (map, (data_idx + cloud.data.size ())) == -1)
  {
    pcl_close (fd);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] Error during munmap ()!\n");
    return (-1);
  }
#endif
  // Close file
#if _WIN32
  CloseHandle(h_native_file);
#else
  pcl_close (fd);
#endif
  return (0);
}

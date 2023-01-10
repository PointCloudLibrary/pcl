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

#ifndef PCL_IO_PCD_IO_IMPL_H_
#define PCL_IO_PCD_IO_IMPL_H_

#include <boost/algorithm/string/trim.hpp> // for trim
#include <fstream>
#include <fcntl.h>
#include <string>
#include <cstdlib>
#include <pcl/common/io.h> // for getFields, ...
#include <pcl/console/print.h>
#include <pcl/io/low_level_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/io/lzf.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::string
pcl::PCDWriter::generateHeader (const pcl::PointCloud<PointT> &cloud, const int nr_points)
{
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << "# .PCD v0.7 - Point Cloud Data file format"
         "\nVERSION 0.7"
         "\nFIELDS";

  const auto fields = pcl::getFields<PointT> ();
 
  std::stringstream field_names, field_types, field_sizes, field_counts;
  for (const auto &field : fields)
  {
    if (field.name == "_")
      continue;
    // Add the regular dimension
    field_names << " " << field.name;
    field_sizes << " " << pcl::getFieldSize (field.datatype);
    if ("rgb" == field.name)
      field_types << " " << "U";
    else
      field_types << " " << pcl::getFieldType (field.datatype);
    int count = std::abs (static_cast<int> (field.count));
    if (count == 0) count = 1;  // check for 0 counts (coming from older converter code)
    field_counts << " " << count;
  }
  oss << field_names.str ();
  oss << "\nSIZE" << field_sizes.str () 
      << "\nTYPE" << field_types.str () 
      << "\nCOUNT" << field_counts.str ();
  // If the user passes in a number of points value, use that instead
  if (nr_points != std::numeric_limits<int>::max ())
    oss << "\nWIDTH " << nr_points << "\nHEIGHT " << 1 << "\n";
  else
    oss << "\nWIDTH " << cloud.width << "\nHEIGHT " << cloud.height << "\n";

  oss << "VIEWPOINT " << cloud.sensor_origin_[0] << " " << cloud.sensor_origin_[1] << " " << cloud.sensor_origin_[2] << " " << 
                         cloud.sensor_orientation_.w () << " " << 
                         cloud.sensor_orientation_.x () << " " << 
                         cloud.sensor_orientation_.y () << " " << 
                         cloud.sensor_orientation_.z () << "\n";
  
  // If the user passes in a number of points value, use that instead
  if (nr_points != std::numeric_limits<int>::max ())
    oss << "POINTS " << nr_points << "\n";
  else
    oss << "POINTS " << cloud.size () << "\n";

  return (oss.str ());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PCDWriter::writeBinary (const std::string &file_name, 
                             const pcl::PointCloud<PointT> &cloud)
{
  if (cloud.empty ())
  {
    PCL_WARN ("[pcl::PCDWriter::writeBinary] Input point cloud has no data!\n");
  }
  int data_idx = 0;
  std::ostringstream oss;
  oss << generateHeader<PointT> (cloud) << "DATA binary\n";
  oss.flush ();
  data_idx = static_cast<int> (oss.tellp ());

#ifdef _WIN32
  HANDLE h_native_file = CreateFileA (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during CreateFile!");
    return (-1);
  }
#else
  int fd = io::raw_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (fd < 0)
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during open!");
    return (-1);
  }
#endif
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  auto fields = pcl::getFields<PointT> ();
  std::vector<int> fields_sizes;
  std::size_t fsize = 0;
  std::size_t data_size = 0;
  std::size_t nri = 0;
  // Compute the total size of the fields
  for (const auto &field : fields)
  {
    if (field.name == "_")
      continue;
    
    int fs = field.count * getFieldSize (field.datatype);
    fsize += fs;
    fields_sizes.push_back (fs);
    fields[nri++] = field;
  }
  fields.resize (nri);
  
  data_size = cloud.size () * fsize;

  // Prepare the map
#ifdef _WIN32
  HANDLE fm = CreateFileMappingA (h_native_file, NULL, PAGE_READWRITE, 0, (DWORD) (data_idx + data_size), NULL);
  if (fm == NULL)
  {
      throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during memory map creation ()!");
      return (-1);
  }
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + data_size));
  CloseHandle (fm);

#else
  // Allocate disk space for the entire file to prevent bus errors.
  const int allocate_res = io::raw_fallocate (fd, data_idx + data_size);
  if (allocate_res != 0)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] raw_fallocate(length=%zu) returned %i. errno: %d strerror: %s\n",
               data_idx + data_size, allocate_res, errno, strerror (errno));

    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during raw_fallocate ()!");
    return (-1);
  }

  char *map = static_cast<char*> (::mmap (nullptr, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1)) //MAP_FAILED)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during mmap ()!");
    return (-1);
  }
#endif

  // Copy the header
  memcpy (&map[0], oss.str ().c_str (), data_idx);

  // Copy the data
  char *out = &map[0] + data_idx;
  for (const auto& point: cloud)
  {
    int nrj = 0;
    for (const auto &field : fields)
    {
      memcpy (out, reinterpret_cast<const char*> (&point) + field.offset, fields_sizes[nrj]);
      out += fields_sizes[nrj++];
    }
  }

  // If the user set the synchronization flag on, call msync
#ifndef _WIN32
  if (map_synchronization_)
    msync (map, data_idx + data_size, MS_SYNC);
#endif

  // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile (map);
#else
  if (::munmap (map, (data_idx + data_size)) == -1)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during munmap ()!");
    return (-1);
  }
#endif
  // Close file
#ifdef _WIN32
  CloseHandle (h_native_file);
#else
  io::raw_close (fd);
#endif
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PCDWriter::writeBinaryCompressed (const std::string &file_name, 
                                       const pcl::PointCloud<PointT> &cloud)
{
  if (cloud.empty ())
  {
    PCL_WARN ("[pcl::PCDWriter::writeBinaryCompressed] Input point cloud has no data!\n");
  }
  int data_idx = 0;
  std::ostringstream oss;
  oss << generateHeader<PointT> (cloud) << "DATA binary_compressed\n";
  oss.flush ();
  data_idx = static_cast<int> (oss.tellp ());

#ifdef _WIN32
  HANDLE h_native_file = CreateFileA (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during CreateFile!");
    return (-1);
  }
#else
  int fd = io::raw_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (fd < 0)
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during open!");
    return (-1);
  }
#endif

  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  auto fields = pcl::getFields<PointT> ();
  std::size_t fsize = 0;
  std::size_t data_size = 0;
  std::size_t nri = 0;
  std::vector<int> fields_sizes (fields.size ());
  // Compute the total size of the fields
  for (const auto &field : fields)
  {
    if (field.name == "_")
      continue;
    
    fields_sizes[nri] = field.count * pcl::getFieldSize (field.datatype);
    fsize += fields_sizes[nri];
    fields[nri] = field;
    ++nri;
  }
  fields_sizes.resize (nri);
  fields.resize (nri);
 
  // Compute the size of data
  data_size = cloud.size () * fsize;

  // If the data is to large the two 32 bit integers used to store the
  // compressed and uncompressed size will overflow.
  if (data_size * 3 / 2 > std::numeric_limits<std::uint32_t>::max ())
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] The input data exceeds the maximum size for compressed version 0.7 pcds of %l bytes.\n",
               static_cast<std::size_t> (std::numeric_limits<std::uint32_t>::max ()) * 2 / 3);
    return (-2);
  }

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
  std::size_t toff = 0;
  for (std::size_t i = 0; i < pters.size (); ++i)
  {
    pters[i] = &only_valid_data[toff];
    toff += static_cast<std::size_t>(fields_sizes[i]) * cloud.size();
  }
  
  // Go over all the points, and copy the data in the appropriate places
  for (const auto& point: cloud)
  {
    for (std::size_t j = 0; j < fields.size (); ++j)
    {
      memcpy (pters[j], reinterpret_cast<const char*> (&point) + fields[j].offset, fields_sizes[j]);
      // Increment the pointer
      pters[j] += fields_sizes[j];
    }
  }

  char* temp_buf = static_cast<char*> (malloc (static_cast<std::size_t> (static_cast<float> (data_size) * 1.5f + 8.0f)));
  unsigned int compressed_final_size = 0;
  if (data_size != 0) {
  // Compress the valid data
  unsigned int compressed_size = pcl::lzfCompress (only_valid_data, 
                                                   static_cast<std::uint32_t> (data_size), 
                                                   &temp_buf[8], 
                                                   static_cast<std::uint32_t> (static_cast<float>(data_size) * 1.5f));
    // Was the compression successful?
    if (compressed_size > 0)
    {
      char *header = &temp_buf[0];
      memcpy (&header[0], &compressed_size, sizeof (unsigned int));
      memcpy (&header[4], &data_size, sizeof (unsigned int));
      data_size = compressed_size + 8;
      compressed_final_size = static_cast<std::uint32_t> (data_size) + data_idx;
    }
    else
    {
  #ifndef _WIN32
      io::raw_close (fd);
  #endif
      resetLockingPermissions (file_name, file_lock);
      PCL_WARN("[pcl::PCDWriter::writeBinaryCompressed] Error during compression!\n");
      return (-1);
    }
  }
  else
  {
    // empty cloud case
    compressed_final_size = 8 + data_idx;
    auto *header = reinterpret_cast<std::uint32_t*>(&temp_buf[0]);
    header[0] = 0; // compressed_size is 0
    header[1] = 0; // data_size is 0
    data_size = 8; // correct data_size to header size
  }

  // Prepare the map
#ifdef _WIN32
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, compressed_final_size, NULL);
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, compressed_final_size));
  CloseHandle (fm);

#else
  // Allocate disk space for the entire file to prevent bus errors.
  const int allocate_res = io::raw_fallocate (fd, compressed_final_size);
  if (allocate_res != 0)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] raw_fallocate(length=%u) returned %i. errno: %d strerror: %s\n",
               compressed_final_size, allocate_res, errno, strerror (errno));

    throw pcl::IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during raw_fallocate ()!");
    return (-1);
  }

  char *map = static_cast<char*> (::mmap (nullptr, compressed_final_size, PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1)) //MAP_FAILED)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during mmap ()!");
    return (-1);
  }
#endif

  // Copy the header
  memcpy (&map[0], oss.str ().c_str (), data_idx);
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
  if (::munmap (map, (compressed_final_size)) == -1)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during munmap ()!");
    return (-1);
  }
#endif

  // Close file
#ifdef _WIN32
  CloseHandle (h_native_file);
#else
  io::raw_close (fd);
#endif
  resetLockingPermissions (file_name, file_lock);

  free (only_valid_data);
  free (temp_buf);
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PCDWriter::writeASCII (const std::string &file_name, const pcl::PointCloud<PointT> &cloud, 
                            const int precision)
{
  if (cloud.empty ())
  {
    PCL_WARN ("[pcl::PCDWriter::writeASCII] Input point cloud has no data!\n");
  }

  if (cloud.width * cloud.height != cloud.size ())
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Number of points different than width * height!");
    return (-1);
  }

  std::ofstream fs;
  fs.open (file_name.c_str (), std::ios::binary);      // Open file
  
  if (!fs.is_open () || fs.fail ())
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Could not open file for writing!");
    return (-1);
  }
  
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  fs.precision (precision);
  fs.imbue (std::locale::classic ());

  const auto fields = pcl::getFields<PointT> ();

  // Write the header information
  fs << generateHeader<PointT> (cloud) << "DATA ascii\n";

  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());
  // Iterate through the points
  for (const auto& point: cloud)
  {
    for (std::size_t d = 0; d < fields.size (); ++d)
    {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (fields[d].name == "_")
        continue;

      int count = fields[d].count;
      if (count == 0) 
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)

      for (int c = 0; c < count; ++c)
      {
        switch (fields[d].datatype)
        {
          case pcl::PCLPointField::BOOL:
          {
            bool value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (bool), sizeof (bool));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT8:
          {
            std::int8_t value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (std::int8_t), sizeof (std::int8_t));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            std::uint8_t value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (std::uint8_t), sizeof (std::uint8_t));
            stream << boost::numeric_cast<std::uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            std::int16_t value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (std::int16_t), sizeof (std::int16_t));
            stream << boost::numeric_cast<std::int16_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            std::uint16_t value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (std::uint16_t), sizeof (std::uint16_t));
            stream << boost::numeric_cast<std::uint16_t>(value);
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            std::int32_t value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (std::int32_t), sizeof (std::int32_t));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            std::uint32_t value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (std::uint32_t), sizeof (std::uint32_t));
            stream << boost::numeric_cast<std::uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT64:
          {
            std::int64_t value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (std::int64_t), sizeof (std::int64_t));
            stream << boost::numeric_cast<std::int64_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT64:
          {
            std::uint64_t value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (std::uint64_t), sizeof (std::uint64_t));
            stream << boost::numeric_cast<std::uint64_t>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            /*
             * Despite the float type, store the rgb field as uint32
             * because several fully opaque color values are mapped to
             * nan.
             */
            if ("rgb" == fields[d].name)
            {
              std::uint32_t value;
              memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (float), sizeof (float));
              stream << boost::numeric_cast<std::uint32_t>(value);
              break;
            }
            float value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (float), sizeof (float));
            if (std::isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<float>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            double value;
            memcpy (&value, reinterpret_cast<const char*> (&point) + fields[d].offset + c * sizeof (double), sizeof (double));
            if (std::isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<double>(value);
            break;
          }
          default:
            PCL_WARN ("[pcl::PCDWriter::writeASCII] Incorrect field data type specified (%d)!\n", fields[d].datatype);
            break;
        }

        if (d < fields.size () - 1 || c < static_cast<int> (fields[d].count - 1))
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PCDWriter::writeBinary (const std::string &file_name, 
                             const pcl::PointCloud<PointT> &cloud, 
                             const pcl::Indices &indices)
{
  if (cloud.empty () || indices.empty ())
  {
    PCL_WARN ("[pcl::PCDWriter::writeBinary] Input point cloud has no data or empty indices given!\n");
  }
  int data_idx = 0;
  std::ostringstream oss;
  oss << generateHeader<PointT> (cloud, static_cast<int> (indices.size ())) << "DATA binary\n";
  oss.flush ();
  data_idx = static_cast<int> (oss.tellp ());

#ifdef _WIN32
  HANDLE h_native_file = CreateFileA (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during CreateFile!");
    return (-1);
  }
#else
  int fd = io::raw_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (fd < 0)
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during open!");
    return (-1);
  }
#endif
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  auto fields = pcl::getFields<PointT> ();
  std::vector<int> fields_sizes;
  std::size_t fsize = 0;
  std::size_t data_size = 0;
  std::size_t nri = 0;
  // Compute the total size of the fields
  for (const auto &field : fields)
  {
    if (field.name == "_")
      continue;
    
    int fs = field.count * getFieldSize (field.datatype);
    fsize += fs;
    fields_sizes.push_back (fs);
    fields[nri++] = field;
  }
  fields.resize (nri);
  
  data_size = indices.size () * fsize;

  // Prepare the map
#ifdef _WIN32
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, data_idx + data_size, NULL);
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + data_size));
  CloseHandle (fm);

#else
  // Allocate disk space for the entire file to prevent bus errors.
  const int allocate_res = io::raw_fallocate (fd, data_idx + data_size);
  if (allocate_res != 0)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PCDWriter::writeBinary] raw_fallocate(length=%zu) returned %i. errno: %d strerror: %s\n",
               data_idx + data_size, allocate_res, errno, strerror (errno));

    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during raw_fallocate ()!");
    return (-1);
  }

  char *map = static_cast<char*> (::mmap (nullptr, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1)) //MAP_FAILED)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during mmap ()!");
    return (-1);
  }
#endif

  // Copy the header
  memcpy (&map[0], oss.str ().c_str (), data_idx);

  char *out = &map[0] + data_idx;
  // Copy the data
  for (const auto &index : indices)
  {
    int nrj = 0;
    for (const auto &field : fields)
    {
      memcpy (out, reinterpret_cast<const char*> (&cloud[index]) + field.offset, fields_sizes[nrj]);
      out += fields_sizes[nrj++];
    }
  }

#ifndef _WIN32
  // If the user set the synchronization flag on, call msync
  if (map_synchronization_)
    msync (map, data_idx + data_size, MS_SYNC);
#endif

  // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile (map);
#else
  if (::munmap (map, (data_idx + data_size)) == -1)
  {
    io::raw_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Error during munmap ()!");
    return (-1);
  }
#endif
  // Close file
#ifdef _WIN32
  CloseHandle(h_native_file);
#else
  io::raw_close (fd);
#endif
  
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PCDWriter::writeASCII (const std::string &file_name, 
                            const pcl::PointCloud<PointT> &cloud, 
                            const pcl::Indices &indices,
                            const int precision)
{
  if (cloud.empty () || indices.empty ())
  {
    PCL_WARN ("[pcl::PCDWriter::writeASCII] Input point cloud has no data or empty indices given!\n");
  }

  if (cloud.width * cloud.height != cloud.size ())
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Number of points different than width * height!");
    return (-1);
  }

  std::ofstream fs;
  fs.open (file_name.c_str (), std::ios::binary);      // Open file
  if (!fs.is_open () || fs.fail ())
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Could not open file for writing!");
    return (-1);
  }

  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  fs.precision (precision);
  fs.imbue (std::locale::classic ());

  const auto fields = pcl::getFields<PointT> ();

  // Write the header information
  fs << generateHeader<PointT> (cloud, static_cast<int> (indices.size ())) << "DATA ascii\n";

  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());

  // Iterate through the points
  for (const auto &index : indices)
  {
    for (std::size_t d = 0; d < fields.size (); ++d)
    {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (fields[d].name == "_")
        continue;

      int count = fields[d].count;
      if (count == 0) 
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)

      for (int c = 0; c < count; ++c)
      {
        switch (fields[d].datatype)
        {
          case pcl::PCLPointField::BOOL:
          {
            bool value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[index]) + fields[d].offset + c * sizeof (bool), sizeof (bool));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT8:
          {
            std::int8_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (std::int8_t), sizeof (std::int8_t));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            std::uint8_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (std::uint8_t), sizeof (std::uint8_t));
            stream << boost::numeric_cast<std::uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            std::int16_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (std::int16_t), sizeof (std::int16_t));
            stream << boost::numeric_cast<std::int16_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            std::uint16_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (std::uint16_t), sizeof (std::uint16_t));
            stream << boost::numeric_cast<std::uint16_t>(value);
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            std::int32_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (std::int32_t), sizeof (std::int32_t));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            std::uint32_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (std::uint32_t), sizeof (std::uint32_t));
            stream << boost::numeric_cast<std::uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT64:
          {
            std::int64_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[index]) + fields[d].offset + c * sizeof (std::int64_t), sizeof (std::int64_t));
            stream << boost::numeric_cast<std::int64_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT64:
          {
            std::uint64_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[index]) + fields[d].offset + c * sizeof (std::uint64_t), sizeof (std::uint64_t));
            stream << boost::numeric_cast<std::uint64_t>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            /*
             * Despite the float type, store the rgb field as uint32
             * because several fully opaque color values are mapped to
             * nan.
             */
            if ("rgb" == fields[d].name)
            {
              std::uint32_t value;
              memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (float), sizeof (float));
              stream << boost::numeric_cast<std::uint32_t>(value);
            }
            else
            {
              float value;
              memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (float), sizeof (float));
              if (std::isnan (value))
                stream << "nan";
              else
                stream << boost::numeric_cast<float>(value);
            }
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            double value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud[index]) + fields[d].offset + c * sizeof (double), sizeof (double));
            if (std::isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<double>(value);
            break;
          }
          default:
            PCL_WARN ("[pcl::PCDWriter::writeASCII] Incorrect field data type specified (%d)!\n", fields[d].datatype);
            break;
        }

        if (d < fields.size () - 1 || c < static_cast<int> (fields[d].count - 1))
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

#endif  //#ifndef PCL_IO_PCD_IO_H_

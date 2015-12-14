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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id: file_io.h 827 2011-05-04 02:00:04Z nizar $
 *
 */

#ifndef PCL_IO_FILE_IO_H_
#define PCL_IO_FILE_IO_H_

#include <pcl/pcl_macros.h>
#include <pcl/common/io.h>
#include <pcl/io/boost.h>
#include <cmath>
#include <sstream>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>

namespace pcl
{
  /** \brief Point Cloud Data (FILE) file format reader interface.
    * Any (FILE) format file reader should implement its virtual methodes.
    * \author Nizar Sallem
    * \ingroup io
    */
  class PCL_EXPORTS FileReader
  {
    public:
      /** \brief empty constructor */ 
      FileReader() {}
      /** \brief empty destructor */ 
      virtual ~FileReader() {}
      /** \brief Read a point cloud data header from a FILE file. 
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given FILE file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * Returns:
        *  * < 0 (-1) on error
        *  * > 0 on success
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
        * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
        * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
        * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
        * \param[out] data_type the type of data (binary data=1, ascii=0, etc)
        * \param[out] data_idx the offset of cloud data within the file
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      virtual int 
      readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, 
                  int &file_version, int &data_type, unsigned int &data_idx, const int offset = 0) = 0;

      /** \brief Read a point cloud data from a FILE file and store it into a pcl/PCLPointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[out] origin the sensor acquisition origin (only for > FILE_V7 - null if not present)
        * \param[out] orientation the sensor acquisition orientation (only for > FILE_V7 - identity if not present)
        * \param[out] file_version the FILE version of the file (either FILE_V6 or FILE_V7)
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      virtual int 
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &file_version, 
            const int offset = 0) = 0;

      /** \brief Read a point cloud data from a FILE file (FILE_V6 only!) and store it into a pcl/PCLPointCloud2.
        * 
        * \note This function is provided for backwards compatibility only and
        * it can only read FILE_V6 files correctly, as pcl::PCLPointCloud2
        * does not contain a sensor origin/orientation. Reading any file 
        * > FILE_V6 will generate a warning. 
        *
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        *
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      int 
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0)
      {
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        int file_version;
        return (read (file_name, cloud, origin, orientation, file_version, offset));
      }

      /** \brief Read a point cloud data from any FILE file, and convert it to the given template format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      template<typename PointT> inline int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud, const int offset  =0)
      {
        pcl::PCLPointCloud2 blob;
        int file_version;
        int res = read (file_name, blob, cloud.sensor_origin_, cloud.sensor_orientation_, 
                        file_version, offset);

        // Exit in case of error
        if (res < 0)
          return res;
        pcl::fromPCLPointCloud2 (blob, cloud);
        return (0);
      }
  };

  /** \brief Point Cloud Data (FILE) file format writer.
    * Any (FILE) format file reader should implement its virtual methodes
    * \author Nizar Sallem
    * \ingroup io
    */
  class PCL_EXPORTS FileWriter
  {
    public:
      /** \brief Empty constructor */ 
      FileWriter () {}

      /** \brief Empty destructor */ 
      virtual ~FileWriter () {}

      /** \brief Save point cloud data to a FILE file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] binary set to true if the file is to be written in a binary
        * FILE format, false (default) for ASCII
        */
      virtual int
      write (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             const bool binary = false) = 0;

      /** \brief Save point cloud data to a FILE file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message (boost shared pointer)
        * \param[in] binary set to true if the file is to be written in a binary
        * FILE format, false (default) for ASCII
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      inline int
      write (const std::string &file_name, const pcl::PCLPointCloud2::ConstPtr &cloud,
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             const bool binary = false)
      {
        return (write (file_name, *cloud, origin, orientation, binary));
      }

      /** \brief Save point cloud data to a FILE file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the pcl::PointCloud data
        * \param[in] binary set to true if the file is to be written in a binary
        * FILE format, false (default) for ASCII
        */
      template<typename PointT> inline int
      write (const std::string &file_name, 
             const pcl::PointCloud<PointT> &cloud, 
             const bool binary = false)
      {
        Eigen::Vector4f origin = cloud.sensor_origin_;
        Eigen::Quaternionf orientation = cloud.sensor_orientation_;

        pcl::PCLPointCloud2 blob;
        pcl::toPCLPointCloud2 (cloud, blob);

        // Save the data
        return (write (file_name, blob, origin, orientation, binary));
      }
  };

  /** \brief insers a value of type Type (uchar, char, uint, int, float, double, ...) into a stringstream.
    *
    * If the value is NaN, it inserst "nan".
    *
    * \param[in] cloud the cloud to copy from
    * \param[in] point_index the index of the point
    * \param[in] point_size the size of the point in the cloud
    * \param[in] field_idx the index of the dimension/field
    * \param[in] fields_count the current fields count
    * \param[out] stream the ostringstream to copy into
    */
  template <typename Type> inline void
  copyValueString (const pcl::PCLPointCloud2 &cloud,
                   const unsigned int point_index, 
                   const int point_size, 
                   const unsigned int field_idx, 
                   const unsigned int fields_count, 
                   std::ostream &stream)
  {
    Type value;
    memcpy (&value, &cloud.data[point_index * point_size + cloud.fields[field_idx].offset + fields_count * sizeof (Type)], sizeof (Type));
    if (pcl_isnan (value))
      stream << "nan";
    else
      stream << boost::numeric_cast<Type>(value);
  }
  template <> inline void
  copyValueString<int8_t> (const pcl::PCLPointCloud2 &cloud,
                           const unsigned int point_index, 
                           const int point_size, 
                           const unsigned int field_idx, 
                           const unsigned int fields_count, 
                           std::ostream &stream)
  {
    int8_t value;
    memcpy (&value, &cloud.data[point_index * point_size + cloud.fields[field_idx].offset + fields_count * sizeof (int8_t)], sizeof (int8_t));
    if (pcl_isnan (value))
      stream << "nan";
    else
      // Numeric cast doesn't give us what we want for int8_t
      stream << boost::numeric_cast<int>(value);
  }
  template <> inline void
  copyValueString<uint8_t> (const pcl::PCLPointCloud2 &cloud,
                            const unsigned int point_index, 
                            const int point_size, 
                            const unsigned int field_idx, 
                            const unsigned int fields_count, 
                            std::ostream &stream)
  {
    uint8_t value;
    memcpy (&value, &cloud.data[point_index * point_size + cloud.fields[field_idx].offset + fields_count * sizeof (uint8_t)], sizeof (uint8_t));
    if (pcl_isnan (value))
      stream << "nan";
    else
      // Numeric cast doesn't give us what we want for uint8_t
      stream << boost::numeric_cast<int>(value);
  }

  /** \brief Check whether a given value of type Type (uchar, char, uint, int, float, double, ...) is finite or not
    *
    * \param[in] cloud the cloud that contains the data
    * \param[in] point_index the index of the point
    * \param[in] point_size the size of the point in the cloud
    * \param[in] field_idx the index of the dimension/field
    * \param[in] fields_count the current fields count
    *
    * \return true if the value is finite, false otherwise
    */
  template <typename Type> inline bool
  isValueFinite (const pcl::PCLPointCloud2 &cloud,
                 const unsigned int point_index, 
                 const int point_size, 
                 const unsigned int field_idx, 
                 const unsigned int fields_count)
  {
    Type value;
    memcpy (&value, &cloud.data[point_index * point_size + cloud.fields[field_idx].offset + fields_count * sizeof (Type)], sizeof (Type));
    if (!pcl_isfinite (value))
      return (false);
    return (true);
  }

  /** \brief Copy one single value of type T (uchar, char, uint, int, float, double, ...) from a string
    * 
    * Uses aoti/atof to do the conversion.
    * Checks if the st is "nan" and converts it accordingly.
    *
    * \param[in] st the string containing the value to convert and copy
    * \param[out] cloud the cloud to copy it to
    * \param[in] point_index the index of the point
    * \param[in] field_idx the index of the dimension/field
    * \param[in] fields_count the current fields count
    */
  template <typename Type> inline void
  copyStringValue (const std::string &st, pcl::PCLPointCloud2 &cloud,
                   unsigned int point_index, unsigned int field_idx, unsigned int fields_count)
  {
    Type value;
    if (boost::iequals(st, "nan"))
    {
      value = std::numeric_limits<Type>::quiet_NaN ();
      cloud.is_dense = false;
    }
    else
    {
      std::istringstream is (st);
      is.imbue (std::locale::classic ());
      if (!(is >> value))
        value = static_cast<Type> (atof (st.c_str ()));
    }

    memcpy (&cloud.data[point_index * cloud.point_step + 
                        cloud.fields[field_idx].offset + 
                        fields_count * sizeof (Type)], reinterpret_cast<char*> (&value), sizeof (Type));
  }

  template <> inline void
  copyStringValue<int8_t> (const std::string &st, pcl::PCLPointCloud2 &cloud,
                           unsigned int point_index, unsigned int field_idx, unsigned int fields_count)
  {
    int8_t value;
    if (boost::iequals(st, "nan"))
    {
      value = static_cast<int8_t> (std::numeric_limits<int>::quiet_NaN ());
      cloud.is_dense = false;
    }
    else
    {
      int val;
      std::istringstream is (st);
      is.imbue (std::locale::classic ());
      //is >> val;  -- unfortunately this fails on older GCC versions and CLANG on MacOS
      if (!(is >> val))
        val = static_cast<int> (atof (st.c_str ()));
      value = static_cast<int8_t> (val);
    }

    memcpy (&cloud.data[point_index * cloud.point_step + 
                        cloud.fields[field_idx].offset + 
                        fields_count * sizeof (int8_t)], reinterpret_cast<char*> (&value), sizeof (int8_t));
  }

  template <> inline void
  copyStringValue<uint8_t> (const std::string &st, pcl::PCLPointCloud2 &cloud,
                           unsigned int point_index, unsigned int field_idx, unsigned int fields_count)
  {
    uint8_t value;
    if (boost::iequals(st, "nan"))
    {
      value = static_cast<uint8_t> (std::numeric_limits<int>::quiet_NaN ());
      cloud.is_dense = false;
    }
    else
    {
      int val;
      std::istringstream is (st);
      is.imbue (std::locale::classic ());
      //is >> val;  -- unfortunately this fails on older GCC versions and CLANG on MacOS
      if (!(is >> val))
        val = static_cast<int> (atof (st.c_str ()));
      value = static_cast<uint8_t> (val);
    }

    memcpy (&cloud.data[point_index * cloud.point_step + 
                        cloud.fields[field_idx].offset + 
                        fields_count * sizeof (uint8_t)], reinterpret_cast<char*> (&value), sizeof (uint8_t));
  }

}

#endif  //#ifndef PCL_IO_FILE_IO_H_

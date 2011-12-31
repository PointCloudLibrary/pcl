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

#ifndef PCL_IO_PLY_IO_H_
#define PCL_IO_PLY_IO_H_

#include "pcl/io/file_io.h"
#include "pcl/io/ply.h"
#include <pcl/PolygonMesh.h>
#include <sstream>

namespace pcl
{
  /** \brief Point Cloud Data (PLY) file format reader.
    *
    * The PLY data format is organized in the following way:
    *   - lines beginning with "comment" are treated as comments
    *   - ply
    *   - format [ascii|binary_little_endian|binary_big_endian] 1.0
    *   - element vertex COUNT
    *   - [ascii/binary] point coordinates
    *   
    * \author Nizar Sallem
    * \ingroup io
    */
  class PCL_EXPORTS PLYReader : public FileReader
  {
    public:
      enum
      {
        PLY_V0 = 0,
        PLY_V1 = 1
      };

      /** \brief Read a point cloud data header from a PLY file.
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given PLY file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * Returns:
        *  * < 0 (-1) on error
        *  * > 0 on success
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[out] ply_version the PLY version read from the file
        * \param[out] data_type the type of PLY data stored in the file
        * \param[out] data_idx the data index
        */
      int 
      readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                  int &ply_version, int &data_type, int &data_idx);

      /** \brief Read a point cloud data from a PLY file and store it into a sensor_msgs/PointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[out] ply_version the PLY version read from the file
        */
      int 
      read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int& ply_version);

      /** \brief Read a point cloud data from a PLY file (PLY_V6 only!) and store it into a sensor_msgs/PointCloud2.
        *
        * \note This function is provided for backwards compatibility only and
        * it can only read PLY_V6 files correctly, as sensor_msgs::PointCloud2
        * does not contain a sensor origin/orientation. Reading any file
        * > PLY_V6 will generate a warning.
        *
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        */
      inline int 
      read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud)
      {
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        int ply_version;
        return read (file_name, cloud, origin, orientation, ply_version);
      }

      /** \brief Read a point cloud data from any PLY file, and convert it to the given template format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        */
      template<typename PointT> inline int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
      {
        sensor_msgs::PointCloud2 blob;
        int ply_version;
        int res = read (file_name, blob, cloud.sensor_origin_, cloud.sensor_orientation_,
                        ply_version);

        // Exit in case of error
        if (res < 0)
          return (res);
        pcl::fromROSMsg (blob, cloud);
        return (0);
      }
      
    private:
      pcl::io::ply::parser parser_;
      bool swap_bytes_;

      /** \brief Copy one single value of type T (uchar, char, uint, int, float, double, ...) from a string
        *
        * Uses atoi/atof to do the conversion.
        * Checks if the st is "nan" and converts it accordingly.
        *
        * \param[in] string_value the string containing the value to convert and copy
        * \param[out] data the output data
        * \param[in] offset the data offset (e.g., where to copy)
        */
      template <typename Type> inline void
      copyStringValue (const std::string &string_value, void* data, size_t offset = 0)
      {
        //char value = (char)atoi (st.at (d + c).c_str ());
        static char* char_ptr;
        char_ptr = (char*) data;
        Type value;
        std::istringstream is(string_value);
        is >> value;
        memcpy (char_ptr+offset, &value, sizeof (Type));
      }
  };

  /** \brief Point Cloud Data (PLY) file format writer.
    * \author Nizar Sallem
    * \ingroup io
    */
  class PCL_EXPORTS PLYWriter : public FileWriter
  {
    public:
      PLYWriter () : mask_ (0) {};
      ~PLYWriter () {};

      /** \brief Generate the header of a PLY v.7 file format
        * \param[in] cloud the point cloud data message
        */
      inline std::string
      generateHeaderBinary (const sensor_msgs::PointCloud2 &cloud)
      {
        return (generateHeader (cloud, true));
      }
      
      /** \brief Generate the header of a PLY v.7 file format
        * \param[in] cloud the point cloud data message
        */
      inline std::string
      generateHeaderASCII (const sensor_msgs::PointCloud2 &cloud)

      {
        return (generateHeader (cloud, false));
      }

      /** \brief Save point cloud data to a PLY file containing n-D points, in ASCII format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[in] precision the specified output numeric stream precision (default: 8)
        */
      int 
      writeASCII (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
                  const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                  const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                  int precision = 8);

      /** \brief Save point cloud data to a PLY file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        */
      int 
      writeBinary (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud,
                   const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                   const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity ());

      /** \brief Save point cloud data to a PLY file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] binary set to true if the file is to be written in a binary
        * PLY format, false (default) for ASCII
        */
      inline int
      write (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             bool binary = false)
      {
        if (binary)
          return (this->writeBinary (file_name, cloud, origin, orientation));
        else
          return (this->writeASCII (file_name, cloud, origin, orientation, 8));
      }

      /** \brief Save point cloud data to a PLY file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message (boost shared pointer)
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] binary set to true if the file is to be written in a binary
        * PLY format, false (default) for ASCII
        */
      inline int
      write (const std::string &file_name, const sensor_msgs::PointCloud2::ConstPtr &cloud, 
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             bool binary = false)
      {
        return (write (file_name, *cloud, origin, orientation, binary));
      }

      /** \brief Save point cloud data to a PLY file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the pcl::PointCloud data
        * \param[in] binary set to true if the file is to be written in a binary
        * PLY format, false (default) for ASCII
        */
      template<typename PointT> inline int
      write (const std::string &file_name, 
             const pcl::PointCloud<PointT> &cloud, 
             bool binary = false)
      {
        Eigen::Vector4f origin = cloud.sensor_origin_;
        Eigen::Quaternionf orientation = cloud.sensor_orientation_;

        sensor_msgs::PointCloud2 blob;
        pcl::toROSMsg (cloud, blob);

        // Save the data
        return (this->write (file_name, blob, origin, orientation, binary));
      }
      
    private:
      /** \brief Generate a PLY header.
        * \param[in] cloud the input point cloud
        * \param[in] binary whether the PLY file should be saved as binary data (true) or ascii (false)
        */
      std::string
      generateHeader (const sensor_msgs::PointCloud2 &cloud, bool binary);

      /** \brief Construct a mask from a list of fields.
        * \param[in] fields_list the list of fields to construct a mask from
        */
      void 
      setMaskFromFieldsList (const std::string& fields_list);

      /** \brief Internally used mask. */
      int mask_;
  };

  namespace io
  {
    /** \brief Load a PLY v.6 file into a templated PointCloud type.
      *
      * Any PLY files containg sensor data will generate a warning as a
      * sensor_msgs/PointCloud2 message cannot hold the sensor origin.
      *
      * \param[in] file_name the name of the file to load
      * \param[in] cloud the resultant templated point cloud
      * \ingroup io
      */
    inline int
    loadPLYFile (const std::string &file_name, sensor_msgs::PointCloud2 &cloud)
    {
      pcl::PLYReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Load any PLY file into a templated PointCloud type.
      * \param[in] file_name the name of the file to load
      * \param[in] cloud the resultant templated point cloud
      * \param[in] origin the sensor acquisition origin (only for > PLY_V7 - null if not present)
      * \param[in] orientation the sensor acquisition orientation if availble, 
      * identity if not present
      * \ingroup io
      */
    inline int
    loadPLYFile (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                 Eigen::Vector4f &origin, Eigen::Quaternionf &orientation)
    {
      pcl::PLYReader p;
      int ply_version;
      return (p.read (file_name, cloud, origin, orientation, ply_version));
    }

    /** \brief Load any PLY file into a templated PointCloud type
      * \param[in] file_name the name of the file to load
      * \param[in] cloud the resultant templated point cloud
      * \ingroup io
      */
    template<typename PointT> inline int
    loadPLYFile (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
    {
      pcl::PLYReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Save point cloud data to a PLY file containing n-D points
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor data acquisition origin (translation)
      * \param[in] orientation the sensor data acquisition origin (rotation)
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      * \ingroup io
      */
    inline int 
    savePLYFile (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
                 const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                 const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                 bool binary_mode = false)
    {
      PLYWriter w;
      return (w.write (file_name, cloud, origin, orientation, binary_mode));
    }

    /** \brief Templated version for saving point cloud data to a PLY file
      * containing a specific given cloud format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      * \ingroup io
      */
    template<typename PointT> inline int
    savePLYFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud, bool binary_mode = false)
    {
      PLYWriter w;
      return (w.write<PointT> (file_name, cloud, binary_mode));
    }

    /** \brief Templated version for saving point cloud data to a PLY file
      * containing a specific given cloud format.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \ingroup io
      */
    template<typename PointT> inline int
    savePLYFileASCII (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PLYWriter w;
      return (w.write<PointT> (file_name, cloud, false));
    }

    /** \brief Templated version for saving point cloud data to a PLY file containing a specific given cloud format.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \ingroup io
      */
    template<typename PointT> inline int
    savePLYFileBinary (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PLYWriter w;
      return (w.write<PointT> (file_name, cloud, true));
    }

    /** \brief Templated version for saving point cloud data to a PLY file containing a specific given cloud format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] indices the set of indices to save
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      * \ingroup io
      */
    template<typename PointT> int
    savePLYFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud,
                 const std::vector<int> &indices, bool binary_mode = false)
    {
      // Copy indices to a new point cloud
      pcl::PointCloud<PointT> cloud_out;
      copyPointCloud (cloud, indices, cloud_out);
      // Save the data
      PLYWriter w;
      return (w.write<PointT> (file_name, cloud_out, binary_mode));
    }

    /** \brief Saves a PolygonMesh in ascii PLY format.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] mesh the polygonal mesh to save
      * \param[in] precision the output ASCII precision default 5
      * \ingroup io
      */
    PCL_EXPORTS int
    savePLYFile (const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision = 5);

  };
}

#endif  //#ifndef PCL_IO_PLY_IO_H_

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
 * $Id: pcd_io.h 2316 2011-08-30 19:45:01Z jspricke $
 *
 */

#ifndef PCL_IO_PCD_IO_H_
#define PCL_IO_PCD_IO_H_

#include "pcl/io/file_io.h"

namespace pcl
{
  /** \brief Point Cloud Data (PCD) file format reader.
    * \author Radu Bogdan Rusu
    * \ingroup io
    */
  class PCL_EXPORTS PCDReader : public FileReader
  {
    public:
      /** Empty constructor */      
      PCDReader() : FileReader() {}
      /** Empty destructor */      
      ~PCDReader() {}
      /** \brief Various PCD file versions.
        *
        * PCD_V6 represents PCD files with version 0.6, which contain the following fields:
        * <ul>
        *  <li> lines beginning with # are treated as comments</li>
        *  <li> FIELDS ...</li>
        *  <li> SIZE ...</li>
        *  <li> TYPE ...</li>
        *  <li> COUNT ...</li>
        *  <li> WIDTH ...</li>
        *  <li> HEIGHT ...</li>
        *  <li> POINTS ...</li>
        *  <li> DATA ascii/binary</li>
        * </ul>
        * Everything that follows <b>DATA</b> is intepreted as data points and
        * will be read accordingly.
        *
        * PCD_V7 represents PCD files with version 0.7 and has an important
        * addon: it adds sensor origin/orientation (aka viewpoint) information
        * to a dataset through the use of a new header field:
        * <ul>
        *  <li> VIEWPOINT tx ty tz qw qx qy qz</li>
        * </ul>
        */
      enum
      {
        PCD_V6 = 0,
        PCD_V7 = 1
      };

      /** \brief Read a point cloud data header from a PCD file. 
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given PCD file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * Returns:
        *  * < 0 (-1) on error
        *  * > 0 on success
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
        * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
        * \param[out] orientation the sensor acquisition orientation (only for > PCD_V7 - identity if not present)
        * \param[out] pcd_version the PCD version of the file (either PCD_V6 or PCD_V7)
        * \param[out] binary_data is true if the PCD file contains binary data, false if ascii 
        * \param[out] data_idx the offset of cloud data within the file
        */
      int 
      readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, 
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version,
                  bool &binary_data, int &data_idx);

      /** \brief Read a point cloud data from a PCD file and store it into a sensor_msgs/PointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
        * \param[out] orientation the sensor acquisition orientation (only for > PCD_V7 - identity if not present)
        * \param[out] pcd_version the PCD version of the file (either PCD_V6 or PCD_V7)
        */
      int 
      read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, 
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version);

      /** \brief Read a point cloud data from a PCD (PCD_V6) and store it into a sensor_msgs/PointCloud2.
        * 
        * \note This function is provided for backwards compatibility only and
        * it can only read PCD_V6 files correctly, as sensor_msgs::PointCloud2
        * does not contain a sensor origin/orientation. Reading any file 
        * > PCD_V6 will generate a warning. 
        *
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        */
      int 
      read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud);

      /** \brief Read a point cloud data from any PCD file, and convert it to the given template format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        */
      template<typename PointT> inline int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
      {
        sensor_msgs::PointCloud2 blob;
        int pcd_version;
        int res = read (file_name, blob, cloud.sensor_origin_, cloud.sensor_orientation_, 
                        pcd_version);

        // Exit in case of error
        if (res < 0)
          return res;
        pcl::fromROSMsg (blob, cloud);
        return 0;
      }
  };

  /** \brief Point Cloud Data (PCD) file format writer.
    * \author Radu Bogdan Rusu
    * \ingroup io
    */
  class PCL_EXPORTS PCDWriter : public FileWriter
  {
    public:
      PCDWriter() : FileWriter() {}
      ~PCDWriter() {}

      /** \brief Generate the header of a PCD file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      std::string
      generateHeaderBinary (const sensor_msgs::PointCloud2 &cloud, 
                            const Eigen::Vector4f &origin, 
                            const Eigen::Quaternionf &orientation);

      /** \brief Generate the header of a PCD file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      std::string
      generateHeaderASCII (const sensor_msgs::PointCloud2 &cloud, 
                           const Eigen::Vector4f &origin, 
                           const Eigen::Quaternionf &orientation);

      /** \brief Save point cloud data to a PCD file containing n-D points, in ASCII format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] precision the specified output numeric stream precision (default: 8)
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
        *
        * As an intermediary solution, precision 8 is used, which guarantees lossless storage for RGB.
        */
      int 
      writeASCII (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
                  const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                  const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                  int precision = 8);

      /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      int 
      writeBinary (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud,
                   const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                   const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity ());

      /** \brief Save point cloud data to a PCD file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] binary set to true if the file is to be written in a binary
        * PCD format, false (default) for ASCII
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
        *
        * As an intermediary solution, precision 8 is used, which guarantees lossless storage for RGB.
        */
      inline int
      write (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             bool binary = false)
      {
        if (binary)
          return (writeBinary (file_name, cloud, origin, orientation));
        else
          return (writeASCII (file_name, cloud, origin, orientation, 8));
      }

      /** \brief Save point cloud data to a PCD file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message (boost shared pointer)
        * \param[in] binary set to true if the file is to be written in a binary PCD format, 
        * false (default) for ASCII
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
        */
      inline int
      write (const std::string &file_name, const sensor_msgs::PointCloud2::ConstPtr &cloud, 
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             bool binary = false)
      {
        return (write (file_name, *cloud, origin, orientation, binary));
      }

      /** \brief Save point cloud data to a PCD file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the pcl::PointCloud data
        * \param[in] binary set to true if the file is to be written in a binary
        * PCD format, false (default) for ASCII
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
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
        return (write (file_name, blob, origin, orientation, binary));
      }
  };

  namespace io
  {
    /** \brief Load a PCD v.6 file into a templated PointCloud type.
      * 
      * Any PCD files > v.6 will generate a warning as a
      * sensor_msgs/PointCloud2 message cannot hold the sensor origin.
      *
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \ingroup io
      */
    inline int 
    loadPCDFile (const std::string &file_name, sensor_msgs::PointCloud2 &cloud)
    {
      pcl::PCDReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Load any PCD file into a templated PointCloud type.
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
      * \param[out] orientation the sensor acquisition orientation (only for >
      * PCD_V7 - identity if not present)
      * \ingroup io
      */
    inline int 
    loadPCDFile (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                 Eigen::Vector4f &origin, Eigen::Quaternionf &orientation)
    {
      pcl::PCDReader p;
      int pcd_version;
      return (p.read (file_name, cloud, origin, orientation, pcd_version));
    }

    /** \brief Load any PCD file into a templated PointCloud type
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \ingroup io
      */
    template<typename PointT> inline int
    loadPCDFile (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
    {
      pcl::PCDReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Save point cloud data to a PCD file containing n-D points
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      * \ingroup io
      */
    inline int 
    savePCDFile (std::string file_name, const sensor_msgs::PointCloud2 &cloud, 
                 const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                 const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                 bool binary_mode = false)
    {
      PCDWriter w;
      return (w.write (file_name, cloud, origin, orientation, binary_mode));
    }

    /** \brief Templated version for saving point cloud data to a PCD file
      * containing a specific given cloud format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      * \ingroup io
      */
    template<typename PointT> inline int
    savePCDFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud, bool binary_mode = false)
    {
      PCDWriter w;
      return (w.write<PointT> (file_name, cloud, binary_mode));
    }

    /** 
      * \brief Templated version for saving point cloud data to a PCD file
      * containing a specific given cloud format.
      *
      *      This version is to retain backwards compatibility.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      * \ingroup io
      */
    template<typename PointT> inline int
    savePCDFileASCII (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PCDWriter w;
      return (w.write<PointT> (file_name, cloud, false));
    }

    /** 
      * \brief Templated version for saving point cloud data to a PCD file
      * containing a specific given cloud format.
      *
      *      This version is to retain backwards compatibility.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \ingroup io
      */
    template<typename PointT> inline int
    savePCDFileBinary (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PCDWriter w;
      return (w.write<PointT> (file_name, cloud, true));
    }

    /** 
      * \brief Templated version for saving point cloud data to a PCD file
      * containing a specific given cloud format
      *
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] indices the set of indices to save
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      * \ingroup io
      */
    template<typename PointT> int
    savePCDFile (const std::string &file_name, 
                 const pcl::PointCloud<PointT> &cloud,
                 const std::vector<int> &indices, 
                 bool binary_mode = false)
    {
      // Copy indices to a new point cloud
      pcl::PointCloud<PointT> cloud_out;
      copyPointCloud (cloud, indices, cloud_out);
      // Save the data
      PCDWriter w;
      return (w.write<PointT> (file_name, cloud_out, binary_mode));
    }
  };
}

#endif  //#ifndef PCL_IO_PCD_IO_H_

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
 * $Id$
 *
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/file_io.h>
#include <boost/interprocess/sync/file_lock.hpp> // for file_lock

namespace pcl
{
  /** \brief Point Cloud Data (PCD) file format reader.
    * \author Radu B. Rusu
    * \ingroup io
    */
  class PCL_EXPORTS PCDReader : public FileReader
  {
    public:
      /** Empty constructor */
      PCDReader () = default;
      /** Empty destructor */
      ~PCDReader () override = default;

      /** \brief Various PCD file versions.
        *
        * PCD_V6 represents PCD files with version 0.6, which contain the following fields:
        *   - lines beginning with # are treated as comments
        *   - FIELDS ...
        *   - SIZE ...
        *   - TYPE ...
        *   - COUNT ...
        *   - WIDTH ...
        *   - HEIGHT ...
        *   - POINTS ...
        *   - DATA ascii/binary
        *
        * Everything that follows \b DATA is interpreted as data points and
        * will be read accordingly.
        *
        * PCD_V7 represents PCD files with version 0.7 and has an important
        * addon: it adds sensor origin/orientation (aka viewpoint) information
        * to a dataset through the use of a new header field:
        *   - VIEWPOINT tx ty tz qw qx qy qz
        */
      enum
      {
        PCD_V6 = 0,
        PCD_V7 = 1
      };

      /** \brief Read a point cloud data header from a PCD-formatted, binary istream.
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given PCD stream. Useful for fast
        * evaluation of the underlying data structure.
        *
        * \attention The PCD data is \b always stored in ROW major format! The
        * read/write PCD methods will detect column major input and automatically convert it.
        *
        * \param[in] binary_istream a std::istream with openmode set to std::ios::binary.
        * \param[out] cloud the resultant point cloud dataset (only these
        *             members will be filled: width, height, point_step,
        *             row_step, fields[]; data is resized but not written)
        * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
        * \param[out] orientation the sensor acquisition orientation (only for > PCD_V7 - identity if not present)
        * \param[out] pcd_version the PCD version of the file (i.e., PCD_V6, PCD_V7)
        * \param[out] data_type the type of data (0 = ASCII, 1 = Binary, 2 = Binary compressed)
        * \param[out] data_idx the offset of cloud data within the file
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      readHeader (std::istream &binary_istream, pcl::PCLPointCloud2 &cloud,
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version,
                  int &data_type, unsigned int &data_idx);

      /** \brief Read a point cloud data header from a PCD file.
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given PCD file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * \attention The PCD data is \b always stored in ROW major format! The
        * read/write PCD methods will detect column major input and automatically convert it.
        *
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only these
        *             members will be filled: width, height, point_step,
        *             row_step, fields[]; data is resized but not written)
        * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
        * \param[out] orientation the sensor acquisition orientation (only for > PCD_V7 - identity if not present)
        * \param[out] pcd_version the PCD version of the file (i.e., PCD_V6, PCD_V7)
        * \param[out] data_type the type of data (0 = ASCII, 1 = Binary, 2 = Binary compressed)
        * \param[out] data_idx the offset of cloud data within the file
        * \param[in] offset the offset of where to expect the PCD Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PCD files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version,
                  int &data_type, unsigned int &data_idx, const int offset = 0) override;


      /** \brief Read a point cloud data header from a PCD file.
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given PCD file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * \attention The PCD data is \b always stored in ROW major format! The
        * read/write PCD methods will detect column major input and automatically convert it.
        *
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only these
        *             members will be filled: width, height, point_step,
        *             row_step, fields[]; data is resized but not written)
        * \param[in] offset the offset of where to expect the PCD Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PCD files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0);

      /** \brief Read the point cloud data (body) from a PCD stream.
        *
        * Reads the cloud points from a text-formatted stream.  For use after
        * readHeader(), when the resulting data_type == 0.
        *
        * \attention This assumes the stream has been seeked to the position
        * indicated by the data_idx result of readHeader().
        *
        * \param[in] stream the stream from which to read the body.
        * \param[out] cloud the resultant point cloud dataset to be filled.
        * \param[in] pcd_version the PCD version of the stream (from readHeader()).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      readBodyASCII (std::istream &stream, pcl::PCLPointCloud2 &cloud, int pcd_version);

      /** \brief Read the point cloud data (body) from a block of memory.
        *
        * Reads the cloud points from a binary-formatted memory block.  For use
        * after readHeader(), when the resulting data_type is nonzero.
        *
        * \param[in] data the memory location from which to read the body.
        * \param[out] cloud the resultant point cloud dataset to be filled.
        * \param[in] pcd_version the PCD version of the stream (from readHeader()).
        * \param[in] compressed indicates whether the PCD block contains compressed
        * data.  This should be true if the data_type returne by readHeader() == 2.
        * \param[in] data_idx the offset of the body, as reported by readHeader().
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      readBodyBinary (const unsigned char *data, pcl::PCLPointCloud2 &cloud,
                       int pcd_version, bool compressed, unsigned int data_idx);

      /** \brief Read a point cloud data from a PCD file and store it into a pcl/PCLPointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[out] origin the sensor acquisition origin (only for > PCD_V7 - null if not present)
        * \param[out] orientation the sensor acquisition orientation (only for > PCD_V7 - identity if not present)
        * \param[out] pcd_version the PCD version of the file (either PCD_V6 or PCD_V7)
        * \param[in] offset the offset of where to expect the PCD Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PCD files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version, const int offset = 0) override;

      /** \brief Read a point cloud data from a PCD (PCD_V6) and store it into a pcl/PCLPointCloud2.
        *
        * \note This function is provided for backwards compatibility only and
        * it can only read PCD_V6 files correctly, as pcl::PCLPointCloud2
        * does not contain a sensor origin/orientation. Reading any file
        * > PCD_V6 will generate a warning.
        *
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset of where to expect the PCD Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PCD files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0);

      /** \brief Read a point cloud data from any PCD file, and convert it to the given template format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset of where to expect the PCD Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PCD files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      template<typename PointT> int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud, const int offset = 0)
      {
        pcl::PCLPointCloud2 blob;
        int pcd_version;
        int res = read (file_name, blob, cloud.sensor_origin_, cloud.sensor_orientation_,
                        pcd_version, offset);

        // If no error, convert the data
        if (res == 0)
          pcl::fromPCLPointCloud2 (blob, cloud);
        return (res);
      }

      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** \brief Point Cloud Data (PCD) file format writer.
    * \author Radu Bogdan Rusu
    * \ingroup io
    */
  class PCL_EXPORTS PCDWriter : public FileWriter
  {
    public:
      PCDWriter() : map_synchronization_(false) {}
      ~PCDWriter() override = default;

      /** \brief Set whether mmap() synchornization via msync() is desired before munmap() calls.
        * Setting this to true could prevent NFS data loss (see
        * http://www.pcl-developers.org/PCD-IO-consistency-on-NFS-msync-needed-td4885942.html).
        * Default: false
        * \note This option should be used by advanced users only!
        * \note Please note that using msync() on certain systems can reduce the I/O performance by up to 80%!
        * \param[in] sync set to true if msync() should be called before munmap()
        */
      void
      setMapSynchronization (bool sync)
      {
        map_synchronization_ = sync;
      }

      /** \brief Generate the header of a PCD file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      std::string
      generateHeaderBinary (const pcl::PCLPointCloud2 &cloud,
                            const Eigen::Vector4f &origin,
                            const Eigen::Quaternionf &orientation);

      /** \brief Generate the header of a BINARY_COMPRESSED PCD file format
        * \param[out] os the stream into which to write the header
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      generateHeaderBinaryCompressed (std::ostream &os,
                                      const pcl::PCLPointCloud2 &cloud,
                                      const Eigen::Vector4f &origin,
                                      const Eigen::Quaternionf &orientation);

      /** \brief Generate the header of a BINARY_COMPRESSED PCD file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      std::string
      generateHeaderBinaryCompressed (const pcl::PCLPointCloud2 &cloud,
                                      const Eigen::Vector4f &origin,
                                      const Eigen::Quaternionf &orientation);

      /** \brief Generate the header of a PCD file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      std::string
      generateHeaderASCII (const pcl::PCLPointCloud2 &cloud,
                           const Eigen::Vector4f &origin,
                           const Eigen::Quaternionf &orientation);

      /** \brief Generate the header of a PCD file format
        * \param[in] cloud the point cloud data message
        * \param[in] nr_points if given, use this to fill in WIDTH, HEIGHT (=1), and POINTS in the header
        * By default, nr_points is set to INTMAX, and the data in the header is used instead.
        */
      template <typename PointT> static std::string
      generateHeader (const pcl::PointCloud<PointT> &cloud,
                      const int nr_points = std::numeric_limits<int>::max ());

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
      writeASCII (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                  const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
                  const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                  const int precision = 8);

      /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      int
      writeBinary (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                   const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
                   const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity ());

      /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY_COMPRESSED format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \return
        * (-1) for a general error
        * (-2) if the input cloud is too large for the file format
        * 0 on success
        */
      int
      writeBinaryCompressed (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
                             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity ());

      /** \brief Save point cloud data to a std::ostream containing n-D points, in BINARY_COMPRESSED format
        * \param[out] os the stream into which to write the data
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \return
        * (-1) for a general error
        * (-2) if the input cloud is too large for the file format
        * 0 on success
        */
      int
      writeBinaryCompressed (std::ostream &os, const pcl::PCLPointCloud2 &cloud,
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
      write (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             const bool binary = false) override
      {
        if (binary)
          return (writeBinary (file_name, cloud, origin, orientation));
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
      write (const std::string &file_name, const pcl::PCLPointCloud2::ConstPtr &cloud,
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             const bool binary = false)
      {
        return (write (file_name, *cloud, origin, orientation, binary));
      }

      /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        */
      template <typename PointT> int
      writeBinary (const std::string &file_name,
                   const pcl::PointCloud<PointT> &cloud);

      /** \brief Save point cloud data to a binary comprssed PCD file
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \return
        * (-1) for a general error
        * (-2) if the input cloud is too large for the file format
        * 0 on success
        */
      template <typename PointT> int
      writeBinaryCompressed (const std::string &file_name,
                             const pcl::PointCloud<PointT> &cloud);

      /** \brief Save point cloud data to a PCD file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] indices the set of point indices that we want written to disk
        */
      template <typename PointT> int
      writeBinary (const std::string &file_name,
                   const pcl::PointCloud<PointT> &cloud,
                   const pcl::Indices &indices);

      /** \brief Save point cloud data to a PCD file containing n-D points, in ASCII format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] precision the specified output numeric stream precision (default: 8)
        */
      template <typename PointT> int
      writeASCII (const std::string &file_name,
                  const pcl::PointCloud<PointT> &cloud,
                  const int precision = 8);

       /** \brief Save point cloud data to a PCD file containing n-D points, in ASCII format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] indices the set of point indices that we want written to disk
        * \param[in] precision the specified output numeric stream precision (default: 8)
        */
      template <typename PointT> int
      writeASCII (const std::string &file_name,
                  const pcl::PointCloud<PointT> &cloud,
                  const pcl::Indices &indices,
                  const int precision = 8);

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
             const bool binary = false)
      {
        if (binary)
          return (writeBinary<PointT> (file_name, cloud));
        return (writeASCII<PointT> (file_name, cloud));
      }

      /** \brief Save point cloud data to a PCD file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the pcl::PointCloud data
        * \param[in] indices the set of point indices that we want written to disk
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
             const pcl::Indices &indices,
             bool binary = false)
      {
        if (binary)
          return (writeBinary<PointT> (file_name, cloud, indices));
        return (writeASCII<PointT> (file_name, cloud, indices));
      }

    protected:
      /** \brief Set permissions for file locking (Boost 1.49+).
        * \param[in] file_name the file name to set permission for file locking
        * \param[in,out] lock the file lock
        */
      void
      setLockingPermissions (const std::string &file_name,
                             boost::interprocess::file_lock &lock);

      /** \brief Reset permissions for file locking (Boost 1.49+).
        * \param[in] file_name the file name to reset permission for file locking
        * \param[in,out] lock the file lock
        */
      void
      resetLockingPermissions (const std::string &file_name,
                               boost::interprocess::file_lock &lock);

    private:
      /** \brief Set to true if msync() should be called before munmap(). Prevents data loss on NFS systems. */
      bool map_synchronization_;
  };

  namespace io
  {
    /** \brief Load a PCD v.6 file into a templated PointCloud type.
      *
      * Any PCD files > v.6 will generate a warning as a
      * pcl/PCLPointCloud2 message cannot hold the sensor origin.
      *
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \ingroup io
      */
    inline int
    loadPCDFile (const std::string &file_name, pcl::PCLPointCloud2 &cloud)
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
    loadPCDFile (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
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
    savePCDFile (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                 const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
                 const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                 const bool binary_mode = false)
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
      * containing a specific given cloud format. The resulting file will be an uncompressed binary.
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
                 const pcl::Indices &indices,
                 const bool binary_mode = false)
    {
      // Save the data
      PCDWriter w;
      return (w.write<PointT> (file_name, cloud, indices, binary_mode));
    }


    /**
      * \brief Templated version for saving point cloud data to a PCD file
      * containing a specific given cloud format. This method will write a compressed binary file.
      *
      *      This version is to retain backwards compatibility.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \ingroup io
      */
    template<typename PointT> inline int
    savePCDFileBinaryCompressed (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PCDWriter w;
      return (w.writeBinaryCompressed<PointT> (file_name, cloud));
    }

  }
}

#include <pcl/io/impl/pcd_io.hpp>

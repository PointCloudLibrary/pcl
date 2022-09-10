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
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/PolygonMesh.h>

namespace pcl
{
  /** \brief Indexed Face set (IFS) file format reader. This file format is used for
    * the Brown Mesh Set for instance.
    * \author Nizar Sallem
    * \ingroup io
    */
  class PCL_EXPORTS IFSReader
  {
    public:
      /** Empty constructor */
      IFSReader () = default;
      /** Empty destructor */
      ~IFSReader () = default;

      /** \brief we support two versions
        * 1.0 classic
        * 1.1 with texture coordinates addon
        */
      enum
      {
        IFS_V1_0 = 0,
        IFS_V1_1 = 1
      };

      /** \brief Read a point cloud data header from an IFS file.
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given IFS file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only header will be filled)
        * \param[out] ifs_version the IFS version of the file (IFS_V1_0 or IFS_V1_1)
        * \param[out] data_idx the offset of cloud data within the file
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                  int &ifs_version, unsigned int &data_idx);

      /** \brief Read a point cloud data from an IFS file and store it into a pcl/PCLPointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PCLPointCloud2 blob read from disk
        * \param[out] ifs_version the IFS version of the file (either IFS_V1_0 or IFS_V1_1)
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, int &ifs_version);

      /** \brief Read a point cloud data from an IFS file and store it into a PolygonMesh.
        * \param[in] file_name the name of the file containing the mesh data
        * \param[out] mesh the resultant PolygonMesh
        * \param[out] ifs_version the IFS version of the file (either IFS_V1_0 or IFS_V1_1)
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int
      read (const std::string &file_name, pcl::PolygonMesh &mesh, int &ifs_version);

      /** \brief Read a point cloud data from an IFS file, and convert it to the
        * given template pcl::PointCloud format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      template<typename PointT> int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
      {
        pcl::PCLPointCloud2 blob;
        int ifs_version;
        cloud.sensor_origin_ = Eigen::Vector4f::Zero ();
        cloud.sensor_orientation_ = Eigen::Quaternionf::Identity ();
        int res = read (file_name, blob, ifs_version);

        // If no error, convert the data
        if (res == 0)
          pcl::fromPCLPointCloud2 (blob, cloud);
        return (res);
      }
  };

  /** \brief Point Cloud Data (IFS) file format writer.
    * \author Nizar Sallem
    * \ingroup io
    */
  class PCL_EXPORTS IFSWriter
  {
    public:
      IFSWriter() = default;
      ~IFSWriter() = default;

      /** \brief Save point cloud data to an IFS file containing 3D points.
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data
        * \param[in] cloud_name the point cloud name to be stored inside the IFS file.
        *
        * \return
        * * 0 on success
        * * < 0 on error
        */
      int
      write (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
             const std::string &cloud_name = "cloud");

      /** \brief Save point cloud data to an IFS file containing 3D points.
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud
        * \param[in] cloud_name the point cloud name to be stored inside the IFS file.
        *
        * \return
        * * 0 on success
        * * < 0 on error
        */
      template<typename PointT> int
      write (const std::string &file_name, const pcl::PointCloud<PointT> &cloud,
             const std::string &cloud_name = "cloud")
      {
        pcl::PCLPointCloud2 blob;
        pcl::toPCLPointCloud2<PointT> (cloud, blob);
        return (write (file_name, blob, cloud_name));
      }
  };

  namespace io
  {
    /** \brief Load an IFS file into a PCLPointCloud2 blob type.
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \return 0 on success < 0 on error
      *
      * \ingroup io
      */
    inline int
    loadIFSFile (const std::string &file_name, pcl::PCLPointCloud2 &cloud)
    {
      pcl::IFSReader p;
      int ifs_version;
      return (p.read (file_name, cloud, ifs_version));
    }

    /** \brief Load any IFS file into a templated PointCloud type.
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \return 0 on success < 0 on error
      *
      * \ingroup io
      */
    template<typename PointT> inline int
    loadIFSFile (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
    {
      pcl::IFSReader p;
      return (p.read<PointT> (file_name, cloud));
    }

    /** \brief Load any IFS file into a PolygonMesh type.
      * \param[in] file_name the name of the file to load
      * \param[out] mesh the resultant mesh
      * \return 0 on success < 0 on error
      *
      * \ingroup io
      */
    inline int
    loadIFSFile (const std::string &file_name, pcl::PolygonMesh &mesh)
    {
      pcl::IFSReader p;
      int ifs_version;
      return (p.read (file_name, mesh, ifs_version));
    }

    /** \brief Save point cloud data to an IFS file containing 3D points
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \return 0 on success < 0 on error
      *
      * \ingroup io
      */
    inline int
    saveIFSFile (const std::string &file_name, const pcl::PCLPointCloud2 &cloud)
    {
      pcl::IFSWriter w;
      return (w.write (file_name, cloud));
    }

    /** \brief Save point cloud data to an IFS file containing 3D points
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud
      * \return 0 on success < 0 on error
      *
      * \ingroup io
      */
    template<typename PointT> int
    saveIFSFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      pcl::IFSWriter w;
      return (w.write<PointT> (file_name, cloud));
    }
  }
}

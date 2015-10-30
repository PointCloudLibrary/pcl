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
 * $Id$
 *
 */

#ifndef PCL_IO_AUTO_IO_H_
#define PCL_IO_AUTO_IO_H_

#include <pcl/pcl_macros.h>
#include <pcl/common/io.h>
#include <pcl/io/boost.h>
#include <cmath>
#include <sstream>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>

namespace pcl
{

  namespace io
  {
    /** \brief Load a file into a PointCloud2 according to extension.
      * \param[in] file_name the name of the file to load
      * \param[out] blob the resultant pcl::PointCloud2 blob
      * \ingroup io
      */
    PCL_EXPORTS int
    load (const std::string& file_name, pcl::PCLPointCloud2& blob);

    /** \brief Load a file into a template PointCloud type according to extension.
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \ingroup io
      */
    template<typename PointT> int
    load (const std::string& file_name, pcl::PointCloud<PointT>& cloud);

    /** \brief Load a file into a PolygonMesh according to extension.
      * \param[in] file_name the name of the file to load
      * \param[out] mesh the resultant pcl::PolygonMesh
      * \ingroup io
      */
    PCL_EXPORTS int
    load (const std::string& file_name, pcl::PolygonMesh& mesh);

    /** \brief Load a file into a TextureMesh according to extension.
      * \param[in] file_name the name of the file to load
      * \param[out] mesh the resultant pcl::TextureMesh
      * \ingroup io
      */
    PCL_EXPORTS int
    load (const std::string& file_name, pcl::TextureMesh& mesh);

    /** \brief Save point cloud data to a binary file when available else to ASCII.
      * \param[in] file_name the output file name
      * \param[in] blob the point cloud data message
      * \param[in] precision float precision when saving to ASCII files
      * \ingroup io
      */
    PCL_EXPORTS int
    save (const std::string& file_name, const pcl::PCLPointCloud2& blob, unsigned precision = 5);

    /** \brief Save point cloud to a binary file when available else to ASCII.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud
      * \param[in] precision float precision when saving to ASCII files
      * \ingroup io
      */
    template<typename PointT> int
    save (const std::string& file_name, const pcl::PointCloud<PointT>& cloud, unsigned precision = 5);

    /** \brief Saves a TextureMesh to a binary file when available else to ASCII.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] tex_mesh the texture mesh to save
      * \param[in] precision float precision when saving to ASCII files
      * \ingroup io
      */
    PCL_EXPORTS int
    save (const std::string &file_name, const pcl::TextureMesh &tex_mesh, unsigned precision = 5);

    /** \brief Saves a PolygonMesh to a binary file when available else to ASCII.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] mesh the polygonal mesh to save
      * \param[in] precision float precision when saving to ASCII files
      * \ingroup io
      */
    PCL_EXPORTS int
    save (const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision = 5);
  }
}

#include <pcl/io/impl/auto_io.hpp>

#endif  //#ifndef PCL_IO_AUTO_IO_H_

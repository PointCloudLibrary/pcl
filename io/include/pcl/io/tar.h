/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

namespace pcl
{
  namespace io
  {
    /** \brief A TAR file's header, as described on 
      * http://en.wikipedia.org/wiki/Tar_%28file_format%29. 
      */
    struct TARHeader
    {
      char file_name[100];
      char file_mode[8];
      char uid[8];
      char gid[8];
      char file_size[12];
      char mtime[12];
      char chksum[8];
      char file_type[1];
      char link_file_name[100];
      char ustar[6];
      char ustar_version[2];
      char uname[32];
      char gname[32];
      char dev_major[8];
      char dev_minor[8];
      char file_name_prefix[155];
      char _padding[12];

      /** \brief get file size */
      unsigned int 
      getFileSize ()
      {
        unsigned int output = 0;
        char *str = file_size;
        for (int i = 0; i < 11; i++)
        {
          output = output * 8 + *str - '0';
          str++;
        }
        return (output);
      }
    };

    /** \brief Save a PointCloud dataset into a TAR file. 
      * Append if the file exists, or create a new one if not.
      * \remark till implemented will return FALSE
      */
      // param[in] tar_filename the name of the TAR file to save the cloud to
      // param[in] cloud the point cloud dataset to save
      // param[in] pcd_filename the internal name of the PCD file that should be stored in the TAR header
    template <typename PointT> bool
    saveTARPointCloud (const std::string& /*tar_filename*/,
                       const PointCloud<PointT>& /*cloud*/,
                       const std::string& /*pcd_filename*/)
    {
      return (false);
    }
  }
}

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

#ifndef PCL_IO_AUTO_IO_IMPL_H_
#define PCL_IO_AUTO_IO_IMPL_H_

// #include <pcl/io/file_io.h>
// #include <pcl/io/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ifs_io.h>
// #include <pcl/io/vtk_io.h>

namespace pcl
{
  namespace io
  {
    template<typename PointT> int
    load (const std::string& file_name, pcl::PointCloud<PointT>& cloud)
    {
      boost::filesystem::path p (file_name.c_str ());
      std::string extension = p.extension ().string ();
      int result = -1;
      if (extension == ".pcd")
        result = pcl::io::loadPCDFile (file_name, cloud);
      else if (extension == ".ply")
        result = pcl::io::loadPLYFile (file_name, cloud);
      else if (extension == ".ifs")
        result = pcl::io::loadIFSFile (file_name, cloud);
      else
      {
        PCL_ERROR ("[pcl::io::load] Don't know how to handle file with extension %s", extension.c_str ());
        result = -1;
      }
      return (result);
    }

    template<typename PointT> int
    save (const std::string& file_name, const pcl::PointCloud<PointT>& cloud)
    {
      boost::filesystem::path p (file_name.c_str ());
      std::string extension = p.extension ().string ();
      int result = -1;
      if (extension == ".pcd")
        result = pcl::io::savePCDFile (file_name, cloud, true);
      else if (extension == ".ply")
        result = pcl::io::savePLYFile (file_name, cloud, true);
      else if (extension == ".ifs")
        result = pcl::io::saveIFSFile (file_name, cloud);
      else
      {
        PCL_ERROR ("[pcl::io::save] Don't know how to handle file with extension %s", extension.c_str ());
        result = -1;
      }
      return (result);
    }
  }
}

#endif //PCL_IO_AUTO_IO_IMPL_H_
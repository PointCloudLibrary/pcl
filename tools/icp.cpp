/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;

int
main (int argc, char **argv)
{
  double dist = 0.05;
  pcl::console::parse_argument (argc, argv, "-d", dist);

  double rans = 0.05;
  pcl::console::parse_argument (argc, argv, "-r", rans);

  int iter = 50;
  pcl::console::parse_argument (argc, argv, "-i", iter);

  bool nonLinear = false;
  pcl::console::parse_argument (argc, argv, "-n", nonLinear);

  std::vector<int> pcd_indices;
  pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  CloudPtr model (new Cloud);
  if (pcl::io::loadPCDFile (argv[pcd_indices[0]], *model) == -1)
  {
    std::cout << "Could not read file" << std::endl;
    return -1;
  }
  std::cout << argv[pcd_indices[0]] << " width: " << model->width << " height: " << model->height << std::endl;

  std::string result_filename (argv[pcd_indices[0]]);
  result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
  pcl::io::savePCDFile (result_filename.c_str (), *model);
  std::cout << "saving first model to " << result_filename << std::endl;

  Eigen::Matrix4f t (Eigen::Matrix4f::Identity ());

  for (size_t i = 1; i < pcd_indices.size (); i++)
  {
    CloudPtr data (new Cloud);
    if (pcl::io::loadPCDFile (argv[pcd_indices[i]], *data) == -1)
    {
      std::cout << "Could not read file" << std::endl;
      return -1;
    }
    std::cout << argv[pcd_indices[i]] << " width: " << data->width << " height: " << data->height << std::endl;

    pcl::IterativeClosestPoint<PointType, PointType> *icp;

    if (nonLinear)
    {
      std::cout << "Using IterativeClosestPointNonLinear" << std::endl;
      icp = new pcl::IterativeClosestPointNonLinear<PointType, PointType>();
    }
    else
    {
      std::cout << "Using IterativeClosestPoint" << std::endl;
      icp = new pcl::IterativeClosestPoint<PointType, PointType>();
    }

    icp->setMaximumIterations (iter);
    icp->setMaxCorrespondenceDistance (dist);
    icp->setRANSACOutlierRejectionThreshold (rans);

    icp->setInputTarget (model);

    icp->setInputSource (data);

    CloudPtr tmp (new Cloud);
    icp->align (*tmp);

    t = t * icp->getFinalTransformation ();

    pcl::transformPointCloud (*data, *tmp, t);

    std::cout << icp->getFinalTransformation () << std::endl;

    *model = *data;

    std::string result_filename (argv[pcd_indices[i]]);
    result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
    pcl::io::savePCDFileBinary (result_filename.c_str (), *tmp);
    std::cout << "saving result to " << result_filename << std::endl;
  }

  return 0;
}

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <string>
#include <iostream>
#include <vector>


using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;


int
main (int argc, char **argv)
{

  int iter = 35;
  pcl::console::parse_argument (argc, argv, "-i", iter);

  float ndt_res = 1.0f;
  pcl::console::parse_argument (argc, argv, "-r", ndt_res);

  double step_size = 0.1;
  pcl::console::parse_argument (argc, argv, "-s", step_size);

  double trans_eps = 0.01;
  pcl::console::parse_argument (argc, argv, "-t", trans_eps);

  float filter_res = 0.2f;
  pcl::console::parse_argument (argc, argv, "-f", filter_res);

  bool display_help = false;
  pcl::console::parse_argument (argc, argv, "--help", display_help);

  if (display_help || argc <= 1)
  {
    std::cout << "Usage: ndt3d [OPTION]... [FILE]..." << std::endl;
    std::cout << "Registers PCD files using 3D Normal Distributions Transform algorithm" << std::endl << std::endl;
    std::cout << "  -i          maximum number of iterations" << std::endl;
    std::cout << "  -r          resolution (in meters) of NDT grid" << std::endl;
    std::cout << "  -s          maximum step size (in meters) of newton optimizer" << std::endl;
    std::cout << "  -t          transformation epsilon used for termination condition" << std::endl;
    std::cout << "  -f          voxel filter resolution (in meters) used on source cloud" << std::endl;
    std::cout << "     --help   display this help and exit" << std::endl;

    return (0);
  }

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
  result_filename = result_filename.substr (result_filename.rfind ('/') + 1);
  pcl::io::savePCDFile (result_filename, *model);
  std::cout << "saving first model to " << result_filename << std::endl;

  Eigen::Matrix4f t (Eigen::Matrix4f::Identity ());

  pcl::ApproximateVoxelGrid<PointType> voxel_filter;
  voxel_filter.setLeafSize (filter_res, filter_res, filter_res);

  for (std::size_t i = 1; i < pcd_indices.size (); i++)
  {
    CloudPtr data (new Cloud);
    if (pcl::io::loadPCDFile (argv[pcd_indices[i]], *data) == -1)
    {
      std::cout << "Could not read file" << std::endl;
      return -1;
    }
    std::cout << argv[pcd_indices[i]] << " width: " << data->width << " height: " << data->height << std::endl;

    auto * ndt = new pcl::NormalDistributionsTransform<PointType, PointType>();

    ndt->setMaximumIterations (iter);
    ndt->setResolution (ndt_res);
    ndt->setStepSize (step_size);
    ndt->setTransformationEpsilon (trans_eps);

    ndt->setInputTarget (model);

    CloudPtr filtered_data (new Cloud);
    voxel_filter.setInputCloud (data);
    voxel_filter.filter (*filtered_data);

    ndt->setInputSource (filtered_data);

    CloudPtr tmp (new Cloud);
    ndt->align (*tmp);

    t *= ndt->getFinalTransformation ();

    pcl::transformPointCloud (*data, *tmp, t);

    std::cout << ndt->getFinalTransformation () << std::endl;

    *model = *data;

    std::string result_filename (argv[pcd_indices[i]]);
    result_filename = result_filename.substr (result_filename.rfind ('/') + 1);
    pcl::io::savePCDFileBinary (result_filename, *tmp);
    std::cout << "saving result to " << result_filename << std::endl;
  }

  return (0);
}

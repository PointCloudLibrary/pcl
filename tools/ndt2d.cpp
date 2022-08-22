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
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt_2d.h>

#include <string>
#include <iostream>
#include <vector>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;


void
selfTest ()
{
  CloudPtr model (new Cloud);
  model->points.emplace_back(1,1,0);  
  model->points.emplace_back(4,4,0); 
  model->points.emplace_back(5,6,0);
  model->points.emplace_back(3,3,0);
  model->points.emplace_back(6,7,0);
  model->points.emplace_back(7,11,0);
  model->points.emplace_back(12,15,0);
  model->points.emplace_back(7,12,0);

  CloudPtr data (new Cloud);
  data->points.emplace_back(3,1,0);
  data->points.emplace_back(7,4,0);
  data->points.emplace_back(9,6,0);

  pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);  
  
  pcl::NormalDistributionsTransform2D<PointType, PointType> ndt;

  ndt.setMaximumIterations (40);
  ndt.setGridCentre (Eigen::Vector2f (0,0));
  ndt.setGridExtent (Eigen::Vector2f (20,20));
  ndt.setGridStep (Eigen::Vector2f (20,20));
  ndt.setOptimizationStepSize (Eigen::Vector3d (0.4,0.4,0.1));
  ndt.setTransformationEpsilon (1e-9);

  ndt.setInputTarget (model);
  ndt.setInputSource (data);

  CloudPtr tmp (new Cloud);
  ndt.align (*tmp);
  std::cout << ndt.getFinalTransformation () << std::endl;
}


int
main (int argc, char **argv)
{
  int iter = 10;
  double grid_step = 3.0;
  double grid_extent = 25.0;
  double optim_step = 1.0;

  pcl::console::parse_argument (argc, argv, "-i", iter);
  pcl::console::parse_argument (argc, argv, "-g", grid_step);
  pcl::console::parse_argument (argc, argv, "-e", grid_extent);
  pcl::console::parse_argument (argc, argv, "-s", optim_step);

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
  try
  {
    pcl::io::savePCDFile (result_filename, *model);
    std::cout << "saving first model to " << result_filename << std::endl;
  }
  catch(pcl::IOException& e)
  {
    std::cerr << e.what() << std::endl;
  }

  Eigen::Matrix4f t (Eigen::Matrix4f::Identity ());

  for (std::size_t i = 1; i < pcd_indices.size (); i++)
  {
    CloudPtr data (new Cloud);
    if (pcl::io::loadPCDFile (argv[pcd_indices[i]], *data) == -1)
    {
      std::cout << "Could not read file" << std::endl;
      return -1;
    }
    std::cout << argv[pcd_indices[i]] << " width: " << data->width << " height: " << data->height << std::endl;

    //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

    pcl::NormalDistributionsTransform2D<PointType, PointType> ndt;

    ndt.setMaximumIterations (iter);
    ndt.setGridCentre (Eigen::Vector2f (15,0));
    ndt.setGridExtent (Eigen::Vector2f (grid_extent,grid_extent));
    ndt.setGridStep (Eigen::Vector2f (grid_step,grid_step));
    ndt.setOptimizationStepSize (optim_step);
    ndt.setTransformationEpsilon (1e-5);

    ndt.setInputTarget (model);
    ndt.setInputSource (data);

    CloudPtr tmp (new Cloud);
    ndt.align (*tmp);

    t *= ndt.getFinalTransformation ();

    pcl::transformPointCloud (*data, *tmp, t);

    std::cout << ndt.getFinalTransformation () << std::endl;

    *model = *data;

    try
    {
      std::string result_filename (argv[pcd_indices[i]]);
      result_filename = result_filename.substr (result_filename.rfind ('/') + 1);
      pcl::io::savePCDFileBinary (result_filename, *tmp);
      std::cout << "saving result to " << result_filename << std::endl;
    }
    catch(pcl::IOException& e)
    {
      std::cerr << e.what() << std::endl;
    }
  }

  return 0;
}

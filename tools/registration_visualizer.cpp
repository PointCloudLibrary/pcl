/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/registration_visualizer.h>

int
main (int argc, char** argv)
{
//  /////////////////////////////////////////////////////////////////////////////////////////////////////

  if (argc != 3)
  {
    std::cerr << "please specify the paths to the two point clouds to be registered" << std::endl;
    exit (0);
  }
//  /////////////////////////////////////////////////////////////////////////////////////////////////////

//  /////////////////////////////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ> inputCloud;
  pcl::PointCloud<pcl::PointXYZ> targetCloud;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], inputCloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the first .pcd file \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], targetCloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the second .pcd file \n");
    return (-1);
  }
//  /////////////////////////////////////////////////////////////////////////////////////////////////////

//  /////////////////////////////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ> inputCloudFiltered;
  pcl::PointCloud<pcl::PointXYZ> targetCloudFiltered;

  pcl::VoxelGrid<pcl::PointXYZ> sor;
//  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
//  sor.setLeafSize (0.05, 0.05, 0.05);
//  sor.setLeafSize (0.1, 0.1, 0.1);
//  sor.setLeafSize (0.4, 0.4, 0.4);
//  sor.setLeafSize (0.5, 0.5, 0.5);

  sor.setInputCloud (inputCloud.makeShared());
  std::cout<<"\n inputCloud.size()="<<inputCloud.size()<<std::endl;
  sor.filter (inputCloudFiltered);
  std::cout<<"\n inputCloudFiltered.size()="<<inputCloudFiltered.size()<<std::endl;

  sor.setInputCloud (targetCloud.makeShared());
  std::cout<<"\n targetCloud.size()="<<targetCloud.size()<<std::endl;
  sor.filter (targetCloudFiltered);
  std::cout<<"\n targetCloudFiltered.size()="<<targetCloudFiltered.size()<<std::endl;
//  /////////////////////////////////////////////////////////////////////////////////////////////////////

//  /////////////////////////////////////////////////////////////////////////////////////////////////////
//  pcl::PointCloud<pcl::PointXYZ>::ConstPtr source = inputCloud.makeShared();
//  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target = targetCloud.makeShared();

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr source = inputCloudFiltered.makeShared();
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target = targetCloudFiltered.makeShared();

  pcl::PointCloud<pcl::PointXYZ> source_aligned;
//  /////////////////////////////////////////////////////////////////////////////////////////////////////

//  /////////////////////////////////////////////////////////////////////////////////////////////////////
  pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ> registrationVisualizer;

  registrationVisualizer.startDisplay();

  registrationVisualizer.setMaximumDisplayedCorrespondences (100);
//  /////////////////////////////////////////////////////////////////////////////////////////////////////

//  ///////////////////////////////////////////////////////////////////////////////////////////////////
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  icp.setMaximumIterations(10000);

//  icp.setMaxCorrespondenceDistance (0.6);
  icp.setMaxCorrespondenceDistance (0.8);
//  icp.setMaxCorrespondenceDistance (1.5);

//  icp.setRANSACOutlierRejectionThreshold (0.1);
  icp.setRANSACOutlierRejectionThreshold (0.6);
//  icp.setRANSACOutlierRejectionThreshold (1.5);
//  icp.setRANSACOutlierRejectionThreshold (5.0);

  icp.setInputSource (source);
  icp.setInputTarget (target);

  // Register the registration algorithm to the RegistrationVisualizer
  registrationVisualizer.setRegistration (icp);

  // Start registration process
  icp.align (source_aligned);

  std::cout << "has converged:" << icp.hasConverged () << " score: " << icp.getFitnessScore () << std::endl;
  std::cout << icp.getFinalTransformation () << std::endl;
//  ///////////////////////////////////////////////////////////////////////////////////////////////////

//  ///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icpnl;
//
//  icpnl.setMaximumIterations(10000);
//
//  icpnl.setMaxCorrespondenceDistance (0.6);
////  icpnl.setMaxCorrespondenceDistance (0.8);
////  icpnl.setMaxCorrespondenceDistance (1.5);
//
////  icpnl.setRANSACOutlierRejectionThreshold (0.1);
//  icpnl.setRANSACOutlierRejectionThreshold (0.8);
////  icpnl.setRANSACOutlierRejectionThreshold (1.5);
////  icpnl.setRANSACOutlierRejectionThreshold (5.0);
//
//  icpnl.setInputCloud  (source);
//  icpnl.setInputTarget (target);
//
//  // Register the registration algorithm to the RegistrationVisualizer
//  registrationVisualizer.setRegistration (icpnl);
//
//  // Start registration process
//  icpnl.align (source_aligned);
//
//  std::cout << "has converged:" << icpnl.hasConverged () << " score: " << icpnl.getFitnessScore () << std::endl;
//  std::cout << icpnl.getFinalTransformation () << std::endl;
//
//  ///////////////////////////////////////////////////////////////////////////////////////////////////

}

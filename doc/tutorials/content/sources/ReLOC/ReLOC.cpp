/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2016-, Open Perception, Inc.
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
*
*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ReLOC.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>


void main (int argc, char** argv)
{
  std::cout << "Load target" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *target) == -1) 
  {
    PCL_ERROR ("Couldn't read file argv[1] \n");
    return ;
  }

  std::cout << "Load source" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *source) == -1) 
  {
    PCL_ERROR ("Couldn't read file argv[2] \n");
    return ;
  }

  //apply a translation to the target for better disambiguation of normal signs
  const Eigen::Vector3f translation_target(0, -0.3, 0.5);
  const Eigen::Quaternionf rotation_target(0.0f, 0.0f, 0.0f, 0.0f);

  pcl::transformPointCloud(*target, *target, translation_target, rotation_target);

  //apply a translation to the source for better disambiguation of normal signs
  const Eigen::Vector3f translation_source(0, 0.3, 0.5);
  const Eigen::Quaternionf rotation_source(0.0f, 0.0f, 0.0f, 0.0f);

  pcl::transformPointCloud(*source, *source, translation_source, rotation_source);


  //Provide an estimation of the average distance between adjacent points of the two clouds (mesh resolution)
  const float mesh_res = 0.004f;


  // Estimate normals for source and target
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  ne.setRadiusSearch (3.0 * mesh_res);

  std::cout << "Estimate target normals" << std::endl;
  pcl::PointCloud<pcl::Normal>::Ptr normals_trg (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud (target);
  ne.compute (*normals_trg);

  std::cout << "Estimate source normals" << std::endl;
  pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(source);
  ne.compute(*normals_src);
  


  //Apply ReLOC algorithm
  pcl::registration::ReLOCInitialAlignment <pcl::PointXYZ, pcl::PointXYZ> ReLOC_ia;
  ReLOC_ia.setInputSource (source);
  ReLOC_ia.setInputTarget (target);
  ReLOC_ia.setSourceNormals (normals_src); 
  ReLOC_ia.setTargetNormals (normals_trg); 

  //set detector parameters
  ReLOC_ia.setSeed (0xFFFFFFFF);
  ReLOC_ia.setFlatKeypointRf (3.0*mesh_res); //5.0 in the original paper
  ReLOC_ia.setFlatKeypointRdiscard (2.0*mesh_res);
  ReLOC_ia.setFlatKeypointR1search (2.0*mesh_res);
  ReLOC_ia.setFlatKeypointR2search (10.0*mesh_res);
  //apply random detection instead of FlatKeypoint detector
  ReLOC_ia.useRandomDetector (true);
  ReLOC_ia.setNrandomKeypoints (300);

  //set FLARE parameters
  ReLOC_ia.setFlareNormalRadius (5.0*mesh_res);
  ReLOC_ia.setFlareTangentRadius (20.0*mesh_res);
  //ReLOC_ia.setFlareXsupportSamplingPerc (0.2); //used to speed up LRFs computation. Consider only a percentage of the points of the support used for the computation of x axis. 

  //set Hough voting and RANSAC parameters
  ReLOC_ia.setHoughSbin (2.0*mesh_res);
  ReLOC_ia.setRansacT (4.0*mesh_res); //8.0 in the original paper

  //estimate rigid motion and apply to source cloud
  pcl::PointCloud <pcl::PointXYZ> source_aligned;
  ReLOC_ia.align (source_aligned);
  Eigen::Matrix4f rigidMotion_4f = ReLOC_ia.getFinalTransformation();

  std::cout << "Estimated rigid motion:" << std::endl;
  std::cout << rigidMotion_4f;

}

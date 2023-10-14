/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id:$
 *
 */

#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_torus.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>

#include <iostream>

#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/sample_consensus/sac_model_sphere.h>

float
rf()
{
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr
getTorus(size_t size)
{
  float R = 1;
  float r = 0.05;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>());
  cl->width    = size;
  cl->height   = 1;
  cl->is_dense = false;
  cl->points.reserve (cl->width * cl->height);
  for (size_t i = 0; i < size; i++) {
    pcl::PointXYZ pt;
    float theta = 2 * M_PI * rf();
    float phi = 2 * M_PI * rf();
    pt.x = (R + r * cos(theta)) * cos(phi) + rf() * 0.05;
    pt.y = (R + r * cos(theta)) * sin(phi) + rf() * 0.05;
    pt.z = r * sin(theta) + rf() * 0.05;
    cl->push_back(pt);
  }
  const double degToRad = 180 / M_PI;
  Eigen::AngleAxisd yawAngle(1, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(1, Eigen::Vector3d::UnitX());

  Eigen::Quaternion<double> q = yawAngle * pitchAngle;

  Eigen::Affine3d trans = Eigen::Affine3d::Identity();
  trans.rotate(q);
  trans.translation() << 1, 1, 1.5;
  pcl::transformPointCloud(*cl, *cl, trans.cast<float>().matrix());

  return cl;
}

int
main(int argc, char** av)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = getTorus(400);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelTorus<pcl::PointXYZ>::Ptr model_s(
      new pcl::SampleConsensusModelTorus<pcl::PointXYZ>(cloud));

  std::vector<int> inliers;
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
  ransac.setDistanceThreshold(0.05);

  ransac.computeModel();
  ransac.getInliers(inliers);

  Eigen::VectorXf  mc;
  ransac.getModelCoefficients(mc);
  std::cout << mc << std::endl;

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud(*cloud, inliers, *final);
  return (0);
}


int
main1(int argc, char** argv)
{
 // initialize PointClouds
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

 // populate our PointCloud with points
 cloud->width    = 500;
 cloud->height   = 1;
 cloud->is_dense = false;
 cloud->points.resize (cloud->width * cloud->height);
 for (pcl::index_t i = 0; i < static_cast<pcl::index_t>(cloud->size ()); ++i)
 {
     (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
     (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
     if( i % 2 == 0)
       (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
     else
       (*cloud)[i].z = -1 * ((*cloud)[i].x + (*cloud)[i].y);
 }

 std::vector<int> inliers;

 // created RandomSampleConsensus object and compute the appropriated model
 pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
   model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
 pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
   model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

   pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
   ransac.setDistanceThreshold (.01);
   ransac.computeModel();
   ransac.getInliers(inliers);

 // copies all inliers of the model computed to another PointCloud
 pcl::copyPointCloud (*cloud, inliers, *final);

 return 0;
}
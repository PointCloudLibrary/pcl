/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#include <pcl/surface/surfel_smoothing.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>


using namespace pcl;

int
main (int argc, char **argv)
{
  if (argc != 5)
  {
    PCL_ERROR ("./surfel_smoothing_test normal_search_radius surfel_scale source_cloud destination_cloud\n");
    return (-1);
  }
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());

  PCDReader reader;
  reader.read (argv[3], *cloud);
  PCL_INFO ("Cloud read: %s\n", argv[3]);

  float normal_search_radius = static_cast<float> (atof (argv[1]));
  float surfel_scale = static_cast<float> (atof (argv[2]));


  NormalEstimation<PointXYZ, Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud);
  search::KdTree<PointXYZ>::Ptr search_tree (new search::KdTree<PointXYZ>);
  normal_estimation.setSearchMethod (search_tree);
  normal_estimation.setRadiusSearch (normal_search_radius);
  normal_estimation.compute (*normals);

  SurfelSmoothing<PointXYZ, Normal> surfel_smoothing (surfel_scale);
  surfel_smoothing.setInputCloud (cloud);
  surfel_smoothing.setInputNormals (normals);
  surfel_smoothing.setSearchMethod (search_tree);
  PointCloud<PointXYZ>::Ptr output_positions;
  PointCloud<Normal>::Ptr output_normals;
  surfel_smoothing.computeSmoothedCloud (output_positions, output_normals);

  PointCloud<PointNormal>::Ptr output_with_normals (new PointCloud<PointNormal> ());
  pcl::concatenateFields (*output_positions, *normals, *output_with_normals);

  io::savePCDFileASCII (argv[4], *output_with_normals);

  return (0);
}

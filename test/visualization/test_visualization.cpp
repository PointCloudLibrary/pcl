/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/test/gtest.h>

#include <pcl/common/generate.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
search::KdTree<PointXYZ>::Ptr tree;
search::KdTree<PointNormal>::Ptr tree2;

// add by ktran to test update functions
PointCloud<PointXYZ>::Ptr cloud1 (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals1 (new PointCloud<PointNormal>);
search::KdTree<PointXYZ>::Ptr tree3;
search::KdTree<PointNormal>::Ptr tree4;

// Test that updatepointcloud works when adding points.
////////////////////////////////////////////////////////////////////////////////
TEST(PCL, PCLVisualizer_updatePointCloudAddPoint)
{
  pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::UniformGenerator<float> > generator;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  generator.fill(3, 1, *cloud);

  // Setup a basic viewport window
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->addPointCloud<pcl::PointXYZ>(cloud);

  cloud->push_back(pcl::PointXYZ());

  viewer->updatePointCloud(cloud);
  viewer->spinOnce(100);
}

// Test that updatepointcloud works when removing points. Ie. modifying vtk data structure to reflect modified pointcloud
// See #4001 and #3452 for previously undetected error.
////////////////////////////////////////////////////////////////////////////////
TEST(PCL, PCLVisualizer_updatePointCloudRemovePoint)
{
  pcl::common::CloudGenerator<pcl::PointXYZRGB, pcl::common::UniformGenerator<float> > generator;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  generator.fill(3, 1, *cloud);

  // Setup a basic viewport window
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_green(cloud, 0, 225, 100);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_green, "sample cloud");

  // remove points one by one)
  while (!cloud->empty()) {
    cloud->erase(cloud->end() - 1);
    viewer->updatePointCloud(cloud, "sample cloud");
    viewer->spinOnce(100);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PCLVisualizer_camera)
{
  PCLVisualizer visualizer;
  visualizer.initCameraParameters ();

  // First test if the intrinsic+extrinsic to OpenGL conversion works back and forth
  Eigen::Matrix3f given_intrinsics (Eigen::Matrix3f::Identity ());
  given_intrinsics (0, 0) = 525.f;
  given_intrinsics (1, 1) = 525.f;
  given_intrinsics (0, 2) = 320.f;
  given_intrinsics (1, 2) = 240.f;

  float M_PI_f = static_cast<float> (M_PI);
  Eigen::Matrix4f given_extrinsics (Eigen::Matrix4f::Identity ());
  given_extrinsics.block<3, 3> (0, 0) = Eigen::AngleAxisf (30.f * M_PI_f / 180.f, Eigen::Vector3f (1.f, 0.f, 0.f)).matrix ();
  given_extrinsics.block<3, 1> (0, 3) = Eigen::Vector3f (10.f, 15.f, 20.f);

  visualizer.setCameraParameters (given_intrinsics, given_extrinsics);
  Eigen::Matrix4f viewer_pose = visualizer.getViewerPose ().matrix ();

  for (std::size_t i = 0; i < 4; ++i)
    for (std::size_t j = 0; j < 4; ++j)
      EXPECT_NEAR (given_extrinsics (i, j), viewer_pose (i, j), 1e-6);


  // Next, check if setting the OpenGL settings translate well back
  // Look towards the x-axis, which equates to a 90 degree rotation around the y-axis
  Eigen::Vector3f trans (10.f, 2.f, 20.f);
  visualizer.setCameraPosition (trans[0], trans[1], trans[2], trans[0] + 1., trans[1], trans[2], 0., 1., 0.);
  viewer_pose = visualizer.getViewerPose ().matrix ();
  Eigen::Matrix3f expected_rotation = Eigen::AngleAxisf (M_PI_f / 2.0f, Eigen::Vector3f (0.f, 1.f, 0.f)).matrix ();
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      EXPECT_NEAR (viewer_pose (i, j), expected_rotation (i, j), 1e-6);
  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_NEAR (viewer_pose (i, 3), trans[i], 1e-6);


  // Now add the bunny point cloud and reset the camera based on the scene (i.e., VTK will compute a new camera pose
  // so that it includes the whole scene in the window)
  /// TODO stuck here, resetCamera () does not seem to work if there is no window present - can't do that on a Mac
//  visualizer.addPointCloud (cloud1);
//  visualizer.resetCamera ();
//  visualizer.spinOnce ();
//  viewer_pose = visualizer.getViewerPose ().matrix ();

//  std::cerr << "reset camera viewer pose:" << std::endl << viewer_pose << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
TEST (PCL, PCLVisualizer_getPointCloudRenderingProperties)
{
  PCLVisualizer visualizer;

  std::string cloud_id = "input_cloud";
  visualizer.addPointCloud (cloud, cloud_id);
  ASSERT_TRUE (visualizer.setPointCloudRenderingProperties (PCL_VISUALIZER_COLOR,
                                                            1., 0., 0., cloud_id));
  double r, g, b;
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE,
                                                             r, g, b, cloud_id));
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY,
                                                             r, g, b, cloud_id));
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_LINE_WIDTH,
                                                             r, g, b, cloud_id));
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_FONT_SIZE,
                                                             r, g, b, cloud_id));
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_REPRESENTATION,
                                                             r, g, b, cloud_id));
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_IMMEDIATE_RENDERING,
                                                             r, g, b, cloud_id));
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_SHADING,
                                                             r, g, b, cloud_id));
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_LUT,
                                                             r, g, b, cloud_id));
  EXPECT_FALSE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_LUT_RANGE,
                                                             r, g, b, cloud_id));

  r = 666.;
  g = 666.;
  b = 666.;
  EXPECT_TRUE (visualizer.getPointCloudRenderingProperties (PCL_VISUALIZER_COLOR,
                                                            r, g, b, cloud_id));

  EXPECT_EQ (r, 1.);
  EXPECT_EQ (g, 0.);
  EXPECT_EQ (b, 0.);
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load file
  pcl::PCLPointCloud2 cloud_blob;
  loadPCDFile (argv[1], cloud_blob);
  fromPCLPointCloud2 (cloud_blob, *cloud);

  // Create search tree
  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud);

  // Normal estimation
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  n.setInputCloud (cloud);
  //n.setIndices (indices[B);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate XYZ and normal information
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
      
  // Create search tree
  tree2.reset (new search::KdTree<PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Process for update cloud
  if (argc == 3)
  {
    pcl::PCLPointCloud2 cloud_blob1;
    loadPCDFile (argv[2], cloud_blob1);
    fromPCLPointCloud2 (cloud_blob1, *cloud1);
        // Create search tree
    tree3.reset (new search::KdTree<PointXYZ> (false));
    tree3->setInputCloud (cloud1);

    // Normal estimation
    NormalEstimation<PointXYZ, Normal> n1;
    PointCloud<Normal>::Ptr normals1 (new PointCloud<Normal> ());
    n1.setInputCloud (cloud1);

    n1.setSearchMethod (tree3);
    n1.setKSearch (20);
    n1.compute (*normals1);

    // Concatenate XYZ and normal information
    pcl::concatenateFields (*cloud1, *normals1, *cloud_with_normals1);
    // Create search tree
    tree4.reset (new search::KdTree<PointNormal>);
    tree4->setInputCloud (cloud_with_normals1);
  }

  // Testing
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

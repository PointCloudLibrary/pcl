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
 * 
 *
 */

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_th_dd = 0.02f;
int   default_max_search = 50;

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

pcl::visualization::PCLVisualizer viewer ("3D Edge Viewer");

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -th_dd X       = the tolerance in meters for difference in depth values between neighboring points (The value is set for 1 meter and is adapted with respect to depth value linearly. (e.g. 2.0*th_dd in 2 meter depth)) (default: "); 
  print_value ("%f", default_th_dd); print_info (")\n");
  print_info ("                     -max_search X  = the max search distance for deciding occluding and occluded edges (default: "); 
  print_value ("%d", default_max_search); print_info (")\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::io::savePCDFile (filename, output, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true); // Save as binary
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

void 
keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
{
  double opacity;
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
      case '1':
        viewer.getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "nan boundary edges");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-opacity, "nan boundary edges");
        break;
      case '2':
        viewer.getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "occluding edges");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-opacity, "occluding edges");
        break;
      case '3':
        viewer.getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "occluded edges");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-opacity, "occluded edges");
        break;
      case '4':
        viewer.getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "high curvature edges");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-opacity, "high curvature edges");
        break;
      case '5':
        viewer.getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "rgb edges");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-opacity, "rgb edges");
        break;
    }
  }
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         float th_dd, int max_search)
{
  CloudPtr cloud (new Cloud);
  fromPCLPointCloud2 (*input, *cloud);

  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<PointXYZRGBA, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setNormalSmoothingSize (10.0f);
  ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);
  ne.setInputCloud (cloud);
  ne.compute (*normal);

  TicToc tt;
  tt.tic ();

  //OrganizedEdgeBase<PointXYZRGBA, Label> oed;
  //OrganizedEdgeFromRGB<PointXYZRGBA, Label> oed;
  //OrganizedEdgeFromNormals<PointXYZRGBA, Normal, Label> oed;
  OrganizedEdgeFromRGBNormals<PointXYZRGBA, Normal, Label> oed;
  oed.setInputNormals (normal);
  oed.setInputCloud (cloud);
  oed.setDepthDisconThreshold (th_dd);
  oed.setMaxSearchNeighbors (max_search);
  oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
  PointCloud<Label> labels;
  std::vector<PointIndices> label_indices;
  oed.compute (labels, label_indices);
  print_info ("Detecting all edges... [done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

  // Make gray point clouds
  for (size_t idx = 0; idx < cloud->points.size (); idx++)
  {
    uint8_t gray = uint8_t ((cloud->points[idx].r + cloud->points[idx].g + cloud->points[idx].b) / 3);
    cloud->points[idx].r = cloud->points[idx].g = cloud->points[idx].b = gray;
  }

  // Display edges in PCLVisualizer
  viewer.setSize (640, 480);
  viewer.addCoordinateSystem (0.2f, "global");
  viewer.addPointCloud (cloud, "original point cloud");
  viewer.registerKeyboardCallback(&keyboard_callback);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGBA>), 
    occluded_edges (new pcl::PointCloud<pcl::PointXYZRGBA>), 
    nan_boundary_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
    high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
    rgb_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::copyPointCloud (*cloud, label_indices[0].indices, *nan_boundary_edges);
  pcl::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
  pcl::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
  pcl::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
  pcl::copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);

  const int point_size = 2;
  viewer.addPointCloud<pcl::PointXYZRGBA> (nan_boundary_edges, "nan boundary edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "nan boundary edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "nan boundary edges");

  viewer.addPointCloud<pcl::PointXYZRGBA> (occluding_edges, "occluding edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluding edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "occluding edges");

  viewer.addPointCloud<pcl::PointXYZRGBA> (occluded_edges, "occluded edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluded edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "occluded edges");

  viewer.addPointCloud<pcl::PointXYZRGBA> (high_curvature_edges, "high curvature edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "high curvature edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, "high curvature edges");

  viewer.addPointCloud<pcl::PointXYZRGBA> (rgb_edges, "rgb edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "rgb edges");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "rgb edges");

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  // Combine point clouds and edge labels
  pcl::PCLPointCloud2 output_edges;
  toPCLPointCloud2 (labels, output_edges);
  concatenateFields (*input, output_edges, output);
}

int
main (int argc, char** argv)
{
  print_info ("Detect 3D edges from organized point cloud data. For more information, use: %s -h\n", argv[0]);
  bool help = false;
  parse_argument (argc, argv, "-h", help);
  if (argc < 3 || help)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing
  float th_dd = default_th_dd;
  int max_search = default_max_search;

  parse_argument (argc, argv, "-th_dd", th_dd);
  parse_argument (argc, argv, "-max_search", max_search);

  print_info ("th_dd: "); print_value ("%f\n", th_dd); 
  print_info ("max_search: "); print_value ("%d\n", max_search); 

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud)) 
    return (-1);

  // Perform the feature estimation
  pcl::PCLPointCloud2 output;

  compute (cloud, output, th_dd, max_search);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}


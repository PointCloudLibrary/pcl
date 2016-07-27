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

// Stdlib
#include <stdlib.h>
#include <cmath>
#include <limits.h>

#include <boost/format.hpp>


// PCL input/output
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

//PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// The segmentation class this example is for
#include <pcl/segmentation/lccp_segmentation.h>

// VTK
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

/// *****  Type Definitions ***** ///

typedef pcl::PointXYZRGBA PointT;  // The point type used for input
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

/// Callback and variables

bool show_normals = false, normals_changed = false;
bool show_adjacency = false;
bool show_supervoxels = false;
bool show_help = true;
float normals_scale;

/** \brief Callback for setting options in the visualizer via keyboard.
 *  \param[in] event_arg Registered keyboard event  */
void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event_arg,
                       void*)
{
  int key = event_arg.getKeyCode ();

  if (event_arg.keyUp ())
    switch (key)
    {
      case (int) '1':
        show_normals = !show_normals;
        normals_changed = true;
        break;
      case (int) '2':
        show_adjacency = !show_adjacency;
        break;
      case (int) '3':
        show_supervoxels = !show_supervoxels;
        break;
      case (int) '4':
        normals_scale *= 1.25;
        normals_changed = true;
        break;
      case (int) '5':
        normals_scale *= 0.8;
        normals_changed = true;
        break;
      case (int) 'd':
      case (int) 'D':
        show_help = !show_help;
        break;
      default:
        break;
    }
}

/// *****  Prototypes helper functions***** ///

/** \brief Displays info text in the specified PCLVisualizer
 *  \param[in] viewer_arg The PCLVisualizer to modify  */
void
printText (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg);

/** \brief Removes info text in the specified PCLVisualizer
 *  \param[in] viewer_arg The PCLVisualizer to modify  */
void
removeText (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg);


/// ---- main ---- ///
int
main (int argc,
      char ** argv)
{
  if (argc < 2)  /// Print Info
  {
    pcl::console::print_info (
\
        "\n\
-- pcl::LCCPSegmentation example -- :\n\
\n\
Syntax: %s input.pcd  [Options] \n\
\n\
Output:\n\
  -o <outname> \n\
          Write segmented point cloud to disk (Type XYZL). If this option is specified without giving a name, the <outputname> defaults to <inputfilename>_out.pcd.\n\
          The content of the file can be changed with the -add and -bin flags\n\
  -novis  Disable visualization\n\
Output options:\n\
  -add    Instead of XYZL, append a label field to the input point cloud which holds the segmentation results (<input_cloud_type>+L)\n\
          If a label field already exists in the input point cloud it will be overwritten by the segmentation\n\
  -bin    Save a binary pcd-file instead of an ascii file \n\
  -so     Additionally write the colored supervoxel image to <outfilename>_svcloud.pcd\n\
  \n\
Supervoxel Parameters: \n\
  -v <voxel resolution> \n\
  -s <seed resolution> \n\
  -c <color weight> \n\
  -z <spatial weight> \n\
  -n <normal_weight> \n\
  -tvoxel - Use single-camera-transform for voxels (Depth-Dependent-Voxel-Grid)\n\
  -refine - Use supervoxel refinement\n\
  -nonormals - Ignore the normals from the input pcd file\n\
  \n\
LCCPSegmentation Parameters: \n\
  -ct <concavity tolerance angle> - Angle threshold for concave edges to be treated as convex. \n\
  -st <smoothness threshold> - Invalidate steps. Value from the interval [0,1], where 0 is the strictest and 1 equals 'no smoothness check' \n\
  -ec - Use extended (less local) convexity check\n\
  -sc - Use sanity criterion to invalidate singular connected patches\n\
  -smooth <mininmal segment size>  - Merge small segments which have fewer points than minimal segment size\n\
    \n",
        argv[0]);
    return (1);
  }

  /// -----------------------------------|  Preparations  |-----------------------------------

  bool sv_output_specified = pcl::console::find_switch (argc, argv, "-so");
  bool show_visualization = (!pcl::console::find_switch (argc, argv, "-novis"));
  bool ignore_provided_normals = pcl::console::find_switch (argc, argv, "-nonormals");
  bool add_label_field = pcl::console::find_switch (argc, argv, "-add");
  bool save_binary_pcd = pcl::console::find_switch (argc, argv, "-bin");
  
  /// Create variables needed for preparations
  std::string outputname ("");
  pcl::PointCloud<PointT>::Ptr input_cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  bool has_normals = false;
  
  /// Get pcd path from command line
  std::string pcd_filename = argv[1];
  PCL_INFO ("Loading pointcloud\n");
  
  /// check if the provided pcd file contains normals
  pcl::PCLPointCloud2 input_pointcloud2;
  if (pcl::io::loadPCDFile (pcd_filename, input_pointcloud2))
  {
    PCL_ERROR ("ERROR: Could not read input point cloud %s.\n", pcd_filename.c_str ());
    return (3);
  }
  pcl::fromPCLPointCloud2 (input_pointcloud2, *input_cloud_ptr);
  if (!ignore_provided_normals)
  {
    if (pcl::getFieldIndex (input_pointcloud2,"normal_x") >= 0)
    {
      pcl::fromPCLPointCloud2 (input_pointcloud2, *input_normals_ptr);
      has_normals = true;

      //NOTE Supposedly there was a bug in old PCL versions that the orientation was not set correctly when recording clouds. This is just a workaround.
      if (input_normals_ptr->sensor_orientation_.w () == 0)
      {
        input_normals_ptr->sensor_orientation_.w () = 1;
        input_normals_ptr->sensor_orientation_.x () = 0;
        input_normals_ptr->sensor_orientation_.y () = 0;
        input_normals_ptr->sensor_orientation_.z () = 0;
      }
    }
    else
      PCL_WARN ("Could not find normals in pcd file. Normals will be calculated. This only works for single-camera-view pointclouds.\n");
  }
  PCL_INFO ("Done making cloud\n");

  ///  Create outputname if not given
  bool output_specified = pcl::console::find_switch (argc, argv, "-o");
  if (output_specified)
  {
    pcl::console::parse (argc, argv, "-o", outputname);

    // If no filename is given, get output filename from inputname (strip seperators and file extension)
    if (outputname.empty () || (outputname.at (0) == '-'))
    {
      outputname = pcd_filename;
      size_t sep = outputname.find_last_of ('/');
      if (sep != std::string::npos)
        outputname = outputname.substr (sep + 1, outputname.size () - sep - 1);

      size_t dot = outputname.find_last_of ('.');
      if (dot != std::string::npos)
        outputname = outputname.substr (0, dot);
    }
  }

/// -----------------------------------|  Main Computation  |-----------------------------------

  ///  Default values of parameters before parsing
  // Supervoxel Stuff
  float voxel_resolution = 0.0075f;
  float seed_resolution = 0.03f;
  float color_importance = 0.0f;
  float spatial_importance = 1.0f;
  float normal_importance = 4.0f;
  bool use_single_cam_transform = false;
  bool use_supervoxel_refinement = false;

  // LCCPSegmentation Stuff
  float concavity_tolerance_threshold = 10;
  float smoothness_threshold = 0.1;
  uint32_t min_segment_size = 0;
  bool use_extended_convexity = false;
  bool use_sanity_criterion = false;
  
  ///  Parse Arguments needed for computation
  //Supervoxel Stuff
  use_single_cam_transform = pcl::console::find_switch (argc, argv, "-tvoxel");
  use_supervoxel_refinement = pcl::console::find_switch (argc, argv, "-refine");

  pcl::console::parse (argc, argv, "-v", voxel_resolution);
  pcl::console::parse (argc, argv, "-s", seed_resolution);
  pcl::console::parse (argc, argv, "-c", color_importance);
  pcl::console::parse (argc, argv, "-z", spatial_importance);
  pcl::console::parse (argc, argv, "-n", normal_importance);

  normals_scale = seed_resolution / 2.0;
  
  // Segmentation Stuff
  pcl::console::parse (argc, argv, "-ct", concavity_tolerance_threshold);
  pcl::console::parse (argc, argv, "-st", smoothness_threshold);
  use_extended_convexity = pcl::console::find_switch (argc, argv, "-ec");
  unsigned int k_factor = 0;
  if (use_extended_convexity)
    k_factor = 1;
  use_sanity_criterion = pcl::console::find_switch (argc, argv, "-sc");
  pcl::console::parse (argc, argv, "-smooth", min_segment_size);

  /// Preparation of Input: Supervoxel Oversegmentation

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  super.setUseSingleCameraTransform (use_single_cam_transform);
  super.setInputCloud (input_cloud_ptr);
  if (has_normals)
    super.setNormalCloud (input_normals_ptr);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

  PCL_INFO ("Extracting supervoxels\n");
  super.extract (supervoxel_clusters);

  if (use_supervoxel_refinement)
  {
    PCL_INFO ("Refining supervoxels\n");
    super.refineSupervoxels (2, supervoxel_clusters);
  }
  std::stringstream temp;
  temp << "  Nr. Supervoxels: " << supervoxel_clusters.size () << "\n";
  PCL_INFO (temp.str ().c_str ());

  PCL_INFO ("Getting supervoxel adjacency\n");
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);

  /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
  pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);

  /// The Main Step: Perform LCCPSegmentation

  PCL_INFO ("Starting Segmentation\n");
  pcl::LCCPSegmentation<PointT> lccp;
  lccp.setConcavityToleranceThreshold (concavity_tolerance_threshold);
  lccp.setSanityCheck (use_sanity_criterion);
  lccp.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
  lccp.setKFactor (k_factor);
  lccp.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
  lccp.setMinSegmentSize (min_segment_size);
  lccp.segment ();

  PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared ();
  lccp.relabelCloud (*lccp_labeled_cloud);
  SuperVoxelAdjacencyList sv_adjacency_list;
  lccp.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization

  /// Creating Colored Clouds and Output
  if (lccp_labeled_cloud->size () == input_cloud_ptr->size ())
  {
    if (output_specified)
    {
      PCL_INFO ("Saving output\n");
      if (add_label_field)
      {
        if (pcl::getFieldIndex (input_pointcloud2, "label") >= 0)
          PCL_WARN ("Input cloud already has a label field. It will be overwritten by the lccp segmentation output.\n");
        pcl::PCLPointCloud2 output_label_cloud2, output_concat_cloud2;
        pcl::toPCLPointCloud2 (*lccp_labeled_cloud, output_label_cloud2);
        pcl::concatenateFields (input_pointcloud2, output_label_cloud2, output_concat_cloud2);
        pcl::io::savePCDFile (outputname + "_out.pcd", output_concat_cloud2, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), save_binary_pcd);
      }
      else
        pcl::io::savePCDFile (outputname + "_out.pcd", *lccp_labeled_cloud, save_binary_pcd);

      if (sv_output_specified)
      {
        pcl::io::savePCDFile (outputname + "_svcloud.pcd", *sv_centroid_normal_cloud, save_binary_pcd);
      }
    }
  }
  else
  {
    PCL_ERROR ("ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
  }

  /// -----------------------------------|  Visualization  |-----------------------------------

  if (show_visualization)
  {
    /// Calculate visualization of adjacency graph
    // Using lines this would be VERY slow right now, because one actor is created for every line (may be fixed in future versions of PCL)
    // Currently this is a work-around creating a polygon mesh consisting of two triangles for each edge
    using namespace pcl;

    typedef LCCPSegmentation<PointT>::VertexIterator VertexIterator;
    typedef LCCPSegmentation<PointT>::AdjacencyIterator AdjacencyIterator;
    typedef LCCPSegmentation<PointT>::EdgeID EdgeID;

    std::set<EdgeID> edge_drawn;

    const unsigned char convex_color [3] = {255, 255, 255};
    const unsigned char concave_color [3] = {255, 0, 0};
    const unsigned char* color;
    
    //The vertices in the supervoxel adjacency list are the supervoxel centroids
    //This iterates through them, finding the edges
    std::pair<VertexIterator, VertexIterator> vertex_iterator_range;
    vertex_iterator_range = boost::vertices (sv_adjacency_list);

    /// Create a cloud of the voxelcenters and map: VertexID in adjacency graph -> Point index in cloud

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");
    
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    for (VertexIterator itr = vertex_iterator_range.first; itr != vertex_iterator_range.second; ++itr)
    {
      const uint32_t sv_label = sv_adjacency_list[*itr];
      std::pair<AdjacencyIterator, AdjacencyIterator> neighbors = boost::adjacent_vertices (*itr, sv_adjacency_list);

      for (AdjacencyIterator itr_neighbor = neighbors.first; itr_neighbor != neighbors.second; ++itr_neighbor)
      {
        EdgeID connecting_edge = boost::edge (*itr, *itr_neighbor, sv_adjacency_list).first;  //Get the edge connecting these supervoxels
        if (sv_adjacency_list[connecting_edge].is_convex)
          color = convex_color;
        else
          color = concave_color;
        
        // two times since we add also two points per edge
        colors->InsertNextTupleValue (color);
        colors->InsertNextTupleValue (color);
        
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (sv_label);
        pcl::PointXYZRGBA vert_curr = supervoxel->centroid_;
        
        
        const uint32_t sv_neighbor_label = sv_adjacency_list[*itr_neighbor];
        pcl::Supervoxel<PointT>::Ptr supervoxel_neigh = supervoxel_clusters.at (sv_neighbor_label);
        pcl::PointXYZRGBA vert_neigh = supervoxel_neigh->centroid_;
        
        points->InsertNextPoint (vert_curr.data);
        points->InsertNextPoint (vert_neigh.data);
          
        // Add the points to the dataset
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
        polyLine->GetPointIds ()->SetNumberOfIds (2);
        polyLine->GetPointIds ()->SetId (0, points->GetNumberOfPoints ()-2);
        polyLine->GetPointIds ()->SetId (1, points->GetNumberOfPoints ()-1);
        cells->InsertNextCell (polyLine);
      }
    }
    polyData->SetPoints (points);
    // Add the lines to the dataset
    polyData->SetLines (cells);
    
    polyData->GetPointData ()->SetScalars (colors);
        
    /// END: Calculate visualization of adjacency graph

    /// Configure Visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->registerKeyboardCallback (keyboardEventOccurred, 0);
    viewer->addPointCloud (lccp_labeled_cloud, "maincloud");

    /// Visualization Loop
    PCL_INFO ("Loading viewer\n");
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);

      /// Show Segmentation or Supervoxels
      viewer->updatePointCloud ( (show_supervoxels) ? sv_labeled_cloud : lccp_labeled_cloud, "maincloud");

      /// Show Normals
      if (normals_changed)
      {
        viewer->removePointCloud ("normals");
        if (show_normals)
        {
          viewer->addPointCloudNormals<pcl::PointNormal> (sv_centroid_normal_cloud, 1, normals_scale, "normals");
          normals_changed = false;
        }
      }
      /// Show Adjacency
      if (show_adjacency)
      {
        viewer->removeShape ("adjacency_graph");
        viewer->addModelFromPolyData (polyData, "adjacency_graph");
      }
      else
      {
        viewer->removeShape ("adjacency_graph");
      }

      if (show_help)
      {
        viewer->removeShape ("help_text");
        printText (viewer);
      }
      else
      {
        removeText (viewer);
        if (!viewer->updateText ("Press d to show help", 5, 10, 12, 1.0, 1.0, 1.0, "help_text"))
          viewer->addText ("Press d to show help", 5, 10, 12, 1.0, 1.0, 1.0, "help_text");
      }

      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }  /// END if (show_visualization)

  return (0);

}  /// END main

/// -------------------------| Definitions of helper functions|-------------------------

void
printText (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg)
{
  std::string on_str = "ON";
  std::string off_str = "OFF";
  if (!viewer_arg->updateText ("Press (1-n) to show different elements (d) to disable this", 5, 72, 12, 1.0, 1.0, 1.0, "hud_text"))
        viewer_arg->addText ("Press (1-n) to show different elements", 5, 72, 12, 1.0, 1.0, 1.0, "hud_text");

  std::string temp = "(1) Supervoxel Normals, currently " + ( (show_normals) ? on_str : off_str);
  if (!viewer_arg->updateText (temp, 5, 60, 10, 1.0, 1.0, 1.0, "normals_text"))
        viewer_arg->addText (temp, 5, 60, 10, 1.0, 1.0, 1.0, "normals_text");

  temp = "(2) Adjacency Graph, currently " + ( (show_adjacency) ? on_str : off_str) + "\n      White: convex; Red: concave";
  if (!viewer_arg->updateText (temp, 5, 38, 10, 1.0, 1.0, 1.0, "graph_text"))
        viewer_arg->addText (temp, 5, 38, 10, 1.0, 1.0, 1.0, "graph_text");

  temp = "(3) Press to show " + ( (show_supervoxels) ? std::string ("SEGMENTATION") : std::string ("SUPERVOXELS"));
  if (!viewer_arg->updateText (temp, 5, 26, 10, 1.0, 1.0, 1.0, "supervoxel_text"))
        viewer_arg->addText (temp, 5, 26, 10, 1.0, 1.0, 1.0, "supervoxel_text");
  
  temp = "(4/5) Press to increase/decrease normals scale, currently " + boost::str (boost::format ("%.3f") % normals_scale);
  if (!viewer_arg->updateText (temp, 5, 14, 10, 1.0, 1.0, 1.0, "normals_scale_text"))
        viewer_arg->addText (temp, 5, 14, 10, 1.0, 1.0, 1.0, "normals_scale_text");
}

void
removeText (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_arg)
{
  viewer_arg->removeShape ("hud_text");
  viewer_arg->removeShape ("normals_text");
  viewer_arg->removeShape ("graph_text");
  viewer_arg->removeShape ("supervoxel_text");
  viewer_arg->removeShape ("normals_scale_text");
}

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

#include <thread>

#include <boost/format.hpp>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/cpc_segmentation.h>

#include <vtkPolyLine.h>

using namespace std::chrono_literals;

/// *****  Type Definitions ***** ///

using PointT = pcl::PointXYZRGBA;  // The point type used for input
using SuperVoxelAdjacencyList = pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList;

/// Callback and variables

bool show_normals = false, normals_changed = false;
bool show_adjacency = false, line_changed = false;
bool show_supervoxels = false;
bool show_segmentation = true;
bool show_help = true;
bool bg_white = false;
float normals_scale;
float line_width = 2.0f;
float textcolor;

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
        show_segmentation = !show_segmentation;
        break;
      case (int) '5':
        normals_scale *= 1.25;
        normals_changed = true;
        break;
      case (int) '6':
        normals_scale *= 0.8;
        normals_changed = true;
        break;
      case (int) '7':
        line_width += 0.5;
        line_changed = true;
        break;
      case (int) '8':
        if (line_width <= 1)
          break;
        line_width -= 0.5;
        line_changed = true;
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
printText (pcl::visualization::PCLVisualizer::Ptr viewer_arg);

/** \brief Removes info text in the specified PCLVisualizer
 *  \param[in] viewer_arg The PCLVisualizer to modify  */
void
removeText (pcl::visualization::PCLVisualizer::Ptr viewer_arg);

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
-- pcl::CPCSegmentation example -- :\n\
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
  -white  Use white background instead of black \n\
  \n\
Supervoxel Parameters: \n\
  -v <voxel resolution> (default 0.0075) \n\
  -s <seed resolution> (default 0.03)\n\
  -c <color weight> (default 0)\n\
  -z <spatial weight> (default 1)\n\
  -n <normal_weight> (default 4)\n\
  -tvoxel - Use single-camera-transform for voxels (Depth-Dependent-Voxel-Grid)\n\
  -refine - Use supervoxel refinement\n\
  -nonormals - Ignore the normals from the input pcd file\n\
  \n\
LCCPSegmentation Parameters: \n\
  -ct <concavity tolerance angle> - Angle threshold in degrees for concave edges to be treated as convex (default 10) \n\
  -st <smoothness threshold> - Invalidate steps. Value from the interval [0,1], where 0 is the strictest and 1 equals 'no smoothness check' (default 0.1)\n\
  -ec - Use extended (less local) convexity check\n\
  -sc - Use sanity criterion to invalidate singular connected patches\n\
  -smooth <mininmal segment size>  - Merge small segments which have fewer points than minimal segment size (default 0)\n\
  \n\
CPCSegmentation Parameters: \n\
  -cut <max_cuts>,<cutting_min_segments>,<min_cut_score> - Plane cutting parameters for splitting of segments\n\
       <max_cuts> - Perform cuts up to this recursion level. Cuts are performed in each segment separately (default 25)\n\
       <cutting_min_segments> - Minimum number of supervoxels in the segment to perform cutting (default 400).\n\
       <min_cut_score> - Minimum score a proposed cut needs to have for being cut (default 0.16)\n\
  -clocal - Use locally constrained cuts (recommended flag)\n\
  -cdir - Use directed weigths (recommended flag) \n\
  -cclean - Use clean cuts. \n\
            Flag set: Only split edges with supervoxels on opposite sites of the cutting-plane \n\
            Flag not set: Split all edges whose centroid is within the seed resolution distance to the cutting-plane\n\
  -citer <num_interations> - Sets the maximum number of iterations for the RANSAC algorithm (default 10000) \n\
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
  std::string outputname;
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
    if (pcl::getFieldIndex (input_pointcloud2, "normal_x") >= 0)
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

    // If no filename is given, get output filename from inputname (strip separators and file extension)
    if (outputname.empty () || (outputname.at (0) == '-'))
    {
      outputname = pcd_filename;
      std::size_t sep = outputname.find_last_of ('/');
      if (sep != std::string::npos)
        outputname = outputname.substr (sep + 1, outputname.size () - sep - 1);

      std::size_t dot = outputname.find_last_of ('.');
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
  bool use_single_cam_transform ;
  bool use_supervoxel_refinement;

  // LCCPSegmentation Stuff
  float concavity_tolerance_threshold = 10;
  float smoothness_threshold = 0.1;
  std::uint32_t min_segment_size = 0;
  bool use_extended_convexity;
  bool use_sanity_criterion;

  // CPCSegmentation Stuff
  float min_cut_score = 0.16;
  unsigned int max_cuts = 25;
  unsigned int cutting_min_segments = 400;
  bool use_local_constrain;
  bool use_directed_cutting;
  bool use_clean_cutting;
  unsigned int ransac_iterations = 10000;

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

  if (pcl::console::find_switch (argc, argv, "-cut"))
  {
    std::vector<float> a;
    pcl::console::parse_x_arguments (argc, argv, "-cut", a);
    max_cuts = a[0];
    if (a.size () > 1)
    {
      cutting_min_segments = a[1];
      if (a.size () > 2)
      {
        min_cut_score = a[2];
      }
    }
  }
  else
  {
    PCL_WARN ("Warning you did not specify the cut argument. No cutting is being done (Fallback to LCCP preprocessing). \nUsage:\n-cut <max_cuts>,<cutting_min_segments>,<min_cut_score> optional: cdir, clocal, citer, cclean\n");
    max_cuts = 0;
  }

  use_local_constrain = pcl::console::find_switch (argc, argv, "-clocal");
  use_directed_cutting = pcl::console::find_switch (argc, argv, "-cdir");
  use_clean_cutting = pcl::console::find_switch (argc, argv, "-cclean");
  pcl::console::parse (argc, argv, "-citer",ransac_iterations);
  
  bg_white = pcl::console::find_switch (argc, argv, "-white");
  textcolor = bg_white?0:1;

  pcl::console::print_info ("Maximum cuts: %d\n", max_cuts);
  pcl::console::print_info ("Minimum segment size: %d\n", cutting_min_segments);
  pcl::console::print_info ("Use local constrain: %d\n", use_local_constrain);
  pcl::console::print_info ("Use directed weights: %d\n", use_directed_cutting);
  pcl::console::print_info ("Use clean cuts: %d\n", use_clean_cutting);
  pcl::console::print_info ("RANSAC iterations: %d\n", ransac_iterations);  

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
  std::map<std::uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

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
  std::multimap<std::uint32_t, std::uint32_t>supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);

  /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
  pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);

  /// Set parameters for LCCP preprocessing and CPC (CPC inherits from LCCP, thus it includes LCCP's functionality)

  PCL_INFO ("Starting Segmentation\n");
  pcl::CPCSegmentation<PointT> cpc;
  cpc.setConcavityToleranceThreshold (concavity_tolerance_threshold);
  cpc.setSanityCheck (use_sanity_criterion);
  cpc.setCutting (max_cuts, cutting_min_segments, min_cut_score, use_local_constrain, use_directed_cutting, use_clean_cutting);
  cpc.setRANSACIterations (ransac_iterations);
  cpc.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
  cpc.setKFactor (k_factor);
  cpc.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
  cpc.setMinSegmentSize (min_segment_size);
  cpc.segment ();
  
  PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
  pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud = sv_labeled_cloud->makeShared ();
  cpc.relabelCloud (*cpc_labeled_cloud);
  SuperVoxelAdjacencyList sv_adjacency_list;
  cpc.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization

  /// Creating Colored Clouds and Output
  if (cpc_labeled_cloud->size () == input_cloud_ptr->size ())
  {
    if (output_specified)
    {
      PCL_INFO ("Saving output\n");
      if (add_label_field)
      {
        if (pcl::getFieldIndex (input_pointcloud2, "label") >= 0)
          PCL_WARN ("Input cloud already has a label field. It will be overwritten by the cpc segmentation output.\n");
        pcl::PCLPointCloud2 output_label_cloud2, output_concat_cloud2;
        pcl::toPCLPointCloud2 (*cpc_labeled_cloud, output_label_cloud2);
        pcl::concatenateFields (input_pointcloud2, output_label_cloud2, output_concat_cloud2);
        pcl::io::savePCDFile (outputname + "_out.pcd", output_concat_cloud2, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), save_binary_pcd);
      }
      else
        pcl::io::savePCDFile (outputname + "_out.pcd", *cpc_labeled_cloud, save_binary_pcd);

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

    using VertexIterator = LCCPSegmentation<PointT>::VertexIterator;
    using AdjacencyIterator = LCCPSegmentation<PointT>::AdjacencyIterator;
    using EdgeID = LCCPSegmentation<PointT>::EdgeID;

    const unsigned char black_color   [3] = {0, 0, 0};
    const unsigned char white_color   [3] = {255, 255, 255};
    const unsigned char concave_color [3] = {255,  0,  0};
    const unsigned char cut_color     [3] = {  0,255,  0};
    const unsigned char* convex_color     = bg_white ? black_color : white_color;
    const unsigned char* color = nullptr;

    //The vertices in the supervoxel adjacency list are the supervoxel centroids
    //This iterates through them, finding the edges
    std::pair<VertexIterator, VertexIterator> vertex_iterator_range;
    vertex_iterator_range = boost::vertices (sv_adjacency_list);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");

    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    for (auto itr = vertex_iterator_range.first; itr != vertex_iterator_range.second; ++itr)
    {
      const std::uint32_t sv_label = sv_adjacency_list[*itr];
      std::pair<AdjacencyIterator, AdjacencyIterator> neighbors = boost::adjacent_vertices (*itr, sv_adjacency_list);

      for (AdjacencyIterator itr_neighbor = neighbors.first; itr_neighbor != neighbors.second; ++itr_neighbor)
      {
        EdgeID connecting_edge = boost::edge (*itr, *itr_neighbor, sv_adjacency_list).first;  //Get the edge connecting these supervoxels
        bool is_convex = sv_adjacency_list[connecting_edge].is_convex;
        bool is_valid = sv_adjacency_list[connecting_edge].is_valid;
        if (is_convex && is_valid)
          color = convex_color;
        else if (is_convex && !is_valid)
          color = cut_color;
        else if (!is_convex && !is_valid)
          color = concave_color;

        // two times since we add also two points per edge
#if (VTK_MAJOR_VERSION < 7) || (VTK_MAJOR_VERSION == 7 && VTK_MINOR_VERSION == 0)
        colors->InsertNextTupleValue (color);
        colors->InsertNextTupleValue (color);
#else       
        colors->InsertNextTypedTuple (color);
        colors->InsertNextTypedTuple (color);
#endif      
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (sv_label);
        pcl::PointXYZRGBA vert_curr = supervoxel->centroid_;

        const std::uint32_t sv_neighbor_label = sv_adjacency_list[*itr_neighbor];
        pcl::Supervoxel<PointT>::Ptr supervoxel_neigh = supervoxel_clusters.at (sv_neighbor_label);
        pcl::PointXYZRGBA vert_neigh = supervoxel_neigh->centroid_;

        points->InsertNextPoint (vert_curr.data);
        points->InsertNextPoint (vert_neigh.data);

        // Add the points to the dataset
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
        polyLine->GetPointIds ()->SetNumberOfIds (2);
        polyLine->GetPointIds ()->SetId (0, points->GetNumberOfPoints () - 2);
        polyLine->GetPointIds ()->SetId (1, points->GetNumberOfPoints () - 1);
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
    float bg_color = bg_white?1:0;
    viewer->setBackgroundColor (bg_color, bg_color, bg_color);
    viewer->registerKeyboardCallback (keyboardEventOccurred, nullptr);
    viewer->addPointCloud (cpc_labeled_cloud, "cpc_cloud");
    /// Visualization Loop
    PCL_INFO ("Loading viewer\n");
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      /// Show Segmentation or Supervoxels      
      if (show_segmentation)
      {
        if (!viewer->contains ("cpc_cloud"))
          viewer->addPointCloud (cpc_labeled_cloud, "cpc_cloud");
      }
      else
        viewer->removePointCloud ("cpc_cloud");
        
      if (show_supervoxels)
      {
        if (!viewer->contains ("sv_cloud"))
          viewer->addPointCloud (sv_labeled_cloud, "sv_cloud");
      }
      else
        viewer->removePointCloud ("sv_cloud");

      /// Show Normals
      if (normals_changed)
      {
        viewer->removePointCloud ("normals");
        normals_changed = false;
      }
      if (show_normals)
      {
        if (!viewer->contains ("normals"))
          viewer->addPointCloudNormals<pcl::PointNormal> (sv_centroid_normal_cloud, 1, normals_scale, "normals");
      }
      else
        viewer->removePointCloud ("normals");
      /// Show Adjacency
      if (line_changed)
      {
        viewer->removeShape ("adjacency_graph");
        line_changed = false;
      }
      if (show_adjacency)
      {
        if (!viewer->contains ("adjacency_graph"))
        {
          viewer->addModelFromPolyData (polyData, "adjacency_graph", 0);
          viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, "adjacency_graph");
        }
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
        if (!viewer->updateText ("Press d to show help", 5, 10, 12, textcolor, textcolor, textcolor, "help_text"))
          viewer->addText ("Press d to show help", 5, 10, 12, textcolor, textcolor, textcolor, "help_text");
      }

      std::this_thread::sleep_for(100ms);
    }
  }  /// END if (show_visualization)

  return (0);

}  /// END main

/// -------------------------| Definitions of helper functions|-------------------------

void
printText (pcl::visualization::PCLVisualizer::Ptr viewer_arg)
{
  std::string on_str = "ON";
  std::string off_str = "OFF";  
  int top = 100;
  if (!viewer_arg->updateText ("Press (1-n) to show different elements (d) to disable this", 5, top, 12, textcolor, textcolor, textcolor, "hud_text"))
    viewer_arg->addText ("Press (1-n) to show different elements", 5, top, 12, textcolor, textcolor, textcolor, "hud_text");
  
  top -= 12;
  std::string temp = "(1) Supervoxel Normals, currently " + ( (show_normals) ? on_str : off_str);
  if (!viewer_arg->updateText (temp, 5, top, 10, textcolor, textcolor, textcolor, "normals_text"))
    viewer_arg->addText (temp, 5, top, 10, textcolor, textcolor, textcolor, "normals_text");
  
  top -= 24;
  temp = "(2) Adjacency Graph, currently " + ( (show_adjacency) ? on_str : off_str) + "\n      White: convex; Red: concave, Green: cut";
  if (!viewer_arg->updateText (temp, 5, top, 10, textcolor, textcolor, textcolor, "graph_text"))
    viewer_arg->addText (temp, 5, top, 10, textcolor, textcolor, textcolor, "graph_text");
  
  top -= 12;
  temp = "(3) Press to show SEGMENTATION";
  if (!viewer_arg->updateText (temp, 5, top, 10, textcolor, textcolor, textcolor, "seg_text"))
    viewer_arg->addText (temp, 5, top, 10, textcolor, textcolor, textcolor, "seg_text");
  
  top -= 12;
  temp = "(4) Press to show SUPERVOXELS";
  if (!viewer_arg->updateText (temp, 5, top, 10, textcolor, textcolor, textcolor, "supervoxel_text"))
    viewer_arg->addText (temp, 5, top, 10, textcolor, textcolor, textcolor, "supervoxel_text");

  top -= 12;
  temp = "(5/6) Press to increase/decrease normals scale, currently " + boost::str (boost::format ("%.3f") % normals_scale);
  if (!viewer_arg->updateText (temp, 5, top, 10, textcolor, textcolor, textcolor, "normals_scale_text"))
    viewer_arg->addText (temp, 5, top, 10, textcolor, textcolor, textcolor, "normals_scale_text");
  top -= 12;
  temp = "(7/8) Press to increase/decrease line width, currently " + boost::str (boost::format ("%.3f") % line_width);
  if (!viewer_arg->updateText (temp, 5, top, 10, textcolor, textcolor, textcolor, "line_width_text"))
    viewer_arg->addText (temp, 5, top, 10, textcolor, textcolor, textcolor, "line_width_text");
}

void
removeText (pcl::visualization::PCLVisualizer::Ptr viewer_arg)
{
  viewer_arg->removeShape ("hud_text");
  viewer_arg->removeShape ("normals_text");
  viewer_arg->removeShape ("line_width_text");
  viewer_arg->removeShape ("graph_text");
  viewer_arg->removeShape ("supervoxel_text");
  viewer_arg->removeShape ("seg_text");
  viewer_arg->removeShape ("normals_scale_text");
}

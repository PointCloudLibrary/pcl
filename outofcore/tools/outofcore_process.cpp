/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id: outofcore_process.cpp 6927 2012-08-23 02:34:54Z stfox88 $
 * \author Justin Rosen
 * \author Stephen Fox
 */

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

// todo: Read clouds as PCLPointCloud2 so we don't need to define PointT explicitly.
//       This also requires our octree to take PCLPointCloud2 as an input.
typedef pcl::PointXYZ PointT;

using namespace pcl;
using namespace pcl::outofcore;

using pcl::console::parse_argument;
using pcl::console::parse_file_extension_argument;
using pcl::console::find_switch;
using pcl::console::print_error;
using pcl::console::print_warn;
using pcl::console::print_info;

#include <boost/foreach.hpp>

typedef OutofcoreOctreeBase<> octree_disk;

const int OCTREE_DEPTH (0);
const int OCTREE_RESOLUTION (1);

PCLPointCloud2::Ptr
getCloudFromFile (boost::filesystem::path pcd_path)
{
  print_info ("Reading: %s ", pcd_path.c_str ());


  // Read PCD file
  PCLPointCloud2::Ptr cloud(new PCLPointCloud2);

  if (io::loadPCDFile (pcd_path.string (), *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file\n");
    exit (-1);
  }

  print_info ("(%d)\n", (cloud->width * cloud->height));

  return cloud;
}

int
outofcoreProcess (std::vector<boost::filesystem::path> pcd_paths, boost::filesystem::path root_dir, 
                  int depth, double resolution, int build_octree_with, bool gen_lod, bool overwrite, bool multiresolution)
{
  // Bounding box min/max pts
  PointT min_pt, max_pt;

  // Iterate over all pcd files resizing min/max
  for (size_t i = 0; i < pcd_paths.size (); i++)
  {

    // Get cloud
    PCLPointCloud2::Ptr cloud = getCloudFromFile (pcd_paths[i]);
    PointCloud<PointXYZ>::Ptr cloudXYZ (new PointCloud<PointXYZ>);

    fromPCLPointCloud2 (*cloud, *cloudXYZ);

    PointT tmp_min_pt, tmp_max_pt;

    if (i == 0)
    {
      // Get initial min/max
      getMinMax3D (*cloudXYZ, min_pt, max_pt);
    }
    else
    {
      getMinMax3D (*cloudXYZ, tmp_min_pt, tmp_max_pt);

      // Resize new min
      if (tmp_min_pt.x < min_pt.x)
        min_pt.x = tmp_min_pt.x;
      if (tmp_min_pt.y < min_pt.y)
        min_pt.y = tmp_min_pt.y;
      if (tmp_min_pt.z < min_pt.z)
        min_pt.z = tmp_min_pt.z;

      // Resize new max
      if (tmp_max_pt.x > max_pt.x)
        max_pt.x = tmp_max_pt.x;
      if (tmp_max_pt.y > max_pt.y)
        max_pt.y = tmp_max_pt.y;
      if (tmp_max_pt.z > max_pt.z)
        max_pt.z = tmp_max_pt.z;
    }
  }

  std::cout << "Bounds: " << min_pt << " - " << max_pt << std::endl;

  // The bounding box of the root node of the out-of-core octree must be specified
  const Eigen::Vector3d bounding_box_min (min_pt.x, min_pt.y, min_pt.z);
  const Eigen::Vector3d bounding_box_max (max_pt.x, max_pt.y, max_pt.z);

  //specify the directory and the root node's meta data file with a
  //".oct_idx" extension (currently it must be this extension)
  boost::filesystem::path octree_path_on_disk (root_dir / "tree.oct_idx");

  print_info ("Writing: %s\n", octree_path_on_disk.c_str ());
  //make sure there isn't an octree there already
  if (boost::filesystem::exists (octree_path_on_disk))
  {
    if (overwrite)
    {
      boost::filesystem::remove_all (root_dir);
    }
    else
    {
      PCL_ERROR ("There's already an octree in the default location. Check the tree directory\n");
      return (-1);
    }
  }

  octree_disk *outofcore_octree;

  //create the out-of-core data structure (typedef'd above)
  if (build_octree_with == OCTREE_DEPTH)
  {
    outofcore_octree = new octree_disk (depth, bounding_box_min, bounding_box_max, octree_path_on_disk, "ECEF");
  }
  else
  {
    outofcore_octree = new octree_disk (bounding_box_min, bounding_box_max, resolution, octree_path_on_disk, "ECEF");
  }

  uint64_t total_pts = 0;

  // Iterate over all pcd files adding points to the octree
  for (size_t i = 0; i < pcd_paths.size (); i++)
  {

    PCLPointCloud2::Ptr cloud = getCloudFromFile (pcd_paths[i]);

    boost::uint64_t pts = 0;
    
    if (gen_lod && !multiresolution)
    {
      print_info ("  Generating LODs\n");
      pts = outofcore_octree->addPointCloud_and_genLOD (cloud);
    }
    else
    {
      pts = outofcore_octree->addPointCloud (cloud, false);
    }
    
    print_info ("Successfully added %lu points\n", pts);
    print_info ("%lu Points were dropped (probably NaN)\n", cloud->width*cloud->height - pts);
    
//    assert ( pts == cloud->width * cloud->height );
    
    total_pts += pts;
  }

  print_info ("Added a total of %lu from %d clouds\n",total_pts, pcd_paths.size ());
  

  double x, y;
  outofcore_octree->getBinDimension (x, y);

  print_info ("  Depth: %i\n", outofcore_octree->getDepth ());
  print_info ("  Resolution: [%f, %f]\n", x, y);

  if(multiresolution)
  {
    print_info ("Generating LOD...\n");
    outofcore_octree->setSamplePercent (0.25);
    outofcore_octree->buildLOD ();
  }

  //free outofcore data structure; the destructor forces buffer flush to disk
  delete outofcore_octree;

  return 0;
}

void
printHelp (int, char **argv)
{
  print_info ("This program is used to process pcd fiels into an outofcore data structure viewable by the");
  print_info ("pcl_outofcore_viewer\n\n");
  print_info ("%s <options> <input>.pcd <output_tree_dir>\n", argv[0]);
  print_info ("\n");
  print_info ("Options:\n");
  print_info ("\t -depth <resolution>           \t Octree depth\n");
  print_info ("\t -resolution <resolution>      \t Octree resolution\n");
  print_info ("\t -gen_lod                      \t Generate octree LODs\n");
  print_info ("\t -overwrite                    \t Overwrite existing octree\n");
  print_info ("\t -multiresolution              \t Generate multiresolutoin LOD\n");
  print_info ("\t -h                            \t Display help\n");
  print_info ("\n");
}

int
main (int argc, char* argv[])
{
  // Check for help (-h) flag
  if (argc > 1)
  {
    if (find_switch (argc, argv, "-h"))
    {
      printHelp (argc, argv);
      return (-1);
    }
  }

  // If no arguments specified
  if (argc - 1 < 1)
  {
    printHelp (argc, argv);
    return (-1);
  }

  if (find_switch (argc, argv, "-debug"))
  {
    pcl::console::setVerbosityLevel ( pcl::console::L_DEBUG );
  }
  
  // Defaults
  int depth = 4;
  double resolution = .1;
  bool gen_lod = false;
  bool multiresolution = false;
  bool overwrite = false;
  int build_octree_with = OCTREE_DEPTH;

  // If both depth and resolution specified
  if (find_switch (argc, argv, "-depth") && find_switch (argc, argv, "-resolution"))
  {
    PCL_ERROR ("Both -depth and -resolution specified, please specify one (Mutually exclusive)\n");
  }
  // Just resolution specified (Update how we build the tree)
  else if (find_switch (argc, argv, "-resolution"))
  {
    build_octree_with = OCTREE_RESOLUTION;
  }

  // Parse options
  parse_argument (argc, argv, "-depth", depth);
  parse_argument (argc, argv, "-resolution", resolution);
  gen_lod = find_switch (argc, argv, "-gen_lod");
  overwrite = find_switch (argc, argv, "-overwrite");

  if (gen_lod && find_switch (argc, argv, "-multiresolution"))
  {
    multiresolution = true;
  }
  

  // Parse non-option arguments for pcd files
  std::vector<int> file_arg_indices = parse_file_extension_argument (argc, argv, ".pcd");

  std::vector<boost::filesystem::path> pcd_paths;
  for (size_t i = 0; i < file_arg_indices.size (); i++)
  {
    boost::filesystem::path pcd_path (argv[file_arg_indices[i]]);
    if (!boost::filesystem::exists (pcd_path))
    {
      PCL_WARN ("File %s doesn't exist", pcd_path.string ().c_str ());
      continue;
    }
    pcd_paths.push_back (pcd_path);

  }

  // Check if we should process any files
  if (pcd_paths.size () < 1)
  {
    PCL_ERROR ("No .pcd files specified\n");
    return -1;
  }

  // Get root directory
  boost::filesystem::path root_dir (argv[argc - 1]);

  // Check if a root directory was specified, use directory of pcd file
  if (root_dir.extension () == ".pcd")
    root_dir = root_dir.parent_path () / (root_dir.stem().string() + "_tree").c_str();

  return outofcoreProcess (pcd_paths, root_dir, depth, resolution, build_octree_with, gen_lod, overwrite, multiresolution);
}

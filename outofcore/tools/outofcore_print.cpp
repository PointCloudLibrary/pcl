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
 * $Id: outofcore_process.cpp 7463 2012-10-05 05:03:04Z stfox88 $
 * \author Justin Rosen
 * \author Stephen Fox
 */

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

// todo: Read clouds as PointCloud2 so we don't need to define PointT explicitly.
//       This also requires our octree to take PointCloud2 as an input.
typedef pcl::PointXYZ PointT;

using namespace pcl;
using namespace pcl::outofcore;
using namespace sensor_msgs;

using pcl::console::parse_argument;
using pcl::console::parse_file_extension_argument;
using pcl::console::find_switch;
using pcl::console::print_error;
using pcl::console::print_warn;
using pcl::console::print_info;
using pcl::console::print;

#include <boost/foreach.hpp>

typedef OutofcoreOctreeBase<> OctreeDisk;
typedef OutofcoreOctreeBaseNode<> OctreeDiskNode;
typedef OutofcoreBreadthFirstIterator<> OctreeBreadthFirstIterator;
typedef OutofcoreDepthFirstIterator<> OctreeDepthFirstIterator;

typedef Eigen::aligned_allocator<PointT> AlignedPointT;

void
printDepth(int depth)
{
  for (int i=0; i < depth; i++)
    PCL_INFO ("  ");
}

void
printNode(OctreeDiskNode *node)
{
  //
}

int
outofcorePrint (boost::filesystem::path tree_root, int print_depth, bool bounding_box=false, bool breadth_first=false)
{
  std::cout << boost::filesystem::absolute (tree_root) << std::endl;

  OctreeDisk* octree;
  octree = new OctreeDisk (tree_root, true);

  Eigen::Vector3d min, max;
  octree->getBoundingBox(min, max);

  // Cloud bounding box
  PCL_INFO (" Bounding Box: <%lf, %lf, %lf> - <%lf, %lf, %lf>\n", min[0], min[1], min[2], max[0], max[1], max[2]);

  // Cloud depth
  int depth = octree->getTreeDepth ();
  PCL_INFO (" Depth: %d\n", depth);
  if (print_depth > depth)
    print_depth = depth;

  // Cloud point counts at each level
  std::vector<boost::uint64_t> lodPoints = octree->getNumPointsVector ();
  PCL_INFO (" Points:\n");
  for (boost::uint64_t i = 0; i < lodPoints.size (); i++)
    PCL_INFO ("   %d: %d\n", i, lodPoints[i]);

  // Cloud voxel side length
  PCL_INFO(" Voxel Side Length: %d\n", octree->getVoxelSideLength ());

  // Cloud voxel count
  std::vector<PointT, AlignedPointT> voxel_centers;
  octree->getOccupiedVoxelCenters(voxel_centers);
  PCL_INFO(" Voxel Count: %d\n", voxel_centers.size ());

  if (!breadth_first)
  {
    OctreeDisk::DepthFirstIterator depth_first_it (*octree);

    while ( *depth_first_it !=0 )
    {
      OctreeDiskNode *node = *depth_first_it;
      int node_depth = node->getDepth();

      printDepth(node_depth);
      std::string metadata_relative_file = node->getMetadataFilename ().string ();
      boost::replace_first(metadata_relative_file, tree_root.parent_path ().string (), "");
      PCL_INFO ("..%s\n", metadata_relative_file.c_str());

      printDepth(node_depth);
      std::string pcd_relative_file = node->getPCDFilename ().string ();
      boost::replace_first(pcd_relative_file, tree_root.parent_path ().string (), "");
      PCL_INFO ("  PCD: ..%s\n", pcd_relative_file.c_str());

      if (bounding_box)
      {
        Eigen::Vector3d min, max;
        node->getBoundingBox(min, max);

        printDepth(node_depth);
        PCL_INFO ("  Bounding Box: <%lf, %lf, %lf> - <%lf, %lf, %lf>\n", min[0], min[1], min[2], max[0], max[1], max[2]);
      }

      depth_first_it++;
    }
  }
  else
  {
    OctreeDisk::BreadthFirstIterator breadth_first_it (*octree);
    breadth_first_it.setMaxDepth(print_depth);
    while ( *breadth_first_it !=0 )
    {
      OctreeDiskNode *node = *breadth_first_it;
      int node_depth = node->getDepth();

      printDepth(node_depth);
      std::string metadata_relative_file = node->getMetadataFilename ().string ();
      boost::replace_first(metadata_relative_file, tree_root.parent_path ().string (), "");
      PCL_INFO ("..%s\n", metadata_relative_file.c_str());

      printDepth(node_depth);
      std::string pcd_relative_file = node->getPCDFilename ().string ();
      boost::replace_first(pcd_relative_file, tree_root.parent_path ().string (), "");
      PCL_INFO ("  PCD: ..%s\n", pcd_relative_file.c_str());

      if (bounding_box)
      {
        Eigen::Vector3d min, max;
        node->getBoundingBox(min, max);

        printDepth(node_depth);
        PCL_INFO ("  Bounding Box: <%lf, %lf, %lf> - <%lf, %lf, %lf>\n", min[0], min[1], min[2], max[0], max[1], max[2]);
      }

      breadth_first_it++;
    }
  }

  return 0;
}


void
printHelp (int, char **argv)
{
  print_info ("This program is used to process pcd fiels into an outofcore data structure viewable by the");
  print_info ("pcl_outofcore_viewer\n\n");
  print_info ("%s <options> <input_tree_dir> \n", argv[0]);
  print_info ("\n");
  print_info ("Options:\n");
  print_info ("\t -depth <depth>                \t Octree depth\n");
  print_info ("\t -bounding_box                 \t Print bounding box info\n");
  print_info ("\t -breadth                      \t Print nodes in breadth-first (Default depth-first)\n");
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

  if (find_switch (argc, argv, "-v"))
    console::setVerbosityLevel (console::L_DEBUG);

  // Defaults
  int depth = INT_MAX;
  bool breadth_first = find_switch (argc, argv, "-breadth");
  bool bounding_box = find_switch (argc, argv, "-bounding_box");

  // Parse options
  parse_argument (argc, argv, "-depth", depth);

  // Parse non-option arguments
  boost::filesystem::path tree_root (argv[argc - 1]);

  // Check if a root directory was specified, use directory of pcd file
  if (boost::filesystem::is_directory (tree_root))
  {
    boost::filesystem::directory_iterator diterend;
    for (boost::filesystem::directory_iterator diter (tree_root); diter != diterend; ++diter)
    {
      const boost::filesystem::path& file = *diter;
      if (!boost::filesystem::is_directory (file))
      {
        if (boost::filesystem::extension (file) == OctreeDiskNode::node_index_extension)
        {
          tree_root = file;
        }
      }
    }
  }

  return outofcorePrint (tree_root, depth, bounding_box, breadth_first);
}

/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2014-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 */

/* Author: Markus Schoeler */

#include <iostream>
#include <fstream>
#include <pcl/simulation/shape_generator_io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

void
displayHelp (const std::string &executable_filename)
{
  pcl::console::print_info (
  "\n----------------------------\n\
  pcl::generate_shape  \n\
  Syntax: %s (Filename of recipe) [-d x]\n\
  \n\
  Optional parameter -d: Set point density of sampling to x points per unit square. Default 200.\n\
  RecipeFile: An ASCII file containing one or more of the following commands (each command goes into a seperate line):\n\
  Primitive shapes:\n\
    Cylinder r h: Create a cylinder with radius r (x/z - plane) and height h (y-axis)\n\
    Cuboid x y z: Create a cuboid with width x, height y and depth z\n\
    Sphere r: Create a sphere with radius r\n\
    Cone r h: Create a cone with radius r (x/z - plane) and height h (y-axis)\n\
    Wedge bx bz tx tz h: Create a wedge with bottom_width bx, bottom_depth bz, top_width tx, top_depth tz and height h (y-axis)\n\
    Torus R r: Create a full torus with outer radius R and inner radius r (x/z-plane)\n\
    Torus R r t_min t_max: Create a section of a torus from t_min till t_max in degrees (measured from the x-axis)\n\
  Transformations (these get applied to the last shape):\n\
    T dx dy dz: Translation along dx, dy and dz\n\
    R [x/y/z] alpha: Rotation around the x,y or z axis of alpha degrees. The axis goes through the object's middle.\n\
    R ax ay az alpha: Rotation around the axis (ax,ay,az) of alpha degrees. The axis goes through the object's middle.\n\
    RC [x/y/z] alpha: Rotation around the x,y or z axis of alpha degrees. The axis goes through the center (0,0,0).\n\
    RC ax ay az alpha: Rotation around the axis (ax,ay,az) of alpha degrees. The axis goes through the center (0,0,0).\n\
  \n\
  Special block instructions:\n\
    Multi block: Everything enclosed in this block will be treated as one object for the next instructions\n\
      Multi_begin\n\
      Multi_end\n\
      \n\
    Cut block: Everything between \"Cut_begin\" and \"by\" will be cut by everything which comes between \"by\" and \"Cut_end\"\n\
      Cut_begin: Start cut block\n\
      by: Goto cutter part\n\
      Cut_end: End cut block\n\n\
    Recipe (Filename of recipe): Includes the recipe from another recipe file. This can also be used within other blocks.\n\
  Other options which can be set in the recipe file or Multi blocks:\n\
    Delete_overlap [True/False]: Set if points which have overlap with other shapes get removed. Default: true\n\
    Label [Single, Merge, Append, Preserve]: Determines how labels of the shapes in a Multi block or recipe file are treated. \n\
          Single: All parts will get a single label.\n\
          Merge: Every part gets its unique label. If parts already have multiple labels they will be merged (default behaviour).\n\
          Append: Keep all labels unique, thus append new labels if parts have multiple labels already.\n\
          Preserve: Preserve the labels of the shapes. Label 0 in part 1 and label 0 in part 2 will stay label 0.\n\
    \n", executable_filename.c_str ());
}

int
main (int argc,
      char **argv)
{
  float point_density = 200;
  if (argc < 2)
  {
    displayHelp (argv[0]);
    exit (1);
  }
  if (pcl::console::find_switch (argc, argv, "-d"))
    pcl::console::parse_argument (argc, argv, "-d", point_density);

  std::string recipe_filename (argv[1]);
  /// Create pcd_filename for the output pcd file
  boost::filesystem::path pcd_filename (recipe_filename);
  pcd_filename.replace_extension (".pcd");

  pcl::simulation::RecipeFile recipe (recipe_filename);
  if (recipe.parsedSuccessful ())
  {
    PCL_INFO ("Generating objects\n");
    pcl::simulation::GeometricShapeBase::PointCloudT::Ptr cloud_ptr = recipe.generate (point_density);    
    PCL_INFO ("Generation finished\n");
    PCL_INFO ("Writing File: %s\n", pcd_filename.c_str ());
    pcl::io::savePCDFileASCII (pcd_filename.string (), *cloud_ptr);
  }
  else
  {
    PCL_ERROR ("Error occured while parsing file %s\n", recipe_filename.c_str ());
    exit (2);
  }
  return (0);
}

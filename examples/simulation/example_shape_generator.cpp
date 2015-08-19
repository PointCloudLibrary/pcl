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

#include <iostream>
#include <pcl/simulation/shape_generator.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

using namespace pcl::simulation;

int
main (int argc,
      char ** argv)
{
  pcl::console::print_info (
  "\n\
----------------------------------------------------------------------------------\n\
  This example will create an animal by combining shapes\n\
  Use the -l parameter to change the labeling of the body parts\n\
  Syntax: %s options\n\
  -l [0/1/2]:\n\
    0 (Single): The object will get a single label\n\
    1 (Merge): Head and torso of the animal will get a unique label (default)\n\
    2 (Append): The labels of the head and torso parts keep their unique label\n\n\
  -d point_density: sample with point_density points per square unit (default: 1000)\n\
  Check results with pcl_viewer by displaying the label field\n\
----------------------------------------------------------------------------------\n", argv[0]);

  // depending on the value of label_handler we will create labels for head and torso (part), labels for each part (append) or a single label for the full object (single).
  MultiShape::LabelHandling label_handler = MultiShape::Merge;
  float point_density = 1000;
  if (pcl::console::find_switch (argc, argv, "-l"))
  {
    unsigned int label_handling;
    pcl::console::parse_argument (argc, argv, "-l", label_handling);
    if (label_handling == 0)
      label_handler = MultiShape::Single;
    else if (label_handling == 1)
      label_handler = MultiShape::Merge;
    else if (label_handling == 2)
      label_handler = MultiShape::Append;
    else
    {
      PCL_INFO ("Could not recognize -l argument using Merge (default)\n");
    }
  }

  if (pcl::console::find_switch (argc, argv, "-d"))
    pcl::console::parse_argument (argc, argv, "-d", point_density);

  GeometricShapePtrVector torso_shapes, head_shapes;

  // ----- Add the Torso -------------
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Cylinder (2, 6)));
  torso_shapes.back ()->rotate (GeometricShapeBase::ZAxis, 90);

  // Leg 1
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Cylinder (0.5, 4)));
  torso_shapes.back ()->translate (-2, -2, -1.5);

  // Leg 2
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Cylinder (0.5, 4)));
  torso_shapes.back ()->translate (2, -2, -1.5);

  // Leg 3
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Cylinder (0.5, 4)));
  torso_shapes.back ()->translate (-2, -2, 1.5);

  // Leg 4
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Cylinder (0.5, 4)));
  torso_shapes.back ()->translate (2, -2, 1.5);

  // Back Spike 1
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Cone (1, 2)));
  torso_shapes.back ()->translate (2, 2, 0);

  // Back Spike 2
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Cone (1, 2)));
  torso_shapes.back ()->translate (1, 2, 0);

  // Back Spike 3
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Cone (1, 2)));
  torso_shapes.back ()->translate (0, 2, 0);

  // Tail
  torso_shapes.push_back (GeometricShapeBase::Ptr (new Torus (3, 0.3, 0, M_PI / 2.f)));
  torso_shapes.back ()->translate (5, -0.5, 0);
  torso_shapes.back ()->rotate (GeometricShapeBase::XAxis, -90);
  torso_shapes.back ()->rotate (GeometricShapeBase::ZAxis, 40);

  // Add all torso parts to one object
  // We use Merge because that way we keep separate labels for the parts and can decide later on if we want to keep or merge them
  // Note: Using Append at this level would not change the results, since it only makes a difference if parts of the multishape contain parts already.
  GeometricShapeBase::Ptr torso (new MultiShape (torso_shapes, true, MultiShape::Merge));

  // ----- Add the head -------------
  head_shapes.push_back (GeometricShapeBase::Ptr (new Sphere (1.5)));
  head_shapes.back ()->translate (-3.2, 2.2, 0);

  // Ear 1
  head_shapes.push_back (GeometricShapeBase::Ptr (new Torus (1, 0.2)));
  head_shapes.back ()->translate (0, 1.5, 0.8);
  head_shapes.back ()->rotate (GeometricShapeBase::XAxis, -80);
  head_shapes.back ()->translate (-3.2, 2.2, 0);

  // Ear 2
  head_shapes.push_back (GeometricShapeBase::Ptr (new Torus (1, 0.2)));
  head_shapes.back ()->translate (0, 1.5, -0.8);
  head_shapes.back ()->rotate (GeometricShapeBase::XAxis, 80);
  head_shapes.back ()->translate (-3.2, 2.2, 0);

  // Trunk
  head_shapes.push_back (GeometricShapeBase::Ptr (new Torus (2, 0.3, 0, M_PI / 3.f)));
  head_shapes.back ()->translate (-1.4, -2, 0);
  head_shapes.back ()->rotate (GeometricShapeBase::XAxis, -90);
  head_shapes.back ()->rotate (GeometricShapeBase::ZAxis, 90);
  head_shapes.back ()->translate (-3.2, 2.2, 0);

  // Add all head parts to one object
  // We use Merge because that way we keep separate labels for the parts and can decide later on if we want to keep or merge them
  // Note: Using Append at this level would not change the results, since it only makes a difference if parts of the multishape contain parts already.
  GeometricShapeBase::Ptr head (new MultiShape (head_shapes, true, MultiShape::Merge));

  // ------ Add head and torso again to one object, since torso and head consist of several parts label_handler=Merge and label_handler=Append yield different results --------
  MultiShape::Ptr animal (new MultiShape (torso, head, true, label_handler));
  PCL_INFO ("Animal generation\n");

  // Generate the points on the shape with a density of point_density points per square unit
  GeometricShapeBase::PointCloudT::Ptr animal_cloud_ptr = animal->generate (point_density);
  PCL_INFO ("Animal generation finished\n");
  PCL_INFO ("Saving pcd file shape_example.pcd\n");
  
  pcl::io::savePCDFileASCII ("shape_example.pcd", *animal_cloud_ptr);
  return 0;
}

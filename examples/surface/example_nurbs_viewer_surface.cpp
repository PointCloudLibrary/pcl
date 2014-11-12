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


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

typedef pcl::PointXYZ Point;

int
main (int argc, char *argv[])
{
  std::string file_3dm;

  if (argc < 2)
  {
    printf ("\nUsage: pcl_example_nurbs_viewer_surface 3dm-out-file\n\n");
    exit (0);
  }
  file_3dm = argv[1];

  pcl::visualization::PCLVisualizer viewer ("B-spline surface viewer");
  viewer.setSize (800, 600);

  int mesh_resolution = 128;

  ON::Begin();

  // load surface
  ONX_Model on_model;
  bool rc = on_model.Read(file_3dm.c_str());

  // print diagnostic
  if ( rc )
    std::cout << "Successfully read: " << file_3dm << std::endl;
  else
    std::cout << "Errors during reading: " << file_3dm << std::endl;

//  ON_TextLog out;
//  on_model.Dump(out);

  if(on_model.m_object_table.Count()==0)
  {
    std::cout << "3dm file does not contain any objects: " << file_3dm << std::endl;
    return -1;
  }

  const ON_Object* on_object = on_model.m_object_table[0].m_object;
  if(on_object==NULL)
  {
    std::cout << "object[0] not valid." << std::endl;
    return -1;
  }

  const ON_NurbsSurface& on_surf = *(ON_NurbsSurface*)on_object;

  pcl::PolygonMesh mesh;
  std::string mesh_id = "mesh_nurbs";
  if(on_model.m_object_table.Count()==1)
  {
    std::cout << "3dm file does not contain a trimming curve: " << file_3dm << std::endl;


    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (on_surf, mesh, mesh_resolution);
  }
  else
  {
    on_object = on_model.m_object_table[1].m_object;
    if(on_object==NULL)
    {
      std::cout << "object[1] not valid." << std::endl;
      return -1;
    }

    const ON_NurbsCurve& on_curv = *(ON_NurbsCurve*)on_object;

    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (on_surf, on_curv, mesh,
                                                                     mesh_resolution);
  }

  viewer.addPolygonMesh (mesh, mesh_id);

  viewer.spin ();
  return 0;
}


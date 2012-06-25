 /*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */

#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>
#include <pcl/gpu/kinfu_large_scale/impl/standalone_marching_cubes.hpp>
#include <pcl/gpu/kinfu_large_scale/world_model.h>
#include <pcl/gpu/kinfu_large_scale/impl/world_model.hpp>

int
main (int argc, char** argv)
{
  //Reading input cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  if(argc < 2) {PCL_ERROR("No pcd to read... Exiting...\n");  return (-1); }

  if (pcl::io::loadPCDFile<pcl::PointXYZI> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", argv[1]);
    return (-1);
  }
  
  // Creating world model object
  pcl::WorldModel<pcl::PointXYZI> wm;
  
  //Adding current cloud to the world model
  wm.addSlice(cloud);
  
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
  std::vector<Eigen::Vector3f> transforms;
  
  //Get world as a vector of cubes 
  wm.getWorldAsCubes (512.0, clouds, transforms, 0.025); // 2.5% overlapp (12 cells with a 512-wide cube)

  //Creating the standalone marching cubes instance
  pcl::gpu::StandaloneMarchingCubes<pcl::PointXYZI> m_cubes;

  //Creating the output
  boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;
  std::vector< boost::shared_ptr<pcl::PolygonMesh> > meshes;

  meshes = m_cubes.getMeshesFromTSDFVector (clouds, transforms);
  
  //Save meshes
  for(int i = 0 ; i < meshes.size () ; ++i)
  {
    std::stringstream name;
    name << "mesh_" << i+1 << ".ply";
    PCL_INFO ("Saving mesh...%d \n", i+1);
    pcl::io::savePLYFile (name.str (), *(meshes[i]));
  }

 PCL_INFO( "Done!\n");
  return (0);
}

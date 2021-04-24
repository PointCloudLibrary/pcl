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
#include <pcl/gpu/kinfu_large_scale/device.h>
#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>
#include <pcl/gpu/kinfu_large_scale/impl/standalone_marching_cubes.hpp>
#include <pcl/gpu/kinfu_large_scale/world_model.h>
#include <pcl/gpu/kinfu_large_scale/impl/world_model.hpp>

#include <pcl/console/parse.h>

int
print_help ()
{
  std::cout << "\nUsage:" << std::endl;
  std::cout << "    pcl_kinfu_largeScale_mesh_output <tsdf_world.pcd> [options]" << std::endl << std::endl ;

  std::cout << "\nAvailable options:" << std::endl;
  std::cout << "    --help, -h                      : print this message" << std::endl;
  std::cout << "    --volume_size <in_meters>       : define integration volume size. MUST match the size used when scanning." << std::endl << std::endl;

  return 0;
}


int
main (int argc, char** argv)
{
  if (pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
    return print_help ();

  //Reading input cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  if (argc < 2) {
    PCL_ERROR ("No pcd file to read... Exiting...\n");
    print_help ();
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZI> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", argv[1]);
    print_help ();
    return (-1);
  }
  
  try {

    // Creating world model object
    pcl::kinfuLS::WorldModel<pcl::PointXYZI> wm;
  
    //Adding current cloud to the world model
    wm.addSlice (cloud);
  
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > transforms;
  
    //Get world as a vector of cubes 
    wm.getWorldAsCubes (pcl::device::kinfuLS::VOLUME_X, clouds, transforms, 0.025); // 2.5% overlapp (12 cells with a 512-wide cube)

    //Creating the standalone marching cubes instance
    float volume_size = pcl::device::kinfuLS::VOLUME_SIZE;
    pcl::console::parse_argument (argc, argv, "--volume_size", volume_size);
    pcl::console::parse_argument (argc, argv, "-vs", volume_size);

    PCL_WARN ("Processing world with volume size set to %.2f meters\n", volume_size);

    pcl::gpu::kinfuLS::StandaloneMarchingCubes<pcl::PointXYZI> m_cubes (pcl::device::kinfuLS::VOLUME_X, pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z, volume_size);

    //~ //Creating the output
    //~ pcl::PolygonMesh::Ptr mesh_ptr_;
    //~ std::vector< pcl::PolygonMesh::Ptr > meshes;

    m_cubes.getMeshesFromTSDFVector (clouds, transforms);

    PCL_INFO ("Done!\n");
    return (0);
  
  }
  catch (const pcl::PCLException& /*e*/) { PCL_ERROR ("PCLException... Exiting...\n"); }
  catch (const std::bad_alloc& /*e*/) { PCL_ERROR ("Bad alloc... Exiting...\n"); }
  catch (const std::exception& /*e*/) { PCL_ERROR ("Exception... Exiting...\n"); }
  return (-1);
}

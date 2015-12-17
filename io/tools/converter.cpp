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
 */

/**
 \author Victor Lamoine
 @b convert_point_clouds_meshes converts OBJ, PCD, PLY, STL, VTK files containing point clouds or meshes into every other format.
 This tool allows to specify the file output type between ASCII, binary and binary compressed.
 **/

//TODO: Inform user about loss of color/alpha during conversion?
// STL does not support color at all
// OBJ does not support color in PCL (the format DOES support color)

#include <vector>

#include <pcl/console/parse.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <boost/make_shared.hpp>

#define ASCII 0
#define BINARY 1
#define BINARY_COMPRESSED 2

/**
 * Display help for this program
 * @param argc[in]
 * @param argv[in]
 */
void
displayHelp (int argc,
             char** argv)
{
  PCL_INFO ("\nUsage: %s [OPTION] SOURCE DEST\n", argv[0]);
  PCL_INFO ("Convert SOURCE point cloud or mesh to DEST.\n\n");

  PCL_INFO ("Available formats types for SOURCE and DEST:\n"
           "\tOBJ (Wavefront)\n"
           "\tPCD (Point Cloud Library)\n"
           "\tPLY (Polygon File Format)\n"
           "\tSTL (STereoLithography)\n"
           "\tVTK (The Visualization Toolkit)\n\n");

  PCL_INFO ("Available options:\n"
           "\t-f, --format Specify DEST output type, available formats are ascii, binary and binary_compressed.\n"
           "\t             When not specified, binary is used as default.\n"
           "\t             OBJ only supports ascii format.\n"
           "\t             binary_compressed is only supported by the PCD file format.\n\n"
           "\t-c --cloud   Output DEST as a point cloud, delete all faces.\n\n");
}

bool
saveMesh (pcl::PolygonMesh& input,
          std::string output_file,
          int output_type);

/**
 * Saves a cloud into the specified file and output type. The file format is automatically parsed.
 * @param input[in] The cloud to be saved
 * @param output_file[out] The output file to be written
 * @param output_type[in] The output file type
 * @return True on success, false otherwise.
 */
bool
savePointCloud (pcl::PCLPointCloud2::Ptr input,
                std::string output_file,
                int output_type)
{
  if (boost::filesystem::path (output_file).extension () == ".pcd")
  {
    //TODO Support precision, origin, orientation
    pcl::PCDWriter w;
    if (output_type == ASCII)
    {
      PCL_INFO ("Saving file %s as ASCII.\n", output_file.c_str ());
      if (w.writeASCII (output_file, *input) != 0)
        return (false);
    }
    else if (output_type == BINARY)
    {
      PCL_INFO ("Saving file %s as binary.\n", output_file.c_str ());
      if (w.writeBinary (output_file, *input) != 0)
        return (false);
    }
    else if (output_type == BINARY_COMPRESSED)
    {
      PCL_INFO ("Saving file %s as binary compressed.\n", output_file.c_str ());
      if (w.writeBinaryCompressed (output_file, *input) != 0)
        return (false);
    }
  }
  else if (boost::filesystem::path (output_file).extension () == ".stl")
  {
    PCL_ERROR ("STL file format does not support point clouds! Aborting.\n");
    return (false);
  }
  else  // OBJ, PLY and VTK
  {
    //TODO: Support precision
    //FIXME: Color is lost during OBJ conversion (OBJ supports color)
    pcl::PolygonMesh mesh;
    mesh.cloud = *input;
    if (!saveMesh (mesh, output_file, output_type))
      return (false);
  }

  return (true);
}

/**
 * Saves a mesh into the specified file and output type. The file format is automatically parsed.
 * @param input[in] The mesh to be saved
 * @param output_file[out] The output file to be written
 * @param output_type[in]  The output file type
 * @return True on success, false otherwise.
 */
bool
saveMesh (pcl::PolygonMesh& input,
          std::string output_file,
          int output_type)
{
  if (boost::filesystem::path (output_file).extension () == ".obj")
  {
    if (output_type == BINARY || output_type == BINARY_COMPRESSED)
      PCL_WARN ("OBJ file format only supports ASCII.\n");

    //TODO: Support precision
    //FIXME: Color is lost during conversion (OBJ supports color)
    PCL_INFO ("Saving file %s as ASCII.\n", output_file.c_str ());
    if (pcl::io::saveOBJFile (output_file, input) != 0)
      return (false);
  }
  else if (boost::filesystem::path (output_file).extension () == ".pcd")
  {
    if (!input.polygons.empty ())
      PCL_WARN ("PCD file format does not support meshes! Only points be saved.\n");
    pcl::PCLPointCloud2::Ptr cloud = boost::make_shared<pcl::PCLPointCloud2> (input.cloud);
    if (!savePointCloud (cloud, output_file, output_type))
      return (false);
  }
  else  // PLY, STL and VTK
  {
    if (output_type == BINARY_COMPRESSED)
      PCL_WARN ("PLY, STL and VTK file formats only supports ASCII and binary output file types.\n");

    if (input.polygons.empty() && boost::filesystem::path (output_file).extension () == ".stl")
    {
      PCL_ERROR ("STL file format does not support point clouds! Aborting.\n");
      return (false);
    }

    PCL_INFO ("Saving file %s as %s.\n", output_file.c_str (), (output_type == ASCII) ? "ASCII" : "binary");
    if (!pcl::io::savePolygonFile (output_file, input, (output_type == ASCII) ? false : true))
      return (false);
  }

  return (true);
}

/**
 * Parse input files and options. Calls the right conversion function.
 * @param argc[in]
 * @param argv[in]
 * @return 0 on success, any other value on failure.
 */
int
main (int argc,
      char** argv)
{
  // Display help
  if (pcl::console::find_switch (argc, argv, "-h") != 0 || pcl::console::find_switch (argc, argv, "--help") != 0)
  {
    displayHelp (argc, argv);
    return (0);
  }

  // Parse all files and options
  std::vector<std::string> supported_extensions;
  supported_extensions.push_back("obj");
  supported_extensions.push_back("pcd");
  supported_extensions.push_back("ply");
  supported_extensions.push_back("stl");
  supported_extensions.push_back("vtk");
  std::vector<int> file_args;
  for (int i = 1; i < argc; ++i)
    for (size_t j = 0; j < supported_extensions.size(); ++j)
      if (boost::algorithm::ends_with(argv[i], supported_extensions[j]))
      {
        file_args.push_back(i);
        break;
      }

  std::string parsed_output_type;
  pcl::console::parse_argument (argc, argv, "-f", parsed_output_type);
  pcl::console::parse_argument (argc, argv, "--format", parsed_output_type);
  bool cloud_output (false);
  if (pcl::console::find_switch (argc, argv, "-c") != 0 ||
      pcl::console::find_switch (argc, argv, "--cloud") != 0)
    cloud_output = true;

  // Make sure that we have one input and one output file only
  if (file_args.size() != 2)
  {
    PCL_ERROR ("Wrong input/output file count!\n");
    displayHelp (argc, argv);
    return (-1);
  }

  // Convert parsed output type to output type
  int output_type (BINARY);
  if (!parsed_output_type.empty ())
  {
    if (parsed_output_type == "ascii")
      output_type = ASCII;
    else if (parsed_output_type == "binary")
      output_type = BINARY;
    else if (parsed_output_type == "binary_compressed")
      output_type = BINARY_COMPRESSED;
    else
    {
      PCL_ERROR ("Wrong output type!\n");
      displayHelp (argc, argv);
      return (-1);
    }
  }

  // Try to load as mesh
  pcl::PolygonMesh mesh;
  if (boost::filesystem::path (argv[file_args[0]]).extension () != ".pcd" &&
      pcl::io::loadPolygonFile (argv[file_args[0]], mesh) != 0)
  {
    PCL_INFO ("Loaded a mesh with %d points (total size is %d) and the following channels:\n%s\n",
             mesh.cloud.width * mesh.cloud.height, mesh.cloud.data.size (), pcl::getFieldsList (mesh.cloud).c_str ());

    if (cloud_output)
      mesh.polygons.clear();

    if (saveMesh (mesh, argv[file_args[1]], output_type))
      return (-1);
  }
  else if (boost::filesystem::path (argv[file_args[0]]).extension () == ".stl")
  {
    PCL_ERROR ("Unable to load %s.\n", argv[file_args[0]]);
    return (-1);
  }
  else
  {
    // PCD, OBJ, PLY or VTK
    if (boost::filesystem::path (argv[file_args[0]]).extension () != ".pcd")
      PCL_WARN ("Could not load %s as a mesh, trying as a point cloud instead.\n", argv[file_args[0]]);

    //Eigen::Vector4f origin; // TODO: Support origin/orientation
    //Eigen::Quaternionf orientation;
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    if (pcl::io::load (argv[file_args[0]], *cloud) < 0)
    {
      PCL_ERROR ("Unable to load %s.\n", argv[file_args[0]]);
      return (-1);
    }

    PCL_INFO ("Loaded a point cloud with %d points (total size is %d) and the following channels:\n%s\n", cloud->width * cloud->height, cloud->data.size (),
              pcl::getFieldsList (*cloud).c_str ());

    if (!savePointCloud (cloud, argv[file_args[1]], output_type))
    {
      PCL_ERROR ("Failed to save %s.\n", argv[file_args[1]]);
      return (-1);
    }
  }
  return (0);
}

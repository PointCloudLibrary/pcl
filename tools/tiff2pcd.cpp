/**
 *  Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 * @brief This file is an app to convert two folders with tiffs to pointclouds
 * @copyright Copyright (2012) KU Leuven
 * @authors Koen Buys
 **/

#include <iostream>
#include <boost/filesystem.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <pcl/io/vtk_lib_io.h>
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkTIFFReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

using namespace pcl;

void processAndSave( vtkSmartPointer<vtkImageData>  depth_data,
                     vtkSmartPointer<vtkImageData>  rgb_data,
                     std::string                    time,
                     float                          focal_length,
                     bool                           format,
                     bool                           color,
                     bool                           depth,
                     bool                           use_output_path,
                     std::string                    output_path)
{
  // Retrieve the entries from the image data and copy them into the output RGB cloud
  int rgb_components = rgb_data->GetNumberOfScalarComponents();
  //std::cout << "RGB comp:" << rgb_components << std::endl;

  if(rgb_components != 3)
  {
    std::cout << "RGB image doesn't have 3 components, proceed with next image" << std::endl;
    return;
  }

  int rgb_dimensions[3];
  rgb_data->GetDimensions (rgb_dimensions);
  //std::cout << "RGB dim1: " << rgb_dimensions[0] << " dim2: " << rgb_dimensions[1] << " dim3: " << rgb_dimensions[2] << std::endl;

  // Retrieve the entries from the image data and copy them into the output RGB cloud
  int depth_components = depth_data->GetNumberOfScalarComponents();
  //std::cout << "Depth comp:" << depth_components << std::endl;

  if(depth_components != 1)
  {
    std::cout << "Depth image doesn't have a single component, proceed with next image" << std::endl;
    return;
  }

  int depth_dimensions[3];
  depth_data->GetDimensions (depth_dimensions);
  //std::cout << "Depth dim1: " << depth_dimensions[0] << " dim2: " << depth_dimensions[1] << " dim3: " << depth_dimensions[2] << std::endl;

  // Check if dimensions fit
  if(rgb_dimensions[0] != depth_dimensions[0] || rgb_dimensions[1] != depth_dimensions[1] || rgb_dimensions[2] != depth_dimensions[2])
  {
    std::cout << "RGB and Depth dimensions don't match, proceed with next image" << std::endl;
    return;
  }

  // Now let's create the clouds
  PointCloud<RGB>          pc_image;
  PointCloud<Intensity>    pc_depth;
  PointCloud<PointXYZRGBA> pc_xyzrgba;

  pc_image.width = pc_depth.width = pc_xyzrgba.width = rgb_dimensions[0];
  pc_image.height = pc_depth.height = pc_xyzrgba.height = rgb_dimensions[1];

  float constant = 1.0f / focal_length;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for(int v = 0; v < rgb_dimensions[1]; v++)
  {
    for(int u = 0; u < rgb_dimensions[0]; u++)
    {
      RGB           color_point;
      Intensity     depth_point;
      PointXYZRGBA  xyzrgba_point;

      color_point.r = xyzrgba_point.r = static_cast<uint8_t> (rgb_data->GetScalarComponentAsFloat(u,v,0,0));
      color_point.g = xyzrgba_point.g = static_cast<uint8_t> (rgb_data->GetScalarComponentAsFloat(u,v,0,1));
      color_point.b = xyzrgba_point.b = static_cast<uint8_t> (rgb_data->GetScalarComponentAsFloat(u,v,0,2));
      xyzrgba_point.a = 0;

      pc_image.points.push_back(color_point);
      pc_depth.points.push_back(depth_point);

      float d =  depth_data->GetScalarComponentAsFloat(u,v,0,0);
      depth_point.intensity = d;

      if(d != 0 && !pcl_isnan(d) && !pcl_isnan(d))
      {
        xyzrgba_point.z = d * 0.001f;
        xyzrgba_point.x = static_cast<float> (u) * d * 0.001f * constant;
        xyzrgba_point.y = static_cast<float> (v) * d * 0.001f * constant;
      }
      else
      {
        xyzrgba_point.z = xyzrgba_point.x = xyzrgba_point.y = bad_point;
      }
      pc_xyzrgba.points.push_back(xyzrgba_point);

    } // for u
  } // for v

  std::stringstream ss;

  if(depth)
  {
    if(use_output_path)
      ss << output_path << "/frame_" << time << "_depth.pcd";
    else
      ss << "frame_" << time << "_depth.pcd";
    pcl::io::savePCDFile (ss.str(), pc_depth, format);
    ss.str(""); //empty
  }

  if(color)
  {
    if(use_output_path)
      ss << output_path << "/frame_" << time << "_color.pcd";
    else
      ss << "frame_" << time << "_color.pcd";
    pcl::io::savePCDFile (ss.str(), pc_image, format);
    ss.str(""); //empty
  }

  if(use_output_path)
    ss << output_path << "/frame_" << time << "_xyzrgba.pcd";
  else
    ss << "frame_" << time << "_xyzrgba.pcd";
  pcl::io::savePCDFile (ss.str(), pc_xyzrgba, format);

  std::cout << "Saved " << time << " to pcd" << std::endl;
  return;
}

void print_usage(void)
{
  PCL_INFO("usage: convert -rgb <rgb_path> -depth <depth_path> -out <output_path> options\n");
  PCL_INFO("This program converts rgb and depth tiff files to pcd files");
  PCL_INFO("Options:\n");
  PCL_INFO("\t -v \t Set verbose output\n");
  PCL_INFO("\t -c \t Create color pcd output\n");
  PCL_INFO("\t -d \t Create depth output\n");
  PCL_INFO("\t -b \t Set to save pcd binary, otherwise ascii\n");
  PCL_INFO("\t -f \t Focal length, default 525\n");
  PCL_INFO("\t -h \t This help\n");
}

int main(int argc, char ** argv)
{
  // TODO: adjust these
  if(argc < 3)
  {
    print_usage();
    exit(-1);
  }

  bool verbose = 0;
  pcl::console::parse_argument (argc, argv, "-v", verbose);

  bool format = 0;
  pcl::console::parse_argument (argc, argv, "-b", format);

  bool color = 0;
  pcl::console::parse_argument (argc, argv, "-c", format);

  bool depth = 0;
  pcl::console::parse_argument (argc, argv, "-d", format);

  std::string rgb_path_, depth_path_, output_path_;

  if(pcl::console::parse_argument (argc, argv, "-rgb", rgb_path_) == 0)
  {
    PCL_ERROR("No RGB Path given\n");
    print_usage();
    exit(-1);
  }
  if(pcl::console::parse_argument (argc, argv, "-depth", depth_path_) == 0)
  {
    PCL_ERROR("No Depth Path given\n");
    print_usage();
    exit(-1);
  }
  bool use_output_path = false;
  if(pcl::console::parse_argument (argc, argv, "-out", output_path_) == 0)
  {
    PCL_ERROR("No Output Path given\n");
    print_usage();
    exit(-1);
  }
  else
  {
    use_output_path = true;
  }

  float focal_length = 525.0;
  pcl::console::parse_argument (argc, argv, "-f", focal_length);

  if(verbose)
    PCL_INFO ("Creating RGB Tiff List\n");

  std::vector<std::string> tiff_rgb_files;
  std::vector<boost::filesystem::path> tiff_rgb_paths;
  boost::filesystem::directory_iterator end_itr;

  if(boost::filesystem::is_directory(rgb_path_))
  {
    for (boost::filesystem::directory_iterator itr(rgb_path_); itr != end_itr; ++itr)
    {
      std::string ext = itr->path().extension().string();
      if(ext.compare(".tiff") == 0)
      {
        tiff_rgb_files.push_back (itr->path ().string ());
        tiff_rgb_paths.push_back (itr->path ());
      }
      else
      {
        // Found non tiff file
      }

      if(verbose)
      {
        std::cout << "Extension" << itr->path().extension() << std::endl;
        std::cout << "Filename" << itr->path().filename() << std::endl;
        //std::cout << "Root_dir" << itr->path().root_directory() << std::endl;
        //std::cout << "Root_path" << itr->path().root_path() << std::endl;
        //std::cout << "Root_name" << itr->path().root_name() << std::endl;
        //std::cout << "rel_path" << itr->path().relative_path() << std::endl;
        //std::cout << "parent_path" << itr->path().parent_path() << std::endl;
      }
    }
  }
  else
  {
    PCL_ERROR("RGB path is not a directory\n");
    exit(-1);
  }

  sort (tiff_rgb_files.begin (), tiff_rgb_files.end ());
  sort (tiff_rgb_paths.begin (), tiff_rgb_paths.end ());

  if(verbose)
    PCL_INFO ("Creating Depth Tiff List\n");

  std::vector<std::string> tiff_depth_files;
  std::vector<boost::filesystem::path> tiff_depth_paths;

  if(boost::filesystem::is_directory(depth_path_))
  {
    for (boost::filesystem::directory_iterator itr(depth_path_); itr != end_itr; ++itr)
    {
      std::string ext = itr->path().extension().string();
      if(ext.compare(".tiff") == 0)
      {
        tiff_depth_files.push_back (itr->path ().string ());
        tiff_depth_paths.push_back (itr->path ());
      }
      else
      {
        // Found non tiff file
      }

      if(verbose)
      {
        std::cout << "Extension" << itr->path().extension() << std::endl;
        std::cout << "Filename" << itr->path().filename() << std::endl;
      }
    }
  }
  else
  {
    PCL_ERROR("Depth path is not a directory\n");
    exit(-1);
  }

  sort (tiff_depth_files.begin (), tiff_depth_files.end ());
  sort (tiff_depth_paths.begin (), tiff_depth_paths.end ());

  for(unsigned int i=0; i<tiff_rgb_paths.size(); i++)
  {
    // Load the input file
    vtkSmartPointer<vtkImageData> rgb_data;
    vtkSmartPointer<vtkTIFFReader> reader = vtkSmartPointer<vtkTIFFReader>::New ();

    // Check if the file is correct
    int ret = reader->CanReadFile (tiff_rgb_files[i].c_str());
    // 0 can't read the file, 1 can't prove it
    if(ret == 0 || ret == 1)
    {
      std::cout << "We have a broken tiff file: " << tiff_rgb_files[i] << std::endl;
      continue;
    }
    // 2 can read it, 3 I'm the correct reader
    if(ret == 2 || ret == 3)
    {
      reader->SetFileName (tiff_rgb_files[i].c_str());
      rgb_data = reader->GetOutput ();
      rgb_data->Update ();

      std::string rgb_filename = tiff_rgb_paths[i].filename().string();
      std::string rgb_time = rgb_filename.substr(6,22);

      //std::cout << "RGB Time: " << rgb_time << std::endl;

      // Try to read the depth file
      int found = 0; // indicates if a corresponding depth file was found
      // Find the correct file name
      for(size_t j = 0; j < tiff_depth_paths.size(); j++)
      {
        std::string depth_filename = tiff_depth_paths[i].filename().string();
        std::string depth_time = depth_filename.substr(6,22);

        if(depth_time.compare(rgb_time) == 0) // found the correct depth
        {
          //std::cout << "Depth Time: " << depth_time << std::endl;
          found = 1;

          // Process here!

          vtkSmartPointer<vtkImageData> depth_data;
          vtkSmartPointer<vtkTIFFReader> depth_reader = vtkSmartPointer<vtkTIFFReader>::New ();

          // Check if the file is correct
          int read = depth_reader->CanReadFile (tiff_depth_files[j].c_str());
          // 0 can't read the file, 1 can't prove it
          if(read == 0 || read == 1)
          {
            std::cout << "We have a broken tiff file: " << tiff_depth_files[j] << std::endl;
            continue;
          }
          // 2 can read it, 3 I'm the correct reader
          if(read == 2 || read == 3)
          {
            depth_reader->SetFileName (tiff_depth_files[j].c_str());
            depth_data = depth_reader->GetOutput ();
            depth_data->Update ();

            processAndSave(depth_data, rgb_data, depth_time, focal_length, format, color, depth, use_output_path, output_path_);
          }

          // TODO: remove this depth entry from vector before break > speed up search time
          break;
        }
        else
        {
          // Continue with the next depth entry
          continue;
        }
        if(found == 0)
        {
          std::cout << "We couldn't find a Depth file for this RGB image" << std::endl;
        }
      } //for depth_paths
    } //if ret = 2 or 3
  } //for rgb paths
  return 0;
}

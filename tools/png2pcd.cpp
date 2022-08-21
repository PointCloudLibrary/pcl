/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id: png2pcd.cpp 6766 2012-08-09 16:44:37Z gioia $
 *
 */

/** \brief PNG 2 PCD converter
 *
 * This converter takes two input: the name of the input PNG file and the name of the output PCD file.
 * It performs the conversion of the PNG file into the PCD file by creating a pcl::PointCloud<pcl::RGB>>
 * point cloud.
 *
 * \author Gioia Ballin
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <vtkImageData.h> // for vtkImageData
#include <vtkSmartPointer.h> // for vtkSmartPointer
#include <vtkPNGReader.h> // for vtkPNGReader

#define RED_MULTIPLIER 0.299
#define GREEN_MULTIPLIER 0.587
#define BLUE_MULTIPLIER 0.114
#define MAX_COLOR_INTENSITY 255

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*                      PNG 2 PCD CONVERTER - Usage Guide                  *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << argv[0] << " [Options] color.png [depth.png] output.pcd" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                                               Show this help." << std::endl;
  std::cout << "     -format 0 | 1:                                    Set the format of the output pcd file." << std::endl;
  std::cout << "     -mode DEFAULT | FORCE_COLOR | FORCE_GRAYSCALE:    Set the working mode of the converter." << std::endl;
  std::cout << "       --intensity_type: FLOAT | UINT_8                Set the desired intensity type" << std::endl;
  std::cout << "                                                       FLOAT is always used when depth input file is valid." << std::endl;
  std::cout << "       --v_viewing_angle angle                         Set vertical viewing angle of 3D camera." << std::endl;
  std::cout << "                                                       Default value is 43 (for Kinect device)." << std::endl;
  std::cout << "                                                       This option is only used when depth input file is valid." << std::endl;
  std::cout << "       --h_viewing_angle angle                         Set horizontal viewing angle of 3D camera." << std::endl;
  std::cout << "                                                       Default value is 57 (for Kinect device)." << std::endl;
  std::cout << "                                                       This option is only used when depth input file is valid." << std::endl;
  std::cout << "       --depth_unit mm | m                             Set unit of depth values in depth.png (default is mm)." << std::endl;
}

template<typename PointInT> void
saveCloud (const std::string &filename, const PointCloud<PointInT> &cloud, bool format)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  savePCDFile (filename, cloud, format);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

inline void
depth2xyz (float v_viewing_angle, float h_viewing_angle,
           int image_width, int image_height, int image_x, int image_y,
           float depth, float &x, float &y, float &z)
{
  constexpr float PI = 3.1415927;

  if (depth <= 0.0f)
  {
    x = y = z = std::numeric_limits<float>::quiet_NaN ();
  }
  else
  {
    float width = depth * std::tan (h_viewing_angle * PI / 180 / 2) * 2;
    float height = depth * std::tan (v_viewing_angle * PI / 180 / 2) * 2;

    x = (image_x - image_width / 2.0) / image_width * width;
    y = (image_height / 2.0 - image_y) / image_height * height;
    z = depth;
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a (or two with color and depth) PNG file to PCD format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .vtk and .ply files
  std::vector<int> png_file_indices = parse_file_extension_argument (argc, argv, ".png");
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");

  if ((png_file_indices.size () != 1 && png_file_indices.size () != 2) || pcd_file_indices.size () != 1)
  {
    print_error ("Need one/two input PNG file and one output PCD file.\n");
    return (-1);
  }

  // Command line parsing
  bool format = false;
  parse_argument (argc, argv, "-format", format);
  print_info ("PCD output format: "); print_value ("%s\n", (format ? "binary" : "ascii"));

  std::string mode = "DEFAULT";
  if (parse_argument (argc, argv, "-mode", mode) != -1)
  {
    if (! (mode == "DEFAULT" || mode == "FORCE_COLOR" || mode == "FORCE_GRAYSCALE") )
    {
      std::cout << "Wrong mode name.\n";
      printHelp (argc, argv);
      exit (-1);
    }
  }

  print_info ("%s mode selected.\n", mode.c_str ());

  // Load the color input file
  vtkSmartPointer<vtkImageData> color_image_data;
  vtkSmartPointer<vtkPNGReader> color_reader = vtkSmartPointer<vtkPNGReader>::New ();
  color_reader->SetFileName (argv[png_file_indices[0]]);
  color_reader->Update ();
  color_image_data = color_reader->GetOutput ();

  int components = color_image_data->GetNumberOfScalarComponents ();
  int dimensions[3];
  color_image_data->GetDimensions (dimensions);

  // Load the depth input file
  vtkSmartPointer<vtkImageData> depth_image_data;
  vtkSmartPointer<vtkPNGReader> depth_reader;
  bool enable_depth = false;
  float v_viewing_angle = 43.0f;
  float h_viewing_angle = 57.0f;
  float depth_unit_magic = 1000.0f;
  if (png_file_indices.size () == 2)
  {
    depth_reader = vtkSmartPointer<vtkPNGReader>::New ();
    depth_reader->SetFileName (argv[png_file_indices[1]]);
    depth_reader->Update ();
    depth_image_data = depth_reader->GetOutput ();

    if (depth_reader->GetNumberOfScalarComponents () != 1)
    {
      print_error ("Component number of depth input file should be 1.\n");
      exit (-1);
    }
    
    int depth_dimensions[3];
    depth_image_data->GetDimensions (depth_dimensions);
    if (depth_dimensions[0] != dimensions[0] || depth_dimensions[1] != dimensions[1])
    {
      print_error ("Width or height of the color and depth input file should be the same.\n");
      exit (-1);
    }

    parse_argument (argc, argv, "--v_viewing_angle", v_viewing_angle);
    parse_argument (argc, argv, "--h_viewing_angle", h_viewing_angle);

    std::string depth_unit;
    if (parse_argument (argc, argv, "--depth_unit", depth_unit) > 0)
    {
      if (depth_unit == "m")
      {
        depth_unit_magic = 1.0f;
      }
      else if (depth_unit == "mm")
      {
        depth_unit_magic = 1000.0f;
      }
      else
      {
        print_error ("Unknown depth unit defined.\n");
        exit (-1);
      }
    }

    enable_depth = true;
  }

  // Retrieve the entries from the image data and copy them into the output RGB cloud
  double* pixel = new double [4]{0.0};
  float depth;

  std::string intensity_type;

  if (mode == "DEFAULT")
  {
    //
    // If the input image is a monochrome image the output cloud will be:
    // - a pcl::PointCloud<pcl::Intensity> if the intensity_type flag is set to FLOAT;
    // - a pcl::PointCloud<pcl::Intensity8u> if the intensity_type flag is set to UINT_8;
    // otherwise it will be a pcl::PointCloud<pcl::RGB>.
    //

    if (pcl::console::parse_argument (argc, argv, "--intensity_type", intensity_type) != -1)
    {
      if (intensity_type != "FLOAT" && intensity_type != "UINT_8")
      {
        print_error ("Wrong intensity option.\n");
        printHelp (argc, argv);
        exit (-1);
      }
    }
    else
    {
      print_error ("The intensity type must be set to enable the PNG conversion.\n");
      exit (-1);
    }

    PointCloud<Intensity> mono_cloud;
    PointCloud<Intensity8u> mono_cloud_u8;
    PointCloud<RGB> color_cloud;
    PointCloud<PointXYZI> mono_depth_cloud;
    PointCloud<PointXYZRGB> rgb_depth_cloud;
    PointCloud<PointXYZRGBA> rgba_depth_cloud;

    switch (components)
    {
      case 1: if (intensity_type == "FLOAT")
      {
        if (enable_depth)
        {
          mono_depth_cloud.width = dimensions[0];
          mono_depth_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
          mono_depth_cloud.is_dense = false;
          mono_depth_cloud.resize (mono_depth_cloud.width * mono_depth_cloud.height);

          for (int y = 0; y < dimensions[1]; y++)
          {
            for (int x = 0; x < dimensions[0]; x++)
            {
              pixel[0] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 0);
              depth = depth_image_data->GetScalarComponentAsFloat (x, y, 0, 0) / depth_unit_magic;

              PointXYZI xyzi;
              depth2xyz (v_viewing_angle, h_viewing_angle,
                         mono_depth_cloud.width, mono_depth_cloud.height, x, y,
                         depth, xyzi.x, xyzi.y, xyzi.z);
              xyzi.intensity = static_cast<float> (pixel[0]) / MAX_COLOR_INTENSITY;

              mono_depth_cloud (x, dimensions[1] - y -1) = xyzi;
            }
          }

          // Save the point cloud into a PCD file
          saveCloud<PointXYZI> (argv[pcd_file_indices[0]], mono_depth_cloud, format);
        }
        else
        {
          mono_cloud.width = dimensions[0];
          mono_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
          mono_cloud.is_dense = true;
          mono_cloud.resize (mono_cloud.width * mono_cloud.height);

          for (int y = 0; y < dimensions[1]; y++)
          {
            for (int x = 0; x < dimensions[0]; x++)
            {
              pixel[0] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 0);

              Intensity gray;
              gray.intensity = static_cast<float> (pixel[0]) / MAX_COLOR_INTENSITY;

              mono_cloud (x, dimensions[1] - y - 1) = gray;
            }
          }

          // Save the point cloud into a PCD file
          saveCloud<Intensity> (argv[pcd_file_indices[0]], mono_cloud, format);
        }
      }
      else
      {
        mono_cloud_u8.width = dimensions[0];
        mono_cloud_u8.height = dimensions[1]; // This indicates that the point cloud is organized
        mono_cloud_u8.is_dense = true;
        mono_cloud_u8.resize (mono_cloud_u8.width * mono_cloud_u8.height);

        for (int y = 0; y < dimensions[1]; y++)
        {
          for (int x = 0; x < dimensions[0]; x++)
          {
            pixel[0] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 0);

            Intensity8u gray;
            gray.intensity = static_cast<std::uint8_t> (pixel[0]);

            mono_cloud_u8 (x, dimensions[1] - y - 1) = gray;
          }
        }

        // Save the point cloud into a PCD file
        saveCloud<Intensity8u> (argv[pcd_file_indices[0]], mono_cloud_u8, format);
      }
      break;

      case 3: if (enable_depth)
      {
        rgb_depth_cloud.width = dimensions[0];
        rgb_depth_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
        rgb_depth_cloud.is_dense = false;
        rgb_depth_cloud.resize (rgb_depth_cloud.width * rgb_depth_cloud.height);

        for (int y = 0; y < dimensions[1]; y++)
        {
          for (int x = 0; x < dimensions[0]; x++)
          {
            pixel[0] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 0);
            pixel[1] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 1);
            pixel[2] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 2);
            depth = depth_image_data->GetScalarComponentAsFloat (x, y, 0, 0) / depth_unit_magic;

            PointXYZRGB xyzrgb;
            depth2xyz (v_viewing_angle, h_viewing_angle,
                       rgb_depth_cloud.width, rgb_depth_cloud.height, x, y,
                       depth, xyzrgb.x, xyzrgb.y, xyzrgb.z);
            xyzrgb.r = static_cast<std::uint8_t> (pixel[0]);
            xyzrgb.g = static_cast<std::uint8_t> (pixel[1]);
            xyzrgb.b = static_cast<std::uint8_t> (pixel[2]);

            rgb_depth_cloud (x, dimensions[1] - y - 1) = xyzrgb;
          }
        }

        // Save the point cloud into a PCD file
        saveCloud<PointXYZRGB> (argv[pcd_file_indices[0]], rgb_depth_cloud, format);
      }
      else
      {
        color_cloud.width = dimensions[0];
        color_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
        color_cloud.is_dense = true;
        color_cloud.resize (color_cloud.width * color_cloud.height);

        for (int y = 0; y < dimensions[1]; y++)
        {
          for (int x = 0; x < dimensions[0]; x++)
          {
            pixel[0] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 0);
            pixel[1] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 1);
            pixel[2] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 2);

            RGB color;
            color.r = static_cast<std::uint8_t> (pixel[0]);
            color.g = static_cast<std::uint8_t> (pixel[1]);
            color.b = static_cast<std::uint8_t> (pixel[2]);

            int rgb = (static_cast<int> (color.r)) << 16 |
                      (static_cast<int> (color.g)) << 8 |
                      (static_cast<int> (color.b));

            color.rgb = static_cast<float> (rgb);

            color.rgba = static_cast<std::uint32_t> (rgb);

            color_cloud (x, dimensions[1] - y - 1) = color;
          }
        }

        // Save the point cloud into a PCD file
        saveCloud<RGB> (argv[pcd_file_indices[0]], color_cloud, format);
      }
      break;

      case 4: if (enable_depth)
      {
        rgba_depth_cloud.width = dimensions[0];
        rgba_depth_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
        rgba_depth_cloud.is_dense = false;
        rgba_depth_cloud.resize (rgba_depth_cloud.width * rgba_depth_cloud.height);

        for (int y = 0; y < dimensions[1]; y++)
        {
          for (int x = 0; x < dimensions[0]; x++)
          {
            pixel[0] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 0);
            pixel[1] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 1);
            pixel[2] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 2);
            pixel[3] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 3);
            depth = depth_image_data->GetScalarComponentAsFloat (x, y, 0, 0) / depth_unit_magic;

            PointXYZRGBA xyzrgba;
            depth2xyz (v_viewing_angle, h_viewing_angle,
                       rgba_depth_cloud.width, rgba_depth_cloud.height, x, y,
                       depth, xyzrgba.x, xyzrgba.y, xyzrgba.z);
            xyzrgba.r = static_cast<std::uint8_t> (pixel[0]);
            xyzrgba.g = static_cast<std::uint8_t> (pixel[1]);
            xyzrgba.b = static_cast<std::uint8_t> (pixel[2]);
            xyzrgba.a = static_cast<std::uint8_t> (pixel[3]);

            rgba_depth_cloud (x, dimensions[1] - y - 1) = xyzrgba;
          }
        }

        // Save the point cloud into a PCD file
        saveCloud<PointXYZRGBA> (argv[pcd_file_indices[0]], rgba_depth_cloud, format);
      }
      else
      {
        color_cloud.width = dimensions[0];
        color_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
        color_cloud.is_dense = true;
        color_cloud.resize (color_cloud.width * color_cloud.height);

        for (int y = 0; y < dimensions[1]; y++)
        {
          for (int x = 0; x < dimensions[0]; x++)
          {
            pixel[0] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 0);
            pixel[1] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 1);
            pixel[2] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 2);
            pixel[3] = color_image_data->GetScalarComponentAsDouble (x, y, 0, 3);

            RGB color;
            color.r = static_cast<std::uint8_t> (pixel[0]);
            color.g = static_cast<std::uint8_t> (pixel[1]);
            color.b = static_cast<std::uint8_t> (pixel[2]);
            color.a = static_cast<std::uint8_t> (pixel[3]);

            int rgb = (static_cast<int> (color.r)) << 16 |
                      (static_cast<int> (color.g)) << 8 |
                      (static_cast<int> (color.b));
            int rgba = (static_cast<int> (color.a)) << 24 |
                       (static_cast<int> (color.r)) << 16 |
                       (static_cast<int> (color.g)) << 8 |
                       (static_cast<int> (color.b));

            color.rgb = static_cast<float> (rgb);
            color.rgba = static_cast<std::uint32_t> (rgba);

            color_cloud (x, dimensions[1] - y - 1) = color;
          }
        }

        // Save the point cloud into a PCD file
        saveCloud<RGB> (argv[pcd_file_indices[0]], color_cloud, format);
      }
      break;
    }
  }
  else if (mode == "FORCE_COLOR")
  {
    //
    // Force the output cloud to be colored even if the input image is a
    // monochrome image.
    //
    if (enable_depth)
    {
      PointCloud<PointXYZRGBA> cloud;
      int dimensions[3];
      color_image_data->GetDimensions (dimensions);
      cloud.width = dimensions[0];
      cloud.height = dimensions[1]; // This indicates that the point cloud is organized
      cloud.is_dense = false;
      cloud.resize (cloud.width * cloud.height);

      for (int y = 0; y < dimensions[1]; y++)
      {
        for (int x = 0; x < dimensions[0]; x++)
        {
          for (int c = 0; c < components; c++)
            pixel[c] = color_image_data->GetScalarComponentAsDouble (x, y, 0, c);
          depth = depth_image_data->GetScalarComponentAsFloat (x, y, 0, 0) / depth_unit_magic;

          PointXYZRGBA color;
          depth2xyz (v_viewing_angle, h_viewing_angle,
                     cloud.width, cloud.height, x, y,
                     depth, color.x, color.y, color.z);
          color.r = 0;
          color.g = 0;
          color.b = 0;
          color.a = 0;

          switch (components)
          {
            case 1:  color.r = static_cast<std::uint8_t> (pixel[0]);
            color.g = static_cast<std::uint8_t> (pixel[0]);
            color.b = static_cast<std::uint8_t> (pixel[0]);
            break;

            case 3:  color.r = static_cast<std::uint8_t> (pixel[0]);
            color.g = static_cast<std::uint8_t> (pixel[1]);
            color.b = static_cast<std::uint8_t> (pixel[2]);
            break;

            case 4:  color.r = static_cast<std::uint8_t> (pixel[0]);
            color.g = static_cast<std::uint8_t> (pixel[1]);
            color.b = static_cast<std::uint8_t> (pixel[2]);
            color.a = static_cast<std::uint8_t> (pixel[3]);
            break;
          }

          cloud (x, dimensions[1] - y -1) = color;
        }
      }

      // Save the point cloud into a PCD file
      saveCloud<PointXYZRGBA> (argv[pcd_file_indices[0]], cloud, format);
    }
    else
    {
      PointCloud<RGB> cloud;
      int dimensions[3];
      color_image_data->GetDimensions (dimensions);
      cloud.width = dimensions[0];
      cloud.height = dimensions[1]; // This indicates that the point cloud is organized
      cloud.is_dense = true;
      cloud.resize (cloud.width * cloud.height);

      for (int y = 0; y < dimensions[1]; y++)
      {
        for (int x = 0; x < dimensions[0]; x++)
        {
          for (int c = 0; c < components; c++)
            pixel[c] = color_image_data->GetScalarComponentAsDouble (x, y, 0, c);

          RGB color;
          color.r = 0;
          color.g = 0;
          color.b = 0;
          color.a = 0;
          color.rgb = 0.0f;
          color.rgba = 0;

          int rgb;
          int rgba;
          switch (components)
          {
            case 1:  color.r = static_cast<std::uint8_t> (pixel[0]);
            color.g = static_cast<std::uint8_t> (pixel[0]);
            color.b = static_cast<std::uint8_t> (pixel[0]);

            rgb = (static_cast<int> (color.r)) << 16 |
                (static_cast<int> (color.g)) << 8 |
                (static_cast<int> (color.b));

            rgba = rgb;
            color.rgb = static_cast<float> (rgb);
            color.rgba = static_cast<std::uint32_t> (rgba);
            break;

            case 3:  color.r = static_cast<std::uint8_t> (pixel[0]);
            color.g = static_cast<std::uint8_t> (pixel[1]);
            color.b = static_cast<std::uint8_t> (pixel[2]);

            rgb = (static_cast<int> (color.r)) << 16 |
                (static_cast<int> (color.g)) << 8 |
                (static_cast<int> (color.b));

            rgba = rgb;
            color.rgb = static_cast<float> (rgb);
            color.rgba = static_cast<std::uint32_t> (rgba);
            break;

            case 4:  color.r = static_cast<std::uint8_t> (pixel[0]);
            color.g = static_cast<std::uint8_t> (pixel[1]);
            color.b = static_cast<std::uint8_t> (pixel[2]);
            color.a = static_cast<std::uint8_t> (pixel[3]);

            rgb = (static_cast<int> (color.r)) << 16 |
                (static_cast<int> (color.g)) << 8 |
                (static_cast<int> (color.b));
            rgba = (static_cast<int> (color.a)) << 24 |
                (static_cast<int> (color.r)) << 16 |
                (static_cast<int> (color.g)) << 8 |
                (static_cast<int> (color.b));

            color.rgb = static_cast<float> (rgb);
            color.rgba = static_cast<std::uint32_t> (rgba);
            break;
          }

          cloud (x, dimensions[1] - y - 1) = color;
        }
      }

      // Save the point cloud into a PCD file
      saveCloud<RGB> (argv[pcd_file_indices[0]], cloud, format);
    }
  }
  else if (mode == "FORCE_GRAYSCALE")
  {
    if (enable_depth)
    {
      PointCloud<PointXYZI> cloud;
      int dimensions[3];
      color_image_data->GetDimensions (dimensions);
      cloud.width = dimensions[0];
      cloud.height = dimensions[1]; // This indicates that the point cloud is organized
      cloud.is_dense = false;
      cloud.resize (cloud.width * cloud.height);

      for (int y = 0; y < dimensions[1]; y++)
      {
        for (int x = 0; x < dimensions[0]; x++)
        {
          for (int c = 0; c < components; c++)
            pixel[c] = color_image_data->GetScalarComponentAsDouble (x, y, 0, c);
          depth = depth_image_data->GetScalarComponentAsFloat (x, y, 0, 0) / depth_unit_magic;

          PointXYZI gray;
          depth2xyz (v_viewing_angle, h_viewing_angle,
                     cloud.width, cloud.height, x, y,
                     depth, gray.x, gray.y, gray.z);

          switch (components)
          {
            case 1:  gray.intensity = static_cast<float> (pixel[0]) / MAX_COLOR_INTENSITY;
            break;

            case 3:  gray.intensity = static_cast<float> ( RED_MULTIPLIER * pixel[0] +
                GREEN_MULTIPLIER * pixel[1] +
                BLUE_MULTIPLIER * pixel[2] ) / MAX_COLOR_INTENSITY;
            break;

            case 4:  gray.intensity = static_cast<float> ( RED_MULTIPLIER * pixel[0] +
                GREEN_MULTIPLIER * pixel[1] +
                BLUE_MULTIPLIER * pixel[2] ) / MAX_COLOR_INTENSITY;
            break;
          }

          cloud (x, dimensions[1] - y - 1) = gray;
        }
      }

      // Save the point cloud into a PCD file
      saveCloud<PointXYZI> (argv[pcd_file_indices[0]], cloud, format);
    }
    else
    //
    // Force the output cloud to be:
    // - a pcl::PointCloud<pcl::Intensity> if the intensity_type flag is set to FLOAT;
    // - a pcl::PointCloud<pcl::Intensity8u> if the intensity_type flag is set to UINT_8;
    // even if the input image is a RGB or a RGBA image.
    //
    {
      PointCloud<Intensity> cloud;
      PointCloud<Intensity8u> cloud8u;

      if (pcl::console::parse_argument (argc, argv, "--intensity_type", intensity_type) != -1)
      {
        if (intensity_type == "FLOAT")
        {
          cloud.width = dimensions[0];
          cloud.height = dimensions[1]; // This indicates that the point cloud is organized
          cloud.is_dense = true;
          cloud.resize (cloud.width * cloud.height);

          for (int y = 0; y < dimensions[1]; y++)
          {
            for (int x = 0; x < dimensions[0]; x++)
            {
              for (int c = 0; c < components; c++)
                pixel[c] = color_image_data->GetScalarComponentAsDouble (x, y, 0, c);

              Intensity gray;

              switch (components)
              {
                case 1:  gray.intensity = static_cast<float> (pixel[0]) / MAX_COLOR_INTENSITY;
                break;

                case 3:  gray.intensity = static_cast<float> ( RED_MULTIPLIER * pixel[0] +
                    GREEN_MULTIPLIER * pixel[1] +
                    BLUE_MULTIPLIER * pixel[2] ) / MAX_COLOR_INTENSITY;
                break;

                case 4:  gray.intensity = static_cast<float> ( RED_MULTIPLIER * pixel[0] +
                    GREEN_MULTIPLIER * pixel[1] +
                    BLUE_MULTIPLIER * pixel[2] ) / MAX_COLOR_INTENSITY;
                break;
              }

              cloud (x, dimensions[1] - y - 1) = gray;
            }
          }

          // Save the point cloud into a PCD file
          saveCloud<Intensity> (argv[pcd_file_indices[0]], cloud, format);
        }
        else if (intensity_type != "UINT_8")
        {
          cloud8u.width = dimensions[0];
          cloud8u.height = dimensions[1]; // This indicates that the point cloud is organized
          cloud8u.is_dense = true;
          cloud8u.resize (cloud8u.width * cloud8u.height);

          for (int y = 0; y < dimensions[1]; y++)
          {
            for (int x = 0; x < dimensions[0]; x++)
            {
              for (int c = 0; c < components; c++)
                pixel[c] = color_image_data->GetScalarComponentAsDouble (x, y, 0, c);

              Intensity8u gray;

              switch (components)
              {
                case 1:  gray.intensity = static_cast<std::uint8_t> (pixel[0]);
                break;

                case 3:  gray.intensity = static_cast<std::uint8_t> ( RED_MULTIPLIER * pixel[0] +
                    GREEN_MULTIPLIER * pixel[1] +
                    BLUE_MULTIPLIER * pixel[2] );
                break;

                case 4:  gray.intensity = static_cast<std::uint8_t> ( RED_MULTIPLIER * pixel[0] +
                    GREEN_MULTIPLIER * pixel[1] +
                    BLUE_MULTIPLIER * pixel[2] );
                break;
              }

              cloud8u (x, dimensions[1] - y - 1) = gray;
            }
          }

          // Save the point cloud into a PCD file
          saveCloud<Intensity8u> (argv[pcd_file_indices[0]], cloud8u, format);
        }
        else
        {
          print_error ("Wrong intensity option.\n");
          printHelp (argc, argv);
          exit (-1);
        }
      }
      else
      {
        print_error ("The intensity type must be set to enable the PNG conversion.\n");
        exit (-1);
      }
    }
  }

  delete[] pixel;

  return 0;
}

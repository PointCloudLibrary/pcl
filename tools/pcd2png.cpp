/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
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

/** \brief PCD 2 PNG converter
 *
 * This converter takes three inputs: names of the input PCD and output PNG files, and the name of the field.
 *
 * \author Sergey Alexandrov
 *
 */

#include <boost/lexical_cast.hpp>

#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/conversions.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "*                       PCD 2 PNG CONVERTER - Usage Guide                  *" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << argv[0] << " [Options] input.pcd output.png" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     --help   : Show this help" << std::endl;
  std::cout << "     --field  : Set the field to extract data from. Supported fields:"       << std::endl;
  std::cout << "                - normal"                                                    << std::endl;
  std::cout << "                * rgb (default)"                                             << std::endl;
  std::cout << "                - label"                                                     << std::endl;
  std::cout << "                - z"                                                         << std::endl;
  std::cout << "                - curvature"                                                 << std::endl;
  std::cout << "                - intensity"                                                 << std::endl;
  std::cout << "     --scale  : Apply scaling to extracted data (only for z, curvature, and" << std::endl;
  std::cout << "                intensity fields). Supported options:"                       << std::endl;
  std::cout << "                - <float> : Scale by a fixed number"                         << std::endl;
  std::cout << "                - auto    : Auto-scale to the full range"                    << std::endl;
  std::cout << "                - no      : No scaling"                                      << std::endl;
  std::cout << "                If the option is omitted then default scaling (depends on"   << std::endl;
  std::cout << "                the field type) will be used."                               << std::endl;
  std::cout << "     --colors : Choose color mapping mode for labels (only for label"        << std::endl;
  std::cout << "                field). Supported options:"                                  << std::endl;
  std::cout << "                * mono    : Use shades of gray (default)"                    << std::endl;
  std::cout << "                - rgb     : Use randomly generated RGB colors"               << std::endl;
  std::cout << std::endl;
  std::cout << "Note: The converter will try to use RGB field if '--field' option is not"    << std::endl;
  std::cout << "      supplied."                                                             << std::endl;
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveImage (const std::string &filename, const pcl::PCLImage& image)
{
  TicToc tt;
  tt.tic ();
  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  savePNGFile (filename, image);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", image.width * image.height); print_info (" points]\n");
}

template<typename T> bool
parseScaleOption (int argc, char** argv, T& pcie)
{
  std::string scaling = "default";
  pcl::console::parse_argument (argc, argv, "--scale", scaling);
  print_info ("Scaling: "); print_value ("%s\n", scaling.c_str());
  if (scaling == "default")
  {
    // scaling option omitted, use whatever defaults image extractor has
  }
  else if (scaling == "no")
  {
    pcie.setScalingMethod(pcie.SCALING_NO);
  }
  else if (scaling == "auto")
  {
    pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
  }
  else
  {
    try
    {
      float factor = boost::lexical_cast<float> (scaling);
      pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
      pcie.setScalingFactor(factor);
    }
    catch (boost::bad_lexical_cast)
    {
      print_error ("The value of --scale option should be \"no\", \"auto\", or a floating point number.\n");
      return false;
    }
  }
  return true;
}

template<typename T> bool
parseColorsOption (int argc, char** argv, T& pcie)
{
  std::string colors = "mono";
  pcl::console::parse_argument (argc, argv, "--colors", colors);
  print_info ("Colors: "); print_value ("%s\n", colors.c_str());
  if (colors == "mono")
  {
    pcie.setColorMode(pcie.COLORS_MONO);
  }
  else if (colors == "rgb")
  {
    pcie.setColorMode(pcie.COLORS_RGB_RANDOM);
  }
  else
  {
    return false;
  }
  return true;
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a PCD file to PNG format.\nFor more information, use: %s --help\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .png files
  std::vector<int> pcd_file_index = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> png_file_index = parse_file_extension_argument (argc, argv, ".png");

  if (pcd_file_index.size () != 1 || png_file_index.size () != 1)
  {
    print_error ("Need one input PCD file and one output PNG file.\n");
    return (-1);
  }

  std::string pcd_filename = argv[pcd_file_index[0]];
  std::string png_filename = argv[png_file_index[0]];

  // Load the input file
  pcl::PCLPointCloud2::Ptr blob (new pcl::PCLPointCloud2);
  if (!loadCloud (pcd_filename, *blob))
  {
    print_error ("Unable to load PCD file.\n");
    return (-1);
  }

  // Check if the cloud is organized
  if (blob->height == 1)
  {
    print_error ("Input cloud is not organized.\n");
    return (-1);
  }

  std::string field_name = "rgb";
  parse_argument (argc, argv, "--field", field_name);
  print_info ("Field name: "); print_value ("%s\n", field_name.c_str());

  pcl::PCLImage image;
  bool extracted;
  if (field_name == "normal")
  {
    PointCloud<Normal> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromNormalField<Normal> pcie;
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "rgb")
  {
    PointCloud<PointXYZRGB> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromRGBField<PointXYZRGB> pcie;
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "label")
  {
    PointCloud<Label> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromLabelField<Label> pcie;
    if (!parseColorsOption(argc, argv, pcie))
      return (-1);
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "z")
  {
    PointCloud<PointXYZ> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromZField<PointXYZ> pcie;
    if (!parseScaleOption(argc, argv, pcie))
      return (-1);
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "curvature")
  {
    PointCloud<Normal> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromCurvatureField<Normal> pcie;
    if (!parseScaleOption(argc, argv, pcie))
      return (-1);
    extracted = pcie.extract(cloud, image);
  }
  else if (field_name == "intensity")
  {
    PointCloud<Intensity> cloud;
    fromPCLPointCloud2 (*blob, cloud);
    PointCloudImageExtractorFromIntensityField<Intensity> pcie;
    if (!parseScaleOption(argc, argv, pcie))
      return (-1);
    extracted = pcie.extract(cloud, image);
  }
  else
  {
    print_error ("Unsupported field \"%s\".\n", field_name.c_str());
    return (-1);
  }

  if (!extracted)
  {
    print_error ("Failed to extract an image from field \"%s\".\n", field_name.c_str());
    return (-1);
  }
  saveImage (png_filename, image);

  return (0);
}

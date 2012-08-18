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
#include <pcl/io/vtk_lib_io.h>

#include <pcl/visualization/image_viewer.h>

#define RED_MULTIPLIER 0.299
#define GREEN_MULTIPLIER 0.587
#define BLUE_MULTIPLIER 0.114

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
  std::cout << "Usage: " << argv[0] << " [Options] input.png output.pcd" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                                            Show this help." << std::endl;
  std::cout << "     -format 0 | 1:                                 Set the format of the output pcd file." << std::endl;
  std::cout << "     -mode DEFAULT | FORCE_COLOR | FORCE_GRAYSCALE: Set the working mode of the converter." << std::endl;

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

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a PNG file to PCD format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .vtk and .ply files
  std::vector<int> png_file_indices = parse_file_extension_argument (argc, argv, ".png");
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (png_file_indices.size () != 1 || pcd_file_indices.size () != 1)
  {
    print_error ("Need one input PNG file and one output PCD file.\n");
    return (-1);
  }

  // Command line parsing
  bool format = false;
  parse_argument (argc, argv, "-format", format);
  print_info ("PCD output format: "); print_value ("%s\n", (format ? "binary" : "ascii"));

  std::string mode = "DEFAULT";
  if (parse_argument (argc, argv, "-mode", mode) != -1)
  {
    if (! (mode.compare ("DEFAULT") == 0 || mode.compare ("FORCE_COLOR") == 0 || mode.compare ("FORCE_GRAYSCALE") == 0) )
    {
	std::cout << "Wrong mode name.\n";
	printHelp (argc, argv);
	exit (-1);
    }
  }

  print_info ("%s mode selected.\n", mode.c_str ());

  // Load the input file
  vtkSmartPointer<vtkImageData> image_data;
  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New ();
  reader->SetFileName (argv[png_file_indices[0]]);
  image_data = reader->GetOutput ();
  image_data->Update ();

  // Retrieve the entries from the image data and copy them into the output RGB cloud
  int components = image_data->GetNumberOfScalarComponents();

  int dimensions[3];
  image_data->GetDimensions (dimensions);

  double* pixel = new double [4];
  memset (pixel, 0, sizeof(double) * 4);

  if (mode.compare ("DEFAULT") == 0)
  {
    //
    // If the input image is a monochrome image the output cloud will be a pcl::PointCloud<pcl::Intensity>,
    // otherwise it will be a pcl::PointCloud<pcl::RGB>
    //

    PointCloud<Intensity> mono_cloud;
    PointCloud<RGB> color_cloud;

    int rgb;
    int rgba;

    switch (components)
    {
      case 1: mono_cloud.width = dimensions[0];
	      mono_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
	      mono_cloud.is_dense = true;
	      mono_cloud.points.resize (mono_cloud.width * mono_cloud.height);

	      for (int z = 0; z < dimensions[2]; z++)
	      {
		for (int y = 0; y < dimensions[1]; y++)
		{
		  for (int x = 0; x < dimensions[0]; x++)
		  {
		    pixel[0] = image_data->GetScalarComponentAsDouble(x, y, 0, 0);

		    Intensity gray;
		    gray.intensity = static_cast<uint8_t> (pixel[0]);

		    mono_cloud (x, dimensions[1] - y - 1) = gray;
		  }
		}
	      }

	      // Save the point cloud into a PCD file
	      saveCloud<Intensity> (argv[pcd_file_indices[0]], mono_cloud, format);
	      break;

      case 3: color_cloud.width = dimensions[0];
	      color_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
	      color_cloud.is_dense = true;
	      color_cloud.points.resize (color_cloud.width * color_cloud.height);

	      for (int z = 0; z < dimensions[2]; z++)
	      {
		for (int y = 0; y < dimensions[1]; y++)
		{
		  for (int x = 0; x < dimensions[0]; x++)
		  {
		    pixel[0] = image_data->GetScalarComponentAsDouble(x, y, 0, 0);
		    pixel[1] = image_data->GetScalarComponentAsDouble(x, y, 0, 1);
		    pixel[2] = image_data->GetScalarComponentAsDouble(x, y, 0, 2);

		    RGB color;
		    color.r = static_cast<uint8_t> (pixel[0]);
		    color.g = static_cast<uint8_t> (pixel[1]);
		    color.b = static_cast<uint8_t> (pixel[2]);

 		    rgb = (static_cast<int> (color.r)) << 16 |
 			  (static_cast<int> (color.g)) << 8 |
 			  (static_cast<int> (color.b));

  		    color.rgb = static_cast<float> (rgb);

		    color.rgba = static_cast<uint32_t> (rgb);

		    color_cloud (x, dimensions[1] - y - 1) = color;
		  }
		}
	      }

	      // Save the point cloud into a PCD file
	      saveCloud<RGB> (argv[pcd_file_indices[0]], color_cloud, format);
	      break;

      case 4: color_cloud.width = dimensions[0];
	      color_cloud.height = dimensions[1]; // This indicates that the point cloud is organized
	      color_cloud.is_dense = true;
	      color_cloud.points.resize (color_cloud.width * color_cloud.height);

	      for (int z = 0; z < dimensions[2]; z++)
	      {
		for (int y = 0; y < dimensions[1]; y++)
		{
		  for (int x = 0; x < dimensions[0]; x++)
		  {
		    pixel[0] = image_data->GetScalarComponentAsDouble(x, y, 0, 0);
		    pixel[1] = image_data->GetScalarComponentAsDouble(x, y, 0, 1);
		    pixel[2] = image_data->GetScalarComponentAsDouble(x, y, 0, 2);
		    pixel[3] = image_data->GetScalarComponentAsDouble(x, y, 0, 3);

		    RGB color;
		    color.r = static_cast<uint8_t> (pixel[0]);
		    color.g = static_cast<uint8_t> (pixel[1]);
		    color.b = static_cast<uint8_t> (pixel[2]);
		    color.a = static_cast<uint8_t> (pixel[3]);

 		    rgb = (static_cast<int> (color.r)) << 16 |
 			  (static_cast<int> (color.g)) << 8 |
 			  (static_cast<int> (color.b));
  		    rgba = (static_cast<int> (color.a)) << 24 |
  			   (static_cast<int> (color.r)) << 16 |
  			   (static_cast<int> (color.g)) << 8 |
  			   (static_cast<int> (color.b));

  		    color.rgb = static_cast<float> (rgb);
  		    color.rgba = static_cast<uint32_t> (rgba);

		    color_cloud (x, dimensions[1] - y - 1) = color;
		  }
		}
	      }

	      // Save the point cloud into a PCD file
	      saveCloud<RGB> (argv[pcd_file_indices[0]], color_cloud, format);
	      break;
    }
  }
  else if (mode.compare ("FORCE_COLOR") == 0)
  {
    //
    // Force the output cloud to be a pcl::PointCloud<pcl::RGB> even if the input image is a
    // monochrome image.
    //

    PointCloud<RGB> cloud;
    int dimensions[3];
    image_data->GetDimensions (dimensions);
    cloud.width = dimensions[0];
    cloud.height = dimensions[1]; // This indicates that the point cloud is organized
    cloud.is_dense = true;
    cloud.points.resize (cloud.width * cloud.height);

    for (int z = 0; z < dimensions[2]; z++)
    {
      for (int y = 0; y < dimensions[1]; y++)
      {
        for (int x = 0; x < dimensions[0]; x++)
        {
          for (int c = 0; c < components; c++)
            pixel[c] = image_data->GetScalarComponentAsDouble(x, y, 0, c);

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
	    case 1:  color.r = static_cast<uint8_t> (pixel[0]);
		     color.g = static_cast<uint8_t> (pixel[0]);
  		     color.b = static_cast<uint8_t> (pixel[0]);

  		     rgb = (static_cast<int> (color.r)) << 16 |
  			   (static_cast<int> (color.g)) << 8 |
  			   (static_cast<int> (color.b));

  		     rgba = rgb;
  		     color.rgb = static_cast<float> (rgb);
  		     color.rgba = static_cast<uint32_t> (rgba);
  		     break;

	    case 3:  color.r = static_cast<uint8_t> (pixel[0]);
  		     color.g = static_cast<uint8_t> (pixel[1]);
  		     color.b = static_cast<uint8_t> (pixel[2]);

  		     rgb = (static_cast<int> (color.r)) << 16 |
  			   (static_cast<int> (color.g)) << 8 |
  			   (static_cast<int> (color.b));

  		     rgba = rgb;
  		     color.rgb = static_cast<float> (rgb);
  		     color.rgba = static_cast<uint32_t> (rgba);
  		     break;

  	   case 4:  color.r = static_cast<uint8_t> (pixel[0]);
  		    color.g = static_cast<uint8_t> (pixel[1]);
  		    color.b = static_cast<uint8_t> (pixel[2]);
  		    color.a = static_cast<uint8_t> (pixel[3]);

 		    rgb = (static_cast<int> (color.r)) << 16 |
 			  (static_cast<int> (color.g)) << 8 |
 			  (static_cast<int> (color.b));
  		    rgba = (static_cast<int> (color.a)) << 24 |
  			   (static_cast<int> (color.r)) << 16 |
  			   (static_cast<int> (color.g)) << 8 |
  			   (static_cast<int> (color.b));

  		    color.rgb = static_cast<float> (rgb);
  		    color.rgba = static_cast<uint32_t> (rgba);
  		    break;
          }

  	  cloud (x, dimensions[1] - y - 1) = color;

        }
      }
    }

    // Save the point cloud into a PCD file
    saveCloud<RGB> (argv[pcd_file_indices[0]], cloud, format);
  }
  else if (mode.compare ("FORCE_GRAYSCALE") == 0)
  {
    //
    // Force the output cloud to be a pcl::PointCloud<pcl::Intensity> even if the input image is a
    // RGB or a RGBA image.
    //

    PointCloud<Intensity> cloud;
    cloud.width = dimensions[0];
    cloud.height = dimensions[1]; // This indicates that the point cloud is organized
    cloud.is_dense = true;
    cloud.points.resize (cloud.width * cloud.height);


    for (int z = 0; z < dimensions[2]; z++)
    {
      for (int y = 0; y < dimensions[1]; y++)
      {
        for (int x = 0; x < dimensions[0]; x++)
        {
          for (int c = 0; c < components; c++)
            pixel[c] = image_data->GetScalarComponentAsDouble(x, y, 0, c);

  	  Intensity gray;

  	  switch (components)
  	  {
  	    case 1:  gray.intensity = static_cast<uint8_t> (pixel[0]);
  		     break;

  	    case 3:  gray.intensity = static_cast<uint8_t> ( RED_MULTIPLIER * pixel[0] +
  						      GREEN_MULTIPLIER * pixel[1] +
  						      BLUE_MULTIPLIER * pixel[2] );
  		     break;

  	    case 4:  gray.intensity = static_cast<uint8_t> ( RED_MULTIPLIER * pixel[0] +
						      GREEN_MULTIPLIER * pixel[1] +
						      BLUE_MULTIPLIER * pixel[2] );
  		     break;
  	}

  	cloud (x, dimensions[1] - y - 1) = gray;
        }
      }
    }

    // Save the point cloud into a PCD file
    saveCloud<Intensity> (argv[pcd_file_indices[0]], cloud, format);
  }

  delete[] pixel;

  return 0;
}





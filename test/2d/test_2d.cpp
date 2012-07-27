/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: test_convolution.cpp nsomani $
 *
 */

#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkPNGReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageData.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkImageImport.h>
#include <vtkJPEGReader.h>
#include <vtkJPEGWriter.h>
#include <vtkImageFlip.h>
#include <vtkPNGWriter.h>
#include <vtkUnsignedCharArray.h>

#include <pcl/2d/convolution_2d.h>
#include <pcl/2d/edge.h>
#include <pcl/2d/keypoint.h>
#include <pcl/2d/morphology.h>

#include <vector>
#include <pcl/io/png_io.h>
#include <gtest/gtest.h>
#include <limits>
#include <fstream>

char *lena;
char *gaus;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
readPNGVector (char *fname, std::vector<pcl::pcl_2d::ImageType> &image)
{
  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New ();
  reader->SetFileName(fname);
  reader->Update();

  vtkSmartPointer<vtkImageData> data = vtkSmartPointer<vtkImageData>::New ();
  data = reader->GetOutput ();
  int dim[3];
  data->GetDimensions (dim);

  vtkUnsignedCharArray *image_data = vtkUnsignedCharArray::SafeDownCast (data->GetPointData ()->GetScalars ());
  vtkIdType tupleIndex = 0;
  unsigned char tuple[] = {0, 0, 0, 0};

  image.resize (dim[0]);
  for (int i = 0; i < dim[0]; i++)
  {
    image[i].resize (dim[1]);
    for (int j = 0; j < dim[1]; j++)
    {
      image[i][j].resize (4);
      float r = 0, g = 0, b = 0, a = 0;
      image_data->GetTupleValue (tupleIndex++, tuple);
      switch (image_data->GetNumberOfComponents ())
      {
        case 1:
          r = g = b = tuple[0];
          a = 255;
          break;
        case 2:
          r = g = b = tuple[0];
          a = tuple[1];
          break;
        case 3:
          r = tuple[0];
          g = tuple[1];
          b = tuple[2];
          a = 255;
          break;
        case 4:
          r = tuple[0];
          g = tuple[1];
          b = tuple[2];
          a = tuple[3];
          break;
      }
      image[i][j][0] = r;
      image[i][j][1] = g;
      image[i][j][2] = b;
      image[i][j][3] = a;
      //      fprintf(f, "%d\n", r);
    }
  }
}

TEST (Convolution, borderOptions)
{
  std::vector<pcl::pcl_2d::ImageType> image;
  readPNGVector (lena, image);

  pcl::pcl_2d::ImageType kernel;
  pcl::pcl_2d::ImageType output;
  pcl::pcl_2d::ImageType input;

  input.resize (image.size ());
  for (int i = 0; i < image.size (); i++)
  {
    input[i].resize (image[i].size ());
    for (int j = 0;j < image[i].size (); j++)
    {
      input[i][j] = image[i][j][0];
    }
  }
  pcl::pcl_2d::convolution_2d *conv = new pcl::pcl_2d::convolution_2d ();
  kernel.resize (3);
  for (int i = 0; i < 3; i++)
  {
    kernel[i].resize (3);
    for (int j = 0; j < 3; j++)
      kernel[i][j] = 0;
  }
  kernel[1][2] = 1;
  kernel[1][0] = -1;
  kernel[2][1] = 1;
  kernel[0][1] = -1;
  conv->convolve (output, kernel, input, pcl::pcl_2d::convolution_2d::BOUNDARY_OPTION_MIRROR);
  for (int i = 1; i < input.size () - 1; i++)
    for (int j = 1; j < input[i].size () - 1; j++)
      EXPECT_NEAR (output[i][j], (input[i][j+1]-input[i][j-1]+input[i+1][j]-input[i-1][j]), 1);

  for (int i = 1; i < input.size () - 1; i++)
  {
    EXPECT_NEAR (output[i][0], (input[i][1]-input[i][0]+input[i+1][0]-input[i-1][0]), 1);
    EXPECT_NEAR (output[i][input[i].size()-1], (input[i][input[i].size()-1]-input[i][input[i].size()-2]+input[i+1][input[i].size()-1]-input[i-1][input[i].size()-1]), 1);
  }
  for (int j = 1; j < input[0].size () - 1; j++)
  {
    EXPECT_NEAR (output[0][j], (input[1][j]-input[0][j]+input[0][j+1]-input[0][j-1]), 1);
    EXPECT_NEAR (output[input.size()-1][j], (input[input.size()-1][j]-input[input.size()-2][j]+input[input.size()-1][j+1]-input[input.size()-1][j-1]), 1);
  }
  conv->convolve (output, kernel, input, pcl::pcl_2d::convolution_2d::BOUNDARY_OPTION_CLAMP);
  
  for (int i = 1;i < input.size () - 1; i++)
    for (int j = 1;j < input[i].size () - 1; j++)
      EXPECT_NEAR (output[i][j], (input[i][j+1]-input[i][j-1]+input[i+1][j]-input[i-1][j]), 1);

  for (int i = 1; i < input.size () - 1; i++)
  {
    EXPECT_NEAR (output[i][0], (input[i][1]-input[i][0]+input[i+1][0]-input[i-1][0]), 1);
    EXPECT_NEAR (output[i][input[i].size()-1], (input[i][input[i].size()-1]-input[i][input[i].size()-2]+input[i+1][input[i].size()-1]-input[i-1][input[i].size()-1]), 1);
  }
  for (int j = 1; j < input[0].size () - 1; j++)
  {
    EXPECT_NEAR (output[0][j], (input[1][j]-input[0][j]+input[0][j+1]-input[0][j-1]), 1);
    EXPECT_NEAR (output[input.size()-1][j], (input[input.size()-1][j]-input[input.size()-2][j]+input[input.size()-1][j+1]-input[input.size()-1][j-1]), 1);
  }
  conv->convolve (output, kernel, input, pcl::pcl_2d::convolution_2d::BOUNDARY_OPTION_ZERO_PADDING);
  
  for (int i = 1; i < input.size () - 1; i++)
  {
    for (int j = 1; j < input[i].size () - 1; j++)
      EXPECT_NEAR (output[i][j], (input[i][j+1]-input[i][j-1]+input[i+1][j]-input[i-1][j]), 1);
  }
  for (int i = 1; i < input.size () - 1; i++)
  {
    EXPECT_NEAR (output[i][0], (input[i][1]+input[i+1][0]-input[i-1][0]), 1);
    EXPECT_NEAR (output[i][input[i].size()-1], (-input[i][input[i].size()-2]+input[i+1][input[i].size()-1]-input[i-1][input[i].size()-1]), 1);
  }
  for (int j = 1; j < input[0].size () - 1; j++)
  {
    EXPECT_NEAR (output[0][j], (input[1][j]+input[0][j+1]-input[0][j-1]), 1);
    EXPECT_NEAR (output[input.size()-1][j], (-input[input.size()-2][j]+input[input.size()-1][j+1]-input[input.size()-1][j-1]), 1);
  }
}

TEST (Convolution, gaussianSmooth)
{
  std::vector<pcl::pcl_2d::ImageType> image, gt_image;
  readPNGVector (lena, image);
  readPNGVector (gaus, gt_image);
  pcl::pcl_2d::ImageType kernel;
  pcl::pcl_2d::ImageType output;
  pcl::pcl_2d::ImageType input;
  pcl::pcl_2d::ImageType gt;

  input.resize (image.size ());
  gt.resize (gt_image.size ());
  for (int i = 0; i < image.size (); i++)
  {
    input[i].resize (image[i].size ());
    gt[i].resize (gt_image[i].size ());
    
    for (int j = 0; j < image[i].size (); j++)
    {
      input[i][j] = image[i][j][0];
      gt[i][j] = gt_image[i][j][0];
    }
  }
  pcl::pcl_2d::convolution_2d *conv = new pcl::pcl_2d::convolution_2d ();
  conv->gaussianSmooth (input, output, 3, 1);
  for(int i = 1; i < input.size () - 1; i++)
    for(int j = 1; j < input[i].size () - 1;j++)
      EXPECT_NEAR (output[i][j], gt[i][j], 1);
}

/** --[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  lena = argv[1]; //lena-grayscale.png
  gaus = argv[2]; //gauss_smooth.png
  return (RUN_ALL_TESTS ());
}
/* ]-- */


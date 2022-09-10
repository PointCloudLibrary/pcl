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
 * $Id: test_convolution.cpp nsomani $
 *
 */

#include <pcl/2d/convolution.h>
#include <pcl/2d/kernel.h>
#include <pcl/2d/edge.h>
#include <pcl/2d/morphology.h>

#include <pcl/test/gtest.h>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

char *lena;
char *gaus;
char *erosion_ref;
char *dilation_ref;
char *opening_ref;
char *closing_ref;
char *erosion_binary_ref;
char *dilation_binary_ref;
char *opening_binary_ref;
char *closing_binary_ref;
char *canny_ref;

using namespace pcl;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (Convolution, borderOptions)
{
  kernel<pcl::PointXYZI> k;
  Convolution<pcl::PointXYZI> conv;

  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr kernel_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile(lena, *input_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  k.setKernelType(kernel<pcl::PointXYZI>::DERIVATIVE_CENTRAL_X);
  k.fetchKernel (*kernel_cloud);

  conv.setKernel(*kernel_cloud);
  conv.setInputCloud(input_cloud);

  conv.setBoundaryOptions(Convolution<pcl::PointXYZI>::BOUNDARY_OPTION_MIRROR);
  conv.filter (*output_cloud);
  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, ((*input_cloud)(j+1,i).intensity-(*input_cloud)(j-1,i).intensity), 1);

  for (int i = 1; i < height - 1; i++)
  {
    EXPECT_NEAR ((*output_cloud)(0,i).intensity, ((*input_cloud)(1,i).intensity-(*input_cloud)(0,i).intensity), 1);
    EXPECT_NEAR ((*output_cloud)(width-1,i).intensity, ((*input_cloud)(width-1,i).intensity-(*input_cloud)(width-2,i).intensity), 1);
  }

  conv.setBoundaryOptions(Convolution<pcl::PointXYZI>::BOUNDARY_OPTION_CLAMP);
  conv.filter (*output_cloud);
  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, ((*input_cloud)(j+1,i).intensity-(*input_cloud)(j-1,i).intensity), 1);

  for (int i = 1; i < height - 1; i++)
  {
    EXPECT_NEAR ((*output_cloud)(0,i).intensity, ((*input_cloud)(1,i).intensity-(*input_cloud)(0,i).intensity), 1);
  //  EXPECT_NEAR ((*output_cloud)(width-1,i).intensity, ((*input_cloud)(width-1,i).intensity-(*input_cloud)(width-2,i).intensity), 1);
  }

  conv.setBoundaryOptions(Convolution<pcl::PointXYZI>::BOUNDARY_OPTION_ZERO_PADDING);
  conv.filter (*output_cloud);
  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, ((*input_cloud)(j+1,i).intensity-(*input_cloud)(j-1,i).intensity), 1);

  for (int i = 1; i < height - 1; i++)
  {
    EXPECT_NEAR ((*output_cloud)(0,i).intensity, ((*input_cloud)(1,i).intensity), 1);
    EXPECT_NEAR ((*output_cloud)(width-1,i).intensity, (-(*input_cloud)(width-2,i).intensity), 1);
  }
}

TEST (Convolution, gaussianSmooth)
{
  kernel<pcl::PointXYZI> k;
  Convolution<pcl::PointXYZI> conv;

  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gt_output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr kernel_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile(lena, *input_cloud);
  pcl::io::loadPCDFile(gaus, *gt_output_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  k.setKernelType(kernel<pcl::PointXYZI>::GAUSSIAN);
  k.setKernelSize(3);
  k.setKernelSigma(1.0f);
  k.fetchKernel (*kernel_cloud);

  conv.setKernel(*kernel_cloud);
  conv.setInputCloud(input_cloud);

  conv.setBoundaryOptions(Convolution<pcl::PointXYZI>::BOUNDARY_OPTION_MIRROR);
  conv.filter (*output_cloud);
  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity, 1);
}

TEST(Edge, sobel)
{
  Edge<pcl::PointXYZI, PointXYZIEdge>::Ptr edge_ (new Edge<pcl::PointXYZI, PointXYZIEdge> ());

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<PointXYZIEdge>::Ptr output_cloud (new pcl::PointCloud<PointXYZIEdge>);

  pcl::io::loadPCDFile(lena, *input_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  edge_->setInputCloud(input_cloud);
  edge_->setOutputType (Edge<pcl::PointXYZI, PointXYZIEdge>::OUTPUT_X_Y);
  edge_->detectEdgeSobel (*output_cloud);

  float gt_x, gt_y;
  for (int i = 1; i < height - 1; i++){
    for (int j = 1; j < width - 1; j++){
      gt_x = (*input_cloud)(j+1,i-1).intensity - (*input_cloud)(j-1,i-1).intensity +
          2*(*input_cloud)(j+1,i).intensity - 2*(*input_cloud)(j-1,i).intensity +
          (*input_cloud)(j+1,i+1).intensity - (*input_cloud)(j-1,i+1).intensity;
      EXPECT_NEAR ((*output_cloud)(j,i).magnitude_x, gt_x, 1);

      gt_y = (*input_cloud)(j-1,i+1).intensity - (*input_cloud)(j-1,i-1).intensity +
          2*(*input_cloud)(j,i+1).intensity - 2*(*input_cloud)(j,i-1).intensity +
          (*input_cloud)(j+1,i+1).intensity - (*input_cloud)(j+1,i-1).intensity;
      EXPECT_NEAR ((*output_cloud)(j,i).magnitude_y, gt_y, 1);
    }
  }
}

TEST(Edge, prewitt)
{
  Edge<pcl::PointXYZI, PointXYZIEdge>::Ptr edge_ (new Edge<pcl::PointXYZI, PointXYZIEdge> ());

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<PointXYZIEdge>::Ptr output_cloud (new pcl::PointCloud<PointXYZIEdge>);

  pcl::io::loadPCDFile(lena, *input_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  edge_->setInputCloud(input_cloud);
  edge_->setOutputType (Edge<pcl::PointXYZI, PointXYZIEdge>::OUTPUT_X_Y);
  edge_->detectEdgePrewitt (*output_cloud);

  float gt_x, gt_y;
  for (int i = 1; i < height - 1; i++){
    for (int j = 1; j < width - 1; j++){
      gt_x = (*input_cloud)(j+1,i-1).intensity - (*input_cloud)(j-1,i-1).intensity +
          (*input_cloud)(j+1,i).intensity - (*input_cloud)(j-1,i).intensity +
          (*input_cloud)(j+1,i+1).intensity - (*input_cloud)(j-1,i+1).intensity;
      EXPECT_NEAR ((*output_cloud)(j,i).magnitude_x, gt_x, 1);

      gt_y = (*input_cloud)(j-1,i-1).intensity - (*input_cloud)(j-1,i+1).intensity +
          (*input_cloud)(j,i-1).intensity - (*input_cloud)(j,i+1).intensity +
          (*input_cloud)(j+1,i-1).intensity - (*input_cloud)(j+1,i+1).intensity;
      EXPECT_NEAR ((*output_cloud)(j,i).magnitude_y, gt_y, 1);
    }
  }
}

TEST(Edge, canny)
{
  Edge<pcl::PointXYZI, PointXYZIEdge>::Ptr edge_ (new Edge<pcl::PointXYZI, PointXYZIEdge> ());

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gt_output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<PointXYZIEdge>::Ptr output_cloud (new pcl::PointCloud<PointXYZIEdge>);

  pcl::io::loadPCDFile(lena, *input_cloud);
  //pcl::io::loadPCDFile("canny.pcd", *gt_output_cloud);
  pcl::io::loadPCDFile(canny_ref, *gt_output_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  edge_->setInputCloud(input_cloud);
  edge_->setOutputType (Edge<pcl::PointXYZI, PointXYZIEdge>::OUTPUT_X_Y);
  edge_->setHysteresisThresholdLow (10);
  edge_->setHysteresisThresholdHigh (50);
  edge_->detectEdgeCanny (*output_cloud);

  for (int i = 1; i < height - 1; i++)
  {
    for (int j = 1; j < width - 1; j++)
    {
      EXPECT_NEAR ((*output_cloud)(j,i).magnitude, (*gt_output_cloud)(j,i).intensity, 255);
    }
  }
}

void threshold(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float thresh){
  for(std::size_t i = 0;i < cloud->height;i++){
    for(std::size_t j = 0;j < cloud->width;j++){
      if((*cloud)(j,i).intensity > thresh)
        (*cloud)(j,i).intensity = 1;
      else
        (*cloud)(j,i).intensity = 0;
    }
  }
}

TEST(Morphology, erosion)
{
  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gt_output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr structuring_element_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile(lena, *input_cloud);
  pcl::io::loadPCDFile(erosion_ref, *gt_output_cloud);

  Morphology<pcl::PointXYZI> morph;
  morph.setInputCloud(input_cloud);
  morph.structuringElementRectangle(*structuring_element_cloud, 3, 3);
  morph.setStructuringElement(structuring_element_cloud);
  morph.erosionGray(*output_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity, 1);

  pcl::io::loadPCDFile(erosion_binary_ref, *gt_output_cloud);
  threshold(input_cloud, 100);
  morph.setInputCloud(input_cloud);
  morph.erosionBinary(*output_cloud);
  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity/255.0, 1);
}

TEST(Morphology, dilation)
{
  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gt_output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr structuring_element_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile(lena, *input_cloud);
  pcl::io::loadPCDFile(dilation_ref, *gt_output_cloud);

  Morphology<pcl::PointXYZI> morph;
  morph.setInputCloud(input_cloud);
  morph.structuringElementRectangle(*structuring_element_cloud, 3, 3);
  morph.setStructuringElement(structuring_element_cloud);
  morph.dilationGray(*output_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity, 1);

  pcl::io::loadPCDFile(dilation_binary_ref, *gt_output_cloud);
  threshold(input_cloud, 100);
  morph.setInputCloud(input_cloud);
  morph.dilationBinary(*output_cloud);
  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity/255.0, 1);
}

TEST(Morphology, opening)
{
  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gt_output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr structuring_element_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile(lena, *input_cloud);
  pcl::io::loadPCDFile(opening_ref, *gt_output_cloud);

  Morphology<pcl::PointXYZI> morph;
  morph.setInputCloud(input_cloud);
  morph.structuringElementRectangle(*structuring_element_cloud, 3, 3);
  morph.setStructuringElement(structuring_element_cloud);
  morph.openingGray(*output_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity, 1);

  pcl::io::loadPCDFile(opening_binary_ref, *gt_output_cloud);
  threshold(input_cloud, 100);
  morph.setInputCloud(input_cloud);
  morph.openingBinary(*output_cloud);
  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity/255.0, 1);
}

TEST(Morphology, closing)
{
  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gt_output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr structuring_element_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile(lena, *input_cloud);
  pcl::io::loadPCDFile(closing_ref, *gt_output_cloud);

  Morphology<pcl::PointXYZI> morph;
  morph.setInputCloud(input_cloud);
  morph.structuringElementRectangle(*structuring_element_cloud, 3, 3);
  morph.setStructuringElement(structuring_element_cloud);
  morph.closingGray(*output_cloud);

  int height = input_cloud->height;
  int width = input_cloud->width;

  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity, 1);

  pcl::io::loadPCDFile(closing_binary_ref, *gt_output_cloud);
  threshold(input_cloud, 100);
  morph.setInputCloud(input_cloud);
  morph.closingBinary(*output_cloud);
  for (int i = 1; i < height - 1; i++)
    for (int j = 1; j < width - 1; j++)
      EXPECT_NEAR ((*output_cloud)(j,i).intensity, (*gt_output_cloud)(j,i).intensity/255.0, 1);
}

/** --[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  lena = argv[1]; //lena.pcd
  gaus = argv[2]; //gauss_smooth.pcd
  erosion_ref = argv[3];
  dilation_ref = argv[4];
  opening_ref = argv[5];
  closing_ref = argv[6];
  erosion_binary_ref = argv[7];
  dilation_binary_ref = argv[8];
  opening_binary_ref = argv[9];
  closing_binary_ref = argv[10];
  canny_ref = argv[11];
  return (RUN_ALL_TESTS ());
}
/* ]-- */


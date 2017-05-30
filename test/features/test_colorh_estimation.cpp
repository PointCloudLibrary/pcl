/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2014, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/colorh.h>
#include <pcl/features/colorh_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>


using namespace pcl;
using namespace pcl::io;
using namespace std;

PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB> ());
vector<int> indices;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ColorHEstimation)
{
  
  // ColorHEstimation
  typedef Histogram<90> ColorHistogram;
  ColorHEstimation<PointXYZRGB, ColorHistogram> ch_rgb;
  PointCloud<ColorHistogram>::Ptr hist_rgb (new PointCloud<ColorHistogram> ());
  ch_rgb.setInputCloud (cloud);
  ch_rgb.computeFeature (*hist_rgb);
  
  ColorHEstimation<PointXYZRGB, ColorHistogram> ch_hsv;
  PointCloud<ColorHistogram>::Ptr hist_hsv (new PointCloud<ColorHistogram> ());
  ch_hsv.setInputCloud (cloud);
  ch_hsv.setHSVHistogram (true);
  ch_hsv.computeFeature (*hist_hsv);
  
  ColorHEstimation<PointXYZRGB, ColorHistogram> ch_yuv;
  PointCloud<ColorHistogram>::Ptr hist_yuv (new PointCloud<ColorHistogram> ());
  ch_yuv.setInputCloud (cloud);
  ch_yuv.setYUVHistogram (true);
  ch_yuv.computeFeature (*hist_yuv);
 

  const double correct_rgb_hist[90] = {0.00445126, 0.0225482, 0.0516637, 0.0421775, 0.0283859, 0.0442207, 0.0518097, 0.0827496, 0.0847198, 0.0771308, 0.0972709, 0.109603, 0.0532691, 0.028094, 0.0262697, 0.0261238, 0.0171483, 0.0299183, 0.028094, 0.0143754, 0.0140105, 0.0114565, 0.0105809, 0.00758902, 0.00758902, 0.00649445, 0.00934034, 0.007662, 0.00386748, 0.00138646, 0.0056188, 0.00569177, 0.00773497, 0.017951, 0.0269994, 0.0687391, 0.102817, 0.101941, 0.0834793, 0.076401, 0.144775, 0.0825306, 0.0297723, 0.0191185, 0.0167104, 0.0322534, 0.0446585, 0.023059, 0.0172212, 0.0148132, 0.0131349, 0.0136457, 0.01277, 0.0112376, 0.0109457, 0.00642148, 0.00445126, 0.00218914, 0.0015324, 0.00138646, 0.0672796, 0.0776416, 0.0467747, 0.040864, 0.0246643, 0.0327642, 0.0259048, 0.0269994, 0.0220374, 0.0338587, 0.0645067, 0.114711, 0.0915061, 0.06385, 0.0283129, 0.0182428, 0.0238616, 0.0277291, 0.0293345, 0.0150321, 0.0109457, 0.0117484, 0.0116754, 0.0107998, 0.015324, 0.0129159, 0.0130619, 0.0091944, 0.0117484, 0.0167104};
  const double correct_hsv_hist[90] = {0.0313047, 0.0474314, 0.0379451, 0.0507151, 0.068812, 0.122884, 0.0201401, 0.00693228, 0.00430531, 0.00211617, 0.00372154, 0.00321074, 0.00386748, 0.0040864, 0.00372154, 0.0113106, 0.0210158, 0.12033, 0.133975, 0.0556772, 0.0576474, 0.0373614, 0.0282399, 0.0219644, 0.0146673, 0.02284, 0.0154699, 0.0168564, 0.0175131, 0.0139375, 0.0195563, 0.0631203, 0.0836252, 0.0732633, 0.0572825, 0.0483071, 0.040937, 0.0350263, 0.0337858, 0.0278751, 0.0287507, 0.0264886, 0.0275102, 0.020432, 0.0183888, 0.0200671, 0.017878, 0.0175131, 0.0285318, 0.0208698, 0.0215995, 0.0260508, 0.0394046, 0.0453882, 0.0422504, 0.0348803, 0.0267805, 0.0212347, 0.0172212, 0.0145213, 0.00197023, 0.00218914, 0.0045972, 0.00846468, 0.0154699, 0.0186807, 0.0336398, 0.0578663, 0.086763, 0.0582312, 0.081728, 0.122738, 0.137259, 0.0707823, 0.0495476, 0.0141565, 0.0113835, 0.0248103, 0.0294075, 0.0148862, 0.00875657, 0.0122592, 0.0163456, 0.0123322, 0.0185347, 0.0166375, 0.0189726, 0.0169294, 0.0148132, 0.0197752};
  const double correct_yuv_hist[90] = {0.00394046, 0.00415937, 0.00985114, 0.0185347, 0.0300642, 0.0887332, 0.105079, 0.117192, 0.0675715, 0.0633392, 0.120841, 0.0961763, 0.0290426, 0.0190455, 0.0143754, 0.0294804, 0.0354641, 0.0269264, 0.0191185, 0.0170753, 0.0171483, 0.0185347, 0.01277, 0.0134997, 0.0086836, 0.00591068, 0.00364857, 0.00233508, 0.00116754, 0.000291886, 7.29714e-05, 0.000218914, 0.000364857, 0.000729714, 0.000583771, 0.000437828, 0.000145943, 0, 0.000291886, 0.0086836, 0.0269264, 0.096979, 0.0828225, 0.0885873, 0.124051, 0.245038, 0.124635, 0.0658932, 0.0826036, 0.0159807, 0.0126241, 0.0186807, 0.00350263, 0.000145943, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7.29714e-05, 0.00109457, 0.0113835, 0.0248103, 0.0933304, 0.094352, 0.225263, 0.423818, 0.0472125, 0.03561, 0.0323263, 0.010289, 0.000364857, 7.29714e-05, 0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 90; i++)
  {
  	EXPECT_NEAR(hist_rgb->points[0].histogram[i], correct_rgb_hist[i], 1e-4);
  	EXPECT_NEAR(hist_hsv->points[0].histogram[i], correct_hsv_hist[i], 1e-4);
  	EXPECT_NEAR(hist_yuv->points[0].histogram[i], correct_yuv_hist[i], 1e-4);
  }	

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, ColorHEstimation3D)
{
  
  // ColorHEstimation
  typedef Histogram<64> ColorHistogram3D;
  ColorHEstimation3D<PointXYZRGB, ColorHistogram3D> ch_rgb;
  PointCloud<ColorHistogram3D>::Ptr hist_rgb (new PointCloud<ColorHistogram3D> ());
  ch_rgb.setInputCloud (cloud);
  ch_rgb.computeFeature (*hist_rgb);
  
  ColorHEstimation3D<PointXYZRGB, ColorHistogram3D> ch_hsv;
  PointCloud<ColorHistogram3D>::Ptr hist_hsv (new PointCloud<ColorHistogram3D> ());
  ch_hsv.setInputCloud (cloud);
  ch_hsv.setHSVHistogram (true);
  ch_hsv.computeFeature (*hist_hsv);
  
  ColorHEstimation3D<PointXYZRGB, ColorHistogram3D> ch_yuv;
  PointCloud<ColorHistogram3D>::Ptr hist_yuv (new PointCloud<ColorHistogram3D> ());
  ch_yuv.setInputCloud (cloud);
  ch_yuv.setYUVHistogram (true);
  ch_yuv.computeFeature (*hist_yuv);
 
  const double correct_rgb_hist[64] = {0.106465, 0.110552, 0.000145943, 0, 0.017951, 0.0379451, 0.000656743, 0.00401343, 0, 0, 0, 0.000145943, 0, 0, 0, 0, 0.0563339, 0.00634851, 7.29714e-05, 0, 0.142075, 0.270797, 0.0176591, 0.00882954, 0.000145943, 0.000291886, 0.00437828, 0.0194834, 7.29714e-05, 0, 0, 0, 0, 0, 0, 0, 0.00284588, 0.00248103, 0.00445126, 0.000364857, 0.000583771, 0.00350263, 0.0877846, 0.0275832, 0.00138646, 0, 0.00233508, 0.0129159, 0, 0, 0, 0, 7.29714e-05, 0, 0.000218914, 0, 0, 0.00124051, 0.0159807, 0.00284588, 0.000364857, 0.000218914, 0.00861062, 0.0198482};
  const double correct_hsv_hist[64] = {0.0061296, 0.0513719, 0.0110917, 0.012697, 0.0112376, 0.0402072, 0.00488908, 0.0158348, 0.0195563, 0.0553123, 0.0015324, 0.000437828, 0.0343695, 0.113762, 0.00218914, 0.000729714, 0.00364857, 0.0129159, 0.00277291, 0.00415937, 0.00262697, 0.00226211, 0.000145943, 7.29714e-05, 0.000875657, 0.000218914, 0, 0, 0.000145943, 0, 0, 0, 0.00299183, 0.118433, 0.0424694, 0.0234238, 0.00481611, 0.0616608, 0.0124051, 0.0269994, 0.00554583, 0.0515908, 0.00167834, 0.0275832, 0.00372154, 0.0659661, 0.000145943, 0.0020432, 0.00423234, 0.0572096, 0.043418, 0.00707823, 0.00401343, 0.00758902, 0.00218914, 0.00379451, 0.00248103, 0.00313777, 7.29714e-05, 0.000145943, 0.00160537, 0.000364857, 0, 0};
  const double correct_yuv_hist[64] = {0, 0, 0, 0, 0, 0.0210887, 0.144046, 0, 0, 0.127189, 0.0208698, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.020359, 0.186807, 0, 0, 0.178999, 0.0877846, 0, 0, 0.000802685, 0, 0, 0, 0.00182428, 0.000437828, 0, 0, 0.00379451, 0.0329101, 0, 0, 0.0726065, 0.0575015, 0, 0, 0, 0, 0, 0, 7.29714e-05, 0.000218914, 0, 0, 0.00467017, 0.0146673, 0, 0, 0.0188996, 0.00445126, 0, 0, 0, 0, 0};
  for (int i = 0; i < 64; i++)
  {
  	EXPECT_NEAR(hist_rgb->points[0].histogram[i], correct_rgb_hist[i], 1e-4);
  	EXPECT_NEAR(hist_hsv->points[0].histogram[i], correct_hsv_hist[i], 1e-4);
  	EXPECT_NEAR(hist_yuv->points[0].histogram[i], correct_yuv_hist[i], 1e-4);
  }


}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `milk_color.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZRGB> (argv[1], *cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `milk_color.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  std::cout << "Loaded the cloud" <<std::endl;
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

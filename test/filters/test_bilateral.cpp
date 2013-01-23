/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id: test_filters.cpp 7683 2012-10-23 02:49:03Z rusu $
 *
 */

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/console/time.h>

using namespace pcl;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (FastBilateralFilter, Filters_Bilateral)
{
  FastBilateralFilter<PointXYZ> fbf;
  fbf.setInputCloud (cloud);
  fbf.setSigmaS (5);
  fbf.setSigmaR (0.03f);
  PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ> ());
  fbf.filter (*cloud_filtered);

  Eigen::Vector3f p_65558 (-0.058448f, -0.189095f, 0.723415f),
      p_84737 (-0.088929f, -0.152957f, 0.746095f),
      p_57966 (0.123646f, -0.397528f, 1.393187f),
      p_39543 (0.560287f, -0.545020f, 1.602833f),
      p_17766 (0.557854f, -0.711976f, 1.762013f),
      p_70202 (0.150500f, -0.160329f, 0.646596f),
      p_102219 (0.175637f, -0.101353f, 0.661631f),
      p_81765 (0.223189f, -0.151714f, 0.708332f);

  for (size_t dim = 0; dim < 3; ++dim)
  {
    EXPECT_NEAR (p_84737[dim], (*cloud_filtered)[84737].getVector3fMap ()[dim], 1e-3);
    EXPECT_NEAR (p_57966[dim], (*cloud_filtered)[57966].getVector3fMap ()[dim], 1e-3);
    EXPECT_NEAR (p_39543[dim], (*cloud_filtered)[39543].getVector3fMap ()[dim], 1e-3);
    EXPECT_NEAR (p_17766[dim], (*cloud_filtered)[17766].getVector3fMap ()[dim], 1e-3);
    EXPECT_NEAR (p_70202[dim], (*cloud_filtered)[70202].getVector3fMap ()[dim], 1e-3);
    EXPECT_NEAR (p_102219[dim], (*cloud_filtered)[102219].getVector3fMap ()[dim], 1e-3);
    EXPECT_NEAR (p_81765[dim], (*cloud_filtered)[81765].getVector3fMap ()[dim], 1e-3);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (FastBilateralFilterOMP, Filters_Bilateral)
{
  std::vector<float> sigma_s; 
  sigma_s.push_back (2.341f);
  sigma_s.push_back (5.2342f);
  sigma_s.push_back (10.29380f);
  std::vector<float> sigma_r; 
  sigma_r.push_back (0.0123f);
  sigma_r.push_back (0.023f);
  sigma_r.push_back (0.0345f);
  pcl::console::TicToc tt;
  for (size_t i = 0; i < 3; i++)
  {
    FastBilateralFilter<PointXYZ> fbf;
    fbf.setInputCloud (cloud);
    fbf.setSigmaS (sigma_s[i]);
    fbf.setSigmaR (sigma_r[i]);
    PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ> ());
    tt.tic ();
    fbf.filter (*cloud_filtered);
    PCL_INFO ("[FastBilateralFilter] filtering took %f ms\n", tt.toc ());

    FastBilateralFilterOMP<PointXYZ> fbf_omp (0);
    fbf_omp.setInputCloud (cloud);
    fbf_omp.setSigmaS (sigma_s[i]);
    fbf_omp.setSigmaR (sigma_r[i]);
    PointCloud<PointXYZ>::Ptr cloud_filtered_omp (new PointCloud<PointXYZ> ());
    tt.tic ();
    fbf_omp.filter (*cloud_filtered_omp);
    PCL_INFO ("[FastBilateralFilterOMP] filtering took %f ms\n", tt.toc ());

    EXPECT_EQ (cloud_filtered_omp->points.size (), cloud_filtered->points.size ());
    for (size_t j = 0; j < cloud_filtered_omp->size (); ++j)
    {
      if (pcl_isnan (cloud_filtered_omp->at (j).x))
        EXPECT_TRUE (pcl_isnan (cloud_filtered->at (j).x));
      else
      {
        EXPECT_NEAR (cloud_filtered_omp->at (j).x, cloud_filtered->at (j).x, 1e-3);
        EXPECT_NEAR (cloud_filtered_omp->at (j).y, cloud_filtered->at (j).y, 1e-3);
        EXPECT_NEAR (cloud_filtered_omp->at (j).z, cloud_filtered->at (j).z, 1e-3);
      }
    }
  }

}

/* ---[ */
int
main (int argc,
      char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `milk_cartoon_all_small_clorox.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  char* file_name = argv[1];
  // Load a standard PCD file from disk
  io::loadPCDFile (file_name, *cloud);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

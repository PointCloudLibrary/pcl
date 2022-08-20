/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <pcl/test/gtest.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

#include <iostream>

using namespace pcl;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;
PointCloud<PointXYZ> cloud;
KdTreePtr tree;

NormalEstimation<PointXYZ, Normal> n;
IntegralImageNormalEstimation<PointXYZ, Normal> ne;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, IntegralImage1D)
{
  const unsigned width = 640;
  const unsigned height = 480;
  const unsigned max_window_size = 5;
  const unsigned min_window_size = 1;
  IntegralImage2D<float,1> integral_image1(true); // calculate second order
  IntegralImage2D<float,1> integral_image2(false);// calculate just first order (other if branch)

  // test for dense data with element stride = 1
  float* data = new float[width * height];
  for(unsigned yIdx = 0; yIdx < height; ++yIdx)
  {
    for(unsigned xIdx = 0; xIdx < width; ++xIdx)
    {
      data[width * yIdx + xIdx] = static_cast<float> (xIdx);
    }
  }

  // calculate integral images
  integral_image1.setInput (data, width, height, 1, width);
  integral_image2.setInput (data, width, height, 1, width);

  // check results
  for (unsigned window_width = min_window_size; window_width < max_window_size; ++window_width)
  {
    for (unsigned window_height = min_window_size; window_height < max_window_size; ++window_height)
    {
      for(unsigned yIdx = 0; yIdx < height - window_height; ++yIdx)
      {
        for(unsigned xIdx = 0; xIdx < width - window_width; ++xIdx)
        {
          //std::cout << xIdx << " : " << yIdx << " - " << window_width << " x " << window_height << " :: " << integral_image1.getFirstOrderSum (xIdx, yIdx, window_width, window_height) * 2 << std::endl;
          EXPECT_EQ (window_height * window_width * (window_width + 2 * xIdx - 1), integral_image1.getFirstOrderSum (xIdx, yIdx, window_width, window_height) * 2);
          EXPECT_EQ (window_height * window_width * (window_width + 2 * xIdx - 1), integral_image2.getFirstOrderSum (xIdx, yIdx, window_width, window_height) * 2);
          EXPECT_EQ (window_height * window_width, integral_image1.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_height * window_width, integral_image2.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));

          int w = window_width + xIdx - 1;
          long result = w * (w + 1) * (2*w + 1) - xIdx * (xIdx - 1) * (2*xIdx - 1);
          EXPECT_EQ (window_height * result, integral_image1.getSecondOrderSum (xIdx, yIdx, window_width, window_height) * 6);
        }
      }
    }
  }
  delete[] data;

  //now test with element-stride 2 and modulo even row stride (no padding)
  unsigned element_stride = 2;
  unsigned row_stride = width * element_stride;
  data = new float[row_stride * height];
  for(unsigned yIdx = 0; yIdx < height; ++yIdx)
  {
    for(unsigned xIdx = 0; xIdx < row_stride; xIdx += element_stride)
    {
      data[row_stride * yIdx + xIdx] = static_cast<float> (xIdx >> 1);
      data[row_stride * yIdx + xIdx + 1] = -1;
    }
  }
  integral_image1.setInput (data, width, height, element_stride, row_stride);
  integral_image2.setInput (data, width, height, element_stride, row_stride);
  for (unsigned window_width = min_window_size; window_width < max_window_size; ++window_width)
  {
    for (unsigned window_height = min_window_size; window_height < max_window_size; ++window_height)
    {
      for(unsigned yIdx = 0; yIdx < height - window_height; ++yIdx)
      {
        for(unsigned xIdx = 0; xIdx < width - window_width; ++xIdx)
        {
          EXPECT_EQ (window_height * window_width * (window_width + 2 * xIdx - 1), integral_image1.getFirstOrderSum (xIdx, yIdx, window_width, window_height) * 2);
          EXPECT_EQ (window_height * window_width * (window_width + 2 * xIdx - 1), integral_image2.getFirstOrderSum (xIdx, yIdx, window_width, window_height) * 2);
          EXPECT_EQ (window_height * window_width, integral_image1.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_height * window_width, integral_image2.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));

          int w = window_width + xIdx - 1;
          long result = w * (w + 1) * (2*w + 1) - xIdx * (xIdx - 1) * (2*xIdx - 1);
          EXPECT_EQ (window_height * result, integral_image1.getSecondOrderSum (xIdx, yIdx, window_width, window_height) * 6);
        }
      }
    }
  }
  delete[] data;

  //now test with odd element-stride 3 and modulo-uneven row_stride
  element_stride = 3;
  row_stride = width * element_stride + 1; // +1 to enforce a non-zero modulo
  data = new float[row_stride * height];
  for (unsigned yIdx = 0; yIdx < height; ++yIdx)
  {
    for (unsigned xIdx = 0; xIdx < width; ++xIdx)
    {
      data[row_stride * yIdx + element_stride * xIdx] = 1.0f;
      data[row_stride * yIdx + element_stride * xIdx + 1] = 2.0f;
      data[row_stride * yIdx + element_stride * xIdx + 2] = static_cast<float> (xIdx);
    }
  }
  integral_image1.setInput (data, width, height, element_stride, row_stride);
  integral_image2.setInput (data, width, height, element_stride, row_stride);
  for (unsigned window_width = min_window_size; window_width < max_window_size; ++window_width)
  {
    for (unsigned window_height = min_window_size; window_height < max_window_size; ++window_height)
    {
      for(unsigned yIdx = 0; yIdx < height - window_height; ++yIdx)
      {
        for(unsigned xIdx = 0; xIdx < width - window_width; ++xIdx)
        {
          EXPECT_EQ (window_width * window_height, integral_image1.getFirstOrderSum (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_width * window_height, integral_image2.getFirstOrderSum (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_width * window_height, integral_image1.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_width * window_height, integral_image2.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));

          EXPECT_EQ (window_width * window_height, integral_image1.getSecondOrderSum (xIdx, yIdx, window_width, window_height));
        }
      }
    }
  }
  //check for second channel
  integral_image1.setInput (data + 1, width, height, element_stride, row_stride);
  integral_image2.setInput (data + 1, width, height, element_stride, row_stride);
  for (unsigned window_width = min_window_size; window_width < max_window_size; ++window_width)
  {
    for (unsigned window_height = min_window_size; window_height < max_window_size; ++window_height)
    {
      for(unsigned yIdx = 0; yIdx < height - window_height; ++yIdx)
      {
        for(unsigned xIdx = 0; xIdx < width - window_width; ++xIdx)
        {
          EXPECT_EQ (window_width * window_height * 2, integral_image1.getFirstOrderSum (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_width * window_height * 2, integral_image2.getFirstOrderSum (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_width * window_height, integral_image1.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_width * window_height, integral_image2.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));

          EXPECT_EQ (window_width * window_height * 4, integral_image1.getSecondOrderSum (xIdx, yIdx, window_width, window_height));
        }
      }
    }
  }

  //check for third channel
  integral_image1.setInput (data + 2, width, height, element_stride, row_stride);
  integral_image2.setInput (data + 2, width, height, element_stride, row_stride);
  for (unsigned window_width = min_window_size; window_width < max_window_size; ++window_width)
  {
    for (unsigned window_height = min_window_size; window_height < max_window_size; ++window_height)
    {
      for(unsigned yIdx = 0; yIdx < height - window_height; ++yIdx)
      {
        for(unsigned xIdx = 0; xIdx < width - window_width; ++xIdx)
        {
          EXPECT_EQ (window_height * window_width * (window_width + 2 * xIdx - 1), integral_image1.getFirstOrderSum (xIdx, yIdx, window_width, window_height) * 2);
          EXPECT_EQ (window_height * window_width * (window_width + 2 * xIdx - 1), integral_image2.getFirstOrderSum (xIdx, yIdx, window_width, window_height) * 2);
          EXPECT_EQ (window_width * window_height, integral_image1.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (window_width * window_height, integral_image2.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));

          int w = window_width + xIdx - 1;
          long result = w * (w + 1) * (2*w + 1) - xIdx * (xIdx - 1) * (2*xIdx - 1);
          EXPECT_EQ (window_height * result, integral_image1.getSecondOrderSum (xIdx, yIdx, window_width, window_height) * 6);
        }
      }
    }
  }
  delete[] data;

  //now test whether count of non finite elements are correct
  element_stride = 3;
  row_stride = width * element_stride + 1; // +1 to enforce a non-zero modulo
  data = new float[row_stride * height];
  for(unsigned yIdx = 0; yIdx < height; ++yIdx)
  {
    for(unsigned xIdx = 0; xIdx < width; ++xIdx)
    {
      // set nans for odd fields
      if (xIdx & 1)
      {
        data[row_stride * yIdx + element_stride * xIdx]     = std::numeric_limits<float>::quiet_NaN ();
        data[row_stride * yIdx + element_stride * xIdx + 1] = std::numeric_limits<float>::quiet_NaN ();
        data[row_stride * yIdx + element_stride * xIdx + 2] = std::numeric_limits<float>::quiet_NaN ();
      }
      else
      {
        data[row_stride * yIdx + element_stride * xIdx] = 3.0f;
        data[row_stride * yIdx + element_stride * xIdx + 1] = 2.0f;
        data[row_stride * yIdx + element_stride * xIdx + 2] = static_cast<float> (xIdx);
      }
    }
  }
  integral_image1.setInput (data, width, height, element_stride, row_stride);
  integral_image2.setInput (data, width, height, element_stride, row_stride);
  for (unsigned window_width = min_window_size; window_width < max_window_size; ++window_width)
  {
    for (unsigned window_height = min_window_size; window_height < max_window_size; ++window_height)
    {
      for(unsigned yIdx = 0; yIdx < height - window_height; ++yIdx)
      {
        for(unsigned xIdx = 0; xIdx < width - window_width; ++xIdx)
        {
          int count = window_height * ((window_width - (xIdx&1) + 1) >> 1);

          EXPECT_EQ (count * 3, integral_image1.getFirstOrderSum (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (count * 3, integral_image2.getFirstOrderSum (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (count, integral_image1.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (count, integral_image2.getFiniteElementsCount (xIdx, yIdx, window_width, window_height));
          EXPECT_EQ (count * 9, integral_image1.getSecondOrderSum (xIdx, yIdx, window_width, window_height));
        }
      }
    }
  }
  delete[] data;


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, IntegralImage3D)
{
  const unsigned width = 640;
  const unsigned height = 480;
  const unsigned max_window_size = 5;
  const unsigned min_window_size = 1;
  IntegralImage2D<float, 3> integral_image3(true);
  unsigned element_stride = 4;
  unsigned row_stride = width * element_stride + 1;
  float* data = new float[row_stride * height];
  for(unsigned yIdx = 0; yIdx < height; ++yIdx)
  {
    for(unsigned xIdx = 0; xIdx < width; ++xIdx)
    {
      data[row_stride * yIdx + xIdx * element_stride] = static_cast<float> (xIdx);
      data[row_stride * yIdx + xIdx * element_stride + 1] = static_cast<float> (yIdx);
      data[row_stride * yIdx + xIdx * element_stride + 2] = static_cast<float> (xIdx + yIdx);
      data[row_stride * yIdx + xIdx * element_stride + 3] = -1000.0f;
    }
  }
  integral_image3.setInput (data, width, height, element_stride, row_stride);
  for (unsigned window_width = min_window_size; window_width < max_window_size; ++window_width)
  {
    for (unsigned window_height = min_window_size; window_height < max_window_size; ++window_height)
    {
      for(unsigned yIdx = 0; yIdx < height - window_height; ++yIdx)
      {
        for(unsigned xIdx = 0; xIdx < width - window_width; ++xIdx)
        {
          IntegralImage2D<float, 3>::ElementType sum = integral_image3.getFirstOrderSum (xIdx, yIdx, window_width, window_height);

          EXPECT_EQ (window_height * window_width * (window_width + 2 * xIdx - 1), sum[0] * 2);
          EXPECT_EQ (window_width * window_height * (window_height + 2 * yIdx - 1), sum[1] * 2);
          EXPECT_EQ (window_width * window_height * (window_height + 2 * yIdx - 1) + window_height * window_width * (window_width + 2 * xIdx - 1), sum[2] * 2);

          IntegralImage2D<float, 3>::SecondOrderType sumSqr = integral_image3.getSecondOrderSum (xIdx, yIdx, window_width, window_height);

          IntegralImage2D<float, 3>::SecondOrderType ground_truth;
          ground_truth.setZero ();
          for (unsigned wy = yIdx; wy < yIdx + window_height; ++wy)
          {
            for (unsigned wx = xIdx; wx < xIdx + window_width; ++wx)
            {
              float* val = data + (wy * row_stride + wx * element_stride);
              //ground_truth[0] += val[0] * val[0];
              ground_truth[1] += val[0] * val[1];
              ground_truth[2] += val[0] * val[2];
              //ground_truth[3] += val[1] * val[1];
              ground_truth[4] += val[1] * val[2];
              ground_truth[5] += val[2] * val[2];
            }
          }

          //EXPECT_EQ (ground_truth [0], sumSqr[0]);
          EXPECT_EQ (ground_truth [1], sumSqr[1]);
          EXPECT_EQ (ground_truth [2], sumSqr[2]);
          //EXPECT_EQ (ground_truth [3], sumSqr[3]);
          EXPECT_EQ (ground_truth [4], sumSqr[4]);
          EXPECT_EQ (ground_truth [5], sumSqr[5]);

          int w = window_width + xIdx - 1;
          long result = w * (w + 1) * (2*w + 1) - xIdx * (xIdx - 1) * (2*xIdx - 1);

          EXPECT_EQ (window_height * result, sumSqr[0] * 6);

          int h = window_height + yIdx - 1;
          result = h * (h + 1) * (2*h + 1) - yIdx * (yIdx - 1) * (2*yIdx - 1);
          EXPECT_EQ (window_width * result, sumSqr[3] * 6);
        }
      }
    }
  }
  delete[] data;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, NormalEstimation)
{
  tree.reset (new search::KdTree<PointXYZ> (false));
  n.setSearchMethod (tree);
  n.setKSearch (10);

  n.setInputCloud (cloud.makeShared ());

  PointCloud<Normal> output;
  n.compute (output);

  EXPECT_EQ (output.size (), cloud.size ());
  EXPECT_EQ (output.width, cloud.width);
  EXPECT_EQ (output.height, cloud.height);

  for (const auto& point: output)
  {
    EXPECT_NEAR (std::abs (point.normal_x),   0, 1e-2);
    EXPECT_NEAR (std::abs (point.normal_y),   0, 1e-2);
    EXPECT_NEAR (std::abs (point.normal_z), 1.0, 1e-2);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IINormalEstimationCovariance)
{
  PointCloud<Normal> output;
  ne.setRectSize (3, 3);
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.compute (output);

  EXPECT_EQ (output.size (), cloud.size ());
  EXPECT_EQ (output.width, cloud.width);
  EXPECT_EQ (output.height, cloud.height);

  for (std::size_t v = 0; v < cloud.height; ++v)
  {
    for (std::size_t u = 0; u < cloud.width; ++u)
    {
      if (!std::isfinite(output (u, v).normal_x) &&
          !std::isfinite(output (u, v).normal_y) &&
          !std::isfinite(output (u, v).normal_z))
        continue;

      EXPECT_NEAR (std::abs (output (u, v).normal_x),   0, 1e-2);
      EXPECT_NEAR (std::abs (output (u, v).normal_y),   0, 1e-2);
      EXPECT_NEAR (std::abs (output (u, v).normal_z), 1.0, 1e-2);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IINormalEstimationAverage3DGradient)
{
  PointCloud<Normal> output;
  ne.setRectSize (3, 3);
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.compute (output);

  EXPECT_EQ (output.size (), cloud.size ());
  EXPECT_EQ (output.width, cloud.width);
  EXPECT_EQ (output.height, cloud.height);

  for (std::size_t v = 0; v < cloud.height; ++v)
  {
    for (std::size_t u = 0; u < cloud.width; ++u)
    {
      if (!std::isfinite(output (u, v).normal_x) &&
          !std::isfinite(output (u, v).normal_y) &&
          !std::isfinite(output (u, v).normal_z))
        continue;

      if (std::abs(std::abs (output (u, v).normal_z) - 1) > 1e-2)
      {
        std::cout << "T:" << u << " , " << v << " : " << output (u, v).normal_x << " , " << output (u, v).normal_y << " , " << output (u, v).normal_z <<std::endl;
      }
      EXPECT_NEAR (std::abs (output (u, v).normal_x),   0, 1e-2);
      EXPECT_NEAR (std::abs (output (u, v).normal_y),   0, 1e-2);
      //EXPECT_NEAR (std::abs (output (u, v).normal_z), 1.0, 1e-2);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IINormalEstimationAverageDepthChange)
{
  PointCloud<Normal> output;
  ne.setRectSize (3, 3);
  ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
  ne.compute (output);

  EXPECT_EQ (output.size (), cloud.size ());
  EXPECT_EQ (output.width, cloud.width);
  EXPECT_EQ (output.height, cloud.height);

  for (std::size_t v = 0; v < cloud.height; ++v)
  {
    for (std::size_t u = 0; u < cloud.width; ++u)
    {
      if (!std::isfinite(output (u, v).normal_x) &&
          !std::isfinite(output (u, v).normal_y) &&
          !std::isfinite(output (u, v).normal_z))
        continue;

      if (std::abs(std::abs (output (u, v).normal_z) - 1) > 1e-2)
      {
        std::cout << "T:" << u << " , " << v << " : " << output (u, v).normal_x << " , " << output (u, v).normal_y << " , " << output (u, v).normal_z <<std::endl;
      }
      EXPECT_NEAR (std::abs (output (u, v).normal_x),   0, 1e-2);
      EXPECT_NEAR (std::abs (output (u, v).normal_y),   0, 1e-2);
      //EXPECT_NEAR (std::abs (output (u, v).normal_z), 1.0, 1e-2);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IINormalEstimationSimple3DGradient)
{
  PointCloud<Normal> output;
  ne.setRectSize (3, 3);
  ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne.compute (output);

  EXPECT_EQ (output.size (), cloud.size ());
  EXPECT_EQ (output.width, cloud.width);
  EXPECT_EQ (output.height, cloud.height);

  for (std::size_t v = 0; v < cloud.height; ++v)
  {
    for (std::size_t u = 0; u < cloud.width; ++u)
    {
      if (!std::isfinite(output (u, v).normal_x) &&
          !std::isfinite(output (u, v).normal_y) &&
          !std::isfinite(output (u, v).normal_z))
        continue;

      if (std::abs(std::abs (output (u, v).normal_z) - 1) > 1e-2)
      {
        std::cout << "T:" << u << " , " << v << " : " << output (u, v).normal_x << " , " << output (u, v).normal_y << " , " << output (u, v).normal_z <<std::endl;
      }
      EXPECT_NEAR (std::abs (output (u, v).normal_x),   0, 1e-2);
      EXPECT_NEAR (std::abs (output (u, v).normal_y),   0, 1e-2);
      //EXPECT_NEAR (std::abs (output (u, v).normal_z), 1.0, 1e-2);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, IINormalEstimationSimple3DGradientUnorganized)
{
  PointCloud<Normal> output;
  cloud.height = 1;
  cloud.resize (cloud.height * cloud.width);
  ne.setInputCloud (cloud.makeShared ());
  ne.setRectSize (3, 3);
  ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne.compute (output);

  EXPECT_EQ (output.size (), 0);
  EXPECT_EQ (output.width, 0);
  EXPECT_EQ (output.height, 0);
}

/* ---[ */
int
main (int argc, char** argv)
{
  cloud.width = 640;
  cloud.height = 480;
  cloud.resize (cloud.width * cloud.height);
  cloud.is_dense = true;
  for (std::size_t v = 0; v < cloud.height; ++v)
  {
    for (std::size_t u = 0; u < cloud.width; ++u)
    {
      cloud (u, v).x = static_cast<float> (u);
      cloud (u, v).y = static_cast<float> (v);
      cloud (u, v).z = 10.0f;
    }
  }

  ne.setInputCloud (cloud.makeShared ());
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());

  return 1;
}
/* ]--- */


/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */
/** \author Bastian Steder */

#include <pcl/pcl_macros.h>
#include <iostream> 
#include <sstream> 
#include <gtest/gtest.h>
#include <pcl/common/vector_average.h>
using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, VectorAverage_mean)
{
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3f::Scalar> weights;
  points.push_back (Eigen::Vector3f (-0.558191f, 0.180822f, -0.809769f));
  weights.push_back (0.160842f);
  points.push_back (Eigen::Vector3f (-0.510641f, 0.290673f, -0.809169f));
  weights.push_back (0.526732f);
  points.push_back (Eigen::Vector3f (-0.440713f, 0.385624f, -0.810597f));
  weights.push_back (0.312427f);
  
  Eigen::Vector3f correct_mean (0.0f, 0.0f, 0.0f);
  float weigth_sum = 0.0f;
  for (unsigned int i = 0; i < points.size (); ++i)
  {
    correct_mean += weights[i]*points[i];
    weigth_sum += weights[i];
  }
  correct_mean /= weigth_sum;
  
  pcl::VectorAverage<float, 3> va;
  for (unsigned int i=0; i<points.size(); ++i)
    va.add(points[i], weights[i]);
  Eigen::Vector3f mean = va.getMean();
  //std::cout << "Correct: "<<correct_mean <<"\n"<< "Result: "<<mean<<"\n";
  EXPECT_NEAR ((mean-correct_mean).norm(), 0.0f, 1e-4);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

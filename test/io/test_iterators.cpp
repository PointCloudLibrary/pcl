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

#include <pcl/test/gtest.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

PointCloud cloud;

void 
init () 
{
  for (int x=-20; x<20; x++) 
    for (int y=-20; y<20; y++) 
      for (int z=-20; z<20; z++) 
        cloud.push_back (Point (static_cast<float> (x), static_cast<float> (y), static_cast<float> (z)));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (PCL, Iterators) 
{
  Point mean (0,0,0);

  for (auto it = cloud.begin(); it != cloud.end(); ++it) 
  {
    for (int i=0;i<3;i++) mean.data[i] += it->data[i];
  }
  for (int i=0;i<3;i++) mean.data[i] /= static_cast<float> (cloud.size ());

  EXPECT_NEAR (mean.x, -0.5, 1e-4);
}


/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  init();
  return (RUN_ALL_TESTS ());
}
/* ]--- */

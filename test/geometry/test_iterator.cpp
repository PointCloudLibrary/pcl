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
 */

#include <gtest/gtest.h>
#include <pcl/common/common.h>
#include <pcl/geometry/line_iterator.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>

using namespace pcl;

template <typename PointT>
void checkSimpleLine8 (unsigned x_start, unsigned y_start, unsigned x_end, unsigned y_end, PointCloud<PointT>& cloud)
{
  PointXYZ point;
  point.x = point.y = point.z = 0.0f;
  for (unsigned yIdx = 0; yIdx < cloud.height; ++yIdx)
  {
    for (unsigned xIdx = 0; xIdx < cloud.width; ++xIdx)
    {
      PointT& point = cloud.points [yIdx * cloud.width + xIdx];
      point.x = float(xIdx);
      point.y = float(yIdx);
      point.z = 0.0f;
    }
  }

  LineIterator lineIt (x_start, y_start, x_end, y_end, cloud.width, LineIterator::Neighbor8);
  // use polymorphic
  OrganizedIndexIterator& iterator = lineIt;
  unsigned idx = 0;
  while (iterator.isValid ())
  {
    PointT& point = cloud[*iterator];
    EXPECT_EQ (point.x, iterator.getColumnIndex ());
    EXPECT_EQ (point.y, iterator.getRowIndex ());
    point.z = 1.0f;
    ++iterator;
    ++idx;
  }
  int dx = x_end - x_start;
  int dy = y_end - y_start;
  unsigned dmax = std::max (abs(dx), abs(dy));
  
  EXPECT_EQ (dmax, idx);
  
  int x_step = 0;
  int y_step = 0;
  
  EXPECT_GT (dmax, 0);
  if (dx == 0)
  {
    x_step = 0;
    y_step = (dy > 0) ? 1 : -1;
  }
  else if (dy == 0)
  {
    y_step = 0;
    x_step = (dx > 0) ? 1 : -1;
  }
  else if (abs(dx) == abs(dy))
  {
    y_step = (dy > 0) ? 1 : -1;
    x_step = (dx > 0) ? 1 : -1;
  }
  else
  {
    // only horizontal, vertical and 45deg diagonal lines handled here
    EXPECT_TRUE (false);
  }
  unsigned xIdx = x_start;
  unsigned yIdx = y_start;
  for (unsigned idx = 0; idx < dmax; ++idx, xIdx += x_step, yIdx += y_step)
  {
    PointT& point = cloud.points [yIdx * cloud.width + xIdx];
    EXPECT_EQ (point.z, 1.0f);
    point.z = 0.0;
  }
  // now all z-values should be 0 again!
  for (unsigned yIdx = 0; yIdx < cloud.height; ++yIdx)
  {
    for (unsigned xIdx = 0; xIdx < cloud.width; ++xIdx)
    {
      //std::cout << "testing  point: " << xIdx << " , " << yIdx << std::endl;
      PointT& point = cloud.points [yIdx * cloud.width + xIdx];
//      if (point.z != 0.0f)
//        std::cout << "point.z != 0.0f at: " << xIdx << " , " << yIdx << std::endl;
      EXPECT_EQ (point.z, 0.0f);
    }
  }
}

template <typename PointT>
void checkGeneralLine (unsigned x_start, unsigned y_start, unsigned x_end, unsigned y_end, PointCloud<PointT>& cloud, bool neighorhood)
{
  PointXYZ point;
  point.x = point.y = point.z = 0.0f;
  for (unsigned yIdx = 0; yIdx < cloud.height; ++yIdx)
  {
    for (unsigned xIdx = 0; xIdx < cloud.width; ++xIdx)
    {
      PointT& point = cloud.points [yIdx * cloud.width + xIdx];
      point.x = float(xIdx);
      point.y = float(yIdx);
      point.z = 0.0f;
    }
  }

  LineIterator::Neighborhood neighbors;
  if (neighorhood)
    neighbors = LineIterator::Neighbor8;
  else
    neighbors = LineIterator::Neighbor4;
    
  LineIterator lineIt (x_start, y_start, x_end, y_end, cloud.width, neighbors);
  // use polymorphic
  OrganizedIndexIterator& iterator = lineIt;
  unsigned idx = 0;
  while (iterator.isValid ())
  {
    PointT& point = cloud [*iterator];
    EXPECT_EQ (point.x, iterator.getColumnIndex ());
    EXPECT_EQ (point.y, iterator.getRowIndex ());
    //std::cout << idx << " :: " << iterator.getPointIndex () << " :: " << iterator.getColumnIndex () << " , " << iterator.getRowIndex () << std::endl;
    point.z = 1.0f;
    ++iterator;
    ++idx;
  }
  
  int dx = x_end - x_start;
  int dy = y_end - y_start;
  unsigned dmax = std::max (abs(dx), abs(dy));

  if (neighorhood)
    EXPECT_EQ (dmax, idx);
  else
    EXPECT_EQ (abs(dx) + abs(dy), idx);
  
  float length = sqrtf (float (dx * dx + dy * dy));
  float dir_x = float (dx) / length;
  float dir_y = float (dy) / length;
  
  // now all z-values should be 0 again!
  for (int yIdx = 0; yIdx < int(cloud.height); ++yIdx)
  {
    for (int xIdx = 0; xIdx < int(cloud.width); ++xIdx)
    {
      PointT& point = cloud.points [yIdx * cloud.width + xIdx];
      if (point.z != 0)
      {
        // point need to be close to line
        float distance = dir_x * float(yIdx - int(y_start)) - dir_y * float(xIdx - int(x_start));
        if (neighorhood)        
          EXPECT_LE (fabs(distance), 0.5f);
        else
          EXPECT_LE (fabs(distance), 0.70711f);
        
        // and within the endpoints
        float lambda = dir_y * float(yIdx - int(y_start)) + dir_x * float(xIdx - int(x_start));
        EXPECT_LE (lambda, length);
        EXPECT_GE (lambda, 0.0f);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, LineIterator8Neighbors)
{
  PointCloud <PointXYZ> cloud;
  cloud.width = 100;
  cloud.height = 100;
  cloud.resize (cloud.width * cloud.height);  
  
  unsigned center_x = 50;
  unsigned center_y = 50;
  unsigned length = 45;
  
  // right
  checkSimpleLine8 (center_x, center_y, center_x + length, center_y, cloud);
  
  // left
  checkSimpleLine8 (center_x, center_y, center_x - length, center_y, cloud);

  // down
  checkSimpleLine8 (center_x, center_y, center_x, center_y - length, cloud);
  
  // up
  checkSimpleLine8 (center_x, center_y, center_x, center_y + length, cloud);

  // up-right
  checkSimpleLine8 (center_x, center_y, center_x + length, center_y + length, cloud);
  
  // up-left
  checkSimpleLine8 (center_x, center_y, center_x - length, center_y + length, cloud);

  // down-right
  checkSimpleLine8 (center_x, center_y, center_x + length, center_y - length, cloud);

  // down-left
  checkSimpleLine8 (center_x, center_y, center_x - length, center_y - length, cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, LineIterator8NeighborsGeneral)
{
  PointCloud <PointXYZ> cloud;
  cloud.width = 100;
  cloud.height = 100;
  cloud.resize (cloud.width * cloud.height);  
  
  unsigned center_x = 50;
  unsigned center_y = 50;
  unsigned length = 45;
  
  const unsigned angular_resolution = 180;
  float d_alpha = float(M_PI / angular_resolution);
  for (unsigned idx = 0; idx < angular_resolution; ++idx)
  {
    unsigned x_end = unsigned (length * cos (float(idx) * d_alpha) + center_x + 0.5);
    unsigned y_end = unsigned (length * sin (float(idx) * d_alpha) + center_y + 0.5);
    
    // right
    checkGeneralLine (center_x, center_y, x_end, y_end, cloud, true);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, LineIterator4NeighborsGeneral)
{
  PointCloud <PointXYZ> cloud;
  cloud.width = 100;
  cloud.height = 100;
  cloud.resize (cloud.width * cloud.height);  
  
  unsigned center_x = 50;
  unsigned center_y = 50;
  unsigned length = 45;
  
  const unsigned angular_resolution = 360;
  float d_alpha = float(2.0 * M_PI / angular_resolution);
  for (unsigned idx = 0; idx < angular_resolution; ++idx)
  {
    unsigned x_end = unsigned (length * cos (float(idx) * d_alpha) + center_x + 0.5);
    unsigned y_end = unsigned (length * sin (float(idx) * d_alpha) + center_y + 0.5);
    
    // right
    checkGeneralLine (center_x, center_y, x_end, y_end, cloud, false);
  }
}

//* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#include <cmath>
#include <pcl/PCLPointCloud2.h> // for PCLPointCloud2
#include <pcl/common/time.h> // for MEASURE_FUNCTION_TIME
#include <pcl/range_image/range_image.h>

#include <algorithm>

namespace pcl 
{

bool RangeImage::debug = false;
int RangeImage::max_no_of_threads = 1;
const int RangeImage::lookup_table_size = 20001;
std::vector<float> RangeImage::asin_lookup_table;
std::vector<float> RangeImage::atan_lookup_table;
std::vector<float> RangeImage::cos_lookup_table;

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::createLookupTables ()
{
  if (!asin_lookup_table.empty ())
    return;
  
  asin_lookup_table.resize (lookup_table_size);
  for (int i=0; i<lookup_table_size; ++i) {
    float value = static_cast<float> (i-(lookup_table_size-1)/2)/static_cast<float> ((lookup_table_size-1)/2);
    asin_lookup_table[i] = asinf (value);
  }
  
  atan_lookup_table.resize (lookup_table_size);
  for (int i=0; i<lookup_table_size; ++i) 
  {
    float value = static_cast<float> (i-(lookup_table_size-1)/2)/static_cast<float> ((lookup_table_size-1)/2);
    atan_lookup_table[i] = std::atan (value);
  }
  
  cos_lookup_table.resize (lookup_table_size);
  
  for (int i = 0; i < lookup_table_size; ++i) 
  {
    float value = static_cast<float> (i) * 2.0f * static_cast<float> (M_PI) / static_cast<float> (lookup_table_size-1);
    cos_lookup_table[i] = std::cos (value);
  }
}



/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getCoordinateFrameTransformation (RangeImage::CoordinateFrame coordinate_frame,
                                             Eigen::Affine3f& transformation)
{
  switch (coordinate_frame)
  {
    case LASER_FRAME:
      transformation (0,0)= 0.0f; transformation (0,1)= 0.0f; transformation (0,2)=1.0f; transformation (0,3)=0.0f;
      transformation (1,0)=-1.0f; transformation (1,1)= 0.0f; transformation (1,2)=0.0f; transformation (1,3)=0.0f;
      transformation (2,0)= 0.0f; transformation (2,1)=-1.0f; transformation (2,2)=0.0f; transformation (2,3)=0.0f;
      transformation (3,0)= 0.0f; transformation (3,1)= 0.0f; transformation (3,2)=0.0f; transformation (3,3)=1.0f;
      break;
    case CAMERA_FRAME:
    default:
      transformation.setIdentity ();
      break;
  }
}

/////////////////////////////////////////////////////////////////////////
RangeImage::RangeImage () : 
  to_range_image_system_ (Eigen::Affine3f::Identity ()),
  to_world_system_ (Eigen::Affine3f::Identity ()),
  angular_resolution_x_ (0), angular_resolution_y_ (0),
  angular_resolution_x_reciprocal_ (0), angular_resolution_y_reciprocal_ (0),
  image_offset_x_ (0), image_offset_y_ (0)
{
  createLookupTables ();
  reset ();
  unobserved_point.x = unobserved_point.y = unobserved_point.z = std::numeric_limits<float>::quiet_NaN ();
  unobserved_point.range = -std::numeric_limits<float>::infinity ();
}

/////////////////////////////////////////////////////////////////////////
void
RangeImage::reset ()
{
  is_dense = true;
  width = height = 0;
  points.clear ();
  to_range_image_system_.setIdentity ();
  to_world_system_.setIdentity ();
  setAngularResolution (deg2rad (0.5f));
  image_offset_x_ = image_offset_y_ = 0;
}

/////////////////////////////////////////////////////////////////////////
void
RangeImage::createEmpty (float angular_resolution, const Eigen::Affine3f& sensor_pose,
                         RangeImage::CoordinateFrame coordinate_frame, float angle_width, float angle_height)
{
  createEmpty (angular_resolution, angular_resolution, sensor_pose, coordinate_frame, angle_width, angle_height);
}

/////////////////////////////////////////////////////////////////////////
void
RangeImage::createEmpty (float angular_resolution_x, float angular_resolution_y, const Eigen::Affine3f& sensor_pose,
                         RangeImage::CoordinateFrame coordinate_frame, float angle_width, float angle_height)
{
  setAngularResolution(angular_resolution_x, angular_resolution_y);
  width  = static_cast<std::uint32_t> (pcl_lrint (std::floor (angle_width * angular_resolution_x_reciprocal_)));
  height = static_cast<std::uint32_t> (pcl_lrint (std::floor (angle_height * angular_resolution_y_reciprocal_)));

  int full_width  = static_cast<int> (pcl_lrint (std::floor (pcl::deg2rad (360.0f)*angular_resolution_x_reciprocal_))),
      full_height = static_cast<int> (pcl_lrint (std::floor (pcl::deg2rad (180.0f)*angular_resolution_y_reciprocal_)));
  image_offset_x_ = (full_width-width)/2;
  image_offset_y_ = (full_height-height)/2;
  is_dense = false;
  getCoordinateFrameTransformation (coordinate_frame, to_world_system_);
  to_world_system_ = sensor_pose * to_world_system_;
  to_range_image_system_ = to_world_system_.inverse (Eigen::Isometry);
  unsigned int size = width*height;
  points.clear ();
  points.resize (size, unobserved_point);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::cropImage (int borderSize, int top, int right, int bottom, int left) {
  //MEASURE_FUNCTION_TIME;
  
  bool topIsDone=true, rightIsDone=true, bottomIsDone=true, leftIsDone=true;
  if (top < 0) {
    top=-1;
    topIsDone=false;
  }
  if (right < 0) 
  {
    right= static_cast<int> (width);
    rightIsDone=false;
  }
  if (bottom < 0) 
  {
    bottom= static_cast<int> (height);
    bottomIsDone=false;
  }
  if (left < 0) 
  {
    left = -1;
    leftIsDone = false;
  }
  
  // Find top border
  while (!topIsDone && top<=bottom) 
  {
    ++top;
    int lineStart = top*width;
    int min_x=std::max(0, left), max_x=std::min(static_cast<int> (width)-1, right);
    for (int x=min_x; x<=max_x && !topIsDone; ++x)
      if (std::isfinite (points[lineStart + x].range))
        topIsDone = true;
  }
  // Check if range image is empty
  if (top >= static_cast<int> (height)) 
  {
    points.clear ();
    width = height = 0;
    return;
  }
  // Find right border
  while (!rightIsDone) 
  {
    --right;
    int min_y=std::max(0, top), max_y=std::min(static_cast<int> (height)-1, bottom);
    for (int y=min_y; y<=max_y && !rightIsDone; ++y)
      if (std::isfinite (points[y*width + right].range))
        rightIsDone = true;
  }
  // Find bottom border
  while (!bottomIsDone) 
  {
    --bottom;
    int lineStart = bottom*width;
    int min_x=std::max(0, left), max_x=std::min(static_cast<int> (width)-1, right);
    for (int x=min_x; x<=max_x && !bottomIsDone; ++x)
      if (std::isfinite (points[lineStart + x].range))
        bottomIsDone = true;
  } 
  // Find left border
  while (!leftIsDone) 
  {
    ++left;
    int min_y=std::max(0, top), max_y=std::min(static_cast<int> (height)-1, bottom);
    for (int y=min_y; y<=max_y && !leftIsDone; ++y)
      if (std::isfinite (points[y*width + left].range))
        leftIsDone = true;
  } 
  left-=borderSize; top-=borderSize; right+=borderSize; bottom+=borderSize;
  
  // Create copy without copying the old points - vector::swap only copies a few pointers, not the content
  PointCloud<PointWithRange>::VectorType tmpPoints;
  points.swap (tmpPoints);
  RangeImage oldRangeImage = *this;
  tmpPoints.swap (oldRangeImage.points);
  
  width = right-left+1; height = bottom-top+1;
  image_offset_x_ = left+oldRangeImage.image_offset_x_;
  image_offset_y_ = top+oldRangeImage.image_offset_y_;
  points.resize (width*height);
  
  //std::cout << oldRangeImage.width<<"x"<<oldRangeImage.height<<" -> "<<width<<"x"<<height<<"\n";
  
  // Copy points
  for (int y=0, oldY=top; y< static_cast<int> (height); ++y,++oldY)
  {
    for (int x=0, oldX=left; x< static_cast<int> (width); ++x,++oldX)
    {
      PointWithRange& currentPoint = points[y*width + x];
      if (oldX<0 || oldX>= static_cast<int> (oldRangeImage.width) || oldY<0 || oldY>= static_cast<int> (oldRangeImage.height))
      {
        currentPoint = unobserved_point;
        continue;
      }
      currentPoint = oldRangeImage[oldY*oldRangeImage.width + oldX];
    }
  }
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::recalculate3DPointPositions () 
{
  for (int y = 0; y < static_cast<int> (height); ++y)
  {
    for (int x = 0; x < static_cast<int> (width); ++x)
    {
      PointWithRange& point = points[y*width + x];
      if (!std::isinf (point.range)) 
        calculate3DPoint (static_cast<float> (x), static_cast<float> (y), point.range, point);
    }
  }
}

/////////////////////////////////////////////////////////////////////////
float* 
RangeImage::getRangesArray () const 
{
  int arraySize = width * height;
  float* ranges = new float[arraySize];
  for (int i=0; i<arraySize; ++i)
    ranges[i] = points[i].range;
  return ranges;
}


/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getIntegralImage (float*& integral_image, int*& valid_points_num_image) const
{
  integral_image = new float[width*height];
  float* integral_image_ptr = integral_image;
  valid_points_num_image = new int[width*height];
  int* valid_points_num_image_ptr = valid_points_num_image;
  for (int y = 0; y < static_cast<int> (height); ++y)
  {
    for (int x = 0; x < static_cast<int> (width); ++x)
    {
      float& integral_pixel = * (integral_image_ptr++);
      integral_pixel = getPoint (x, y).range;
      int& valid_points_num = * (valid_points_num_image_ptr++);
      valid_points_num = 1;
      if (std::isinf (integral_pixel))
      {
        integral_pixel = 0.0f;
        valid_points_num = 0;
      }
      float left_value=0, top_left_value=0, top_value=0;
      int left_valid_points=0, top_left_valid_points=0, top_valid_points=0;
      if (x>0)
      {
        left_value = integral_image[y*width+x-1];
        left_valid_points = valid_points_num_image[y*width+x-1];
        if (y>0)
        {
          top_left_value = integral_image[ (y-1)*width+x-1];
          top_left_valid_points = valid_points_num_image[ (y-1)*width+x-1];
        }
      }
      if (y>0)
      {
        top_value = integral_image[ (y-1)*width+x];
        top_valid_points = valid_points_num_image[ (y-1)*width+x];
      }
      
      integral_pixel += left_value + top_value - top_left_value;
      valid_points_num += left_valid_points + top_valid_points - top_left_valid_points;
    }
  }
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::setUnseenToMaxRange ()
{
  for (auto &point : points)
    if (std::isinf (point.range))
      point.range = std::numeric_limits<float>::infinity ();
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getHalfImage (RangeImage& half_image) const
{
  half_image.setAngularResolution (2.0f*angular_resolution_x_, 2.0f*angular_resolution_y_);
  half_image.image_offset_x_ = image_offset_x_/2;
  half_image.image_offset_y_ = image_offset_y_/2;
  half_image.width  = width/2;
  half_image.height = height/2;
  half_image.is_dense = is_dense;
  half_image.clear ();
  half_image.resize (half_image.width*half_image.height);
  
  int src_start_x = 2*half_image.image_offset_x_ - image_offset_x_,
      src_start_y = 2*half_image.image_offset_y_ - image_offset_y_;
  
  for (int dst_y=0; dst_y < static_cast<int> (half_image.height); ++dst_y)
  {
    for (int dst_x=0; dst_x < static_cast<int> (half_image.width); ++dst_x)
    {
      PointWithRange& dst_point = half_image.getPoint (dst_x, dst_y);
      dst_point=unobserved_point;
      int src_x_min = src_start_x + 2*dst_x,
          src_x_max = src_x_min + 1,
          src_y_min = src_start_y + 2*dst_y,
          src_y_max = src_y_min + 1;
      for (int src_x=src_x_min; src_x<=src_x_max; ++src_x)
      {
        for (int src_y=src_y_min; src_y<=src_y_max; ++src_y)
        {
          if (!isObserved (src_x, src_y))
            continue;
          const PointWithRange& src_point = getPoint (src_x, src_y);
          if (std::isfinite (dst_point.range) && src_point.range > dst_point.range)
            continue;
          dst_point = src_point;
        }
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getSubImage (int sub_image_image_offset_x, int sub_image_image_offset_y, int sub_image_width,
                         int sub_image_height, int combine_pixels, RangeImage& sub_image) const
{
  sub_image.setAngularResolution (static_cast<float> (combine_pixels)*angular_resolution_x_,
                                  static_cast<float> (combine_pixels)*angular_resolution_y_);
  sub_image.image_offset_x_ = sub_image_image_offset_x;
  sub_image.image_offset_y_ = sub_image_image_offset_y;
  sub_image.width = sub_image_width;
  sub_image.height = sub_image_height;
  sub_image.is_dense = is_dense;
  sub_image.clear ();
  sub_image.resize (sub_image.width*sub_image.height);
  
  int src_start_x = combine_pixels*sub_image.image_offset_x_ - image_offset_x_,
      src_start_y = combine_pixels*sub_image.image_offset_y_ - image_offset_y_;
  
  for (int dst_y=0; dst_y < static_cast<int> (sub_image.height); ++dst_y)
  {
    for (int dst_x=0; dst_x < static_cast<int> (sub_image.width); ++dst_x)
    {
      PointWithRange& dst_point = sub_image.getPoint (dst_x, dst_y);
      dst_point=unobserved_point;
      int src_x_min = src_start_x + combine_pixels*dst_x,
          src_x_max = src_x_min + combine_pixels-1,
          src_y_min = src_start_y + combine_pixels*dst_y,
          src_y_max = src_y_min + combine_pixels-1;
      for (int src_x=src_x_min; src_x<=src_x_max; ++src_x)
      {
        for (int src_y=src_y_min; src_y<=src_y_max; ++src_y)
        {
          if (!isInImage (src_x, src_y))
            continue;
          const PointWithRange& src_point = getPoint (src_x, src_y);
          if (std::isfinite (dst_point.range) && src_point.range > dst_point.range)
            continue;
          dst_point = src_point;
        }
      }
    }
  }
}


/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getMinMaxRanges (float& min_range, float& max_range) const
{
  min_range = std::numeric_limits<float>::infinity ();
  max_range = -std::numeric_limits<float>::infinity ();
  for (const auto &point : points)
  {
    float range = point.range;
    if (!std::isfinite (range))
      continue;
    min_range = (std::min) (min_range, range);
    max_range = (std::max) (max_range, range);
  }
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::change3dPointsToLocalCoordinateFrame ()
{
  to_world_system_.setIdentity ();
  to_range_image_system_.setIdentity ();
  recalculate3DPointPositions ();
}

/////////////////////////////////////////////////////////////////////////
float* 
RangeImage::getInterpolatedSurfaceProjection (const Eigen::Affine3f& pose, int pixel_size, float world_size) const
{
  float max_dist = 0.5f*world_size,
        cell_size = world_size/float (pixel_size);
  float world2cell_factor = 1.0f/cell_size,
        world2cell_offset = 0.5f*float (pixel_size)-0.5f;
  float cell2world_factor = cell_size,
        cell2world_offset = -max_dist + 0.5f*cell_size;
  Eigen::Affine3f inverse_pose = pose.inverse (Eigen::Isometry);
  
  int no_of_pixels = pixel_size*pixel_size;
  float* surface_patch = new float[no_of_pixels];
  std::fill_n(surface_patch, no_of_pixels, -std::numeric_limits<float>::infinity ());

  Eigen::Vector3f position = inverse_pose.translation ();
  int middle_x, middle_y;
  getImagePoint (position, middle_x, middle_y);
  int min_search_radius = 2;
  bool still_in_range = true;

  for (int radius=0;  still_in_range;  ++radius) 
  {
    int x=middle_x-radius-1, y=middle_y-radius;  // Top left - 1
    still_in_range = radius<min_search_radius;
    for (int i=0; i<8*radius || (radius==0&&i==0); ++i)
    {
      if (i<=2*radius) ++x; else if (i<=4*radius) ++y; else if (i<=6*radius) --x; else --y;

      Eigen::Vector3f point1, point2, point3;
      if (!isValid (x,y) || !isValid (x+1,y+1))
        continue;
      getPoint (x, y, point1);
      point1 = pose*point1;
      if (std::abs (point1[2]) > max_dist)
        continue;
      
      getPoint (x+1, y+1, point2);
      point2 = pose*point2;
      if (std::abs (point2[2]) > max_dist)
        continue;
      
      for (int triangle_idx=0; triangle_idx<=1; ++triangle_idx)
      {
        if (triangle_idx==0)  // First triangle
        {
          if (!isValid (x,y+1))
            continue;
          getPoint (x, y+1, point3);
        }
        else  // Second triangle
        {
          if (!isValid (x+1,y))
            continue;
          getPoint (x+1, y, point3);
        }
        point3 = pose*point3;
        if (std::abs (point3[2]) > max_dist)
          continue;
        
        // Are all the points either left, right, on top or below the bottom of the surface patch?
        if ( (point1[0] < -max_dist  &&  point2[0] < -max_dist  &&  point3[0] < -max_dist) ||
            (point1[0] >  max_dist  &&  point2[0] >  max_dist  &&  point3[0] >  max_dist) ||
            (point1[1] < -max_dist  &&  point2[1] < -max_dist  &&  point3[1] < -max_dist) ||
            (point1[1] >  max_dist  &&  point2[1] >  max_dist  &&  point3[1] >  max_dist))
        {
          continue;
        }
        
        still_in_range = true;
        
        // Now we have a valid triangle (point1, point2, point3) in our new patch
        float cell1_x = world2cell_factor*point1[0] + world2cell_offset,
              cell1_y = world2cell_factor*point1[1] + world2cell_offset,
              cell1_z = point1[2],
              cell2_x = world2cell_factor*point2[0] + world2cell_offset,
              cell2_y = world2cell_factor*point2[1] + world2cell_offset,
              cell2_z = point2[2],
              cell3_x = world2cell_factor*point3[0] + world2cell_offset,
              cell3_y = world2cell_factor*point3[1] + world2cell_offset,
              cell3_z = point3[2];
        
        int min_cell_x = (std::max) (0, int (pcl_lrint (std::ceil ( (std::min) (cell1_x, (std::min) (cell2_x, cell3_x)))))),
            max_cell_x = (std::min) (pixel_size-1, int (pcl_lrint (std::floor ( (std::max) (cell1_x,
                                                                       (std::max) (cell2_x, cell3_x)))))),
            min_cell_y = (std::max) (0, int (pcl_lrint (std::ceil ( (std::min) (cell1_y, (std::min) (cell2_y, cell3_y)))))),
            max_cell_y = (std::min) (pixel_size-1, int (pcl_lrint (std::floor ( (std::max) (cell1_y,
                                                                       (std::max) (cell2_y, cell3_y))))));
        if (max_cell_x<min_cell_x || max_cell_y<min_cell_y)
          continue;
        
        // We will now do the following:
        //   For each cell in the rectangle defined by the four values above,
        //   we test if it is in the original triangle (in 2D).
        //   If this is the case, we calculate the actual point on the 3D triangle, thereby interpolating the result
        //   See http://www.blackpawn.com/texts/pointinpoly/default.html
        Eigen::Vector2f cell1 (cell1_x, cell1_y),
                        cell2 (cell2_x, cell2_y),
                        cell3 (cell3_x, cell3_y),
                        v0 = cell3 - cell1,
                        v1 = cell2 - cell1;
        float dot00 = v0.dot (v0),
              dot01 = v0.dot (v1),
              dot11 = v1.dot (v1),
              invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        
        for (int cell_x=min_cell_x; cell_x<=max_cell_x; ++cell_x)
        {
          for (int cell_y=min_cell_y; cell_y<=max_cell_y; ++cell_y)
          {
            Eigen::Vector2f current_cell (cell_x, cell_y),
                            v2 = current_cell - cell1;
            float dot02 = v0.dot (v2),
                  dot12 = v1.dot (v2),
                  u = (dot11 * dot02 - dot01 * dot12) * invDenom,
                  v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            bool point_in_triangle = (u > -0.01) && (v >= -0.01) && (u + v <= 1.01);
            
            if (!point_in_triangle)
              continue;
            
            float new_value = cell1_z + u* (cell3_z-cell1_z) + v* (cell2_z-cell1_z);
            
            float& value = surface_patch[cell_y*pixel_size + cell_x];
            if (std::isinf (value))
              value = new_value;
            else
              value = (std::min) (value, new_value);
          }
        }
      }
    }
  }
  
  // Now find out, if there should be max ranges
  for (int cell_y=0; cell_y<pixel_size; ++cell_y)
  {
    for (int cell_x=0; cell_x<pixel_size; ++cell_x)
    {
      int index= cell_y*pixel_size + cell_x;
      float& value = surface_patch[index];
      if (!std::isinf (value))
        continue;
      
      // Go through immediate neighbors
      bool is_background = false;
      for (int cell2_y=cell_y-1; cell2_y<=cell_y+1&&!is_background; ++cell2_y)
      {
        for (int cell2_x=cell_x-1; cell2_x<=cell_x+1; ++cell2_x)
        {
          if (cell2_x<0||cell2_x>=pixel_size||cell2_y<0||cell2_y>=pixel_size || (cell2_x==cell_x && cell2_y==cell_y))
            continue;
          float neighbor_value = surface_patch[cell2_y*pixel_size + cell2_x];
          if (std::isfinite (neighbor_value))
          {
            float cell_pos_x = static_cast<float> (cell_x) + 0.6f * static_cast<float> (cell_x - cell2_x),
                  cell_pos_y = static_cast<float> (cell_y) + 0.6f * static_cast<float> (cell_y - cell2_y);
            Eigen::Vector3f fake_point (cell2world_factor* (cell_pos_x)+cell2world_offset,
                                        cell2world_factor*cell_pos_y+cell2world_offset, neighbor_value);
            fake_point = inverse_pose*fake_point;
            float range_difference = getRangeDifference (fake_point);
            if (range_difference > max_dist)
            {
              value = std::numeric_limits<float>::infinity ();
              is_background = true;
              break;
            }
          }
        }
      }
      if (is_background)
      {
        // Set all -INFINITY neighbors to INFINITY
        for (int cell2_y=cell_y-1; cell2_y<=cell_y+1; ++cell2_y)
        {
          for (int cell2_x=cell_x-1; cell2_x<=cell_x+1; ++cell2_x)
          {
            if (cell2_x<0||cell2_x>=pixel_size||cell2_y<0||cell2_y>=pixel_size || (cell2_x==cell_x && cell2_y==cell_y))
              continue;
            int index2 = cell2_y*pixel_size + cell2_x;
            float& neighbor_value = surface_patch[index2];
            if (std::isinf (neighbor_value) && neighbor_value<0)
              neighbor_value = std::numeric_limits<float>::infinity ();
          }
        }
      }
    }
  }
  
  return surface_patch;
}

/////////////////////////////////////////////////////////////////////////
float* RangeImage::getInterpolatedSurfaceProjection (const Eigen::Vector3f& point, int pixel_size,
                                                     float world_size) const
{
  Eigen::Affine3f pose = getTransformationToViewerCoordinateFrame (point);
  return (getInterpolatedSurfaceProjection (pose, pixel_size, world_size));
}

/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::getNormalBasedUprightTransformation (const Eigen::Vector3f& point, float max_dist,
                                                 Eigen::Affine3f& transformation) const
{
  int x, y;
  getImagePoint (point, x, y);

  Eigen::Vector3f neighbor;
  VectorAverage3f vector_average;
  float max_dist_squared=max_dist*max_dist, max_dist_reciprocal=1.0f/max_dist;
  
  bool still_in_range = true;
  for (int radius=1;  still_in_range;  ++radius) 
  {
    int x2=x-radius-1, y2=y-radius;  // Top left - 1
    still_in_range = false;
    for (int i=0; i<8*radius; ++i)
    {
      if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
      if (!isValid (x2, y2))
      {
        continue;
      }
      getPoint (x2, y2, neighbor);
      float distance_squared = (neighbor-point).squaredNorm ();
      if (distance_squared > max_dist_squared)
      {
        continue;
      }
      still_in_range = true;
      float distance = std::sqrt (distance_squared),
            weight = distance*max_dist_reciprocal;
      vector_average.add (neighbor, weight);
    }
  }
  
  Eigen::Vector3f normal, point_on_plane;
  if (vector_average.getNoOfSamples () > 10)
  {
    Eigen::Vector3f eigen_values, eigen_vector2, eigen_vector3;
    vector_average.doPCA (eigen_values, normal, eigen_vector2, eigen_vector3);
    if (normal.dot ( (vector_average.getMean ()-getSensorPos ()).normalized ()) < 0.0f)
      normal *= -1.0f;
    point_on_plane = (normal.dot (vector_average.getMean ()) - normal.dot (point))*normal + point;
  }
  else
  {
    if (!getNormalForClosestNeighbors (x, y, 2, point, 15, normal, &point_on_plane, 1))
      return false;
  }
  getTransformationFromTwoUnitVectorsAndOrigin (Eigen::Vector3f (0.0f, 1.0f, 0.0f),
                                                normal, point_on_plane, transformation);
  
  return (true);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getSurfaceAngleChangeImages (int radius, float*& angle_change_image_x, float*& angle_change_image_y) const
{
  MEASURE_FUNCTION_TIME;
  int size = width*height;
  angle_change_image_x = new float[size];
  angle_change_image_y = new float[size];
  for (int y=0; y<int (height); ++y)
  {
    for (int x=0; x<int (width); ++x)
    {
      int index = y*width+x;
      getSurfaceAngleChange (x, y, radius, angle_change_image_x[index], angle_change_image_y[index]);
    }
  }
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getAcutenessValueImages (int pixel_distance, float*& acuteness_value_image_x,
                                     float*& acuteness_value_image_y) const
{
  MEASURE_FUNCTION_TIME;
  int size = width*height;
  acuteness_value_image_x = new float[size];
  acuteness_value_image_y = new float[size];
  for (int y=0; y<int (height); ++y)
  {
    for (int x=0; x<int (width); ++x)
    {
      int index = y*width+x;
      acuteness_value_image_x[index] = getAcutenessValue (x, y, x+pixel_distance, y);
      acuteness_value_image_y[index] = getAcutenessValue (x, y, x, y+pixel_distance);
    }
  }
}

/////////////////////////////////////////////////////////////////////////
float* 
RangeImage::getImpactAngleImageBasedOnLocalNormals (int radius) const
{
  MEASURE_FUNCTION_TIME;
  int size = width*height;
  float* impact_angle_image = new float[size];
  for (int y=0; y<int (height); ++y)
  {
    for (int x=0; x<int (width); ++x)
    {
      impact_angle_image[y*width+x] = getImpactAngleBasedOnLocalNormal (x, y, radius);
    }
  }
  return impact_angle_image;
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getRangeImageWithSmoothedSurface (int radius, RangeImage& smoothed_range_image) const
{
  int step_size = (std::max) (1, radius/2);
  int no_of_nearest_neighbors = static_cast<int> (pow (static_cast<double> (radius / step_size + 1), 2.0));
  
  smoothed_range_image = *this;
  Eigen::Vector3f sensor_pos = getSensorPos ();
  for (int y=0; y<int (height); ++y)
  {
    for (int x=0; x<int (width); ++x)
    {
      PointWithRange& point = smoothed_range_image.getPoint (x, y);
      if (std::isinf (point.range))
        continue;
      Eigen::Vector3f normal, mean, eigen_values;
      float used_squared_max_distance;
      getSurfaceInformation (x, y, radius, point.getVector3fMap (), no_of_nearest_neighbors,
                             step_size, used_squared_max_distance,
                             normal, mean, eigen_values);
      
      Eigen::Vector3f viewing_direction = (point.getVector3fMap ()-sensor_pos).normalized ();
      float new_range = normal.dot (mean-sensor_pos) / normal.dot (viewing_direction);
      point.range = new_range;
      calculate3DPoint (static_cast<float> (x), static_cast<float> (y), point.range, point);
      
      const PointWithRange& original_point = getPoint (x, y);
      float distance_squared = squaredEuclideanDistance (original_point, point);
      if (distance_squared > used_squared_max_distance)
        point = original_point;
    }
  }
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::extractFarRanges (const pcl::PCLPointCloud2& point_cloud_data,
                              PointCloud<PointWithViewpoint>& far_ranges)
{
  int x_idx = -1, y_idx = -1, z_idx = -1,
      vp_x_idx = -1, vp_y_idx = -1, vp_z_idx = -1, distance_idx = -1;
  for (int d = 0; d < static_cast<int> (point_cloud_data.fields.size ()); ++d)
  {
    if (point_cloud_data.fields[d].name == "x") x_idx = d;
    if (point_cloud_data.fields[d].name == "y") y_idx = d;
    if (point_cloud_data.fields[d].name == "z") z_idx = d;
    if (point_cloud_data.fields[d].name == "vp_x") vp_x_idx = d;
    if (point_cloud_data.fields[d].name == "vp_y") vp_y_idx = d;
    if (point_cloud_data.fields[d].name == "vp_z") vp_z_idx = d;
    if (point_cloud_data.fields[d].name == "distance") distance_idx = d;
  }
  
  if (x_idx<0 || y_idx<0 || z_idx<0 || vp_x_idx<0 || vp_y_idx<0 || vp_z_idx<0 || distance_idx<0)
  {
    return;
  }
  
  int point_step = point_cloud_data.point_step;
  const unsigned char* data = &point_cloud_data.data[0];
  int x_offset = point_cloud_data.fields[x_idx].offset,
      y_offset = point_cloud_data.fields[y_idx].offset,
      z_offset = point_cloud_data.fields[z_idx].offset,
      vp_x_offset = point_cloud_data.fields[vp_x_idx].offset,
      vp_y_offset = point_cloud_data.fields[vp_y_idx].offset,
      vp_z_offset = point_cloud_data.fields[vp_z_idx].offset,
      distance_offset = point_cloud_data.fields[distance_idx].offset;
  
  for (uindex_t point_idx = 0; point_idx < point_cloud_data.width*point_cloud_data.height; ++point_idx)
  {
    float x = *reinterpret_cast<const float*> (data+x_offset), 
          y = *reinterpret_cast<const float*> (data+y_offset), 
          z = *reinterpret_cast<const float*> (data+z_offset),
          vp_x = *reinterpret_cast<const float*> (data+vp_x_offset),
          vp_y = *reinterpret_cast<const float*> (data+vp_y_offset),
          vp_z = *reinterpret_cast<const float*> (data+vp_z_offset),
          distance = *reinterpret_cast<const float*> (data+distance_offset);
    data+=point_step;
    
    if (!std::isfinite (x) && std::isfinite (distance))
    {
      PointWithViewpoint point;
      point.x=distance; point.y=y; point.z=z;
      point.vp_x=vp_x; point.vp_y=vp_y; point.vp_z=vp_z;
      far_ranges.push_back (point);
    }
  }
  far_ranges.width= far_ranges.size ();  far_ranges.height = 1;
  far_ranges.is_dense = false;
}

/////////////////////////////////////////////////////////////////////////
float
RangeImage::getOverlap (const RangeImage& other_range_image, const Eigen::Affine3f& relative_transformation,
                        int search_radius, float max_distance, int pixel_step) const
{
  int hits_counter=0, valid_points_counter=0;
  
  float max_distance_squared = max_distance*max_distance;
  
#pragma omp parallel for \
  default(none) \
  shared(max_distance_squared, other_range_image, pixel_step, relative_transformation, search_radius) \
  schedule(dynamic, 1) \
  reduction(+ : valid_points_counter) \
  reduction(+ : hits_counter) \
  num_threads(max_no_of_threads)
  for (int other_y=0; other_y<int (other_range_image.height); other_y+=pixel_step)
  {
    for (int other_x=0; other_x<int (other_range_image.width); other_x+=pixel_step)
    {
      const PointWithRange& point = other_range_image.getPoint (other_x, other_y);
      if (!std::isfinite (point.range))
        continue;
      ++valid_points_counter;
      Eigen::Vector3f transformed_point = relative_transformation * point.getVector3fMap ();
      int x,y;
      getImagePoint (transformed_point, x, y);
      float closest_distance = max_distance_squared;
      Eigen::Vector3f closest_point (0.0f, 0.0f, 0.0f);
      bool found_neighbor = false;
      for (int y2=y-pixel_step*search_radius; y2<=y+pixel_step*search_radius; y2+=pixel_step)
      {
        for (int x2=x-pixel_step*search_radius; x2<=x+pixel_step*search_radius; x2+=pixel_step)
        {
          const PointWithRange& neighbor = getPoint (x2, y2);
          if (!std::isfinite (neighbor.range))
            continue;
          float distance = (transformed_point-neighbor.getVector3fMap ()).squaredNorm ();
          if (distance < closest_distance)
          {
            closest_distance = distance;
            closest_point = neighbor.getVector3fMap ();
            found_neighbor = true;
          }
        }
      }

      if (found_neighbor)
      {
        ++hits_counter;
      }
    }
  }
  return static_cast<float> (hits_counter)/static_cast<float> (valid_points_counter);
}

/////////////////////////////////////////////////////////////////////////
void
RangeImage::getBlurredImageUsingIntegralImage (int blur_radius, float* integral_image, int* valid_points_num_image,
                                               RangeImage& blurred_image) const
{
  this->copyTo(blurred_image);
  for (int y=0; y<int (height); ++y)
  {
    for (int x=0; x<int (width); ++x)
    {
      const PointWithRange& old_point = getPoint (x, y);
      PointWithRange& new_point = blurred_image.getPoint (x, y);
      if (!std::isfinite (old_point.range))
        continue;
      
      int top= (std::max) (-1, y-blur_radius-1), right = (std::min) (static_cast<int> (width)-1, x+blur_radius), bottom =
          (std::min) (static_cast<int> (height)-1, y+blur_radius), left= (std::max) (-1, x-blur_radius-1);
      
      float top_left_value=0, top_right_value=0,
            bottom_right_value=integral_image[bottom*width+right], bottom_left_value=0;
      int top_left_valid_points=0, top_right_valid_points=0,
          bottom_right_valid_points=valid_points_num_image[bottom*width+right], bottom_left_valid_points=0;
      if (left>=0)
      {
        bottom_left_value = integral_image[bottom*width+left];
        bottom_left_valid_points = valid_points_num_image[bottom*width+left];
        if (top>=0)
        {
          top_left_value = integral_image[top*width+left];
          top_left_valid_points = valid_points_num_image[top*width+left];
        }
      }
      if (top>=0)
      {
        top_right_value = integral_image[top*width+right];
        top_right_valid_points = valid_points_num_image[top*width+right];
      }
      int valid_points_num = bottom_right_valid_points + top_left_valid_points - bottom_left_valid_points -
                             top_right_valid_points;
      new_point.range = (bottom_right_value + top_left_value - bottom_left_value - top_right_value) / 
                        static_cast<float> (valid_points_num);
    }
  }
  blurred_image.recalculate3DPointPositions ();
}

/////////////////////////////////////////////////////////////////////////
void
RangeImage::getBlurredImage (int blur_radius, RangeImage& blurred_image) const
{
  //MEASURE_FUNCTION_TIME;
  
  if (blur_radius > 1)  // For a high blur radius it's faster to use integral images
  {
    float* integral_image;
    int* valid_points_num_image;
    getIntegralImage (integral_image, valid_points_num_image);
    getBlurredImageUsingIntegralImage (blur_radius, integral_image, valid_points_num_image, blurred_image);
    delete[] integral_image;
    delete[] valid_points_num_image;
    return;
  }
  
  this->copyTo(blurred_image);
  
  if (blur_radius==0)
    return;
  
  for (int y=0; y < static_cast<int> (height); ++y)
  {
    for (int x=0; x < static_cast<int> (width); ++x)
    {
      PointWithRange& new_point = blurred_image.getPoint (x, y);
      const PointWithRange& original_point = getPoint (x, y);
      if (!std::isfinite (original_point.range))
        continue;
      
      new_point.range = 0.0f;
      float weight_sum = 0.0f;
      for (int y2=y-blur_radius; y2<y+blur_radius; ++y2)
      {
        for (int x2=x-blur_radius; x2<x+blur_radius; ++x2)
        {
          if (!isValid (x2,y2))
            continue;
          new_point.range += getPoint (x2, y2).range;
          weight_sum += 1.0f;
        }
      }
      new_point.range /= weight_sum;
    }
  }
  blurred_image.recalculate3DPointPositions ();
}

/////////////////////////////////////////////////////////////////////////
void
RangeImage::copyTo (RangeImage& other) const
{
  other = *this;
}

}  // namespace end


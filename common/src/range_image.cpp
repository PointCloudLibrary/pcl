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
 */

/**
\author Bastian Steder
**/

#include <cstddef>
#include <Eigen/StdVector>
#include <iostream>
using std::cout;
using std::cerr;
#include <cmath>
#include <set>

#include <pcl/range_image/range_image.h>
#include <pcl/common/transformation_from_correspondences.h>

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
  if (!asin_lookup_table.empty())
    return;
  
  //MEASURE_FUNCTION_TIME;
  
  asin_lookup_table.resize(lookup_table_size);
  for (int i=0; i<lookup_table_size; ++i) {
    float value = float(i-lookup_table_size/2)/float(lookup_table_size/2);
    asin_lookup_table[i] = asinf(value);
    //std::cout << "asinf("<<value<<") = "<<asin_lookup_table[i]<<"\n";
  }
  
  atan_lookup_table.resize(lookup_table_size);
  for (int i=0; i<lookup_table_size; ++i) {
    float value = float(i-lookup_table_size/2)/float(lookup_table_size/2);
    atan_lookup_table[i] = atanf(value);
    //std::cout << "atanf("<<value<<") = "<<atan_lookup_table[i]<<"\n";
  }
  
  cos_lookup_table.resize(lookup_table_size);
  for (int i=0; i<lookup_table_size; ++i) 
  {
    float value = (float) (i*2.0f*M_PI/float(lookup_table_size-1));
    cos_lookup_table[i] = cosf(value);
    //std::cout << "cosf("<<value<<") = "<<cos_lookup_table[i]<<"\n";
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
  //std::cout << PVARN (transformation * Eigen::Vector3f (1,2,3));
}

/////////////////////////////////////////////////////////////////////////
RangeImage::RangeImage () : RangeImage::BaseClass ()
{
  createLookupTables();
  reset ();
  unobserved_point.x = unobserved_point.y = unobserved_point.z = std::numeric_limits<float>::quiet_NaN ();
  unobserved_point.range = -std::numeric_limits<float>::infinity ();
}

/////////////////////////////////////////////////////////////////////////
RangeImage::~RangeImage ()
{
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
RangeImage::createEmpty(float angular_resolution, const Eigen::Affine3f& sensor_pose,
                        RangeImage::CoordinateFrame coordinate_frame, float angle_width, float angle_height)
{
  angular_resolution_ = angular_resolution;
  angular_resolution_reciprocal_ = 1.0f / angular_resolution_;
  width = pcl_lrint(floor(angle_width*angular_resolution_reciprocal_));
  height = pcl_lrint(floor(angle_height*angular_resolution_reciprocal_));
  image_offset_x_ = image_offset_y_ = 0;  // TODO: FIX THIS
  is_dense = false;
  getCoordinateFrameTransformation(coordinate_frame, to_world_system_);
  to_world_system_ = sensor_pose * to_world_system_;
  to_range_image_system_ = to_world_system_.inverse ();
  unsigned int size = width*height;
  points.clear();
  points.resize(size, unobserved_point);
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::integrateFarRanges (const PointCloud<PointWithViewpoint>& far_ranges)
{
  float x_real, y_real, range_of_current_point;
  for (std::vector<PointWithViewpoint, Eigen::aligned_allocator<PointWithViewpoint> >::const_iterator it
       =far_ranges.points.begin (); it!=far_ranges.points.end (); ++it)
  {
    //if (!isFinite (*it))  // Check for NAN etc
      //continue;
    Vector3fMapConst current_point = it->getVector3fMap ();
    
    this->getImagePoint (current_point, x_real, y_real, range_of_current_point);
    
    int floor_x = pcl_lrint (floor (x_real)), floor_y = pcl_lrint (floor (y_real)),
        ceil_x  = pcl_lrint (ceil (x_real)),  ceil_y  = pcl_lrint (ceil (y_real));
    
    int neighbor_x[4], neighbor_y[4];
    neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
    neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
    neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
    neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;
    //std::cout << x_real<<","<<y_real<<": ";
    
    for (int i=0; i<4; ++i)
    {
      int x=neighbor_x[i], y=neighbor_y[i];
      if (!isInImage (x, y))
        continue;
      PointWithRange& image_point = getPoint (x, y);
      if (!pcl_isfinite (image_point.range))
        image_point.range = std::numeric_limits<float>::infinity ();
    }
  }
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::cropImage (int borderSize, int top, int right, int bottom, int left) {
  //MEASURE_FUNCTION_TIME;
  
  //std::cout << PVARC (top) << PVARC (right) << PVARC (bottom) << PVARN (left);
  bool topIsDone=true, rightIsDone=true, bottomIsDone=true, leftIsDone=true;
  if (top < 0) {
    top=-1;
    topIsDone=false;
  }
  if (right < 0) {
    right= (int)width;
   rightIsDone=false;
  }
  if (bottom < 0) {
    bottom= (int)height;
    bottomIsDone=false;
  }
  if (left < 0) {
    left=-1;
    leftIsDone=false;
  }
  
  // Find top border
  while (!topIsDone && top<=bottom) {
    ++top;
    int lineStart = top*width;
    for (int x=left; x<=right && !topIsDone; ++x)
      if (pcl_isfinite (points[lineStart + x].range))
        topIsDone = true;
  }
  // Check if range image is empty
  if (top > bottom) {
    std::cerr << __PRETTY_FUNCTION__ << ": Range image is empty.\n";
    points.clear ();
    width = height = 0;
    return;
  }
  // Find right border
  while (!rightIsDone) {
    for (int y=top; y<=bottom && !rightIsDone; ++y)
      if (pcl_isfinite (points[y*width + right].range))
        rightIsDone = true;
  }
  // Find bottom border
  while (!bottomIsDone) {
    --bottom;
    int lineStart = bottom*width;
    for (int x=left; x<=right && !bottomIsDone; ++x)
      if (pcl_isfinite (points[lineStart + x].range))
        bottomIsDone = true;
  } 
  // Find left border
  while (!leftIsDone) {
    ++left;
    for (int y=top; y<=bottom && !leftIsDone; ++y)
      if (pcl_isfinite (points[y*width + left].range))
        leftIsDone = true;
  } 
  //cout << "borderSize is "<<borderSize<<"\n";
  left-=borderSize; top-=borderSize; right+=borderSize; bottom+=borderSize;
  
  //cout << top<<","<<right<<","<<bottom<<","<<left<<"\n";
  
  // Create copy without copying the old points - vector::swap only copies a few pointers, not the content
  std::vector<PointWithRange, Eigen::aligned_allocator<PointWithRange> > tmpPoints;
  points.swap (tmpPoints);
  RangeImage oldRangeImage = *this;
  tmpPoints.swap (oldRangeImage.points);
  
  width = right-left+1; height = bottom-top+1;
  image_offset_x_ = left+oldRangeImage.image_offset_x_;
  image_offset_y_ = top+oldRangeImage.image_offset_y_;
  points.resize (width*height);
  
  // Copy points
  for (int y=0, oldY=top; y< (int)height; ++y,++oldY) {
    for (int x=0, oldX=left; x< (int)width; ++x,++oldX) {
      //cout << oldX<<","<<oldY <<" => "<< x<<","<<y<<"\n";
      PointWithRange& currentPoint = points[y*width + x];
      if (oldX<0 || oldX>= (int)oldRangeImage.width || oldY<0 || oldY>= (int)oldRangeImage.height) {
        currentPoint = unobserved_point;
        continue;
      }
      currentPoint = oldRangeImage.points[oldY*oldRangeImage.width + oldX];
      //cout << "Copying point "<<currentPoint<<".\n";
    }
  }
  //cout << "Uncropped range image has size "<<oldRangeImage.width<<"x"<<oldRangeImage.height<<".\n";
  //cout << "Cropped range image has size "<<width<<"x"<<height<<".\n";
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::recalculate3DPointPositions () 
{
  //MEASURE_FUNCTION_TIME;
  for (int y=0; y< (int)height; ++y) {
    for (int x=0; x< (int)width; ++x) {
      PointWithRange& point = points[y*width + x];
      if (!pcl_isinf (point.range)) {
        calculate3DPoint ((float) x, (float) y, point.range, point);
        //int x2,y2;
        //getImagePoint (point.x, point.y, point.z, x2, y2);
        //if (x2!=x || y2!=y)
          //cout << PVARC (x)<<PVARC (y)<<PVARC (x2)<<PVARN (y2);
      }
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
  for (int y=0; y< (int)height; ++y)
  {
    for (int x=0; x< (int)width; ++x)
    {
      float& integral_pixel = * (integral_image_ptr++);
      integral_pixel = getPoint (x, y).range;
      int& valid_points_num = * (valid_points_num_image_ptr++);
      valid_points_num = 1;
      if (pcl_isinf (integral_pixel))
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
RangeImage::getBlurredImageUsingIntegralImage (int blur_radius, float* integral_image, int* valid_points_num_image,
                                               RangeImage& blurred_image) const
{
  blurred_image = *this;
  for (int y=0; y<int (height); ++y)
  {
    for (int x=0; x<int (width); ++x)
    {
      const PointWithRange& old_point = getPoint (x, y);
      PointWithRange& new_point = blurred_image.getPoint (x, y);
      if (!pcl_isfinite (old_point.range))
        continue;
      
      int top= (std::max) (-1, y-blur_radius-1), right = (std::min) (int (width)-1, x+blur_radius), bottom =
          (std::min) (int (height)-1, y+blur_radius), left= (std::max) (-1, x-blur_radius-1);
      
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
      new_point.range = (bottom_right_value + top_left_value - bottom_left_value - top_right_value) / valid_points_num;
      //cout << PVARC (new_point.range)<<PVARC (top_left_value)<<PVARC (top_right_value)<<PVARC (bottom_left_value)
      //     << PVARC (bottom_right_value);
    }
  }
  blurred_image.recalculate3DPointPositions ();
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getBlurredImage (int blur_radius, RangeImage& blurred_image) const
{
  MEASURE_FUNCTION_TIME;
  
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
  
  blurred_image = *this;
  
  if (blur_radius==0)
    return;
  
  for (int y=0; y<int (height); ++y)
  {
    for (int x=0; x<int (width); ++x)
    {
      PointWithRange& new_point = blurred_image.getPoint (x, y);
      const PointWithRange& original_point = getPoint (x, y);
      if (!pcl_isfinite (original_point.range))
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
RangeImage::setUnseenToMaxRange ()
{
  for (unsigned int i=0; i<points.size (); ++i)
    if (pcl_isinf (points[i].range))
      points[i].range = std::numeric_limits<float>::infinity ();
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getHalfImage (RangeImage& half_image) const
{
  half_image.angular_resolution_ = 2.0f * angular_resolution_;
  half_image.image_offset_x_ = image_offset_x_/2;
  half_image.image_offset_y_ = image_offset_y_/2;
  half_image.width  = width/2;
  half_image.height = height/2;
  half_image.is_dense = is_dense;
  half_image.points.clear ();
  half_image.points.resize (half_image.width*half_image.height);
  
  int src_start_x = 2*half_image.image_offset_x_ - image_offset_x_,
      src_start_y = 2*half_image.image_offset_y_ - image_offset_y_;
  
  for (int dst_y=0; dst_y< (int)half_image.height; ++dst_y)
  {
    for (int dst_x=0; dst_x< (int)half_image.width; ++dst_x)
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
          if (pcl_isfinite (dst_point.range) && src_point.range > dst_point.range)
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
  sub_image.angular_resolution_ = angular_resolution_*combine_pixels;
  sub_image.image_offset_x_ = sub_image_image_offset_x;
  sub_image.image_offset_y_ = sub_image_image_offset_y;
  sub_image.width = sub_image_width;
  sub_image.height = sub_image_height;
  sub_image.is_dense = is_dense;
  sub_image.points.clear ();
  sub_image.points.resize (sub_image.width*sub_image.height);
  
  int src_start_x = combine_pixels*sub_image.image_offset_x_ - image_offset_x_,
      src_start_y = combine_pixels*sub_image.image_offset_y_ - image_offset_y_;
  
  for (int dst_y=0; dst_y< (int)sub_image.height; ++dst_y)
  {
    for (int dst_x=0; dst_x< (int)sub_image.width; ++dst_x)
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
          if (pcl_isfinite (dst_point.range) && src_point.range > dst_point.range)
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
  for (unsigned int i=0; i<points.size (); ++i)
  {
    float range = points[i].range;
    if (!pcl_isfinite (range))
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
  //MEASURE_FUNCTION_TIME;
  //if (debug) cout << __PRETTY_FUNCTION__<<" called.\n";
  
  float max_dist = 0.5f*world_size,
        cell_size = world_size/float (pixel_size);
  float world2cell_factor = 1.0f/cell_size,
        world2cell_offset = 0.5f*float (pixel_size)-0.5f;
  float cell2world_factor = cell_size,
        cell2world_offset = -max_dist + 0.5f*cell_size;
  Eigen::Affine3f inverse_pose = pose.inverse ();
  
  int no_of_pixels = pixel_size*pixel_size;
  float* surface_patch = new float[no_of_pixels];
  SET_ARRAY (surface_patch, -std::numeric_limits<float>::infinity (), no_of_pixels);
  
  //float min_x_f=width-1, max_x_f=0, min_y_f=height-1, max_y_f=0;
  //Eigen::Vector3f dest_point (max_dist, max_dist, max_dist);
  //for (int x=0; x<=1; ++x)
  //{
    //dest_point[0] = -dest_point[0];
    //for (int y=0; y<=1; ++y)
    //{
      //dest_point[1] = -dest_point[1];
      //for (int z=0; z<=1; ++z)
      //{
        //dest_point[2] = -dest_point[2];
        //Eigen::Vector3f src_point = inverse_pose * dest_point;
        //float image_x, image_y;
        //getImagePoint (src_point, image_x, image_y);
        //min_x_f = (std::min) (min_x_f, image_x);
        //max_x_f = (std::max) (max_x_f, image_x);
        //min_y_f = (std::min) (min_y_f, image_y);
        //max_y_f = (std::max) (max_y_f, image_y);
      //}
    //}
  //}
  //int min_x = max (0, int (pcl_lrint (floor (min_x_f))-1)), max_x = (std::min) (int (width)-1,
  //                                                                          int (pcl_lrint (ceil (max_x_f))+1)),
      //min_y = max (0, int (pcl_lrint (floor (min_y_f))-1)), max_y = (std::min) (int (height)-1,
      //                                                                      int (pcl_lrint (ceil (max_y_f))+1));
  //cout << "Searching through range image area of size "<<max_x-min_x<<"x"<<max_y-min_y<<".\n";
  
  Eigen::Vector3f position = inverse_pose.translation ();
  int middle_x, middle_y;
  getImagePoint (position, middle_x, middle_y);
  int min_search_radius = 2;
  bool still_in_range = true;
  //cout << "Starting radius search around "<<middle_x<<","<<middle_y<<"\n";
  for (int radius=0;  still_in_range;  ++radius) 
  {
    //cout << PVARN (radius);
    int x=middle_x-radius-1, y=middle_y-radius;  // Top left - 1
    still_in_range = radius<min_search_radius;
    for (int i=0; i<8*radius || (radius==0&&i==0); ++i)
    {
      if (i<=2*radius) ++x; else if (i<=4*radius) ++y; else if (i<=6*radius) --x; else --y;
      //cout << x<<","<<y<<"  ";

      Eigen::Vector3f point1, point2, point3;
      if (!isValid (x,y) || !isValid (x+1,y+1))
        continue;
      getPoint (x, y, point1);
      point1 = pose*point1;
      if (fabs (point1[2]) > max_dist)
        continue;
      
      getPoint (x+1, y+1, point2);
      point2 = pose*point2;
      if (fabs (point2[2]) > max_dist)
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
        if (fabs (point3[2]) > max_dist)
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
        
        int min_cell_x = (std::max) (0, int (pcl_lrint (ceil ( (std::min) (cell1_x, (std::min) (cell2_x, cell3_x)))))),
            max_cell_x = (std::min) (pixel_size-1, int (pcl_lrint (floor ( (std::max) (cell1_x,
                                                                       (std::max) (cell2_x, cell3_x)))))),
            min_cell_y = (std::max) (0, int (pcl_lrint (ceil ( (std::min) (cell1_y, (std::min) (cell2_y, cell3_y)))))),
            max_cell_y = (std::min) (pixel_size-1, int (pcl_lrint (floor ( (std::max) (cell1_y,
                                                                       (std::max) (cell2_y, cell3_y))))));
        if (max_cell_x<min_cell_x || max_cell_y<min_cell_y)
          continue;
        
        //cout << "Current triangle covers area of size "<<max_cell_x-min_cell_x<<"x"<<max_cell_y-min_cell_y<<".\n";
        
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
            
            //cout << "Is point ("<<current_cell[0]<<","<<current_cell[1]<<") in triangle "
                 //<< " ("<<cell1[0]<<","<<cell1[1]<<"), "
                 //<< " ("<<cell2[0]<<","<<cell2[1]<<"), "
                 //<< " ("<<cell3[0]<<","<<cell3[1]<<"): "
                 //<< (point_in_triangle ? "Yes":"No") << "\n";
            
            if (!point_in_triangle)
              continue;
            
            float new_value = cell1_z + u* (cell3_z-cell1_z) + v* (cell2_z-cell1_z);
            //if (fabs (new_value) > max_dist)
              //cerr << "WTF:"<<PVARN (new_value)<<"\n";
            
            float& value = surface_patch[cell_y*pixel_size + cell_x];
            if (pcl_isinf (value))
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
      if (!pcl_isinf (value))
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
          if (pcl_isfinite (neighbor_value))
          {
            float cell_pos_x = cell_x + 0.6f* (cell_x-cell2_x),
                  cell_pos_y = cell_y + 0.6f* (cell_y-cell2_y);
            Eigen::Vector3f fake_point (cell2world_factor* (cell_pos_x)+cell2world_offset,
                                        cell2world_factor*cell_pos_y+cell2world_offset, neighbor_value);
            fake_point = inverse_pose*fake_point;
            float range_difference = getRangeDifference (fake_point);
            if (range_difference > max_dist)
            {
              //if (debug) cout << PVARN (range_difference);
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
            if (pcl_isinf (neighbor_value) && neighbor_value<0)
            {
              neighbor_value = std::numeric_limits<float>::infinity ();
            }
          }
        }
      }
    }
  }
  
  return surface_patch;
}

//float* RangeImage::getInterpolatedSurfaceProjection (const Eigen::Affine3f& pose, int pixel_size, float world_size) const
//{
  ////MEASURE_FUNCTION_TIME;
  ////cout << PVARC (pixel_size)<<PVARN (world_size);
  
  //int min_search_radius = 2;
  
  //Eigen::Affine3f inverse_pose = getInverse (pose);
  //Eigen::Vector3f position = getTranslation (inverse_pose);
  
  //float max_dist  = 0.5f*world_size;
  
  //int no_of_pixels = pixel_size*pixel_size;
  //float* surface_patch = new float[no_of_pixels];
  //SET_ARRAY (surface_patch, -std::numeric_limits<float>::infinity (), no_of_pixels);
  
  //float* weights = new float[no_of_pixels];
  //SET_ARRAY (weights, 0.0f, no_of_pixels);
  
  //float cell_size = world_size/float (pixel_size),
        //cell_factor = 1.0f/cell_size,
        //cell_offset = max_dist - 0.5f*cell_size;
  
  //int x, y;
  //float middle_range;
  //getImagePoint (position, x, y, middle_range);
  
  //float min_range = middle_range - 1.5f*max_dist,
        //max_range = middle_range + 1.5f*max_dist;
  
  //float max_dist_from_minimum = 0.2*max_dist;
  
  //float pixel_radius_factor = sinf (0.5f*angular_resolution_);
  
  //bool still_in_range = true;
  //for (int radius=0;  still_in_range;  ++radius) 
  //{
    //int x2=x-radius-1, y2=y-radius;  // Top left - 1
    //still_in_range = radius<min_search_radius;
    //for (int i=0; i<8*radius || (radius==0&&i==0); ++i)
    //{
      //if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
      //if (!isInImage (x2, y2))
        //continue;
      //const PointWithRange& point = getPoint (x2, y2);
      //if (point.range < min_range || point.range > max_range)
        //continue;
      ////calculate3DPoint (x2, y2, max_range, point);
      
      //float pixel_radius = pixel_radius_factor*point.range;
      //int interpolation_radius = pcl_lrint (pixel_radius/cell_size);
      ////cout << PVARN (interpolation_radius);
      
      //Eigen::Vector3f transformed_point = pose*Eigen::Vector3f (point.x, point.y, point.z);
      //float cell_x = cell_factor * (transformed_point[0]+cell_offset),
            //cell_y = cell_factor * (transformed_point[1]+cell_offset),
            //value = transformed_point[2];
      //int cell_x_int=pcl_lrint (cell_x), cell_y_int=pcl_lrint (cell_y);
      //for (int cell_to_update_y=cell_y_int-interpolation_radius; cell_to_update_y<=cell_y_int+interpolation_radius; ++cell_to_update_y)
      //{
        //for (int cell_to_update_x=cell_x_int-interpolation_radius; cell_to_update_x<=cell_x_int+interpolation_radius; ++cell_to_update_x)
        //{
          //if (cell_to_update_x<0 || cell_to_update_x>= (int)pixel_size || cell_to_update_y<0 || cell_to_update_y>= (int)pixel_size || fabs (value)>max_dist)
            //continue;
          //still_in_range = true;
          //float current_weight = hypot (cell_to_update_x-cell_x, cell_to_update_y-cell_y)/ ( (interpolation_radius+0.5f)*sqrtf (2.0f));
          //int index = cell_to_update_y*pixel_size + cell_to_update_x;
          //float& cell = surface_patch[index];
          //float& weight = weights[index];
          ////cout << PVARC (value)<<PVARC (cell);
          //if (!pcl_isfinite (cell) || value < cell-max_dist_from_minimum)
          //{
            //cell = value;
            //weight = 1.0f;
          //}
          //else if (value <= cell+max_dist_from_minimum)
          //{
            //weight += current_weight;
            //cell += (value-cell)/weight;
          //}
          ////cout << PVARN (cell);
        //}
      //}
    //}
  //}
  
  //bool something_changed = true;
  //while (something_changed)
  //{
    //something_changed = false;
    //for (int y=0; y<pixel_size; ++y)
    //{
      //for (int x=0; x<pixel_size; ++x)
      //{
        //float& value = surface_patch[y*pixel_size + x];
        //if (!pcl_isinf (value) || value>0.0f)
          //continue;
        //bool interpolation_done=false;
        //for (int y2=y-1; y2<=y+1&&!interpolation_done; ++y2)
        //{
          //for (int x2=x-1; x2<=x+1; ++x2)
          //{
            //if (x2<0||x2>=pixel_size||y2<0||y2>=pixel_size || (x2==x && y2==y))
              //continue;
            //float& neighbor_value = surface_patch[y2*pixel_size + x2];
            //if (!pcl_isfinite (neighbor_value))
            //{
              //if (neighbor_value>0)
              //{
                //value = std::numeric_limits<float>::infinity ();
                //something_changed = true;
                //interpolation_done = true;
              //}
              //continue;
            //}
            //Eigen::Vector3f fake_point (x*cell_size-cell_offset, y*cell_size-cell_offset, neighbor_value);
            //fake_point = inverse_pose*fake_point;
            //int image_x, image_y;
            //getImagePoint (fake_point, image_x, image_y);
            //if (!isInImage (image_x, image_y))
              //continue;
            //const PointWithRange& point = getPoint (image_x, image_y);
            //if (point.range < min_range)
              //continue;
            //if (point.range > max_range)
            //{
              //value = std::numeric_limits<float>::infinity ();
            //}
            //else
            //{
              //Eigen::Vector3f transformed_point = pose*Eigen::Vector3f (point.x, point.y, point.z);
              //float new_value = transformed_point[2];
              //if (new_value < -max_dist)
                //continue;
              //else if (new_value > max_dist)
                //value = std::numeric_limits<float>::infinity ();
              //else {
                //int new_cell_x = pcl_lrint (cell_factor * (transformed_point[0]+cell_offset)),
                    //new_cell_y = pcl_lrint (cell_factor * (transformed_point[1]+cell_offset));
                //if (fabs (new_cell_x-x)<=1 && fabs (new_cell_y-y)<=1)
                  //value = new_value;
                //else
                  //continue;
              //}
            //}
            //something_changed = true;
            //interpolation_done = true;
            //break;
          //}
        //}
      //}
    //}
  //}
  //delete weights;
  
  //return surface_patch;
//}

//float* RangeImage::getInterpolatedSurfaceProjection (const Eigen::Affine3f& pose, int pixel_size, float world_size) const
//{
  ////MEASURE_FUNCTION_TIME;
  
  //int raytracing_steps_max = 100;
  
  //float* surface_patch = new float[pixel_size*pixel_size];
  ////SET_ARRAY (surface_patch, -std::numeric_limits<float>::infinity (), pixel_size);
  //Eigen::Affine3f inverse_pose = getInverse (pose);
  
  //float max_dist = 0.5f*world_size,
        //cell_size = world_size/float (pixel_size),
        //cell2world_factor = cell_size,
        //cell2world_offset = -max_dist + 0.5f*cell_size,
        //world2cell_factor = 1.0f/cell_size,
        //world2cell_offset = 0.5f*float (pixel_size)-0.5f;
  
  //for (int y=0; y<pixel_size; ++y)
  //{
    //for (int x=0; x<pixel_size; ++x)
    //{
      //float& value = surface_patch[y*pixel_size + x];
      //value = -std::numeric_limits<float>::infinity ();
      //Eigen::Vector3f raytracing_point (cell2world_factor*x+cell2world_offset, cell2world_factor*y+cell2world_offset, max_dist),
                      //raytracing_point_tranformed = inverse_pose * raytracing_point;
      //int raytracing_end_x, raytracing_end_y;
      //getImagePoint (raytracing_point_tranformed, raytracing_end_x, raytracing_end_y);
      //raytracing_point[2] = -max_dist;
      //raytracing_point_tranformed = inverse_pose * raytracing_point;
      //int raytracing_start_x, raytracing_start_y;
      //getImagePoint (raytracing_point_tranformed, raytracing_start_x, raytracing_start_y);
      //int raytracing_steps = (std::min) (raytracing_steps_max, 1+int (pcl_lrint (hypot (raytracing_end_x-raytracing_start_x, raytracing_end_y-raytracing_start_y))));
      ////cout << " ("<<raytracing_start_x<<","<<raytracing_start_y<<") -> ("<<raytracing_end_x<<","<<raytracing_end_y<<") => " << PVARN (raytracing_steps);
      
      //float raytracing_step_size = world_size/ (raytracing_steps-1);
      
      //int unknown_points = 0,
          //empty_points = 0;
      //for (int raytracing_step=0; raytracing_step<raytracing_steps; ++raytracing_step)
      //{
        //float current_value = raytracing_point[2];
        //raytracing_point_tranformed = inverse_pose * raytracing_point;
        //raytracing_point[2] += raytracing_step_size;
        
        //PointWithRange image_point;
        //float raytracing_point_range = checkPoint (raytracing_point_tranformed, image_point);
        
        //if (pcl_isinf (image_point.range))
        //{
          //if (image_point.range < 0)
            //++unknown_points;
          //else
            //++empty_points;
          //continue;
        //}
        
        //Eigen::Vector3f image_point_transformed = pose*Eigen::Vector3f (image_point.x, image_point.y, image_point.z);
        //float image_point_value = image_point_transformed[2];
        
        //int image_point_cell_x = pcl_lrint (world2cell_factor*image_point_transformed[0] + world2cell_offset),
            //image_point_cell_y = pcl_lrint (world2cell_factor*image_point_transformed[1] + world2cell_offset);
        
        //if (image_point_cell_x!=x && image_point_cell_y!=y && fabs (current_value-image_point_value) > 0.5f*raytracing_step_size)
        //{
          //if (image_point.range < raytracing_point_range)
            //++unknown_points;
          //else
            //++empty_points;
          //continue;
        //}
        
        //value = image_point_value;
        //break;
      //}
      ////if (pcl_isinf (value))
        ////cout << PVARC (empty_points)<<PVARN (unknown_points);
      //if (pcl_isinf (value))
      //{
        //if (empty_points > 0)
          //value = std::numeric_limits<float>::infinity ();
        //continue;
      //}
      
      //if (value < -max_dist)
        //value = -std::numeric_limits<float>::infinity ();
      //else if (value > max_dist)
        //value = std::numeric_limits<float>::infinity ();
    //}
  //}
  //return surface_patch;
//}

float* RangeImage::getInterpolatedSurfaceProjection (const Eigen::Vector3f& point, int pixel_size,
                                                     float world_size) const
{
  Eigen::Affine3f pose = getTransformationToViewerCoordinateFrame (point);
  return getInterpolatedSurfaceProjection (pose, pixel_size, world_size);
}


/////////////////////////////////////////////////////////////////////////
bool 
RangeImage::getNormalBasedUprightTransformation (const Eigen::Vector3f& point, float max_dist,
                                                 Eigen::Affine3f& transformation) const
{
  //MEASURE_FUNCTION_TIME;
  //cout << PVARN (max_dist);
  int x, y;
  getImagePoint (point, x, y);
  //cout << "Point is "<<x<<","<<y<<"\n";
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
        //cout << "v";
        continue;
      }
      getPoint (x2, y2, neighbor);
      float distance_squared = (neighbor-point).squaredNorm ();
      if (distance_squared > max_dist_squared)
      {
        //cout << "d";
        continue;
      }
      still_in_range = true;
      float distance = sqrtf (distance_squared),
            weight = distance*max_dist_reciprocal;
      vector_average.add (neighbor, weight);
    }
  }
  //cout << PVARN (vector_average.getNoOfSamples ());
  
  Eigen::Vector3f normal, point_on_plane;
  if (vector_average.getNoOfSamples () > 10)
  {
    Eigen::Vector3f eigen_values, eigen_vector2, eigen_vector3;
    vector_average.doPCA (eigen_values, normal, eigen_vector2, eigen_vector3);
    if (normal.dot ( (vector_average.getMean ()-getSensorPos ()).normalized ()) < 0.0f)
      normal *= -1.0f;
    point_on_plane = (normal.dot (vector_average.getMean ()) - normal.dot (point))*normal + point;
    //cout << "Using original normal.\n";
  }
  else
  {
    if (!getNormalForClosestNeighbors (x, y, 2, point, 15, normal, &point_on_plane, 1))
      return false;
    //cout << "Using k-search normal instead.\n";
  }
  //RangeImage::getTransformationFromTwoUnitVectorsAndOrigin (Eigen::Vector3f (0.0f, -1.0f, 0.0f),
  //                                                          normal, point, transformation);
  getTransformationFromTwoUnitVectorsAndOrigin (Eigen::Vector3f (0.0f, 1.0f, 0.0f),
                                                normal, point_on_plane, transformation);
  
  return true;
}

//float* RangeImage::getSurfaceChangeImage (int radius) const
//{
  //MEASURE_FUNCTION_TIME;
  //int size = width*height;
  //float* surface_change_image = new float[size];
  //for (int y=0; y<int (height); ++y)
  //{
    //for (int x=0; x<int (width); ++x)
    //{
      //surface_change_image[y*width+x] = getSurfaceChange (x, y, radius);
    //}
  //}
  //return surface_change_image;
//}

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


//float RangeImage::getSurfaceChange (int x, int y, int radius) const
//{
  //if (!isValid (x,y))
    //return -std::numeric_limits<float>::infinity ();
  
  //bool point_left_is_valid   = isValid (x-radius, y),
       //point_right_is_valid  = isValid (x+radius, y),
       //point_top_is_valid    = isValid (x, y-radius),
       //point_bottom_is_valid = isValid (x, y+radius);
 //bool normal1_is_valid  =  point_top_is_valid    && point_right_is_valid,
      //normal2_is_valid  =  point_right_is_valid  && point_bottom_is_valid,
      //normal3_is_valid  =  point_bottom_is_valid && point_left_is_valid,
      //normal4_is_valid  =  point_left_is_valid   && point_top_is_valid;
  //int no_of_valid_normals = normal1_is_valid+normal2_is_valid+normal3_is_valid+normal4_is_valid;
  //if (no_of_valid_normals < 2)
    //return -std::numeric_limits<float>::infinity ();
  
  //// TODO: If INFINITY neighbor use sensor beam direction
  
  //Eigen::Vector3f point, point_left, point_right, point_top, point_bottom;
  //getPoint (x, y, point);
  //if (point_left_is_valid)
    //get1dPointAverage (x-1, y, -1,  0, radius, point_left);
  //if (point_right_is_valid)
    //get1dPointAverage (x+1, y,  1,  0, radius, point_right);
  //if (point_top_is_valid)
    //get1dPointAverage (x, y-1,  0, -1, radius, point_top);
  //if (point_bottom_is_valid)
    //get1dPointAverage (x, y+1,  0,  1, radius, point_bottom);
  
  //Eigen::Vector3f direction_left   = point_left-point,
                  //direction_right  = point_right-point,
                  //direction_top    = point_top-point,
                  //direction_bottom = point_bottom-point;
  
  //Eigen::Vector3f normal1 (0,0,0), normal2 (0,0,0), normal3 (0,0,0), normal4 (0,0,0);
  //if (normal1_is_valid)
    //normal1 = direction_right.cross (direction_top).normalized ();
  //if (normal2_is_valid)
    //normal2 = direction_bottom.cross (direction_right).normalized ();
  //if (normal3_is_valid)
    //normal3 = direction_left.cross (direction_bottom).normalized ();
  //if (normal4_is_valid)
    //normal4 = direction_top.cross (direction_left).normalized ();
  //// The direction of the normals is already facing to the observer because of the used order in the cross products

  //float normal_diff_sum = 0.0f,  normal_diff_sum_weight = 0.0f;
  //if (normal1_is_valid && normal2_is_valid)
  //{
    //normal_diff_sum += normal1.dot (normal2);
    //normal_diff_sum_weight += 1.0f;
  //}
  //if (normal1_is_valid && normal3_is_valid)
  //{
    //normal_diff_sum += normal1.dot (normal3);
    //normal_diff_sum_weight += 1.0f;
  //}
  //if (normal1_is_valid && normal4_is_valid)
  //{
    //normal_diff_sum += normal1.dot (normal4);
    //normal_diff_sum_weight += 1.0f;
  //}
  //if (normal2_is_valid && normal3_is_valid)
  //{
    //normal_diff_sum += normal2.dot (normal3);
    //normal_diff_sum_weight += 1.0f;
  //}
  //if (normal2_is_valid && normal4_is_valid)
  //{
    //normal_diff_sum += normal2.dot (normal4);
    //normal_diff_sum_weight += 1.0f;
  //}
  //if (normal3_is_valid && normal4_is_valid)
  //{
    //normal_diff_sum += normal3.dot (normal4);
    //normal_diff_sum_weight += 1.0f;
  //}
  //float average_normal_diff = normal_diff_sum/normal_diff_sum_weight;
  
  //Eigen::Vector3f average_normal (0,0,0);
  //if (normal1_is_valid)
    //average_normal += normal1;
  //if (normal2_is_valid)
    //average_normal += normal2;
  //if (normal3_is_valid)
    //average_normal += normal3;
  //if (normal4_is_valid)
    //average_normal += normal4;
  //average_normal.normalize ();
  
  ////float normal_diff_sum = 0.0f,  normal_diff_sum_weight = 0.0f;
  ////if (normal1_is_valid)
  ////{
    ////normal_diff_sum += average_normal.dot (normal1);
    ////normal_diff_sum_weight += 1.0f;
  ////}
  ////if (normal2_is_valid)
  ////{
    ////normal_diff_sum += average_normal.dot (normal2);
    ////normal_diff_sum_weight += 1.0f;
  ////}
  ////if (normal3_is_valid)
  ////{
    ////normal_diff_sum += average_normal.dot (normal3);
    ////normal_diff_sum_weight += 1.0f;
  ////}
  ////if (normal4_is_valid)
  ////{
    ////normal_diff_sum += average_normal.dot (normal4);
    ////normal_diff_sum_weight += 1.0f;
  ////}
  ////float average_normal_diff = normal_diff_sum/normal_diff_sum_weight;
  ////average_normal_diff = 1.0f - average_normal_diff;
  //average_normal_diff = acosf (average_normal_diff)/deg2rad (180.0f);
  
  //float is_convex_weight = 0.0f;
  //if (point_left_is_valid)
    //is_convex_weight -= direction_left.dot (average_normal);
  //if (point_right_is_valid)
    //is_convex_weight -= direction_right.dot (average_normal);
  //if (point_top_is_valid)
    //is_convex_weight -= direction_top.dot (average_normal);
  //if (point_bottom_is_valid)
    //is_convex_weight -= direction_bottom.dot (average_normal);
  
  //if (is_convex_weight < 0.0f)
    //average_normal_diff *= -1.0f;
  
  //return average_normal_diff;
//}

//void RangeImage::getSurfaceChange (int x, int y, int radius, float& surface_change_x, float& surface_change_y) const
//{
  //surface_change_x = surface_change_y = -std::numeric_limits<float>::infinity ();
  //if (!isValid (x,y) || !isObserved (x-radius,y-radius) || !isObserved (x+radius,y-radius) ||
  //    !isObserved (x-radius,y+radius) || !isObserved (x+radius,y+radius))
    //return;
  //const PointWithRange& point_top_left     = getPoint (x-radius,y-radius),
                      //& point_top_right    = getPoint (x+radius,y-radius),
                      //& point_bottom_left  = getPoint (x-radius,y+radius),
                      //& point_bottom_right = getPoint (x+radius,y+radius);
  //Eigen::Vector3f point;
  //getPoint (x, y, point);
  //Eigen::Vector3f sensor_beam_direction = point-getSensorPos ();
  
  //Eigen::Vector3f direction_top_left,
                  //direction_top_right,
                  //direction_bottom_left,
                  //direction_bottom_right;
  
  //if (pcl_isinf (point_top_left.range))
    //direction_top_left = sensor_beam_direction;
  //else
    //direction_top_left = getEigenVector3f (point_top_left)-point;
  
  //if (pcl_isinf (point_top_right.range))
    //direction_top_right = sensor_beam_direction;
  //else
    //direction_top_right = getEigenVector3f (point_top_right)-point;
  
  //if (pcl_isinf (point_bottom_left.range))
    //direction_bottom_left = sensor_beam_direction;
  //else
    //direction_bottom_left = getEigenVector3f (point_bottom_left)-point;
   
  //if (pcl_isinf (point_bottom_right.range))
    //direction_bottom_right = sensor_beam_direction;
  //else
    //direction_bottom_right = getEigenVector3f (point_bottom_right)-point;
  
  //Eigen::Vector3f normal_left   = direction_top_left.cross (direction_bottom_left).normalized (),
                  //normal_right  = direction_bottom_right.cross (direction_top_right).normalized (),
                  //normal_top    = direction_top_right.cross (direction_top_left).normalized (),
                  //normal_bottom = direction_bottom_left.cross (direction_bottom_right).normalized (); 
  //// The direction of the normals is already facing to the observer because of the used order in the cross products
  
  //float normal_diff_left_right = acosf (normal_left.dot (normal_right))/deg2rad (180.0f);
  //Eigen::Vector3f average_normal_left_right = (normal_left+normal_right).normalized ();
  //float is_convex_weight_left_right = - direction_top_left.dot (average_normal_left_right)
                                      //- direction_top_right.dot (average_normal_left_right)
                                      //- direction_bottom_left.dot (average_normal_left_right)
                                      //- direction_bottom_right.dot (average_normal_left_right);
  //if (is_convex_weight_left_right < 0.0f)
    //normal_diff_left_right *= -1.0f;
  
  //float normal_diff_top_bottom = acosf (normal_top.dot (normal_bottom))/deg2rad (180.0f);
  //Eigen::Vector3f average_normal_top_bottom = (normal_top+normal_bottom).normalized ();
  //float is_convex_weight_top_bottom = - direction_top_left.dot (average_normal_top_bottom)
                                      //- direction_top_right.dot (average_normal_top_bottom)
                                      //- direction_bottom_left.dot (average_normal_top_bottom)
                                      //- direction_bottom_right.dot (average_normal_top_bottom);
  //if (is_convex_weight_top_bottom < 0.0f)
    //normal_diff_top_bottom *= -1.0f;
  
  //surface_change_x = normal_diff_left_right;
  //surface_change_y = normal_diff_top_bottom;
//}

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
      //if (pcl_isfinite (acuteness_value_image_x[index]) && fabsf (acuteness_value_image_x[index])>1)
        //cerr << PVARN (acuteness_value_image_x[index]);
      //if (pcl_isfinite (acuteness_value_image_y[index]) && fabsf (acuteness_value_image_y[index])>1)
        //cerr << PVARN (acuteness_value_image_y[index]);
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

//void RangeImage::getLocalNormals (int radius) const
//{
  //MEASURE_FUNCTION_TIME;
  
  //int no_of_nearest_neighbors = pow (radius+1, 2);
  
  //Eigen::Vector3f normal;
  
  //int size = width*height;
  //#pragma omp parallel for default (shared) schedule (dynamic, 50)
  //for (int y=0; y<int (height); ++y)
  //{
    //for (int x=0; x<int (width); ++x)
    //{
      //const PointWithRange& point = getPoint (x, y);
      //if (pcl_isinf (point.range))
        //continue;
      //getNormalForClosestNeighbors (x, y, radius, point, no_of_nearest_neighbors, normal, 1);
    //}
  //}
//}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::getRangeImageWithSmoothedSurface (int radius, RangeImage& smoothed_range_image) const
{
  //MEASURE_FUNCTION_TIME;
  
  int step_size = (std::max) (1, radius/2);
  //cout << PVARN (step_size);
  int no_of_nearest_neighbors = (int) pow ( (double) (radius/step_size + 1), 2.0);
  
  smoothed_range_image = *this;
  Eigen::Vector3f sensor_pos = getSensorPos ();
  for (int y=0; y<int (height); ++y)
  {
    for (int x=0; x<int (width); ++x)
    {
      PointWithRange& point = smoothed_range_image.getPoint (x, y);
      if (pcl_isinf (point.range))
        continue;
      Eigen::Vector3f normal, mean, eigen_values;
      float used_squared_max_distance;
      getSurfaceInformation (x, y, radius, point.getVector3fMap (), no_of_nearest_neighbors,
                             step_size, used_squared_max_distance,
                             normal, mean, eigen_values);
      
      Eigen::Vector3f viewing_direction = (point.getVector3fMap ()-sensor_pos).normalized ();
      float new_range = normal.dot (mean-sensor_pos) / normal.dot (viewing_direction);
      point.range = new_range;
      calculate3DPoint ((float) x, (float) y, point.range, point);
      
      const PointWithRange& original_point = getPoint (x, y);
      float distance_squared = squaredEuclideanDistance (original_point, point);
      if (distance_squared > used_squared_max_distance)
        point = original_point;
    }
  }
}

/////////////////////////////////////////////////////////////////////////
void 
RangeImage::extractFarRanges (const sensor_msgs::PointCloud2& point_cloud_data,
                              PointCloud<PointWithViewpoint>& far_ranges)
{
  int x_idx = -1, y_idx = -1, z_idx = -1,
      vp_x_idx = -1, vp_y_idx = -1, vp_z_idx = -1, distance_idx = -1;
  for (size_t d = 0; d < point_cloud_data.fields.size (); ++d)
  {
    if (point_cloud_data.fields[d].name == "x") x_idx = (int) d;
    if (point_cloud_data.fields[d].name == "y") y_idx = (int) d;
    if (point_cloud_data.fields[d].name == "z") z_idx = (int) d;
    if (point_cloud_data.fields[d].name == "vp_x") vp_x_idx = (int) d;
    if (point_cloud_data.fields[d].name == "vp_y") vp_y_idx = (int) d;
    if (point_cloud_data.fields[d].name == "vp_z") vp_z_idx = (int) d;
    if (point_cloud_data.fields[d].name == "distance") distance_idx = (int) d;
  }
  
  if (x_idx<0 || y_idx<0 || z_idx<0 || vp_x_idx<0 || vp_y_idx<0 || vp_z_idx<0 || distance_idx<0)
  {
    //cout << PVARC (x_idx)<<PVARC (y_idx)<<PVARC (z_idx)<<PVARC (vp_x_idx)
    //     << PVARC (vp_y_idx)<<PVARC (vp_z_idx)<<PVARN (distance_idx);
    return;
  }
  //cout << "Point cloud might have far ranges\n";
  
  int point_step = point_cloud_data.point_step;
  const unsigned char* data = &point_cloud_data.data[0];
  int x_offset = point_cloud_data.fields[x_idx].offset,
      y_offset = point_cloud_data.fields[y_idx].offset,
      z_offset = point_cloud_data.fields[z_idx].offset,
      vp_x_offset = point_cloud_data.fields[vp_x_idx].offset,
      vp_y_offset = point_cloud_data.fields[vp_y_idx].offset,
      vp_z_offset = point_cloud_data.fields[vp_z_idx].offset,
      distance_offset = point_cloud_data.fields[distance_idx].offset;
  
  for (size_t point_idx = 0; point_idx < point_cloud_data.width*point_cloud_data.height; ++point_idx)
  {
    float x = * (float*) (data+x_offset), y = * (float*) (data+y_offset), z = * (float*) (data+z_offset),
          vp_x = * (float*) (data+vp_x_offset),
          vp_y = * (float*) (data+vp_y_offset),
          vp_z = * (float*) (data+vp_z_offset),
          distance = * (float*) (data+distance_offset);
    data+=point_step;
    
    if (!pcl_isfinite (x) && pcl_isfinite (distance))
    {
      //std::cout << "Found max range.\n";
      PointWithViewpoint point;
      point.x=distance; point.y=y; point.z=z;
      point.vp_x=vp_x; point.vp_y=vp_y; point.vp_z=vp_z;
      far_ranges.points.push_back (point);
    }
  }
  far_ranges.width= (uint32_t) far_ranges.points.size ();  far_ranges.height = 1;
  far_ranges.is_dense = false;
}

Eigen::Affine3f RangeImage::doIcp (const RangeImage::VectorOfEigenVector3f& points,
                                   const Eigen::Affine3f& initial_guess, int search_radius, 
                                   float max_distance_start, float max_distance_end, int num_iterations) const
{
  Eigen::Affine3f ret = initial_guess;
  
  float max_distance = max_distance_start, 
        max_distance_reduction = (max_distance_start-max_distance_end)/float (num_iterations);
  
  TransformationFromCorrespondences transformation_from_correspondeces;
  for (int iteration=1; iteration<=num_iterations; ++iteration)
  {
    float max_distance_squared = max_distance*max_distance;
    transformation_from_correspondeces.reset ();
    # pragma omp parallel for num_threads(max_no_of_threads) default(shared) schedule(dynamic, 100)
    for (int point_idx=0; point_idx< (int)points.size (); ++point_idx)
    {
      const Eigen::Vector3f& point = points[point_idx];
      Eigen::Vector3f transformed_point = ret * point;
      int x,y;
      getImagePoint (transformed_point, x, y);
      float closest_distance = max_distance_squared;
      Eigen::Vector3f closest_point (0.0f, 0.0f, 0.0f);
      bool found_neighbor = false;
      for (int y2=y-search_radius; y2<=y+search_radius; ++y2)
      {
        for (int x2=x-search_radius; x2<=x+search_radius; ++x2)
        {
          const PointWithRange& neighbor = getPoint (x2, y2);
          if (!pcl_isfinite (neighbor.range))
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
        //cout << PVARN (closest_distance);
#       pragma omp critical
        transformation_from_correspondeces.add (point, closest_point);
      }
    }
    //cout << PVARN (transformation_from_correspondeces.getNoOfSamples ());
    //cout << PVARN (iteration);
    if (transformation_from_correspondeces.getNoOfSamples () < 3)
      return ret;
    // TODO: check if change
    ret = transformation_from_correspondeces.getTransformation ();
    //cout << ret<<"\n";
    
    max_distance -= max_distance_reduction;
  }

  //cout << PVARN (initial_guess.matrix ())<<PVARN (ret.matrix ());
  
  return ret;
}

Eigen::Affine3f
RangeImage::doIcp (const RangeImage& other_range_image,
                   const Eigen::Affine3f& initial_guess, int search_radius,
                   float max_distance_start, float max_distance_end,
                   int num_iterations, int pixel_step_start, int pixel_step_end) const
{
  Eigen::Affine3f ret = initial_guess;
  
  float max_distance = max_distance_start, 
        max_distance_reduction = (max_distance_start-max_distance_end)/float (num_iterations);
  
  TransformationFromCorrespondences transformation_from_correspondeces;
  for (int iteration=1; iteration<=num_iterations; ++iteration)
  {
    float max_distance_squared = max_distance*max_distance;
    transformation_from_correspondeces.reset ();
    
    float progress = float(iteration)/float(num_iterations);
    int pixel_step = pcl_lrint (float(pixel_step_start) + powf(progress, 3)*float(pixel_step_end-pixel_step_start));
    //cout << PVARC(iteration) << PVARN(pixel_step);
    
    # pragma omp parallel for num_threads(max_no_of_threads) default(shared) schedule(dynamic, 1)
    for (int other_y=0; other_y<int(other_range_image.height); other_y+=pixel_step)
    {
      for (int other_x=0; other_x<int(other_range_image.width); other_x+=pixel_step)
      {
        const PointWithRange& point = other_range_image.getPoint (other_x, other_y);
        if (!pcl_isfinite (point.range))
          continue;
        Eigen::Vector3f transformed_point = ret * point.getVector3fMap();
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
            if (!pcl_isfinite (neighbor.range))
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
          //cout << PVARN (closest_distance);
#         pragma omp critical
          transformation_from_correspondeces.add (point.getVector3fMap(), closest_point);
        }
      }
    }
    //cout << PVARN (transformation_from_correspondeces.getNoOfSamples ());
    //cout << PVARN (iteration);
    if (transformation_from_correspondeces.getNoOfSamples () < 3)
      return ret;
    // TODO: check if change
    ret = transformation_from_correspondeces.getTransformation ();
    //cout << ret<<"\n";
    
    max_distance -= max_distance_reduction;
  }

  //cout << PVARN (initial_guess.matrix ())<<PVARN (ret.matrix ());
  
  return ret;
}

float
RangeImage::getOverlap (const RangeImage& other_range_image, const Eigen::Affine3f& relative_transformation,
                        int search_radius, float max_distance, int pixel_step) const
{
  int hits_counter=0, valid_points_counter=0;
  
  float max_distance_squared = max_distance*max_distance;
  
  # pragma omp parallel for num_threads(max_no_of_threads) default(shared) schedule(dynamic, 1) \
                        reduction(+ : valid_points_counter) reduction(+ : hits_counter)
  for (int other_y=0; other_y<int(other_range_image.height); other_y+=pixel_step)
  {
    for (int other_x=0; other_x<int(other_range_image.width); other_x+=pixel_step)
    {
      const PointWithRange& point = other_range_image.getPoint (other_x, other_y);
      if (!pcl_isfinite (point.range))
        continue;
      ++valid_points_counter;
      Eigen::Vector3f transformed_point = relative_transformation * point.getVector3fMap();
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
          if (!pcl_isfinite (neighbor.range))
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
  return float(hits_counter)/float(valid_points_counter);
}

void
RangeImage::extractPlanes (float initial_max_plane_error,
                           std::vector<RangeImage::ExtractedPlane>& planes) const
{
  //MEASURE_FUNCTION_TIME;
  
  planes.clear();
  
  std::vector<bool> point_still_valid;
  point_still_valid.resize(points.size(), true);
  std::vector<int> points_on_plane;
  std::vector<float> distances_to_neighbors;
  
  VectorAverage3f vector_average;
  for (int start_point_idx = 0; start_point_idx<int(points.size()); ++start_point_idx)
  {
    if (!isValid(start_point_idx) || !point_still_valid[start_point_idx])
      continue;
    points_on_plane.clear();
    vector_average.reset();
    
    points_on_plane.push_back(start_point_idx);
    point_still_valid[start_point_idx] = false;
    const PointWithRange& start_point = points[start_point_idx];
    vector_average.add(start_point.getVector3fMap());
    
    Eigen::Vector3f eigen_values(0,0,0), plane_normal(0,0,0), eigen_vector2(0,0,0), eigen_vector3(0,0,0);
    float plane_d=0;
    bool first_plane_estimate_done = false;
    
    for (int check_neighbors_idx=0; check_neighbors_idx<int(points_on_plane.size()); ++check_neighbors_idx)
    {
      int current_idx = points_on_plane[check_neighbors_idx],
          current_y   = current_idx/width,
          current_x   = current_idx - current_y*width;
      const PointWithRange& current_point = points[current_idx];
      
      distances_to_neighbors.clear();
      for (int neighbor_y=current_y-1; neighbor_y<=current_y+1; ++neighbor_y)
        for (int neighbor_x=current_x-1; neighbor_x<=current_x+1; ++neighbor_x)
          if (isValid(neighbor_x, neighbor_y))
            distances_to_neighbors.push_back(
                (points[neighbor_y*width+neighbor_x].getVector3fMap()-current_point.getVector3fMap()).squaredNorm());
      std::sort(distances_to_neighbors.begin(), distances_to_neighbors.end());
      if (distances_to_neighbors.empty())
        continue;
      float max_squared_distance = 4.0f*distances_to_neighbors[distances_to_neighbors.size()/2];
      
      for (int neighbor_y=current_y-1; neighbor_y<=current_y+1; ++neighbor_y)
      {
        for (int neighbor_x=current_x-1; neighbor_x<=current_x+1; ++neighbor_x)
        {
          int neighbor_idx = neighbor_y*width + neighbor_x;
          if (!isValid(neighbor_x, neighbor_y) || !point_still_valid[neighbor_idx])
            continue;
          const PointWithRange& neighbor = points[neighbor_idx];
          float squared_distance_to_current_point = (neighbor.getVector3fMap()-current_point.getVector3fMap()).squaredNorm();
          if (squared_distance_to_current_point > max_squared_distance)
          {
            //cout << "Skipping a point because of distance jump.\n";
            continue;
          }
          
          if (first_plane_estimate_done)
          {
            float distance_to_plane = plane_d-plane_normal.dot(neighbor.getVector3fMap());
            //cout << PVARN(distance_to_plane);
            if (fabsf(distance_to_plane) > initial_max_plane_error)
              continue;
            
            bool do_point_above_check = true;
            if (do_point_above_check)
            {
              float normal_factor = (distance_to_plane<0 ? -initial_max_plane_error : initial_max_plane_error);
              Eigen::Vector3f point_above = neighbor.getVector3fMap()-normal_factor*plane_normal;
              int point_above_image_x, point_above_image_y;
              getImagePoint(point_above, point_above_image_x, point_above_image_y);
              if (isValid(point_above_image_x, point_above_image_y)) {
                float point_above_squared_error =
                  (point_above-getPoint(point_above_image_x, point_above_image_y).getVector3fMap()).squaredNorm();
                if (point_above_squared_error < pow(0.5*initial_max_plane_error, 2))
                {
                  //std::cout << "Kicking out a point since it has a point above.\n";
                  continue;
                }
              }
            }
          }
          points_on_plane.push_back(neighbor_idx);
          point_still_valid[neighbor_idx] = false;
          vector_average.add(neighbor.getVector3fMap());
        }
      }
      if (vector_average.getNoOfSamples() >= 3)
      {
        vector_average.doPCA (eigen_values, plane_normal, eigen_vector2, eigen_vector3);
        plane_d = plane_normal.dot(vector_average.getMean());
        first_plane_estimate_done = true;
      }
    }
    
    if (vector_average.getNoOfSamples() < 3)
      continue;
    planes.push_back(ExtractedPlane());
    ExtractedPlane& plane = planes.back();
    plane.point_indices = points_on_plane;
    
    bool reduce_maximum_plane_error_regarding_sigma = false;
    if (reduce_maximum_plane_error_regarding_sigma)
    {
      float new_max_plane_error = 3.0f*sqrtf(eigen_values[0]);
      vector_average.reset();
      for (int plane_point_idx=0; plane_point_idx<int(points_on_plane.size()); ++plane_point_idx) {
        int point_idx = points_on_plane[plane_point_idx];
        const PointWithRange& point = points[point_idx];
        float distance_to_plane = fabsf(plane_d-plane_normal.dot(point.getVector3fMap()));
        if (distance_to_plane > new_max_plane_error)
          continue;
        plane.point_indices.push_back(point_idx);
        vector_average.add(point.getVector3fMap());
      }
      if (vector_average.getNoOfSamples() < 3)
      {
        planes.pop_back();
        continue;
      }
      vector_average.doPCA (plane.eigen_values, plane.eigen_vector1, plane.eigen_vector2, plane.eigen_vector3);
    }
    else
    {
      plane.eigen_values = eigen_values;
      plane.eigen_vector1 = plane_normal;
      plane.eigen_vector2 = eigen_vector2;
      plane.eigen_vector3 = eigen_vector3;
    }
    plane.normal = plane.eigen_vector1;
    plane.mean = vector_average.getMean();
    plane.d = plane_normal.dot(plane.mean);
    
    Eigen::Affine3f trans;
    getTransformationFromTwoUnitVectorsAndOrigin (plane.eigen_vector2, plane.eigen_vector3, plane.mean, trans);
    float min_x =  std::numeric_limits<float>::infinity (),
          max_x = -std::numeric_limits<float>::infinity (),
          min_y =  std::numeric_limits<float>::infinity (),
          max_y = -std::numeric_limits<float>::infinity (),
          min_z =  std::numeric_limits<float>::infinity (),
          max_z = -std::numeric_limits<float>::infinity ();
    for (int plane_point_idx=0; plane_point_idx<int(plane.point_indices.size()); ++plane_point_idx) {
      Eigen::Vector3f trans_point = trans * points[plane.point_indices[plane_point_idx]].getVector3fMap ();
      min_x = std::min(min_x, trans_point.x());
      max_x = std::max(max_x, trans_point.x());
      min_y = std::min(min_y, trans_point.y());
      max_y = std::max(max_y, trans_point.y());
      min_z = std::min(min_z, trans_point.z());
      max_z = std::max(max_z, trans_point.z());
    }
    plane.maximum_extensions = Eigen::Vector3f(max_x-min_x, max_y-min_y, max_z-min_z);
    //cout << "Plane has approximate size "
         //<< plane.maximum_extensions[0]<<"x"<<plane.maximum_extensions[1]<<"x"<<plane.maximum_extensions[2]
         //<< "m (sqrt(eigenvalues)="
         //<< sqrtf(plane.eigen_values[0])<<","<<sqrtf(plane.eigen_values[1])<<","<<sqrtf(plane.eigen_values[2])<<").\n";
    
    //for (int plane_point_idx=0; plane_point_idx<int(plane.point_indices.size()); ++plane_point_idx) {
      //((RangeImage*)this)->points[plane.point_indices[plane_point_idx]].range = NAN;
    //}
  }
  //cout << "\n\n";
}

}  // namespace end


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


#include <iostream>
using std::cout;
using std::cerr;
#include <map>
#include <set>
#include <cmath>
#include <Eigen/Geometry>
#include <pcl/pcl_macros.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/point_cloud.h>
#include <pcl/common/vector_average.h>
#include <pcl/features/range_image_border_extractor.h>

#define USE_OMP 1

namespace pcl {

RangeImageBorderExtractor::RangeImageBorderExtractor(const RangeImage* range_image) : BaseClass(), range_image_(range_image),
  border_scores_left_(NULL), border_scores_right_(NULL), border_scores_top_(NULL), border_scores_bottom_(NULL),
  surface_structure_(NULL), border_descriptions_(NULL), shadow_border_informations_(NULL), border_directions_(NULL),
  surface_change_scores_(NULL), surface_change_directions_(NULL)
{
}

RangeImageBorderExtractor::~RangeImageBorderExtractor()
{
  clearData();
}

void RangeImageBorderExtractor::setRangeImage(const RangeImage* range_image)
{
  clearData();
  range_image_ = range_image;
}

void RangeImageBorderExtractor::clearData()
{
  //cerr << __PRETTY_FUNCTION__<<" called.\n";
  
  delete[] border_scores_left_;    border_scores_left_   = NULL;
  delete[] border_scores_right_;   border_scores_right_  = NULL;
  delete[] border_scores_top_;     border_scores_top_    = NULL;
  delete[] border_scores_bottom_;  border_scores_bottom_ = NULL;
  if (range_image_==NULL)
  { 
    if (surface_structure_!=NULL || shadow_border_informations_!=NULL || border_directions_!=NULL)
      std::cerr << __PRETTY_FUNCTION__ << ": Can't erase elements of surface_structure_ since range_image_ is NULL.\n";
  }
  else
  {
    for (int i=0; i<int(range_image_->width*range_image_->height); ++i)
    {
      if (surface_structure_!=NULL)
        delete surface_structure_[i];
      if (shadow_border_informations_!=NULL)
        delete shadow_border_informations_[i];
      if (border_directions_!=NULL)
        delete border_directions_[i];
    }
  }
  delete[] surface_structure_; surface_structure_ = NULL;
  delete border_descriptions_; border_descriptions_ = NULL;
  delete[] shadow_border_informations_; shadow_border_informations_ = NULL;
  delete[] border_directions_; border_directions_ = NULL;
  
  delete[] surface_change_scores_;  surface_change_scores_ = NULL;
  delete[] surface_change_directions_;  surface_change_directions_ = NULL;
}

void RangeImageBorderExtractor::extractLocalSurfaceStructure()
{
  if (surface_structure_ != NULL)
    return;
  //MEASURE_FUNCTION_TIME;
  
  int width  = range_image_->width,
      height = range_image_->height,
      array_size = width*height;
  surface_structure_ = new LocalSurface*[array_size];
  int step_size = (std::max)(1, parameters_.pixel_radius_plane_extraction/2);
  //cout << PVARN(step_size);
  int no_of_nearest_neighbors = pow ((double)(parameters_.pixel_radius_plane_extraction/step_size + 1), 2.0);
# if USE_OMP
//#   pragma omp parallel for default(shared) schedule(dynamic, 10)
#   pragma omp parallel for default(shared)
# endif
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      LocalSurface*& local_surface = surface_structure_[index];
      local_surface = NULL;
      if (!range_image_->isValid(index))
        continue;
      local_surface = new LocalSurface;
      Eigen::Vector3f point;
      range_image_->getPoint(x, y, point);
      //cout << PVARN(point);
      if (!range_image_->getSurfaceInformation(x, y, parameters_.pixel_radius_plane_extraction, point,
                                  no_of_nearest_neighbors, step_size, local_surface->max_neighbor_distance_squared,
                                  local_surface->normal_no_jumps, local_surface->neighborhood_mean_no_jumps,
                                  local_surface->eigen_values_no_jumps,  &local_surface->normal,
                                  &local_surface->neighborhood_mean, &local_surface->eigen_values))
      {
        delete local_surface;
        local_surface = NULL;
      }
      
      //cout << x<<","<<y<<": ("<<local_surface->normal_no_jumps[0]<<","<<local_surface->normal_no_jumps[1]<<","<<local_surface->normal_no_jumps[2]<<")\n";
    }
  }
  
  // TODO: WTF!? pragma inverts normal... <- Fixed I think
  
  //LocalSurface** surface_structure2 = new LocalSurface*[array_size];
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      //int index = y*width + x;
      //LocalSurface*& local_surface1 = surface_structure_[index];
      //LocalSurface*& local_surface = surface_structure2[index];
      //local_surface = NULL;
      //if (!range_image_->isValid(index))
        //continue;
      //local_surface = new LocalSurface;
      //range_image_->getPoint(x, y, point);
      ////cout << PVARN(point);
      //range_image_->getSurfaceInformation(x, y, parameters_.pixel_radius_plane_extraction, point, no_of_nearest_neighbors, step_size,
                                          //local_surface->max_neighbor_distance_squared,
                                          //local_surface->normal_no_jumps, local_surface->neighborhood_mean_no_jumps, local_surface->smallest_eigenvalue_no_jumps,
                                          //&local_surface->normal, &local_surface->neighborhood_mean, &local_surface->smallest_eigenvalue);
      
      //if (local_surface->normal_no_jumps != surface_structure_[index]->normal_no_jumps)
      //{
        //cout << "With pragma: "<<x<<","<<y<<": ("<<local_surface1->normal_no_jumps[0]<<","<<local_surface1->normal_no_jumps[1]<<","<<local_surface1->normal_no_jumps[2]<<")\n";
        //cout << "Without pragma: "<<x<<","<<y<<": ("<<local_surface->normal_no_jumps[0]<<","<<local_surface->normal_no_jumps[1]<<","<<local_surface->normal_no_jumps[2]<<")\n";
      //}
    //}
  //}

}

void RangeImageBorderExtractor::extractBorderScoreImages()
{
  if (border_scores_left_ != NULL)
    return;
  
  extractLocalSurfaceStructure();
  
  //MEASURE_FUNCTION_TIME;
  
  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  border_scores_left_   = new float[size];
  border_scores_right_  = new float[size];
  border_scores_top_    = new float[size];
  border_scores_bottom_ = new float[size];
  
# if USE_OMP
//#   pragma omp parallel for default(shared) schedule(dynamic, 10)
#   pragma omp parallel for default(shared)
# endif
  for (int y=0; y<height; ++y) {
    for (int x=0; x<width; ++x) {
      int index = y*width + x;
      float& left=border_scores_left_[index]; float& right=border_scores_right_[index];
      float& top=border_scores_top_[index]; float& bottom=border_scores_bottom_[index]; 
      LocalSurface* local_surface_ptr = surface_structure_[index];
      if (local_surface_ptr==NULL)
      {
        left=right=top=bottom = 0.0f;
        continue;
      }
      
      left   = getNeighborDistanceChangeScore(*local_surface_ptr, x, y, -1,  0, parameters_.pixel_radius_borders);
      right  = getNeighborDistanceChangeScore(*local_surface_ptr, x, y,  1,  0, parameters_.pixel_radius_borders);
      top    = getNeighborDistanceChangeScore(*local_surface_ptr, x, y,  0, -1, parameters_.pixel_radius_borders);
      bottom = getNeighborDistanceChangeScore(*local_surface_ptr, x, y,  0,  1, parameters_.pixel_radius_borders);
    }
  }
}

float* RangeImageBorderExtractor::updatedScoresAccordingToNeighborValues(const float* border_scores) const
{
  float* new_scores = new float[range_image_->width*range_image_->height];
  float* new_scores_ptr = new_scores;
  for (int y=0; y<(int)range_image_->height; ++y) {
    for (int x=0; x<(int)range_image_->width; ++x) {
      *(new_scores_ptr++) = updatedScoreAccordingToNeighborValues(x, y, border_scores);
    }
  }
  return new_scores;
}

void RangeImageBorderExtractor::updateScoresAccordingToNeighborValues()
{
  extractBorderScoreImages();
  
  //MEASURE_FUNCTION_TIME;
  
  float* left_with_propagated_neighbors = updatedScoresAccordingToNeighborValues(border_scores_left_);
  delete[] border_scores_left_;
  border_scores_left_ = left_with_propagated_neighbors;
  float* right_with_propagated_neighbors = updatedScoresAccordingToNeighborValues(border_scores_right_);
  delete[] border_scores_right_;
  border_scores_right_ = right_with_propagated_neighbors;
  float* top_with_propagated_neighbors = updatedScoresAccordingToNeighborValues(border_scores_top_);
  delete[] border_scores_top_;
  border_scores_top_ = top_with_propagated_neighbors;
  float* bottom_with_propagated_neighbors = updatedScoresAccordingToNeighborValues(border_scores_bottom_);
  delete[] border_scores_bottom_;
  border_scores_bottom_ = bottom_with_propagated_neighbors;
}
 
void RangeImageBorderExtractor::findAndEvaluateShadowBorders()
{
  if (shadow_border_informations_ != NULL)
    return;
  
  if (border_scores_left_==NULL)
  {
    std::cerr << __PRETTY_FUNCTION__<<": border score images not available!\n";
  }
  
  //MEASURE_FUNCTION_TIME;
  
  int width  = range_image_->width,
      height = range_image_->height;
  shadow_border_informations_ = new ShadowBorderIndices*[width*height];
  for (int y=0; y<(int)height; ++y) {
    for (int x=0; x<(int)width; ++x) {
      int index = y*width+x;
      ShadowBorderIndices*& shadow_border_indices = shadow_border_informations_[index];
      shadow_border_indices = NULL;
      int shadow_border_idx;
      
      if (changeScoreAccordingToShadowBorderValue(x, y, -1, 0, border_scores_left_, border_scores_right_, shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->left = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 1, 0, border_scores_right_, border_scores_left_, shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->right = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 0, -1, border_scores_top_, border_scores_bottom_, shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->top = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 0, 1, border_scores_bottom_, border_scores_top_, shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->bottom = shadow_border_idx;
      }
      
      //if (checkPotentialBorder(x, y, -1, 0, border_scores_left_, border_scores_right_, shadow_border_idx))
      //{
        //shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        //shadow_border_indices->left = shadow_border_idx;
      //}
      //if (checkPotentialBorder(x, y, 1, 0, border_scores_right_, border_scores_left_, shadow_border_idx))
      //{
        //shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        //shadow_border_indices->right = shadow_border_idx;
      //}
      //if (checkPotentialBorder(x, y, 0, -1, border_scores_top_, border_scores_bottom_, shadow_border_idx))
      //{
        //shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        //shadow_border_indices->top = shadow_border_idx;
      //}
      //if (checkPotentialBorder(x, y, 0, 1, border_scores_bottom_, border_scores_top_, shadow_border_idx))
      //{
        //shadow_border_indices = (shadow_border_indices==NULL ? new ShadowBorderIndices : shadow_border_indices);
        //shadow_border_indices->bottom = shadow_border_idx;
      //}
    }
  }
}

float* RangeImageBorderExtractor::getAnglesImageForBorderDirections()
{
  //MEASURE_FUNCTION_TIME;
  
  calculateBorderDirections();
  
  int width  = range_image_->width,
      height = range_image_->height,
      array_size = width*height;
  float* angles_image = new float[array_size];
  
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float& angle = angles_image[index];
      angle = -std::numeric_limits<float>::infinity ();
      const Eigen::Vector3f* border_direction_ptr = border_directions_[index];
      if (border_direction_ptr == NULL)
        continue;
      const Eigen::Vector3f& border_direction = *border_direction_ptr;
      const PointWithRange& point = range_image_->getPoint(index);
      
      float border_direction_in_image_x, border_direction_in_image_y;
      float tmp_factor = point.range*range_image_->getAngularResolution();
      range_image_->getImagePoint(point.x+tmp_factor*border_direction[0], point.y+tmp_factor*border_direction[1], point.z+tmp_factor*border_direction[2],
                                border_direction_in_image_x, border_direction_in_image_y);
      border_direction_in_image_x -= x;  border_direction_in_image_y -= y;
      angle = atan2(border_direction_in_image_y, border_direction_in_image_x);
    }
  }
  return angles_image;
}

float* RangeImageBorderExtractor::getAnglesImageForSurfaceChangeDirections()
{
  //MEASURE_FUNCTION_TIME;
  
  calculateSurfaceChanges();
  
  int width  = range_image_->width,
      height = range_image_->height,
      array_size = width*height;
  float* angles_image = new float[array_size];
  
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float& angle = angles_image[index];
      angle = -std::numeric_limits<float>::infinity ();
      float surface_change_score = surface_change_scores_[index];
      if (surface_change_score <= 0.1f)
        continue;
      const Eigen::Vector3f& direction = surface_change_directions_[index];
      const PointWithRange& point = range_image_->getPoint(index);
      
      float border_direction_in_image_x, border_direction_in_image_y;
      float tmp_factor = point.range*range_image_->getAngularResolution();
      range_image_->getImagePoint(point.x+tmp_factor*direction[0], point.y+tmp_factor*direction[1], point.z+tmp_factor*direction[2],
                                border_direction_in_image_x, border_direction_in_image_y);
      border_direction_in_image_x -= x;  border_direction_in_image_y -= y;
      angle = atan2(border_direction_in_image_y, border_direction_in_image_x);
      //if (!pcl_isfinite(angle))
        //cerr << PVARC(direction) << PVARC(point) << PVARC(border_direction_in_image_x) << PVARN(border_direction_in_image_y);
      if (angle <= deg2rad(-90.0f))
        angle += M_PI;
      else if (angle > deg2rad(90.0f))
        angle -= M_PI;
    }
  }
  return angles_image;
}



void RangeImageBorderExtractor::classifyBorders()
{
  if (border_descriptions_ != NULL)
    return;
  
  // Get local plane approximations
  extractLocalSurfaceStructure();
  
  // Get scores for every point, describing how probable a border in that direction is
  extractBorderScoreImages();
  
  // Propagate values to neighboring pixels
  updateScoresAccordingToNeighborValues();
  
  // Change border score according to the existence of a shadow border
  findAndEvaluateShadowBorders();
  
  //MEASURE_FUNCTION_TIME;
  
  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  
  BorderDescription initial_border_description;
  initial_border_description.traits = 0;
  border_descriptions_ = new PointCloudOut;
  border_descriptions_->width = width;
  border_descriptions_->height = height;
  border_descriptions_->is_dense = true;
  border_descriptions_->points.resize(size, initial_border_description);
  
  //std::vector<BorderDescription*> border_points;
  for (int y=0; y<(int)height; ++y) {
    for (int x=0; x<(int)width; ++x) {
      int index = y*width+x;
      BorderDescription& border_description = border_descriptions_->points[index];
      border_description.x = x;
      border_description.y = y;
      BorderTraits& border_traits = border_description.traits;
      
      ShadowBorderIndices* shadow_border_indices = shadow_border_informations_[index];
      if (shadow_border_indices == NULL)
        continue;
      
      int shadow_border_index = shadow_border_indices->left;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, -1, 0, border_scores_left_, shadow_border_index))
      {
        BorderTraits& shadow_traits = border_descriptions_->points[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_RIGHT] = true;
        for (int index3=index-1; index3>shadow_border_index; --index3)
        {
          BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_RIGHT] = true;
        }
      }
      
      shadow_border_index = shadow_border_indices->right;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 1, 0, border_scores_right_, shadow_border_index))
      {
        BorderTraits& shadow_traits = border_descriptions_->points[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_LEFT] = true;
        for (int index3=index+1; index3<shadow_border_index; ++index3)
        {
          BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_LEFT] = true;
        }
      }
      
      shadow_border_index = shadow_border_indices->top;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 0, -1, border_scores_top_, shadow_border_index))
      {
        BorderTraits& shadow_traits = border_descriptions_->points[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_BOTTOM] = true;
        for (int index3=index-width; index3>shadow_border_index; index3-=width)
        {
          //cout << "Adding veil point at "<<(index3-index)%width<<","<<(index3-index)/width<<".\n";
          BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_BOTTOM] = true;
        }
      }
      
      shadow_border_index = shadow_border_indices->bottom;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 0, 1, border_scores_bottom_, shadow_border_index))
      {
        BorderTraits& shadow_traits = border_descriptions_->points[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_TOP] = true;
        for (int index3=index+width; index3<shadow_border_index; index3+=width)
        {
          //cout << "Adding veil point at "<<(index3-index)%width<<","<<(index3-index)/width<<".\n";
          BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_TOP] = true;
        }
      }
      
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER])
      //{
        //border_points.push_back(&border_description);
      //}
    }
  }
  
  //-----Find out connectivity between border cells-----
  //for (unsigned int i=0; i<border_points.size(); ++i)
  //{
    //BorderDescription& border_description = *border_points[i];
    //BorderTraits& border_traits = border_description.traits;
    //int x=border_description.x, y=border_description.y;
    //int index = y*width + x;
    //float angle = getObstacleBorderAngle(border_traits);
    
    //for (int y2=y-1; y2<=y+1; ++y2)
    //{
      //for (int x2=x-1; x2<=x+1; ++x2)
      //{
        //if (!range_image_->isInImage(x2, y2) || (x2==x&&y2==y))
          //continue;
        //int index2 = y2*width + x2;
        //BorderDescription& neighbor_border_description = border_descriptions_->points[index2];
        //BorderTraits& neighbor_border_traits = neighbor_border_description.traits;
        //if (!neighbor_border_traits[BORDER_TRAIT__OBSTACLE_BORDER])
          //continue;
        //float neighbor_angle = getObstacleBorderAngle(neighbor_border_traits);
        //float anglesDiff = fabs(normAngle(angle-neighbor_angle));
        //if (anglesDiff > 0.5f*M_PI)
          //continue;
        //float border_between_points_score = getNeighborDistanceChangeScore(*surface_structure_[index], x, y, x2-x,  y2-y, 1);
        //if (fabsf(border_between_points_score) >= 0.95f*parameters_.minimum_border_probability)
          //continue;
        
        //border_description.neighbors.push_back(&neighbor_border_description);
      //}
    //}
    //// Point without any neighbor => no border
    //if (border_description.neighbors.empty())
    //{
      //border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = false;
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT])
      //{
        //border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT] = false;
        //BorderTraits& shadow_border = border_descriptions_->points[shadow_border_informations_[index]->left].traits;
        //shadow_border[BORDER_TRAIT__SHADOW_BORDER_RIGHT] = false;
        //shadow_border[BORDER_TRAIT__SHADOW_BORDER] = shadow_border[BORDER_TRAIT__SHADOW_BORDER_LEFT]   ||
                                                     //shadow_border[BORDER_TRAIT__SHADOW_BORDER_TOP]    ||
                                                     //shadow_border[BORDER_TRAIT__SHADOW_BORDER_BOTTOM];
      //}
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT])
      //{
        //border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT] = false;
        //BorderTraits& shadow_border = border_descriptions_->points[shadow_border_informations_[index]->right].traits;
        //shadow_border[BORDER_TRAIT__SHADOW_BORDER_LEFT] = false;
        //shadow_border[BORDER_TRAIT__SHADOW_BORDER] = shadow_border[BORDER_TRAIT__SHADOW_BORDER_RIGHT] ||
                                                     //shadow_border[BORDER_TRAIT__SHADOW_BORDER_TOP]   ||
                                                     //shadow_border[BORDER_TRAIT__SHADOW_BORDER_BOTTOM];
      //}
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP])
      //{
        //border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP] = false;
        //BorderTraits& shadow_border = border_descriptions_->points[shadow_border_informations_[index]->top].traits;
        //shadow_border[BORDER_TRAIT__SHADOW_BORDER_BOTTOM] = false;
        //shadow_border[BORDER_TRAIT__SHADOW_BORDER] = shadow_border[BORDER_TRAIT__SHADOW_BORDER_LEFT]  ||
                                                     //shadow_border[BORDER_TRAIT__SHADOW_BORDER_RIGHT] ||
                                                     //shadow_border[BORDER_TRAIT__SHADOW_BORDER_TOP];
      //}
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM])
      //{
        //border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM] = false;
        //BorderTraits& shadow_border = border_descriptions_->points[shadow_border_informations_[index]->bottom].traits;
        //shadow_border[BORDER_TRAIT__SHADOW_BORDER_TOP] = false;
        //shadow_border[BORDER_TRAIT__SHADOW_BORDER] = shadow_border[BORDER_TRAIT__SHADOW_BORDER_LEFT]   ||
                                                     //shadow_border[BORDER_TRAIT__SHADOW_BORDER_RIGHT]  ||
                                                     //shadow_border[BORDER_TRAIT__SHADOW_BORDER_BOTTOM];
      //}
    //}
    //else
    //{
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT])
      //{
        //int shadow_border_index = shadow_border_informations_[index]->left;
        ////if (!border_descriptions_->points[shadow_border_index].traits[BORDER_TRAIT__SHADOW_BORDER])
          ////std::cout << "Left: "<<shadow_border_index-index<<".\n";
        //for (int index3=index-1; index3>shadow_border_index; --index3)
        //{
          //BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          //veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_RIGHT] = true;
        //}
      //}
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT])
      //{
        //int shadow_border_index = shadow_border_informations_[index]->right;
        ////if (!border_descriptions_->points[shadow_border_index].traits[BORDER_TRAIT__SHADOW_BORDER])
          ////std::cout << "Right: "<<shadow_border_index-index<<".\n";
        //for (int index3=index+1; index3<shadow_border_index; ++index3)
        //{
          //BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          //veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_LEFT] = true;
        //}
      //}
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP])
      //{
        //int shadow_border_index = shadow_border_informations_[index]->top;
        ////if (!border_descriptions_->points[shadow_border_index].traits[BORDER_TRAIT__SHADOW_BORDER])
          ////std::cout << "Top: "<<(shadow_border_index-index)/float(width)<<".\n";
        //for (int index3=index-width; index3>shadow_border_index; index3-=width)
        //{
          ////cout << "Adding veil point at "<<(index3-index)%width<<","<<(index3-index)/width<<".\n";
          //BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          //veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_BOTTOM] = true;
        //}
      //}
      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM])
      //{
        //int shadow_border_index = shadow_border_informations_[index]->bottom;
        ////if (!border_descriptions_->points[shadow_border_index].traits[BORDER_TRAIT__SHADOW_BORDER])
          ////std::cout << "Bottom: "<<(shadow_border_index-index)/float(width)<<".\n";
        //for (int index3=index+width; index3<shadow_border_index; index3+=width)
        //{
          ////cout << "Adding veil point at "<<(index3-index)%width<<","<<(index3-index)/width<<".\n";
          //BorderTraits& veil_point = border_descriptions_->points[index3].traits;
          //veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_TOP] = true;
        //}
      //}
    //}
  //}
}

void RangeImageBorderExtractor::calculateBorderDirections()
{
  if (border_directions_!=NULL)
    return;
  classifyBorders();
  
  //MEASURE_FUNCTION_TIME;
  
  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  border_directions_ = new Eigen::Vector3f*[size];
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      calculateBorderDirection(x, y);
    }
  }
  
  Eigen::Vector3f** average_border_directions = new Eigen::Vector3f*[size];
  int radius = parameters_.pixel_radius_border_direction;
  int minimum_weight = radius+1;
  float min_cos_angle=cosf(deg2rad(120.0f));
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      Eigen::Vector3f*& average_border_direction = average_border_directions[index];
      average_border_direction = NULL;
      const Eigen::Vector3f* border_direction = border_directions_[index];
      if (border_direction==NULL)
        continue;
      average_border_direction = new Eigen::Vector3f(*border_direction);
      float weight_sum = 1.0f;
      for (int y2=(std::max)(0, y-radius); y2<=(std::min)(y+radius, height-1); ++y2)
      {
        for (int x2=(std::max)(0, x-radius); x2<=(std::min)(x+radius, width-1); ++x2)
        {
          int index2 = y2*width + x2;
          const Eigen::Vector3f* neighbor_border_direction = border_directions_[index2];
          if (neighbor_border_direction==NULL || index2==index)
            continue;
          
          // Oposite directions?
          float cos_angle = neighbor_border_direction->dot(*border_direction);
          if (cos_angle<min_cos_angle)
          {
            //cout << "Reject. "<<PVARC(min_cos_angle)<<PVARC(cos_angle)<<PVARAN(acosf(cos_angle));
            continue;
          }
          //else
            //cout << "No reject\n";
          
          // Border in between?
          float border_between_points_score = getNeighborDistanceChangeScore(*surface_structure_[index], x, y, x2-x,  y2-y, 1);
          if (fabsf(border_between_points_score) >= 0.95f*parameters_.minimum_border_probability)
            continue;
          
          *average_border_direction += *neighbor_border_direction;
          weight_sum += 1.0f;
        }
      }
      if (lrint (weight_sum) < minimum_weight)
      {
        delete average_border_direction;
        average_border_direction=NULL;
      }
      else
        average_border_direction->normalize();
    }
  }
  
  for (int i=0; i<size; ++i)
    delete border_directions_[i];
  delete[] border_directions_;
  border_directions_ = average_border_directions;
}

void RangeImageBorderExtractor::calculateSurfaceChanges()
{
  if (surface_change_scores_!=NULL)
    return;
  
  calculateBorderDirections();
  
  //MEASURE_FUNCTION_TIME;
  
  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  surface_change_scores_ = new float[size];
  surface_change_directions_ = new Eigen::Vector3f[size];
  Eigen::Vector3f sensor_pos = range_image_->getSensorPos();
# if USE_OMP
//#   pragma omp parallel for default(shared) schedule(dynamic, 10)
#   pragma omp parallel for default(shared)
# endif
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float& surface_change_score = surface_change_scores_[index];
      surface_change_score = 0.0f;
      Eigen::Vector3f& surface_change_direction = surface_change_directions_[index];
      surface_change_direction.setZero();
      
      const BorderTraits& border_traits = border_descriptions_->points[index].traits;
      if (border_traits[BORDER_TRAIT__VEIL_POINT] || border_traits[BORDER_TRAIT__SHADOW_BORDER])
        continue;
      if (border_directions_[index]!=NULL)
      {
        surface_change_score = 1.0f;
        surface_change_direction = *border_directions_[index];
        //if (fabsf(surface_change_direction.norm()-1.0f) > 0.001)
          //cerr << surface_change_direction[0]<<","<<surface_change_direction[1]<<","<<surface_change_direction[2]
          //     <<" has norm "<<surface_change_direction.norm()<<"\n";
        //else
          //cerr<<"OK";
      }
      else
      {
        if (!calculateMainPrincipalCurvature(x, y, parameters_.pixel_radius_principal_curvature,
                                             surface_change_score, surface_change_direction))
        {
          surface_change_score = 0.0f;
          continue;
        }
      }
    }
  }
  //blurSurfaceChanges();
}


void RangeImageBorderExtractor::blurSurfaceChanges()
{
  //MEASURE_FUNCTION_TIME;
  
  //int blur_radius = parameters_.interest_image_blur_size;
  int blur_radius = 1;
  if (blur_radius==0)
    return;
  
  const RangeImage& range_image = *range_image_;
  
  Eigen::Vector3f* blurred_directions = new Eigen::Vector3f[range_image.width*range_image.height];
  float* blurred_scores = new float[range_image.width*range_image.height];
  for (int y=0; y<int(range_image.height); ++y)
  {
    for (int x=0; x<int(range_image.width); ++x)
    {
      int index = y*range_image.width + x;
      Eigen::Vector3f& new_point = blurred_directions[index];
      new_point.setZero();
      float& new_score = blurred_scores[index];
      new_score = 0.0f;
      if (!range_image.isValid(index))
        continue;
      const BorderTraits& border_traits = border_descriptions_->points[index].traits;
      if (border_traits[BORDER_TRAIT__VEIL_POINT] || border_traits[BORDER_TRAIT__SHADOW_BORDER])
        continue;
      const Eigen::Vector3f& point = surface_change_directions_[index];
      float counter = 0.0f;
      for (int y2=y-blur_radius; y2<y+blur_radius; ++y2)
      {
        for (int x2=x-blur_radius; x2<x+blur_radius; ++x2)
        {
          if (!range_image.isInImage(x2,y2))
            continue;
          int index2 = y2*range_image.width + x2;
          float score = surface_change_scores_[index2];
          //if (score == 0.0f)
            //continue;
            //
          if (score > 0.0f)
          {
            Eigen::Vector3f& neighbor = surface_change_directions_[index2];
            //if (fabs(neighbor.norm()-1) > 1e-4)
              //cerr<<PVARN(neighbor)<<PVARN(score);
            if (point.dot(neighbor)<0.0f)
              neighbor *= -1.0f;
            new_point += score*neighbor;
          }
          new_score += score;
          counter += 1.0f;
        }
      }
      new_point.normalize();
      if (counter > 0.0f)
        new_score /= counter;
    }
  }
  delete[] surface_change_directions_;
  surface_change_directions_ = blurred_directions;
  delete[] surface_change_scores_;
  surface_change_scores_ = blurred_scores;
}

void RangeImageBorderExtractor::computeFeature(PointCloudOut& output)
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
  output.points.clear();
  
  if (indices_)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": Sorry, usage of indices for the extraction is not supported for range image border extraction (yet).\n\n";
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  
  if (range_image_==NULL)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImage is not set. Sorry, the border extraction works on range images, not on normal point clouds."
              << " Use setRangeImage(...).\n\n";
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  output = getBorderDescriptions();
}

void RangeImageBorderExtractor::compute(PointCloudOut& output)
{
  computeFeature(output);
}


///////////////////
/////OLD STUFF/////
///////////////////



//void RangeImageBorderExtractor::unifySurfaceChangeDirections()
//{
  //calculateSurfaceChanges();
  
  //Eigen::Transform3f to_viewer_coordinate_frame, to_world_coordinate_system;
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      //if (!range_image_->isValid(x, y))
        //continue;
      //int index = y*width + x;
      //getRotationToViewerCoordinateFrame(range_image_->getPoint(index).getVector3fMap(), to_viewer_coordinate_frame);
      //getInverse(to_viewer_coordinate_frame, to_world_coordinate_system);
      
      //Eigen::Vector3f& surface_change_direction = surface_change_directions_[index];
      //Eigen::Vector3f transformed_direction = to_viewer_coordinate_frame * surface_change_direction;
      //transformed_direction[0] = fabsf(transformed_direction[0]);
      //transformed_direction[1] = fabsf(transformed_direction[1]);
      //transformed_direction[2] = fabsf(transformed_direction[2]);
      //surface_change_direction = to_world_coordinate_system*transformed_direction;
    //}
  //}
//}

//RangeImageBorderExtractor::Parameters::Parameters() : 
    //pixel_radius_borders(2),
    //minimum_border_probability(0.75),
    ////border_probability_prior(0.05),
    ////probability_distribution_acuteness(PiecewiseLinearFunction(10.0f, 0.0f)),
    ////probability_distribution_acuteness_given_border(PiecewiseLinearFunction(10.0f, 0.0f)),
    ////probability_distribution_acuteness_change(PiecewiseLinearFunction(10.0f, 0.0f)),
    ////probability_distribution_acuteness_change_given_border(PiecewiseLinearFunction(10.0f, 0.0f))
//{
  //// The following parameters_ are learned from an example scene
  ////float pda_tmp[11] = {1, 0.825572, 0.684774, 0.5667, 0.474496, 0.394675, 0.317035, 0.231982, 0.154149, 0.0878983, 0};
  ////std::copy(pda_tmp, pda_tmp + sizeof(pda_tmp)/sizeof(*pda_tmp),
  ////std::inserter(probability_distribution_acuteness.getDataPoints(),
                ////probability_distribution_acuteness.getDataPoints().end()));
  ////float pdagb_tmp[11] = {1, 1, 1, 1, 1, 1, 0.99941, 0.952696, 0.780718, 0.519958, 0};
  ////std::copy(pdagb_tmp, pdagb_tmp + sizeof(pdagb_tmp)/sizeof(*pdagb_tmp),
  ////std::inserter(probability_distribution_acuteness_given_border.getDataPoints(),
                ////probability_distribution_acuteness_given_border.getDataPoints().end()));
  ////float pdac_tmp[11] = {1, 0.257751, 0.167341, 0.109468, 0.0685638, 0.0403848, 0.0207717, 0.0111089, 0.00558984, 0.00218904, 0};
  ////std::copy(pdac_tmp, pdac_tmp + sizeof(pdac_tmp)/sizeof(*pdac_tmp),
  ////std::inserter(probability_distribution_acuteness_change.getDataPoints(),
                ////probability_distribution_acuteness_change.getDataPoints().end()));
  ////float pdacgb_tmp[11] = {1, 0.982729, 0.784047, 0.604682, 0.451084, 0.318745, 0.212103, 0.139726, 0.0836137, 0.0343333, 0};
  ////std::copy(pdacgb_tmp, pdacgb_tmp + sizeof(pdacgb_tmp)/sizeof(*pdacgb_tmp),
  ////std::inserter(probability_distribution_acuteness_change_given_border.getDataPoints(),
                ////probability_distribution_acuteness_change_given_border.getDataPoints().end()));
//}

//void RangeImage::classifyBorders(float minImpactAngle, PointCloud<BorderDescription>& border_descriptions) const {
  ////MEASURE_FUNCTION_TIME;
  
  //float cosMinImpactAngle = cosf(minImpactAngle);
  //float cosMinImpactAngle2 = cosf(2.0f*minImpactAngle);
  //bool doNormalFiltering = true;

  //int noOfPoints = width * height;
  
  //border_descriptions.width = width;
  //border_descriptions.height = height;
  ////border_descriptions.is_dense = true;
  //border_descriptions.points.resize(noOfPoints);
  //for (int i=0; i<noOfPoints; ++i)
    //border_descriptions.points[i].traits = 0;
  
  //Eigen::Vector4f sensor_pos = toWorldSystem_*Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
  
  //std::vector<int> neighborhoodIndices;
  //for (int y=0; y<(int)height-1; ++y) {
    //for (int x=0; x<(int)width-1; ++x) {
      //int index = y*width+x;
      //const PointWithRange& point = points[index];
      
      //for (int i=0; i<2; ++i) {
        //int neighbor_offset_x=1, neighbor_offset_y=0;
        //if (i==1) {
          //neighbor_offset_x=0, neighbor_offset_y=1;
        //}
        
        //int neighbor_x=x+neighbor_offset_x, neighbor_y=y+neighbor_offset_y;
        //const PointWithRange& neighbor = points[neighbor_y*width+neighbor_x];
        
        ////cout << "Neighbor "<<x<<","<<y<<" -> "<<neighbor_x<<","<<neighbor_y<<"\n";
        
        //float cos_impact_angle = getCosImpactAngle(point, neighbor);
        
        //int obstacle_point_x, obstacle_point_y, shadow_point_x, shadow_point_y, offset_x, offset_y;
        //if (cos_impact_angle > 0.0f) {
          //obstacle_point_x=x; obstacle_point_y=y;
          //shadow_point_x=neighbor_x; shadow_point_y=neighbor_y;
          //offset_x=neighbor_offset_x; offset_y=neighbor_offset_y;
        //}
        //else {
          //obstacle_point_x=neighbor_x; obstacle_point_y=neighbor_y;
          //shadow_point_x=x; shadow_point_y=y;
          //offset_x=-neighbor_offset_x; offset_y=-neighbor_offset_y;
          //cos_impact_angle = -cos_impact_angle;
        //}
        //const PointWithRange& obstacle_point = points[obstacle_point_y*width+obstacle_point_x];
        ////const PointWithRange& shadow_point   = points[shadow_point_y*width+shadow_point_x];
        //BorderTraits& obstacle_border_traits = border_descriptions.points[obstacle_point_y*width+obstacle_point_x].traits;
        //BorderTraits& shadow_border_traits   = border_descriptions.points[shadow_point_y*width+shadow_point_x].traits;
        
        //if (cos_impact_angle <= cosMinImpactAngle2) continue;  // Definitely not a jump?
        
        ////cout << PVARAN(acosf(cos_impact_angle));
        
        //// Check if the jump regarding two or more pixel distance is high enough
        //if (cos_impact_angle <= cosMinImpactAngle) {
          //// If the angle is already small in 1 pixel distance lets try more pixel distances
          //bool foundJump = false;
          ////cout << "Neighbor "<<neighbor_x<<","<<neighbor_y<<" to "<<x<<","<<y<<" is close ("<<RAD2DEG(acosf(cos_impact_angle))<<"deg).\n";
          //for (int neighbor2X=obstacle_point_x+2*offset_x, neighbor2Y=obstacle_point_y+2*offset_y;
               //neighbor2X<=obstacle_point_x+3*offset_x && neighbor2Y<=obstacle_point_y+3*offset_y;
               //neighbor2X+=offset_x, neighbor2Y+=offset_y)
          //{
            //float cos_impact_angle2 = 0.0f;
            //if (isInImage(neighbor2X, neighbor2Y)) {
              //const PointWithRange& neighbor2 = points[neighbor2Y*width+neighbor2X];
              //cos_impact_angle2 = getCosImpactAngle(obstacle_point, neighbor2);
            //}
            //else {
              //break;
            //}
            ////cout << "Trying "<<neighbor2X<<","<<neighbor2Y<<": "<<RAD2DEG(acosf(cos_impact_angle2))<<"deg.\n";
            //if (cos_impact_angle2 <= cosMinImpactAngle) continue;
            ////cout << "Angle in "<<max(fabs(neighbor2X-x), fabs(neighbor2Y-y))<<" pixel distance is OK.\n";
            ////cos_impact_angle = cos_impact_angle2;
            //foundJump = true;
            //break;
          //}
          //if (!foundJump) continue;
        //}
        
        //// Check if there has been a significant change in the angle, to decide if this is
        //// a real border or just a plane captured at a flat angle
        //int other_neighbor_x = obstacle_point_x-offset_x, other_neighbor_y = obstacle_point_y-offset_y;
        //bool clearlyAJump = false;
        //if (isInImage(other_neighbor_x, other_neighbor_y)) {
          //const PointWithRange& otherNeighbor = points[other_neighbor_y*width + other_neighbor_x];
          //float cosNeighborImpactAngle = getCosImpactAngle(otherNeighbor, obstacle_point);
          ////cout << PVARC(other_neighbor_x)<<PVARC(other_neighbor_y)<<PVARC(obstacle_point_x)<<PVARN(obstacle_point_y);
          ////cout << PVARC(otherNeighbor)<<PVARC(obstacle_point)<<PVARAN(acosf(cosNeighborImpactAngle));
          
          //if (cosNeighborImpactAngle >= cosMinImpactAngle) {  // Is the neighbor closer to the sensor_pos also a jump?
            ////continue;
          //}
          //else if (cosNeighborImpactAngle <= cosMinImpactAngle2) {  // Is there clearly a change in gradient?
            ////clearlyAJump = true;
          //}
        //}
        
        //if (doNormalFiltering && !clearlyAJump) {
          //int radius=2;
          //int normalPoint1X = obstacle_point_x - (radius+1)*offset_x,
              //normalPoint1Y = obstacle_point_y - (radius+1)*offset_y,
              //normalPoint2X = shadow_point_x,
              //normalPoint2Y = shadow_point_y;
          //if (isInImage(normalPoint1X,normalPoint1Y) && !pcl_isinf(points[normalPoint1Y*width+normalPoint1X].range) &&
              //isInImage(normalPoint2X,normalPoint2Y) && !pcl_isinf(points[normalPoint2Y*width+normalPoint2X].range))
          //{
            //const PointWithRange& normalPoint1 = points[normalPoint1Y*width+normalPoint1X],
                                //& normalPoint2 = points[normalPoint2Y*width+normalPoint2X];
            //neighborhoodIndices.clear();
            //for (int y2=normalPoint1Y-radius; y2<=normalPoint1Y+radius; ++y2) {
              //for (int x2=normalPoint1X-radius; x2<=normalPoint1X+radius; ++x2) {
                //if (!isInImage(x2,y2) || pcl_isinf(points[y2*width+x2].range))  continue;
                //neighborhoodIndices.push_back(y2*width + x2);
              //}
            //}
            //Eigen::Vector4f planeParameters1;
            //float curvature1;
            //pcl::computePointNormal(range_image, neighborhoodIndices, planeParameters1, curvature1);
            //pcl::flipNormalTowardsViewpoint(normalPoint1, sensor_pos[0], sensor_pos[1], sensor_pos[2], planeParameters1);
            
            //neighborhoodIndices.clear();
            //for (int y2=normalPoint2Y-radius; y2<=normalPoint2Y+radius; ++y2) {
              //for (int x2=normalPoint2X-radius; x2<=normalPoint2X+radius; ++x2) {
                //if (!isInImage(x2,y2) || pcl_isinf(points[y2*width+x2].range))  continue;
                //neighborhoodIndices.push_back(y2*width + x2);
              //}
            //}
            //Eigen::Vector4f planeParameters2;
            //float curvature2;
            //pcl::computePointNormal(range_image, neighborhoodIndices, planeParameters2, curvature2);
            //pcl::flipNormalTowardsViewpoint(normalPoint2, sensor_pos[0], sensor_pos[1], sensor_pos[2], planeParameters2);
            
            //float cosAngleBetweenNormals = planeParameters1[0]*planeParameters2[0] +
                                           //planeParameters1[1]*planeParameters2[1] +
                                           //planeParameters1[2]*planeParameters2[2];
            
            //if (acosf(cosAngleBetweenNormals) < DEG2RAD(10.0f))
              //continue;
            ////cout << PVARAN(acosf(cosAngleBetweenNormals));
          //}
        //}

          ////neighborhoodIndices.clear();
          ////for (int y2=normalPoint2Y-radius; y2<=normalPoint2Y+radius; ++y2) {
            ////for (int x2=normalPoint2X-radius; x2<=normalPoint2X+radius; ++x2) {
              ////if (!isInImage(x2,y2) || pcl_isinf(points[y2*width+x2].range))  continue;
              ////neighborhoodIndices.push_back(y2*width + x2);
            ////}
          ////}
          ////Eigen::Vector4f planeParameters2;
          ////float curvature2;
          ////pcl::computePointNormal(range_image, neighborhoodIndices, planeParameters2, curvature2);
          ////pcl::flipNormalTowardsViewpoint(neighbor, sensor_pos[0], sensor_pos[1], sensor_pos[2], planeParameters2);

          
          ////neighborhoodIndices.clear();
          ////for (int y2=neighbor_y-radius; y2<=neighbor_y+radius; ++y2) {
            ////for (int x2=neighbor_x-radius; x2<=neighbor_x+radius; ++x2) {
              ////if (!isInImage(x2,y2) || pcl_isinf(points[y2*width+x2].range))  continue;
              ////neighborhoodIndices.push_back(y2*width + x2);
            ////}
          ////}
          ////Eigen::Vector4f planeParametersNeighbor;
          ////float curvatureNeighbor;
          ////pcl::computePointNormal(range_image, neighborhoodIndices, planeParametersNeighbor, curvatureNeighbor);
          ////pcl::flipNormalTowardsViewpoint(neighbor, sensor_pos[0], sensor_pos[1], sensor_pos[2], planeParametersNeighbor);
          
          ////float cosAngleBetweenNormals = planeParameters[0]*planeParametersNeighbor[0] +
                                         ////planeParameters[1]*planeParametersNeighbor[1] +
                                         ////planeParameters[2]*planeParametersNeighbor[2];
          //////cout << PVARN(cosAngleBetweenNormals);
          ////if (cosAngleBetweenNormals > cosf(DEG2RAD(2.0f)))
            ////reallyIsBorder = false;
        ////}

        ////if (reallyIsBorder && !pcl_isinf(point.range) && !pcl_isinf(neighbor.range)) {
          ////int radius=3;
          ////std::vector<int> neighborhoodIndices;
          ////for (int y2=y-radius; y2<=y+radius; ++y2) {
            ////for (int x2=x-radius; x2<=x+radius; ++x2) {
              ////if (!isInImage(x2,y2) || pcl_isinf(points[y2*width+x2].range))  continue;
              ////neighborhoodIndices.push_back(y2*width + x2);
            ////}
          ////}
          ////Eigen::Vector4f planeParameters;
          ////float curvature;
            ////pcl::computePointNormal(range_image, neighborhoodIndices, planeParameters, curvature);
          
          ////float distance1 = planeParameters[0]*point.x + planeParameters[1]*point.y + planeParameters[2]*point.z + planeParameters[3];
          ////float distance2 = planeParameters[0]*neighbor.x + planeParameters[1]*neighbor.y + planeParameters[2]*neighbor.z + planeParameters[3];
          
          ////float maxPlaneError = 0.02;
          ////if (fabs(distance1) < maxPlaneError && fabs(distance2) < maxPlaneError)
            ////reallyIsBorder = false;
        ////}

          ////float distanceDiff = 1.0f - min(fabs(distance1), fabs(distance2))/max(fabs(distance1), fabs(distance2));
          ////cout << distance1<<","<<distance2<<" => "<<distanceDiff << "\n";
          
          ////if (distanceDiff < 0.1)
            ////reallyIsBorder = false;
          //////pcl::flipNormalTowardsViewpoint(point, sensor_pos[0], sensor_pos[1], sensor_pos[2], planeParameters);
          
          //////float cosAngleBetweenNormals = planeParameters[0]*planeParametersNeighbor[0] +
                                         //////planeParameters[1]*planeParametersNeighbor[1] +
                                         //////planeParameters[2]*planeParametersNeighbor[2];
          //////cout << PVARN(cosAngleBetweenNormals);
          //////if (cosAngleBetweenNormals > cosf(DEG2RAD(2.0f)))
            //////reallyIsBorder = false;
        ////}
        
        
        //obstacle_border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = true;
        //shadow_border_traits[BORDER_TRAIT__SHADOW_BORDER] = true;
        
        //if (offset_x>0) {
          //obstacle_border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT] = true;
          //shadow_border_traits[BORDER_TRAIT__SHADOW_BORDER_LEFT] = true;
        //}
        //else if (offset_x<0) {
          //obstacle_border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT] = true;
          //shadow_border_traits[BORDER_TRAIT__SHADOW_BORDER_RIGHT] = true;
        //}
        //else if (offset_y>0) {
          //obstacle_border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM] = true;
          //shadow_border_traits[BORDER_TRAIT__SHADOW_BORDER_TOP] = true;
        //}
        //else {
          //obstacle_border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP] = true;
          //shadow_border_traits[BORDER_TRAIT__SHADOW_BORDER_BOTTOM] = true;
        //}
      //}
    //}
  //}
//}


//namespace { // Anonymous namespace for local struct - only usable in this file
  //struct RegionGrowingElement 
  //{
    //RegionGrowingElement(float distance, int index) : distance(distance), index(index) {}
    //float distance;
    //int index;
    //bool operator < (const RegionGrowingElement& other) const {return (distance!=other.distance ? distance<other.distance : index<other.index);}
  //};
//}


//Eigen::Vector3f** RangeImageBorderExtractor::getBorderDirections(const RangeImage& range_image, const PointCloud<BorderDescription>& border_descriptions,
                                                                 //float search_radius)
//{
  //MEASURE_FUNCTION_TIME;

  //bool useNormals = true;
  //int min_no_of_points_for_border_direction = 5;
  //int min_no_of_points_for_normal_estimation = 8;
  //int max_no_of_points_for_normal_estimation = 20;
  
  //Eigen::Vector3f sensor_pos = range_image.getSensorPos();
  
  //int width  = range_image.width,
      //height = range_image.height,
      //array_size = width*height;
  //float max_distance_squared = search_radius*search_radius,
        //max_distance_for_normal_squared = search_radius * search_radius;
  
//# if SHOW_DEBUG_IMAGES
    //float* angle_image = new float[array_size];
    //SET_ARRAY(angle_image, -std::numeric_limits<float>::infinity (), array_size);
//# endif
  
  //Eigen::Vector3f** border_direction_image = new Eigen::Vector3f*[array_size];
  
  //#pragma omp parallel for default(shared) schedule(dynamic, 20)
  //for (int y=0; y<height; ++y)
  //{
    //for (int x=0; x<width; ++x)
    //{
      //int point_index = y*width + x;
      //Eigen::Vector3f*& border_direction_ptr = border_direction_image[point_index];
      //border_direction_ptr = NULL;
      
      //const BorderDescription& border_description = border_descriptions.points[point_index];
      //const BorderTraits& border_traits = border_description.traits;
      //if (!border_traits[BORDER_TRAIT__OBSTACLE_BORDER] || border_traits[BORDER_TRAIT__SHADOW_BORDER])
        //continue;
      
      //const PointWithRange& point = range_image.getPoint(x, y);
      
      ////------------------------------------
      //// Region Growing to get normal vector
      //VectorAverage3f vector_average_surface;
      //std::set<int> checked_points;
      //std::set<RegionGrowingElement> unchecked_points;
      //unchecked_points.insert(RegionGrowingElement(0.0f, point_index));
      //while (useNormals && !unchecked_points.empty()) {
        //int neighbor_index = unchecked_points.begin()->index;
        //float distance_squared = unchecked_points.begin()->distance;
        //checked_points.insert(neighbor_index);
        //unchecked_points.erase(unchecked_points.begin());
        
        //const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
        //if (pcl_isinf(neighbor.range))
          //continue;
        //if (distance_squared > max_distance_for_normal_squared && (int)vector_average_surface.getNoOfSamples()>=min_no_of_points_for_normal_estimation)
          //continue;
        //const BorderTraits& neighbor_border_traits = border_descriptions.points[neighbor_index].traits;
        //if (neighbor_border_traits[BORDER_TRAIT__SHADOW_BORDER])
          //continue;
        ////cout << neighbor_index<<", "<<std::flush;
        
        //vector_average_surface.add(Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z));
        //if ((int)vector_average_surface.getNoOfSamples() >= max_no_of_points_for_normal_estimation)
          //break;
        
        ////if ((int)vector_average_surface.getNoOfSamples() + (int)unchecked_points.size() > lrint(1.5f*max_no_of_points_for_normal_estimation))
          ////continue;
        
        //int y2=neighbor_index/width, x2=neighbor_index - y2*width;
        //for (int y3=y2-1; y3<=y2+1; ++y3)
        //{
          //for (int x3=(y3==y2 ? x2-1 : x2); x3<=(y3==y2 ? x2+1 : x2); x3+=2)
          //{
            //int index3 = y3*width+x3;
            //if (range_image.isInImage(x3, y3) && checked_points.find(index3)==checked_points.end()) {
              //const PointWithRange& point3 = range_image.getPoint(index3);
              //float distance3_squared = squaredEuclideanDistance(point, point3);
              //unchecked_points.insert(RegionGrowingElement(distance3_squared, index3));
            //}
          //}
        //}
      //}
      ////cout << PVARN(vector_average_surface.getNoOfSamples());
      ////----------------------------------------------------------------------------------
      
      //Eigen::Vector3f direction_check_vector, surface_normal, surface_mean;
      //if (useNormals) {
        //Eigen::Vector3f eigen_values, eigen_vector1, eigen_vector2, eigen_vector3;
        //vector_average_surface.doPCA(eigen_values, eigen_vector1, eigen_vector2, eigen_vector3);
        //surface_mean = vector_average_surface.getMean();
        //surface_normal = eigen_vector1;
        //if (surface_normal.dot((sensor_pos-surface_mean).normalized()) < 0.0f)
          //surface_normal *= -1.0f;
        //direction_check_vector[0] = point.x-surface_mean[0];
        //direction_check_vector[1] = point.y-surface_mean[1];
        //direction_check_vector[2] = point.z-surface_mean[2];
        //direction_check_vector.normalize();
      //}
      //else
      //{
        //// TODO: Get vector direction based on the average border angle (mean during growing)
        //surface_normal[0] = sensor_pos[0]-point.x;
        //surface_normal[1] = sensor_pos[1]-point.y;
        //surface_normal[2] = sensor_pos[2]-point.z;
        //surface_normal.normalize();
        //surface_mean = Eigen::Vector3f(point.x, point.y, point.z);
        //direction_check_vector[0] = 1.0f;
        //direction_check_vector[1] = 0.0f;
        //direction_check_vector[2] = 0.0f;
        //direction_check_vector.normalize();
      //}
      
      ////----------------------------------------------------------------------------------
      //// Region Growing to get border points for the calculation of the dominant direction
      //VectorAverage3f vector_average_border;
      //unchecked_points.clear();
      //checked_points.clear();
      //unchecked_points.insert(RegionGrowingElement(0.0f, point_index));
      ////cout << "\n-----\n";
      //while (!unchecked_points.empty()) {
        //int neighbor_index = unchecked_points.begin()->index;
        //const BorderDescription& neighbor_border_description = border_descriptions.points[neighbor_index];
        //checked_points.insert(neighbor_index);
        //unchecked_points.erase(unchecked_points.begin());
        //const PointWithRange& neighbor = range_image.getPoint(neighbor_index);
        
        ////cout << PVARN(sqrtf(distance_squared));
        //float distance_squared = squaredEuclideanDistance(point, neighbor);
        //if (distance_squared > max_distance_squared && (int)vector_average_border.getNoOfSamples()>=min_no_of_points_for_border_direction)
          //continue;
        
        //for (int i=0; i<(int)neighbor_border_description.neighbors.size(); ++i)
        //{
          //const BorderDescription* border_description3 = neighbor_border_description.neighbors[i];
          //int x3 = border_description3->x,
              //y3 = border_description3->y,
              //index3 = y3*width + x3;
          //if (checked_points.find(index3)==checked_points.end())
          //{
            //const PointWithRange& point3 = range_image.getPoint(index3);
            //float distance3_squared = squaredEuclideanDistance(point, point3);
            //unchecked_points.insert(RegionGrowingElement(distance3_squared, index3));
            ////float distance_2d = hypot(x3-x, y3-y);
            ////unchecked_points.insert(RegionGrowingElement(distance_2d, index3));
          //}
        //}
        
        ////cout << neighbor_border_description.x-x<<","<<neighbor_border_description.y-y<<" - ";
        //Eigen::Vector3f neighbor_e(neighbor.x, neighbor.y, neighbor.z),
                        //point_on_plane = (surface_normal.dot(surface_mean)-surface_normal.dot(neighbor_e))*surface_normal + neighbor_e;
        //vector_average_border.add(point_on_plane);
        ////vector_average_border.add(Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z));
      //}
      ////cout << PVARN(vector_average_border.getNoOfSamples());
        
      
      //if ((int)vector_average_border.getNoOfSamples() < min_no_of_points_for_border_direction)
        //continue;
      
      //Eigen::Vector3f eigen_values, eigen_vector1, eigen_vector2, eigen_vector3;
      //vector_average_border.doPCA(eigen_values, eigen_vector1, eigen_vector2, eigen_vector3);
      ////cout << "Eigenvalues: "<<eigen_values[0]<<", "<<eigen_values[1]<<", "<<eigen_values[2]<<"\n";
      

      //border_direction_ptr = new Eigen::Vector3f;
      //Eigen::Vector3f& border_direction = *border_direction_ptr;
      //border_direction = eigen_vector3;
      
      //Eigen::Transform3f transformation;
      //getTransformationFromTwoUnitVectors(direction_check_vector, surface_normal, transformation);
      //if ((transformation*border_direction)[0] < 0)
        //border_direction *= -1.0f;
      
      ////cout << border_direction[0]<<", "<<border_direction[1]<<", "<<border_direction[2]<<"\n";
      
//#     if SHOW_DEBUG_IMAGES
      //float border_direction_in_image_x, border_direction_in_image_y;
      //float tmp_factor = point.range*range_image.getAngularResolution();
      //range_image.getImagePoint(point.x+tmp_factor*border_direction[0], point.y+tmp_factor*border_direction[1], point.z+tmp_factor*border_direction[2],
                                //border_direction_in_image_x, border_direction_in_image_y);
      //border_direction_in_image_x -= x;  border_direction_in_image_y -= y;
      //angle_image[point_index] = atan2(border_direction_in_image_y, border_direction_in_image_x);
//#     endif
    //}
  //}
  
//# if SHOW_DEBUG_IMAGES
    //stringstream ss;
    //ss << " for radius "<<search_radius<<"m";
    //std::string for_radius = ss.str();
    
    //static RangeImageVisualizer* range_image_widget = new RangeImageVisualizer("Range Image with borders");
    //range_image_widget->setRangeImage(range_image);
    //for (int y=0; y<(int)range_image.height; ++y)
    //{
      //for (int x=0; x<(int)range_image.width; ++x)
      //{
        //const BorderDescription& border_description = border_descriptions.points[y*range_image.width + x];
        //const BorderTraits& border_traits = border_description.traits;
        //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER])
          //if (border_traits[BORDER_TRAIT__SHADOW_BORDER])
            //range_image_widget->markPoint(x, y, wxCYAN_PEN);
          //else
            //range_image_widget->markPoint(x, y, wxGREEN_PEN);
        //else if (border_traits[BORDER_TRAIT__SHADOW_BORDER])
          //range_image_widget->markPoint(x, y, wxRED_PEN);
        //for (unsigned int i=0; i<border_description.neighbors.size(); ++i)
        //{
          //range_image_widget->markLine(border_description.x, border_description.y,
                                      //border_description.neighbors[i]->x, border_description.neighbors[i]->y, wxGREEN_PEN);
        //}
      //}
    //}
    
    ////static ImageWidgetWX* angle_image_widget = new ImageWidgetWX;
    ////ImageWidgetWX* angle_image_widget = new ImageWidgetWX;
    ////angle_image_widget->setAngleImage(angle_image, width, height, ("angle image"+for_radius).c_str());
//#endif  // #if SHOW_DEBUG_IMAGES
  
  //return border_direction_image;
//}

#if 0
std::vector<float> acuteness_values, acuteness_change_values, acuteness_values_border, acuteness_change_values_border;
void RangeImageBorderExtractor::updateStatistic(float acuteness, float acuteness_change, bool is_border) const
{
  acuteness_values.push_back(fabs(acuteness));
  acuteness_change_values.push_back(acuteness_change);
  if (is_border)
  {
    //cout << PVARN(acuteness);
    acuteness_values_border.push_back(fabs(acuteness));
    acuteness_change_values_border.push_back(acuteness_change);
  }
}

void RangeImageBorderExtractor::outputStatistic() const
{
  float step_size = 0.1f;
  
  #define ADD_STATISTIC_OUTPUT(STATISTICS_VECTOR, PARAM_MEMBER, TMP_NAME) \
  { \
    std::sort(STATISTICS_VECTOR.begin(), STATISTICS_VECTOR.end()); \
    cout << "  float "<<#TMP_NAME<<"[11] = {1, "; \
    float current_step = 0.1f; \
    for (int idx=0; idx<int(STATISTICS_VECTOR.size()); ++idx) \
    { \
      float value = STATISTICS_VECTOR[idx]; \
      while (value > current_step) \
      { \
        cout << 1.0f - (float(idx)/float(STATISTICS_VECTOR.size()))<<", "; \
        current_step += step_size; \
      } \
    } \
    cout << "0};\n" \
         << "  std::copy("<<#TMP_NAME<<", "<<#TMP_NAME<<" + sizeof("<<#TMP_NAME<<")/sizeof(*"<<#TMP_NAME<<"),\n" \
            "  std::inserter("<<#PARAM_MEMBER<<".getDataPoints(),\n" \
         << "                "<<#PARAM_MEMBER<<".getDataPoints().end()));\n"; \
  }
  
  ADD_STATISTIC_OUTPUT(acuteness_values, probability_distribution_acuteness, pda_tmp);
  ADD_STATISTIC_OUTPUT(acuteness_values_border, probability_distribution_acuteness_given_border, pdagb_tmp);
  ADD_STATISTIC_OUTPUT(acuteness_change_values, probability_distribution_acuteness_change, pdac_tmp);
  ADD_STATISTIC_OUTPUT(acuteness_change_values_border, probability_distribution_acuteness_change_given_border, pdacgb_tmp);
  
  #undef ADD_STATISTIC_OUTPUT
}



// Check if the impact angle is small
//float RangeImageBorderExtractor::getImpactAngleBorderProbability(const RangeImage& range_image, int x1, int y1, int x2, int y2, float max_impact_angle) const
//{
  //int offset_x = x2-x1,
      //offset_y = y2-y1;
  
  //float max_border_impact_angle = deg2rad(15.0f);
  //if (!range_image.isInImage(x2, y2))
    //return 0.0f;
  //const PointWithRange& point1 = range_image.getPoint(x1, y1);
  //const PointWithRange& point2 = range_image.getPoint(x2, y2);
  //float impact_angle = range_image.getImpactAngle(point1, point2);
  ////if (fabs(impact_angle) > deg2rad(20.0f))
    ////return 0.0f;
  
  //float min_impact_angle = std::numeric_limits<float>::infinity ();
  //for (int i=1; i<=3; ++i)
  //{
    //int neighbor_x=x1+i*offset_x, neighbor_y=y1+i*offset_y;
    //if (!range_image.isValid(neighbor_x, neighbor_y))
      //continue;
    //const PointWithRange& neighbor = range_image.getPoint(neighbor_x, neighbor_y);
    //float neighbor_impact_angle = range_image.getImpactAngle(point1, neighbor);
    //if (neighbor_impact_angle < 0)
      //return 0.0f;
    //min_impact_angle = (std::min)(min_impact_angle, fabsf(neighbor_impact_angle));
  //}
  //if (min_impact_angle > max_border_impact_angle)
    //return 0.0f;
  
  //int other_neighbor_x=x1-offset_x, other_neighbor_y=y1-offset_y;
  //if (!range_image.isValid(other_neighbor_x, other_neighbor_y))
    //return 0;
  //const PointWithRange& other_neighbor = range_image.getPoint(other_neighbor_x, other_neighbor_y); 
  //float other_impact_angle = range_image.getImpactAngle(point1, other_neighbor);
  //if (other_impact_angle<0 || other_impact_angle > min_impact_angle + DEG2RAD(10.0f))
    //return 1;
  //else 
    //return 0;

  ////cout << PVARAN(impact_angle);
  
  ////float border_prob_angle = (fabs(impact_angle)>=max_impact_angle ? 0.0f : 1.0f - powf(fabs(impact_angle)/max_impact_angle, 2));
  
  ////return border_prob_angle;
//}

//float RangeImageBorderExtractor::getSurfaceChangeBorderProbability(const RangeImage& range_image, int x, int y, 
                                                                   //int offset_x, int offset_y, float max_surface_change_angle) const
//{
  //return 1;
  //float surface_change_x, surface_change_y;
  //float surface_change = range_image.getSurfaceChange(x, y, 3);
  //if (surface_change > 0.2)
    //return 1.0f;
  //else
  //{
    ////cout << PVARN(surface_change);
    //return 0.0f;
  //}

  ////const PointWithRange& point = range_image.getPoint(x, y);
  ////int x2=x+offset_x, y2=y+offset_y, x3=x-offset_x, y3=y-offset_y;
  ////if (!range_image.isInImage(x2, y2) || !range_image.isInImage(x3, y3))
    ////return 1.0f;
  ////const PointWithRange& neighbor1 = range_image.getPoint(x2, y2);
  ////const PointWithRange& neighbor2 = range_image.getPoint(x3, y3);
  ////if (!pcl_isfinite(neighbor1.range) || !pcl_isfinite(neighbor2.range))
    ////return 1.0f;
  
  ////float surface_change_angle = range_image.getSurfaceChange(point, neighbor1, neighbor2);
  ////if (surface_change_angle < max_surface_change_angle)
    ////return 1.0f;
  ////float border_prob_surface_change = (surface_change_angle < max_surface_change_angle ? 1.0f : 1.0f - pow((range_image.getSurfaceChange(point, neighbor1, neighbor2)-max_surface_change_angle)/(deg2rad(180.0f)-max_surface_change_angle), 3));
  //////cout << PVARN(border_prob_surface_change);
  
  ////return border_prob_surface_change;
//}

float RangeImageBorderExtractor::getNormalChangeBorderProbability(const RangeImage& range_image, int radius, int x, int y, int offset_x, int offset_y) const
{
  float ret = 0.0f;
  int neighbor_x = x+offset_x,
      neighbor_y = y+offset_y;
  const PointWithRange& point = range_image.getPoint(x, y);
  
  Eigen::Vector3f normal1, normal2;
  if (range_image.getNormal(neighbor_x, neighbor_y, 1, normal2, 1))
  {
    int other_neighbor_x = x-offset_x, other_neighbor_y = y-offset_y;
    if (range_image.getNormalForClosestNeighbors(other_neighbor_x, other_neighbor_y, 2, point, 8, normal1, 1))
    {
      Eigen::Vector3f viewing_direction = (range_image.getSensorPos() - range_image.getEigenVector3f(point)).normalized();
      
      float viewing_angle1 = acosf(fabs(viewing_direction.dot(normal1))),
            viewing_angle2 = acosf(fabs(viewing_direction.dot(normal2)));
      if (viewing_angle1 < viewing_angle2)
        ret = (viewing_angle2-viewing_angle1)/deg2rad(90.0f);
      //cout << PVARAC(viewing_angle1)<<PVARAC(viewing_angle2)<<PVARN(ret);
    }
  }
  return ret;
}

float RangeImageBorderExtractor::getDistanceChangeScore(const RangeImage& range_image, int x, int y, int offset_x, int offset_y, int max_pixel_distance) const
{
  float average_neighbor_distance1 = range_image.getAverageEuclideanDistance(x, y, offset_x, offset_y, max_pixel_distance),
        average_neighbor_distance2 = range_image.getAverageEuclideanDistance(x, y, -offset_x, -offset_y, max_pixel_distance);
  if ((pcl_isinf(average_neighbor_distance1) && pcl_isinf(average_neighbor_distance2)) ||
      average_neighbor_distance1<0.0f || average_neighbor_distance2<0.0)
    return 0.0f;
  if (pcl_isinf(average_neighbor_distance1) || pcl_isinf(average_neighbor_distance2))
    return 1.0f;
  float distance_change_score = (std::min)(average_neighbor_distance1, average_neighbor_distance2) /
                                (std::max)(average_neighbor_distance1, average_neighbor_distance2);
  distance_change_score = 1.0f - pow(distance_change_score, 1);
  return distance_change_score;
}


float RangeImageBorderExtractor::getBorderProbability(const RangeImage& range_image, int x, int y, int offset_x, int offset_y,
                                                      const float* acuteness_value_image_x, const float* acuteness_value_image_y) const
{
  float acuteness = getPrecalculatedAcuteness(x, y, offset_x, offset_y, range_image.width, range_image.height, acuteness_value_image_x, acuteness_value_image_y);
  
  if (!pcl_isfinite(acuteness))
    return 0.0f;
  
  //if (fabs(acuteness) < 0.80f)
    //return 0.0f;
  
  float prob_acuteness = parameters_.probability_distribution_acuteness.getValue(fabsf(acuteness)),
        prob_acuteness_given_border = parameters_.probability_distribution_acuteness_given_border.getValue(fabsf(acuteness)),
        border_prob_acuteness_only = (prob_acuteness_given_border*parameters_.border_probability_prior)/prob_acuteness;
  float border_prob = border_prob_acuteness_only;
  
  //if (border_prob_acuteness_only < parameters_.minimum_border_probability)
    //return 0.0f;
  
  float acuteness_other_side = getPrecalculatedAcuteness(x, y, -offset_x, -offset_y, range_image.width, range_image.height,
                                                         acuteness_value_image_x, acuteness_value_image_y);
  if (pcl_isinf(acuteness_other_side))
    return 0.0f;
  float acuteness_change = 0.5f*(acuteness+acuteness_other_side);
  //if (acuteness*acuteness_change <= 0.0f)  // Other direction actually more acute?
    //return 0.0f;
  
  float prob_acuteness_change = parameters_.probability_distribution_acuteness_change.getValue(fabsf(acuteness_change)),
        prob_acuteness_change_given_border = parameters_.probability_distribution_acuteness_change_given_border.getValue(fabsf(acuteness_change));
  border_prob *= (prob_acuteness_change_given_border)/prob_acuteness_change;
  
  float average_acuteness = getAverageAcuteness(range_image, x, y, offset_x, offset_y, acuteness),
        average_acuteness_other_side = getAverageAcuteness(range_image, x, y, -offset_x, -offset_y, acuteness_other_side),
        average_acuteness_change = 0.5f*(average_acuteness + average_acuteness_other_side);
  if (average_acuteness*average_acuteness_change <= 0.0f)  // Other direction actually more acute?
    return 0.0f;

   float prob_average_acuteness = parameters_.probability_distribution_acuteness.getValue(fabsf(average_acuteness)),
         prob_average_acuteness_given_border = parameters_.probability_distribution_acuteness_given_border.getValue(fabsf(average_acuteness)),
         prob_average_acuteness_change = parameters_.probability_distribution_acuteness_change.getValue(fabsf(average_acuteness_change)),
         prob_average_acuteness_change_given_border = parameters_.probability_distribution_acuteness_change_given_border.getValue(fabsf(average_acuteness_change));
   
   float border_prob_from_average_acuteness = (prob_average_acuteness_given_border*prob_average_acuteness_change_given_border*parameters_.border_probability_prior) /
                                                 (prob_average_acuteness*prob_average_acuteness_change);
   if (!pcl_isfinite(border_prob_from_average_acuteness))
     return 0.0f;
   border_prob = (std::min)(border_prob, border_prob_from_average_acuteness);

  if (average_acuteness < 0.0f)
    border_prob *= -1.0f;
   
  //if (fabs(average_acuteness) < 0.80f)
    //border_prob = 0.0f;
   
   
   //border_prob = getDistanceChangeScore(range_image, x, y, offset_x, offset_y, parameters_.pixel_radius_borders);
   
  //float acuteness_change_using_normal = 0.0f;
  //if (range_image.isValid(x,y) && range_image.isValid(x+offset_x, y+offset_y))
  //{
    //acuteness_change_using_normal = getNormalChangeBorderProbability(range_image, 1, x, y, offset_x, offset_y);
  //}
  

 

  //if (prob_acuteness_change_given_border > prob_acuteness_change/prob_border)
    //prob_acuteness_change_given_border = prob_acuteness_change/prob_border;
  //float prob_acuteness_change_given_border = prob_acuteness_change;//1.0f - powf(fabsf(acuteness_change), 1);
  
  
  
  //if (border_prob < 0.0f || border_prob>1.0f)
  //{
    //cout << PVARC(acuteness)<<PVARC(acuteness_change)
         //<< PVARC(prob_border)<<PVARC(prob_acuteness)<<PVARC(prob_acuteness_given_border)
         //<< PVARC(prob_acuteness_change)<<PVARC(prob_acuteness_change_given_border)
         //<< PVARN(border_prob);
  //}
  
  
  
  //float weight_acuteness_distant = 1.0f, weight_acuteness_direct_neighbor = 1.0f, weight_acuteness_change = 4.0f;
  //float max_value = 0.0f;
  //float border_score = 0.0f;
  
  //border_score += weight_acuteness_distant*fabsf(acuteness);
  //max_value += weight_acuteness_distant;
  
  //if (acuteness*acuteness_change < 0.0f)  // Other direction actually more acute?
    //return 0.0f;
  //border_score += weight_acuteness_change*fabsf(acuteness_change);
  //max_value += weight_acuteness_change;
  
  //float acuteness_direct_neighbor = range_image.getAcutenessValue(x, y, x+offset_x, y+offset_y);
  //border_score += weight_acuteness_direct_neighbor*fabsf(acuteness_direct_neighbor);
  //max_value += weight_acuteness_direct_neighbor;

  ////if (range_image.isValid(x,y) && range_image.isValid(x+offset_x, y+offset_y))
  ////{
    ////float normal_change = getNormalChangeBorderProbability(range_image, 1, x, y, x+offset_x, y+offset_y);
    ////border_score += normal_change;
  ////}



  ////cout << __func__<<"("<<PVARC(x)<<PVARC(y)<<PVARC(offset_x)<<PVARC(offset_y)<<PVARC(max_pixel_distance)<<PVAR(acuteness)<<")\n";
  
  
  
  ////border_score *= pow(fabsf(acuteness), 1);
  ////cout << acuteness_change << "\n";
  ////border_score *= 0.5f * fabs(acuteness_change);
  ////border_score *= fabsf(acuteness-range_image.getAcutenessValue(x, y, x-offset_x, y-offset_y));
  ////border_score *= fabsf(range_image.getAcutenessValue(x, y, x+offset_x, y+offset_y));

  ////border_score *= fabs(range_image.getAcutenessValue(x, y, x+3*offset_x, y+3*offset_y));
  
  ////float normal_based_acuteness = range_image.getNormalBasedAcutenessValue(x, y, 2);
  
  ////border_score *= (std::max)(0.0f, fabsf(acuteness) - normal_based_acuteness);
  
  ////float distance_change_score = getDistanceChangeScore(range_image, x, y, offset_x, offset_y, max_pixel_distance);
  ////border_score *= distance_change_score;
  
  ////int other_neighbor_x=x-offset_x, other_neighbor_y=y-offset_y;
  ////float other_neighbor_acuteness = range_image.getAcutenessValue(other_neighbor_x, other_neighbor_y, x, y);
  ////float other_neighbor_score = (pcl_isinf(other_neighbor_acuteness) ? 0.0f : (std::min)(1.0f, fabsf(acuteness-other_neighbor_acuteness)));
  ////border_score = other_neighbor_score * fabsf(acuteness);
  ////if (range_image.isValid(x,y) && range_image.isValid(x+offset_x, y+offset_y))
  ////{
    ////float normal_change_score = getNormalChangeBorderProbability(range_image, 1, x, y, x+offset_x, y+offset_y);
    ////border_score *= 3*normal_change_score;
  ////}
  ////float surface_change = range_image.getSurfaceChange(x, y, max_pixel_distance);
  ////if (pcl_isinf(surface_change) || surface_change * acuteness < 0.0f)
    ////return 0.0f;
  ////border_score *= fabs(surface_change);
  
  ////Eigen::Vector3f normal1, normal2;
  ////range_image.getNormal(x, y, 1, normal1, 1);
  ////range_image.getNormalForClosestNeighbors(x, y, 2, range_image.getPoint(x,y), 8,normal2, 1);
  ////border_score *= acosf(normal1.dot(normal2))/deg2rad(180.0f);
  
  ////border_score = pow(border_score, 0.5f);
  
  //border_score /= max_value;

  
  //if (border_prob > parameters_.minimum_border_probability)
    //cout << PVARC(acuteness)<<PVARC(prob_acuteness)<<PVARC(prob_acuteness_given_border)<<PVARC(border_prob_acuteness_only)<<PVARN(border_prob);
  
  //if (border_prob > parameters_.minimum_border_probability)
  //{
    //cout << "Y: ";
  //}
  //cout << PVARN(acuteness-acuteness_other_side);

  //cout << PVARC(border_prob)<<PVARN(parameters_.minimum_border_probability);
  //updateStatistic(average_acuteness, average_acuteness_change, border_prob>parameters_.minimum_border_probability);
  //updateStatistic(range_image, x, y, offset_x, offset_y, acuteness, acuteness_change, border_prob>parameters_.minimum_border_probability);

  //if (acuteness_direct_neighbor < 0.0f)
    //border_prob *= -1.0f;
  //cout << PVARC(x)<<PVARC(y)<<PVARC(offset_x)<<PVARC(offset_y)<<PVARC(acuteness)<<PVARN(border_score);
  return border_prob;
}

float RangeImageBorderExtractor::getPrecalculatedAcuteness(int x, int y, int offset_x, int offset_y, int width, int height,
                                                           const float* acuteness_value_image_x, const float* acuteness_value_image_y) const
{
  offset_x *= parameters_.pixel_radius_borders;
  offset_y *= parameters_.pixel_radius_borders;
  if (offset_y == 0)
  {
    if (offset_x > 0)
      return (acuteness_value_image_x[y*width+x]);
    else
      return (x+offset_x >= 0 ? -acuteness_value_image_x[y*width + x+offset_x] : -std::numeric_limits<float>::infinity ());
  }
  else
  {
    if (offset_y > 0)
      return (acuteness_value_image_y[y*width+x]);
    else
      return (y+offset_y >= 0 ? -acuteness_value_image_y[(y+offset_y)*width + x] : -std::numeric_limits<float>::infinity ());
  }
}

float RangeImageBorderExtractor::getAverageAcuteness(const RangeImage& range_image, int x, int y,
                                                     int offset_x, int offset_y, float acuteness_in_pixel_radius_borders) const
{
  float average_acuteness = acuteness_in_pixel_radius_borders;
  for (int i=1; i<parameters_.pixel_radius_borders; ++i)
  {
    int neighbor_x = x+i*offset_x,
        neighbor_y = y+i*offset_y;
    average_acuteness += range_image.getAcutenessValue(x, y, neighbor_x, neighbor_y);
  }
  average_acuteness /= parameters_.pixel_radius_borders;
  return average_acuteness;
}

//bool RangeImageBorderExtractor::checkPotentialBorder(const RangeImage& range_image, int x, int y, int offset_x, int offset_y,
                                                     //float* border_scores, float* border_scores_other_direction, int& shadow_border_idx) const
//{
  //float border_score = border_scores[y*range_image.width+x];
  //if (border_score<parameters_.minimum_border_probability)
    //return false;
  //int neighbor_x=x-offset_x, neighbor_y=y-offset_y;
  //float neighbor_border_score = border_scores[neighbor_y*range_image.width+neighbor_x];
  //if (range_image.isInImage(neighbor_x, neighbor_y) && neighbor_border_score>border_score)
    //return false;
  
  //shadow_border_idx = -1;
  //float best_shadow_border_score = -0.5*parameters_.minimum_border_probability;
  
  //for (int neighbor_distance=1; neighbor_distance<=parameters_.pixel_radius_borders; ++neighbor_distance)
  //{
    //neighbor_x=x+neighbor_distance*offset_x; neighbor_y=y+neighbor_distance*offset_y;
    //if (!range_image.isInImage(neighbor_x, neighbor_y))
     //break;
    //if (shadow_border_idx < 0)  // If we already found a valid shadow border, we don't have to check for better points anymore
    //{
      //float neighbor_border_score = border_scores[neighbor_y*range_image.width+neighbor_x];
      //if (neighbor_border_score > border_score)
        //return false;
    //}
    //float neighbor_shadow_border_score = border_scores_other_direction[neighbor_y*range_image.width+neighbor_x];
    
    //if (neighbor_shadow_border_score < best_shadow_border_score)
    //{
      //shadow_border_idx = neighbor_y*range_image.width + neighbor_x;
      //best_shadow_border_score = neighbor_shadow_border_score;
    //}
    ////else if (shadow_border_idx >= 0)  // We expect to find the maximum with a constant increase
      ////break;
  //}
  //if (shadow_border_idx > 0)
  //{
    //return true;
  //}
  //return false;
//}
#endif

//}  // namespace end
}  // namespace end

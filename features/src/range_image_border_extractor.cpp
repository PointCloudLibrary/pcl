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

#include <iostream>
using std::cout;
using std::cerr;
#include <cmath>
#include <pcl/pcl_macros.h>
#include <pcl/range_image/range_image.h>
#include <pcl/point_cloud.h>
#include <pcl/features/range_image_border_extractor.h>
#include <Eigen/Core> // for Vector3f

namespace pcl
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
RangeImageBorderExtractor::RangeImageBorderExtractor(const RangeImage* range_image) :
  range_image_(range_image), range_image_size_during_extraction_(0),
  surface_structure_(nullptr), border_descriptions_(nullptr), shadow_border_informations_(nullptr), border_directions_(nullptr),
  surface_change_scores_(nullptr), surface_change_directions_(nullptr)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
RangeImageBorderExtractor::~RangeImageBorderExtractor()
{
  clearData ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::setRangeImage (const RangeImage* range_image)
{
  clearData ();
  range_image_ = range_image;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::clearData ()
{
  //std::cout << PVARC(range_image_size_during_extraction_)<<PVARN((void*)this);
  for (int i=0; i<range_image_size_during_extraction_; ++i)
  {
    if (surface_structure_!=nullptr)
      delete surface_structure_[i];
    if (shadow_border_informations_!=nullptr)
      delete shadow_border_informations_[i];
    if (border_directions_!=nullptr)
      delete border_directions_[i];
  }
  delete[] surface_structure_; surface_structure_ = nullptr;
  delete[] shadow_border_informations_; shadow_border_informations_ = nullptr;
  delete[] border_directions_; border_directions_ = nullptr;
  delete border_descriptions_; border_descriptions_ = nullptr;

  delete[] surface_change_scores_;  surface_change_scores_ = nullptr;
  delete[] surface_change_directions_;  surface_change_directions_ = nullptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::extractLocalSurfaceStructure ()
{
  if (surface_structure_ != nullptr)
    return;
  //std::cerr << __PRETTY_FUNCTION__<<" called (this="<<(void*)this<<").\n";
  //MEASURE_FUNCTION_TIME;

  const auto width  = range_image_->width;
  const auto height = range_image_->height;
  range_image_size_during_extraction_ = width*height;
  const auto array_size = range_image_size_during_extraction_;
  surface_structure_ = new LocalSurface*[array_size];
  const auto step_size = std::max(1, parameters_.pixel_radius_plane_extraction/2);
  //std::cout << PVARN(step_size);
  const auto sqrt_neighbors = parameters_.pixel_radius_plane_extraction/step_size + 1;
  const auto no_of_nearest_neighbors = sqrt_neighbors * sqrt_neighbors;

  // iteration_type should be at least as big as unsigned int (decltype of height)
  // But OpenMP requires signed size. Here we choose the minimum size that fits the bill
  using iteration_type = std::conditional_t<sizeof(int) == sizeof(long int), long long int, long int>;

#if OPENMP_LEGACY_CONST_DATA_SHARING_RULE
#pragma omp parallel for \
  default(none) \
  schedule(dynamic, 10) \
  num_threads(parameters_.max_no_of_threads)
#else
#pragma omp parallel for \
  default(none) \
  shared(height, no_of_nearest_neighbors, step_size, width) \
  schedule(dynamic, 10) \
  num_threads(parameters_.max_no_of_threads)
#endif
  for (iteration_type y=0; y<height; ++y)
  {
    for (unsigned int x=0; x<width; ++x)
    {
      std::size_t index = y*width + x;
      LocalSurface*& local_surface = surface_structure_[index];
      local_surface = nullptr;
      if (!range_image_->isValid(index))
        continue;
      local_surface = new LocalSurface;
      Eigen::Vector3f point;
      range_image_->getPoint(x, y, point);
      //std::cout << PVARN(point);
      if (!range_image_->getSurfaceInformation(x, y, parameters_.pixel_radius_plane_extraction, point,
                                  no_of_nearest_neighbors, step_size, local_surface->max_neighbor_distance_squared,
                                  local_surface->normal_no_jumps, local_surface->neighborhood_mean_no_jumps,
                                  local_surface->eigen_values_no_jumps,  &local_surface->normal,
                                  &local_surface->neighborhood_mean, &local_surface->eigen_values))
      {
        delete local_surface;
        local_surface = nullptr;
      }

      //std::cout << x<<","<<y<<": ("<<local_surface->normal_no_jumps[0]<<","<<local_surface->normal_no_jumps[1]<<","<<local_surface->normal_no_jumps[2]<<")\n";
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::extractBorderScoreImages ()
{
  if (!border_scores_left_.empty())
    return;

  extractLocalSurfaceStructure();

  //MEASURE_FUNCTION_TIME;

  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  border_scores_left_.resize (size);
  border_scores_right_.resize (size);
  border_scores_top_.resize (size);
  border_scores_bottom_ .resize (size);

#pragma omp parallel for \
  default(none) \
  shared(height, width) \
  schedule(dynamic, 10) \
  num_threads(parameters_.max_no_of_threads)
  for (int y=0; y<height; ++y) {
    for (int x=0; x<width; ++x) {
      int index = y*width + x;
      float& left=border_scores_left_[index]; float& right=border_scores_right_[index];
      float& top=border_scores_top_[index]; float& bottom=border_scores_bottom_[index];
      LocalSurface* local_surface_ptr = surface_structure_[index];
      if (local_surface_ptr==nullptr)
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
float*
RangeImageBorderExtractor::updatedScoresAccordingToNeighborValues (const float* border_scores) const
{
  float* new_scores = new float[range_image_->width*range_image_->height];
  float* new_scores_ptr = new_scores;
  for (int y=0; y < static_cast<int> (range_image_->height); ++y)
    for (int x=0; x < static_cast<int> (range_image_->width); ++x)
      *(new_scores_ptr++) = updatedScoreAccordingToNeighborValues(x, y, border_scores);
  return (new_scores);
}

std::vector<float>
RangeImageBorderExtractor::updatedScoresAccordingToNeighborValues (const std::vector<float>& border_scores) const
{
  std::vector<float> new_border_scores;
  new_border_scores.reserve (range_image_->width*range_image_->height);
  for (int y=0; y < static_cast<int> (range_image_->height); ++y)
    for (int x=0; x < static_cast<int> (range_image_->width); ++x)
      new_border_scores.push_back (updatedScoreAccordingToNeighborValues(x, y, border_scores.data ()));
  return new_border_scores;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::updateScoresAccordingToNeighborValues ()
{
  extractBorderScoreImages();

  //MEASURE_FUNCTION_TIME;

  border_scores_left_ = updatedScoresAccordingToNeighborValues(border_scores_left_);
  border_scores_right_ = updatedScoresAccordingToNeighborValues(border_scores_right_);
  border_scores_top_ = updatedScoresAccordingToNeighborValues(border_scores_top_);
  border_scores_bottom_ = updatedScoresAccordingToNeighborValues(border_scores_bottom_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::findAndEvaluateShadowBorders ()
{
  if (shadow_border_informations_ != nullptr)
    return;

  if (border_scores_left_.empty ())
  {
    std::cerr << __PRETTY_FUNCTION__<<": border score images not available!\n";
  }

  //MEASURE_FUNCTION_TIME;

  int width  = range_image_->width,
      height = range_image_->height;
  shadow_border_informations_ = new ShadowBorderIndices*[width*height];
  for (int y = 0; y < static_cast<int> (height); ++y)
  {
    for (int x = 0; x < static_cast<int> (width); ++x)
    {
      int index = y*width+x;
      ShadowBorderIndices*& shadow_border_indices = shadow_border_informations_[index];
      shadow_border_indices = nullptr;
      int shadow_border_idx;

      if (changeScoreAccordingToShadowBorderValue(x, y, -1, 0, border_scores_left_.data (), border_scores_right_.data (), shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==nullptr ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->left = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 1, 0, border_scores_right_.data (), border_scores_left_.data (), shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==nullptr ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->right = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 0, -1, border_scores_top_.data (), border_scores_bottom_.data (), shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==nullptr ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->top = shadow_border_idx;
      }
      if (changeScoreAccordingToShadowBorderValue(x, y, 0, 1, border_scores_bottom_.data (), border_scores_top_.data (), shadow_border_idx))
      {
        shadow_border_indices = (shadow_border_indices==nullptr ? new ShadowBorderIndices : shadow_border_indices);
        shadow_border_indices->bottom = shadow_border_idx;
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
float*
RangeImageBorderExtractor::getAnglesImageForBorderDirections ()
{
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
      if (border_direction_ptr == nullptr)
        continue;
      const Eigen::Vector3f& border_direction = *border_direction_ptr;
      const PointWithRange& point = range_image_->getPoint(index);

      float border_direction_in_image_x, border_direction_in_image_y;
      float tmp_factor = point.range*range_image_->getAngularResolution();
      range_image_->getImagePoint(point.x+tmp_factor*border_direction[0], point.y+tmp_factor*border_direction[1], point.z+tmp_factor*border_direction[2],
                                border_direction_in_image_x, border_direction_in_image_y);
      border_direction_in_image_x -= static_cast<float> (x);  border_direction_in_image_y -= static_cast<float> (y);
      angle = std::atan2 (border_direction_in_image_y, border_direction_in_image_x);
    }
  }
  return angles_image;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
float*
RangeImageBorderExtractor::getAnglesImageForSurfaceChangeDirections ()
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
      border_direction_in_image_x -= static_cast<float> (x);  border_direction_in_image_y -= static_cast<float> (y);
      angle = std::atan2 (border_direction_in_image_y, border_direction_in_image_x);
      if (angle <= deg2rad (-90.0f))
        angle += static_cast<float> (M_PI);
      else if (angle > deg2rad (90.0f))
        angle -= static_cast<float> (M_PI);
    }
  }
  return (angles_image);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::classifyBorders ()
{
  if (border_descriptions_ != nullptr)
    return;

  // Get local plane approximations
  extractLocalSurfaceStructure();

  // Get scores for every point, describing how probable a border in that direction is
  extractBorderScoreImages();

  // Propagate values to neighboring pixels
  updateScoresAccordingToNeighborValues();

  // Change border score according to the existence of a shadow border
  findAndEvaluateShadowBorders();

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

  for (int y = 0; y < static_cast<int> (height); ++y)
  {
    for (int x = 0; x < static_cast<int> (width); ++x)
    {
      int index = y*width+x;
      BorderDescription& border_description = (*border_descriptions_)[index];
      border_description.x = x;
      border_description.y = y;
      BorderTraits& border_traits = border_description.traits;

      ShadowBorderIndices* shadow_border_indices = shadow_border_informations_[index];
      if (shadow_border_indices == nullptr)
        continue;

      int shadow_border_index = shadow_border_indices->left;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, -1, 0, border_scores_left_.data (), shadow_border_index))
      {
        BorderTraits& shadow_traits = (*border_descriptions_)[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_RIGHT] = true;
        for (int index3=index-1; index3>shadow_border_index; --index3)
        {
          BorderTraits& veil_point = (*border_descriptions_)[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_RIGHT] = true;
        }
      }

      shadow_border_index = shadow_border_indices->right;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 1, 0, border_scores_right_.data (), shadow_border_index))
      {
        BorderTraits& shadow_traits = (*border_descriptions_)[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_LEFT] = true;
        for (int index3=index+1; index3<shadow_border_index; ++index3)
        {
          BorderTraits& veil_point = (*border_descriptions_)[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_LEFT] = true;
        }
      }

      shadow_border_index = shadow_border_indices->top;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 0, -1, border_scores_top_.data (), shadow_border_index))
      {
        BorderTraits& shadow_traits = (*border_descriptions_)[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_BOTTOM] = true;
        for (int index3=index-width; index3>shadow_border_index; index3-=width)
        {
          //std::cout << "Adding veil point at "<<(index3-index)%width<<","<<(index3-index)/width<<".\n";
          BorderTraits& veil_point = (*border_descriptions_)[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_BOTTOM] = true;
        }
      }

      shadow_border_index = shadow_border_indices->bottom;
      if (shadow_border_index >= 0 && checkIfMaximum(x, y, 0, 1, border_scores_bottom_.data (), shadow_border_index))
      {
        BorderTraits& shadow_traits = (*border_descriptions_)[shadow_border_index].traits;
        border_traits[BORDER_TRAIT__OBSTACLE_BORDER] = border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM] = true;
        shadow_traits[BORDER_TRAIT__SHADOW_BORDER] = shadow_traits[BORDER_TRAIT__SHADOW_BORDER_TOP] = true;
        for (int index3=index+width; index3<shadow_border_index; index3+=width)
        {
          //std::cout << "Adding veil point at "<<(index3-index)%width<<","<<(index3-index)/width<<".\n";
          BorderTraits& veil_point = (*border_descriptions_)[index3].traits;
          veil_point[BORDER_TRAIT__VEIL_POINT] = veil_point[BORDER_TRAIT__VEIL_POINT_TOP] = true;
        }
      }

      //if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER])
      //{
        //border_points.push_back(&border_description);
      //}
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::calculateBorderDirections ()
{
  if (border_directions_!=nullptr)
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

  auto** average_border_directions = new Eigen::Vector3f*[size];
  int radius = parameters_.pixel_radius_border_direction;
  int minimum_weight = radius+1;
  float min_cos_angle=std::cos(deg2rad(120.0f));
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      Eigen::Vector3f*& average_border_direction = average_border_directions[index];
      average_border_direction = nullptr;
      const Eigen::Vector3f* border_direction = border_directions_[index];
      if (border_direction==nullptr)
        continue;
      average_border_direction = new Eigen::Vector3f(*border_direction);
      float weight_sum = 1.0f;
      for (int y2=(std::max)(0, y-radius); y2<=(std::min)(y+radius, height-1); ++y2)
      {
        for (int x2=(std::max)(0, x-radius); x2<=(std::min)(x+radius, width-1); ++x2)
        {
          int index2 = y2*width + x2;
          const Eigen::Vector3f* neighbor_border_direction = border_directions_[index2];
          if (neighbor_border_direction==nullptr || index2==index)
            continue;

          // Opposite directions?
          float cos_angle = neighbor_border_direction->dot(*border_direction);
          if (cos_angle<min_cos_angle)
          {
            //std::cout << "Reject. "<<PVARC(min_cos_angle)<<PVARC(cos_angle)<<PVARAN(acosf(cos_angle));
            continue;
          }
          //else
            //std::cout << "No reject\n";

          // Border in between?
          float border_between_points_score = getNeighborDistanceChangeScore(*surface_structure_[index], x, y, x2-x,  y2-y, 1);
          if (std::abs(border_between_points_score) >= 0.95f*parameters_.minimum_border_probability)
            continue;

          *average_border_direction += *neighbor_border_direction;
          weight_sum += 1.0f;
        }
      }
      if (pcl_lrint (weight_sum) < minimum_weight)
      {
        delete average_border_direction;
        average_border_direction=nullptr;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::calculateSurfaceChanges ()
{
  if (surface_change_scores_!=nullptr)
    return;

  calculateBorderDirections();

  //MEASURE_FUNCTION_TIME;

  int width  = range_image_->width,
      height = range_image_->height,
      size   = width*height;
  surface_change_scores_ = new float[size];
  surface_change_directions_ = new Eigen::Vector3f[size];
#pragma omp parallel for \
  default(none) \
  shared(height, width) \
  schedule(dynamic, 10) \
  num_threads(parameters_.max_no_of_threads)
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float& surface_change_score = surface_change_scores_[index];
      surface_change_score = 0.0f;
      Eigen::Vector3f& surface_change_direction = surface_change_directions_[index];
      surface_change_direction.setZero();

      const BorderTraits& border_traits = (*border_descriptions_)[index].traits;
      if (border_traits[BORDER_TRAIT__VEIL_POINT] || border_traits[BORDER_TRAIT__SHADOW_BORDER])
        continue;
      if (border_directions_[index]!=nullptr)
      {
        surface_change_score = 1.0f;
        surface_change_direction = *border_directions_[index];
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::blurSurfaceChanges ()
{
  int blur_radius = 1;

  const RangeImage& range_image = *range_image_;

  auto* blurred_directions = new Eigen::Vector3f[range_image.width*range_image.height];
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
      const BorderTraits& border_traits = (*border_descriptions_)[index].traits;
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
            //if (std::abs(neighbor.norm()-1) > 1e-4)
              //std::cerr<<PVARN(neighbor)<<PVARN(score);
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::computeFeature (PointCloudOut& output)
{
  output.clear();

  if (indices_)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": Sorry, usage of indices for the extraction is not supported for range image border extraction (yet).\n\n";
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  if (range_image_==nullptr)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImage is not set. Sorry, the border extraction works on range images, not on normal point clouds."
              << " Use setRangeImage(...).\n\n";
    output.width = output.height = 0;
    output.clear ();
    return;
  }
  output = getBorderDescriptions ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
RangeImageBorderExtractor::compute (PointCloudOut& output)
{
  computeFeature (output);
}

}  // namespace end

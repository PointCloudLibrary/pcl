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
 *
 */

#include <iostream>
#include <vector>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/polynomial_calculations.h>
#include <pcl/range_image/range_image.h>

namespace pcl 
{

/////////////////////////////////////////////////////////////////////////
NarfKeypoint::NarfKeypoint (RangeImageBorderExtractor* range_image_border_extractor, float support_size) :
    BaseClass (), interest_image_ (NULL), interest_points_ (NULL)
{
  name_ = "NarfKeypoint";
  clearData ();
  setRangeImageBorderExtractor (range_image_border_extractor);
  if (support_size > 0.0f)
    parameters_.support_size = support_size;
}

/////////////////////////////////////////////////////////////////////////
NarfKeypoint::~NarfKeypoint ()
{
  //std::cerr << __PRETTY_FUNCTION__<<" called.\n";
  clearData ();
}

/////////////////////////////////////////////////////////////////////////
void
  NarfKeypoint::clearData ()
{
  //std::cerr << __PRETTY_FUNCTION__<<" called.\n";
  
  for (size_t scale_space_idx = 1; scale_space_idx<border_extractor_scale_space_.size (); ++scale_space_idx)
    delete border_extractor_scale_space_[scale_space_idx];
  border_extractor_scale_space_.clear ();
  for (size_t scale_space_idx = 1; scale_space_idx<range_image_scale_space_.size (); ++scale_space_idx)
    delete range_image_scale_space_[scale_space_idx];
  range_image_scale_space_.clear ();
  for (size_t scale_space_idx = 1; scale_space_idx<interest_image_scale_space_.size (); ++scale_space_idx)
    delete[] interest_image_scale_space_[scale_space_idx];
  interest_image_scale_space_.clear ();
  is_interest_point_image_.clear ();
  delete[] interest_image_; interest_image_=NULL;
  delete interest_points_;  interest_points_=NULL;
}

/////////////////////////////////////////////////////////////////////////
void
NarfKeypoint::setRangeImageBorderExtractor (RangeImageBorderExtractor* range_image_border_extractor)
{
  clearData ();
  range_image_border_extractor_ = range_image_border_extractor;
}

/////////////////////////////////////////////////////////////////////////
void
NarfKeypoint::setRangeImage (const RangeImage* range_image)
{
  clearData ();
  range_image_border_extractor_->setRangeImage (range_image);
}

/////////////////////////////////////////////////////////////////////////
void
NarfKeypoint::calculateScaleSpace ()
{
  //MEASURE_FUNCTION_TIME;
  
  if (range_image_border_extractor_ == NULL || !range_image_border_extractor_->hasRangeImage () ||
      !border_extractor_scale_space_.empty ())  // Nothing to compute or already done
    return;
  border_extractor_scale_space_.push_back (range_image_border_extractor_);
  range_image_scale_space_.push_back (const_cast<RangeImage*> (reinterpret_cast<const RangeImage*> (&range_image_border_extractor_->getRangeImage ())));
  
  if (!parameters_.use_recursive_scale_reduction)
    return;
  
  while (0.5f*range_image_scale_space_.back ()->getAngularResolution () < deg2rad (2.0f))
  {
    range_image_scale_space_.push_back (getRangeImage ().getNew ());
    range_image_scale_space_[range_image_scale_space_.size ()-2]->getHalfImage (*range_image_scale_space_.back ());
    border_extractor_scale_space_.push_back (new RangeImageBorderExtractor);
    border_extractor_scale_space_.back ()->getParameters () = range_image_border_extractor_->getParameters ();
    border_extractor_scale_space_.back ()->setRangeImage (range_image_scale_space_.back ());
  }
}

#define USE_BEAM_AVERAGE 1

namespace 
{  // Some helper functions in an anonymous namespace - only available in this file
  inline void 
  nkdGetScores (float distance_factor, float surface_change_score, float pixelDistance,
                float optimal_distance, float& negative_score, float& positive_score)
  {
    negative_score = 1.0f - 0.5f * surface_change_score * (std::max) (1.0f - distance_factor/optimal_distance, 0.0f);
    negative_score = powf (negative_score, 2);
    
    if (pixelDistance < 2.0)
      positive_score = surface_change_score;
    else
      positive_score = surface_change_score * (1.0f-distance_factor);
  }
  
  inline float 
  nkdGetDirectionAngle (const Eigen::Vector3f& direction, const Eigen::Affine3f& rotation)
  {
    Eigen::Vector3f rotated_direction = rotation*direction;
    Eigen::Vector2f direction_vector (rotated_direction[0], rotated_direction[1]);
    direction_vector.normalize ();
    float angle = 0.5f*normAngle (2.0f*acosf (direction_vector[0]));
    return (angle);
  }
  
  inline void 
  propagateInvalidBeams (int new_radius, std::vector<bool>& old_beams, std::vector<bool>& new_beams)
  {
    new_beams.clear ();
    new_beams.resize (std::max (8*new_radius,1), false);
    if (new_radius >= 2)
    {
      float mapping_factor = 1.0f + (1.0f / static_cast<float> (new_radius-1));
      for (size_t old_idx=0; old_idx<old_beams.size (); ++old_idx)
      {
        if (old_beams[old_idx])
        {
          int middle_idx = static_cast<int> (pcl_lrint (mapping_factor * static_cast<float> (old_idx)));
          for (int idx_offset=-1; idx_offset<=1; ++idx_offset)
          {
            if (idx_offset != 0)
            {
              int old_neighbor_idx = static_cast<int> (old_idx) + idx_offset;
              if (old_neighbor_idx<0)
                old_neighbor_idx += static_cast<int> (old_beams.size ());
              if (old_neighbor_idx>= static_cast<int> (old_beams.size ()))
                old_neighbor_idx -= static_cast<int> (old_beams.size ());
              if (!old_beams[old_neighbor_idx])
                continue;
            }
            int new_idx = middle_idx+idx_offset;
            if (new_idx<0)
              new_idx += static_cast<int> (new_beams.size ());
            if (new_idx>= static_cast<int> (new_beams.size ()))
              new_idx -= static_cast<int> (new_beams.size ());
            new_beams[new_idx] = true;
          }
        }
      }
    }
  }
  
  inline bool
  isBetterInterestPoint (const InterestPoint& p1, const InterestPoint& p2)
  {
    return (p1.strength > p2.strength);
  }
  
  inline bool
  secondPairElementIsGreater (const std::pair<int, float>& p1, const std::pair<int, float>& p2)
  {
    return (p1.second > p2.second);
  }

}  // end anonymous namespace

void 
NarfKeypoint::calculateInterestImage () 
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
  if (interest_image_!=NULL)  // Already done
    return;
  
  if (parameters_.calculate_sparse_interest_image)
    calculateSparseInterestImage ();
  else
    calculateCompleteInterestImage ();
}

void 
NarfKeypoint::calculateCompleteInterestImage ()
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
  if (parameters_.support_size <= 0.0f)
  {
    std::cerr << __PRETTY_FUNCTION__<<": parameters_.support_size is not set!\n";
    return;
  }
  if (range_image_border_extractor_==NULL)
  {
    std::cerr << __PRETTY_FUNCTION__<<": range_image_border_extractor_ is not set!\n";
    return;
  }
  
  float search_radius = 0.5f * parameters_.support_size,
        radius_squared = search_radius*search_radius,
        radius_reciprocal = 1.0f / search_radius;
  
  calculateScaleSpace ();
  //std::cout << PVARN(range_image_scale_space_.size ());
  
  std::vector<float> start_usage_ranges;
  start_usage_ranges.resize (range_image_scale_space_.size ());
  start_usage_ranges[int (range_image_scale_space_.size ())-1] = 0.0f;
  for (int scale_idx = int (range_image_scale_space_.size ())-2;  scale_idx >= 0; --scale_idx)
  {
    start_usage_ranges[scale_idx] = parameters_.support_size / 
      tanf (static_cast<float> (parameters_.optimal_range_image_patch_size) * range_image_scale_space_[scale_idx+1]->getAngularResolution ());
    //std::cout << PVARN(start_usage_ranges[scale_idx]);
  }
  
  //double interest_value_calculation_start_time = getTime ();
  interest_image_scale_space_.clear ();
  interest_image_scale_space_.resize (range_image_scale_space_.size (), NULL);
  for (int scale_idx = int (range_image_scale_space_.size ())-1;  scale_idx >= 0; --scale_idx)
  {
    const RangeImage& range_image = *range_image_scale_space_[scale_idx];
    RangeImageBorderExtractor& border_extractor = *border_extractor_scale_space_[scale_idx];
    int original_max_no_of_threads = border_extractor.getParameters ().max_no_of_threads;
    border_extractor.getParameters ().max_no_of_threads = parameters_.max_no_of_threads;
    const ::pcl::PointCloud<BorderDescription>& border_descriptions = border_extractor.getBorderDescriptions ();
    const float* surface_change_scores = border_extractor.getSurfaceChangeScores ();
    const Eigen::Vector3f* surface_change_directions = border_extractor.getSurfaceChangeDirections ();
    float start_usage_range = start_usage_ranges[scale_idx];
    
    int width  = range_image.width,
        height = range_image.height,
        array_size = width*height;

    float* interest_image = new float[array_size];
    interest_image_scale_space_[scale_idx] = interest_image;
    //for (int i=0; i<array_size; ++i)
      //interest_image[i] = -1.0f;
    
    const int angle_histogram_size = 18;
    std::vector<float> angle_histogram;
    angle_histogram.resize(angle_histogram_size);
    
    std::vector<bool> was_touched;
    was_touched.resize (array_size, false);
    std::vector<int> neighbors_to_check;
    
#   pragma omp parallel for num_threads (parameters_.max_no_of_threads) default (shared) \
                            firstprivate (was_touched, neighbors_to_check, angle_histogram) schedule (dynamic, 10)
    for (int index=0; index<array_size; ++index)
    {
      float& interest_value = interest_image[index];
      interest_value = 0.0f;
      if (!range_image.isValid (index))
        continue;
      int y = index/range_image.width,
          x = index - y*range_image.width;
      
      const BorderTraits& border_traits = border_descriptions.points[index].traits;
      if (border_traits[BORDER_TRAIT__SHADOW_BORDER] || border_traits[BORDER_TRAIT__VEIL_POINT])
        continue;
      
      const PointWithRange& point = range_image.getPoint (index);
      
      if (point.range < start_usage_range)  // Point is close enough that we can use the value calculated at a lower resolution
      {
        const RangeImage& half_range_image = *range_image_scale_space_[scale_idx+1];
        float* half_interest_image = interest_image_scale_space_[scale_idx+1];
        int half_x = std::min (x/2, int (half_range_image.width)-1),
            half_y = std::min (y/2, int (half_range_image.height)-1);
        interest_value = half_interest_image[half_y*half_range_image.width + half_x];
        continue;
      }
      
      Eigen::Affine3f rotation_to_viewer_coordinate_system;
      range_image.getRotationToViewerCoordinateFrame (point.getVector3fMap (),
                                                      rotation_to_viewer_coordinate_system);
      float negative_score = 1.0f;
      
      // -----Start region growing to cover all connected points within the support size-----
      neighbors_to_check.clear ();
      neighbors_to_check.push_back (index);
      was_touched[index] = true;
      
      angle_histogram.clear ();
      angle_histogram.resize(angle_histogram_size, 0);
      for (size_t neighbors_to_check_idx=0; neighbors_to_check_idx<neighbors_to_check.size (); ++neighbors_to_check_idx)
      {
        int index2 = neighbors_to_check[neighbors_to_check_idx];
        if (!range_image.isValid (index2))
          continue;
        const BorderTraits& border_traits2 = border_descriptions.points[index2].traits;
        if (border_traits2[BORDER_TRAIT__SHADOW_BORDER] || border_traits2[BORDER_TRAIT__VEIL_POINT])
          continue;
        int y2 = index2/range_image.width,
            x2 = index2 - y2*range_image.width;
        const PointWithRange& point2 = range_image.getPoint (index2);
        
        float pixelDistance = static_cast<float> (std::max (abs (x2-x), abs (y2-y)));
        float distance_squared = squaredEuclideanDistance (point, point2);
        if (pixelDistance > 2.0f)  // Always consider immediate neighbors, even if to far away
        {
          if (distance_squared>radius_squared)
            continue;
        }
        
        for (int y3=std::max (0,y2-1); y3<=std::min (y2+1,int (range_image.height)-1); ++y3)
        {
          for (int x3=std::max (0,x2-1); x3<=std::min (x2+1,int (range_image.width)-1); ++x3)
          {
            int index3 = y3*range_image.width + x3;
            if (!was_touched[index3])
            {
              neighbors_to_check.push_back (index3);
              was_touched[index3] = true;
            }
          }
        }
        
        float surface_change_score = surface_change_scores[index2];
        if (surface_change_score < parameters_.min_surface_change_score)  // Pixel not 'interesting' enough to consider?
          continue;
        const Eigen::Vector3f& surface_change_direction = surface_change_directions[index2];
        
        float distance = sqrtf (distance_squared);
        float distance_factor = radius_reciprocal*distance;
        float positive_score, current_negative_score;
        nkdGetScores (distance_factor, surface_change_score, pixelDistance,
                     parameters_.optimal_distance_to_high_surface_change,
                     current_negative_score, positive_score);
        float angle = nkdGetDirectionAngle (surface_change_direction, rotation_to_viewer_coordinate_system);
        int histogram_cell = (std::min) (angle_histogram_size-1,
                          static_cast<int> (pcl_lrint (floorf ( (angle+deg2rad (90.0f))/deg2rad (180.0f) * angle_histogram_size))));
        float& histogram_value = angle_histogram[histogram_cell];
        
        histogram_value = (std::max) (histogram_value, positive_score);
        negative_score  = (std::min) (negative_score, current_negative_score);
      }
      
      // Reset was_touched to false
      for (size_t neighbors_to_check_idx=0; neighbors_to_check_idx<neighbors_to_check.size (); ++neighbors_to_check_idx)
        was_touched[neighbors_to_check[neighbors_to_check_idx]] = false;
      
      float angle_change_value = 0.0f;
      for (int histogram_cell1=0; histogram_cell1<angle_histogram_size-1; ++histogram_cell1)
      {
        if (angle_histogram[histogram_cell1]==0.0f)
          continue;
        for (int histogram_cell2=histogram_cell1+1; histogram_cell2<angle_histogram_size; ++histogram_cell2)
        {
          if (angle_histogram[histogram_cell2]==0.0f)
            continue;
          // TODO: lookup table for the following:
          float normalized_angle_diff = 2.0f*float (histogram_cell2-histogram_cell1)/float (angle_histogram_size);
          normalized_angle_diff = (normalized_angle_diff <= 1.0f ? normalized_angle_diff : 2.0f-normalized_angle_diff);
          
          angle_change_value = std::max (angle_histogram[histogram_cell1] * angle_histogram[histogram_cell2] *
                                         normalized_angle_diff,   angle_change_value);
        }
      }
      angle_change_value = sqrtf (angle_change_value);
      interest_value = negative_score * angle_change_value;
      
      if (parameters_.add_points_on_straight_edges)
      {
        float max_histogram_cell_value = 0.0f;
        for (int histogram_cell=0; histogram_cell<angle_histogram_size; ++histogram_cell)
          max_histogram_cell_value = (std::max) (max_histogram_cell_value, angle_histogram[histogram_cell]);
        //std::cout << PVARN(max_histogram_cell_value);
        interest_value = 0.5f*(interest_value+max_histogram_cell_value);
        //std::cout << PVARN(interest_value);
      }
    }
    
    border_extractor.getParameters ().max_no_of_threads = original_max_no_of_threads;
  }
  
  if (interest_image_scale_space_.empty ())
    interest_image_ = NULL;
  else
    interest_image_ = interest_image_scale_space_[0];
}

void 
NarfKeypoint::calculateSparseInterestImage ()
{
  if (parameters_.support_size <= 0.0f)
  {
    std::cerr << __PRETTY_FUNCTION__<<": parameters_.support_size is not set!\n";
    return;
  }
  if (range_image_border_extractor_==NULL)
  {
    std::cerr << __PRETTY_FUNCTION__<<": range_image_border_extractor_ is not set!\n";
    return;
  }

  float search_radius = 0.5f * parameters_.support_size,
        radius_reciprocal = 1.0f / search_radius,
        increased_radius = 1.5f * search_radius,
        increased_radius_squared = increased_radius*increased_radius,
        radius_overhead = increased_radius-search_radius,
        radius_overhead_squared = radius_overhead*radius_overhead;
  
  const RangeImage& range_image = range_image_border_extractor_->getRangeImage ();
  RangeImageBorderExtractor& border_extractor = *range_image_border_extractor_;
  int original_max_no_of_threads = border_extractor.getParameters ().max_no_of_threads;
  border_extractor.getParameters ().max_no_of_threads = parameters_.max_no_of_threads;
  const ::pcl::PointCloud<BorderDescription>& border_descriptions = border_extractor.getBorderDescriptions ();
  const float* surface_change_scores = border_extractor.getSurfaceChangeScores ();
  const Eigen::Vector3f* surface_change_directions = border_extractor.getSurfaceChangeDirections ();
  
  int width  = range_image.width,
      height = range_image.height,
      array_size = width*height;
  
  interest_image_ = new float[array_size];
  
  for (int index=0; index<array_size; ++index)
  {
    interest_image_[index] = 0.0f;
    if (!range_image.isValid (index))
      continue;
    const BorderTraits& border_traits = border_descriptions.points[index].traits;
    if (border_traits[BORDER_TRAIT__SHADOW_BORDER] || border_traits[BORDER_TRAIT__VEIL_POINT])
      continue;
    interest_image_[index] = 2.0f;
  }
  
  const int angle_histogram_size = 18;
  std::vector<float> angle_histogram;
  angle_histogram.resize(angle_histogram_size);
  std::vector<std::vector<std::pair<int, float> > > angle_elements (angle_histogram_size);
  std::vector<bool> relevant_point_still_valid;
  
  std::vector<bool> was_touched;
  was_touched.resize (array_size, false);
  std::vector<int> neighbors_to_check,
                   neighbors_within_radius_overhead;
  
  //double interest_value_calculation_start_time = getTime ();
# pragma omp parallel for default (shared) num_threads (parameters_.max_no_of_threads) schedule (guided, 10) \
                          firstprivate (was_touched, neighbors_to_check, angle_histogram, neighbors_within_radius_overhead, \
                                        angle_elements, relevant_point_still_valid) 
  for (int index=0; index<array_size; ++index)
  {
    if (interest_image_[index] <= 1.0f)
      continue;
    int y = index/range_image.width,
        x = index - y*range_image.width;
    
    const PointWithRange& point = range_image.getPoint (index);
    
    Eigen::Affine3f rotation_to_viewer_coordinate_system;
    range_image.getRotationToViewerCoordinateFrame (point.getVector3fMap (),
                                                    rotation_to_viewer_coordinate_system);
    
    // -----Start region growing to cover all connected points within the support size-----
    neighbors_to_check.clear ();
    neighbors_to_check.push_back (index);
    neighbors_within_radius_overhead.clear ();
    was_touched[index] = true;
    
    for (int angle_histogram_idx=0; angle_histogram_idx<angle_histogram_size; ++angle_histogram_idx)
    {
      angle_histogram[angle_histogram_idx] = 0;
      angle_elements[angle_histogram_idx].clear ();
    }
    
    for (size_t neighbors_to_check_idx=0; neighbors_to_check_idx<neighbors_to_check.size (); ++neighbors_to_check_idx)
    {
      int index2 = neighbors_to_check[neighbors_to_check_idx];
      if (!range_image.isValid (index2))
        continue;
      const BorderTraits& border_traits2 = border_descriptions.points[index2].traits;
      if (border_traits2[BORDER_TRAIT__SHADOW_BORDER] || border_traits2[BORDER_TRAIT__VEIL_POINT])
        continue;
      int y2 = index2/range_image.width,
          x2 = index2 - y2*range_image.width;
      const PointWithRange& point2 = range_image.getPoint (index2);
      
      float pixelDistance = static_cast<float> (std::max (abs (x2-x), abs (y2-y)));

      float distance_squared = squaredEuclideanDistance (point, point2);
      if (distance_squared <= radius_overhead_squared) 
        neighbors_within_radius_overhead.push_back (index2);

      if (pixelDistance > 2.0f)  // Always consider immediate neighbors, even if to far away
      {
        if (distance_squared>increased_radius_squared)
          continue;
      }
      
      for (int y3=std::max (0,y2-1); y3<=std::min (y2+1,int (range_image.height)-1); ++y3)
      {
        for (int x3=std::max (0,x2-1); x3<=std::min (x2+1,int (range_image.width)-1); ++x3)
        {
          int index3 = y3*range_image.width + x3;
          if (!was_touched[index3])
          {
            neighbors_to_check.push_back (index3);
            was_touched[index3] = true;
          }
        }
      }
      
      float surface_change_score = surface_change_scores[index2];
      if (surface_change_score < parameters_.min_surface_change_score)  // Pixel not 'interesting' enough to consider?
        continue;
      const Eigen::Vector3f& surface_change_direction = surface_change_directions[index2];
      
      float angle = nkdGetDirectionAngle (surface_change_direction, rotation_to_viewer_coordinate_system);
      int histogram_cell = (std::min) (angle_histogram_size-1,
                                       static_cast<int> (pcl_lrint (floorf ( (angle+deg2rad (90.0f))/deg2rad (180.0f) * angle_histogram_size))));
      float& histogram_value = angle_histogram[histogram_cell];
      histogram_value = (std::max) (histogram_value, surface_change_score);
      angle_elements[histogram_cell].push_back (std::make_pair(index2, surface_change_score));
    }
    
    // Reset was_touched to false
    for (size_t neighbors_to_check_idx=0; neighbors_to_check_idx<neighbors_to_check.size (); ++neighbors_to_check_idx)
      was_touched[neighbors_to_check[neighbors_to_check_idx]] = false;
    
    float angle_change_value = 0.0f;
    for (int histogram_cell1=0; histogram_cell1<angle_histogram_size-1; ++histogram_cell1)
    {
      if (angle_histogram[histogram_cell1]==0.0f)
        continue;
      for (int histogram_cell2=histogram_cell1+1; histogram_cell2<angle_histogram_size; ++histogram_cell2)
      {
        if (angle_histogram[histogram_cell2]==0.0f)
          continue;
        // TODO: lookup table for the following:
        float normalized_angle_diff = 2.0f*float (histogram_cell2-histogram_cell1)/float (angle_histogram_size);
        normalized_angle_diff = (normalized_angle_diff <= 1.0f ? normalized_angle_diff : 2.0f-normalized_angle_diff);
        
        angle_change_value = std::max (angle_histogram[histogram_cell1] * angle_histogram[histogram_cell2] *
                                       normalized_angle_diff,   angle_change_value);
      }
    }
    angle_change_value = sqrtf (angle_change_value);
    float maximum_interest_value = angle_change_value;
    
    if (parameters_.add_points_on_straight_edges)
    {
      float max_histogram_cell_value = 0.0f;
      for (int histogram_cell=0; histogram_cell<angle_histogram_size; ++histogram_cell)
        max_histogram_cell_value = (std::max) (max_histogram_cell_value, angle_histogram[histogram_cell]);
      maximum_interest_value = 0.5f * (maximum_interest_value+max_histogram_cell_value);
    }
    
    // Every point in distance search_radius cannot have a higher value
    // Therefore: if too low, set all to zero. Else calculate properly
    if (maximum_interest_value < parameters_.min_interest_value)
      for (size_t neighbors_idx=0; neighbors_idx<neighbors_within_radius_overhead.size (); ++neighbors_idx)
        interest_image_[neighbors_within_radius_overhead[neighbors_idx]] = 0.0f;
    else
    {
      // Reduce number of neighbors to go through by filtering close by points with the same angle
      bool do_neighbor_size_reduction = true;
      if (do_neighbor_size_reduction)
      {
        float min_distance_between_relevant_points = 0.25f * search_radius,
              min_distance_between_relevant_points_squared = powf(min_distance_between_relevant_points, 2);
        for (int angle_histogram_idx=0; angle_histogram_idx<angle_histogram_size; ++angle_histogram_idx)
        {
          std::vector<std::pair<int,float> >& relevent_point_indices = angle_elements[angle_histogram_idx];
          std::sort(relevent_point_indices.begin(), relevent_point_indices.end(), secondPairElementIsGreater);
          relevant_point_still_valid.clear();
          relevant_point_still_valid.resize(relevent_point_indices.size(), true);
          for (int rpi_idx1=0; rpi_idx1<int(relevent_point_indices.size ())-1; ++rpi_idx1)
          {
            if (!relevant_point_still_valid[rpi_idx1])
              continue;
            const PointWithRange& relevant_point1 = range_image.getPoint (relevent_point_indices[rpi_idx1].first);
            for (int rpi_idx2=rpi_idx1+1; rpi_idx2<int(relevent_point_indices.size ()); ++rpi_idx2)
            {
              if (!relevant_point_still_valid[rpi_idx2])
                continue;
              const PointWithRange& relevant_point2 = range_image.getPoint (relevent_point_indices[rpi_idx2].first);
              float distance_squared = (relevant_point1.getVector3fMap ()-relevant_point2.getVector3fMap ()).norm ();
              if (distance_squared > min_distance_between_relevant_points_squared)
                continue;
              relevant_point_still_valid[rpi_idx2] = false;
            }
          }
          int newPointIdx=0;
          for (int oldPointIdx=0; oldPointIdx<int(relevant_point_still_valid.size()); ++oldPointIdx) {
            if (relevant_point_still_valid[oldPointIdx])
              relevent_point_indices[newPointIdx++] = relevent_point_indices[oldPointIdx];
          }
          relevent_point_indices.resize(newPointIdx);
        }
      }
      
      // Caclulate interest values for neighbors
      for (size_t neighbors_idx=0; neighbors_idx<neighbors_within_radius_overhead.size (); ++neighbors_idx)
      {
        int index2 = neighbors_within_radius_overhead[neighbors_idx];
        int y2 = index2/range_image.width,
            x2 = index2 - y2*range_image.width;
        const PointWithRange& point2 = range_image.getPoint (index2);
        float& interest_value = interest_image_[index2];
        if (interest_value <= 1.0f)
          continue;
        float negative_score = 1.0;

        for (int angle_histogram_idx=0; angle_histogram_idx<angle_histogram_size; ++angle_histogram_idx)
        {
          float& histogram_value = angle_histogram[angle_histogram_idx];
          histogram_value = 0;
          const std::vector<std::pair<int,float> >& relevent_point_indices = angle_elements[angle_histogram_idx];
          for (size_t rpi_idx=0; rpi_idx<relevent_point_indices.size (); ++rpi_idx)
          {
            int index3 = relevent_point_indices[rpi_idx].first;
            int y3 = index3/range_image.width,
                x3 = index3 - y3*range_image.width;
            const PointWithRange& point3 = range_image.getPoint (index3);
            float surface_change_score = relevent_point_indices[rpi_idx].second;
            
            float pixelDistance = static_cast<float> (std::max (abs (x3-x2), abs (y3-y2)));
            float distance = (point3.getVector3fMap ()-point2.getVector3fMap ()).norm ();
            float distance_factor = radius_reciprocal*distance;
            float positive_score, current_negative_score;
            nkdGetScores (distance_factor, surface_change_score, pixelDistance,
                         parameters_.optimal_distance_to_high_surface_change,
                         current_negative_score, positive_score);
            histogram_value = (std::max) (histogram_value, positive_score);
            negative_score  = (std::min) (negative_score, current_negative_score);
          }
        }
        float angle_change_value = 0.0f;
        for (int histogram_cell1=0; histogram_cell1<angle_histogram_size-1; ++histogram_cell1)
        {
          if (angle_histogram[histogram_cell1]==0.0f)
            continue;
          for (int histogram_cell2=histogram_cell1+1; histogram_cell2<angle_histogram_size; ++histogram_cell2)
          {
            if (angle_histogram[histogram_cell2]==0.0f)
              continue;
            // TODO: lookup table for the following:
            float normalized_angle_diff = 2.0f*float (histogram_cell2-histogram_cell1)/float (angle_histogram_size);
            normalized_angle_diff = (normalized_angle_diff <= 1.0f ? normalized_angle_diff : 2.0f-normalized_angle_diff);
            angle_change_value = std::max (angle_change_value, angle_histogram[histogram_cell1] *
                                                               angle_histogram[histogram_cell2] *
                                                               normalized_angle_diff);
          }
        }
        angle_change_value = sqrtf (angle_change_value);
        interest_value = negative_score * angle_change_value;
        if (parameters_.add_points_on_straight_edges)
        {
          float max_histogram_cell_value = 0.0f;
          for (int histogram_cell=0; histogram_cell<angle_histogram_size; ++histogram_cell)
            max_histogram_cell_value = (std::max) (max_histogram_cell_value, angle_histogram[histogram_cell]);
          //std::cout << PVARN(max_histogram_cell_value);
          interest_value = 0.5f*(interest_value+max_histogram_cell_value);
          //std::cout << PVARN(interest_value);
        }
      }
    }
  }
  
  border_extractor.getParameters ().max_no_of_threads = original_max_no_of_threads;
}

void 
NarfKeypoint::calculateInterestPoints ()
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";

  if (interest_points_ != NULL)
    return;

  calculateInterestImage ();
  
  interest_points_ = new ::pcl::PointCloud<InterestPoint>;
  
  float max_distance_squared = powf (0.3f*parameters_.support_size, 2);
  
  const RangeImage& range_image = range_image_border_extractor_->getRangeImage ();
  const ::pcl::PointCloud<BorderDescription>& border_descriptions =
      range_image_border_extractor_->getBorderDescriptions ();
  int width  = range_image.width,
      height = range_image.height,
      size = width*height;
  is_interest_point_image_.clear ();
  is_interest_point_image_.resize (size, false);
  
  typedef double RealForPolynomial;
  PolynomialCalculationsT<RealForPolynomial> polynomial_calculations;
  BivariatePolynomialT<RealForPolynomial> polynomial (2);
  std::vector<Eigen::Matrix<RealForPolynomial, 3, 1> > sample_points;
  std::vector<RealForPolynomial> x_values, y_values;
  std::vector<int> types;
  std::vector<bool> invalid_beams, old_invalid_beams;
  
  pcl::PointCloud<InterestPoint>::VectorType tmp_interest_points;
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float interest_value = interest_image_[index];
      if (!range_image.isValid (index) || interest_value < parameters_.min_interest_value)
        continue;
      const PointWithRange& point = range_image.getPoint (index);
      bool is_maximum = true;
      for (int y2=y-1; y2<=y+1&&is_maximum&&parameters_.do_non_maximum_suppression; ++y2)
      {
        for (int x2=x-1; x2<=x+1; ++x2)
        {
          if (!range_image.isInImage (x2,y2))
            continue;
          int index2 = y2*width + x2;
          float interest_value2 = interest_image_[index2];
          if (interest_value2 <= interest_value)
            continue;
          is_maximum = false;
          break;
        }
      }
      if (!is_maximum)
        continue;
      
      PointWithRange keypoint_3d = point;
      int keypoint_x_int=x, keypoint_y_int=y;
      
      int no_of_polynomial_approximations_per_point = parameters_.no_of_polynomial_approximations_per_point;
      if (!parameters_.do_non_maximum_suppression)
        no_of_polynomial_approximations_per_point = 0;
      
      for (int poly_step=0; poly_step<no_of_polynomial_approximations_per_point; ++poly_step)
      {
        sample_points.clear ();
        invalid_beams.clear ();
        old_invalid_beams.clear ();
        for (int radius=0, stop=false;  !stop;  ++radius) 
        {
          std::swap (invalid_beams, old_invalid_beams);
          propagateInvalidBeams (radius, old_invalid_beams, invalid_beams);
          int x2=keypoint_x_int-radius-1, y2=keypoint_y_int-radius;  // Top left - 1
          stop = true;
          for (int i=0; (radius==0&&i==0) || i<8*radius; ++i)
          {
            if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
            if (invalid_beams[i] || !range_image.isValid (x2, y2))
              continue;
            int index2 = y2*width + x2;
            const BorderTraits& neighbor_border_traits = border_descriptions.points[index2].traits;
            if (neighbor_border_traits[BORDER_TRAIT__SHADOW_BORDER] || neighbor_border_traits[BORDER_TRAIT__VEIL_POINT])
            {
              invalid_beams[i] = true;
              continue;
            }
            const PointWithRange& neighbor = range_image.getPoint (index2);
            float distance_squared = squaredEuclideanDistance (point, neighbor);
            if (distance_squared>max_distance_squared)
            {
              invalid_beams[i] = true;
              continue;
            }
            stop = false; // There is a point in range -> Have to check further distances
            
            float interest_value2 = interest_image_[index2];
            sample_points.push_back (Eigen::Vector3d (x2-keypoint_x_int, y2-keypoint_y_int, interest_value2));
          }
        }
        if (!polynomial_calculations.bivariatePolynomialApproximation (sample_points, 2, polynomial))
          continue;

        polynomial.findCriticalPoints (x_values, y_values, types);
        
        if (!types.empty () && types[0]==0)
        {
          float keypoint_x = static_cast<float> (x_values[0]+keypoint_x_int),
                keypoint_y = static_cast<float> (y_values[0]+keypoint_y_int);
          
          keypoint_x_int = static_cast<int> (pcl_lrint (keypoint_x));
          keypoint_y_int = static_cast<int> (pcl_lrint (keypoint_y));
          
          range_image.calculate3DPoint (keypoint_x, keypoint_y, keypoint_3d);
          if (!pcl_isfinite (keypoint_3d.range))
          {
            keypoint_3d = point;
            break;
          }
        }
        else
        {
          break;
        }
      }
      
      InterestPoint interest_point;
      interest_point.getVector3fMap () = keypoint_3d.getVector3fMap ();
      interest_point.strength = interest_value;
      tmp_interest_points.push_back (interest_point);
    }
  }
  
  std::sort (tmp_interest_points.begin (), tmp_interest_points.end (), isBetterInterestPoint);
  
  float min_distance_squared = powf (parameters_.min_distance_between_interest_points*parameters_.support_size, 2);
  for (size_t int_point_idx=0; int_point_idx<tmp_interest_points.size (); ++int_point_idx)
  {
    if (parameters_.max_no_of_interest_points > 0  &&  int (interest_points_->size ()) >= parameters_.max_no_of_interest_points)
      break;
    const InterestPoint& interest_point = tmp_interest_points[int_point_idx];
    
    bool better_point_too_close = false;
    for (size_t int_point_idx2=0; int_point_idx2<interest_points_->points.size (); ++int_point_idx2)
    {
      const InterestPoint& interest_point2 = interest_points_->points[int_point_idx2];
      float distance_squared = (interest_point.getVector3fMap ()-interest_point2.getVector3fMap ()).squaredNorm ();
      if (distance_squared < min_distance_squared)
      {
        better_point_too_close = true;
        break;
      }
    }
    if (better_point_too_close)
      continue;
    interest_points_->points.push_back (interest_point);
    int image_x, image_y;
    //std::cout << interest_point.x<<","<<interest_point.y<<","<<interest_point.z<<", "<<std::flush;
    range_image.getImagePoint (interest_point.getVector3fMap (), image_x, image_y);
    if (range_image.isValid (image_x, image_y))
      is_interest_point_image_[image_y*width + image_x] = true;
  }
  interest_points_->width = static_cast<uint32_t> (interest_points_->points.size ());
  interest_points_->height = 1;
  interest_points_->is_dense = true;
}

const RangeImage& 
NarfKeypoint::getRangeImage ()
{
  return (range_image_border_extractor_->getRangeImage ());
}

void 
NarfKeypoint::detectKeypoints (NarfKeypoint::PointCloudOut& output)
{
  output.points.clear ();
  
  if (indices_)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": Sorry, usage of indices for the extraction is not supported for NARF interest points (yet).\n\n";
    return;
  }
  
  if (range_image_border_extractor_ == NULL)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImageBorderExtractor member is not set. "
              <<   "Sorry, this is needed for the NARF keypoint extraction.\n\n";
    return;
  }
  
  if (!range_image_border_extractor_->hasRangeImage ())
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImage is not set. Sorry, the NARF keypoint extraction works on range images, "
                   "not on normal point clouds.\n\n"
              << " Use setRangeImage (...).\n\n";
    return;
  }
  
  calculateInterestPoints ();
  
  int size = getRangeImage ().width * getRangeImage ().height;
  
  for (int index=0; index<size; ++index)
  {
    if (!is_interest_point_image_[index])
      continue;
    output.points.push_back (index);
  }
}

void 
NarfKeypoint::compute (NarfKeypoint::PointCloudOut& output)
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  detectKeypoints (output);
}

}  // end namespace pcl

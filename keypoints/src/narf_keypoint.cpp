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

/* \author Bastian Steder */

#include <iostream>
using std::cout;
using std::cerr;
#include <vector>
using std::vector;
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/polynomial_calculations.h>
#include <pcl/range_image/range_image.h>
//#include <pcl/common/vector_average.h>


#define USE_OMP 1

namespace pcl {


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
  clearData ();
}

/////////////////////////////////////////////////////////////////////////
void
  NarfKeypoint::clearData ()
{
  //cerr << __PRETTY_FUNCTION__<<" called.\n";
  
  for (size_t scale_space_idx = 1; scale_space_idx<border_extractor_scale_space_.size (); ++scale_space_idx)
    delete border_extractor_scale_space_[scale_space_idx];
  border_extractor_scale_space_.clear ();
  for (size_t scale_space_idx = 1; scale_space_idx<range_image_scale_space_.size (); ++scale_space_idx)
    delete range_image_scale_space_[scale_space_idx];
  range_image_scale_space_.clear ();
  for (size_t scale_space_idx = 1; scale_space_idx<interest_image_scale_space_.size (); ++scale_space_idx)
    delete interest_image_scale_space_[scale_space_idx];
  interest_image_scale_space_.clear();
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
  range_image_scale_space_.push_back ((RangeImage*)&range_image_border_extractor_->getRangeImage ());
  while ((int)range_image_scale_space_.back ()->width > parameters_.optimal_range_image_patch_size &&
         (int)range_image_scale_space_.back ()->height > parameters_.optimal_range_image_patch_size)
  {
    range_image_scale_space_.push_back (getRangeImage().getNew());
    range_image_scale_space_[range_image_scale_space_.size ()-2]->getHalfImage (*range_image_scale_space_.back ());
    border_extractor_scale_space_.push_back (new RangeImageBorderExtractor);
    border_extractor_scale_space_.back ()->getParameters () = range_image_border_extractor_->getParameters ();
    border_extractor_scale_space_.back ()->setRangeImage (range_image_scale_space_.back ());
    border_extractor_scale_space_.back ()->getSurfaceChangeScores();
  }
  //std::cout << PVARN(range_image_scale_space_.size());
}

#define USE_BEAM_AVERAGE 1

namespace {  // Some helper functions in an anonymous namespace - only available in this file
  inline void nkdGetScores (float distance_factor, int pixel_distance, float surface_change_score,
                           float optimal_distance, float& negative_score, float& positive_score)
  {
    //float negative_score_regarding_pixel_distance = 0.0f;//min (0.0f, 0.5f*pixel_distance-1.0f);
    negative_score = 1.0f - surface_change_score * (std::max) (1.0f - distance_factor/optimal_distance, 0.0f);
    negative_score = powf (negative_score, 2);
    //cout << PVARC (surface_change_score)<<PVARC (distance_factor)<<PVARC (optimal_distance)<<PVARN (negative_score);
    
    //if (negative_score < 1.0f)
      //cout << PVARC (surface_change_score)<<PVARC (distance_factor)<<PVARN (negative_score);
    positive_score = surface_change_score * (1.0f-fabsf (distance_factor-optimal_distance));
    //positive_score = surface_change_score;
  }
  void translateDirection180To360 (Eigen::Vector2f& direction_vector)
  {
    // The following code does the equivalent to this:
    // Get the angle of the 2D direction (with positive x) alpha, and return the direction of 2*alpha
    // We do this to create a normal angular wrap-around at -180,180 instead of the one at -90,90, 
    // enabling us to calculate the average angle as the direction of the sum of all unit vectors.
    // We use sin (2a)=2*sin (a)*cos (a) and cos (2a)=2cos^2 (a)-1 so that we needn't actually compute the angles,
    // which would be expensive
    float cos_a = direction_vector[0],
          cos_2a = 2*cos_a*cos_a - 1.0f,
          sin_a = direction_vector[1],
          sin_2a = 2.0f*sin_a*cos_a;
    direction_vector[0] = cos_2a;
    direction_vector[1] = sin_2a;
  }
  void translateDirection360To180 (Eigen::Vector2f& direction_vector)
  {
    // Inverse of the above
    float cos_2a = direction_vector[0],
          cos_a = sqrtf (0.5f* (cos_2a+1.0f)),
          sin_2a = direction_vector[1],
          sin_a = sin_2a / (2.0f*cos_a);
    direction_vector[0] = cos_a;
    direction_vector[1] = sin_a;
  }
  inline Eigen::Vector2f nkdGetDirectionVector (const Eigen::Vector3f& direction, const Eigen::Affine3f& rotation)
  {
    //if (fabsf (direction.norm ()-1.0f) > 0.001)
      //cerr << direction[0]<<","<<direction[1]<<","<<direction[2]<<" has norm "<<direction.norm ()<<"\n";
    //else
      //cerr<<"OK";
    Eigen::Vector3f rotated_direction = rotation*direction;
    Eigen::Vector2f direction_vector (rotated_direction[0], rotated_direction[1]);
    direction_vector.normalize ();
    if (direction_vector[0]<0.0f)
      direction_vector *= -1.0f;
    

#   if USE_BEAM_AVERAGE
      translateDirection180To360 (direction_vector);
#   endif
    ////cout << PVARN (direction_vector);
    
    return direction_vector;
  }
  inline float nkdGetDirectionAngle (const Eigen::Vector3f& direction, const Eigen::Affine3f& rotation)
  {
    Eigen::Vector3f rotated_direction = rotation*direction;
    Eigen::Vector2f direction_vector (rotated_direction[0], rotated_direction[1]);
    direction_vector.normalize ();
    float angle = 0.5f*normAngle (2.0f*acosf (direction_vector[0]));
    //std::cout << PVARN (direction_vector)<<PVARAN (angle);
    return angle;
  }
  
  inline void propagateInvalidBeams (int new_radius, std::vector<bool>& old_beams, std::vector<bool>& new_beams)
  {
    new_beams.clear ();
    new_beams.resize (std::max (8*new_radius,1), false);
    if (new_radius >= 2)
    {
      float mapping_factor = 1.0f+ (1.0f/ (new_radius-1));
      for (size_t old_idx=0; old_idx<old_beams.size (); ++old_idx)
      {
        if (old_beams[old_idx])
        {
          int middle_idx = pcl_lrint (mapping_factor*old_idx);
          //cout << "Radius "<<new_radius-1<<", beam "<<old_idx<<" is invalid =>"<<PVAR (middle_idx)<<"\n";
          for (int idx_offset=-1; idx_offset<=1; ++idx_offset)
          {
            if (idx_offset != 0)
            {
              int old_neighbor_idx = old_idx+idx_offset;
              if (old_neighbor_idx<0)
                old_neighbor_idx += old_beams.size ();
              if (old_neighbor_idx>= (int)old_beams.size ())
                old_neighbor_idx -= old_beams.size ();
              if (!old_beams[old_neighbor_idx])
                continue;
            }
            int new_idx = middle_idx+idx_offset;
            if (new_idx<0)
              new_idx += new_beams.size ();
            if (new_idx>= (int)new_beams.size ())
              new_idx -= new_beams.size ();
            new_beams[new_idx] = true;
            //cout << "  beam "<<new_idx<<" is invalid =>\n";
          }
        }
      }
      //cout << "\n";
    }
    //for (size_t i=0; i<new_beams.size (); ++i)
      //cout << (int)new_beams[i]<<" ";
    //cout << "\n";
  }
  
  inline bool
    isBetterInterestPoint (const InterestPoint& p1, const InterestPoint& p2)
  {
    return p1.strength > p2.strength;
  }

}  // end anonymous namespace

void NarfKeypoint::calculateInterestImage ()
{
  //MEASURE_FUNCTION_TIME;

  if (interest_image_!=NULL)  // Already done
    return;
  
  if (parameters_.support_size <= 0.0f)
  {
    cerr << __PRETTY_FUNCTION__<<": parameters_.support_size is not set!\n";
    return;
  }
  if (range_image_border_extractor_==NULL)
  {
    cerr << __PRETTY_FUNCTION__<<": range_image_border_extractor_ is not set!\n";
    return;
  }
  
  float search_radius = 0.5*parameters_.support_size,
        radius_squared = search_radius*search_radius,
        radius_reciprocal = 1.0f/search_radius;
  
  calculateScaleSpace ();
  
  std::vector<float> start_usage_ranges;
  start_usage_ranges.resize (range_image_scale_space_.size());
  start_usage_ranges[int(range_image_scale_space_.size())-1] = 0.0f;
  for (int scale_idx = int(range_image_scale_space_.size())-2;  scale_idx >= 0; --scale_idx)
    start_usage_ranges[scale_idx] = parameters_.support_size / tanf(parameters_.optimal_range_image_patch_size *
                                                  range_image_scale_space_[scale_idx+1]->getAngularResolution());
  interest_image_scale_space_.clear();
  interest_image_scale_space_.resize(range_image_scale_space_.size(), NULL);
  for (int scale_idx = int(range_image_scale_space_.size())-1;  scale_idx >= 0; --scale_idx)
  {
    const RangeImage& range_image = *range_image_scale_space_[scale_idx];
    RangeImageBorderExtractor& border_extractor = *border_extractor_scale_space_[scale_idx];
    const ::pcl::PointCloud<BorderDescription>& border_descriptions = border_extractor.getBorderDescriptions ();
    const float* surface_change_scores = border_extractor.getSurfaceChangeScores ();
    const Eigen::Vector3f* surface_change_directions = border_extractor.getSurfaceChangeDirections ();
    float start_usage_range = start_usage_ranges[scale_idx];
    //cout << PVARC(scale_idx) << PVARN(start_usage_range);
    
    int width  = range_image.width,
        height = range_image.height,
        array_size = width*height;

    float* interest_image = new float[array_size];
    interest_image_scale_space_[scale_idx] = interest_image;
    for (int i=0; i<array_size; ++i)
      interest_image[i] = 1.0f;
    
    int angle_histogram_size = 18;
    float* angle_histograms = new float[angle_histogram_size * array_size];
    memset (angle_histograms, 0, angle_histogram_size*array_size*sizeof (*angle_histograms));
    
    vector<bool> invalid_beams, old_invalid_beams;
    
    //double histogram_time = -getTime ();
#   pragma omp parallel for num_threads(parameters_.max_no_of_threads) default(shared) private(invalid_beams, old_invalid_beams) schedule(dynamic, 1)
    for (int y=0; y<height; ++y)
    {
      for (int x=0; x<width; ++x)
      {
        //cout << "\n"<<x<<","<<y<<"\n\n";
        int index = y*width + x;
        if (!range_image.isValid (x, y))
        {
          interest_image[index] = 0.0f;
          continue;
        }
        
        const PointWithRange& point = range_image.getPoint (index);
        
        if (point.range+search_radius < start_usage_range)  // Point and all neighbors are evaluated at a higher scale -> skip
          continue;
        
        const BorderTraits& border_traits = border_descriptions.points[index].traits;
        if (border_traits[BORDER_TRAIT__SHADOW_BORDER] || border_traits[BORDER_TRAIT__VEIL_POINT])
          continue;
        
        float surface_change_score = surface_change_scores[index];
        if (surface_change_score < parameters_.min_surface_change_score)  // Pixel not 'interesting' enough to consider?
          continue;
        
        const Eigen::Vector3f& surface_change_direction = surface_change_directions[index];
        
        invalid_beams.clear ();
        old_invalid_beams.clear ();
        for (int radius=0, stop=false;  !stop;  ++radius) 
        {
          std::swap (invalid_beams, old_invalid_beams);
          propagateInvalidBeams (radius, old_invalid_beams, invalid_beams);
          //if (x==91 && y==88)
          //{
            //for (size_t i=0; i<invalid_beams.size (); ++i)
              //cout << (int)invalid_beams[i]<<" ";
            //cout << "\n";
          //}
          
          int x2=x-radius-1, y2=y-radius;  // Top left - 1
          stop = true;
          for (int i=0; (radius==0&&i==0) || i<8*radius; ++i)
          {
            if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
            if (invalid_beams[i] || !range_image.isValid (x2, y2))
              continue;
            int neighbor_index = y2*width+x2;
            
            const PointWithRange& neighbor = range_image.getPoint (neighbor_index);
            
            if (neighbor.range < start_usage_range)  // Point is evaluated at a higher scale -> skip
              continue;
            
            const BorderTraits& neighbor_border_traits = border_descriptions.points[neighbor_index].traits;
            if (neighbor_border_traits[BORDER_TRAIT__SHADOW_BORDER] || neighbor_border_traits[BORDER_TRAIT__VEIL_POINT])
            {
              invalid_beams[i] = true;
              continue;
            }
            
            float distance_squared = squaredEuclideanDistance (point, neighbor);
            if (distance_squared>radius_squared)
            {
              invalid_beams[i] = true;
              continue;
            }
            stop = false; // There is a point in range -> Have to check further distances
            
            float* angle_histogram = angle_histograms+angle_histogram_size*neighbor_index;
            float distance_factor = radius_reciprocal*sqrtf (distance_squared);
            float positive_score, current_negative_score;
            nkdGetScores (distance_factor, radius, surface_change_score,
                         parameters_.optimal_distance_to_high_surface_change,
                         current_negative_score, positive_score);
            // TODO: lookup table for the following:
            Eigen::Affine3f rotation_to_viewer_coordinate_system;
            range_image.getRotationToViewerCoordinateFrame (neighbor.getVector3fMap (),
                                                           rotation_to_viewer_coordinate_system);
            float angle = nkdGetDirectionAngle (surface_change_direction, rotation_to_viewer_coordinate_system);
            int histogram_cell = (std::min) (angle_histogram_size-1,
                                    (int)pcl_lrint (floorf ( (angle+deg2rad (90.0f))/deg2rad (180.0f) * angle_histogram_size)));
            float& histogram_value = angle_histogram[histogram_cell];
            float& negative_score = interest_image[neighbor_index];
            
#           pragma omp critical
            {
              histogram_value = (std::max) (histogram_value, positive_score);
              negative_score  = (std::min) (negative_score, current_negative_score);
            }
          }
          //if (x==91 && y==88)
          //{
            //for (size_t i=0; i<invalid_beams.size (); ++i)
              //cout << (int)invalid_beams[i]<<" ";
            //cout << "\n";
          //}
        }
      }
    }
    //histogram_time += getTime ();
    //cout << PVARN (histogram_time);
    
    for (int y=0; y<height; ++y)
    {
      for (int x=0; x<width; ++x)
      {
        if (!range_image.isValid (x, y))
          continue;
        int index = y*width + x;
        float& interest_value = interest_image[index];
        const PointWithRange& point = range_image.getPoint(index);
        if (point.range < start_usage_range)
        {
          const RangeImage& half_range_image = *range_image_scale_space_[scale_idx+1];
          float* half_interest_image = interest_image_scale_space_[scale_idx+1];
          int half_x = std::min(x/2, int(half_range_image.width)-1),
              half_y = std::min(y/2, int(half_range_image.height)-1);
          interest_value = half_interest_image[half_y*half_range_image.width + half_x];
          continue;
        }
        float* angle_histogram = angle_histograms+angle_histogram_size*index;
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
            //cout << PVARC (histogram_cell1)<<PVARC (histogram_cell2)<<PVARN (normalized_angle_diff);
            
            angle_change_value = std::max (angle_histogram[histogram_cell1] * angle_histogram[histogram_cell2] *
                                           normalized_angle_diff,   angle_change_value);
          }
        }
        angle_change_value = sqrtf (angle_change_value);
        //cout << x<<","<<y<<": neg="<<interest_value<<", pos="<< angle_change_value<<"\n";
        interest_value *= angle_change_value;  // interest_value already contains a punishment according
                                               // to how much the immediate neighborhood changes
        
        if (parameters_.add_points_on_straight_edges)
        {
          float max_histogram_cell_value = 0.0f;
          for (int histogram_cell=0; histogram_cell<angle_histogram_size; ++histogram_cell)
            max_histogram_cell_value = (std::max) (max_histogram_cell_value, angle_histogram[histogram_cell]);
          interest_value = (std::min) (interest_value+max_histogram_cell_value, 1.0f);
        }
        
        //cout << PVARN (overall_negative_score);
        //overall_negative_score = -powf (-overall_negative_score, 2);  // Some scaling
      }
    }
    delete[] angle_histograms;
  }
  
  if (interest_image_scale_space_.empty())
    interest_image_ = NULL;
  else
    interest_image_ = interest_image_scale_space_[0];
  
# if USE_OMP
//#   pragma omp parallel for default (shared)
# endif
}

void NarfKeypoint::calculateInterestPoints ()
{
  //MEASURE_FUNCTION_TIME;
  
  //TODO: bivariate polynomials to get exact point position
  if (interest_points_ != NULL)
  {
    //cout << "Interest points member is not NULL => Doing nothing.\n";
    return;
  }
  calculateInterestImage ();
  
  interest_points_ = new ::pcl::PointCloud<InterestPoint>;
  
  float max_distance_squared = powf (0.3f*parameters_.support_size, 2);
  
  //cout << PVARN (range_image_border_extractor_->getParameters ());
  //cout << PVARN (this->getParameters ());
  
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  
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
  
  std::vector<InterestPoint> tmp_interest_points;
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float interest_value = interest_image_[index];
      if (interest_value < parameters_.min_interest_value)
        continue;
      const PointWithRange& point = range_image.getPoint (index);
      bool is_maximum = true;
      for (int y2=y-2; y2<=y+2&&is_maximum&&parameters_.do_non_maximum_suppression; ++y2)
      {
        for (int x2=x-2; x2<=x+2; ++x2)
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
        {
          //std::cout << "Could not compute polynomial approximation.\n";
          continue;
        }
        polynomial.findCriticalPoints (x_values, y_values, types);
        
        //for (size_t critical_point_idx=0; critical_point_idx<types.size (); ++critical_point_idx)
        //{
          //std::cout << "Found a "<< (types[critical_point_idx]==0 ? "maximum" :
                                    //(types[critical_point_idx]==1 ? "minimum" : "saddle point"))
                    //<< " at (" << x_values[critical_point_idx] << ", "<< y_values[critical_point_idx] << ").\n";
        //}
        
        if (!types.empty () && types[0]==0)
        {
          //if (fabsf(x_values[0])>3 || fabsf(y_values[0]>3))
            //break;
          float keypoint_x = x_values[0]+keypoint_x_int,
                keypoint_y = y_values[0]+keypoint_y_int;
          //cout << PVARC(poly_step) << PVARC(keypoint_x)<<PVARN(keypoint_y);
          
          keypoint_x_int = pcl_lrint (keypoint_x);
          keypoint_y_int = pcl_lrint (keypoint_y);
          //float keypoint_score = polynomial.getValue (x_values[0], y_values[0]);
          
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
      //cout << "\n\n";
      
      InterestPoint interest_point;
      interest_point.getVector3fMap () = keypoint_3d.getVector3fMap ();
      //interest_point.strength = std::min (keypoint_score, interest_value);
      interest_point.strength = interest_value;
      //interest_point.strength = interest_value;
      interest_point.strength = interest_value;
      tmp_interest_points.push_back (interest_point);
      
      //cout << "Original point: ("<<x<<","<<y<<" - "<<interest_value<<"), "
           //<< "polynomial point ("<<keypoint_x<<","<<keypoint_y<<" - "<<keypoint_score<<")\n";
      
      //is_interest_point_image_[keypoint_y_int*width+keypoint_x_int] = true;
    }
  }
  
  std::sort (tmp_interest_points.begin (), tmp_interest_points.end (), isBetterInterestPoint);
  
  float min_distance_squared = powf (parameters_.min_distance_between_interest_points*parameters_.support_size, 2);
  for (size_t int_point_idx=0; int_point_idx<tmp_interest_points.size (); ++int_point_idx)
  {
    if (parameters_.max_no_of_interest_points > 0  &&  int(interest_points_->size()) >= parameters_.max_no_of_interest_points)
      break;
    const InterestPoint& interest_point = tmp_interest_points[int_point_idx];
    //cout << PVARN (interest_point.strength);
    
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
    range_image.getImagePoint (interest_point.getVector3fMap (), image_x, image_y);
    if (range_image.isValid (image_x, image_y))
      is_interest_point_image_[image_y*width + image_x] = true;
  }
  interest_points_->width = interest_points_->points.size ();
  interest_points_->height = 1;
  interest_points_->is_dense = true;

#if 0

  float min_distance_squared = powf (parameters_.min_distance_between_interest_points*parameters_.support_size, 2),
        distance_for_additional_points = parameters_.distance_for_additional_points*parameters_.support_size,
        distance_for_additional_points_squared = distance_for_additional_points*distance_for_additional_points;
  is_interest_point_image_.clear ();
  is_interest_point_image_.resize (size, true);
  
  std::multimap<float, Eigen::Vector2i> ordered_points;
  for (int y=0; y<height; ++y)
  {
    for (int x=0; x<width; ++x)
    {
      int index = y*width + x;
      float interest_value = interest_image_[index];
      if (interest_value <= parameters_.min_interest_value || !range_image.isValid (index))
      {
        is_interest_point_image_[index] = false;
        continue;
      }
      ordered_points.insert (std::make_pair (interest_value, Eigen::Vector2i (x,y)));
    }
  }
  
  vector<int> neighbor_indices;
  vector<int> interest_point_indices;
  for (std::multimap<float, Eigen::Vector2i>::const_reverse_iterator it=ordered_points.rbegin ();
       it!=ordered_points.rend (); ++it)
  {
    int x=it->second[0], y=it->second[1], index = y*width + x;
    if (!is_interest_point_image_[index])
      continue;
    const PointWithRange& point = range_image.getPoint (index);
    InterestPoint interest_point;
    interest_point.x=point.x;  interest_point.y=point.y;  interest_point.z=point.z;
    interest_point.strength = interest_image_[index];
    
    bool is_maxmimum = true;
    bool stop = false;
    neighbor_indices.clear ();
    for (int radius=1;  !stop;  ++radius) 
    {
      int x2=x-radius-1, y2=y-radius;  // Top left - 1
      stop = true;
      for (int i=0; i<8*radius; ++i)
      {
        if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
        int neighbor_index = y2*width+x2;
        if (!range_image.isValid (x2, y2))
          continue;
        const PointWithRange& neighbor = range_image.getPoint (neighbor_index);
        if (radius>=parameters_.min_pixel_distance_between_interest_points &&
            squaredEuclideanDistance (point, neighbor)>min_distance_squared)
        {
          continue;
        }
        stop = false; // There is a point in range -> Have to check further distances
        neighbor_indices.push_back (neighbor_index);
        if (interest_image_[neighbor_index] > interest_point.strength)
          is_maxmimum = false;
      }
    }
    if (!parameters_.do_non_maximum_suppression || is_maxmimum)
    {
      interest_point_indices.push_back (index);
      for (unsigned int i=0; i<neighbor_indices.size (); ++i)
        is_interest_point_image_[neighbor_indices[i]] = false;
    }
    else
    {
      is_interest_point_image_[index] = false;
    }
  }
  
  for (unsigned int i=0; i<interest_point_indices.size (); ++i)
  {
    int index = interest_point_indices[i];
    const PointWithRange& point = range_image.getPoint (index);
    interest_points_->points.push_back (InterestPoint ());
    interest_points_->points.back ().getVector3fMap () = point.getVector3fMap ();
    interest_points_->points.back ().strength = interest_image_[index];
    if (distance_for_additional_points_squared > 0.0f)
    {
      float y=index/range_image.width, x=index-y*range_image.width;
      bool still_in_range = true;
      for (int radius=1;  still_in_range;  ++radius) 
      {
        int x2=x-radius-1, y2=y-radius;  // Top left - 1
        still_in_range = false;
        for (int i=0; i<8*radius; ++i)
        {
          if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
          if (!range_image.isValid (x2, y2))
            continue;
          int neighbor_index = y2*width+x2;
          const PointWithRange& neighbor = range_image.getPoint (neighbor_index);
          if (squaredEuclideanDistance (point, neighbor) > distance_for_additional_points_squared)
            continue;
          still_in_range = true;
          float neighbor_interest_value = interest_image_[neighbor_index];
          if (neighbor_interest_value > 0.5f*parameters_.min_interest_value)
          {
            //cout << "Adding "<<x2<<","<<y2<<" as neighbor of "<<x<<","<<y<<".\n";
            is_interest_point_image_[neighbor_index] = true;
            interest_points_->points.push_back (InterestPoint ());
            interest_points_->points.back ().getVector3fMap () = neighbor.getVector3fMap ();
            interest_points_->points.back ().strength = interest_image_[neighbor_index];
          }
        }
      }
    }
  }
#endif
}

const RangeImage& NarfKeypoint::getRangeImage ()
{
  return range_image_border_extractor_->getRangeImage ();
}

void NarfKeypoint::detectKeypoints (NarfKeypoint::PointCloudOut& output)
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
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

void NarfKeypoint::compute (NarfKeypoint::PointCloudOut& output)
{
  detectKeypoints (output);
}


//void NarfKeypoint::blurInterestImage ()
//{
  ////MEASURE_FUNCTION_TIME;
  
  //int blur_radius = parameters_.interest_image_blur_size;
  ////int blur_radius = 1;
  //if (blur_radius==0)
    //return;
  
  //const RangeImage& range_image = range_image_border_extractor_->getRangeImage ();
  //float* blurred_image = new float[range_image.width*range_image.height];
  
  //for (int y=0; y<int (range_image.height); ++y)
  //{
    //for (int x=0; x<int (range_image.width); ++x)
    //{
      //float& new_point = blurred_image[y*range_image.width + x];
      //new_point = 0.0f;
      //float weight_sum = 0.0f;
      //for (int y2=y-blur_radius; y2<y+blur_radius; ++y2)
      //{
        //for (int x2=x-blur_radius; x2<x+blur_radius; ++x2)
        //{
          //if (!range_image.isInImage (x2,y2))
            //continue;
          //new_point += interest_image_[y2*range_image.width + x2];
          //weight_sum += 1.0f;
        //}
      //}
      //new_point /= weight_sum;
    //}
  //}
  //delete[] interest_image_;
  //interest_image_ = blurred_image;
//}


}  // end namespace pcl

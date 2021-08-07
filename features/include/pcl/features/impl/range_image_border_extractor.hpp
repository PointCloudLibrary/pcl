/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 * Author: Bastian Steder
 */

#include <pcl/range_image/range_image.h>

namespace pcl {

////////// STATIC //////////
float RangeImageBorderExtractor::getObstacleBorderAngle(const BorderTraits& border_traits)
{
  float x=0.0f, y=0.0f;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT])
    ++x;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT])
    --x;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP])
    --y;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM])
    ++y;
  
  return std::atan2(y, x);
}

inline std::ostream& operator << (std::ostream& os, const RangeImageBorderExtractor::Parameters& p)
{
  os << PVARC(p.pixel_radius_borders)<<PVARC(p.pixel_radius_plane_extraction)<<PVARC(p.pixel_radius_border_direction)
     << PVARC(p.minimum_border_probability)<<PVARN(p.pixel_radius_principal_curvature);
  return (os);
}

////////// NON-STATIC //////////


float RangeImageBorderExtractor::getNeighborDistanceChangeScore(
    const RangeImageBorderExtractor::LocalSurface& local_surface,
    int x, int y, int offset_x, int offset_y, int pixel_radius) const
{
  const PointWithRange& point = range_image_->getPoint(x, y);
  PointWithRange neighbor;
  range_image_->get1dPointAverage(x+offset_x, y+offset_y, offset_x, offset_y, pixel_radius, neighbor);
  if (std::isinf(neighbor.range))
  {
    if (neighbor.range < 0.0f)
      return 0.0f;
    //std::cout << "INF edge -> Setting to 1.0\n";
    return 1.0f;  // TODO: Something more intelligent
  }
  
  float neighbor_distance_squared = squaredEuclideanDistance(neighbor, point);
  if (neighbor_distance_squared <= local_surface.max_neighbor_distance_squared)
    return 0.0f;
  float ret = 1.0f - std::sqrt (local_surface.max_neighbor_distance_squared / neighbor_distance_squared);
  if (neighbor.range < point.range)
    ret = -ret;
  return ret;
}

//float RangeImageBorderExtractor::getNormalBasedBorderScore(const RangeImageBorderExtractor::LocalSurface& local_surface,
                                                           //int x, int y, int offset_x, int offset_y) const
//{
  //PointWithRange neighbor;
  //range_image_->get1dPointAverage(x+offset_x, y+offset_y, offset_x, offset_y, parameters_.pixel_radius_borders, neighbor);
  //if (std::isinf(neighbor.range))
  //{
    //if (neighbor.range < 0.0f)
      //return 0.0f;
    //else
      //return 1.0f;  // TODO: Something more intelligent (Compare normal with viewing direction)
  //}
  
  //float normal_distance_to_plane_squared = local_surface.smallest_eigenvalue_no_jumps;
  //float distance_to_plane = local_surface.normal_no_jumps.dot(local_surface.neighborhood_mean_no_jumps-neighbor.getVector3fMap());
  //bool shadow_side = distance_to_plane < 0.0f;
  //float distance_to_plane_squared = pow(distance_to_plane, 2);
  //if (distance_to_plane_squared <= normal_distance_to_plane_squared)
    //return 0.0f;
  //float ret = 1.0f - (normal_distance_to_plane_squared/distance_to_plane_squared);
  //if (shadow_side)
    //ret = -ret;
  ////std::cout << PVARC(normal_distance_to_plane_squared)<<PVAR(distance_to_plane_squared)<<" => "<<ret<<"\n";
  //return ret;
//}

bool RangeImageBorderExtractor::get3dDirection(const BorderDescription& border_description, Eigen::Vector3f& direction,
                                               const LocalSurface* local_surface)
{
  const BorderTraits border_traits = border_description.traits;
  
  int delta_x=0, delta_y=0;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_RIGHT])
    ++delta_x;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_LEFT])
    --delta_x;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_TOP])
    --delta_y;
  if (border_traits[BORDER_TRAIT__OBSTACLE_BORDER_BOTTOM])
    ++delta_y;
  
  if (delta_x==0 && delta_y==0)
    return false;
  
  int x=border_description.x, y=border_description.y;
  const PointWithRange& point = range_image_->getPoint(x, y);
  Eigen::Vector3f neighbor_point;
  range_image_->calculate3DPoint(static_cast<float> (x+delta_x), static_cast<float> (y+delta_y), point.range, neighbor_point);
  //std::cout << "Neighborhood point is "<<neighbor_point[0]<<", "<<neighbor_point[1]<<", "<<neighbor_point[2]<<".\n";
  
  if (local_surface!=nullptr)
  {
    // Get the point that lies on the local plane approximation
    Eigen::Vector3f sensor_pos = range_image_->getSensorPos(),
                    viewing_direction = neighbor_point-sensor_pos;

    float lambda = (local_surface->normal_no_jumps.dot(local_surface->neighborhood_mean_no_jumps-sensor_pos)/
                   local_surface->normal_no_jumps.dot(viewing_direction));
    neighbor_point = lambda*viewing_direction + sensor_pos;
    //std::cout << "Neighborhood point projected onto plane is "<<neighbor_point[0]<<", "<<neighbor_point[1]<<", "<<neighbor_point[2]<<".\n";
  }
  //std::cout << point.x<<","<< point.y<<","<< point.z<<" -> "<< direction[0]<<","<< direction[1]<<","<< direction[2]<<"\n";
  direction = neighbor_point-point.getVector3fMap();
  direction.normalize();
  
  return true;
}

void RangeImageBorderExtractor::calculateBorderDirection(int x, int y)
{
  int index = y*range_image_->width + x;
  Eigen::Vector3f*& border_direction = border_directions_[index];
  border_direction = nullptr;
  const BorderDescription& border_description = (*border_descriptions_)[index];
  const BorderTraits& border_traits = border_description.traits;
  if (!border_traits[BORDER_TRAIT__OBSTACLE_BORDER])
    return;
  border_direction = new Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  if (!get3dDirection(border_description, *border_direction, surface_structure_[index]))
  {
    delete border_direction;
    border_direction = nullptr;
    return;
  }
}

bool RangeImageBorderExtractor::changeScoreAccordingToShadowBorderValue(int x, int y, int offset_x, int offset_y, float* border_scores,
                                                                        float* border_scores_other_direction, int& shadow_border_idx) const
{
  float& border_score = border_scores[y*range_image_->width+x];

  shadow_border_idx = -1;
  if (border_score<parameters_.minimum_border_probability)
    return false;
  
  if (border_score == 1.0f) 
  {  // INF neighbor?
    if (range_image_->isMaxRange(x+offset_x, y+offset_y))
    {
      shadow_border_idx = (y+offset_y)*range_image_->width + x+offset_x;
      return true;
    }
  }
  
  float best_shadow_border_score = 0.0f;
  
  for (int neighbor_distance=1; neighbor_distance<=parameters_.pixel_radius_borders; ++neighbor_distance)
  {
    int neighbor_x=x+neighbor_distance*offset_x, neighbor_y=y+neighbor_distance*offset_y;
    if (!range_image_->isInImage(neighbor_x, neighbor_y))
      continue;
    float neighbor_shadow_border_score = border_scores_other_direction[neighbor_y*range_image_->width+neighbor_x];
    
    if (neighbor_shadow_border_score < best_shadow_border_score)
    {
      shadow_border_idx = neighbor_y*range_image_->width + neighbor_x;
      best_shadow_border_score = neighbor_shadow_border_score;
    }
  }
  if (shadow_border_idx >= 0)
  {
    //std::cout << PVARC(border_score)<<PVARN(best_shadow_border_score);
    //border_score *= (std::max)(0.9f, powf(-best_shadow_border_score, 0.1f));  // TODO: Something better
    border_score *= (std::max)(0.9f, 1-powf(1+best_shadow_border_score, 3));
    if (border_score>=parameters_.minimum_border_probability)
      return true;
  }
  shadow_border_idx = -1;
  border_score = 0.0f;  // Since there was no shadow border found we set this value to zero, so that it does not influence the maximum search
  return false;
}

float RangeImageBorderExtractor::updatedScoreAccordingToNeighborValues(int x, int y, const float* border_scores) const
{
  float max_score_bonus = 0.5f;
  
  float border_score = border_scores[y*range_image_->width+x];
  
  // Check if an update can bring the score to a value higher than the minimum
  if (border_score + max_score_bonus*(1.0f-border_score) < parameters_.minimum_border_probability)
    return border_score;
  
  float average_neighbor_score=0.0f, weight_sum=0.0f;
  for (int y2=y-1; y2<=y+1; ++y2)
  {
    for (int x2=x-1; x2<=x+1; ++x2)
    {
      if (!range_image_->isInImage(x2, y2) || (x2==x&&y2==y))
        continue;
      average_neighbor_score += border_scores[y2*range_image_->width+x2];
      weight_sum += 1.0f;
    }
  }
  average_neighbor_score /=weight_sum;
  
  if (average_neighbor_score*border_score < 0.0f)
    return border_score;
  
  float new_border_score = border_score + max_score_bonus * average_neighbor_score * (1.0f-std::abs(border_score));
  
  //std::cout << PVARC(border_score)<<PVARN(new_border_score);
  return new_border_score;
}

bool RangeImageBorderExtractor::checkPotentialBorder(int x, int y, int offset_x, int offset_y, float* border_scores,
                                                     float* border_scores_other_direction, int& shadow_border_idx) const
{
  float& border_score = border_scores[y*range_image_->width+x];
  if (border_score<parameters_.minimum_border_probability)
    return false;
  
  shadow_border_idx = -1;
  float best_shadow_border_score = -0.5f*parameters_.minimum_border_probability;
  
  for (int neighbor_distance=1; neighbor_distance<=parameters_.pixel_radius_borders; ++neighbor_distance)
  {
    int neighbor_x=x+neighbor_distance*offset_x, neighbor_y=y+neighbor_distance*offset_y;
    if (!range_image_->isInImage(neighbor_x, neighbor_y))
      continue;
    float neighbor_shadow_border_score = border_scores_other_direction[neighbor_y*range_image_->width+neighbor_x];
    
    if (neighbor_shadow_border_score < best_shadow_border_score)
    {
      shadow_border_idx = neighbor_y*range_image_->width + neighbor_x;
      best_shadow_border_score = neighbor_shadow_border_score;
    }
  }
  if (shadow_border_idx >= 0)
  {
    return true;
  }
  border_score = 0.0f;  // Since there was no shadow border found we set this value to zero, so that it does not influence the maximum search
  return false;
}

bool RangeImageBorderExtractor::checkIfMaximum(int x, int y, int offset_x, int offset_y, float* border_scores, int shadow_border_idx) const
{
  float border_score = border_scores[y*range_image_->width+x];
  int neighbor_x=x-offset_x, neighbor_y=y-offset_y;
  if (range_image_->isInImage(neighbor_x, neighbor_y) && border_scores[neighbor_y*range_image_->width+neighbor_x] > border_score)
    return false;
  
  for (int neighbor_distance=1; neighbor_distance<=parameters_.pixel_radius_borders; ++neighbor_distance)
  {
    neighbor_x=x+neighbor_distance*offset_x; neighbor_y=y+neighbor_distance*offset_y;
    if (!range_image_->isInImage(neighbor_x, neighbor_y))
      continue;
    int neighbor_index = neighbor_y*range_image_->width + neighbor_x;
    if (neighbor_index==shadow_border_idx)
      return true;
    
    float neighbor_border_score = border_scores[neighbor_index];
    if (neighbor_border_score > border_score)
      return false;
  }
  return true;
}

bool RangeImageBorderExtractor::calculateMainPrincipalCurvature(int x, int y, int radius, float& magnitude,
                                                                Eigen::Vector3f& main_direction) const
{
  magnitude = 0.0f;
  int index = y*range_image_->width+x;
  LocalSurface* local_surface = surface_structure_[index];
  if (local_surface==nullptr)
    return false;
  //const PointWithRange& point = range_image_->getPointNoCheck(x,y);
  
  //Eigen::Vector3f& normal = local_surface->normal_no_jumps;
  //Eigen::Matrix3f to_tangent_plane = Eigen::Matrix3f::Identity() - normal*normal.transpose();
  
  VectorAverage3f vector_average;
  bool beams_valid[9];
  for (int step=1; step<=radius; ++step)
  {
    int beam_idx = 0;
    for (int y2=y-step; y2<=y+step; y2+=step)
    {
      for (int x2=x-step; x2<=x+step; x2+=step)
      {
        bool& beam_valid = beams_valid[beam_idx++];
        if (step==1)
        {
          if (x2==x && y2==y)
            beam_valid = false;
          else
            beam_valid = true;
        }
        else
          if (!beam_valid)
            continue;
        //std::cout << x2-x<<","<<y2-y<<"  ";
        
        if (!range_image_->isValid(x2,y2))
          continue;
        
        int index2 = y2*range_image_->width + x2;
        
        const BorderTraits& border_traits = (*border_descriptions_)[index2].traits;
        if (border_traits[BORDER_TRAIT__VEIL_POINT] || border_traits[BORDER_TRAIT__SHADOW_BORDER])
        {
          beam_valid = false;
          continue;
        }
        
        //const PointWithRange& point2 = range_image_->getPoint(index2);
        LocalSurface* local_surface2 = surface_structure_[index2];
        if (local_surface2==nullptr)
          continue;
        Eigen::Vector3f& normal2 = local_surface2->normal_no_jumps;
        //float distance_squared = squaredEuclideanDistance(point, point2);
        //vector_average.add(to_tangent_plane*normal2);
        vector_average.add(normal2);
      }
    }
  }
  //std::cout << "\n";
  if (vector_average.getNoOfSamples() < 3)
    return false;
  
  Eigen::Vector3f eigen_values, eigen_vector1, eigen_vector2;
  vector_average.doPCA(eigen_values, eigen_vector1, eigen_vector2, main_direction);
  magnitude = std::sqrt (eigen_values[2]);
  //magnitude = eigen_values[2];
  //magnitude = 1.0f - powf(1.0f-magnitude, 5);
  //magnitude = 1.0f - powf(1.0f-magnitude, 10);
  //magnitude += magnitude - powf(magnitude,2);
  //magnitude += magnitude - powf(magnitude,2);
  
  //magnitude = std::sqrt (local_surface->eigen_values[0]/local_surface->eigen_values.sum());
  //magnitude = std::sqrt (local_surface->eigen_values_no_jumps[0]/local_surface->eigen_values_no_jumps.sum());

  //if (surface_structure_[y*range_image_->width+x+1]==NULL||surface_structure_[y*range_image_->width+x-1]==NULL)
  //{
    //magnitude = -std::numeric_limits<float>::infinity ();
    //return false;
  //}
  //float angle2 = std::acos(surface_structure_[y*range_image_->width+x+1]->normal.dot(local_surface->normal)),
        //angle1 = std::acos(surface_structure_[y*range_image_->width+x-1]->normal.dot(local_surface->normal));
  //magnitude = angle2-angle1;

  return std::isfinite(magnitude);
}

}  // namespace end

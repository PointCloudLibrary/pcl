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

#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>

#include <iostream>
#include <fstream>
#include <cmath>
using std::cout;
using std::cerr;
using std::vector;

using Eigen::Vector3f;

#include <pcl/range_image/range_image.h>
#include <pcl/common/vector_average.h>
#include <pcl/common/common_headers.h>

namespace pcl 
{
int Narf::max_no_of_threads = 1;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Narf::Narf() : 
  position_ (), transformation_ (), surface_patch_ (NULL), 
  surface_patch_pixel_size_ (0), surface_patch_world_size_ (), 
  surface_patch_rotation_ (), descriptor_ (NULL), descriptor_size_ (0)
{
  reset();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Narf::~Narf()
{
  reset();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Narf::Narf (const Narf& other) : 
  position_ (), transformation_ (), surface_patch_ (NULL), 
  surface_patch_pixel_size_ (0), surface_patch_world_size_ (), 
  surface_patch_rotation_ (), descriptor_ (NULL), descriptor_size_ (0)
{
  deepCopy (other);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const Narf& 
Narf::operator= (const Narf& other)
{
  deepCopy (other);
  return (*this);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::reset ()
{
  delete[] descriptor_;
  descriptor_ = NULL;
  descriptor_size_ = 0;
  delete[] surface_patch_;
  surface_patch_ = NULL;
  surface_patch_pixel_size_ = 0;
  surface_patch_world_size_ = 0.0f;
  surface_patch_rotation_ = 0.0f;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::deepCopy (const Narf& other)
{
  //cout << __PRETTY_FUNCTION__<<" called.\n";
  if (&other == this)
    return;
  
  position_ = other.position_;
  transformation_ = other.transformation_;
  
  if (surface_patch_pixel_size_ != other.surface_patch_pixel_size_)
  {
    surface_patch_pixel_size_ = other.surface_patch_pixel_size_;
    delete[] surface_patch_;
    surface_patch_ = new float[surface_patch_pixel_size_*surface_patch_pixel_size_];
  }
  memcpy(surface_patch_, other.surface_patch_, sizeof(*surface_patch_)*surface_patch_pixel_size_*surface_patch_pixel_size_);
  surface_patch_world_size_ = other.surface_patch_world_size_;
  surface_patch_rotation_ = other.surface_patch_rotation_;
  
  if (descriptor_size_ != other.descriptor_size_)
  {
    descriptor_size_ = other.descriptor_size_;
    delete[] descriptor_;
    descriptor_ = new float[descriptor_size_];
  }
  memcpy(descriptor_, other.descriptor_, sizeof(*descriptor_)*descriptor_size_);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
Narf::extractDescriptor (int descriptor_size)
{
  float weight_for_first_point = 2.0f; // The weight for the last point is always 1.0f
  int no_of_beam_points = getNoOfBeamPoints();
  float weight_factor = -2.0f*(weight_for_first_point-1.0f) / ((weight_for_first_point+1.0f)*float(no_of_beam_points-1)),
        weight_offset = 2.0f*weight_for_first_point / (weight_for_first_point+1.0f);

  if (descriptor_size != descriptor_size_)
  {
    descriptor_size_ = descriptor_size;
    delete descriptor_;
    descriptor_ = new float[descriptor_size_];
  }
  float angle_step_size = deg2rad (360.0f) / static_cast<float> (descriptor_size_);
  //cout << PVARN(no_of_beam_points)<<PVARN(surface_patch_pixel_size_);

  float cell_size = surface_patch_world_size_/float(surface_patch_pixel_size_),
        cell_factor = 1.0f/cell_size,
        cell_offset = 0.5f*(surface_patch_world_size_ - cell_size),
        max_dist = 0.5f*surface_patch_world_size_,
        beam_point_factor = (max_dist-0.5f*cell_size) / static_cast<float> (no_of_beam_points);

  for (int descriptor_value_idx=0; descriptor_value_idx<descriptor_size_; ++descriptor_value_idx)
  {
    float& descriptor_value = descriptor_[descriptor_value_idx];
    descriptor_value = 0.0f;
    float angle = static_cast<float> (descriptor_value_idx) * angle_step_size + surface_patch_rotation_,
          beam_point_factor_x = sinf(angle) * beam_point_factor,
          beam_point_factor_y = -cosf (angle) * beam_point_factor;
     
    std::vector<float> beam_values (no_of_beam_points + 1);
    float current_cell = 0.0f;
    for (int beam_point_idx=0; beam_point_idx<=no_of_beam_points; ++beam_point_idx)
    {
      float& beam_value = beam_values[beam_point_idx];

      float beam_point_x = beam_point_factor_x * static_cast<float> (beam_point_idx),
            beam_point_y = beam_point_factor_y * static_cast<float> (beam_point_idx);
      float beam_point_cell_x = cell_factor * (beam_point_x + cell_offset),
            beam_point_cell_y = cell_factor * (beam_point_y + cell_offset);

      int cell_x = static_cast<int> (pcl_lrint (beam_point_cell_x)), cell_y = static_cast<int> (pcl_lrint (beam_point_cell_y));
      beam_value = surface_patch_[cell_y*surface_patch_pixel_size_ + cell_x];
      if (!pcl_isfinite(beam_value))
      {
        if (beam_value > 0.0f)
          beam_value = max_dist;
        else
          beam_value = -std::numeric_limits<float>::infinity ();
      }
    }
    for (int beam_value_idx=0; beam_value_idx<no_of_beam_points; ++beam_value_idx)
    {
      float beam_value1=beam_values[beam_value_idx],
            beam_value2=beam_values[beam_value_idx+1];

      float current_weight = weight_factor*float(beam_value_idx) + weight_offset;
      float diff = beam_value2-beam_value1;
      current_cell += current_weight * diff;
    }
    // Scaling for easy descriptor distances:
    current_cell = atan2f (current_cell, max_dist) / deg2rad(180.0f);  // Scales the result to [-0.5, 0.5]
    descriptor_value = current_cell;
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
Narf::extractFromRangeImage (const RangeImage& range_image, const Eigen::Affine3f& pose, int descriptor_size,
                             float support_size, int surface_patch_pixel_size)
{
  reset();
  transformation_ = pose;
  position_ = pose.inverse (Eigen::Isometry).translation ();
  surface_patch_world_size_ = support_size;
  surface_patch_pixel_size_ = surface_patch_pixel_size;
  surface_patch_rotation_ = 0.0f;
  surface_patch_ = range_image.getInterpolatedSurfaceProjection(pose, surface_patch_pixel_size_, surface_patch_world_size_);
  int new_pixel_size = 2*surface_patch_pixel_size_;
  int blur_radius = 1;
  float* new_surface_patch = getBlurredSurfacePatch(new_pixel_size, blur_radius);
  delete[] surface_patch_;
  surface_patch_pixel_size_ = new_pixel_size;
  surface_patch_ = new_surface_patch;
  extractDescriptor(descriptor_size);
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
Narf::extractFromRangeImage (const RangeImage& range_image, float x, float y, int descriptor_size, float support_size)
{
  if (!range_image.isValid (static_cast<int> (pcl_lrint (x)), static_cast<int> (pcl_lrint (y))))
    return (false);
  Eigen::Vector3f feature_pos;
  range_image.calculate3DPoint(x, y, feature_pos);
  
  return (extractFromRangeImage (range_image, feature_pos, descriptor_size, support_size));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
Narf::extractFromRangeImage (const RangeImage& range_image, const InterestPoint& interest_point, int descriptor_size, float support_size)
{
  return extractFromRangeImage(range_image, Eigen::Vector3f(interest_point.x, interest_point.y, interest_point.z), descriptor_size, support_size);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
Narf::extractFromRangeImage (const RangeImage& range_image, const Eigen::Vector3f& interest_point, int descriptor_size, float support_size)
{
  if (!range_image.getNormalBasedUprightTransformation(interest_point, 0.5f*support_size, transformation_))
    return false;
  return extractFromRangeImage(range_image, transformation_, descriptor_size, support_size);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool 
Narf::extractFromRangeImageWithBestRotation (const RangeImage& range_image, const Eigen::Vector3f& interest_point,
                                             int descriptor_size, float support_size)
{
  extractFromRangeImage(range_image, interest_point, descriptor_size, support_size);
  vector<float> rotations, strengths;
  getRotations(rotations, strengths);
  if (rotations.empty())
    return false;
  float best_rotation=rotations[0], best_strength=strengths[0];
  for (unsigned int i=1; i<rotations.size(); ++i)
  {
    if (strengths[i] > best_strength)
    {
      best_rotation = rotations[i];
      best_strength = strengths[i];
    }
  }
  
  transformation_ = Eigen::AngleAxisf(-best_rotation, Eigen::Vector3f(0.0f, 0.0f, 1.0f))*transformation_;
  surface_patch_rotation_ = best_rotation;
  return extractDescriptor(descriptor_size_);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float* 
Narf::getBlurredSurfacePatch (int new_pixel_size, int blur_radius) const
{
  float new_to_old_factor = float(surface_patch_pixel_size_)/float(new_pixel_size);
  int new_size = new_pixel_size*new_pixel_size;
  
  float* integral_image = new float[new_size];
  float* integral_image_ptr = integral_image;
  for (int y=0; y<new_pixel_size; ++y)
  {
    for (int x=0; x<new_pixel_size; ++x)
    {
      float& integral_pixel = *(integral_image_ptr++);
      int old_x = static_cast<int> (pcl_lrint (floor (new_to_old_factor * float (x)))),
          old_y = static_cast<int> (pcl_lrint (floor (new_to_old_factor * float (y))));
      integral_pixel = surface_patch_[old_y*surface_patch_pixel_size_ + old_x];
      if (pcl_isinf(integral_pixel))
        integral_pixel = 0.5f*surface_patch_world_size_;
      float left_value=0, top_left_value=0, top_value=0;
      if (x>0)
      {
        left_value = integral_image[y*new_pixel_size+x-1];
        if (y>0)
          top_left_value = integral_image[(y-1)*new_pixel_size+x-1];
      }
      if (y>0)
        top_value = integral_image[(y-1)*new_pixel_size+x];
      
      integral_pixel += left_value + top_value - top_left_value;
      //cout << PVARC(x)<<PVARC(y)<<PVARC(left_value)<<PVARC(top_value)<<PVARC(top_left_value)<<PVARN(integral_pixel)<<PVARN(integral_image[y*new_pixel_size+x-1]);
    }
  }
  
  float* new_surface_patch = new float[new_size];
  float* new_surface_patch_ptr = new_surface_patch;
  for (int y=0; y<new_pixel_size; ++y)
  {
    for (int x=0; x<new_pixel_size; ++x)
    {
      float& new_value = *(new_surface_patch_ptr++);
      
      int top = (std::max) (-1, y-blur_radius-1), right=(std::min) (new_pixel_size-1, x+blur_radius), bottom = (std::min) (new_pixel_size-1, y+blur_radius), left=(std::max) (-1, x-blur_radius-1),
          pixelSum = (right-left)*(bottom-top);
      float normalizationFactor = 1.0f / static_cast<float> (pixelSum);
      
      float top_left_value=0, top_right_value=0, bottom_right_value=integral_image[bottom*new_pixel_size+right], bottom_left_value=0;
      if (left>=0)
      {
        bottom_left_value = integral_image[bottom*new_pixel_size+left];
        if (top>=0)
          top_left_value = integral_image[top*new_pixel_size+left];
      }
      if (top>=0)
        top_right_value = integral_image[top*new_pixel_size+right];
      new_value = normalizationFactor * (bottom_right_value + top_left_value - bottom_left_value - top_right_value);
    }
  }
  delete[] integral_image;
  return new_surface_patch;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::extractFromRangeImageAndAddToList (const RangeImage& range_image, const Eigen::Vector3f& interest_point, int descriptor_size,
                                         float support_size, bool rotation_invariant, std::vector<Narf*>& feature_list)
{
  Narf* feature = new Narf;
  if (!feature->extractFromRangeImage(range_image, interest_point, descriptor_size, support_size))
  {
    delete feature;
    return;
  }
  if (!rotation_invariant)
  {
    feature_list.push_back(feature);
    return;
  }
  vector<float> rotations, strengths;
  feature->getRotations(rotations, strengths);
  feature->getRotatedVersions(range_image, rotations, feature_list);
  delete feature;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::extractFromRangeImageAndAddToList (const RangeImage& range_image, float image_x, float image_y, int descriptor_size,
                                         float support_size, bool rotation_invariant, std::vector<Narf*>& feature_list)
{
  if (!range_image.isValid (static_cast<int> (pcl_lrint (image_x)), static_cast<int> (pcl_lrint (image_y))))
    return;
  Eigen::Vector3f feature_pos;
  range_image.calculate3DPoint(image_x, image_y, feature_pos);
  extractFromRangeImageAndAddToList(range_image, feature_pos, descriptor_size, support_size, rotation_invariant, feature_list);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::extractForInterestPoints (const RangeImage& range_image, const PointCloud<InterestPoint>& interest_points,
                                int descriptor_size, float support_size, bool rotation_invariant, std::vector<Narf*>& feature_list)
{
  # pragma omp parallel for num_threads(max_no_of_threads) default(shared) schedule(dynamic, 10)
  //!!! nizar 20110408 : for OpenMP sake on MSVC this must be kept signed
  for (int interest_point_idx = 0; interest_point_idx < int (interest_points.points.size ()); ++interest_point_idx)
  {
    const InterestPoint& interest_point = interest_points.points[interest_point_idx];
    Vector3fMapConst point = interest_point.getVector3fMap ();
    
    Narf* feature = new Narf;
    if (!feature->extractFromRangeImage(range_image, point, descriptor_size, support_size))
    {
      delete feature;
    }
    else {
      if (!rotation_invariant)
      {
#       pragma omp critical
        {
          feature_list.push_back(feature);
        }
      }
      else {
        vector<float> rotations, strengths;
        feature->getRotations(rotations, strengths);
        {
          //feature->getRotatedVersions(range_image, rotations, feature_list);
          for (unsigned int i=0; i<rotations.size(); ++i)
          {
            float rotation = rotations[i];
            Narf* feature2 = new Narf(*feature);  // Call copy constructor
            feature2->transformation_ = Eigen::AngleAxisf(-rotation, Eigen::Vector3f(0.0f, 0.0f, 1.0f))*feature2->transformation_;
            feature2->surface_patch_rotation_ = rotation;
            if (!feature2->extractDescriptor(feature2->descriptor_size_))
            {
              delete feature2;
              continue;
            }
#           pragma omp critical
            {
              feature_list.push_back(feature2);
            }
          }
        }
        delete feature;
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::extractForEveryRangeImagePointAndAddToList (const RangeImage& range_image, int descriptor_size, float support_size,
                                                  bool rotation_invariant, std::vector<Narf*>& feature_list)
{
  for (unsigned int y=0; y<range_image.height; ++y)
  {
    for (unsigned int x=0; x<range_image.width; ++x)
    {
      extractFromRangeImageAndAddToList(range_image, static_cast<float> (x), static_cast<float> (y), descriptor_size, support_size,
                                        rotation_invariant, feature_list);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::getRotations (std::vector<float>& rotations, std::vector<float>& strengths) const
{
  int angle_steps_no = (std::max) (descriptor_size_, 36);
  float min_angle_dist_between_rotations = deg2rad(70.0f);
  float angle_step_size1 = deg2rad (360.0f) / static_cast<float> (angle_steps_no);
  float angle_step_size2 = deg2rad (360.0f) / static_cast<float> (descriptor_size_);
  
  float score_normalization = 1.0f / static_cast<float> (descriptor_size_);
  
  std::multimap<float, float> scored_orientations;
  for (int step=0; step<angle_steps_no; ++step)
  {
    float angle = static_cast<float> (step) * angle_step_size1;
    float score = 0.0f;
    
    for (int descriptor_value_idx=0; descriptor_value_idx<descriptor_size_; ++descriptor_value_idx)
    {
      float value = descriptor_[descriptor_value_idx];
      float angle2 = static_cast<float> (descriptor_value_idx) * angle_step_size2;
      float distance_weight = powf (1.0f - fabsf (normAngle (angle - angle2)) / deg2rad (180.0f), 2.0f);
      
      score += value * distance_weight;
    }
    score = score_normalization*score + 0.5f; // Scale value to [0,1]
    scored_orientations.insert(std::make_pair(score, angle));
  }
  
  //for (std::multimap<float, float>::const_iterator it=scored_orientations.begin(); it!=scored_orientations.end(); ++it)
    //cout << "Score "<<it->first<<" for angle "<<rad2deg(it->second)<<".\n";
  
  float min_score = scored_orientations.begin()->first,
        max_score = scored_orientations.rbegin()->first;
  
  float min_score_for_remaining_rotations = max_score - 0.2f*(max_score-min_score);
  scored_orientations.erase(scored_orientations.begin(), scored_orientations.upper_bound(min_score_for_remaining_rotations));
  //cout << "There are "<<scored_orientations.size()<<" potential orientations left after filtering out bad scores.\n";
  
  while (!scored_orientations.empty())
  {
    std::multimap<float, float>::iterator best_remaining_orientation_it = scored_orientations.end();
    --best_remaining_orientation_it;
    rotations.push_back(best_remaining_orientation_it->second);
    strengths.push_back(best_remaining_orientation_it->first);
    scored_orientations.erase(best_remaining_orientation_it);
    for (std::multimap<float, float>::iterator it = scored_orientations.begin(); it!=scored_orientations.end();)
    {
      std::multimap<float, float>::iterator current_it = it++;
      if (normAngle(current_it->second - rotations.back()) < min_angle_dist_between_rotations)
        scored_orientations.erase(current_it);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::getRotatedVersions (const RangeImage&, const std::vector<float>& rotations, std::vector<Narf*>& features) const
{
  for (unsigned int i=0; i<rotations.size(); ++i)
  {
    float rotation = rotations[i];
    
    Narf* feature = new Narf(*this);  // Call copy constructor
    feature->transformation_ = Eigen::AngleAxisf(-rotation, Eigen::Vector3f(0.0f, 0.0f, 1.0f))*feature->transformation_;
    feature->surface_patch_rotation_ = rotation;
    if (!feature->extractDescriptor(feature->descriptor_size_))
    {
      delete feature;
      continue;
    }
    features.push_back(feature);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::saveHeader (std::ostream& file) const
{
  file << "\n"<<getHeaderKeyword()<<" "<<Narf::VERSION<<" ";
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::saveBinary (std::ostream& file) const
{
  saveHeader(file);
  pcl::saveBinary(position_.matrix(), file);
  pcl::saveBinary(transformation_.matrix(), file);
  file.write(reinterpret_cast<const char*>(&surface_patch_pixel_size_), sizeof(surface_patch_pixel_size_));
  file.write(reinterpret_cast<const char*>(surface_patch_),
             surface_patch_pixel_size_*surface_patch_pixel_size_*sizeof(*surface_patch_));
  file.write(reinterpret_cast<const char*>(&surface_patch_world_size_), sizeof(surface_patch_world_size_));
  file.write(reinterpret_cast<const char*>(&surface_patch_rotation_), sizeof(surface_patch_rotation_));
  file.write(reinterpret_cast<const char*>(&descriptor_size_), sizeof(descriptor_size_));
  file.write(reinterpret_cast<const char*>(descriptor_), descriptor_size_*sizeof(*descriptor_));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::saveBinary (const std::string& filename) const
{
  std::ofstream file;
  file.open(filename.c_str());
  saveBinary(file);
  file.close();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int 
Narf::loadHeader(std::istream& file) const
{
  size_t pos_in_file = static_cast<size_t> (file.tellg ());
  file.width (getHeaderKeyword ().size()+10); // limit maximum number of bytes to read
  std::string header;
  file >> header;
  file.width(0); // remove limit
  if (header != getHeaderKeyword())
  {
    file.seekg(pos_in_file);  // Go back to former pos in file
    return -1;
  }
  int version;
  file >> version;
  file.ignore (1);  // Skip the space after the version number
  
  return (version);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::loadBinary (std::istream& file)
{
  reset();
  int version = loadHeader(file);
  if (version<0)
  {
    std::cerr << __PRETTY_FUNCTION__ << "Incorrect header!\n";
    return;
  }
  pcl::loadBinary(position_.matrix(), file);
  pcl::loadBinary(transformation_.matrix(), file);
  file.read(reinterpret_cast<char*>(&surface_patch_pixel_size_), sizeof(surface_patch_pixel_size_));
  surface_patch_ = new float[surface_patch_pixel_size_*surface_patch_pixel_size_];
  file.read(reinterpret_cast<char*>(surface_patch_),
            surface_patch_pixel_size_*surface_patch_pixel_size_*sizeof(*surface_patch_));
  file.read(reinterpret_cast<char*>(&surface_patch_world_size_), sizeof(surface_patch_world_size_));
  file.read(reinterpret_cast<char*>(&surface_patch_rotation_), sizeof(surface_patch_rotation_));
  file.read(reinterpret_cast<char*>(&descriptor_size_), sizeof(descriptor_size_));
  descriptor_ = new float[descriptor_size_];
  if (file.eof())
    cout << ":-(\n";
  file.read (reinterpret_cast<char*>(descriptor_), descriptor_size_*sizeof(*descriptor_));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
Narf::loadBinary (const std::string& filename)
{
  std::ifstream file;
  file.open (filename.c_str ());
  loadBinary (file);
  file.close ();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
NarfDescriptor::NarfDescriptor (const RangeImage* range_image, const std::vector<int>* indices) : 
  BaseClass (), range_image_ (), parameters_ ()
{
  setRangeImage (range_image, indices);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
NarfDescriptor::~NarfDescriptor ()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
NarfDescriptor::setRangeImage (const RangeImage* range_image, const std::vector<int>* indices)
{
  range_image_ = range_image;
  if (indices != NULL)
  {
    IndicesPtr indicesptr (new std::vector<int> (*indices));
    setIndices (indicesptr);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
NarfDescriptor::computeFeature(NarfDescriptor::PointCloudOut& output)
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
  output.points.clear();
  
  if (range_image_==NULL)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImage is not set. Sorry, the NARF descriptor calculation works on range images, not on normal point clouds."
              << " Use setRangeImage(...).\n\n";
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (parameters_.support_size <= 0.0f)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": support size is not set. Use getParameters().support_size = ...\n\n";
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  std::vector<Narf*> feature_list;
  if (indices_)
  {
    for (size_t indices_idx=0; indices_idx<indices_->size(); ++indices_idx)
    {
      int point_index = (*indices_)[indices_idx];
      int y=point_index/range_image_->width, x=point_index - y*range_image_->width;
      Narf::extractFromRangeImageAndAddToList(*range_image_, static_cast<float> (x), static_cast<float> (y), 36, parameters_.support_size,
                                              parameters_.rotation_invariant, feature_list);
    }
  }
  else
  {
    for (unsigned int y=0; y<range_image_->height; ++y)
    {
      for (unsigned int x=0; x<range_image_->width; ++x)
      {
        Narf::extractFromRangeImageAndAddToList(*range_image_, static_cast<float> (x), static_cast<float> (y), 36, parameters_.support_size,
                                                parameters_.rotation_invariant, feature_list);
      }
    }
  }
  
  // Copy to NARF36 struct
  output.points.resize(feature_list.size());
  for (unsigned int i=0; i<feature_list.size(); ++i)
  {
    feature_list[i]->copyToNarf36(output.points[i]);
  }
  
  // Cleanup
  for (size_t i=0; i<feature_list.size(); ++i)
    delete feature_list[i];
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
NarfDescriptor::compute(NarfDescriptor::PointCloudOut& output)
{
  computeFeature(output);
}
}  // namespace end


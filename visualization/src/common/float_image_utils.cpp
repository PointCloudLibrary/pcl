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

#include <pcl/pcl_config.h>

#include <pcl/visualization/common/float_image_utils.h>
#include <cmath>
#include <algorithm>
#include <pcl/pcl_macros.h>

using std::cout;
using std::cerr;

void 
pcl::visualization::FloatImageUtils::getColorForFloat (float value, unsigned char& r, unsigned char& g, unsigned char& b) 
{
  if (std::isinf (value)) 
  {
    if (value > 0.0f) 
    {
      r = 150;  g = 150;  b = 200;  // INFINITY
      return;
    }
    r = 150;  g = 200;  b = 150;  // -INFINITY
    return;
  }
  if (!std::isfinite (value)) 
  {
    r = 200;  g = 150;  b = 150;  // -INFINITY
    return;
  }
  
  r = g = b = 0;
  value *= 10;
  if (value <= 1.0) 
  {  // black -> purple
    b = static_cast<unsigned char> (pcl_lrint(value*200));
    r = static_cast<unsigned char> (pcl_lrint(value*120));
  }
  else if (value <= 2.0) 
  {  // purple -> blue
    b = static_cast<unsigned char> (200 + pcl_lrint((value-1.0)*55));
    r = static_cast<unsigned char> (120 - pcl_lrint((value-1.0)*120));
  }
  else if (value <= 3.0) 
  {  // blue -> turquoise
    b = static_cast<unsigned char> (255 - pcl_lrint((value-2.0)*55));
    g = static_cast<unsigned char> (pcl_lrint((value-2.0)*200));
  }
  else if (value <= 4.0) 
  {  // turquoise -> green
    b = static_cast<unsigned char> (200 - pcl_lrint((value-3.0)*200));
    g = static_cast<unsigned char> (200 + pcl_lrint((value-3.0)*55));
  }
  else if (value <= 5.0) 
  {  // green -> greyish green
    g = static_cast<unsigned char> (255 - pcl_lrint((value-4.0)*100));
    r = static_cast<unsigned char> (pcl_lrint((value-4.0)*120));
  }
  else if (value <= 6.0) 
  { // greyish green -> red
    r = static_cast<unsigned char> (100 + pcl_lrint((value-5.0)*155));
    g = static_cast<unsigned char> (120 - pcl_lrint((value-5.0)*120));
    b = static_cast<unsigned char> (120 - pcl_lrint((value-5.0)*120));
  }
  else if (value <= 7.0) 
  {  // red -> yellow
    r = 255;
    g = static_cast<unsigned char> (pcl_lrint((value-6.0)*255));
  }
  else 
  {  // yellow -> white
    r = 255;
    g = 255;
    b = static_cast<unsigned char> (pcl_lrint((value-7.0)*255.0/3.0));
  }
}

void 
pcl::visualization::FloatImageUtils::getColorForAngle (float value, unsigned char& r, unsigned char& g, unsigned char& b) 
{
  if (std::isinf (value)) 
  {
    if (value > 0.0f) 
    {
      r = 150;  g = 150;  b = 200;  // INFINITY
      return;
    }
    r = 150;  g = 200;  b = 150;  // -INFINITY
    return;
  }
  if (!std::isfinite (value)) 
  {
    r = 200;  g = 150;  b = 150;  // -INFINITY
    return;
  }
  
  r = g = b = 0;
  if (value < -M_PI/2.0f) 
  {  // black -> blue
    b = static_cast<unsigned char> (pcl_lrint(255*(value+float(M_PI))/(float(M_PI)/2.0f)));
  }
  else if (value <= 0.0f) 
  {  // blue -> white
    b = 255;
    r = g = static_cast<unsigned char> (pcl_lrint(255*(value+float(M_PI/2))/(float(M_PI)/2.0f)));
  }
  else if (value <= M_PI/2.0f) 
  {  // white -> green
    g = 255;
    r = b = static_cast<unsigned char> (255-pcl_lrint(255*(value)/(float(M_PI)/2.0f)));
  }
  else 
  {  // green -> black
    g = static_cast<unsigned char> (255-pcl_lrint(255*(value-M_PI/2.0f)/(float(M_PI)/2.0f)));
  }
  //std::cout << 180.0f*value/M_PI<<"deg => "<<(int)r<<", "<<(int)g<<", "<<(int)b<<"\n";
}

void 
pcl::visualization::FloatImageUtils::getColorForHalfAngle (float value, unsigned char& r, unsigned char& g, unsigned char& b) 
{
  getColorForAngle(2.0f*value, r, g, b);
}

unsigned char* 
pcl::visualization::FloatImageUtils::getVisualImage (const float* float_image, int width, int height, float min_value, float max_value, bool gray_scale) 
{
  //MEASURE_FUNCTION_TIME;
  
  //std::cout << "Image is of size "<<width<<"x"<<height<<"\n";
  int size = width*height;
  int arraySize = 3 * size;
  auto* data = new unsigned char[arraySize];
  unsigned char* dataPtr = data;
  
  bool recalculateMinValue = std::isinf (min_value),
       recalculateMaxValue = std::isinf (max_value);
  if (recalculateMinValue) min_value = std::numeric_limits<float>::infinity ();
  if (recalculateMaxValue) max_value = -std::numeric_limits<float>::infinity ();
  
  if (recalculateMinValue || recalculateMaxValue) 
  {
    for (int i=0; i<size; ++i) 
    {
      float value = float_image[i];
      if (!std::isfinite(value)) continue;
      if (recalculateMinValue)  min_value = (std::min)(min_value, value);
      if (recalculateMaxValue)  max_value = (std::max)(max_value, value);
    }
  }
  //std::cout << "min_value is "<<min_value<<" and max_value is "<<max_value<<".\n";
  float factor = 1.0f / (max_value-min_value), offset = -min_value;
  
  for (int i=0; i<size; ++i) 
  {
    unsigned char& r=*(dataPtr++), & g=*(dataPtr++), & b=*(dataPtr++);
    float value = float_image[i];
    
    if (!std::isfinite(value)) 
    {
      getColorForFloat(value, r, g, b);
      continue;
    }
    
    // Normalize value to [0, 1]
    value = std::max (0.0f, std::min (1.0f, factor * (value + offset)));
    
    // Get a color from the value in [0, 1]
    if (gray_scale) 
    {
      r = g = b = static_cast<unsigned char> (pcl_lrint (value * 255));
    }
    else 
    {
      getColorForFloat(value, r, g, b);
    }
    //std::cout << "Setting pixel "<<i<<" to "<<(int)r<<", "<<(int)g<<", "<<(int)b<<".\n";
  }
  
  return data;
}

unsigned char* 
pcl::visualization::FloatImageUtils::getVisualImage (const unsigned short* short_image, int width, int height,
                                                     unsigned short min_value, unsigned short max_value,
                                                     bool gray_scale)
{
  //MEASURE_FUNCTION_TIME;
  
  //std::cout << "Image is of size "<<width<<"x"<<height<<"\n";
  int size = width*height;
  int arraySize = 3 * size;
  auto* data = new unsigned char[arraySize];
  unsigned char* dataPtr = data;
  
  float factor = 1.0f / float (max_value - min_value), offset = float (-min_value);
  
  for (int i=0; i<size; ++i) 
  {
    unsigned char& r=*(dataPtr++), & g=*(dataPtr++), & b=*(dataPtr++);
    float value = short_image[i];
    
    // Normalize value to [0, 1]
    value = std::max (0.0f, std::min (1.0f, factor * (value + offset)));
    
    // Get a color from the value in [0, 1]
    if (gray_scale) 
    {
      r = g = b = static_cast<unsigned char> (pcl_lrint(value*255));
    }
    else 
    {
      getColorForFloat(value, r, g, b);
    }
    //std::cout << "Setting pixel "<<i<<" to "<<(int)r<<", "<<(int)g<<", "<<(int)b<<".\n";
  }
  
  return data;
}

unsigned char* 
pcl::visualization::FloatImageUtils::getVisualAngleImage (const float* angle_image, int width, int height) 
{
  int size = width*height;
  int arraySize = 3 * size;
  auto* data = new unsigned char[arraySize];
  unsigned char* dataPtr = data;
  
  for (int i=0; i<size; ++i) 
  {
    unsigned char& r=*(dataPtr++), & g=*(dataPtr++), & b=*(dataPtr++);
    float angle = angle_image[i];
    
    getColorForAngle(angle, r, g, b);
  }
  
  return data;
}

unsigned char* 
pcl::visualization::FloatImageUtils::getVisualHalfAngleImage (const float* angle_image, int width, int height) 
{
  int size = width*height;
  int arraySize = 3 * size;
  auto* data = new unsigned char[arraySize];
  unsigned char* dataPtr = data;
  
  for (int i=0; i<size; ++i) 
  {
    unsigned char& r=*(dataPtr++), & g=*(dataPtr++), & b=*(dataPtr++);
    float angle = angle_image[i];
    
    getColorForHalfAngle(angle, r, g, b);
  }
  
  return data;
}

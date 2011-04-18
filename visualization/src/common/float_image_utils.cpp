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

#include <pcl/visualization/common/float_image_utils.h>
#include <cmath>
#include <pcl/win32_macros.h>

using std::cout;
using std::cerr;
using std::min;
using std::max;


void 
pcl::visualization::FloatImageUtils::getColorForFloat (float value, unsigned char& r, unsigned char& g, unsigned char& b) 
{
  if (pcl_isinf (value)) 
  {
    if (value > 0.0f) 
    {
      r = 150;  g = 150;  b = 200;  // INFINITY
      return;
    }
    r = 150;  g = 200;  b = 150;  // -INFINITY
    return;
  }
  if (!pcl_isfinite (value)) 
  {
    r = 200;  g = 150;  b = 150;  // -INFINITY
    return;
  }
  
  r = g = b = 0;
  value *= 10;
  if (value <= 1.0) 
  {  // black -> purple
    b = pcl_lrint(value*200);
    r = pcl_lrint(value*120);
  }
  else if (value <= 2.0) 
  {  // purple -> blue
    b = 200 + pcl_lrint((value-1.0)*55);
    r = 120 - pcl_lrint((value-1.0)*120);
  }
  else if (value <= 3.0) 
  {  // blue -> turquoise
    b = 255 - pcl_lrint((value-2.0)*55);
    g = pcl_lrint((value-2.0)*200);
  }
  else if (value <= 4.0) 
  {  // turquoise -> green
    b = 200 - pcl_lrint((value-3.0)*200);
    g = 200 + pcl_lrint((value-3.0)*55);
  }
  else if (value <= 5.0) 
  {  // green -> greyish green
    g = 255 - pcl_lrint((value-4.0)*100);
    r = pcl_lrint((value-4.0)*120);
  }
  else if (value <= 6.0) 
  { // greyish green -> red
    r = 100 + pcl_lrint((value-5.0)*155);
    g = 120 - pcl_lrint((value-5.0)*120);
    b = 120 - pcl_lrint((value-5.0)*120);
  }
  else if (value <= 7.0) 
  {  // red -> yellow
    r = 255;
    g = pcl_lrint((value-6.0)*255);
  }
  else 
  {  // yellow -> white
    r = 255;
    g = 255;
    b = pcl_lrint((value-7.0)*255.0/3.0);
  }
}

void 
pcl::visualization::FloatImageUtils::getColorForAngle (float value, unsigned char& r, unsigned char& g, unsigned char& b) 
{
  if (pcl_isinf (value)) 
  {
    if (value > 0.0f) 
    {
      r = 150;  g = 150;  b = 200;  // INFINITY
      return;
    }
    r = 150;  g = 200;  b = 150;  // -INFINITY
    return;
  }
  if (!pcl_isfinite (value)) 
  {
    r = 200;  g = 150;  b = 150;  // -INFINITY
    return;
  }
  
  r = g = b = 0;
  if (value < -M_PI/2.0f) 
  {  // black -> blue
    b = (pcl_lrint(255*(value+float(M_PI))/(float(M_PI)/2.0f)));
  }
  else if (value <= 0.0f) 
  {  // blue -> white
    b = 255;
    r = g = (pcl_lrint(255*(value+float(M_PI/2))/(float(M_PI)/2.0f)));
  }
  else if (value <= M_PI/2.0f) 
  {  // white -> green
    g = 255;
    r = b = (255-pcl_lrint(255*(value)/(float(M_PI)/2.0f)));
  }
  else 
  {  // green -> black
    g = (255-pcl_lrint(255*(value-M_PI/2.0f)/(float(M_PI)/2.0f)));
  }
  //cout << 180.0f*value/M_PI<<"deg => "<<(int)r<<", "<<(int)g<<", "<<(int)b<<"\n";
}

void 
pcl::visualization::FloatImageUtils::getColorForHalfAngle (float value, unsigned char& r, unsigned char& g, unsigned char& b) 
{
  getColorForAngle(2.0f*value, r, g, b);
}



unsigned char* 
pcl::visualization::FloatImageUtils::getVisualImage (const float* floatImage, int width, int height, float minValue, float maxValue, bool grayScale) 
{
  //MEASURE_FUNCTION_TIME;
  
  //cout << "Image is of size "<<width<<"x"<<height<<"\n";
  int size = width*height;
  int arraySize = 3 * size;
  unsigned char* data = new unsigned char[arraySize];
  unsigned char* dataPtr = data;
  
  bool recalculateMinValue = pcl_isinf (minValue),
       recalculateMaxValue = pcl_isinf (maxValue);
  if (recalculateMinValue) minValue = std::numeric_limits<float>::infinity ();
  if (recalculateMaxValue) maxValue = -std::numeric_limits<float>::infinity ();
  
  if (recalculateMinValue || recalculateMaxValue) 
  {
    for (int i=0; i<size; ++i) 
    {
      float value = floatImage[i];
      if (!pcl_isfinite(value)) continue;
      if (recalculateMinValue)  minValue=min(minValue, value);
      if (recalculateMaxValue)  maxValue=max(maxValue, value);
    }
  }
  //cout << "minValue is "<<minValue<<" and maxValue is "<<maxValue<<".\n";
  float factor = 1.0 / (maxValue-minValue), offset = -minValue;
  
  for (int i=0; i<size; ++i) 
  {
    unsigned char& r=*(dataPtr++), & g=*(dataPtr++), & b=*(dataPtr++);
    float value = floatImage[i];
    
    if (!pcl_isfinite(value)) 
    {
      getColorForFloat(value, r, g, b);
      continue;
    }
    
    // Normalize value to [0, 1]
    value = max(0.0f, min(1.0f, factor*(value + offset)));
    
    // Get a color from the value in [0, 1]
    if (grayScale) 
    {
      r = g = b = pcl_lrint(value*255);
    }
    else 
    {
      getColorForFloat(value, r, g, b);
    }
    //cout << "Setting pixel "<<i<<" to "<<(int)r<<", "<<(int)g<<", "<<(int)b<<".\n";
  }
  
  return data;
}

unsigned char* 
pcl::visualization::FloatImageUtils::getVisualAngleImage (const float* angle_image, int width, int height) 
{
  int size = width*height;
  int arraySize = 3 * size;
  unsigned char* data = new unsigned char[arraySize];
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
  unsigned char* data = new unsigned char[arraySize];
  unsigned char* dataPtr = data;
  
  for (int i=0; i<size; ++i) 
  {
    unsigned char& r=*(dataPtr++), & g=*(dataPtr++), & b=*(dataPtr++);
    float angle = angle_image[i];
    
    getColorForHalfAngle(angle, r, g, b);
  }
  
  return data;
}


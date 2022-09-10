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

#include <pcl/visualization/range_image_visualizer.h>

pcl::visualization::RangeImageVisualizer::RangeImageVisualizer (const std::string& name) : ImageViewer (name)
{
}

pcl::visualization::RangeImageVisualizer::~RangeImageVisualizer () = default;

// void 
// pcl::visualization::RangeImageVisualizer::setRangeImage (const pcl::RangeImage& range_image, float min_value, float max_value, bool grayscale)
// {
//   float* ranges = range_image.getRangesArray ();
//   setFloatImage(ranges, range_image.width, range_image.height, min_value, max_value, grayscale);
  
//   delete[] ranges;
// }

void 
pcl::visualization::RangeImageVisualizer::showRangeImage (const pcl::RangeImage& range_image, float min_value, float max_value, bool grayscale)
{
  float* ranges = range_image.getRangesArray ();
  showFloatImage(ranges, range_image.width, range_image.height, min_value, max_value, grayscale);
  
  delete[] ranges;
}

pcl::visualization::RangeImageVisualizer* 
pcl::visualization::RangeImageVisualizer::getRangeImageWidget (
    const pcl::RangeImage& range_image, float min_value, float max_value, bool grayscale, const std::string& name)
{
  auto* range_image_widget = new RangeImageVisualizer (name);
  range_image_widget->showRangeImage (range_image, min_value, max_value, grayscale);
  return range_image_widget;
}

void 
pcl::visualization::RangeImageVisualizer::visualizeBorders (
    const pcl::RangeImage& range_image, float min_value, float max_value, bool grayscale,
    const pcl::PointCloud<pcl::BorderDescription>& border_descriptions)
{  
  showRangeImage(range_image, min_value, max_value, grayscale);
  for (std::size_t y=0; y<range_image.height; ++y)
  {
    for (std::size_t x=0; x<range_image.width; ++x)
    {
      const pcl::BorderDescription& border_description = border_descriptions[y*range_image.width + x];
      const pcl::BorderTraits& border_traits = border_description.traits;
      if (border_traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
      {
        markPoint (x, y, green_color);
        //for (unsigned int i = 0; i < border_description.neighbors.size(); ++i)
          //range_image_widget->markLine (border_description.x, border_description.y,
                                        //border_description.neighbors[i]->x, border_description.neighbors[i]->y, wxGREEN_PEN);
      }
      else if (border_traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        markPoint(x, y, blue_color);
      else if (border_traits[pcl::BORDER_TRAIT__VEIL_POINT])
        markPoint(x, y, red_color);
    }
  }
}

pcl::visualization::RangeImageVisualizer* 
pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (
    const pcl::RangeImage& range_image, float min_value, float max_value, bool grayscale,
    const pcl::PointCloud<pcl::BorderDescription>& border_descriptions, const std::string& name)
{
  //std::cout << PVARN(range_image)<<PVARN(min_value)<<PVARN(max_value)<<PVARN(grayscale);
  auto* range_image_widget = new RangeImageVisualizer;
  range_image_widget->visualizeBorders (range_image, min_value, max_value, grayscale,
                                        border_descriptions);
  range_image_widget->setWindowTitle (name);
  return range_image_widget;
}

pcl::visualization::RangeImageVisualizer* 
pcl::visualization::RangeImageVisualizer::getAnglesWidget (const pcl::RangeImage& range_image, float* angles_image,
                                                           const std::string& name)
{
  auto* widget = new RangeImageVisualizer;
  widget->showAngleImage(angles_image, range_image.width, range_image.height);
  widget->setWindowTitle (name);
  return widget;
}

pcl::visualization::RangeImageVisualizer* 
pcl::visualization::RangeImageVisualizer::getHalfAnglesWidget (const pcl::RangeImage& range_image,
                                                                float* angles_image, const std::string& name)
{
  auto* widget = new RangeImageVisualizer;
  widget->showHalfAngleImage(angles_image, range_image.width, range_image.height);
  widget->setWindowTitle (name);
  return widget;
}

pcl::visualization::RangeImageVisualizer* 
pcl::visualization::RangeImageVisualizer::getInterestPointsWidget (
    const pcl::RangeImage& range_image, const float* interest_image, float min_value, float max_value, 
    const pcl::PointCloud<pcl::InterestPoint>& interest_points, const std::string& name)
{
  auto* widget = new RangeImageVisualizer;
  widget->showFloatImage (interest_image, range_image.width, range_image.height, min_value, max_value);
  widget->setWindowTitle (name);
  for (const auto &interest_point : interest_points.points)
  {
    float image_x, image_y;
    range_image.getImagePoint (interest_point.x, interest_point.y, interest_point.z, image_x, image_y);
    widget->markPoint (static_cast<std::size_t> (image_x), static_cast<std::size_t> (image_y), green_color, red_color);
    //std::cout << "Marking point "<<image_x<<","<<image_y<<"\n";
  }
  return widget;
}

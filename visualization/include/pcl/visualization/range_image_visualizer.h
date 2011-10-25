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
 *
 */

#include <pcl/pcl_config.h>

#ifndef PCL_VISUALIZATION_RANGE_IMAGE_VISUALIZER_H_
#define PCL_VISUALIZATION_RANGE_IMAGE_VISUALIZER_H_

// PCL includes
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/image_viewer.h>

namespace pcl
{
  namespace visualization
  {
    /** \brief Range image visualizer class.
      * \author Bastian Steder
      * \ingroup visualization
      */
    class PCL_EXPORTS RangeImageVisualizer : public ImageViewer
    {
      public:
        // =====CONSTRUCTOR & DESTRUCTOR=====
        //! Constructor
        RangeImageVisualizer (const std::string& name="Range Image");
        //! Destructor
        ~RangeImageVisualizer ();
        
        // =====PUBLIC STATIC METHODS=====
        /** Get a widget visualizing the given range image.
         *  You are responsible for deleting it after usage! */
        static RangeImageVisualizer* getRangeImageWidget (const pcl::RangeImage& range_image, float min_value,
                                                  float max_value, bool grayscale, const std::string& name="Range image");
        
        /** Visualize the given range image and the detected borders in it.
         *  Borders on the obstacles are marked green, borders on the background are marked bright blue. */
        void visualizeBorders (const pcl::RangeImage& range_image, float min_value, float max_value, bool grayscale,
                               const pcl::PointCloud<pcl::BorderDescription>& border_descriptions);
        
        /** Same as above, but returning a new widget. You are responsible for deleting it after usage! */
        static RangeImageVisualizer* getRangeImageBordersWidget (const pcl::RangeImage& range_image, float min_value,
                      float max_value, bool grayscale, const pcl::PointCloud<pcl::BorderDescription>& border_descriptions,
                      const std::string& name="Range image with borders");
        
        /** Get a widget visualizing the given angle image (assuming values in (-PI, PI]).
         *  -PI and PI will return the same color
         *  You are responsible for deleting it after usage! */
        static RangeImageVisualizer* getAnglesWidget (const pcl::RangeImage& range_image, float* angles_image, const std::string& name);
        
        /** Get a widget visualizing the given angle image (assuming values in (-PI/2, PI/2]).
         *  -PI/2 and PI/2 will return the same color
         *  You are responsible for deleting it after usage! */
        static RangeImageVisualizer* getHalfAnglesWidget (const pcl::RangeImage& range_image, float* angles_image, const std::string& name);

        
        /** Get a widget visualizing the interest values and extracted interest points.
         * The interest points will be marked green.
         *  You are responsible for deleting it after usage! */
        static RangeImageVisualizer* getInterestPointsWidget (const pcl::RangeImage& range_image, const float* interest_image, float min_value, float max_value,
                                                              const pcl::PointCloud<pcl::InterestPoint>& interest_points, const std::string& name);

        // =====PUBLIC METHODS=====
        //! Visualize a range image
        /* void  */
        /* setRangeImage (const pcl::RangeImage& range_image,  */
        /*                float min_value = -std::numeric_limits<float>::infinity (),  */
        /*                float max_value =  std::numeric_limits<float>::infinity (),  */
        /*                bool grayscale  = false); */

        void 
        showRangeImage (const pcl::RangeImage& range_image, 
                       float min_value = -std::numeric_limits<float>::infinity (), 
                       float max_value =  std::numeric_limits<float>::infinity (), 
                       bool grayscale  = false);
        
      protected:
        // =====PROTECTED MEMBER VARIABLES=====
        std::string name_;
    };
  }
}

#endif  //#define PCL_VISUALIZATION_RANGE_IMAGE_VISUALIZER_H_

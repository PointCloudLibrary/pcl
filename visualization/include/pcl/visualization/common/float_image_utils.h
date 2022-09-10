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

#pragma once

#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>

#include <limits>

namespace pcl
{
  namespace visualization
  {
    /** @b Provide some gerneral functionalities regarding 2d float arrays, e.g., for visualization purposes
      * \author Bastian Steder
      * \ingroup visualization
      */
    class PCL_EXPORTS FloatImageUtils
    {
      public:
        // =====STATIC METHODS=====
        /** Get RGB color values for a given float in [0, 1] or special cases like -INFINITY(light green), INFINITY(light blue), NAN(light red) */
        static void 
        getColorForFloat (float value, unsigned char& r, unsigned char& g, unsigned char& b);
        
        /** Get RGB color values for a given float in [-PI, PI] or special cases like -INFINITY(light green), INFINITY(light blue), NAN(light red)
          * The colors are black(-PI) -> blue(-PI/2) -> white(0) -> green(PI/2) -> black(PI) with accordant values in between */
        static void 
        getColorForAngle (float value, unsigned char& r, unsigned char& g, unsigned char& b);
        
        /** Get RGB color values for a given float in [0, PI] or special cases like -INFINITY(light green), INFINITY(light blue), NAN(light red)
          * The colors are black(-PI/2) -> blue(-PI/4) -> white(0) -> green(PI/4) -> black(PI/2) with accordant values in between */
        static void 
        getColorForHalfAngle (float value, unsigned char& r, unsigned char& g, unsigned char& b);
        
        /** Use getColorForFloat for all elements of the given arrays, whereas the values are first normalized to [0,1],
         * either using the given min/max values or based on the actual minimal and maximal values existing in the array.
         * The output is a byte array of size 3*width*height containing the colors in RGB order.
         * If gray_scale is true, the outcome will still be an RGB image, but all colors apart for the special colors of
         * non-finite numbers, will be gray values */
        static unsigned char* 
        getVisualImage (const float* float_image, int width, int height, float min_value=-std::numeric_limits<float>::infinity (), float max_value=std::numeric_limits<float>::infinity (), bool gray_scale=false);
        
        /** Use getColorForFloat for all elements of the given arrays, whereas the values are normalized to [0,1]
         *  with the given min/max values.
         *  The output is a byte array of size 3*width*height containing the colors in RGB order.
         *  If gray_scale is true, the outcome will still be an RGB image, but all colors will be gray values. */
        static unsigned char* 
        getVisualImage (const unsigned short* float_image, int width, int height,
                        unsigned short min_value=0,
                        unsigned short max_value=std::numeric_limits<unsigned short>::max (),
                        bool gray_scale=false);
        
        /** Use getColorForAngle for all elements of the given arrays. */
        static unsigned char* 
        getVisualAngleImage (const float* angle_image, int width, int height);

        /** Use getColorForHalfAngle for all elements of the given arrays. */
        static unsigned char* 
        getVisualHalfAngleImage (const float* angle_image, int width, int height);
    };

  }  // namespace end
}

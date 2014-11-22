/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */

#ifndef PCL_COMMON_COLORS_H
#define PCL_COMMON_COLORS_H

#include <pcl/pcl_macros.h>

namespace pcl
{

  struct RGB;

  PCL_EXPORTS RGB
  getRandomColor (double min = 0.2, double max = 2.8);

  /** Color lookup table consisting of 256 colors structured in a maximally
    * discontinuous manner. Generated using the method of Glasbey et al.
    * (see https://github.com/taketwo/glasbey) */
  class PCL_EXPORTS GlasbeyLUT
  {

    public:

      /** Get a color from the lookup table with a given id.
        *
        * The id should be less than the size of the LUT (see size()). */
      static RGB at (unsigned int color_id);

      /** Get the number of colors in the lookup table.
        *
        * Note: the number of colors is different from the number of elements
        * in the lookup table (each color is defined by three bytes). */
      static size_t size ();

      /** Get a raw pointer to the lookup table. */
      static const unsigned char* data ();

  };

}

#endif /* PCL_COMMON_COLORS_H */


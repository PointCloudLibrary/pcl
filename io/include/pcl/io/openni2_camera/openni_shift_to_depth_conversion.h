/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011 Willow Garage, Inc.
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
 */
#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI2

#ifndef __OPENNI_SHIFT_TO_DEPTH_CONVERSION
#define __OPENNI_SHIFT_TO_DEPTH_CONVERSION

#include <vector>
#include <limits>

namespace openni_wrapper
{
  /** \brief This class provides conversion of the openni 11-bit shift data to depth;
    */
  class PCL_EXPORTS ShiftToDepthConverter
  {
    public:
      /** \brief Constructor. */
      ShiftToDepthConverter () : init_(false) {}

      /** \brief Destructor. */
      virtual ~ShiftToDepthConverter () {};

      /** \brief This method generates a look-up table to convert openni shift values to depth
        */
      void
      generateLookupTable ()
      {
        // lookup of 11 bit shift values
        const std::size_t table_size = 1<<10;

        lookupTable_.clear();
        lookupTable_.resize(table_size);

        // constants taken from openni driver
        static const int16_t nConstShift = 800;
        static const double nParamCoeff = 4.000000;
        static const double dPlanePixelSize = 0.104200;
        static const double nShiftScale = 10.000000;
        static const double dPlaneDsr = 120.000000;
        static const double dPlaneDcl = 7.500000;

        std::size_t i;
        double dFixedRefX;
        double dMetric;

        for (i=0; i<table_size; ++i)
        {
          // shift to depth calculation from opnni
          dFixedRefX = (static_cast<double>(i - nConstShift) / nParamCoeff)-0.375;
          dMetric = dFixedRefX * dPlanePixelSize;
          lookupTable_[i] = static_cast<float>((nShiftScale * ((dMetric * dPlaneDsr / (dPlaneDcl - dMetric)) + dPlaneDsr) ) / 1000.0f);
        }

        init_ = true;
      }

      /** \brief Generate a look-up table for converting openni shift values to depth
         */
      inline float
      shiftToDepth (uint16_t shift_val)
      {
        assert (init_);

        static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

        float ret = bad_point;

        // lookup depth value in shift lookup table
        if (shift_val<lookupTable_.size())
          ret = lookupTable_[shift_val];

        return ret;
      }

      inline bool isInitialized() const
      {
        return init_;
      }

    protected:
      std::vector<float> lookupTable_;
      bool init_;
  } ;
}

#endif
#endif //__OPENNI_SHIFT_TO_DEPTH_CONVERSION

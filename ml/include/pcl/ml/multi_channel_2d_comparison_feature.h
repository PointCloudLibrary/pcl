/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
  
#ifndef PCL_ML_MULTI_CHANNEL_2D_COMPARISON_FEATURE_H_
#define PCL_ML_MULTI_CHANNEL_2D_COMPARISON_FEATURE_H_

#include <pcl/common/common.h>

#include <istream>
#include <ostream>

namespace pcl
{

  /** \brief Feature for comparing two sample points in 2D multi-channel data. */
  template <class PointT>
  class PCL_EXPORTS MultiChannel2DComparisonFeature
  {
  public:
      /** \brief Constructor. */
      MultiChannel2DComparisonFeature () : p1 (), p2 (), channel (0) {}
      /** \brief Destructor. */
      virtual ~MultiChannel2DComparisonFeature () {}

      /** \brief Serializes the feature to a stream.
        * \param[out] stream The destination for the serialization.
        */
      inline void 
      serialize (std::ostream & stream) const
      {
        p1.serialize (stream);
        p2.serialize (stream);
        stream.write (reinterpret_cast<const char*> (&channel), sizeof (channel));
      }

      /** \brief Deserializes the feature from a stream. 
        * \param[in] stream The source for the deserialization.
        */
      inline void 
      deserialize (std::istream & stream)
      {
        p1.deserialize (stream);
        p2.deserialize (stream);
        stream.read (reinterpret_cast<char*> (&channel), sizeof (channel));
      }

    public:
      /** \brief First sample point. */
      PointT p1;
      /** \brief Second sample point. */
      PointT p2;

      /** \brief Specifies which channel is used for comparison. */
      unsigned char channel;
  };

}

#endif

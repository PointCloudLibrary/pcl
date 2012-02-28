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

#ifndef PCL_FEATURES_COLOR_GRADIENT_MODALITY
#define PCL_FEATURES_COLOR_GRADIENT_MODALITY

#include "pcl/recognition/quantizable_modality.h"

#include "pcl/pcl_base.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"


namespace pcl
{

  struct EIGEN_ALIGN16 PointRGB
  {
    union
    {
      union
      {
        struct
        {
          uint8_t b;
          uint8_t g;
          uint8_t r;
          uint8_t _unused;
        };
        float rgb;
      };
      uint32_t rgba;
    };

    inline PointRGB ()
    {}

    inline PointRGB (const uint8_t b, const uint8_t g, const uint8_t r)
      : b (b), g (g), r (r), _unused (0)
    {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


  /** \brief A point structure representing Euclidean xyz coordinates, and the intensity value.
    * \ingroup common
    */
  struct EIGEN_ALIGN16 GradientXY
  {
    union
    {
      struct
      {
        float x;
        float y;
        float angle;
        float magnitude;
      };
      float data[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		inline bool operator< (const GradientXY & rhs)
		{
			return magnitude > rhs.magnitude;
		}
  };
  inline std::ostream& operator << (std::ostream& os, const GradientXY& p)
  {
    os << "(" << p.x << "," << p.y << " - " << p.magnitude << ")";
    return (os);
  }

  // --------------------------------------------------------------------------

  template <typename PointInT>
  class ColorGradientModality
    : public QuantizableModality, public PCLBase<PointInT>
  {
    
  public:

    typedef typename pcl::PointCloud<PointInT> PointCloudIn;

    ColorGradientModality ();

    virtual ~ColorGradientModality ();

    inline void
    setGradientMagnitudeThreshold (const float threshold)
    {
      gradient_magnitude_threshold_ = threshold;
    }

    inline QuantizedMap &
    getQuantizedMap () 
    { 
      return filtered_quantized_color_gradients_; 
    }

    inline QuantizedMap &
    getSpreadedQuantizedMap () 
    { 
      return spreaded_filtered_quantized_color_gradients_; 
    }

    void 
    extractFeatures (
      MaskMap & mask,
      const int numOfFeatures,
      const int modalityIndex,
      std::vector< QuantizedMultiModFeature > & features );

    /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
      * \param cloud the const boost shared pointer to a PointCloud message
      */
    virtual void 
    setInputCloud (const typename PointCloudIn::ConstPtr &cloud) 
    { 
      input_ = cloud; 
    }

    virtual void
    processInputData ();

  protected:

    void
    computeMaxColorGradients ();

    void
    quantizeColorGradients ();

    void
    filterQuantizedColorGradients ();

  private:

    float gradient_magnitude_threshold_;
    pcl::PointCloud< pcl::GradientXY > color_gradients_;

    pcl::QuantizedMap quantized_color_gradients_;
    pcl::QuantizedMap filtered_quantized_color_gradients_;
    pcl::QuantizedMap spreaded_filtered_quantized_color_gradients_;

  };

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::ColorGradientModality<PointInT>::
ColorGradientModality ()
: gradient_magnitude_threshold_ (80.0f)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::ColorGradientModality<PointInT>::
~ColorGradientModality ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
processInputData ()
{
  // extract color gradients
  computeMaxColorGradients ();

  // quantize gradients
  quantizeColorGradients ();

  // filter quantized gradients to get only dominants one + thresholding
  filterQuantizedColorGradients ();

  // spread filtered quantized gradients
  //spreadFilteredQunatizedColorGradients ();
  const int spreadingSize = 8;
  pcl::QuantizedMap::spreadQuantizedMap (
    filtered_quantized_color_gradients_,
    spreaded_filtered_quantized_color_gradients_,
    spreadingSize );
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
extractFeatures (
  MaskMap & mask,
  const int numOfFeatures,
  const int modalityIndex,
  std::vector< QuantizedMultiModFeature > & features )
{
  const int width = mask.getWidth ();
  const int height = mask.getHeight ();

	struct Candidate
	{
		GradientXY gradient;

		int x;
		int y;	

		bool operator< (const Candidate & rhs)
		{
			return gradient.magnitude > rhs.gradient.magnitude;
		}
	};

	std::list<Candidate> list1;
	std::list<Candidate> list2;


  for (int rowIndex = 0; rowIndex < height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < width; ++colIndex)
    {
      if (mask (colIndex, rowIndex) != 0)
      {
        GradientXY gradient = color_gradients_ (colIndex, rowIndex);
        if (gradient.magnitude > gradient_magnitude_threshold_)
        {
          Candidate candidate;
          candidate.gradient = gradient;
          candidate.x = colIndex;
          candidate.y = rowIndex;

          list1.push_back (candidate);
        }
      }
    }
  }

  list1.sort();

  if (list1.size () <= numOfFeatures)
  {
    for (std::list<Candidate>::iterator iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
    {
      QuantizedMultiModFeature feature;
          
      feature.x = iter1->x;
      feature.y = iter1->y;
      feature.modalityIndex = modalityIndex;
      feature.quantizedValue = filtered_quantized_color_gradients_ (iter1->x, iter1->y);

      features.push_back (feature);
    }
    return;
  }

  int distance = list1.size () / numOfFeatures + 1; // ??? 
  while (list2.size () != numOfFeatures)
  {
    const int sqrDistance = distance*distance;
    for (std::list<Candidate>::iterator iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
    {
      bool candidateAccepted = true;

      for (std::list<Candidate>::iterator iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
      {
        const float dx = iter1->x - iter2->x;
        const float dy = iter1->y - iter2->y;
        const float tmpDistance = dx*dx + dy*dy;

        if (tmpDistance < distance)
        {
          candidateAccepted = false;
          break;
        }
      }

      if (candidateAccepted)
      {
        list2.push_back (*iter1);
      }

      if (list2.size () == numOfFeatures) break;
    }
    --distance;
  }

  for (std::list<Candidate>::iterator iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
  {
    QuantizedMultiModFeature feature;
          
    feature.x = iter2->x;
    feature.y = iter2->y;
    feature.modalityIndex = modalityIndex;
    feature.quantizedValue = filtered_quantized_color_gradients_ (iter2->x, iter2->y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
computeMaxColorGradients ()
{
  const int width = input_->width;
  const int height = input_->height;

  color_gradients_.points.resize (width*height);
  color_gradients_.width = width;
  color_gradients_.height = height;

  for (int rowIndex = 0; rowIndex < height-2; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < width-2; ++colIndex)
    {
      const int index0 = rowIndex*width+colIndex;
      const int indexC = rowIndex*width+colIndex+2;
      const int indexR = (rowIndex+2)*width+colIndex;

      //const int indexD = (rowIndex+1)*width+colIndex+1;

      const unsigned char r0 = input_->points[index0].r;
      const unsigned char g0 = input_->points[index0].g;
      const unsigned char b0 = input_->points[index0].b;

      const unsigned char rC = input_->points[indexC].r;
      const unsigned char gC = input_->points[indexC].g;
      const unsigned char bC = input_->points[indexC].b;

      const unsigned char rR = input_->points[indexR].r;
      const unsigned char gR = input_->points[indexR].g;
      const unsigned char bR = input_->points[indexR].b;

      const float rDx = static_cast<float> (rC) - static_cast<float> (r0);
      const float gDx = static_cast<float> (gC) - static_cast<float> (g0);
      const float bDx = static_cast<float> (bC) - static_cast<float> (b0);

      const float rDy = static_cast<float> (rR) - static_cast<float> (r0);
      const float gDy = static_cast<float> (gR) - static_cast<float> (g0);
      const float bDy = static_cast<float> (bR) - static_cast<float> (b0);

      const float sqrMagR = rDx*rDx + rDy*rDy;
      const float sqrMagG = gDx*gDx + gDy*gDy;
      const float sqrMagB = bDx*bDx + bDy*bDy;

      if (sqrMagR > sqrMagG && sqrMagR > sqrMagB)
      {
        GradientXY gradient;
        gradient.magnitude = sqrt(sqrMagR);
        gradient.angle = atan2(rDy, rDx)*180.0f/3.14f;
        gradient.x = colIndex;
        gradient.y = rowIndex;

        color_gradients_(colIndex+1, rowIndex+1) = gradient;
      }
      else if (sqrMagG > sqrMagB)
      {
        GradientXY gradient;
        gradient.magnitude = sqrt(sqrMagG);
        gradient.angle = atan2(gDy, gDx)*180.0f/3.14f;
        gradient.x = colIndex;
        gradient.y = rowIndex;

        color_gradients_(colIndex+1, rowIndex+1) = gradient;
      }
      else
      {
        GradientXY gradient;
        gradient.magnitude = sqrt(sqrMagB);
        gradient.angle = atan2(bDy, bDx)*180.0f/3.14f;
        gradient.x = colIndex;
        gradient.y = rowIndex;

        color_gradients_(colIndex+1, rowIndex+1) = gradient;
      }
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
quantizeColorGradients ()
{
  const int width = input_->width;
  const int height = input_->height;

  quantized_color_gradients_.initialize(width, height);

  //unsigned char quantizationMap[16] = {0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7};
  unsigned char quantizationMap[16] = {1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8};

  const float angleScale = 1.0f/22.6f;
  for (int rowIndex = 0; rowIndex < height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < width; ++colIndex)
    {
      if (color_gradients_ (colIndex, rowIndex).magnitude < gradient_magnitude_threshold_) 
      {
        quantized_color_gradients_ (colIndex, rowIndex) = 0;
        continue;
      }

      const int quantizedValue = static_cast<int> (color_gradients_ (colIndex, rowIndex).angle * angleScale);
      quantized_color_gradients_ (colIndex, rowIndex) = quantizationMap[quantizedValue];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
filterQuantizedColorGradients ()
{
  const int width = input_->width;
  const int height = input_->height;

  filtered_quantized_color_gradients_.initialize(width, height);

  // filter data
  for (int rowIndex = 1; rowIndex < height-1; ++rowIndex)
  {
    for (int colIndex = 1; colIndex < width-1; ++colIndex)
    {
      unsigned char histogram[9] = {0,0,0,0,0,0,0,0,0};

      {
        unsigned char * dataPtr = quantized_color_gradients_.getData () + (rowIndex-1)*width+colIndex-1;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
      }
      {
        unsigned char * dataPtr = quantized_color_gradients_.getData () + rowIndex*width+colIndex-1;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
      }
      {
        unsigned char * dataPtr = quantized_color_gradients_.getData () + (rowIndex+1)*width+colIndex-1;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
      }

      unsigned char maxHistValue = 0;
      int maxHistIndex = -1;

      if (maxHistValue < histogram[1]) {maxHistIndex = 0; maxHistValue = histogram[1];}
      if (maxHistValue < histogram[2]) {maxHistIndex = 1; maxHistValue = histogram[2];}
      if (maxHistValue < histogram[3]) {maxHistIndex = 2; maxHistValue = histogram[3];}
      if (maxHistValue < histogram[4]) {maxHistIndex = 3; maxHistValue = histogram[4];}
      if (maxHistValue < histogram[5]) {maxHistIndex = 4; maxHistValue = histogram[5];}
      if (maxHistValue < histogram[6]) {maxHistIndex = 5; maxHistValue = histogram[6];}
      if (maxHistValue < histogram[7]) {maxHistIndex = 6; maxHistValue = histogram[7];}
      if (maxHistValue < histogram[8]) {maxHistIndex = 7; maxHistValue = histogram[8];}

      if (maxHistIndex != -1 && maxHistValue >= 5)
      {
        filtered_quantized_color_gradients_ (colIndex, rowIndex) = 0x1 << maxHistIndex;
      }
      else
      {
        filtered_quantized_color_gradients_ (colIndex, rowIndex) = 0;
      }
    }
  }
}

#endif PCL_FEATURES_COLOR_GRADIENT_MODALITY

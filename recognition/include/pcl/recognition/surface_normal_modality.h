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

#ifndef PCL_FEATURES_SURFACE_NORMAL_MODALITY
#define PCL_FEATURES_SURFACE_NORMAL_MODALITY

#include "pcl/recognition/quantizable_modality.h"

#include "pcl/pcl_base.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"


namespace pcl
{

  struct DistanceMap
  {
    int width_;
    int height_;
    float * data_;

    DistanceMap () : width_ (-1), height_ (-1), data_ (NULL) {}
    virtual ~DistanceMap () {}

    inline int 
    getWidth () 
    {
      return (width_); 
    }

    inline int 
    getHeight () 
    { 
      return (height_); 
    }
    
    inline float* 
    getData () 
    { 
      return (data_); 
    }

    void 
    initialize (const int width, const int height)
    {
      if (data_ != NULL && (width != width_ || height != height_))
        release ();

      if (data_ == NULL)
      {
        data_ = new float[width*height];
        width_ = width;
        height_ = height;
      }

      memset (data_, 0, width*height);
    }

    void 
    release ()
    {
      if (data_ != NULL) 
        delete[] data_;

      data_ = NULL;
      width_ = -1;
      height_ = -1;
    }

    inline float& 
    operator() (const int colIndex, const int rowIndex)
    {
      return (data_[rowIndex*width_ + colIndex]);
    }
  };

  struct QuantizedNormalLookUpTable
  {
    int rangeX;
    int rangeY;
    int rangeZ;

    int offsetX;
    int offsetY;
    int offsetZ;

    int sizeX;
    int sizeY;
    int sizeZ;

    unsigned char * lut;

    QuantizedNormalLookUpTable () : 
      rangeX (-1), rangeY (-1), rangeZ (-1), 
      offsetX (-1), offsetY (-1), offsetZ (-1), 
      sizeX (-1), sizeY (-1), size (-1), lut (NULL) 
    {};

    //~QuantizedNormalLookUpTable() { if (lut != NULL) free16(lut); };
    ~QuantizedNormalLookUpTable () 
    { 
      if (lut != NULL) 
        delete[] lut; 
    };

    void 
    initializeLUT (const int rangeXArg, const int rangeYArg, const int rangeZArg)
    {
      // @todo: this should be range_x = range_x_arg;
      rangeX = rangeXArg;
      rangeY = rangeYArg;
      rangeZ = rangeZArg;
      sizeX = rangeXArg+1;
      sizeY = rangeYArg+1;
      sizeZ = rangeZArg+1;
      offsetX = rangeXArg/2;
      offsetY = rangeYArg/2;
      offsetZ = rangeZArg;

      //if (lut != NULL) free16(lut);
      //lut = malloc16(sizeX*sizeY*sizeZ);

      if (lut != NULL) 
        delete[] lut;
      lut = new unsigned char[sizeX*sizeY*sizeZ];

      const int numOfNormals = 8;
      // @todo: Needs to be changed
      cv::Point3f refNormals[numOfNormals];
      
      const float normal0Angle = 40.0f * 3.14f / 180.0f;
      refNormals[0].x = cos (normal0Angle);
      refNormals[0].y = 0.0f;
      refNormals[0].z = -sin (normal0Angle);

      const float invNumOfNormals = 1.0f/numOfNormals;
      for (int normalIndex = 1; normalIndex < numOfNormals; ++normalIndex)
      {
        const float angle = normalIndex * 3.14f * 2.0f * invNumOfNormals;

        refNormals[normalIndex].x = cos(angle)*refNormals[0].x - sin(angle)*refNormals[0].y;
        refNormals[normalIndex].y = sin(angle)*refNormals[0].x + cos(angle)*refNormals[0].y;
        refNormals[normalIndex].z = refNormals[0].z;
      }

      // normalize normals
      for (int normalIndex = 0; normalIndex < numOfNormals; ++normalIndex)
      {
        const float length = sqrt (refNormals[normalIndex].x * refNormals[normalIndex].x +
                                   refNormals[normalIndex].y * refNormals[normalIndex].y +
                                   refNormals[normalIndex].z * refNormals[normalIndex].z);

        const float invLength = 1.0f / length;

        refNormals[normalIndex].x *= invLength;
        refNormals[normalIndex].y *= invLength;
        refNormals[normalIndex].z *= invLength;
      }

      // set LUT
      for (int zIndex = 0; zIndex < sizeZ; ++zIndex)
      {
        for (int yIndex = 0; yIndex < sizeY; ++yIndex)
        {
          for (int xIndex = 0; xIndex < sizeX; ++xIndex)
          {
            // @todo: Needs to be changed
            cv::Point3f normal (xIndex - rangeX/2, yIndex - rangeY/2, zIndex - rangeZ);
            const float length = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
            const float invLength = 1.0f / (length + 0.00001f);

            normal.x *= invLength;
            normal.y *= invLength;
            normal.z *= invLength;

            float maxResponse = -1.0f;
            int maxIndex = -1;

            for (int normalIndex = 0; normalIndex < numOfNormals; ++normalIndex)
            {
              const float response = normal.x * refNormals[normalIndex].x +
                                     normal.y * refNormals[normalIndex].y +
                                     normal.z * refNormals[normalIndex].z;

              const float absResponse = fabs (response);
              if (maxResponse < absResponse)
              {
                maxResponse = absResponse;
                maxIndex = normalIndex;
              }

              lut[zIndex*sizeY*sizeX + yIndex*sizeX + xIndex] = 0x1 << maxIndex;
            }
          }
        }
      }
    };

    inline unsigned char 
    operator() (const float x, const float y, const float z) const
    {
      const int xIndex = x*offsetX + offsetX;
      const int yIndex = y*offsetY + offsetY;
      const int zIndex = z*rangeZ + rangeZ;

      const int index = zIndex*sizeY*sizeX + yIndex*sizeX + xIndex;

      return (lut[index]);
    };

    inline unsigned char 
    operator() (const int index) const
    {
      return (lut[index]);
    }
  };


  template <typename PointInT>
  class SurfaceNormalModality : public QuantizableModality, public PCLBase<PointInT>
  {
    public:

      typedef typename pcl::PointCloud<PointInT> PointCloudIn;

      SurfaceNormalModality ();

      virtual ~SurfaceNormalModality ();

      inline QuantizedMap &
      getQuantizedMap () 
      { 
        return (filtered_quantized_surface_normals_); 
      }

      inline QuantizedMap &
      getSpreadedQuantizedMap () 
      { 
        return (spreaded_quantized_surface_normals_); 
      }

      void 
      extractFeatures (MaskMap & mask, 
                       const int numOfFeatures, 
                       const int modalityIndex,
                       std::vector<QuantizedMultiModFeature> & features);

      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param[in] cloud the const boost shared pointer to a PointCloud message
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
      quantizeSurfaceNormals ();

      void
      filterQuantizedSurfaceNormals ();

      void
      computeDistanceMap (MaskMap & input,
                          DistanceMap & output);

    private:

      float feature_distance_threshold_;

      QuantizedNormalLookUpTable normal_lookup_;

      pcl::PointCloud<pcl::Normal> surface_normals_;
      pcl::QuantizedMap quantized_surface_normals_;
      pcl::QuantizedMap filtered_quantized_surface_normals_;
      pcl::QuantizedMap spreaded_quantized_surface_normals_;

  };

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::SurfaceNormalModality<PointInT>::
SurfaceNormalModality ()
: feature_distance_threshold_ (1.0f)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::SurfaceNormalModality<PointInT>::~SurfaceNormalModality ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::processInputData ()
{
  // compute surface normals
  pcl::LinearLeastSquaresNormalEstimation<PointInT, pcl::Normal> ne;
  //ne.setMaxDepthChangeFactor(1.0f);
  //ne.setNormalSmoothingSize(9.0f);
  //ne.setDepthDependentSmoothing(true);
  ne.setInputCloud (input_);
  ne.compute (surface_normals_);

  // quantize surface normals
  quantizeSurfaceNormals ();

  // filter quantized surface normals
  filterQuantizedSurfaceNormals ();

  // spread quantized surface normals
  const int spreadingSize = 8;
  pcl::QuantizedMap::spreadQuantizedMap (filtered_quantized_surface_normals_,
                                         spreaded_quantized_surface_normals_,
                                         spreadingSize);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::extractFeatures (
  MaskMap & mask,
  const int numOfFeatures,
  const int modalityIndex,
  std::vector<QuantizedMultiModFeature> & features )
{
  const int width = mask.getWidth ();
  const int height = mask.getHeight ();

  //cv::Mat maskImage(height, width, CV_8U, mask.mask);
  //cv::erode(maskImage, maskImage

	struct Candidate
	{
		Normal normal;
    float distance;

    unsigned char binIndex;

		int x;
		int y;	

		bool 
    operator< (const Candidate & rhs)
		{
			return (distance > rhs.distance);
		}
	};

  // create distance maps for every quantization value
  //cv::Mat distanceMaps[8];
  //for (int mapIndex = 0; mapIndex < 8; ++mapIndex)
  //{
  //  distanceMaps[mapIndex] = ::cv::Mat::zeros(height, width, CV_8U);
  //}

  MaskMap maskMaps[8];
  for (int mapIndex = 0; mapIndex < 8; ++mapIndex)
    maskMaps[mapIndex].initialize (width, height);

  unsigned char map[255];
  memset(map, 0, 255);

  map[0x1<<0] = 0;
  map[0x1<<1] = 1;
  map[0x1<<2] = 2;
  map[0x1<<3] = 3;
  map[0x1<<4] = 4;
  map[0x1<<5] = 5;
  map[0x1<<6] = 6;
  map[0x1<<7] = 7;

  QuantizedMap distanceMapIndices;
  distanceMapIndices.initialize (width, height);
  //memset (distanceMapIndices.data, 0, sizeof (distanceMapIndices.data[0])*width*height);

  for (int rowIndex = 0; rowIndex < height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < width; ++colIndex)
    {
      if (mask (colIndex, rowIndex) != 0)
      {
        //const unsigned char quantizedValue = quantized_surface_normals_ (rowIndex, colIndex);
        const unsigned char quantizedValue = filtered_quantized_surface_normals_ (colIndex, rowIndex);

        if (quantizedValue == 0) 
          continue;
        const int distMapIndex = map[quantizedValue];

        distanceMapIndices (colIndex, rowIndex) = distMapIndex;
        //distanceMaps[distMapIndex].at<unsigned char>(rowIndex, colIndex) = 255;
        maskMaps[distMapIndex] (colIndex, rowIndex) = 255;
      }
    }
  }

  DistanceMap distanceMaps[8];
  for (int mapIndex = 0; mapIndex < 8; ++mapIndex)
    computeDistanceMap (maskMaps[mapIndex], distanceMaps[mapIndex]);

	std::list<Candidate> list1;
	std::list<Candidate> list2;

  float weights[8] = {0,0,0,0,0,0,0,0};

  const int off = 4;
  for (int rowIndex = off; rowIndex < height-off; ++rowIndex)
  {
    for (int colIndex = off; colIndex < width-off; ++colIndex)
    {
      if (mask (colIndex, rowIndex) != 0)
      {
        //const unsigned char quantizedValue = quantized_surface_normals_ (rowIndex, colIndex);
        const unsigned char quantizedValue = filtered_quantized_surface_normals_ (colIndex, rowIndex);

        const float nx = surface_normals_ (colIndex, rowIndex).normal_x;
        const float ny = surface_normals_ (colIndex, rowIndex).normal_y;
        const float nz = surface_normals_ (colIndex, rowIndex).normal_z;

        if (quantizedValue != 0 && !(pcl_isnan (nx) || pcl_isnan (ny) || pcl_isnan (nz)))
        {
          const int distanceMapIndex = map[quantizedValue];

          //const float distance = distanceMaps[distanceMapIndex].at<float> (rowIndex, colIndex);
          const float distance = distanceMaps[distanceMapIndex] (colIndex, rowIndex);

          if (distance >= feature_distance_threshold_)
          {
            Candidate candidate;

            candidate.distance = distance;
            candidate.x = colIndex;
            candidate.y = rowIndex;
            candidate.binIndex = distanceMapIndex;

            list1.push_back (candidate);

            ++weights[distanceMapIndex];
          }
        }
      }
    }
  }

  for (std::list<Candidate>::iterator iter = list1.begin (); iter != list1.end (); ++iter)
    iter->distance *= 1.0f / weights[iter->binIndex];

  list1.sort ();

  if (list1.size() <= numOfFeatures)
  {
    features.reserve (list1.size ());
    for (std::list<Candidate>::iterator iter = list1.begin(); iter != list1.end(); ++iter)
    {
      QuantizedMultiModFeature feature;

      feature.x = iter->x;
      feature.y = iter->y;
      feature.modalityIndex = modalityIndex;
      feature.quantizedValue = filtered_quantized_surface_normals_ (iter->x, iter->y);

      features.push_back (feature);
    }

    return;
  }

  int distance = list1.size () / numOfFeatures + 1; // ???  @todo:!:!:!:!:!:!
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

        if (tmpDistance < sqrDistance)
        {
          candidateAccepted = false;
          break;
        }
      }

      if (candidateAccepted)
        list2.push_back (*iter1);

      if (list2.size() == numOfFeatures) break;
    }
    --distance;
  }

  for (std::list<Candidate>::iterator iter2 = list2.begin(); iter2 != list2.end(); ++iter2)
  {
    QuantizedMultiModFeature feature;

    feature.x = iter2->x;
    feature.y = iter2->y;
    feature.modalityIndex = modalityIndex;
    feature.quantizedValue = filtered_quantized_surface_normals_ (iter2->x, iter2->y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::quantizeSurfaceNormals ()
{
  const int width = input_->width;
  const int height = input_->height;

  quantized_surface_normals_.initialize (width, height);

  for (int rowIndex = 0; rowIndex < height; ++rowIndex)
  {
    for (int colIndex = 0; colIndex < width; ++colIndex)
    {
      const float normalX = surface_normals_ (colIndex, rowIndex).normal_x;
      const float normalY = surface_normals_ (colIndex, rowIndex).normal_y;
      const float normalZ = surface_normals_ (colIndex, rowIndex).normal_z;

      if (pcl_isnan(normalX) || pcl_isnan(normalY) || pcl_isnan(normalZ) || normalZ > 0)
      {
        quantized_surface_normals_ (colIndex, rowIndex) = 0;
        continue;
      }

      //quantized_surface_normals_.data[rowIndex*width+colIndex] =
      //  normal_lookup_(normalX, normalY, normalZ);

      float angle = atan2 (normalY, normalX)*180.0f/3.14f;

      if (angle < 0.0f) angle += 360.0f;
      if (angle >= 360.0f) angle -= 360.0f;

      int binIndex = static_cast<int> (angle*8.0f/360.0f);

      //quantized_surface_normals_.data[rowIndex*width+colIndex] = 0x1 << binIndex;
      quantized_surface_normals_ (colIndex, rowIndex) = binIndex;
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::filterQuantizedSurfaceNormals ()
{
  const int width = input_->width;
  const int height = input_->height;

  filtered_quantized_surface_normals_.initialize(width, height);

  // filter data
  for (int rowIndex = 1; rowIndex < height-1; ++rowIndex)
  {
    for (int colIndex = 1; colIndex < width-1; ++colIndex)
    {
      unsigned char histogram[9] = {0,0,0,0,0,0,0,0,0};

      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (rowIndex-1)*width+colIndex-1;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
      }
      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + rowIndex*width+colIndex-1;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
      }
      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (rowIndex+1)*width+colIndex-1;
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

      if (maxHistIndex != -1 && maxHistValue >= 1)
      {
        filtered_quantized_surface_normals_ (colIndex, rowIndex) = 0x1 << maxHistIndex;
      }
      else
      {
        filtered_quantized_surface_normals_ (colIndex, rowIndex) = 0;
      }

      //filtered_quantized_color_gradients_.data[rowIndex*width+colIndex] = quantized_color_gradients_.data[rowIndex*width+colIndex];
    }
  }

  //cv::Mat data1(quantized_surface_normals_.height, quantized_surface_normals_.width, CV_8U, quantized_surface_normals_.data);
  //cv::Mat data2(filtered_quantized_surface_normals_.height, filtered_quantized_surface_normals_.width, CV_8U, filtered_quantized_surface_normals_.data);

  //cv::medianBlur(data1, data2, 3);

  //for (int rowIndex = 0; rowIndex < height; ++rowIndex)
  //{
  //  for (int colIndex = 0; colIndex < width; ++colIndex)
  //  {
  //    filtered_quantized_surface_normals_.data[rowIndex*width+colIndex] = 0x1 << filtered_quantized_surface_normals_.data[rowIndex*width+colIndex];
  //  }
  //}
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>void
pcl::SurfaceNormalModality<PointInT>::computeDistanceMap (
  MaskMap & input,
  DistanceMap & output)
{
  const int width = input.getWidth ();
  const int height = input.getHeight ();

  output.initialize (width, height);

  // compute distance map
  //float *distanceMap = new float[input_->points.size ()];
  unsigned char *maskMap = input.getData ();
  float *distanceMap = output.getData ();
  for (size_t index = 0; index < width*height; ++index)
  {
    if (maskMap[index] == 0)
      distanceMap[index] = 0.0f;
    else
      distanceMap[index] = width + height;
  }

  // first pass
  float* previous_row = distanceMap;
  float* current_row = previous_row + width;
  for (size_t ri = 1; ri < height; ++ri)
  {
    for (size_t ci = 1; ci < width; ++ci)
    {
      const float upLeft  = previous_row [ci - 1] + 1.4f; //distanceMap[(ri-1)*input_->width + ci-1] + 1.4f;
      const float up      = previous_row [ci] + 1.0f;     //distanceMap[(ri-1)*input_->width + ci] + 1.0f;
      const float upRight = previous_row [ci + 1] + 1.4f; //distanceMap[(ri-1)*input_->width + ci+1] + 1.4f;
      const float left    = current_row  [ci - 1] + 1.0f;  //distanceMap[ri*input_->width + ci-1] + 1.0f;
      const float center  = current_row  [ci];             //distanceMap[ri*input_->width + ci];

      const float minValue = std::min (std::min (upLeft, up), std::min (left, upRight));

      if (minValue < center)
        current_row [ci] = minValue; //distanceMap[ri * input_->width + ci] = minValue;
    }
    previous_row = current_row;
    current_row += width;
  }

  float* next_row    = distanceMap + width * (height - 1);
  current_row = next_row - width;
  // second pass
  for (int ri = height-2; ri >= 0; --ri)
  {
    for (int ci = width-2; ci >= 0; --ci)
    {
      const float lowerLeft  = next_row [ci - 1] + 1.4f;    //distanceMap[(ri+1)*input_->width + ci-1] + 1.4f;
      const float lower      = next_row [ci] + 1.0f;        //distanceMap[(ri+1)*input_->width + ci] + 1.0f;
      const float lowerRight = next_row [ci + 1] + 1.4f;    //distanceMap[(ri+1)*input_->width + ci+1] + 1.4f;
      const float right      = current_row [ci + 1] + 1.0f; //distanceMap[ri*input_->width + ci+1] + 1.0f;
      const float center     = current_row [ci];            //distanceMap[ri*input_->width + ci];

      const float minValue = std::min (std::min (lowerLeft, lower), std::min (right, lowerRight));

      if (minValue < center)
        current_row [ci] = minValue; //distanceMap[ri*input_->width + ci] = minValue;
    }
  }
}


#endif    // PCL_FEATURES_SURFACE_NORMAL_MODALITY

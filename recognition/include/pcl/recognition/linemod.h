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

#ifndef PCL_FEATURES_LINEMOD
#define PCL_FEATURES_LINEMOD

#include <vector>

#include "pcl/recognition/quantizable_modality.h"
#include "pcl/recognition/region_xy.h"
#include "pcl/recognition/sparse_quantized_multi_mod_template.h"

namespace pcl
{

  class EnergyMaps
  {

  public:

    EnergyMaps () : width_ (-1), height_ (-1), numOfBins_ (-1) {};
    virtual ~EnergyMaps () {};

    inline int getWidth () { return width_; }
    inline int getHeight () { return height_; }
    inline int getNumOfBins () { return numOfBins_; }

    void initialize (const int width, const int height, const int numOfBins)
    {
      maps_.resize(numOfBins, NULL);
      width_ = width;
      height_ = height;
      numOfBins_ = numOfBins;

      const int mapsSize = width*height;

      for (int mapIndex = 0; mapIndex < maps_.size (); ++mapIndex)
      {
        maps_[mapIndex] = new unsigned char[mapsSize];
        memset (maps_[mapIndex], 0, mapsSize);
      }
    }

    void releaseAll ()
    {
      for (int mapIndex = 0; mapIndex < maps_.size (); ++mapIndex)
      {
        if (maps_[mapIndex] != NULL) delete[] maps_[mapIndex];
      }

      maps_.clear ();
      width_ = -1;
      height_ = -1;
      numOfBins_ = -1;
    }

    inline unsigned char & operator() (const int binIndex, const int colIndex, const int rowIndex)
    {
      return maps_[binIndex][rowIndex*width_ + colIndex];
    }

    inline unsigned char & operator() (const int binIndex, const int index)
    {
      return maps_[binIndex][index];
    }

    inline unsigned char * operator() (const int binIndex)
    {
      return maps_[binIndex];
    }

  private:

    int width_;
    int height_;
    int numOfBins_;
    std::vector< unsigned char* > maps_;

  };

  class LinearizedMaps
  {

  public:

    LinearizedMaps () : width_ (-1), height_ (-1), memWidth_ (-1), memHeight_ (-1), stepSize_ (-1) {};
    virtual ~LinearizedMaps () {};

    inline int getWidth () { return width_; }
    inline int getHeight () { return height_; }
    inline int getStepSize () { return stepSize_; }
    inline int getMapMemorySize () { return memWidth_ * memHeight_; }

    void initialize (const int width, const int height, const int stepSize)
    {
      maps_.resize(stepSize*stepSize, NULL);
      width_ = width;
      height_ = height;
      memWidth_ = width / stepSize;
      memHeight_ = height / stepSize;
      stepSize_ = stepSize;

      const int mapsSize = memWidth_ * memHeight_;

      for (int mapIndex = 0; mapIndex < maps_.size (); ++mapIndex)
      {
        maps_[mapIndex] = new unsigned char[2*mapsSize];
        memset (maps_[mapIndex], 0, 2*mapsSize);
      }
    }

    void releaseAll ()
    {
      for (int mapIndex = 0; mapIndex < maps_.size (); ++mapIndex)
      {
        if (maps_[mapIndex] != NULL) delete[] maps_[mapIndex];
      }

      maps_.clear ();
      width_ = -1;
      height_ = -1;
      memWidth_ = -1;
      memHeight_ = -1;
      stepSize_ = -1;
    }

    inline unsigned char* operator() (const int colIndex, const int rowIndex)
    {
      return maps_[rowIndex*stepSize_ + colIndex];
    }

    inline unsigned char* getOffsetMap (const int colIndex, const int rowIndex)
    {
      const int mapCol = colIndex % stepSize_;
      const int mapRow = rowIndex % stepSize_;

      const int mapMemColIndex = colIndex / stepSize_;
      const int mapMemRowIndex = rowIndex / stepSize_;

      return maps_[mapRow*stepSize_ + mapCol] + mapMemRowIndex*memWidth_ + mapMemColIndex;
    }

  private:

    int width_;
    int height_;
    int memWidth_;
    int memHeight_;
    int stepSize_;
    std::vector< unsigned char* > maps_;

  };

  struct LINEMODDetection
  {
    int x;
    int y;
    int templateID;
    float score;
  };

  /**
    * \brief Template matching using the LINEMOD approach.
    * \author Stefan Holzer, Stefan Hinterstoisser
    */
  class LINEMOD
  {
  
  public:

    /** \brief Constructor */
    LINEMOD();

    /** \brief Destructor */
    virtual ~LINEMOD();

    /** \brief Creates a template from the specified data and adds it to the matching queue. */
    int 
    createAndAddTemplate (
      std::vector< QuantizableModality* > & modalities,
      std::vector< MaskMap* > & masks,
      RegionXY & region );

    void
    detectTemplates (
      std::vector< QuantizableModality* > & modalities,
      std::vector< LINEMODDetection > & detections );

    inline SparseQuantizedMultiModTemplate &
    getTemplate (
    const int templateID ) { return templates_[templateID]; }

  private:

    /** template storage */
    std::vector< SparseQuantizedMultiModTemplate > templates_;

  };

}

#endif PCL_FEATURES_LINEMOD

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

#include "pcl/recognition/linemod.h"

#include <fstream>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::LINEMOD::
LINEMOD() :
  templates_ ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::LINEMOD::
~LINEMOD()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
int 
pcl::LINEMOD::
createAndAddTemplate (
  std::vector< pcl::QuantizableModality* > & modalities,
  std::vector< pcl::MaskMap* > & masks,
  pcl::RegionXY & region )
{
  // assuming width and height is same for all modalities; should we check this??
  //const int width = modalities[0]->getQuantizedMap().getWidth ();
  //const int height = modalities[0]->getQuantizedMap().getHeight ();

  SparseQuantizedMultiModTemplate linemodTemplate;
      
  // select N features from every modality (N = 50, hardcoded; CHANGE this to a parameter!!!)
  const int numOfFeaturesPerModality = 50;
  const size_t numOfModalities = modalities.size();
  for (int modalityIndex = 0; modalityIndex < numOfModalities; ++modalityIndex)
  {
    modalities[modalityIndex]->extractFeatures(*(masks[modalityIndex]), numOfFeaturesPerModality, modalityIndex, linemodTemplate.features);
  }

  // up to now all features are relative to the input frame; make them relative to the region center
  //const int centerX = region.x+region.width/2;
  //const int centerY = region.y+region.height/2;

  const size_t numOfFeatures = linemodTemplate.features.size();
  for (int featureIndex = 0; featureIndex < numOfFeatures; ++featureIndex)
  {
    //linemodTemplate.features[featureIndex].x -= centerX;
    //linemodTemplate.features[featureIndex].y -= centerY;
    linemodTemplate.features[featureIndex].x -= region.x;
    linemodTemplate.features[featureIndex].y -= region.y;
  }

  // set region relative to the center
  linemodTemplate.region.x = region.x;
  linemodTemplate.region.y = region.y;
  //linemodTemplate.region.x = region.x - centerX;
  //linemodTemplate.region.y = region.y - centerY;
  linemodTemplate.region.width = region.width;
  linemodTemplate.region.height = region.height;

  // add template to template storage
  templates_.push_back(linemodTemplate);

  return static_cast<int> (templates_.size () - 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
detectTemplates (
  std::vector< QuantizableModality* > & modalities,
  std::vector< LINEMODDetection > & detections )
{
  // create energy maps
  std::vector< EnergyMaps > modalityEnergyMaps;
  const size_t numOfModalities = modalities.size();
  for (int modalityIndex = 0; modalityIndex < numOfModalities; ++modalityIndex)
  {
    QuantizedMap & quantizedMap = modalities[modalityIndex]->getSpreadedQuantizedMap ();

    const int width = quantizedMap.getWidth ();
    const int height = quantizedMap.getHeight ();

    unsigned char * quantizedData = quantizedMap.getData ();

    const int numOfBins = 8;
    EnergyMaps energyMaps;
    energyMaps.initialize (width, height, numOfBins);
    //std::vector< unsigned char* > energyMaps(numOfBins);
    for (int binIndex = 0; binIndex < numOfBins; ++binIndex)
    {
      //energyMaps[binIndex] = new unsigned char[width*height];
      //memset (energyMaps[binIndex], 0, width*height);

      unsigned char val0 = 0x1 << binIndex; // e.g. 00100000
      unsigned char val1 = (0x1 << (binIndex+1)%8) | (0x1 << (binIndex+9)%8); // e.g. 01010000
      for (int index = 0; index < width*height; ++index)
      {
        if ((val0 & quantizedData[index]) != 0)
          ++energyMaps (binIndex, index);
        if ((val1 & quantizedData[index]) != 0)
          ++energyMaps (binIndex, index);
      }
    }

    modalityEnergyMaps.push_back (energyMaps);
  }

  // create linearized maps
  const int stepSize = 8;
  std::vector< std::vector< LinearizedMaps > > modalityLinearizedMaps;
  for (int modalityIndex = 0; modalityIndex < numOfModalities; ++modalityIndex)
  {
    const int width = modalityEnergyMaps[modalityIndex].getWidth ();
    const int height = modalityEnergyMaps[modalityIndex].getHeight ();

    std::vector< LinearizedMaps > linearizedMaps;
    const int numOfBins = modalityEnergyMaps[modalityIndex].getNumOfBins ();
    for (int binIndex = 0; binIndex < numOfBins; ++binIndex)
    {
      unsigned char * energyMap = modalityEnergyMaps[modalityIndex] (binIndex);

      LinearizedMaps maps;
      maps.initialize (width, height, stepSize);
      for (int mapRow = 0; mapRow < stepSize; ++mapRow)
      {
        for (int mapCol = 0; mapCol < stepSize; ++mapCol)
        {
          unsigned char * linearizedMap = maps (mapCol, mapRow);

          // copy data from energy maps
          const int linWidth = width/stepSize;
          const int linHeight = height/stepSize;
          for (int rowIndex = 0; rowIndex < linHeight; ++rowIndex)
          {
            for (int colIndex = 0; colIndex < linWidth; ++colIndex)
            {
              const int tmpColIndex = colIndex*stepSize + mapCol;
              const int tmpRowIndex = rowIndex*stepSize + mapRow;

              linearizedMap[rowIndex*linWidth + colIndex] = energyMap[tmpRowIndex*width + tmpColIndex];
            }
          }
        }
      }

      linearizedMaps.push_back (maps);
    }

    modalityLinearizedMaps.push_back (linearizedMaps);
  }

  // compute scores for templates
  const int width = modalityEnergyMaps[0].getWidth ();
  const int height = modalityEnergyMaps[0].getHeight ();
  for (size_t templateIndex = 0; templateIndex < templates_.size (); ++templateIndex)
  {
    const int memWidth = width / stepSize;
    const int memHeight = height / stepSize;
    const int memSize = memWidth * memHeight;

    unsigned char * scoreSums = new unsigned char[memSize];
    memset (scoreSums, 0, memSize);

    int maxScore = 0;
    for (size_t featureIndex = 0; featureIndex < templates_[templateIndex].features.size (); ++featureIndex)
    {
      QuantizedMultiModFeature & feature = templates_[templateIndex].features[featureIndex];

      //feature.modalityIndex;
      for (int binIndex = 0; binIndex < 8; ++binIndex)
      {
        if ((feature.quantizedValue & (0x1<<binIndex)) != 0)
        {
          maxScore += 2;

          unsigned char * data = modalityLinearizedMaps[feature.modalityIndex][binIndex].getOffsetMap (feature.x, feature.y);
          for (int memIndex = 0; memIndex < memSize; ++memIndex)
          {
            scoreSums[memIndex] += data[memIndex];
          }
        }
      }
    }

    const float invMaxScore = 1.0f / maxScore;
    
    int maxValue = 0;
    int maxIndex = 0;
    for (int memIndex = 0; memIndex < memSize; ++memIndex)
    {
      if (scoreSums[memIndex] > maxValue) 
      {
        maxValue = scoreSums[memIndex];
        maxIndex = memIndex;
      }
    }

    const int maxColIndex = (maxIndex % memWidth) * stepSize;
    const int maxRowIndex = (maxIndex / memWidth) * stepSize;

    LINEMODDetection detection;
    detection.x = maxColIndex;
    detection.y = maxRowIndex;
    detection.templateID = static_cast<int> (templateIndex);
    detection.score = maxValue * invMaxScore;

    detections.push_back (detection);

    delete[] scoreSums;
  }

  // release data
  for (size_t modalityIndex = 0; modalityIndex < modalityLinearizedMaps.size (); ++modalityIndex)
  {
    modalityEnergyMaps[modalityIndex].releaseAll ();
    for (size_t binIndex = 0; binIndex < modalityLinearizedMaps[modalityIndex].size (); ++binIndex)
      modalityLinearizedMaps[modalityIndex][binIndex].releaseAll ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
saveTemplates (const char* file_name)
{
  std::ofstream file_stream;
  file_stream.open (file_name, std::ofstream::out | std::ofstream::binary);

  serialize (file_stream);

  file_stream.close ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
loadTemplates (const char* file_name)
{
  std::ifstream file_stream;
  file_stream.open (file_name, std::ofstream::in | std::ofstream::binary);

  deserialize (file_stream);

  file_stream.close ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
serialize (::std::ostream & stream)
{
  const int num_of_templates = static_cast<int> (templates_.size ());
  write (stream, num_of_templates);
  for (int template_index = 0; template_index < num_of_templates; ++template_index)
  {
    templates_[template_index].serialize (stream);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
deserialize (::std::istream & stream)
{
  templates_.clear ();

  int num_of_templates;
  read (stream, num_of_templates);
  templates_.resize (num_of_templates);
  for (int template_index = 0; template_index < num_of_templates; ++template_index)
  {
    templates_[template_index].deserialize (stream);
  }
}

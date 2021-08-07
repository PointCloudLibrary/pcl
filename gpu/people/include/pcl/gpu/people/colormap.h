/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 * @author: Koen Buys, Anatoly Baksheev
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/gpu/people/tree.h>
#include <pcl/gpu/containers/device_array.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {         

      /** @brief gives a label and returns the color out of the colormap */         
      pcl::RGB getLColor(unsigned char l);

        /** @brief gives a label and returns the color out of the colormap */         
      pcl::RGB getLColor (pcl::Label l);
        
      void colorLMap(int W, int H, const trees::Label* l, unsigned char* c);       
      void colorLMap(const PointCloud<pcl::Label>& cloud_in, PointCloud<pcl::RGB>& colormap_out);             
      
 
      extern const unsigned char LUT_COLOR_LABEL[];
      extern const int LUT_COLOR_LABEL_LENGTH;
      
      PCL_EXPORTS void uploadColorMap(DeviceArray<pcl::RGB>& color_map);      
      PCL_EXPORTS void colorizeLabels(const DeviceArray<pcl::RGB>& color_map, const DeviceArray2D<unsigned char>& labels, DeviceArray2D<pcl::RGB>& color_labels);



      PCL_EXPORTS void colorizeMixedLabels(const DeviceArray<RGB>& color_map, const DeviceArray2D<unsigned char>& labels, 
                                           const DeviceArray2D<RGB>& image, DeviceArray2D<RGB>& color_labels);


      inline void colorFG ( int W, int H, const unsigned char* labels, unsigned char* c )
      {
        int numPix = W*H;
        for(int pi = 0; pi < numPix; ++pi)           
          if(labels[pi] !=0 ) 
          {
            c[3*pi+0] = 0xFF;
            c[3*pi+1] = 0x00;
            c[3*pi+2] = 0x00;
          }          
      }



    } // end namespace people
  } // end namespace gpu
} // end namespace pcl

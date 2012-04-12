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
 * @author: Koen Buys
 */

#ifndef PCL_GPU_PEOPLE_COLORMAP_H_
#define PCL_GPU_PEOPLE_COLORMAP_H_

//#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
//#include <pcl/gpu/people/tree_live.h>
#include <pcl/gpu/people/tree.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace display
      {
        static const unsigned char LUT_COLOR_LABEL[] = {
         50, 34, 22,
         24,247,196,
         33,207,189,
        254,194,127,
         88,115,175,
        158, 91, 64,
         14, 90,  2,
        100,156,227,
        243,167, 17,
        145,194,184,
        234,171,147,
        220,112, 93,
         93,132,163,
        122,  4, 85,
         75,168, 46,
         15,  5,108,
        180,125,107,
        157, 77,167,
        214, 89, 73,
         52,183, 58,
         54,155, 75,
        249, 61,187,
        143, 57, 11,
        246,198,  0,
        202,177,251,
        229,115, 80,
        159,185,  1,
        186,213,229,
         82, 47,144,
        140, 69,139,
        189,115,117,
         80, 57,150 };
        
        static const unsigned int LUT_COLOR_LABEL_LENGTH = 32;
/*  
        // @todo: delete this function
        cv::Scalar getLColor ( const uint8_t l)
        {
          const unsigned char* c = LUT_COLOR_LABEL + 3*l;
          return cv::Scalar( c[0], c[1], c[2] );
        }
*/
        /*
         * @brief gives a label and returns the color out of the colormap
         */  
        pcl::RGB getLColor (const uint8_t l)
        {
          pcl::RGB p;
          const unsigned char* c = LUT_COLOR_LABEL + 3*l;
          p.r = c[0];
          p.g = c[1];
          p.b = c[2];
          return p;
        }

        /*
         * @brief gives a label and returns the color out of the colormap
         */  
        pcl::RGB getLColor (pcl::Label l)
        {
          pcl::RGB p;
          unsigned char lab = static_cast<unsigned char> (l.label);
          const unsigned char* c = LUT_COLOR_LABEL + 3*lab;
          p.r = c[0];
          p.g = c[1];
          p.b = c[2];
          return p;
        }

        void colorLMap ( int W, int H, const pcl::gpu::people::trees::Label* l, unsigned char* c )
        {
          int numPix = W*H;
          for(int pi=0;pi<numPix;++pi) 
          {
            const unsigned char* color = LUT_COLOR_LABEL + 3*l[pi];
            c[3*pi+0] = color[0];
            c[3*pi+1] = color[1];
            c[3*pi+2] = color[2];
          }
        }

        void colorLMap (const pcl::PointCloud<pcl::Label>& cloud_in, pcl::PointCloud<pcl::RGB>&   colormap_out)
        {
          for(size_t i = 0; i < cloud_in.size (); i++)
          {
            colormap_out.points.push_back (getLColor (cloud_in.points[i] ));  
          }
          colormap_out.width = cloud_in.width;
          colormap_out.height = cloud_in.height;
        } 

        void colorFG ( int W, int H, const uint16_t* labels, unsigned char* c )
        {
          int numPix = W*H;
          for(int pi=0;pi<numPix;++pi)           
            if(labels[pi] !=0 ) 
            {
              c[3*pi+0] = 0xFF;
              c[3*pi+1] = 0x00;
              c[3*pi+2] = 0x00;
            }          
        }

        int colorLMap( const cv::Mat& lmap, cv::Mat& cmap )
        {
          if( lmap.depth() != CV_8U )  return -1;
          if( lmap.channels() != 1 )   return -1;

          int W = lmap.size().width;
          int H = lmap.size().height;

          cmap.create( H, W, CV_8UC3 );

          colorLMap( W,H, static_cast<const pcl::gpu::people::trees::Label*>(lmap.data), static_cast<unsigned char*>(cmap.data));
          //colorLMap( W,H, (const pcl::gpu::people::trees::Label*)lmap.data, (unsigned char*)cmap.data);

          return 0;
        }
      } // end namespace display
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
#endif //PCL_GPU_PEOPLE_COLORMAP_H_

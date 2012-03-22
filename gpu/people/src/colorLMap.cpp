/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart, Koen Buys
 * ************************************************* */

//#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/people/trees/tree_live.h>
#include <pcl/people/trees/tree.h>
#include <pcl/people/display/colormap.h>

namespace pcl
{
  namespace people
  {
    namespace display
    {
/*
      // @todo: delete this function
      cv::Scalar getLColor ( const uint8_t l)
      {
        const unsigned char* c = LUT_COLOR_LABEL + 3*l;
        return cv::Scalar( c[0], c[1], c[2] );
      }
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

      void colorLMap ( int W, int H,
                       const pcl::people::trees::Label* l,
                       unsigned char* c )
      {
        int numPix = W*H;
        for(int pi=0;pi<numPix;++pi) {
          const unsigned char* color = LUT_COLOR_LABEL + 3*l[pi];
          c[3*pi+0] = color[0];
          c[3*pi+1] = color[1];
          c[3*pi+2] = color[2];
        }
      }

      void colorFG ( int W, int H,
                     const uint16_t* labels,
                     unsigned char* c )
      {
        int numPix = W*H;
        for(int pi=0;pi<numPix;++pi) {
          if(labels[pi] !=0 ) {
            c[3*pi+0] = 0xFF;
            c[3*pi+1] = 0x00;
            c[3*pi+2] = 0x00;
          }
        }
      }

      int colorLMap( const cv::Mat& lmap, cv::Mat& cmap )
      {
        if( lmap.depth() != CV_8U )  return -1;
        if( lmap.channels() != 1 )   return -1;

        int W = lmap.size().width;
        int H = lmap.size().height;

        cmap.create( H, W, CV_8UC(3) );

        colorLMap( W,H, (const pcl::people::trees::Label*)lmap.data, (unsigned char*)cmap.data);

        return 0;
      }
    } // end namespace display
  } // end namespace people
} // end namespace PCL

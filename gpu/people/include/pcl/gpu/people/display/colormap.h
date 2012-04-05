/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart, Koen Buys
 * ************************************************* */

//#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/gpu/people/trees/tree_live.h>
#include <pcl/gpu/people/trees/tree.h>

namespace pcl
{
  namespace gpu
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
          unsigned char lab = l.label;
          const unsigned char* c = LUT_COLOR_LABEL + 3*lab;
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

        void colorLMap (const pcl::PointCloud<pcl::Label>& cloud_in,
                        pcl::PointCloud<pcl::RBG>&   colormap_out)
        {
          for(size_t i; i < cloud_in.size (); i++)
          {
            colormap_out.points.push_back (getLColor (cloud_in.points[i] ));  
          }
          colormap_out.width = cloud_in.width;
          colormap_out.heigth = cloud_in.heigth;
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
      } // end namespace display
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl

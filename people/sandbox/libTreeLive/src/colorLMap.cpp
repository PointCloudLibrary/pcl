/* *************************************************
 *
 * Copyright (2011) Willow Garage
 *
 * Author : Cedric Cagniart, Koen Buys
 * ************************************************* */

#include <opencv2/opencv.hpp>
#include <libTreeLive/TreeLive.h>
#include "commonTrees/Tree.h"

namespace TreeLive 
{

static const unsigned char LUT[] = {
50,34,22,
24,247,196,
33,207,189,
254,194,127,
88,115,175,
158,91,64,
14,90,2,
100,156,227,
243,167,17,
145,194,184,
234,171,147,
220,112,93,
93,132,163,
122,4,85,
75,168,46,
15,5,108,
180,125,107,
157,77,167,
214,89,73,
52,183,58,
54,155,75,
249,61,187,
143,57,11,
246,198,0,
202,177,251,
229,115,80,
159,185,1,
186,213,229,
82,47,144,
140,69,139,
189,115,117,
80,57,150 };

/*
//@todo get a system in the limb colors
static const unsigned char LUT[] = {
70,255,0,         // 0 Lfoot
90,255,0,         // 1 Lleg
110,255,0,        // 2 Lknee
130,255,0,        // 3 Lthigh
0,255,70,         // 4 Rfoot
0,255,90,         // 5 Rleg
0,255,110,        // 6 Rknee
0,255,130,        // 7 Rthigh
140,140,140,      // 8 Rhips
110,110,110,      // 9 Lhips
230,230,230,      // 10 Neck
70,0,255,         // 11 Rarm
90,0,255,         // 12 Relbow
110,0,255,        // 13 Rforearm
130,0,255,        // 14 Rhand
0,70,255,         // 15 Larm
0,90,255,         // 16 Lelbow
0,110,255,        // 17 Lforearm
0,130,255,        // 18 Lhand
255,127,127,      // 19 FaceLB
255,63,127,       // 20 FaceRB
255,127,63,       // 21 FaceLT
255,63,63,        // 22 FaceRT
200,200,200,      // 23 Rchest
170,170,170,      // 24 Lchest
0,0,0,            // 25
0,0,0,            // 26
0,0,0,            // 27
0,0,0,            // 28
0,0,0,            // 29
0,0,0,            // 30
0,0,0 };          // 31
*/

cv::Scalar getLColor( const uint8_t l)
{
	const unsigned char* c = LUT + 3*l;
	return cv::Scalar( c[0], c[1], c[2] );
}

void colorLMap( int W, int H,
				const Tree::Label* l,
				unsigned char* c )
{
	int numPix = W*H;
	for(int pi=0;pi<numPix;++pi) {
		const unsigned char* color = LUT + 3*l[pi];
		c[3*pi+0] = color[0];
		c[3*pi+1] = color[1];
		c[3*pi+2] = color[2];
	}
}



void colorFG( int W, int H,
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

	colorLMap( W,H, (const Tree::Label*)lmap.data, (unsigned char*)cmap.data); 

	return 0;
}



} // end namespace TreeLive

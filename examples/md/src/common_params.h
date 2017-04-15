#ifndef _COMMON_PARAMS_H_
#define _COMMON_PARAMS_H_

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <boost/thread/thread.hpp>

using namespace std;

#define NUM_2D_DESCRIPTORS                      4
#define NUM_3D_DESCRIPTORS                      3
#define NUM_DERIVED_EVALS                       4
#define TOTAL_DESC_COUNT                        ( NUM_2D_DESCRIPTORS + NUM_3D_DESCRIPTORS + NUM_DERIVED_EVALS )
#define	MIN_KEYPOINT_COUNT                      0x6
#define	MIN_CORRESPONDENCE_COUNT                0x4
#define DESC_INVALID                            -2


#define OUTPUT_RUNTIME_PERF                     -1
#define OUPUT_RESULT_SUMMARY                     0
#define OUTPUT_TRANSFORMATIONS                   1
#define OUTPUT_DETAILS                           2


// SIFT keypoint parameters
#define MIN_SCALE                                0.005
#define NR_OCTAVES                               10
#define NR_SCALES_PER_OCTAVE                     8
#define MIN_CONTRAST                             1.5

#define	IMAGE_2D_WIDTH	640	
#define	IMAGE_2D_HEIGHT	480

#define	IMAGE_2D_WIDTH_CENTER   IMAGE_2D_WIDTH/2
#define	IMAGE_2D_HEIGHT_CENTER  IMAGE_2D_HEIGHT/2

enum descriptorType	
{
                     DESC_SIFT,
                     DESC_SURF,
                     DESC_ORB,
                     DESC_BRISK,

                     DESC_FPFH33,
                     DESC_SHOT352, 
                     DESC_SHOT1344,

                     DESC_MD2dR,		// RANSAC applied to best/all 2D descriptors only.
                     DESC_MD3dR,		// RANSAC applied to best/all 3D descriptors only.
                     DESC_MD3dUR,		// RANSAC applied to unique KPs across 3D descriptors.
                     DESC_MDv,			// Voting applied to best/all descriptors (2D+3D).
};
/*
                     DESC_MDaR,			// RANSAC applied to best/all descriptors (2D+3D)-issue: threshold between 2D & 3D.
                     DESC_SIFT_PC,
                     DESC_SURF_PC,
                     DESC_ORB_PC,
                     DESC_BRISK_PC,
                
                     DESC_INVALID
*/

enum keyPointType2D	{ MDO_KP2D_SIFT  = 0x1000, MDO_KP2D_SURF, MDO_KP2D_ORB, MDO_KP2D_BRISK, MDO_KP2D_FAST, MDO_KP2D_INVALID };
enum descriptorType2D	{ MDO_D2D_SIFT	 = 0x1100, MDO_D2D_SURF,  MDO_D2D_ORB,  MDO_D2D_BRISK, MDO_D2D_INVALID };
enum keyPointType3D	{ MDO_KP3D_SIFT  = 0x1200, MDO_KP3D_INVALID };
enum descriptorType3D	{ MDO_D3D_FPFH33 = 0x1300, MDO_D3D_SHOT352, MDO_D3D_SHOT1344, MDO_D3D_INVALID };

#endif

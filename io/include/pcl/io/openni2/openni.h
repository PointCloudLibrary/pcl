/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 
#pragma once
 
#ifdef HAVE_OPENNI2
 
#include <pcl/pcl_config.h>

#if defined __GNUC__
#  pragma GCC system_header
#endif

#include <OpenNI.h>
#include <OniVersion.h>

// Standard resolutions, ported from OpenNI 1.x. To be removed later.
#define 	XN_QQVGA_X_RES   160
#define 	XN_QQVGA_Y_RES   120
#define 	XN_CGA_X_RES   320
#define 	XN_CGA_Y_RES   200
#define 	XN_QVGA_X_RES   320
#define 	XN_QVGA_Y_RES   240
#define 	XN_VGA_X_RES   640
#define 	XN_VGA_Y_RES   480
#define 	XN_SVGA_X_RES   800
#define 	XN_SVGA_Y_RES   600
#define 	XN_XGA_X_RES   1024
#define 	XN_XGA_Y_RES   768
#define 	XN_720P_X_RES   1280
#define 	XN_720P_Y_RES   720
#define 	XN_SXGA_X_RES   1280
#define 	XN_SXGA_Y_RES   1024
#define 	XN_UXGA_X_RES   1600
#define 	XN_UXGA_Y_RES   1200
#define 	XN_1080P_X_RES   1920
#define 	XN_1080P_Y_RES   1080
#define 	XN_QCIF_X_RES   176
#define 	XN_QCIF_Y_RES   144
#define 	XN_240P_X_RES   423
#define 	XN_240P_Y_RES   240
#define 	XN_CIF_X_RES   352
#define 	XN_CIF_Y_RES   288
#define 	XN_WVGA_X_RES   640
#define 	XN_WVGA_Y_RES   360
#define 	XN_480P_X_RES   864
#define 	XN_480P_Y_RES   480
#define 	XN_576P_X_RES   1024
#define 	XN_576P_Y_RES   576
#define 	XN_DV_X_RES   960
#define 	XN_DV_Y_RES   720

#endif // HAVE_OPENNI2

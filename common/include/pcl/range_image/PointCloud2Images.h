/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Intelligent Robotics Lab, DLUT.
*  Author: Yufeng Gu, Yan Zhuang
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
*   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
*     of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
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
*/

/**
* \file PointCloud2Images.h
* \Created on: May 10, 2018
* \https://github.com/GuYufeng93/Pointcloud-to-Images
*/

#ifndef PCL_POINTCLOUD2IMAGES_H_
#define PCL_POINTCLOUD2IMAGES_H_

#include <map>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
//#include <windows.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common_headers.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include "omp.h"

using namespace std;

namespace pcl
{
	struct SPoint
	{
		float x, y, z, r, i, g, b, l;
		float angle, height, radius;
	};

	struct SPixel
	{
		bool flag = 0;
		float x, y, z, i, l, depth, n;
		unsigned short BA_val, depth_val, intensity_val, PBA_val, N_val;
		unsigned short r, g, b;
		unsigned int index = -1;
	};

	class singleton
	{
	protected:
		singleton() {}
	private:
		static singleton* globaldata;
	public:
		static singleton* instance();
		vector<SPoint> data;
		int picnum = 1;
		vector<vector<SPixel>>vimage;
		vector<vector<vector<SPixel>>>n_image;
	};

	singleton* singleton::globaldata = NULL;

	class PointCloud2Image
	{

	public:
		PointCloud2Image();
		virtual ~PointCloud2Image();

	public:
		
		/** \data input */
		void 
		data_input(const string str, const pcl::PointXYZ Viewpoint, vector<SPoint>&data);
		void
		data_input(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, const pcl::PointXYZ Viewpoint, vector<SPoint>&data);

		/** \Mapping scattered 3D point clouds to 2D structures */
		void
		get_Matrix(singleton *globaldata, const int w_degree, const int h_degree, const int w_n_pixel, const int h_n_pixel, const int picnum);

		/** \Feature extraction */
		void
		get_Feature(singleton *globaldata);

		/** \Image output */
		void 
		draw_picture_BA(const string path, const vector<vector<SPixel>>&vimage);
		void 
		draw_picture_Intensity(const string path, const vector<vector<SPixel>>&vimage);
		void 
		draw_picture_PBA(const string path, const vector<vector<SPixel>>&vimage);
		void 
		draw_picture_RGB(const string path, const vector<vector<SPixel>>&vimage);
		void 
		draw_picture_N(const string path, const vector<vector<SPixel>>&vimage);
		void 
		draw_picture_Depth(const string path, const vector<vector<SPixel>>&vimage);

	private:

		/** \Extract various features */
		void 
		get_feature_BA(const vector<SPoint>&data, vector<vector<SPixel>>&vimage);

		void 
		get_feature_PBA(const vector<SPoint>&data, vector<vector<SPixel>>&vimage);

		void 
		get_feature_N(const vector<SPoint>&data, vector<vector<SPixel>>&vimage);

		void 
		get_feature_Intensity(const vector<SPoint>&data, vector<vector<SPixel>>&vimage);

		void 
		get_feature_Depth(const vector<SPoint>&data, vector<vector<SPixel>>&vimage);
	};
	

	class Point3d
	{
	public:
		float x, y, z;
		Point3d(void);
		Point3d(const Point3d& tmpP);
		friend Point3d operator - (const Point3d &p1, const Point3d &p2);
		friend float operator *(const Point3d &p1, const Point3d &p2);
		float Dist(void);
		Point3d & Normalize();
	};

	
}

#endif


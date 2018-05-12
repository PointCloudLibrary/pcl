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
* \file PointCloud2Images.cpp
* \Created on: May 10, 2018
* \author: Yufeng Gu (guyufeng@mail.dlut.edu.cn)
*/

//#include "stdafx.h"
#include "PointCloud2Images.h"

namespace pcl
{
	singleton* singleton::instance(void)
	{
		if (globaldata == NULL)
			globaldata = new singleton();
		return globaldata;
	}

	//////////////////////////////////////////////////////////////////////
	Point3d::Point3d(void)
	{
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}

	Point3d::Point3d(const Point3d& tmpP)
	{
		this->x = tmpP.x;
		this->y = tmpP.y;
		this->z = tmpP.z;
	}
	Point3d operator - (const Point3d &p1, const Point3d &p2)
	{
		Point3d po;
		po.x = p1.x - p2.x;
		po.y = p1.y - p2.y;
		po.z = p1.z - p2.z;
		return po;
	}

	float operator *(const Point3d  &p1, const Point3d  &p2)
	{
		return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z);
	}

	float Point3d::Dist(void)
	{
		return sqrt(x*x + y*y + z*z);
	}

	Point3d & Point3d::Normalize()
	{
		float n = float(sqrt(x*x + y*y + z*z));
		if (n > float(0))
		{
			x /= n; y /= n; z /= n;
		}
		return *this;
	}

	//////////////////////////////////////////////////////////////////////
	PointCloud2Image::PointCloud2Image() {}
	PointCloud2Image::~PointCloud2Image() {}

	void
	PointCloud2Image::data_input(const string str, const pcl::PointXYZ Viewpoint, vector<SPoint>&data)
	{
		data.clear();
		ifstream infile;
		infile.open(str);
		float x_, y_, z_, i_;
		float r_, g_, b_, l_;
		while (infile >> x_ >> y_ >> z_ >> i_ >> r_ >> g_ >> b_ >> l_)
		{
			SPoint p;
			p.x = x_ - Viewpoint.x;
			p.y = y_ - Viewpoint.y;
			p.z = z_ - Viewpoint.z;
			p.i = i_;
			p.r = r_;
			p.g = g_;
			p.b = b_;
			p.l = l_;
			data.push_back(p);
		}
		infile.close();

	}

	void
	PointCloud2Image::data_input(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, const pcl::PointXYZ Viewpoint, vector<SPoint>&data)
	{
		data.clear();
		for (int i = 0; i < cloud.points.size();i++)
		{
			SPoint p;
			p.x = cloud.points[i].x - Viewpoint.x;
			p.y = cloud.points[i].y - Viewpoint.y;
			p.z = cloud.points[i].z - Viewpoint.z;
			p.i = cloud.points[i].a;
			p.r = cloud.points[i].r;
			p.g = cloud.points[i].g;
			p.b = cloud.points[i].b;
			data.push_back(p);
		}
	}

	//////////////////////////////////////////////////////////////////////
	#define RAD_TO_DEG (180 / (4 * atan(1)))
	#define DEG_TO_RAD ((4 * atan(1)) / 180) 

	void
	PointCloud2Image::get_Matrix(singleton *globaldata, const int w_degree, const int h_degree, const int w_n_pixel , const int h_n_pixel, const int picnum)
	{
		if (w_degree*h_degree*w_n_pixel*h_n_pixel*picnum == 0)
		{
			cout << "The parameter should not be 0 " << endl;
			return;
		}
		float angel_increase = 360.0 / picnum;
		globaldata->picnum = picnum;
		globaldata->n_image.clear();
		globaldata->n_image.resize(globaldata->picnum);
		for (int i = 0; i<globaldata->picnum; i++)
		{
			globaldata->n_image[i].resize(w_n_pixel);
			for (int j = 0; j<w_n_pixel; j++)
			{
				globaldata->n_image[i][j].resize(h_n_pixel);
			}
		}

		map<int, vector<long long>>angle2index_map;
		for (long long i = 0; i < globaldata->data.size(); i++)
		{
			globaldata->data[i].angle = RAD_TO_DEG * atan2(globaldata->data[i].y, globaldata->data[i].x) + 180;
			globaldata->data[i].radius = sqrt(globaldata->data[i].x*globaldata->data[i].x + globaldata->data[i].y*globaldata->data[i].y);
			angle2index_map[int(globaldata->data[i].angle)].push_back(i);
		}

		#pragma omp parallel for
		for (int i = 0; i < globaldata->picnum; i++)
		{
			for (multimap<int, vector<long long>>::iterator it = angle2index_map.begin(); it != angle2index_map.end(); it++)
			{
				if (!(fabs((*it).first - i*angel_increase) < (w_degree / 2 + 1) ||
					(360 - fabs((*it).first - i*angel_increase) < (w_degree / 2 + 1))))
					continue;

				for (int t = 0; t < (*it).second.size(); t++)
				{
					int j = (*it).second[t];
					int w, h;
					float ag, depth;
					if (fabs(globaldata->data[j].angle - i*angel_increase)<w_degree / 2)
					{
						ag = globaldata->data[j].angle - i*angel_increase;
					}
					else if (360 - fabs(globaldata->data[j].angle - i*angel_increase)<w_degree / 2)
					{
						if (globaldata->data[j].angle - i*angel_increase >= 0) ag = -(360 - fabs(globaldata->data[j].angle - i*angel_increase));
						else  ag = (360 - fabs(globaldata->data[j].angle - i*angel_increase));
					}
					else
						continue;
					depth = (2 * cos(fabs(ag)*DEG_TO_RAD)*globaldata->data[j].radius);

					w = int(w_n_pixel / 2 * (1 + tan(ag*DEG_TO_RAD) / tan(w_degree / 2 * DEG_TO_RAD)));
					h = floor((1 + globaldata->data[j].z / (globaldata->data[j].radius*cos(ag*DEG_TO_RAD)))*h_n_pixel / 2);

					if (h < 0 || h >= h_n_pixel)
						continue;
					if (globaldata->n_image[i][w][h].flag&&globaldata->n_image[i][w][h].depth_val < depth)
						continue;

					globaldata->n_image[i][w][h].r = unsigned short(globaldata->data[j].r);
					globaldata->n_image[i][w][h].g = unsigned short(globaldata->data[j].g);
					globaldata->n_image[i][w][h].b = unsigned short(globaldata->data[j].b);
					globaldata->n_image[i][w][h].l = unsigned short(globaldata->data[j].l);
					globaldata->n_image[i][w][h].x = (globaldata->data[j].x);
					globaldata->n_image[i][w][h].y = (globaldata->data[j].y);
					globaldata->n_image[i][w][h].z = (globaldata->data[j].z);
					globaldata->n_image[i][w][h].i = unsigned short(globaldata->data[j].i);
					globaldata->n_image[i][w][h].depth = depth;
					globaldata->n_image[i][w][h].flag = true;
					globaldata->n_image[i][w][h].index = j;
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////
	using namespace cv;

	void
	PointCloud2Image::get_feature_BA(const vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		unsigned short ba_grey;
		Point3d _position;
		for (int i = 0; i < vimage.size(); i++)
			for (int j = 0; j < vimage[i].size(); j++)
			{
				if (vimage[i][j].flag == 0)
				{
					ba_grey = 0;
					continue;
				}

				int i_nb = i - 1;
				int j_nb = j + 1;
				if (i_nb < 0)i_nb = 0;
				if (j_nb >= vimage[i].size())j_nb = vimage[i].size() - 1;

				Point3d pa, pc, p;
				pa.x = data[vimage[i][j].index].x; pa.y = data[vimage[i][j].index].y; pa.z = data[vimage[i][j].index].z;
				pa = pa - _position;
				if (vimage[i_nb][j_nb].flag != 0)
				{
					pc.x = data[vimage[i_nb][j_nb].index].x; pc.y = data[vimage[i_nb][j_nb].index].y; pc.z = data[vimage[i_nb][j_nb].index].z;
				}

				pc = pc - _position;
				p = pa - pc;

				float angle = acos((p * (pc.Normalize())) / p.Dist());

				ba_grey = 255 - angle / 3.1415927 * 255;
				if (ba_grey < 0) ba_grey = 0;
				if (ba_grey > 255) ba_grey = 255;

				vimage[i][j].BA_val = unsigned short(ba_grey);
			}
	}

	void
	PointCloud2Image::get_feature_N(const vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		int nei[18] = { -1,-1, 0,-1, 1,-1, -1,0,  0,0,  1,0, -1,1,  0,1,  1,1 };
		int nei_t = 1;
		for (int i = 0; i < vimage.size(); i++)
		{
			for (int j = 0; j < vimage[i].size(); j++)
			{
				if (vimage[i][j].flag == 0)
				{
					vimage[i][j].N_val = 0;
					continue;
				}
				pcl::PointCloud<pcl::PointXYZL> Cpoints;
				for (int t = 0; t < nei_t; t++)
				{
					for (int p = 0; p < 9; p++)
					{
						int i_, j_;
						i_ = i + (t + 1)*nei[2 * p];
						j_ = j + (t + 1)*nei[2 * p + 1];
						if (i_ < 0 || i_ >= vimage.size())continue;
						if (j_ < 0 || j_ >= vimage[i].size())continue;
						if (vimage[i_][j_].flag == 0)continue;

						pcl::PointXYZL _p;
						_p.x = data[vimage[i_][j_].index].x;
						_p.y = data[vimage[i_][j_].index].y;
						_p.z = data[vimage[i_][j_].index].z;
						Cpoints.push_back(_p);

					}
				}
				if (Cpoints.points.size() < 3)
				{
					vimage[i][j].N_val = 255;
					continue;
				}

				EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
				Eigen::Vector4f xyz_centroid;
				EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
				EIGEN_ALIGN16 Eigen::Vector3f eigen_values;

				if (pcl::computeMeanAndCovarianceMatrix(Cpoints, covariance_matrix, xyz_centroid) == 0)
				{
					vimage[i][j].N_val = 255;
					continue;
				}
				pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
				vimage[i][j].N_val = int(fabs(eigen_vectors(2, 0)) * 255);
				vimage[i][j].n = fabs(eigen_vectors(2, 0));

			}
		}

	}

	void
	PointCloud2Image::get_feature_PBA(const vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		for (int i = 1; i < vimage.size() - 1; i++)
		{
			for (int j = 1; j < vimage[i].size() - 1; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					vimage[i][j].PBA_val = 0;
					continue;
				}
				SPoint p0; p0.x = 0; p0.y = 0; p0.z = 0;

				SPoint p1, p2, p3;
				int p1_x, p1_y, p3_x, p3_y;
				p2 = data[vimage[i][j].index];

				float d = vimage[0].size() / 180.0;
				float d1 = fabs(j * 1.0 / d - 45);
				float d2 = d1 + 1.0 / d;
				float ref = tan(d2 / 180 * 3.1416) / tan(d1 / 180 * 3.1416);

				if (j > vimage[0].size() / 2)
				{
					p1_x = i - 1;
					p1_y = j + 1;
					p3_x = i + 1;
					p3_y = j + 1;

				}
				else
				{
					p1_x = i - 1;
					p1_y = j - 1;
					p3_x = i + 1;
					p3_y = j - 1;

				}

				if (vimage[p1_x][p1_y].flag == 0)p1 = p0;
				else p1 = data[vimage[p1_x][p1_y].index];
				if (vimage[p3_x][p3_y].flag == 0)p3 = p0;
				else p3 = data[vimage[p3_x][p3_y].index];

				double side[3];
				side[0] = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
				side[1] = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) + pow(p1.z - p3.z, 2));
				side[2] = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) + pow(p3.z - p2.z, 2));

				if ((side[0] * side[2]) == 0) { vimage[i][j].PBA_val = 0; continue; }
				float angle = acos((pow(side[0], 2) + pow(side[2], 2) - pow(side[1], 2)) / 2 / (side[0] * side[2]));

				if ((sqrt(p1.x *p1.x + p1.y*p1.y + p1.z*p1.z) + sqrt(p3.x *p3.x + p3.y*p3.y + p3.z*p3.z))>(2 * sqrt(p2.x *p2.x + p2.y*p2.y + p2.z*p2.z)*0.999))

				{
					angle = 2 * 3.1416 - angle;
				}
				vimage[i][j].PBA_val = (angle / 2 / 3.1416 * 255);

			}
		}

	}

	void
	PointCloud2Image::get_feature_Depth(const vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		struct  P_coordinate
		{
			int i, j;
		};

		multimap<float, P_coordinate>Depth2color_map;

		for (int i = 0; i < vimage.size(); i++)
		{
			for (int j = 0; j < vimage[i].size(); j++)
			{
				if (vimage[i][j].flag == 0)continue;
				P_coordinate _pc;
				_pc.i = i;
				_pc.j = j;
				Depth2color_map.insert(make_pair(vimage[i][j].depth, _pc));
			}
		}
		int total_P = Depth2color_map.size() + 1;
		float i = 0;
		for (multimap<float, P_coordinate>::iterator it = Depth2color_map.begin(); it != Depth2color_map.end(); it++)
		{
			i++;
			vimage[(*it).second.i][(*it).second.j].depth_val = int(i * 255 / total_P);
		}
	}

	void
	PointCloud2Image::get_feature_Intensity(const vector<SPoint>&data, vector<vector<SPixel>>&vimage)
	{
		struct  P_coordinate
		{
			int i, j;
		};

		multimap<float, P_coordinate>Intensity2color_map;

		for (int i = 0; i < vimage.size(); i++)
		{
			for (int j = 0; j < vimage[i].size(); j++)
			{
				if (vimage[i][j].flag == 0)continue;
				P_coordinate _pc;
				_pc.i = i;
				_pc.j = j;
				Intensity2color_map.insert(make_pair(vimage[i][j].i, _pc));
			}
		}
		int total_P = Intensity2color_map.size() + 1;
		float i = 0;
		for (multimap<float, P_coordinate>::iterator it = Intensity2color_map.begin(); it != Intensity2color_map.end(); it++)
		{
			i++;
			vimage[(*it).second.i][(*it).second.j].intensity_val = int(i * 255 / total_P);
		}
	}

	void
	PointCloud2Image::get_Feature(singleton *globaldata)
	{
		#pragma omp parallel for
		for (int i = 0; i < globaldata->n_image.size(); i++)
		{
			#pragma omp parallel
			{
				get_feature_N(globaldata->data, globaldata->n_image[i]);
				get_feature_BA(globaldata->data, globaldata->n_image[i]);
				get_feature_PBA(globaldata->data, globaldata->n_image[i]);
				get_feature_Depth(globaldata->data, globaldata->n_image[i]);
				get_feature_Intensity(globaldata->data, globaldata->n_image[i]);
			}
		}
	}

	//////////////////////////////////////////////////////////////////////

	void
	PointCloud2Image::draw_picture_BA(const string path, const vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = 255 - vimage[i][j].BA_val;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void
	PointCloud2Image::draw_picture_PBA(const string path, const vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = 255 - vimage[i][j].PBA_val;
				if (rgb_grey < 127)rgb_grey = 64;
				else rgb_grey = 192;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void
	PointCloud2Image::draw_picture_N(const string path, const vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = vimage[i][j].N_val;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void
	PointCloud2Image::draw_picture_Intensity(const string path, const vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = 255 - vimage[i][j].intensity_val;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void
	PointCloud2Image::draw_picture_Depth(const string path, const vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				int rgb_grey = vimage[i][j].depth_val;
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = rgb_grey;
				rgba[1] = rgb_grey;
				rgba[2] = rgb_grey;
				rgba[3] = 255;
			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}

	void 
	PointCloud2Image::draw_picture_RGB(const string path, const vector<vector<SPixel>>&vimage)
	{
		int width = vimage.size();
		int height = vimage[0].size();
		cv::Mat mat(height, width, CV_8UC4);
		for (int i = 0; i<width; i++)
		{
			for (int j = 0; j<height; j++)
			{
				if (vimage[i][j].flag == 0)
				{
					cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
					rgba[0] = 0;
					rgba[1] = 0;
					rgba[2] = 0;
					rgba[3] = 0;
					continue;
				}
				cv::Vec4b& rgba = mat.at<cv::Vec4b>((height - 1) - j, (width - 1) - i);
				rgba[0] = uchar(vimage[i][j].r);
				rgba[1] = uchar(vimage[i][j].g);
				rgba[2] = uchar(vimage[i][j].b);
				rgba[3] = 255;

			}
		}
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(path.c_str(), mat, compression_params);
	}
}

int main(int argc, char* argv[])
{
	using namespace pcl;
	const int w_degree = 60;	//Vertical viewing angle
	const int h_degree = 180;	//Horizontal viewing angle
	const int w_n_pixel = 360;	//Picture width
	const int h_n_pixel = 360;	//Picture height
	const int picnum = 60;		//Picture num

	//The best point of view is to select the middle of the scene or the data acquisition track
	const pcl::PointXYZ Viewpoint (0,0,0);//Viewpoint
	
	string data_in = "data_xyzirgbl.txt";
	string images_out = "Image//";

	if (argc == 1)
	{
		data_in = "..//..//Points2Image//data_xyzirgbl.txt";
		images_out = "..//..//Points2Image//Image//";
	}
	else if (strcmp(argv[1], "help") == 0 || strcmp(argv[1], "h") == 0)
	{
		cout << "Program input: [Data_input path] and [Images_output path]" << endl;
		cout << "Data Format:[x y z i r g b l]" << endl;
		return 0;
	}
	else if (argc == 3)
	{
		data_in = argv[1];
		images_out = argv[2];
	}
	else if (argc > 3)
	{
		cout << "Program input: [Data_input path] and [Images_output path]" << endl;
		cout << "Data Format:[x y z i r g b l]" << endl;
		return 0;
	}

	singleton *globaldata = singleton::instance();
	PointCloud2Image pci;

	double start, end;
	//start = GetTickCount();
	pci.data_input(data_in, Viewpoint, globaldata->data);
	if (globaldata->data.size()==0)
	{
		cout << "Data_path : "<< data_in << endl;
		cout << "Point cloud size is empty! " << endl;
		//system("pause");
		return 0;
	}
	//end = GetTickCount();
	//cout << ">>> Data_input ok! Time consuming " << (end - start) / 1000 << "s" << " Data size: " << globaldata->data.size() << endl;

	//start = GetTickCount();
	pci.get_Matrix(globaldata, w_degree, h_degree, w_n_pixel, h_n_pixel, picnum);
	//end = GetTickCount();
	//cout << ">>> Get_Matrix ok! Time consuming " << (end - start) / 1000 << "s" << endl;

	//start = GetTickCount();
	pci.get_Feature(globaldata);
	//end = GetTickCount();
	//cout << ">>> Get_Feature ok! Time consuming " << (end - start) / 1000 << "s" << endl;

	//start = GetTickCount();
	#pragma omp parallel for
	for (int i = 0; i < globaldata->picnum; i++)
	{
		stringstream ss; string t;
		ss << i; ss >> t;
		#pragma omp parallel
		{
			pci.draw_picture_PBA(string(images_out) + "PBA_" + t + ".png", globaldata->n_image[i]);
			pci.draw_picture_N(string(images_out) + "N_" + t + ".png", globaldata->n_image[i]);
			pci.draw_picture_BA(string(images_out) + "BA_" + t + ".png", globaldata->n_image[i]);
			pci.draw_picture_Intensity(string(images_out) + "I_" + t + ".png", globaldata->n_image[i]);
			pci.draw_picture_Depth(string(images_out) + "Depth_" + t + ".png", globaldata->n_image[i]);
			pci.draw_picture_RGB(string(images_out) + "RGB_" + t + ".png", globaldata->n_image[i]);
		}
	}
	//end = GetTickCount();
	//cout << ">>> Draw_picture ok! Time consuming " << (end - start) / 1000 << "s" << endl;
	//system("pause");
	return 0;
}


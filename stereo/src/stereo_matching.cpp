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
 *
 */

#include "pcl/stereo/stereo_matching.h"

pcl::StereoMatching::StereoMatching(void)
{

	disp_map_ = NULL;
	disp_map_trg_ = NULL;

	width_ = -1; 
	height_ = -1;

}

pcl::StereoMatching::~StereoMatching(void)
{
	if ( disp_map_ != NULL)
	{
		delete [] disp_map_;
		//disp_map_ = NULL;
	}

	if ( disp_map_trg_ != NULL)
	{
		delete [] disp_map_trg_;
		//disp_map_trg_ = NULL;
	}

}

void pcl::StereoMatching::medianFilter(int radius)
{

	//TODO: do median filter

}

void pcl::StereoMatching::leftRightCheck(float* map_ref, float* map_trg)
{
	//TODO: do left right check

}


pcl::GrayStereoMatching::GrayStereoMatching()
{

}

pcl::GrayStereoMatching::~GrayStereoMatching()
{

}

void pcl::GrayStereoMatching::preProcessing(unsigned char *ref, unsigned char *right)
{
	//TODO: grayscale pre processing for stereo pairs

}


void pcl::GrayStereoMatching::compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height)
{
	//TODO: Check here the single channel for the image; probably convert them if colored and maybe output a warning
	
	if ( disp_map_ == NULL)
	{
		disp_map_ = new float[width * height];
		disp_map_trg_ = new float[width * height];		
	}
	else if ( width_ != width || height_ != height)
	{
		delete [] disp_map_;
		delete [] disp_map_trg_;

		disp_map_ = new float[width * height];
		disp_map_trg_ = new float[width * height];
	}

	width_ = width;
	height_ = height;

	if ( is_pre_proc_)
		preProcessing(ref_img, trg_img);

	compute_impl(ref_img, trg_img);

	if ( is_lr_check_)
	{
		//TODO: swap images, compute disp map trg
				
		leftRightCheck( disp_map_, disp_map_trg_);
	}

}


//const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl::StereoMatching::getPointCloud(float uC, float vC, float focal, float baseline)
void pcl::GrayStereoMatching::getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZI> &cloud, unsigned char *ref_img) 
{
	//TODO: compute cloud from disparity map
	
	//disp map has not been computed yet..
	if ( disp_map_ == NULL)
	{
		//Add a pcl warning
		return;
	}

	//TODOcheck that ref img, if it exists, has same size as disp map

	//cloud needs to be re-allocated
	if (cloud.width != width_ || cloud.height != height_)
	{
		//cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width_, height_) );
		cloud.resize(width_*height_);
		cloud.width = width_;
		cloud.height = height_;

	}

	//Loop
	pcl::PointXYZI temp_point;
	for ( int j=0; j<height_; j++)
	{
		for ( int i=0; i<width_; i++)
		{
			if ( disp_map_[ j*width_ + i] > 0 )
			{

				temp_point.z = ( baseline * focal ) / disp_map_[ j*width_ + i];
				temp_point.x = ( (i-u_c) * temp_point.z) / focal;
				temp_point.y = ( (j-v_c) * temp_point.z) / focal;

				if ( ref_img != NULL)
					temp_point.intensity = ref_img[ j*width_ + i];
				else
					temp_point.intensity = 255;

				cloud[ j*width_ + i] = temp_point;
			}
		}
	}

	return;
}


pcl::AdaptiveCostSOStereoMatching::AdaptiveCostSOStereoMatching()
{

}

void pcl::AdaptiveCostSOStereoMatching::compute_impl(unsigned char* ref_img, unsigned char* trg_img)
{

	//TODO: body of the specific stereo matching algorithm
}
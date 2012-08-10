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

	ref_img_ = NULL;
	trg_img_ = NULL;

	pp_ref_img_ = NULL;
	pp_trg_img_ = NULL;

	width_ = -1; 
	height_ = -1;

	max_disp_ = -1;
	x_off_ = 0;

	ratio_filter_ = 0;
	peak_filter_ = 0;

	is_pre_proc_ = false;
	is_lr_check_ = false;
	lr_check_th_ = 1;

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

	if ( ref_img_ != NULL)
	{
		delete [] ref_img_;
		delete [] trg_img_;
	}

	if ( pp_ref_img_ != NULL)
	{
		delete [] pp_ref_img_;
		delete [] pp_trg_img_;
	}

}

void pcl::StereoMatching::medianFilter(int radius)
{

	//TODO: do median filter
	int side = radius*2+1;

	short int *out = new short int [width_*height_];
	memset(out, 0, width_*height_*sizeof(short int));

	short int *v = new short int [side*side];

	for(int y=radius; y<height_-radius; y++)
	{
		for(int x=radius; x<width_-radius; x++){

			if(disp_map_[y*width_+x] <= 0)
				out[y*width_+x] = disp_map_[y*width_+x];
			else
			{

				int n=0;
				for(int j=-radius; j<=radius; j++)
				{
					for(int i=-radius; i<=radius; i++)
					{
						if(disp_map_[(y+j)*width_ + x+i] > 0)
						{
							v[n] = disp_map_[(y+j)*width_ + x+i];
							n++;
						}
					}
				}

				std::sort(v, v+n);
				out[y*width_+x] = v[n/2];
			}
		}
	}

	short int* temp_ptr = out;
	out = disp_map_;
	disp_map_ = temp_ptr;

	delete [] out;
	delete [] v;
}

void pcl::StereoMatching::getVisualMap(pcl::PointCloud<pcl::RGB>::Ptr vMap)
{

	if ( vMap->width != width_ || vMap->height != height_)
	{
		vMap->resize(width_*height_);
		vMap->width = width_;
		vMap->height = height_;
	}

	pcl::RGB invalid_val;
	invalid_val.r = 0;
	invalid_val.g = 255;
	invalid_val.b = 0;

	float scale = 255.0f/(16 * max_disp_);

	for(int y=0; y<height_; y++)
	{
		for(int x=0; x<width_; x++)
		{
			if(disp_map_[y * width_+ x] <= 0)
			{
				vMap->at(x,y) = invalid_val;
			}
			else
			{
				unsigned char val = (unsigned char) floor(scale*disp_map_[y * width_+ x]);
				vMap->at(x,y).r = val;
				vMap->at(x,y).g = val;
				vMap->at(x,y).b = val;
			}
		}
	}
}

void pcl::StereoMatching::leftRightCheck()
{
	short int p1, p2, p2i;

	for(int y=0; y<height_; y++)
	{
		for(int x=0; x<width_; x++)
		{
			if( disp_map_[y * width_ + x] > 0 )
			{
			
				p1 = disp_map_[y * width_ + x] / 16;
				
				p2i = x - p1 - x_off_;

				if ( p2i >= 0)
				{
					p2 = disp_map_trg_[y * width_ + p2i] / 16;

					if( abs(p1-p2) > lr_check_th_)
						disp_map_[y* width_ + x] = -8;
				}
			}
		}
	}

}


pcl::GrayStereoMatching::GrayStereoMatching()
{

}

pcl::GrayStereoMatching::~GrayStereoMatching()
{

}

void pcl::GrayStereoMatching::preProcessing(unsigned char *img, unsigned char *pp_img)
{
	//TODO: grayscale pre processing for stereo pairs
	int radius = 4;							//default value, could be exported
	int n = 2*radius+1;
	int area = n*n;
	int threshold = 127;

	int sum=0; 
	int *v = new int[width_];
	memset(v, 0, sizeof(int)*width_);

	for(int x=0; x<n; x++)
	{
		for(int y=0; y<n; y++)
		{
			v[x] += img[y*width_+x];
		}
	}

	for(int x=radius+1; x<width_-radius; x++)
	{
		for(int y=0; y<n; y++)
		{
			v[x+radius] += img[y*width_ + x+radius];
		}
	}

	for ( int y=0; y<=radius; y++)
	{
		for ( int x=0; x<width_; x++)
		{
			pp_img[y*width_+x] = img[y*width_+x];
		}
	}

	for(int y=radius+1; y<height_-radius; y++)
	{
		for ( int x = 0; x<=radius; x++)
		{
			pp_img[y*width_+x] = img[y*width_+x];
		}

		sum= 0;
		for(int x = 0; x<n; x++)
		{
			v[x] = v[x] + img[ (y+radius)*width_+x] - img[ (y-radius-1)*width_+x];
			sum += v[x];
		}

		for(int x=radius+1; x<width_-radius; x++)
		{
			v[x+radius] = v[x+radius] + img[ (y+radius)*width_ + x+radius] - img[ (y-radius-1)*width_+ x+radius];
			sum = sum + v[x+radius] - v[x-radius-1];

			short int temp = img[y*width_+x] - (sum/area);

			if(temp < -threshold)
				pp_img[y*width_+x] = 0;
			else if(temp > 	threshold)
				pp_img[y*width_+x] = threshold + threshold;
			else
				pp_img[y*width_+x] = temp + threshold;

		}

		for ( int x = width_-radius; x<width_; x++)
		{
			pp_img[y*width_+x] = img[y*width_+x];
		}
	}

	for ( int y=height_-radius; y<height_; y++)
	{
		for ( int x=0; x<width_; x++)
		{
			pp_img[y*width_+x] = img[y*width_+x];
		}
	}

	delete [] v;
}

void pcl::GrayStereoMatching::imgFlip(unsigned char * & img)
{
	unsigned char *temp_row = new unsigned char[width_];

	for (int j = 0; j < height_; j++)
	{
		memcpy(temp_row, img + j*width_, sizeof(unsigned char) * width_);
		for (int i = 0; i < width_; i++)
		{
			img[j * width_ + i] = temp_row[width_ - 1 - i];
		}
	}

	delete [] temp_row;
}

void pcl::GrayStereoMatching::compute(pcl::PointCloud<pcl::RGB> &ref, pcl::PointCloud<pcl::RGB> &trg)
{

	if (ref.width != trg.width || ref.height != trg.height)
	{

		PCL_ERROR(
			"[pcl::GrayStereoMatching::compute] Error. The two input clouds have different sizes. Aborting..\n"
		);
		return;
	}

	if ( (ref_img_ != NULL) && (width_ != ref.width || height_ != ref.height) )
	{
		delete [] ref_img_;
		delete [] trg_img_;

		ref_img_ = NULL;
		trg_img_ = NULL;
	}

	if ( ref_img_ == NULL)
	{
		ref_img_ = new unsigned char[ref.width * ref.height];	
		trg_img_ = new unsigned char[ref.width * ref.height];	
	}

	for ( unsigned int j=0; j<ref.height; j++)
	{
		for ( unsigned int i=0; i<ref.width; i++)
		{
			ref_img_[ j*ref.width + i] = ( ref[j*ref.width + i].r + ref[j*ref.width + i].g + ref[j*ref.width + i].b) / 3;
			trg_img_[ j*ref.width + i] = ( trg[j*ref.width + i].r + trg[j*ref.width + i].g + trg[j*ref.width + i].b) / 3;
			//ref_img_[ j*ref.width + i] = ( ref(j,i).r + ref(j,i).g + ref(j,i).b) / 3;
			//trg_img_[ j*ref.width + i] = ( trg(j,i).r + trg(j,i).g + trg(j,i).b) / 3;
			
		}
	}

	compute(ref_img_, trg_img_, ref.width, ref.height);

}

void pcl::GrayStereoMatching::compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height)
{
		
	//Check that a suitable value of max_disp has been selected
	if ( max_disp_ <= 0)
	{
		PCL_ERROR(
			"[pcl::StereoMatching::compute] Error. A positive max_disparity value has not be correctly inserted. Aborting..\n"
		);
		return;
	}

	if ( (disp_map_ != NULL) && (width_ != width || height_ != height) )
	{
		delete [] disp_map_;
		disp_map_ = NULL;

		if ( disp_map_trg_ != NULL)
		{
			delete [] disp_map_trg_;
			disp_map_trg_ = NULL;
		}

		if ( pp_ref_img_ != NULL)
		{
			delete [] pp_ref_img_;
			delete [] pp_trg_img_;
			pp_ref_img_ = NULL;
			pp_trg_img_ = NULL;
		}
	}

	if ( disp_map_ == NULL)
	{
		disp_map_ = new short int[width * height];	
			
		width_ = width;
		height_ = height;
	}
		

	if ( is_lr_check_ && disp_map_trg_ == NULL)
	{
		disp_map_trg_ = new short int[width * height];	
	}

	if ( !is_lr_check_ && disp_map_trg_ != NULL)
	{
		delete [] disp_map_trg_;
		disp_map_trg_ = NULL;
	}

	if ( is_pre_proc_ && pp_ref_img_ == NULL)
	{
		pp_ref_img_ = new unsigned char[width_*height_];
		pp_trg_img_ = new unsigned char[width_*height_];
	}

	if ( !is_pre_proc_ && pp_ref_img_ != NULL)
	{
		delete [] pp_ref_img_;
		delete [] pp_trg_img_;
		pp_ref_img_ = NULL;
		pp_trg_img_ = NULL;
	}

	memset(disp_map_, 0, sizeof(short int)*height_*width_);

	if ( is_pre_proc_)
	{
		preProcessing(ref_img, pp_ref_img_);
		preProcessing(trg_img, pp_trg_img_);
	}

	if (is_lr_check_)
	{

		if ( is_pre_proc_)
		{
			imgFlip(pp_ref_img_);
			imgFlip(pp_trg_img_);

			compute_impl(pp_trg_img_, pp_ref_img_);

			imgFlip(pp_ref_img_);
			imgFlip(pp_trg_img_);
		}
		else
		{
			imgFlip(ref_img);
			imgFlip(trg_img);

			compute_impl(trg_img, ref_img);

			imgFlip(ref_img);
			imgFlip(trg_img);
		}

		for (int j = 0; j < height_; j++)
			for (int i = 0; i < width_; i++)
				disp_map_trg_[j * width_ + i] = disp_map_[j * width_ + width_ - 1 - i];

	}

	if ( is_pre_proc_)
		compute_impl(pp_ref_img_, pp_trg_img_);
	else
		compute_impl(ref_img, trg_img);

	if ( is_lr_check_)
	{
		leftRightCheck();
	}

}

bool pcl::GrayStereoMatching::getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::RGB>::Ptr texture) 
{

	//disp map has not been computed yet..
	if ( disp_map_ == NULL)
	{

		PCL_ERROR(
			"[pcl::StereoMatching::getPointCloud] Error: a disparity map has not been computed yet. The resulting cloud can not be computed..\n"
		);

		return false;
	}

	
	if ( texture->width != width_ || texture->height != height_)
	{
		PCL_ERROR(
			"[pcl::StereoMatching::getPointCloud] Error: the size of the texture cloud does not match that of the computed range map. The resulting cloud can not be computed..\n"
		);

			return false;
		}

	//cloud needs to be re-allocated
	if (cloud->width != width_ || cloud->height != height_)
	{
		//cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width_, height_) );
		cloud->resize(width_*height_);
		cloud->width = width_;
		cloud->height = height_;
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
				
				temp_point.intensity = ( texture->at(j*width_+i).r +texture->at(j*width_+i).g + texture->at(j*width_+i).b) / 3.0f;
			
				cloud->at( j*width_ + i ) = temp_point;
			}
		}
	}

}

//const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl::StereoMatching::getPointCloud(float uC, float vC, float focal, float baseline)
bool pcl::GrayStereoMatching::getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	
	//disp map has not been computed yet..
	if ( disp_map_ == NULL)
	{

		PCL_ERROR(
			"[pcl::StereoMatching::getPointCloud] Error: a disparity map has not been computed yet. The resulting cloud can not be computed..\n"
		);

		return false;
	}

	//cloud needs to be re-allocated
	if (cloud->width != width_ || cloud->height != height_)
	{
		//cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width_, height_) );
		cloud->resize(width_*height_);
		cloud->width = width_;
		cloud->height = height_;
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
				temp_point.intensity = 255;

				cloud->at(j*width_ + i) = temp_point;
			}
		}
	}

	return true;
}

//TODO
//pcl::AdaptiveCostSOStereoMatching::AdaptiveCostSOStereoMatching()
//{
//
//}
//
//void pcl::AdaptiveCostSOStereoMatching::compute_impl(unsigned char* ref_img, unsigned char* trg_img)
//{
//
//	//TODO: body of the specific stereo matching algorithm
//}
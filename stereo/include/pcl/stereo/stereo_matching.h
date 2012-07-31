
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
 */



#ifndef PCL_STEREO_H_
#define PCL_STEREO_H_

//#include <pcl/io/grabber.h>
//#include <pcl/common/time_trigger.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>

namespace pcl
{
	/** \brief Stereo Matching abstract class 
	*
	* The class performs stereo matching on a rectified stereo pair 
	* Includes the following functionalities:
	*	* preprocessing of the image pair, to improve robustness against photometric distortions
	*		(wrt. to a spatially constant additive photometric factor)
	*	* postprocessing: filtering of wrong disparities via Peak Filter (eliminating ambiguities due to low-textured regions) 
	*		and Ratio Filter (eliminating generic matching ambiguities, similar to that present in OpenCV Block Matching Stereo)
	*	* postprocessing: Left-Right consistency check (eliminates wrong disparities at the cost of twice the stereo matching 
	*		computation)
	*	* postprocessing: subpixel refinement of computed disparities, to reduce the depth quantization effect
	*	* postprocessing: smoothing of the disparity map via median filter
	*	* after stereo matching a PCL point cloud can be computed, given the stereo intrinsic (focal, principal point  
	*		coordinates) and extrinsic (baseline) calibration parameters
	*
    * \author Federico Tombari (federico.tombari@unibo.it)
    * \ingroup stereo
    */

	class PCL_EXPORTS StereoMatching
	{

	public:
		
		StereoMatching(void);

		virtual ~StereoMatching(void);

		void 
		setMaxDisparity(int max_disp){ 
			max_disp_ = max_disp;
		};
		
		void 
		setXOffset(int x_off){ 
			x_off_ = x_off; 
		};

		void 
		setRatioFilter(int ratio_filter){ 
			ratio_filter_ = ratio_filter;
		};
		
		void 
		setPeakFilter(int peak_filter){ 
			peak_filter_ = peak_filter;
		};

		void 
		setPreProcessing(bool is_pre_proc){ 
			is_pre_proc_ = is_pre_proc;
		};
		
		void 
		setLeftRightCheck(bool is_lr_check){ 
			is_lr_check_ = is_lr_check;
		};
		
		void 
		setLeftRightCheckThreshold(int lr_check_th){ 
			lr_check_th_ = lr_check_th;
		};

		virtual void 
		preProcessing(unsigned char *img, unsigned char *pp_img) = 0;

		virtual void 
		compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height) = 0;

		void 
		medianFilter(int radius);

		//should the cloud be handled by the StereoMatching class or should it be left to the user?
		//const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud(float uC, float vC, float focal, float baseline);
		virtual void 
		getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZI> &cloud, unsigned char *ref_img = NULL) = 0;

		void 
		getVisualMap(unsigned char *&map);

	protected:

		short int *disp_map_;

		//used for lr check
		short int *disp_map_trg_;

		//used for pre processing
		unsigned char* pp_ref_img_;
		unsigned char* pp_trg_img_;

		int width_;
		int height_;

		int max_disp_;
		int x_off_;

		int ratio_filter_;
		int peak_filter_;

		bool is_pre_proc_;
		bool is_lr_check_;
		int lr_check_th_;

		virtual void 
		imgFlip(unsigned char * & img) = 0;

		virtual void 
		compute_impl(unsigned char* ref_img, unsigned char* trg_img) = 0;

		void leftRightCheck();

		inline short int 
		computeStereoSubpixel(int dbest, int s1, int s2, int s3)
		{
			int den = (s1+s3-2*s2);
			if(den!=0)
				return (short int) (16*dbest + (((s1 - s3)*8) / den));
			else
				return (short int)(dbest*16);
		}

		inline short int 
		doStereoRatioFilter(int *acc, short int dbest, int sad_min, int ratio_filter, int maxdisp, int precision=100)
		{

			int sad_second_min = std::numeric_limits<int>::max();

			for(int d=0; d<dbest-1; d++)
				if(acc[d]<sad_second_min)
					sad_second_min = acc[d];

			for(int d=dbest+2; d<maxdisp; d++)
				if(acc[d]<sad_second_min)
					sad_second_min = acc[d];

			if(sad_min*precision  > (precision-ratio_filter)*sad_second_min)
				return -2;
			else	
				return dbest;
		}

		inline short int 
		doStereoPeakFilter(int *acc, short int dbest, int peak_filter, int maxdisp)
		{
			int da = (dbest>1) ? ( acc[dbest-2] - acc[dbest] ) : (acc[dbest+2] - acc[dbest]);
			int db =  (dbest<maxdisp-2) ? (acc[dbest+2] - acc[dbest]) : (acc[dbest-2] - acc[dbest]);	

			if(da + db < peak_filter)
				return -4;
			else
				return dbest;
		}

	};

	/** \brief Stereo Matching abstract class for Grayscale images 
	*
	* The class implements some functionalities of pcl::StereoMatching specific for grayscale stereo processing
	*
    * \author Federico Tombari (federico.tombari@unibo.it)
    * \ingroup stereo
    */

	class PCL_EXPORTS GrayStereoMatching : public StereoMatching
	{
	public:

		GrayStereoMatching(void);
		virtual ~GrayStereoMatching(void);

		virtual void 
		compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height);

		virtual void 
		getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZI> &cloud, unsigned char *ref_img = NULL);

	protected:

		virtual void 
		compute_impl(unsigned char* ref_img, unsigned char* trg_img) = 0;

		virtual void 
		preProcessing(unsigned char *img, unsigned char *pp_img);

		virtual void 
		imgFlip(unsigned char * & img);

	};

	/** \brief Block based (or fixed window) Stereo Matching class
	*
	* This class implements the baseline Block-based - aka Fixed Window -  stereo matching algorithm.
	* The algorithm includes a running box filter so that the computational complexity is independent of 
	*	the size of the window ( O(1) wrt. to the size of window)
	* The algorithm is based on the Sum of Absolute Differences (SAD) matching function
	* Only works with grayscale (single channel) rectified images
	*
    * \author Federico Tombari (federico.tombari@unibo.it)
    * \ingroup stereo
    */

	class PCL_EXPORTS BlockBasedStereoMatching : public GrayStereoMatching
	{
	public:

		BlockBasedStereoMatching(void);

		virtual ~BlockBasedStereoMatching(void) 
		{
		};

		void 
		setRadius(int radius)
		{
			radius_=radius;
		};
		

	private:

		virtual void 
		compute_impl(unsigned char* ref_img, unsigned char* trg_img);

		int radius_;

	};

	// TODO
	//class PCL_EXPORTS AdaptiveCostSOStereoMatching : public GrayStereoMatching
	//{
	//public:

	//	AdaptiveCostSOStereoMatching(void);

	//	virtual ~AdaptiveCostSOStereoMatching(void) {};

	//private:

	//	virtual void compute_impl(unsigned char* ref_img, unsigned char* trg_img);
	//};

}

#endif

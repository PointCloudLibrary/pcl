
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

/** \brief Stereo Matching class performing stereo match on a rectified stereo pair and returning a point cloud
    * \author Federico Tombari
    * \ingroup stereo
    */


#ifndef PCL_STEREO_H_
#define PCL_STEREO_H_

//#include <pcl/io/grabber.h>
//#include <pcl/common/time_trigger.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>

namespace pcl
{
	class PCL_EXPORTS StereoMatching
	{
    public:
      
      StereoMatching (void);
      virtual ~StereoMatching (void);

      void 
      setMaxDisparity (int max_disp) { max_disp_ = max_disp; }
      
      void 
      setXOffset (int x_off) { x_off_ = x_off; }

      void 
      setPreProcessing (bool is_pre_proc) { is_pre_proc_ = is_pre_proc; }
      void 
      setLeftRightCheck (bool is_lr_check) { is_lr_check_ = is_lr_check; }

      virtual void 
      preProcessing (unsigned char *ref, unsigned char *right) = 0;

      virtual void 
      compute (unsigned char* ref_img, unsigned char* trg_img, int width, int height) = 0;

      void 
      medianFilter (int radius);

      //should the cloud be handled by the StereoMatching class or should it be left to the user?
      //const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud(float uC, float vC, float focal, float baseline);
      virtual void 
      getPointCloud (float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZI> &cloud, unsigned char *ref_img = NULL) = 0;

    protected:

      float *disp_map_;
      float *disp_map_trg_;

      int width_;
      int height_;

      int max_disp_;
      int x_off_;

      int ratio_filter_;

      bool is_pre_proc_;
      bool is_lr_check_;

      //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_; 

      virtual void 
      compute_impl (unsigned char* ref_img, unsigned char* trg_img) = 0;

      void 
      leftRightCheck (float* map_ref, float* map_trg);

	};

	class PCL_EXPORTS GrayStereoMatching : public StereoMatching
	{
    public:

      GrayStereoMatching(void);
      virtual ~GrayStereoMatching(void);

      virtual void 
      compute (unsigned char* ref_img, unsigned char* trg_img, int width, int height);

      virtual void 
      getPointCloud (float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZI> &cloud, unsigned char *ref_img = NULL);

    protected:

      virtual void 
      compute_impl (unsigned char* ref_img, unsigned char* trg_img) = 0;
      
      virtual void 
      preProcessing (unsigned char *ref, unsigned char *right);

	};

	class PCL_EXPORTS AdaptiveCostSOStereoMatching : public GrayStereoMatching
	{
    public:

      AdaptiveCostSOStereoMatching(void);

      virtual ~AdaptiveCostSOStereoMatching(void) {};

    protected:

      virtual void 
      compute_impl (unsigned char* ref_img, unsigned char* trg_img);
	};
}

#endif

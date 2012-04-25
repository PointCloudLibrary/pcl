/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
* @author: Koen Buys, Anatoly Baksheev
*/

#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/label_common.h>

#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/label_conversion.h>
#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>
#include "internal.h"

#include <pcl/common/time.h>

#define AREA_THRES      200 // for euclidean clusterization 1 
#define AREA_THRES2     100 // for euclidean clusterization 2 
#define CLUST_TOL_SHS   0.05
#define DELTA_HUE_SHS   5

using namespace std;
using namespace pcl;
using namespace pcl::gpu::people;

void optimized_shs2(const PointCloud<PointXYZRGB> &cloud, float tolerance, const PointIndices &indices_in, PointIndices &indices_out, float delta_hue);
void optimized_shs3(const PointCloud<PointXYZRGB> &cloud, float tolerance, const PointIndices &indices_in, PointIndices &indices_out, float delta_hue);
void optimized_shs4(const PointCloud<PointXYZ> &cloud, const float *hue, float tolerance, const PointIndices &indices_in, cv::Mat flowermat, float delta_hue);
void optimized_shs5(const PointCloud<PointXYZ> &cloud, const float *hue, float tolerance, const PointIndices &indices_in, cv::Mat flowermat, float delta_hue);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::people::PeopleDetector::PeopleDetector() 
    : fx_(525.f), fy_(525.f), cx_(319.5f), cy_(239.5f), delta_hue_tolerance_(5), do_shs_(true)
{
  // allocation buffers with default sizes
  // if input size is other than the defaults, 
  // then the buffers will be reallocated at processing time.
  // This cause only penalty for first frame ( one reallocation of each buffer )
  allocate_buffers();
}

void
pcl::gpu::people::PeopleDetector::setIntrinsics (float fx, float fy, float cx, float cy)
{
  fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
}

void pcl::gpu::people::PeopleDetector::allocate_buffers(int rows, int cols)
{ 
  device::Dilatation::prepareRect5x5Kernel(kernelRect5x5_);  

  cloud_host_.width  = cols;
  cloud_host_.height = rows;
  cloud_host_.points.resize(cols * rows);
  cloud_host_.is_dense = false;

  hue_host_.width  = cols;
  hue_host_.height = rows;
  hue_host_.points.resize(cols * rows);
  hue_host_.is_dense = false;

  depth_host_.width  = cols;
  depth_host_.height = rows;
  depth_host_.points.resize(cols * rows);
  depth_host_.is_dense = false;

  cloud_device_.create(rows, cols);
  hue_device_.create(rows, cols);

  depth_device1_.create(rows, cols);
  depth_device2_.create(rows, cols);
  fg_mask_.create(rows, cols);
  fg_mask_grown_.create(rows, cols);

  
}

void pcl::gpu::people::PeopleDetector::process(const Depth& depth, const Image& rgba)
{  
  allocate_buffers(depth.rows(), depth.cols());

  depth_device1_ = depth;

  const device::Image& i = (const device::Image&)rgba;
  device::computeHueWithNans(i, depth_device1_, hue_device_);  
  //TODO compute cloud device on GPU using intrinsics; 
  //TODO download cloud as some part of the algorithm are CPU only
  
  //TODO Hope this is temporary and after porting to GPU the download will be deleted
  int c;
  hue_device_.download(hue_host_.points, c);
  
  // uses cloud device, cloud host, depth device, hue device and other buffers
  process();
}

void
pcl::gpu::people::PeopleDetector::process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  allocate_buffers(cloud->height, cloud->width);

  const float qnan = std::numeric_limits<float>::quiet_NaN();

  for(size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud_host_.points[i].x = cloud->points[i].x;
    cloud_host_.points[i].y = cloud->points[i].y;
    cloud_host_.points[i].z = cloud->points[i].z;

    bool valid = isFinite(cloud_host_.points[i]);

    hue_host_.points[i] = !valid ? qnan : device::computeHue(cloud->points[i].rgba);
    depth_host_.points[i] = !valid ? 0 : static_cast<unsigned short>(cloud_host_.points[i].z * 1000); //m -> mm
  }
  cloud_device_.upload(cloud_host_.points, cloud_host_.width);
  hue_device_.upload(hue_host_.points, hue_host_.width);
  depth_device1_.upload(depth_host_.points, depth_host_.width);


  // uses cloud device, cloud host, depth device, hue device and other buffers
  process();
}

void
pcl::gpu::people::PeopleDetector::process ()
{
  int cols = cloud_device_.cols();
  int rows = cloud_device_.rows();
      
  RDFBodyPartsDetector::BlobMatrix sorted;

  {
    ScopeTime time("ev1");
    rdf_detector_->computeLabels(depth_device1_);
    rdf_detector_->step2_selectBetterName(cloud_host_, AREA_THRES, sorted);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // if we found a neck display the tree, and continue with processing
  if(sorted[Neck].size() != 0)
  {
    int c = 0;
    label_skeleton::Tree2 t;
    label_skeleton::buildTree(sorted, cloud_host_, Neck, c, t);
    
    const pcl::PointIndices& seed = t.indices;
    
    cv::Mat flowermat(rows, cols, CV_8U, cv::Scalar(0));    
    {
      ScopeTime time("shs");    
      optimized_shs5(cloud_host_, &hue_host_.points[0], CLUST_TOL_SHS, seed, flowermat, DELTA_HUE_SHS);
    }
    
    fg_mask_.upload(flowermat.data, flowermat.step, rows, cols);
    device::Dilatation::invoke(fg_mask_, kernelRect5x5_, fg_mask_grown_);

    device::prepareForeGroundDepth(depth_device1_, fg_mask_grown_, depth_device2_);

    //// //////////////////////////////////////////////////////////////////////////////////////////////// //
    //// The second label evaluation

    RDFBodyPartsDetector::BlobMatrix sorted2;
    {
      ScopeTime time("ev2");
      rdf_detector_->computeLabels(depth_device2_);
      rdf_detector_->step2_selectBetterName(cloud_host_, AREA_THRES2, sorted2);
    }

    //brief Test if the second tree is build up correctly
    if(sorted2[Neck].size() != 0)
    {
      ScopeTime time("bt");
      label_skeleton::Tree2 t2;
      label_skeleton::buildTree(sorted2, cloud_host_, Neck, c, t2);
      int par = 0;
      for(int f = 0; f < NUM_PARTS; f++)
      {
        if(t2.parts_lid[f] == NO_CHILD)
        {
          cerr << "1;";
          par++;
        }
        else
           cerr << "0;";
      }
      static int counter = 0; // TODO move this logging to PeopleApp
      cerr << t2.nr_parts << ";" << par << ";" << t2.total_dist_error << ";" << t2.norm_dist_error << ";" << counter++ << ";" << endl;
    }

    //output: Tree2 and PointCloud<XYZRGBL> 
  }
}


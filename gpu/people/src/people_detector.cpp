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
void optimized_shs4(const PointCloud<PointXYZRGB> &cloud, float tolerance, const PointIndices &indices_in, cv::Mat flowermat, float delta_hue);

void pcl::gpu::people::PeopleDetector::allocate_buffers(int rows, int cols)
{
  // allocation buffers with default sizes
  // if input size is other than the defaults, 
  // then the buffers will be reallocated at processing time.
  // This cause only penalty for first frame ( one reallocation of each buffer )

  cloud_device_.create(rows * cols);

  depth_device_.create(rows, cols);
  depth_device2_.create(rows, cols);
  fg_mask_.create(rows, cols);
  fg_mask_grown_.create(rows, cols);

  device::Dilatation::prepareRect5x5Kernel(kernelRect5x5_);
}

void
pcl::gpu::people::PeopleDetector::process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  int cols = cloud->width;
  int rows = cloud->height;

  allocate_buffers(rows, cols);

  // Bring the pointcloud to the GPU memory
  cloud_device_.upload(cloud->points);

  // Convert the float z values to unsigned shorts, also converts from m to mm
  const DeviceArray<device::float8>& c = (const DeviceArray<device::float8>&)cloud_device_;
  device::convertCloud2Depth(c, rows, cols, depth_device_);

  RDFBodyPartsDetector::BlobMatrix sorted;

  {
    ScopeTime time("ev1");
    rdf_detector_->computeLabels(depth_device_);
    rdf_detector_->step2_selectBetterName(cloud, AREA_THRES, sorted);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // if we found a neck display the tree, and continue with processing
  if(sorted[Neck].size() != 0)
  {
    int c = 0;
    label_skeleton::Tree2 t;
    label_skeleton::buildTree(sorted, *cloud, Neck, c, t);

    cv::Mat mask(rows, cols, CV_8UC1, cv::Scalar(0));

    label_skeleton::makeFGMaskFromPointCloud(mask, t.indices, *cloud);

    const pcl::PointIndices& seed = t.indices;
    // //////////////////////////////////////////////////////////////////////////////////////////////// //
    // The second kdtree evaluation = seeded hue segmentation
    // Reuse the fist searchtree for this, in order to NOT build it again!

    cv::Mat flowermat(rows, cols, CV_8U, cv::Scalar(0));

    //pcl::PointIndices flower;
    {
      ScopeTime time("shs");
    //pcl::seededHueSegmentation(cloud_in, stree, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);
    //optimized_shs3(*cloud, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);
      optimized_shs4(*cloud, CLUST_TOL_SHS, seed, flowermat, DELTA_HUE_SHS);
    }

    //for(size_t i = 0; i < flower.indices.size(); i++)
    //{
    //  int index = flower.indices[i];
    //  unsigned int y = index / cloud->width;
    //  unsigned int x = index % cloud->width;
    //  flowermat.at<unsigned char>(y,x) = 255;
    //}

    fg_mask_.upload(flowermat.data, flowermat.step, rows, cols);
    device::Dilatation::invoke(fg_mask_, kernelRect5x5_, fg_mask_grown_);

    device::prepareForeGroundDepth(depth_device_, fg_mask_grown_, depth_device2_);

    //// //////////////////////////////////////////////////////////////////////////////////////////////// //
    //// The second label evaluation

    RDFBodyPartsDetector::BlobMatrix sorted2;
    {
      ScopeTime time("ev2");
      rdf_detector_->computeLabels(depth_device2_);
      rdf_detector_->step2_selectBetterName(cloud, AREA_THRES2, sorted2);
    }

    //brief Test if the second tree is build up correctly
    if(sorted2[Neck].size() != 0)
    {
      ScopeTime time("bt");
      label_skeleton::Tree2 t2;
      label_skeleton::buildTree(sorted2, *cloud, Neck, c, t2);
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


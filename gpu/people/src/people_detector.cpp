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

#define AREA_THRES      200 // for euclidean clusterization 1 
#define AREA_THRES2     100 // for euclidean clusterization 2 
#define CLUST_TOL_SHS   0.05
#define DELTA_HUE_SHS   5

using namespace std;
using namespace pcl;
using namespace pcl::gpu::people;

void optimized_shs2(const PointCloud<PointXYZRGB> &cloud, float tolerance, const PointIndices &indices_in, PointIndices &indices_out, float delta_hue);
void optimized_shs3(const PointCloud<PointXYZRGB> &cloud, float tolerance, const PointIndices &indices_in, PointIndices &indices_out, float delta_hue);

namespace
{
  void 
  testIfCorrectTree(const PointCloud<PointXYZRGB>::ConstPtr &cloud, const RDFBodyPartsDetector::BlobMatrix& sorted, int c)
  {    
    static int counter = 0;
    ++counter;

    //brief Test if the second tree is build up correctly
    if(sorted[Neck].size() != 0)
    {
      label_skeleton::Tree2 t2;
      label_skeleton::buildTree(sorted, *cloud, Neck, c, t2);
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
      cerr << t2.nr_parts << ";" << par << ";" << t2.total_dist_error << ";" << t2.norm_dist_error << ";" << counter << ";" << endl;
    }
  }
}

void
pcl::gpu::people::PeopleDetector::process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
    int cols = cloud->width;
    int rows = cloud->height;

    // Bring the pointcloud to the GPU memory
    cloud_device_.upload(cloud->points);

    // Convert the float z values to unsigned shorts, also converts from m to mm
    const DeviceArray<device::float8>& c = (const DeviceArray<device::float8>&)cloud_device_;
    device::convertCloud2Depth(c, rows, cols, depth_device_);    
             
    rdf_detector_->computeLabels(depth_device_);

    // Create a new struct to put the results in
    RDFBodyPartsDetector::BlobMatrix sorted;    
    rdf_detector_->step2_selectBetterName(cloud, AREA_THRES, sorted);
     
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
      pcl::PointIndices flower;
      //pcl::seededHueSegmentation(cloud_in, stree, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);
      optimized_shs2(*cloud, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);

      cv::Mat flowermat(rows, cols, CV_8UC3, cv::Scalar(0));
      label_skeleton::makeImageFromPointCloud(flowermat, flower, *cloud);

#ifdef WRITE
      pngwrite("f_",counter_, flowermat);
#endif
      cv::Mat flowergrownmat(rows, cols, CV_8UC3, cv::Scalar(0));               
      cv::dilate(flowermat, flowergrownmat, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

      //cv::Mat mask7;
      //cv::cvtColor(flowergrownmat, mask7, CV_BGR2GRAY);
      //cv::threshold(mask7, mask7, 0, 255, cv::THRESH_BINARY);

      //DeviceArray2D<unsigned char> mask7_dev;
      //mask7_dev.upload(mask7.ptr(), mask7.step, mask7.rows, mask7.cols);      

      cv::Mat dmat(rows, cols, CV_16U);
      depth_device_.download(dmat.ptr<unsigned short>(), dmat.step);

      //device::prepareForeGroundDepth(depth_device_, mask7_dev, depth_device2_);

      cv::Mat dmat2(rows, cols, CV_16U);
      for(int v = 0; v < rows; v++)
      {
        for(int u = 0; u < cols; u++)
        {
          bool cond = flowergrownmat.at<cv::Vec3b>(v,u)[0] != 0 || flowergrownmat.at<cv::Vec3b>(v,u)[1] != 0 || flowergrownmat.at<cv::Vec3b>(v,u)[2] != 0;
              
          dmat2.at<short>(v,u) = cond ? dmat.at<short>(v,u) : std::numeric_limits<short>::max();
        }
      }
      
      depth_device2_.upload(dmat2.ptr<unsigned short>(), dmat2.step, dmat2.rows, dmat2.cols);

      //// //////////////////////////////////////////////////////////////////////////////////////////////// //
      //// The second label evaluation

      // Process the depthimage        
      rdf_detector_->computeLabels(depth_device2_);                        
      RDFBodyPartsDetector::BlobMatrix sorted2;            
      rdf_detector_->step2_selectBetterName(cloud, AREA_THRES2, sorted2);
                
      testIfCorrectTree(cloud, sorted2, c);                
    }
    // This is kept to count the number of process steps have been taken    
}


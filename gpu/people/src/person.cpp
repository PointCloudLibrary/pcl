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

#include <pcl/gpu/people/person.h>
#include <pcl/gpu/people/label_common.h>
#include <pcl/gpu/utils/repacks.hpp>

#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/label_conversion.h>
#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>
#include "tree_live.h"

#include <pcl/io/pcd_io.h>

#include "internal.h"

#include <boost/lexical_cast.hpp>

#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#define AREA_THRES      200 // for euclidean clusterization 1 
#define AREA_THRES2     100 // for euclidean clusterization 2 
#define CLUST_TOL_SHS   0.05
#define DELTA_HUE_SHS   5

//#define WRITE

using namespace boost;
using namespace std;
using namespace pcl;;

void optimized_elec(const PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat& src_labels, float tolerance,
                    std::vector<std::vector<PointIndices> > &labeled_clusters,
                    unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster, unsigned int num_parts,
                    bool brute_force_border, float radius_scale);

void optimized_shs2(const PointCloud<PointXYZRGB> &cloud, float tolerance, PointIndices &indices_in, PointIndices &indices_out, float delta_hue);


namespace cv { template<> class DataType<pcl::RGB> { public: enum { type = CV_8UC4 }; }; }
namespace temporary
{
    void pngwrite(const std::string& prefix, int counter, cv::Mat& host)
    {  
      cv::imwrite(prefix + lexical_cast<string>(counter) + ".png", host);
    }
   
    template<typename T> void pngwrite(const std::string& prefix, int counter, const pcl::gpu::DeviceArray2D<T>& arr)
    {
      cv::Mat host(arr.rows(), arr.cols(), cv::DataType<T>::type);
      arr.download(host.data, host.step);
      pngwrite(prefix, counter, host);
    }
}
using namespace temporary;



void
pcl::gpu::people::Person::process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
    int cols = cloud->width;
    int rows = cloud->height;

    // Bring the pointcloud to the GPU memory
    cloud_device_.upload(cloud->points);

    // Convert the float z values to unsigned shorts, also converts from m to mm
    const DeviceArray<device::float8>& c = (const DeviceArray<device::float8>&)cloud_device_;
    device::convertCloud2Depth(c, rows, cols, depth_device_);

    /// @todo rewrite this to a pointcloud::Ptr
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in;    
    cloud_in = *cloud;
         
    // Process the depthimage (CUDA)    
    rdf_detector_->process(depth_device_);
    
#if !defined(WRITE)
    /// how to save it now
    RDFBodyPartsDetector::Image colores_labels;
    rdf_detector_->colorizeLabels(rdf_detector_->getLabels(), colores_labels);
    pngwrite("s1_", counter_, rdf_detector_->getLabels());
    pngwrite("c1_", counter_, colores_labels);

#endif
    // Create a new struct to put the results in
    RDFBodyPartsDetector::BlobMatrix sorted;    
    rdf_detector_->step2_selectBetterName(cloud, AREA_THRES, sorted);


    // ////////////////////////////////////////////////////////////////////////////////////////// //
    // DISPLAY INTERMEDIATE RESULTS UPTILL FINDING OF THE TREES, NOT EVALUATING TREES YET
#if defined(WRITE)
    // color
    pcl::gpu::people::display::colorLMap( lmap, cmap );
    pngwrite("c_", counter_, cmap);
#endif

    // ////////////////////////////////////////////////////////////////////////////////////////////// //
    // if we found a neck display the tree, and continue with processing
    if(sorted[10].size() != 0)
    {
        unsigned int c = 0;
        label_skeleton::Tree2 t;
        label_skeleton::buildTree(sorted, cloud_in, Neck, c, t);

        cv::Mat mask(cloud_in.height, cloud_in.width, CV_8UC1, cv::Scalar(0));

        label_skeleton::makeFGMaskFromPointCloud(mask, t.indices, cloud_in);

        pcl::PointIndices seed;
#if defined(DISPL_BINMASK) || defined(WRITE)
        cv::Mat binmask(cloud_in.height, cloud_in.width, CV_8UC3, cv::Scalar(0));
#endif
        for(unsigned int v = 0; v < cloud_in.height; v++)
        {
            for(unsigned int u = 0; u < cloud_in.width; u++)
            {
                if(mask.at<char>(v,u) == cv::GC_PR_FGD)
                {
#if defined(DISPL_BINMASK) || defined(WRITE)
                    binmask.at<cv::Vec3b>(v,u)[0] = cloud_in.points[v*cloud_in.width + u].b;
                    binmask.at<cv::Vec3b>(v,u)[1] = cloud_in.points[v*cloud_in.width + u].g;
                    binmask.at<cv::Vec3b>(v,u)[2] = cloud_in.points[v*cloud_in.width + u].r;
#endif
                    seed.indices.push_back(v*cloud_in.width + u);
                }
            }
        }
#ifdef WRITE
        pngwrite("b_", counter_, binmask);
#endif

        // //////////////////////////////////////////////////////////////////////////////////////////////// //
        // The second kdtree evaluation = seeded hue segmentation
        // Reuse the fist searchtree for this, in order to NOT build it again!
        pcl::PointIndices flower;
        //pcl::seededHueSegmentation(cloud_in, stree, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);
        optimized_shs2(cloud_in, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);

        cv::Mat flowermat(cloud_in.height, cloud_in.width, CV_8UC3, cv::Scalar(0));
        label_skeleton::makeImageFromPointCloud(flowermat, flower, cloud_in);

#ifdef WRITE
        pngwrite("f_",counter_, flowermat);
#endif
        cv::Mat flowergrownmat(rows, cols, CV_8UC3, cv::Scalar(0));               
        cv::dilate(flowermat, flowergrownmat, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
#ifdef WRITE
        pngwrite("g_", counter_, flowergrownmat);
#endif

        cv::Mat dmat(rows, cols, CV_16U);
        depth_device_.download(dmat.ptr<unsigned short>(), dmat.step);

        cv::Mat dmat2(rows, cols, CV_16U);
        for(int v = 0; v < rows; v++)
        {
            for(int u = 0; u < cols; u++)
            {
                bool cond = flowergrownmat.at<cv::Vec3b>(v,u)[0] != 0 || flowergrownmat.at<cv::Vec3b>(v,u)[1] != 0 || flowergrownmat.at<cv::Vec3b>(v,u)[2] != 0;
                
                dmat2.at<short>(v,u) = cond ? dmat.at<short>(v,u) : std::numeric_limits<short>::max();
            }
        }

        // //////////////////////////////////////////////////////////////////////////////////////////////// //
        // The second label evaluation

        depth_device2_.upload(dmat2.ptr<unsigned short>(), dmat2.step, dmat2.rows, dmat2.cols);

        // Process the depthimage        
        rdf_detector_->process(depth_device2_);                        
        RDFBodyPartsDetector::BlobMatrix sorted2;            
        rdf_detector_->step2_selectBetterName(cloud, AREA_THRES2, sorted2);

        
        rdf_detector_->colorizeLabels(rdf_detector_->getLabels(), cmap_device_);                                             

#if !defined(WRITE)
        pngwrite("d2_", counter_, depth_device2_);        
        pngwrite("s2_", counter_, rdf_detector_->getLabels());
        RDFBodyPartsDetector::Image colores_labels2;
        rdf_detector_->colorizeLabels(rdf_detector_->getLabels(), colores_labels2);
        pngwrite("c2_", counter_, colores_labels2);
#endif


        // Test if the second tree is build up correctly
        if(sorted2[10].size() != 0)
        {
            label_skeleton::Tree2 t2;
            label_skeleton::buildTree(sorted, cloud_in, Neck, c, t2);
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
            cerr<< t2.nr_parts << ";" << par << ";" << t2.total_dist_error << ";" << t2.norm_dist_error << ";" << counter_ << ";" << endl;
        }
    }
    // This is kept to count the number of process steps have been taken
    counter_++;
}


void
pcl::gpu::people::Person::readPersonXMLConfig (std::istream& is)
{
  boost::property_tree::ptree pt;
  read_xml(is,pt);
}

void
pcl::gpu::people::Person::writePersonXMLConfig (std::ostream& os)
{
  boost::property_tree::ptree pt;
  pt.add("version", XML_VERSION);
//  boost::property_tree::ptree& node = pt.add("person", "");
//  node.put("name", name_);
  pt.add("person.name", name_);

  write_xml(os,pt);
}

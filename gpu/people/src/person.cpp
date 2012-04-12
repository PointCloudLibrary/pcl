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

#ifndef PCL_GPU_PEOPLE_PERSON_HPP_
#define PCL_GPU_PEOPLE_PERSON_HPP_

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


#define AREA_THRES      200
#define AREA_THRES2     100
#define CLUST_TOL       0.05
#define CLUST_TOL_SHS   0.05
#define DELTA_HUE_SHS   5
#define MAX_CLUST_SIZE  25000
#define WRITE



using namespace boost;
using namespace std;
using namespace pcl;;

void optimized_elec(const PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat& src_labels, float tolerance,
                    std::vector<std::vector<PointIndices> > &labeled_clusters,
                    unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster, unsigned int num_parts,
                    bool brute_force_border, float radius_scale);

void optimized_shs2(const PointCloud<PointXYZRGB> &cloud, float tolerance, PointIndices &indices_in, PointIndices &indices_out, float delta_hue);

void
pcl::gpu::people::Person::process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
    static int counter = 0;
    ++counter;

    cloud_device_.upload(cloud->points);

    const DeviceArray<device::float8>& c = (const DeviceArray<device::float8>&)cloud_device_;
    device::convertCloud2Depth(c, cloud->height, cloud->width, depth_device_);
    
    /// @todo rewrite this to a pointcloud::Ptr
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in_filt;
    cloud_in = *cloud;

    cv::Mat dmat(cloud_in.height, cloud_in.width, CV_16U);

    // Project pointcloud back into the imageplane
    // TODO: do this directly in GPU?
    pcl::gpu::people::label_skeleton::makeDepthImage16FromPointCloud(dmat, cloud_in);
    depth_device_.download(dmat.ptr<unsigned short>(), dmat.step);

    // Process the depthimage (CUDA)
    //m_proc->process(dmat, m_lmap);    
    //m_proc->process(depth_device_, m_lmap);
    m_proc->process(depth_device_, lmap_device_);
    m_lmap.create(depth_device_.rows(), depth_device_.cols(), CV_8U);
    lmap_device_.download(m_lmap.data, m_lmap.step);
    
    cv::Mat lmap(cloud_in.height, cloud_in.width, CV_8UC1);
    pcl::gpu::people::label_skeleton::smoothLabelImage(m_lmap, dmat, lmap);

#ifdef WRITE
    cv::imwrite("d_" + lexical_cast<string>(counter) + ".png", dmat);
    cv::imwrite("l_" + lexical_cast<string>(counter) + ".png", m_lmap);
    cv::imwrite("s_" + lexical_cast<string>(counter) + ".png", lmap);

    cv::Mat input(cloud_in.height, cloud_in.width, CV_8UC3);
    pcl::gpu::people::label_skeleton::makeImageFromPointCloud(input, cloud_in);
    cv::imwrite("i_" + lexical_cast<string>(counter) + ".png", input);
#endif

    pcl::PointCloud<pcl::PointXYZRGBL> cloud_labels;

    pcl::gpu::people::conversion::colorLabelPointCloudFromArray(cloud_in, lmap.data, cloud_labels);
    /*
    // Creating the Search object for the search method of the extraction
    pcl::search::OrganizedNeighbor<pcl::PointXYZRGBL>::Ptr stree (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBL>);
    stree->setInputCloud(cloud_labels.makeShared());
    */
    std::vector<std::vector<pcl::PointIndices> > cluster_indices(NUM_PARTS);
    
    // Make all the clusters
    optimized_elec(cloud_in, lmap, CLUST_TOL, cluster_indices, AREA_THRES, MAX_CLUST_SIZE, NUM_PARTS, false, 1.f);

    // Create a new struct to put the results in
    std::vector<std::vector<label_skeleton::Blob2, Eigen::aligned_allocator<label_skeleton::Blob2> > >       sorted;
    //clear out our matrix before starting again with it
    sorted.clear();
    //Set fixed size of outer vector length = number of parts
    sorted.resize(NUM_PARTS);

    //create the blob2 matrix
    pcl::gpu::people::label_skeleton::sortIndicesToBlob2 ( cloud_labels, AREA_THRES, sorted, cluster_indices );
    //Build relationships between the blobs
    pcl::gpu::people::label_skeleton::buildRelations ( sorted );

    // ////////////////////////////////////////////////////////////////////////////////////////// //
    // DISPLAY INTERMEDIATE RESULTS UPTILL FINDING OF THE TREES, NOT EVALUATING TREES YET
#if defined(WRITE)
    // color
    pcl::gpu::people::display::colorLMap( lmap, cmap );
    cv::imwrite("c_" + lexical_cast<string>(counter) + ".png", cmap);
#endif

    // ////////////////////////////////////////////////////////////////////////////////////////////// //
    // if we found a neck display the tree, and continue with processing
    if(sorted[10].size() != 0)
    {
        unsigned int c = 0;
        pcl::gpu::people::label_skeleton::Tree2 t;
        pcl::gpu::people::label_skeleton::buildTree(sorted, cloud_in, Neck, c, t);

        cv::Mat mask(cloud_in.height, cloud_in.width, CV_8UC1, cv::Scalar(0));

        pcl::gpu::people::label_skeleton::makeFGMaskFromPointCloud(mask, t.indices, cloud_in);

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
        cv::imwrite("b_" + lexical_cast<string>(counter) + ".png", binmask);
#endif

        // //////////////////////////////////////////////////////////////////////////////////////////////// //
        // The second kdtree evaluation = seeded hue segmentation
        // Reuse the fist searchtree for this, in order to NOT build it again!
        pcl::PointIndices flower;
        //pcl::seededHueSegmentation(cloud_in, stree, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);
        optimized_shs2(cloud_in, CLUST_TOL_SHS, seed, flower, DELTA_HUE_SHS);

        cv::Mat flowermat(cloud_in.height, cloud_in.width, CV_8UC3, cv::Scalar(0));
        pcl::gpu::people::label_skeleton::makeImageFromPointCloud(flowermat, flower, cloud_in);

#ifdef WRITE
        cv::imwrite("f_" + lexical_cast<string>(counter) + ".png", flowermat);
#endif
        cv::Mat flowergrownmat(cloud_in.height, cloud_in.width, CV_8UC3, cv::Scalar(0));

        int erosion_size = 2;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT ,
            cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
            cv::Point( erosion_size, erosion_size ) );

        cv::dilate(flowermat, flowergrownmat, element);

#ifdef WRITE
        cv::imwrite("g_" + lexical_cast<string>(counter) + ".png", flowergrownmat);
#endif

        cv::Mat dmat2(cloud_in.height, cloud_in.width, CV_16U);
        for(unsigned int v = 0; v < cloud_in.height; v++)
        {
            for(unsigned int u = 0; u < cloud_in.width; u++)
            {
                if(flowergrownmat.at<cv::Vec3b>(v,u)[0] != 0 || flowergrownmat.at<cv::Vec3b>(v,u)[1] != 0 || flowergrownmat.at<cv::Vec3b>(v,u)[2] != 0)
                {
                    dmat2.at<short>(v,u) = dmat.at<short>(v,u);
                }
                else
                {
                    dmat2.at<short>(v,u) = std::numeric_limits<short>::max();
                }
            }
        }

        // //////////////////////////////////////////////////////////////////////////////////////////////// //
        // The second label evaluation

        depth_device2_.upload(dmat2.ptr<unsigned short>(), dmat2.step, dmat2.rows, dmat2.cols);

        // Process the depthimage
        //m_proc->process(dmat2, m_lmap);
        //m_proc->process(depth_device2_, m_lmap);
        m_proc->process(depth_device2_, lmap_device2_);
        m_lmap.create(depth_device2_.rows(), depth_device2_.cols(), CV_8U);
        lmap_device2_.download(m_lmap.data, m_lmap.step);
        
        cv::Mat lmap2(cloud_in.height, cloud_in.width, CV_8UC1);
        pcl::gpu::people::label_skeleton::smoothLabelImage(m_lmap, dmat2, lmap2);
        //cv::medianBlur(m_lmap, lmap2, 3);

        pcl::gpu::people::display::colorLMap( lmap2, cmap );

#ifdef WRITE
        cv::imwrite("d2_" + lexical_cast<string>(counter) + ".png", dmat2);
        cv::imwrite("l2_" + lexical_cast<string>(counter) + ".png", m_lmap);
        cv::imwrite("s2_" + lexical_cast<string>(counter) + ".png", lmap2);
        cv::imwrite("c2_" + lexical_cast<string>(counter) + ".png", cmap);
#endif
        pcl::PointCloud<pcl::PointXYZRGBL> cloud_labels2;
        pcl::gpu::people::conversion::colorLabelPointCloudFromArray(cloud_in, lmap2.data, cloud_labels2);

#ifdef WRITE
        pcl::io::savePCDFileASCII ("2de_it_colorLabeledPointCloud.pcd", cloud_labels2);
        std::cout << "Saved " << cloud_labels2.points.size () << " data points to 2de_it_colorLabeledPointCloud.pcd." << std::endl; 
#endif

        std::vector<std::vector<pcl::PointIndices> > cluster_indices2;
        cluster_indices2.resize(NUM_PARTS);

        // avoid having the search tree to be build again
        //pcl::extractLabeledEuclideanClusters<pcl::PointXYZRGBL>(cloud_labels2, stree, CLUST_TOL, cluster_indices2, AREA_THRES2, MAX_CLUST_SIZE, NUM_PARTS);
        optimized_elec(cloud_in, lmap2, CLUST_TOL, cluster_indices2, AREA_THRES2, MAX_CLUST_SIZE, NUM_PARTS, false, 1.0f);

        std::vector<std::vector<pcl::gpu::people::label_skeleton::Blob2, Eigen::aligned_allocator<pcl::gpu::people::label_skeleton::Blob2> > >       sorted2;
        //clear out our matrix before starting again with it
        sorted2.clear();
        //Set fixed size of outer vector length = number of parts
        sorted2.resize(NUM_PARTS);
        //create the blob2 matrix
        pcl::gpu::people::label_skeleton::sortIndicesToBlob2 ( cloud_labels2, AREA_THRES2, sorted2, cluster_indices2 );
        pcl::gpu::people::label_skeleton::buildRelations ( sorted2 );

        // Test if the second tree is build up correctly
        if(sorted2[10].size() != 0)
        {
            pcl::gpu::people::label_skeleton::Tree2 t2;
            pcl::gpu::people::label_skeleton::buildTree(sorted, cloud_in, Neck, c, t2);
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
            cerr<< t2.nr_parts << ";" << par << ";" << t2.total_dist_error << ";" << t2.norm_dist_error << ";" << counter << ";" << endl;
        }
        counter++;
    }
}


pcl::gpu::people::Person::Person (std::string& tree_file) : 
                  max_cluster_size_(25000),
                  number_of_parts_(25),
                  number_of_trees_(1),
                  cluster_area_threshold_(200),
                  cluster_area_threshold_shs_(100),
                  cluster_tolerance_(0.05),
                  delta_hue_tolerance_(5),
                  elec_radius_scale_(1.0f),
                  elec_brute_force_border_(false),
                  do_shs_(true),
                  dilation_size_(2), 
                  name_("Generic")
{
/// Load the first tree
std::ifstream fin (tree_file.c_str ());
assert (fin.is_open ());
m_proc.reset(new pcl::gpu::people::trees::MultiTreeLiveProc (fin));
fin.close ();
};

int
pcl::gpu::people::Person::addTree (std::string& tree_file)
{
  if(number_of_trees_ >= MAX_NR_TREES)
  {
    PCL_INFO ("Can't add another tree, we are already at max");
    return -1;
  }
  std::ifstream fin(tree_file.c_str() );
  if(!fin.is_open())
  {
    PCL_INFO ("Couldn't open this tree file");
    return -1;
  }
  m_proc->addTree(fin);
  fin.close();
  number_of_trees_++;
  return 1;
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


#endif // PCL_GPU_PEOPLE_PERSON_HPP_

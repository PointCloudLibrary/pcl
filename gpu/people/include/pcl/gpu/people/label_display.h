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
 * $Id: $
 * @author Koen Buys
 * @file display.h
 * @brief This file contains the function prototypes for display functions
 */
#ifndef PCL_GPU_PEOPLE_LABEL_DISPLAY_H_
#define PCL_GPU_PEOPLE_LABEL_DISPLAY_H_
// our headers
#include "pcl/gpu/people/label_blob2.h"
#include "pcl/gpu/people/label_common.h"

// std
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

// opencv drawing stuff
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS specific includes
//#include "pcl_ros/point_cloud.h"
//#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/Point.h>

//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

//#include <image_geometry/pinhole_camera_model.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace label_skeleton 
      {

        typedef std::pair<int,int> two_labels_t;
        typedef std::vector<two_labels_t> two_labels_list_t;

        /**
         * @brief This function draws lines between the nodes of the tree starting from the given part_label & part_number
         * This function uses recursion to draw each branch of the tree
         * @param[in] sorted the blob matrix
         * @param[in] part_label the part label to start with
         * @param[in] part_lid the specific lid of this part
         * @param[out] markerArray array in which the markers will be pushed
         * @todo Check where the parent blob should be allocated
         **/
//        void drawMarkerTree(  std::vector<std::vector<Blob2> >& sorted,
//                              int                               part_label,
//                              int                               part_lid,
//                              visualization_msgs::MarkerArray&  markerArray)
//        {
//          assert(part_label >= 0);
//          assert(part_label < NUM_PARTS);
//          assert(part_lid >= 0);
//        #ifdef VERBOSE
//          std::cout << "(I) : drawMarkerTree(): pl " << part_label << " plid " << part_lid << std::endl;
//        #endif
//          // Test if we don't request a part that doesn't exist and thus throw a segfault
//          if((int) sorted[part_label].size() <= part_lid){
//            std::cout << "(E) : drawMarkerTree(): a unknown part identification was requested, couldn't find it back in the part matrix" << std::endl;
//            exit(0);
//          }
//        
//          int nr_children = LUT_nr_children[part_label];
//        
//          //std::cout << "(I) : drawMarkerTree() : p " << part_label << " has " << nr_children << " children" << std::endl;
//        
//          // we break the recursion in the leaves
//          if(nr_children == 0)
//            return;
//        
//          Blob2 p = sorted[part_label][part_lid]; 
//        
//          // iterate over all possible children
//          for(int i = 0; i < nr_children; i++){
//            // check if this child has a valid child_id, leaf test should be redundant
//            if((p.child_id[i] != NO_CHILD) && (p.child_id[i] != LEAF)){
//              // check if this id is not outside the range it is allowed to go
//              if(p.child_lid[i] < (int) sorted[p.child_label[i]].size()){
//        
//                Blob2 child = sorted[p.child_label[i]][p.child_lid[i]];
//        
//                visualization_msgs::Marker marker;
//                marker.header.frame_id = "/openni_rgb_optical_frame";
//                marker.ns = "Kinematic_Trees";
//                marker.id = p.id*1000 + i;                  //Create a unique id for each marker in the array
//                marker.type = visualization_msgs::Marker::LINE_LIST;
//        
//                marker.action = visualization_msgs::Marker::ADD;
//        
//                marker.pose.orientation.x = 0.0;
//                marker.pose.orientation.y = 0.0;
//                marker.pose.orientation.z = 0.0;
//                marker.pose.orientation.w = 1.0;
//        
//                marker.scale.x = 0.010;
//                marker.scale.y = 0.010;
//                marker.scale.z = 0.010;
//        
//                marker.color.r = 1.0f;
//                marker.color.g = 1.0f;
//                marker.color.b = 1.0f;
//                marker.color.a = 1.0;
//                marker.lifetime = ros::Duration();
//        
//                std::stringstream ss;
//                ss << "p " << p.label << " c " << child.label << std::endl;
//                marker.text = ss.str();
//        
//                geometry_msgs::Point po;
//                po.x = p.mean[0];// +(double) (rand() % 100)/10000.0;
//                po.y = p.mean[1];// +(double) (rand() % 100)/10000.0;
//                po.z = p.mean[2];// +(double) (rand() % 100)/10000.0;
//                marker.points.push_back(po);
//                po.x = child.mean[0];
//                po.y = child.mean[1];
//                po.z = child.mean[2];
//                marker.points.push_back(po);
//                markerArray.markers.push_back(marker);
//                // recursive call for all of this children
//                drawMarkerTree( sorted, p.child_label[i], p.child_lid[i], markerArray);
//              }
//            }
//          }
//          return;
//        }

//        bool isValidPart(std::vector<geometry_msgs::Point> part_label_locations, int pid)
//        {
//          return ( part_label_locations[pid].x != 0 ||  part_label_locations[pid].y != 0 ||  part_label_locations[pid].z != 0 );
//        }

        /**
         * @brief This function recursively draws lines between the nodes of the tree starting from the given part_label & part_number.
         * The displayed tree is "prettified" by adding fake parts and lines, such as the shoulders and their connections.
         * @param[in] sorted the blob matrix
         * @param[in] part_label the part label to start with
         * @param[in] part_lid the specific lid of this part
         * @param[out] markerArray array in which the markers will be pushed
         * @todo Check where the parent blob should be allocated
         **/
//        void drawPrettyMarkerTree(  std::vector<std::vector<Blob2> >& sorted,
//                                    int                               part_label,
//                                    int                               part_lid,
//                                    visualization_msgs::MarkerArray&  markerArray
//                                    )
//        {
//        
//          std::vector<int> marker_part_labels;
//          std::vector<geometry_msgs::Point> part_label_locations;
//          part_label_locations.resize(NUM_PARTS);
//        
//          /* Omit:
//           * MUST BE (PARENT,CHILD)
//           * LChest --> Lupper arm
//           * RChest --> Rupper arm
//           */
//          two_labels_list_t omit_list;
//          omit_list.push_back(two_labels_t(Lchest,Larm));
//          omit_list.push_back(two_labels_t(Rchest,Rarm));
//        
//          drawPrettyMarkerTreeRecurse( sorted, part_label, part_lid, omit_list, markerArray, part_label_locations);
//        
//          /* Add a bunch of links to make the person look better
//           */
//        
//          visualization_msgs::Marker marker;
//        
//          // Figure out where the shoulders should be. 
//        
//          if ( isValidPart(part_label_locations, Rchest) && isValidPart(part_label_locations, Lchest) && isValidPart(part_label_locations, Neck) )
//            {
//        
//              // x right, y down, z forward
//        
//              // Vector between R-L chest
//              geometry_msgs::Point cdiff;
//              cdiff.x = part_label_locations[Rchest].x - part_label_locations[Lchest].x;
//              cdiff.y = part_label_locations[Rchest].y - part_label_locations[Lchest].y;
//              cdiff.z = part_label_locations[Rchest].z - part_label_locations[Lchest].z;
//              float meany = (part_label_locations[Rchest].y + part_label_locations[Lchest].y)/2.0;
//              // To join the neck and shoulders, 
//              // add a fraction of the chest width to the neck location and drop y a bit. Reasonable estimate.
//              float cdiff_factor = 0.75;
//              float yoffset_factor = 0.25;
//              geometry_msgs::Point lshoulder;
//              lshoulder.x = part_label_locations[Neck].x - cdiff_factor*cdiff.x;
//              lshoulder.y = part_label_locations[Neck].y - cdiff_factor*cdiff.y - yoffset_factor*(part_label_locations[Neck].y - meany);
//              lshoulder.z = part_label_locations[Neck].z - cdiff_factor*cdiff.z;
//              geometry_msgs::Point rshoulder;
//              rshoulder.x = part_label_locations[Neck].x + cdiff_factor*cdiff.x;
//              rshoulder.y = part_label_locations[Neck].y + cdiff_factor*cdiff.y - yoffset_factor*(part_label_locations[Neck].y - meany);
//              rshoulder.z = part_label_locations[Neck].z + cdiff_factor*cdiff.z;
//        
//              addLine(part_label_locations[Neck], lshoulder, 5000, Neck, -1, marker);
//              markerArray.markers.push_back(marker);
//        
//              addLine(part_label_locations[Neck], rshoulder, 5001, Neck, -1, marker);
//              markerArray.markers.push_back(marker);
//        
//              // Join the shoulders and upper arms
//              if ( isValidPart(part_label_locations, Larm) ) 
//              {
//                addLine(lshoulder, part_label_locations[Larm], 5002, -1, Larm, marker);
//                markerArray.markers.push_back(marker);
//              }
//              else
//              {
//                std::cout << "(I) :The left arm part is missing, cannot draw shoulder --> left arm." << std::endl;
//              }
//        
//              if ( isValidPart(part_label_locations, Rarm) ) 
//              {
//                addLine(rshoulder, part_label_locations[Rarm], 5003, -1, Rarm, marker); 
//                markerArray.markers.push_back(marker);
//              }
//              else
//              {
//                std::cout << "(I) : The right arm part is missing, cannot draw shoulder --> right arm." << std::endl;
//              }
//            }
//            else
//            {
//              std::cout << "(I) : Either the neck, or the left or right chest part is missing. Cannot figure out where the shoulders should be." << std::endl;
//            }
//        
//        
//          // Join the top of the head and the bottom of the head to make a pretty square face  
//          if ( isValidPart(part_label_locations,FaceLT) && isValidPart(part_label_locations,FaceRT) )
//          {
//            addLine(part_label_locations[FaceLT], part_label_locations[FaceRT], 5004, FaceLT, FaceRT, marker);
//            markerArray.markers.push_back(marker);
//          }
//          if ( isValidPart(part_label_locations,FaceLB) && isValidPart(part_label_locations,FaceRB) )
//          {
//            addLine(part_label_locations[FaceLB], part_label_locations[FaceRB], 5005, FaceLB, FaceRB, marker);
//            markerArray.markers.push_back(marker);
//          }
//        
//          // Join the chest and the hips to make a pretty square torso 
//          if ( isValidPart(part_label_locations,Rchest) && isValidPart(part_label_locations,Lchest) ) 
//          {
//            addLine(part_label_locations[Lchest], part_label_locations[Rchest], 5006, Lchest, Rchest, marker);
//            markerArray.markers.push_back(marker);
//          }
//          if ( isValidPart(part_label_locations,Rhips) && isValidPart(part_label_locations,Lhips) ) 
//          {
//            addLine(part_label_locations[Lhips], part_label_locations[Rhips], 5007, Lhips, Rhips, marker);
//            markerArray.markers.push_back(marker);
//          }
//        }
//        
        /**
         * @brief Format a line marker
         */
//        void addLine(geometry_msgs::Point p0, geometry_msgs::Point p1, int id, int plabel, int clabel, visualization_msgs::Marker& marker)
//        {
//          marker.header.frame_id = "/openni_rgb_optical_frame";
//          marker.ns = "Kinematic_Trees";
//          marker.id = id;                  //Create a unique id for each marker in the array
//          marker.type = visualization_msgs::Marker::LINE_LIST;
//        
//          marker.action = visualization_msgs::Marker::ADD;
//        
//          marker.pose.orientation.x = 0.0;
//          marker.pose.orientation.y = 0.0;
//          marker.pose.orientation.z = 0.0;
//          marker.pose.orientation.w = 1.0;
//        
//          marker.scale.x = 0.010;
//          marker.scale.y = 0.010;
//          marker.scale.z = 0.010;
//        
//          marker.color.r = 1.0f;
//          marker.color.g = 1.0f;
//          marker.color.b = 1.0f;
//          marker.color.a = 1.0;
//          marker.lifetime = ros::Duration();
//        
//          std::stringstream ss;
//          ss << "p " << plabel << " c " << clabel << std::endl;
//          marker.text = ss.str();
//        
//          marker.points.push_back(p0);
//          marker.points.push_back(p1);
//        }
//        
        
        /**
         * @brief This function recursively draws lines between the nodes of the tree starting from the given part_label & part_number.
         * The displayed tree is "prettified" by adding fake parts and lines, such as the shoulders and their connections.
         * @param[in] sorted the blob matrix
         * @param[in] part_label the part label to start with
         * @param[in] part_lid the specific lid of this part
         * @param[in] omit_list a list of part number pairs whose links should be omitted from the tree
         * @param[out] markerArray array in which the markers will be pushed
         * @todo Check where the parent blob should be allocated
         **/
//        void drawPrettyMarkerTreeRecurse(  std::vector<std::vector<Blob2> >& sorted,
//                                           int                               part_label,
//                                           int                               part_lid,
//                                           two_labels_list_t& omit_list,
//                                           visualization_msgs::MarkerArray&  markerArray,
//                                           std::vector<geometry_msgs::Point>& part_label_locations
//                                           )
//        {
//          assert(part_label >= 0);
//          assert(part_label < NUM_PARTS);
//          assert(part_lid >= 0);
//        #ifdef VERBOSE
//          std::cout << "(I) : drawPrettyMarkerTreeRecurse(): pl " << part_label << " plid " << part_lid << std::endl;
//        #endif
//          // Test that the part exists to avoid throwing a segfault
//          if((int) sorted[part_label].size() <= part_lid){
//            std::cout << "(E) : drawPrettyMarkerTreeRecurse(): the requested part id doesn't exist in the part matrix" << std::endl;
//            exit(0);
//          }
//        
//          int nr_children = LUT_nr_children[part_label];
//        
//          // we break the recursion in the leaves
//          if(nr_children == 0)
//            return;
//        
//          Blob2 p = sorted[part_label][part_lid]; 
//        
//          // iterate over all possible children
//          for(int i = 0; i < nr_children; i++){
//            // check if this child has a valid child_id, leaf test should be redundant
//            if((p.child_id[i] != NO_CHILD) && (p.child_id[i] != LEAF)){
//              // check if this id is not outside the range it is allowed to go
//              if(p.child_lid[i] < (int) sorted[p.child_label[i]].size()){
//        
//                Blob2 child = sorted[p.child_label[i]][p.child_lid[i]];
//        
//                // If we should omit this pair, just recurse without drawing.
//                bool omit = false;
//                for (uint ip = 0; ip < omit_list.size(); ip++) {
//                  if (omit_list[ip].first == p.label && omit_list[ip].second == child.label) {
//                    omit = true;
//                    break;
//                  }
//                }
//        
//                geometry_msgs::Point p0;
//                p0.x = p.mean[0];
//                p0.y = p.mean[1];
//                p0.z = p.mean[2];
//                if (p.label == 10) part_label_locations[p.label] = p0; // label 10 doesn't have any parents
//                geometry_msgs::Point p1;
//                p1.x = child.mean[0];
//                p1.y = child.mean[1];
//                p1.z = child.mean[2];
//                part_label_locations[child.label] = p1;
//        
//                if (!omit)
//                {
//                  visualization_msgs::Marker marker;
//                  addLine(p0, p1, p.id*1000 + i, p.label, child.label, marker);
//                  markerArray.markers.push_back(marker);
//                }
//        
//                // recursive call for all of this children
//                drawPrettyMarkerTreeRecurse( sorted, (int)p.child_label[i], p.child_lid[i], omit_list, markerArray, part_label_locations);
//              }
//            }
//          }
//          return;
//        }
//        
        /**
         * @brief This function generates a pointIndices list of a tree
         * @param[in] sorted the matrix of blob2's
         * @param[in] part_label the label to start with
         * @param[in] part_lid the lid of the label to start with
         * @param[in] cloud_in the original input cloud for which the indices in blob2 count
         * @param[out] cloud_list the generated pointIndices list
         * @return the number of total point entries in the cloud
         **/
        int makeTreePointIndices( std::vector<std::vector<Blob2> >&   sorted,
                                  int                                 part_label,
                                  int                                 part_lid,
                                  pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                                  pcl::PointIndices&                  cloud_list)
        {
          if(sorted.size() <= 0)
          {
            std::cout << "(E) : makeTreePointIndices: hey man, don't fool me, you gave me an empty blob matrix" << std::endl;
            return -1;
          }
          int total_nr_entries = 0;

          // iterate over the number of pixels that are part of this label
          for(unsigned int in = 0; in < sorted[part_label][part_lid].indices.indices.size(); in++)
          {
            unsigned int index = sorted[part_label][part_lid].indices.indices[in];
            cloud_list.indices.push_back(index);
            total_nr_entries++;
          }

          int nr_children = LUT_nr_children[part_label];

          if(nr_children == 0)
            return total_nr_entries;

          // iterate over all possible children
          for(int i = 0; i < nr_children; i++){
            // check if this child has a valid child_id, leaf test should be redundant
            if(( sorted[part_label][part_lid].child_id[i] != NO_CHILD) && ( sorted[part_label][part_lid].child_id[i] != LEAF)){
                total_nr_entries += makeTreePointIndices( sorted, sorted[part_label][part_lid].child_label[i], sorted[part_label][part_lid].child_lid[i], cloud_in, cloud_list);
            }
          }
          return total_nr_entries;
        }

        /**
         * @brief This function generates a pointcloud of a tree
         * @param[in] sorted the matrix of blob2's
         * @param[in] part_label the label to start with
         * @param[in] part_lid the lid of the label to start with
         * @param[in] cloud_in the original input cloud for which the indices in blob2 count
         * @param[out] cloud_out the generated pointcloud
         * @return the number of total point entries in the cloud
         **/
        int makeTreePointCloud( std::vector<std::vector<Blob2> >&   sorted,
                                int                                 part_label,
                                int                                 part_lid,
                                pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                                pcl::PointCloud<pcl::PointXYZRGB>&  cloud_out)
        {
          if(sorted.size() <= 0)
          {
            std::cout << "(E) : hey man, don't fool me, you gave me an empty blob matrix" << std::endl;
            return -1;
          }
          int total_nr_entries = 0;

          // iterate over the number of pixels that are part of this label
          for(unsigned int in = 0; in < sorted[part_label][part_lid].indices.indices.size(); in++)
          {
            unsigned int index = sorted[part_label][part_lid].indices.indices[in];
            cloud_out.points.push_back(cloud_in.points[index]);
            total_nr_entries++;
          }

          int nr_children = LUT_nr_children[part_label];

          if(nr_children == 0)
            return total_nr_entries;

          // iterate over all possible children
          for(int i = 0; i < nr_children; i++){
            // check if this child has a valid child_id, leaf test should be redundant
            if(( sorted[part_label][part_lid].child_id[i] != NO_CHILD) && ( sorted[part_label][part_lid].child_id[i] != LEAF)){
                total_nr_entries += makeTreePointCloud( sorted, sorted[part_label][part_lid].child_label[i], sorted[part_label][part_lid].child_lid[i], cloud_in, cloud_out);
            }
          }
          cloud_out.height = 1;
          cloud_out.width = total_nr_entries;
          return total_nr_entries;
        }

        /**
         * @brief This function generates a pointcloud of the blob after the blob evaluation
         * @param[in] sorted the matrix of blob2's
         * @param[in] cmap the OpenCV matrix with the blob colors
         * @param[in] cloud_in the original input cloud for which the indices in blob2 count
         * @param[out] cloud_out the generated pointcloud
         * @return the number of total point entries in the cloud, heck don't know why you would need this
         **/
        int makeBlobPointCloud( std::vector<std::vector<Blob2> >&   sorted,
                                cv::Mat&                            cmap,
                                pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                                pcl::PointCloud<pcl::PointXYZRGB>&  cloud_out)
        {
          if(sorted.size() <= 0)
          {
            std::cout << "(E) : hey man, don't fool me, you gave me an empty blob matrix" << std::endl;
            return -1;
          }

          cloud_out.height = 1;
          int total_nr_entries = 0;

          // iterate over all labels
          for(unsigned int l = 0; l < sorted.size(); l++)
          {
            // iterate over number of blobs for this label
            for(unsigned int i = 0; i < sorted[l].size(); i++)
            {
              // iterate over the number of pixels that are part of this label
              for(unsigned int in = 0; in < sorted[l][i].indices.indices.size(); in++)
              {
                unsigned int index = sorted[l][i].indices.indices[in];
                pcl::PointXYZRGB p = cloud_in.points[index];
                //cv::Point2i uv(index/cloud_in.width ,index%cloud_in.width);
                cv::Point2i uv(index%cloud_in.width,index/cloud_in.width );
                cv::Vec3b& bgr = cmap.at<cv::Vec3b>(uv);
                p.r = bgr[2];
                p.g = bgr[1];
                p.b = bgr[0];
                cloud_out.points.push_back(p);
                total_nr_entries++;
              }
            }
          }
          cloud_out.width = total_nr_entries;
          return total_nr_entries;
        }

      } // end namespace LabelSkel
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
#endif //#ifndef LABELSKEL_DISPLAY_H


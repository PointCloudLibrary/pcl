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
 * @file tree.h
 * @brief This file contains the function prototypes for the tree building functions.
 */
 
#pragma once
 
// our headers
#include "pcl/gpu/people/label_blob2.h"   //this one defines the blob structure
#include "pcl/gpu/people/label_common.h"  //this one defines the LUT's
#include "pcl/gpu/people/person_attribs.h"

// std
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdexcept>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

namespace pcl
{
  namespace gpu
  {
    namespace people    
    {           
     /**
       * @brief This structure contains all parameters to describe the segmented tree
       */
      struct Tree2 
      {
        //Inline constructor
        Tree2() : id(NO_CHILD), lid(NO_CHILD), nr_parts(0)
        {
          for(int &part : parts_lid)
            part = NO_CHILD;
        }
       
        int     id;                     // specific identification number of this tree
        part_t  label;                  // labels which part the root of this tree is
        int     lid;                    // label id, which number of this type of part is this
        int     nr_parts;               // the number of parts in this tree
        int     parts_lid[NUM_PARTS];   // Indicate the used parts
        float   total_dist_error;       // sum of all distance errors
        float   norm_dist_error;         // total_dist_error/nr_parts

        Eigen::Vector4f  mean;          // mean in xyz
        Eigen::Matrix3f  cov;           // covariance in 3x3 matrix
        Eigen::Vector3f  eigenval;      // eigenvalue of blob
        Eigen::Matrix3f  eigenvect;     // eigenvector of blob

        pcl::PointIndices indices;      // The indices of the pointcloud
        Eigen::Vector4f   min;          // The min of the bounding box
        Eigen::Vector4f   max;          // The max of the bounding box
      };

      inline std::ostream& operator << (std::ostream& os, const Tree2& t)
      {
        os << " Tree2 id " << t.id << " label " << t.label << " lid " << t.lid << " nr_parts " << t.nr_parts << std::endl;
        os << " total_dist_error " << t.total_dist_error << " norm_dist_error " << t.norm_dist_error << std::endl;
        os << " mean " << t.mean(0) << " , " << t.mean(1) << " , " << t.mean(2) << " , " << t.mean(3) << std::endl;
        os << " cov " << std::endl << t.cov << std::endl;
        os << " eigenval " << t.eigenval(0) << " , " << t.eigenval(1) << " , " << t.eigenval(2) << std::endl;
        os << " eigenvect " << std::endl << t.eigenvect << std::endl;
        os << " min " << t.min(0) << " , " << t.min(1) << " , " << t.min(2) << " , " << t.min(3) << std::endl;
        os << " max " << t.max(0) << " , " << t.max(1) << " , " << t.max(2) << " , " << t.max(3) << std::endl;
        os << " indices length " << t.indices.indices.size() << std::endl;
        return (os);
      }     

      /**
       * @brief This function sets the children of the leaf nodes to leaf, meaning that we came to the correct end
       * @param[in] sorted The matrix of all blobs
       * @param[in] label The label of which all children are to be set as leafs
       * @return Zero if everything went well
       **/
      inline int
      leafBlobVector(   std::vector<std::vector<Blob2, Eigen::aligned_allocator<Blob2> > >& sorted,
                            int                               label )
      {
        if(sorted[label].empty ())
          return 0;
        for(auto &blob : sorted[label])
        {
          for(int j = 0; j < MAX_CHILD; j++)
            blob.child_id[j] = LEAF;
        }
        return 0;
      }

      /**
       * @brief This function sets the specific child of the vector to no child, meaning that there are no such children
       * @param[in] sorted The matrix of all blobs
       * @param[in] label The label of which the child must be set to NO_CHILD
       * @param[in] child_number The index of the respective child that must be set
       * @return Zero if everything went well
       **/
      inline int
      noChildBlobVector(  std::vector<std::vector<Blob2, Eigen::aligned_allocator<Blob2> > >& sorted,
                              int                               label,
                              int                               child_number)
      {
        if(sorted[label].empty ())
          return 0;
        for(auto &blob : sorted[label]){
          blob.child_id[child_number] = NO_CHILD;
        }
        return 0;
      }

      /**
       * @brief This function test if children were found for this label
       * @return True if this label has valid children
       **/
      inline bool
      hasThisLabelChildren ( std::vector<std::vector<Blob2, Eigen::aligned_allocator<Blob2> > >& sorted,
                                  part_t                            label,
                                  int                               child_number)
      {
        if(sorted[label].empty ())
          return false;
        for(const auto &blob : sorted[label])
          if((blob.child_id[child_number] != NO_CHILD) && (blob.child_id[child_number] != LEAF))
            return true;
        return false;
      }

      /**
       * @brief This is the evaluation function used to compare two blobs
       * @param[in] parent    pointer to the parent blob
       * @param[in] child     pointer to the child blob
       * @param[in] child_nr  the number of the child
       * @return it returns the distance error from the ideal parent child distance, it returns -1.0 if it goes over threshold
       * @todo what if child is second link in stead of first link (ea forearm in stead of elbow for arm)
       **/
      inline float
      evaluateBlobs (Blob2& parent, Blob2& child, int child_nr)
      {
        float root = sqrt(pow(parent.mean(0) - child.mean(0), 2) +
                          pow(parent.mean(1) - child.mean(1), 2) +
                          pow(parent.mean(2) - child.mean(2), 2));
        float offset = std::fabs(LUT_ideal_length[(int)parent.label][child_nr] - root);
        if(offset > LUT_max_length_offset[(int)parent.label][child_nr])
          return -1.0;
        return offset;
      }

      /**
       * @brief This is the evaluation function used to compare two blobs
       * @param[in] parent    pointer to the parent blob
       * @param[in] child     pointer to the child blob
       * @param[in] child_nr  the number of the child
       * @param person_attribs
       * @return it returns the distance error from the ideal parent child distance, it returns -1.0 if it goes over threshold
       * @todo what if child is second link in stead of first link (ea forearm in stead of elbow for arm)
       **/
      inline float
      evaluateBlobs (Blob2& parent,
                     Blob2& child,
                     int child_nr,
                     PersonAttribs::Ptr person_attribs)
      {
        float root = sqrt(pow(parent.mean(0) - child.mean(0), 2) +
                          pow(parent.mean(1) - child.mean(1), 2) +
                          pow(parent.mean(2) - child.mean(2), 2));
        float offset = std::fabs(person_attribs->part_ideal_length_[(int)parent.label][child_nr] - root);
        if(offset > person_attribs->max_length_offset_[(int)parent.label][child_nr])
          return -1.0;
        return offset;
      }

      /**
       * @brief This function evaluates an entire row of parent segments for the best child segments
       * @param[in] sorted this is the array of all blobs
       * @param[in] parent_label this is the part label that indicates the row
       * @param[in] child_label  this is the part label that indicates the childs needed to be investigated
       * @param[in] child_number the number of this child in the parent, some parents have multiple childs
       * @return zero if successful
       * @todo once we have good evaluation function reconsider best_value
       **/
      inline int
      evaluateBlobVector( std::vector<std::vector<Blob2, Eigen::aligned_allocator<Blob2> > >& sorted,
                              unsigned int                      parent_label,
                              int                               child_label,
                              int                               child_number)
      {
        // Check the input data
        assert(parent_label < NUM_PARTS);
        assert(child_label >= 0);
        assert(child_number >= 0);
        assert(child_number < MAX_CHILD);

        if(sorted[parent_label].empty ()){
          return 0;   //if my size is 0, this is solved by my parent in his iteration	
        }
        if(sorted[child_label].empty ()){
          noChildBlobVector(sorted, parent_label, child_number);
          return 0;
        }
        // go over all parents in this vector
        for(std::size_t p = 0; p < sorted[parent_label].size(); p++){
          float best_value = std::numeric_limits<float>::max(); 
          int best_child_id = NO_CHILD;
          int best_child_lid = 0;                               // this must be as low as possible, still overruled by id
          float value = 0.0;

          // go over all children in this vector
          for(std::size_t c = 0; c < sorted[child_label].size(); c++){
            value = evaluateBlobs(sorted[parent_label][p], sorted[child_label][c], child_number);
            // Value should contain offset from the ideal position
            // Is -1 if it goes above threshold
            if(value < best_value && value != -1.0){
              best_child_id = sorted[child_label][c].id;
              best_child_lid = c;
              best_value = value;
            }
          }
          assert(parent_label < sorted.size());
          assert(p < sorted[parent_label].size());
          assert(child_label < (int) sorted.size());
          //Set the correct child in the parent
          sorted[parent_label][p].child_id[child_number] = best_child_id;
          sorted[parent_label][p].child_lid[child_number] = best_child_lid;
          sorted[parent_label][p].child_dist[child_number] = best_value;
          sorted[parent_label][p].child_label[child_number] = child_label;
        }
        return 0;
      }

      /**
       * @brief This function evaluates an entire row of parent segments for the best child segments
       * @param[in] sorted this is the array of all blobs
       * @param[in] parent_label this is the part label that indicates the row
       * @param[in] child_label  this is the part label that indicates the childs needed to be investigated
       * @param[in] child_number the number of this child in the parent, some parents have multiple childs
       * @param person_attribs
       * @return zero if successful
       * @todo once we have good evaluation function reconsider best_value
       **/
      inline int
      evaluateBlobVector( std::vector<std::vector<Blob2, Eigen::aligned_allocator<Blob2> > >& sorted,
                              unsigned int                      parent_label,
                              int                               child_label,
                              int                               child_number,
                              PersonAttribs::Ptr                person_attribs)
      {
        // Check the input data
        assert(parent_label < NUM_PARTS);
        assert(child_label >= 0);
        assert(child_number >= 0);
        assert(child_number < MAX_CHILD);

        if(sorted[parent_label].empty ()){
          return 0;   //if my size is 0, this is solved by my parent in his iteration
        }
        if(sorted[child_label].empty ()){
          noChildBlobVector(sorted, parent_label, child_number);
          return 0;
        }
        // go over all parents in this vector
        for(std::size_t p = 0; p < sorted[parent_label].size(); p++){
          float best_value = std::numeric_limits<float>::max();
          int best_child_id = NO_CHILD;
          int best_child_lid = 0;                               // this must be as low as possible, still overruled by id
          float value = 0.0;

          // go over all children in this vector
          for(std::size_t c = 0; c < sorted[child_label].size(); c++){
            value = evaluateBlobs(sorted[parent_label][p], sorted[child_label][c], child_number, person_attribs);
            // Value should contain offset from the ideal position
            // Is -1 if it goes above threshold
            if(value < best_value && value != -1.0){
              best_child_id = sorted[child_label][c].id;
              best_child_lid = c;
              best_value = value;
            }
          }
          assert(parent_label < sorted.size());
          assert(p < sorted[parent_label].size());
          assert(child_label < (int) sorted.size());
          //Set the correct child in the parent
          sorted[parent_label][p].child_id[child_number] = best_child_id;
          sorted[parent_label][p].child_lid[child_number] = best_child_lid;
          sorted[parent_label][p].child_dist[child_number] = best_value;
          sorted[parent_label][p].child_label[child_number] = child_label;
        }
        return 0;
      }

      /**
       * @brief This function goes over the sorted matrix and fills in the optimal parent and child relations
       * @param[in] sorted a matrix with all found good blobs arranged according to label and order
       * @return zero if everything went well, negative on an error
       * @todo This function also fixes the kinematic chain, we should implement this in a xml or LUT
       * @todo look if we can't get a more efficient implementation (iterator together with sortBlobs perhaps?)
       */
      inline int
      buildRelations( std::vector<std::vector<Blob2, Eigen::aligned_allocator<pcl::gpu::people::Blob2> > >& sorted)
      {
        PCL_VERBOSE("[pcl::gpu::people::buildRelations] : (I) : buildRelations : regular version\n");
        if(sorted.empty ()){
          std::cout << "(E) : Damn you, you gave me an empty matrix!" << std::endl;
          return (-1);
        }
        // Iterate over all parts
        for(std::size_t p = 0; p < sorted.size(); p ++)
        {
          switch(p){
            // These are multinodes and end nodes ///
            case Neck:
              evaluateBlobVector(sorted, p, FaceRB, 0);
              evaluateBlobVector(sorted, p, FaceLB, 1);
              evaluateBlobVector(sorted, p, Rchest, 2);
              evaluateBlobVector(sorted, p, Lchest, 3);
              break;
            case 0:                                 // this is the Lfoot
            case 4:                                 // this is the Rfoot
            case 14:                                // this is the Rhand
            case 18:                                // this is the Lhand
            case 21:                                // this is the FaceLT
            case 22:                                // this is the FaceRT
              leafBlobVector(sorted, p);            //fill in the children of leafs
              break; 
            case 23:					// this is the Rchest
              evaluateBlobVector(sorted, p, 11, 0);	//Child 0 is Rarm
              evaluateBlobVector(sorted, p, 8, 1);	//Child 1 is Rhips
              break;
            case 24:					// this is the Lchest
              evaluateBlobVector(sorted, p, 15, 0);	//Child 0 is Larm
              evaluateBlobVector(sorted, p, 9, 1);	//Child 1 is Lhips
              break; 
            // FROM HERE ALL THE REGULAR MIDDLE NODES  ///
            case 1:                               //this is the Lleg
              evaluateBlobVector(sorted,p, 0, 0); //Child 0 is Lfeet
              break;
            case 2:                               //this is the Lknee
              evaluateBlobVector(sorted,p, 1, 0); //Child 0 is Lleg
              break;
            case 3:                               //this is the Lthigh
              evaluateBlobVector(sorted,p, 2, 0); //Child 0 is Lknee
              break;
            case 5:                               //this is the Rleg
              evaluateBlobVector(sorted,p, 4, 0); //Child Rfoot
              break;
            case 6:                               //this is the Rknee
              evaluateBlobVector(sorted,p, 5, 0); //Child Rleg
              break;
            case 7:                               //this is the Rthigh
              evaluateBlobVector(sorted,p, 6, 0); //Child Rknee
              break;
            case 8:                               //this is the Rhips
              evaluateBlobVector(sorted,p, 7, 0); //Child Rthigh
              break;
            case 9:                               //this is the Lhips
              evaluateBlobVector(sorted,p, 3, 0); //Child Lthigh
              break;
            case Rarm:
              evaluateBlobVector(sorted,p, Relbow, 0);
              if(!hasThisLabelChildren(sorted, Rarm, 0))
                evaluateBlobVector(sorted,p, Rforearm, 0);
              break;
            case 12:                               //this is the Relbow
              evaluateBlobVector(sorted,p, 13, 0); //Child Rforearm
              break;
            case 13:                               //this is the Rforearm
              evaluateBlobVector(sorted,p, 14, 0); //Child Rhand
              break;
            case Larm:
              evaluateBlobVector(sorted,p, Lelbow, 0);
              if(!hasThisLabelChildren(sorted, Larm, 0))
                evaluateBlobVector(sorted,p, Lforearm, 0);
              break;
            case 16:                               //this is the Lelbow
              evaluateBlobVector(sorted,p, 17, 0); //Child Lforearm
              break;
            case 17:                               //this is the Lforearm
              evaluateBlobVector(sorted,p, 18, 0); //Child Lhand
              break;
            case 19:                               //this is the FaceLB
              evaluateBlobVector(sorted,p, 21, 0); //Child FaceLT
              break;
            case 20:                               //this is the FaceRB
              evaluateBlobVector(sorted,p, 22, 0); //Child FaceRT
              break;
            default:
              break;
          }
        }
        return 0;	
      }

      /**
       * @brief This function goes over the sorted matrix and fills in the optimal parent and child relations
       * @param[in] sorted a matrix with all found good blobs arranged according to label and order
       * @param person_attribs
       * @return zero if everything went well, negative on an error
       * @todo This function also fixes the kinematic chain, we should implement this in a xml or LUT
       * @todo look if we can't get a more efficient implementation (iterator together with sortBlobs perhaps?)
       */
      inline int
      buildRelations( std::vector<std::vector<Blob2, Eigen::aligned_allocator<pcl::gpu::people::Blob2> > >& sorted,
                      PersonAttribs::Ptr person_attribs)
      {
        PCL_DEBUG("[pcl::gpu::people::buildRelations] : (D) : person specific version\n");
        if(sorted.empty ()){
          PCL_ERROR("[pcl::gpu::people::buildRelations] : (E) : Damn you, you gave me an empty matrix!\n");
          return (-1);
        }
        // Iterate over all parts
        for(std::size_t p = 0; p < sorted.size(); p ++)
        {
          switch(p){
            // These are multinodes and end nodes ///
            case Neck:
              evaluateBlobVector(sorted, p, FaceRB, 0, person_attribs);
              evaluateBlobVector(sorted, p, FaceLB, 1, person_attribs);
              evaluateBlobVector(sorted, p, Rchest, 2, person_attribs);
              evaluateBlobVector(sorted, p, Lchest, 3, person_attribs);
              break;
            case 0:                                 // this is the Lfoot
            case 4:                                 // this is the Rfoot
            case 14:                                // this is the Rhand
            case 18:                                // this is the Lhand
            case 21:                                // this is the FaceLT
            case 22:                                // this is the FaceRT
              leafBlobVector(sorted, p);            //fill in the children of leafs
              break;
            case 23:          // this is the Rchest
              evaluateBlobVector(sorted, p, 11, 0, person_attribs); //Child 0 is Rarm
              evaluateBlobVector(sorted, p, 8, 1, person_attribs);  //Child 1 is Rhips
              break;
            case 24:          // this is the Lchest
              evaluateBlobVector(sorted, p, 15, 0, person_attribs); //Child 0 is Larm
              evaluateBlobVector(sorted, p, 9, 1, person_attribs);  //Child 1 is Lhips
              break;
            // FROM HERE ALL THE REGULAR MIDDLE NODES  ///
            case 1:                               //this is the Lleg
              evaluateBlobVector(sorted,p, 0, 0, person_attribs); //Child 0 is Lfeet
              break;
            case 2:                               //this is the Lknee
              evaluateBlobVector(sorted,p, 1, 0, person_attribs); //Child 0 is Lleg
              break;
            case 3:                               //this is the Lthigh
              evaluateBlobVector(sorted,p, 2, 0, person_attribs); //Child 0 is Lknee
              break;
            case 5:                               //this is the Rleg
              evaluateBlobVector(sorted,p, 4, 0, person_attribs); //Child Rfoot
              break;
            case 6:                               //this is the Rknee
              evaluateBlobVector(sorted,p, 5, 0, person_attribs); //Child Rleg
              break;
            case 7:                               //this is the Rthigh
              evaluateBlobVector(sorted,p, 6, 0, person_attribs); //Child Rknee
              break;
            case 8:                               //this is the Rhips
              evaluateBlobVector(sorted,p, 7, 0, person_attribs); //Child Rthigh
              break;
            case 9:                               //this is the Lhips
              evaluateBlobVector(sorted,p, 3, 0, person_attribs); //Child Lthigh
              break;
            case Rarm:
              evaluateBlobVector(sorted,p, Relbow, 0, person_attribs);
              if(!hasThisLabelChildren(sorted, Rarm, 0))
                evaluateBlobVector(sorted,p, Rforearm, 0, person_attribs);
              break;
            case 12:                               //this is the Relbow
              evaluateBlobVector(sorted,p, 13, 0, person_attribs); //Child Rforearm
              break;
            case 13:                               //this is the Rforearm
              evaluateBlobVector(sorted,p, 14, 0, person_attribs); //Child Rhand
              break;
            case Larm:
              evaluateBlobVector(sorted,p, Lelbow, 0, person_attribs);
              if(!hasThisLabelChildren(sorted, Larm, 0))
                evaluateBlobVector(sorted,p, Lforearm, 0, person_attribs);
              break;
            case 16:                               //this is the Lelbow
              evaluateBlobVector(sorted,p, 17, 0, person_attribs); //Child Lforearm
              break;
            case 17:                               //this is the Lforearm
              evaluateBlobVector(sorted,p, 18, 0, person_attribs); //Child Lhand
              break;
            case 19:                               //this is the FaceLB
              evaluateBlobVector(sorted,p, 21, 0, person_attribs); //Child FaceLT
              break;
            case 20:                               //this is the FaceRB
              evaluateBlobVector(sorted,p, 22, 0, person_attribs); //Child FaceRT
              break;
            default:
              break;
          }
        }
        return 0;
      }



      inline int browseTree (const std::vector<std::vector <Blob2, Eigen::aligned_allocator<Blob2> > >&  sorted,
                             Tree2& tree,
                             int part_label,
                             int part_lid)
      {
        int nr_children = LUT_nr_children[part_label];
        tree.nr_parts++;
        tree.parts_lid[part_label] = part_lid;

        const Blob2& blob = sorted[part_label][part_lid];

        // iterate over the number of pixels that are part of this label
        const auto& indices = blob.indices.indices;
        tree.indices.indices.insert(tree.indices.indices.end(), indices.begin(), indices.end());

        if(nr_children == 0)
          return 0;

        // iterate over all possible children
        for(int i = 0; i < nr_children; i++)
        {
          // check if this child has a valid child_id, leaf test should be redundant
          if(blob.child_id[i] != NO_CHILD && blob.child_id[i] != LEAF)
          {
            tree.total_dist_error += blob.child_dist[i];
            browseTree( sorted, tree, blob.child_label[i], blob.child_lid[i]);
          }
        }
        return 0;
      }

      inline int browseTree (const std::vector<std::vector <Blob2, Eigen::aligned_allocator<Blob2> > >&  sorted,
                             Tree2&             tree,
                             int                part_label,
                             int                part_lid,
                             PersonAttribs::Ptr person_attribs)
      {
        int nr_children = person_attribs->nr_of_children_[part_label];
        tree.nr_parts++;
        tree.parts_lid[part_label] = part_lid;

        const Blob2& blob = sorted[part_label][part_lid];

        // iterate over the number of pixels that are part of this label
        const auto& indices = blob.indices.indices;
        tree.indices.indices.insert(tree.indices.indices.end(), indices.begin(), indices.end());
        
        if(nr_children == 0)
          return 0;

        // iterate over all possible children
        for(int i = 0; i < nr_children; i++)
        {
          // check if this child has a valid child_id, leaf test should be redundant
          if(blob.child_id[i] != NO_CHILD && blob.child_id[i] != LEAF)
          {
            tree.total_dist_error += blob.child_dist[i];
            browseTree( sorted, tree, blob.child_label[i], blob.child_lid[i]);
          }
        }
        return 0;
      }

      inline int buildTree ( const std::vector<std::vector <Blob2, Eigen::aligned_allocator<Blob2> > >&  sorted,
                             const pcl::PointCloud<pcl::PointXYZ>&  cloud_in,
                             part_t                                 part_label,
                             int                                    part_lid,
                             Tree2&                                 tree)
      {
        if(sorted.empty ())
        {
          std::cout << "(E) : buildTree(): hey man, don't fool me, you gave me an empty blob matrix" << std::endl;
          return -1;
        }
        tree.label = part_label;
        tree.lid = part_lid;
        tree.total_dist_error = 0;
        tree.nr_parts = 0;

        browseTree(sorted, tree, part_label, part_lid);

        pcl::getMinMax3D(cloud_in, tree.indices, tree.min, tree.max);
        pcl::compute3DCentroid(cloud_in, tree.indices, tree.mean);
        pcl::computeCovarianceMatrixNormalized(cloud_in, tree.indices, tree.mean, tree.cov);

        pcl::eigen33(tree.cov, tree.eigenvect, tree.eigenval);

        tree.norm_dist_error = tree.total_dist_error/tree.nr_parts;

        return 0;
      }

      inline int buildTree ( const std::vector<std::vector <Blob2, Eigen::aligned_allocator<Blob2> > >&  sorted,
                             const pcl::PointCloud<pcl::PointXYZ>&  cloud_in,
                             part_t                                 part_label,
                             int                                    part_lid,
                             Tree2&                                 tree,
                             PersonAttribs::Ptr                     person_attribs)
      {
        if(sorted.empty ())
        {
          std::cout << "(E) : buildTree(): hey man, don't fool me, you gave me an empty blob matrix" << std::endl;
          return -1;
        }
        tree.label = part_label;
        tree.lid = part_lid;
        tree.total_dist_error = 0;
        tree.nr_parts = 0;

        browseTree(sorted, tree, part_label, part_lid, person_attribs);

        pcl::getMinMax3D(cloud_in, tree.indices, tree.min, tree.max);
        pcl::compute3DCentroid(cloud_in, tree.indices, tree.mean);
        pcl::computeCovarianceMatrixNormalized(cloud_in, tree.indices, tree.mean, tree.cov);

        pcl::eigen33(tree.cov, tree.eigenvect, tree.eigenval);

        tree.norm_dist_error = tree.total_dist_error/tree.nr_parts;

        return 0;
      }

    } //end namespace people
  } // end namespace gpu
} // end namespace pcl

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
 * @file segment.h
 * @brief This file contains the function prototypes for the segmentation functions
 */

#pragma once

// our headers
#include "pcl/gpu/people/label_blob2.h"
#include "pcl/gpu/people/label_common.h"

// std
#include <vector>

// opencv drawing stuff
//#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/PointIndices.h>

#include <pcl/common/time.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace label_skeleton
      {
        /*
         * \brief this function smooths the label image based on label and depth
         * \param[in] lmap_in the cvMat with the labels, must be CV_8UC1
         * \param[in] dmap the cvMat with the depths, must be CV_16U in mm
         * \param[out] lmap_out the smoothed output labelmap as cvMat
         * \param[in] patch_size make the patch size for smoothing
         * \param[in] depthThres the z-distance threshold
         * \todo add a Gaussian contribution function to depth and vote
         **/
        //inline void smoothLabelImage ( cv::Mat&      lmap_in,
        //                        cv::Mat&      dmap,
        //                        cv::Mat&      lmap_out,
        //                        unsigned int  patch_size,
        //                        unsigned int  depthThres)
        //{
        //  // check depth
        //  assert(lmap_in.depth() == CV_8UC1);
        //  assert(dmap.depth() == CV_16U);
        //  assert(lmap_out.depth() == CV_8UC1);
        //  // check size
        //  assert(lmap_in.rows == dmap.rows);
        //  assert(lmap_in.cols == dmap.cols);
        //  assert(lmap_out.rows == dmap.rows);
        //  assert(lmap_out.cols == dmap.cols);

        //  unsigned int half_patch = static_cast<int> (patch_size/2);

        //  // iterate over the height of the image (from 2 till 478)
        //  for(unsigned int h = (0 + half_patch); h < (lmap_in.rows - half_patch); h++)
        //  {
        //    // iterate over the width of the image (from 2 till 638)
        //    for(unsigned int w = (0 + half_patch); w < (lmap_in.cols - half_patch); w++)
        //    {
        //      short depth = dmap.at<short>(h, w);
        //      unsigned int votes[NUM_PARTS];
        //      //this should be unneeded but to test
        //      for(int j = 0 ; j< NUM_PARTS; j++)
        //        votes[j] = 0;

        //      // iterate over the size of the patch in the height
        //      for(unsigned int h_l = (h - half_patch); h_l <= (h + half_patch); h_l++)
        //      {
        //        // iterate over the size of the patch in the width
        //        for(unsigned int w_l = (w - half_patch); w_l <= (w + half_patch); w_l++)
        //        {
        //          // get the depth of this part of the patch
        //          short depth_l = dmap.at<short>(h_l,w_l);
        //          // evaluate the difference to the centroid 
        //          if(std::abs(depth - depth_l) < static_cast<int> (depthThres))
        //          {
        //            char label = lmap_in.at<char>(h_l,w_l);
        //            if(label >= 0 && label < NUM_PARTS)
        //              votes[static_cast<unsigned int> (label)]++;
        //            else
        //              std::cout << "(E) : smoothLabelImage(): I've read a label that is non valid" << std::endl;
        //          }
        //        }
        //      }

        //      unsigned int max = 0;
        //      char label = lmap_in.at<char>(h,w);

        //      // iterate over the bin to find the max
        //      for(char i=0; i<NUM_PARTS; i++)
        //      {
        //        if(votes[static_cast<int> (i)] > max)
        //        {
        //          max = votes[static_cast<int> (i)];
        //          label = i;
        //        }
        //      }
        //      lmap_out.at<char>(h,w) = label;
        //    }
        //  }
        //}

        /*
         * \brief this function smooths the label image based on label and depth
         * \param[in] lmap_in the cvMat with the labels, must be CV_8UC1
         * \param[in] dmap the cvMat with the depths, must be CV_16U in mm
         * \param[out] lmap_out the smoothed output labelmap as cvMat
         * \todo make the patch size a parameter
         * \todo make the z-distance a parameter
         * \todo add a Gaussian contribution function to depth and vote
         **/
        //inline void smoothLabelImage2 ( cv::Mat&  lmap_in,
        //                        cv::Mat&  dmap,
        //                        cv::Mat&  lmap_out)
        //{
        //  // check depth
        //  assert(lmap_in.depth() == CV_8UC1);
        //  assert(dmap.depth() == CV_16U);
        //  assert(lmap_out.depth() == CV_8UC1);
        //  // check size
        //  assert(lmap_in.rows == dmap.rows);
        //  assert(lmap_in.cols == dmap.cols);
        //  assert(lmap_out.rows == dmap.rows);
        //  assert(lmap_out.cols == dmap.cols);

        //  //unsigned int patch_size = 5;
        //  unsigned int half_patch = 2;
        //  unsigned int depthThres = 300; // Think this is in mm, verify this!!!!!

        //  // iterate over the height of the image (from 2 till 478)
        //  unsigned int endrow = (lmap_in.rows - half_patch);
        //  unsigned int endcol = (lmap_in.cols - half_patch);
        //  for(unsigned int h = (0 + half_patch); h < endrow; h++)
        //  {
        //    unsigned int endheight = (h + half_patch);

        //    // iterate over the width of the image (from 2 till 638)
        //    for(unsigned int w = (0 + half_patch); w < endcol; w++)
        //    {
        //      unsigned int endwidth = (w + half_patch);

        //      short depth = dmap.at<short>(h, w);
        //      unsigned int votes[NUM_PARTS];
        //      //this should be unneeded but to test
        //      for(int j = 0 ; j< NUM_PARTS; j++)
        //        votes[j] = 0;

        //      // iterate over the size of the patch in the height
        //      for(unsigned int h_l = (h - half_patch); h_l <= endheight; h_l++)
        //      {
        //        // iterate over the size of the patch in the width
        //        for(unsigned int w_l = (w - half_patch); w_l <= endwidth; w_l++)
        //        {
        //          // get the depth of this part of the patch
        //          short depth_l = dmap.at<short>(h_l,w_l);
        //          // evaluate the difference to the centroid 
        //          if(std::abs(depth - depth_l) < static_cast<int> (depthThres))
        //          {
        //            char label = lmap_in.at<char>(h_l,w_l);
        //            if(label >= 0 && label < NUM_PARTS)
        //              votes[static_cast<unsigned int>(label)]++;
        //            else
        //              std::cout << "(E) : smoothLabelImage(): I've read a label that is non valid" << std::endl;
        //          }
        //        }
        //      }

        //      unsigned int max = 0;
        //      char label = lmap_in.at<char>(h,w);

        //      // iterate over the bin to find the max
        //      for(char i=0; i<NUM_PARTS; i++)
        //      {
        //        if(votes[static_cast<unsigned int>(i)] > max)
        //        {
        //          max = votes[static_cast<unsigned int>(i)];
        //          label = i;
        //        }
        //      }
        //      lmap_out.at<char>(h,w) = label;
        //    }
        //  }
        //}

        /**
         * @brief this function smooths the label image based on label and depth
         * @param[in] lmap_in the cvMat with the labels, must be CV_8UC1
         * @param[in] dmap the cvMat with the depths, must be CV_16U in mm
         * @param[out] lmap_out the smoothed output labelmap as cvMat
         * @todo make the patch size a parameter
         * @todo make the z-distance a parameter
         * @todo add a Gaussian contribution function to depth and vote
         **/
        inline void smoothLabelImage ( cv::Mat&  lmap_in,
                                cv::Mat&  dmap,
                                cv::Mat&  lmap_out)
        {
          // check depth
          assert(lmap_in.depth() == CV_8UC1);
          assert(dmap.depth() == CV_16U);
          assert(lmap_out.depth() == CV_8UC1);
          // check size
          assert(lmap_in.rows == dmap.rows);
          assert(lmap_in.cols == dmap.cols);
          assert(lmap_out.rows == dmap.rows);
          assert(lmap_out.cols == dmap.cols);

          //unsigned int patch_size = 5;
          unsigned int half_patch = 2;
          unsigned int depthThres = 300; // Think this is in mm, verify this!!!!!

          // iterate over the height of the image (from 2 till 478)
          unsigned int endrow = (lmap_in.rows - half_patch);
          unsigned int endcol = (lmap_in.cols - half_patch);
          unsigned int votes[NUM_PARTS];
          unsigned int endheight, endwidth;
          const short* drow;
          char *loutrow;
          short depth;
          const short* drow_offset;
          const char* lrow_offset;
          short depth_l;
          char label;
          for(unsigned int h = (0 + half_patch); h < endrow; h++)
          {
            endheight = (h + half_patch);

            drow = dmap.ptr<short>(h);
            loutrow = lmap_out.ptr<char>(h);

            // iterate over the width of the image (from 2 till 638)
            for(unsigned int w = (0 + half_patch); w < endcol; w++)
            {
              endwidth = (w + half_patch);

              depth = drow[w];
              // reset votes
              for(int j = 0 ; j< NUM_PARTS; j++)
                votes[j] = 0;

              // iterate over the size of the patch in the height
              for(unsigned int h_l = (h - half_patch); h_l <= endheight; h_l++)
              {
                drow_offset = dmap.ptr<short>(h_l);
                lrow_offset = lmap_in.ptr<char>(h_l);

                // iterate over the size of the patch in the width
                for(unsigned int w_l = (w - half_patch); w_l <= endwidth; w_l++)
                {
                  // get the depth of this part of the patch
                  depth_l = drow_offset[w_l];
                  // evaluate the difference to the centroid 
                  if(std::abs(depth - depth_l) < static_cast<int> (depthThres))
                  {
                    label = lrow_offset[w_l];
                    votes[static_cast<unsigned int>(label)]++;
                  }
                }
              }

              unsigned int max = 0;

              // iterate over the bin to find the max
              for(char i=0; i<NUM_PARTS; i++)
              {
                if(votes[static_cast<unsigned int>(i)] > max)
                {
                  max = votes[static_cast<unsigned int>(i)];
                  loutrow[w] = i;
                }
              }
            }
          }
        }

        /**
         * @brief This function takes a number of indices with label and sorts them in the blob matrix
         * @param[in] cloud_in the original input pointcloud from which everything was calculated
         * @param[in] sizeThres the minimal size needed to be considered as a valid blob
         * @param[out] sorted the matrix in which every blob will be pushed back
         * @param[in] indices the matrix of PointIndices
         * @todo implement the eigenvalue evaluation again
         * @todo do we still need sizeThres?
         **/
        inline void sortIndicesToBlob2 ( const pcl::PointCloud<pcl::PointXYZ>&                       cloud_in,
                                  unsigned int                                                          sizeThres,
                                  std::vector< std::vector<Blob2, Eigen::aligned_allocator<Blob2> > >&  sorted,
                                  std::vector< std::vector<pcl::PointIndices> >&                        indices)
        {
          assert(sorted.size () == indices.size ());

          unsigned int id = 0;
          // Iterate over all labels
          for(unsigned int lab = 0; lab < indices.size(); ++lab)
          {
            unsigned int lid = 0;
            // Iterate over all blobs of this label
            for(unsigned int l = 0; l < indices[lab].size(); l++)
            {
              // If the blob has enough pixels
              if(indices[lab][l].indices.size() > sizeThres)
              {
                Blob2 b;

                b.indices = indices[lab][l];

                pcl::compute3DCentroid(cloud_in, b.indices, b.mean);
#ifdef NEED_STATS
                pcl::computeCovarianceMatrixNormalized(cloud_in, b.indices, b.mean, b.cov);
                pcl::getMinMax3D(cloud_in, b.indices, b.min, b.max);
                pcl::eigen33(b.cov, b.eigenvect, b.eigenval);
#endif
                // Check if it is a valid blob
                //if((b.eigenval(0) < LUT_max_part_size[(int) lab]) && (b.mean(2) != 0))
                if((b.mean(2) != 0))
                {
                  b.id = id;
                  id++;
                  b.label = static_cast<part_t> (lab);
                  b.lid = lid;
                  lid++;
                  sorted[lab].push_back(b); 
                }
              }
            }
          }         
        }

        /**
         * @brief This function is a stupid helper function to debug easilier, it print out how the matrix was sorted
         * @param[in] sorted the matrix of blobs
         * @return Zero if everything went well
         **/
        inline int giveSortedBlobsInfo ( std::vector<std::vector<Blob2, Eigen::aligned_allocator<Blob2> > >& sorted)
        {
          std::cout << "(I) : giveSortedBlobsInfo(): Size of outer vector: " << sorted.size() << std::endl;
          for(unsigned int i = 0; i < sorted.size(); i++)
          {
            std::cout << "(I) : giveSortedBlobsInfo(): Found " << sorted[i].size() << " parts of type " << i << std::endl;
            std::cout << "(I) : giveSortedBlobsInfo(): indices : ";
            for(unsigned int j = 0; j < sorted[i].size(); j++)
            {
              std::cout << " id:" << sorted[i][j].id << " lid:" << sorted[i][j].lid;
            }
            std::cout << std::endl;
          }
          return 0;
        }
      } // end namespace label_skeleton
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl

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


#ifndef PCL_GPU_PEOPLE_RDF_BODYPARTS_DETECTOR_H
#define PCL_GPU_PEOPLE_RDF_BODYPARTS_DETECTOR_H

#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/gpu/people/label_blob2.h>
#include <pcl/gpu/people/label_common.h>
#include "pcl/gpu/people/person_attribs.h"
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

namespace pcl
{
  namespace device
  {
    // This just seems hacky ...
    class MultiTreeLiveProc;
  }

  namespace gpu
  {
    namespace people
    {
      class PCL_EXPORTS RDFBodyPartsDetector
      {
        public:
          typedef boost::shared_ptr<RDFBodyPartsDetector> Ptr;          
          typedef std::vector<std::vector<Blob2, Eigen::aligned_allocator<Blob2> > > BlobMatrix;
          
          typedef DeviceArray2D<unsigned char> Labels;
          typedef DeviceArray2D<unsigned short> Depth;
          typedef DeviceArray2D<pcl::RGB> Image;

          /** \brief This is the constructor **/
          RDFBodyPartsDetector(const std::vector<std::string>& tree_files,
              int default_buffer_rows = 480, int default_buffer_cols = 640);

          /**
           * \brief This function does the complete RDF evaluation in a discrete manner and builds the blob matrix
           */
          void process(const Depth& depth, const PointCloud<PointXYZ>& cloud, int min_pts_per_cluster);
          // This are the different sub-parts of process()
          /**
           * \brief This function processes based on the RDF with probabilistic voting scheme
           */
          void processProb (const Depth& depth);

          /**
           * \brief This smooths the labels and does the connected components
           */
          void processSmooth (const Depth& depth, const PointCloud<PointXYZ>& cloud, int min_pts_per_cluster);

          /**
           * \brief This processes the blob_matrix_
           * \return negative if failed
           */
          int
          processRelations ();

          /**
           * \brief This processes the blob_matrix_
           * \param[in] person_attribs the custom person parameters
           * \return negative if failed
           */
          int
          processRelations (PersonAttribs::Ptr person_attribs);

          //getters
          const Labels& getLabels() const;
          const pcl::device::LabelProbability& getProbability() const;
          const pcl::device::LabelProbability& getProbability1() const;
          const pcl::device::LabelProbability& getProbability2() const;
          const pcl::device::LabelProbability& getPrevProbability1() const;
          const pcl::device::LabelProbability& getPrevProbability2() const;
          size_t getNumberTrees() const;
          const BlobMatrix& getBlobMatrix() const;

          
          /** \brief This contains the final body part labels **/
          Labels labels_;
          /** \brief This contains the smoothed final body part labels **/
          Labels labels_smoothed_;

          /** These contain the histograms of the labels for this detector **/
          pcl::device::LabelProbability P_l_;             // the one is current worked in
          pcl::device::LabelProbability P_l_Gaus_;        // the current Gaussian buffer
          pcl::device::LabelProbability P_l_Gaus_Temp_;   // the current Gaussian buffer to store the intermediate
          pcl::device::LabelProbability P_l_1_;           // storage for the first iteration
          pcl::device::LabelProbability P_l_2_;           // storage for the second iteration

          /** These contain the histograms of the labels for this detector of the previous timestep **/
          pcl::device::LabelProbability P_l_prev_1_;  // for the first iteration
          pcl::device::LabelProbability P_l_prev_2_;  // for the second iteration

        private:
          boost::shared_ptr<device::MultiTreeLiveProc> impl_;
          
          int max_cluster_size_;
          float cluster_tolerance_;

          BlobMatrix blob_matrix_;

          // hide this buffer using pImpl
          std::vector<unsigned char>    lmap_host_;
          std::vector<int>              dst_labels_;
          std::vector<int>              region_sizes_;
          std::vector<int>              remap_;
          std::vector<float>            means_storage_;
          DeviceArray2D<int>            comps_;
          DeviceArray2D<unsigned char>  edges_;

          void allocate_buffers(int rows = 480, int cols = 640);
      };
    }
  }
}

#endif /* PCL_GPU_PEOPLE_RDF_BODYPARTS_DETECTOR_H */

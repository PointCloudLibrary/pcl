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
 * @author: Koen Buys
 */

#ifndef PCL_GPU_PEOPLE_FACE_DETECTOR_H_
#define PCL_GPU_PEOPLE_FACE_DETECTOR_H_

#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>

#include "NCVHaarObjectDetection.hpp"

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      class FaceDetector
      {
        public:
          typedef boost::shared_ptr<FaceDetector> Ptr;
          //typedef DeviceArray2D<unsigned char> Labels;
          //typedef DeviceArray2D<unsigned short> Depth;
          //typedef DeviceArray2D<pcl::RGB> Image;

          /** \brief This is the constructor **/
          FaceDetector ( int cols, int rows);

          NCVStatus
          loadFromXML2(const std::string                   &filename,
                       HaarClassifierCascadeDescriptor     &haar,
                       std::vector<HaarStage64>            &haar_stages,
                       std::vector<HaarClassifierNode128>  &haarClassifierNodes,
                       std::vector<HaarFeature64>          &haar_features);

          static NCVStatus
          loadFromNVBIN(const std::string &filename,
                        HaarClassifierCascadeDescriptor &haar,
                        std::vector<HaarStage64> &haar_stages,
                        std::vector<HaarClassifierNode128> &haarClassifierNodes,
                        std::vector<HaarFeature64> &haar_features);

          NCVStatus
          ncvHaarLoadFromFile_host(const std::string &filename,
                                   HaarClassifierCascadeDescriptor &haar,
                                   NCVVector<HaarStage64> &h_haar_stages,
                                   NCVVector<HaarClassifierNode128> &h_haar_nodes,
                                   NCVVector<HaarFeature64> &h_haar_features);

          NCVStatus
          ncvHaarGetClassifierSize(const std::string &filename, Ncv32u &numStages,
                                   Ncv32u &numNodes, Ncv32u &numFeatures);

          NCVStatus
          NCVprocess(pcl::PointCloud<pcl::RGB>&           cloud_in,
                     pcl::PointCloud<pcl::Intensity32u>&  cloud_out,
                     HaarClassifierCascadeDescriptor      &haar,
                     NCVVector<HaarStage64>               &d_haar_stages,
                     NCVVector<HaarClassifierNode128>     &d_haar_nodes,
                     NCVVector<HaarFeature64>             &d_haar_features,
                     NCVVector<HaarStage64>               &h_haar_stages,
                     INCVMemAllocator                     &gpu_allocator,
                     INCVMemAllocator                     &cpu_allocator,
                     cudaDeviceProp                       &device_properties,
                     Ncv32u                               width=640,
                     Ncv32u                               height=480,
                     NcvBool                              bFilterRects=false,
                     NcvBool                              bLargestFace=true);

          int
          configure (std::string cascade_file_name);

          /** \brief Process step, this wraps the Nvidia code **/
          void process (pcl::PointCloud<pcl::RGB>& cloud,
                        pcl::PointCloud<pcl::Intensity32u>& cloud_out);

          /** \brief largest object sets return configuration **/
          inline void setLargestObject (bool largest_object)
          {
            largest_object_ = largest_object;
          }

          inline bool getLargestObject () const
          {
            return largest_object_;
          }

          /** \brief Set the cuda GPU to use **/
          void setDeviceId (int id);

          int getCols () const
          {
            return cols_;
          }

          void setCols (int cols)
          {
            cols_ = cols;
          }

          int getRows () const
          {
            return rows_;
          }

          void setRows (int rows)
          {
            rows_ = rows;
          }

          std::string
          getCascadeFileName () const
          {
            return cascade_file_name_;
          }

          void
          setCascadeFileName (std::string cascadeFileName)
          {
            cascade_file_name_ = cascadeFileName;
          }

          /** \brief Get the cuda GPU device id in use **/
          int
          getDeviceId() {return cuda_dev_id_;}

        private:
          bool                largest_object_;      /** \brief only give back largest object **/
          bool                filter_rects_;        /** \brief rectangular filter **/

          int                 cuda_dev_id_;         /** \brief indicates which GPU to use for this **/
          cudaDeviceProp      cuda_dev_prop_;

          std::string         cascade_file_name_;

          int                 rows_;                // should default to 480
          int                 cols_;                // should default to 640

          HaarClassifierCascadeDescriptor         haar_clas_casc_descr_;
          NCVVectorAlloc<HaarStage64>*            haar_stages_dev_;
          NCVVectorAlloc<HaarStage64>*            haar_stages_host_;
          NCVVectorAlloc<HaarClassifierNode128>*  haar_nodes_dev_;
          NCVVectorAlloc<HaarClassifierNode128>*  haar_nodes_host_;
          NCVVectorAlloc<HaarFeature64>*          haar_features_dev_;
          NCVVectorAlloc<HaarFeature64>*          haar_features_host_;

          INCVMemAllocator*                       gpu_allocator_;
          INCVMemAllocator*                       cpu_allocator_;

          NCVMemStackAllocator*                   gpu_stack_allocator_;
          NCVMemStackAllocator*                   cpu_stack_allocator_;

          NCVMemStackAllocator*                   gpu_counter_;
          NCVMemStackAllocator*                   cpu_counter_;

      };
    }
  }
}


#endif /* PCL_GPU_PEOPLE_FACE_DETECTOR_H_ */

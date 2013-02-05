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
          loadFromXML(const std::string &filename,
                      HaarClassifierCascadeDescriptor &haar,
                      std::vector<HaarStage64> &haarStages,
                      std::vector<HaarClassifierNode128> &haarClassifierNodes,
                      std::vector<HaarFeature64> &haarFeatures);

          static NCVStatus
          loadFromNVBIN(const std::string &filename,
                        HaarClassifierCascadeDescriptor &haar,
                        std::vector<HaarStage64> &haarStages,
                        std::vector<HaarClassifierNode128> &haarClassifierNodes,
                        std::vector<HaarFeature64> &haarFeatures);

          NCVStatus
          ncvHaarLoadFromFile_host(const std::string &filename,
                                   HaarClassifierCascadeDescriptor &haar,
                                   NCVVector<HaarStage64> &h_HaarStages,
                                   NCVVector<HaarClassifierNode128> &h_HaarNodes,
                                   NCVVector<HaarFeature64> &h_HaarFeatures);

          NCVStatus
          ncvHaarGetClassifierSize(const std::string &filename, Ncv32u &numStages,
                                   Ncv32u &numNodes, Ncv32u &numFeatures);

          NCVStatus
          NCVprocess(pcl::PointCloud<pcl::RGB>,
                     Ncv32u width,
                     Ncv32u height,
                     NcvBool bFilterRects,
                     NcvBool bLargestFace,
                     HaarClassifierCascadeDescriptor &haar,
                     NCVVector<HaarStage64> &d_haarStages,
                     NCVVector<HaarClassifierNode128> &d_haarNodes,
                     NCVVector<HaarFeature64> &d_haarFeatures,
                     NCVVector<HaarStage64> &h_haarStages,
                     INCVMemAllocator &gpuAllocator,
                     INCVMemAllocator &cpuAllocator,
                     cudaDeviceProp &devProp);

          int
          configure (std::string cascade_file_name);

          /** \brief Process step, this wraps the Nvidia code **/
          void process ();

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
          void allocate_buffers(int rows = 480, int cols = 640);

          bool                largest_object_;      /** \brief only give back largest object **/

          int                 cuda_dev_id_;         /** \brief indicates which GPU to use for this **/
          cudaDeviceProp      cuda_dev_prop_;

          std::string         cascade_file_name_;

          int                 rows_;                // should default to 480
          int                 cols_;                // should default to 640

      };
    }
  }
}


#endif /* PCL_GPU_PEOPLE_FACE_DETECTOR_H_ */

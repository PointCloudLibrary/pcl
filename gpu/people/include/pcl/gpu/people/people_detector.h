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

#ifndef PCL_GPU_PEOPLE_PERSON_H_
#define PCL_GPU_PEOPLE_PERSON_H_

#include <iostream>
#include <sstream>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/people/label_common.h>
#include <pcl/gpu/people/tree.h>
#include <pcl/gpu/people/person_attribs.h>
#include <opencv2/core/core.hpp>

#include <pcl/gpu/people/rdf_bodyparts_detector.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      struct FaceDetector
      {
          typedef boost::shared_ptr<FaceDetector> Ptr;
      };

      struct OtherDetector
      {
          typedef boost::shared_ptr<OtherDetector> Ptr;
      };

      class PCL_EXPORTS PeopleDetector
      {
        public:

          typedef boost::shared_ptr<PeopleDetector> Ptr;
          typedef pcl::PointXYZRGB InputPointT;

          DeviceArray<InputPointT> cloud_device_;

          typedef DeviceArray2D<unsigned short> Depth;
          typedef DeviceArray2D<unsigned char> Labels;
          typedef DeviceArray2D<unsigned char> Mask;
          typedef DeviceArray2D<pcl::RGB> Image;

          Depth depth_device_;
          Depth depth_device2_;
          Mask fg_mask_;
          Mask fg_mask_grown_;

          RDFBodyPartsDetector::Ptr rdf_detector_;
          FaceDetector::Ptr face_detector_;
          OtherDetector::Ptr other_detector_;
          PersonAttribs::Ptr person_attribs_;

          /** \brief Class constructor. */
          PeopleDetector () : number_of_parts_(25), delta_hue_tolerance_(5),
                  do_shs_(true), dilation_size_(2)
          {
            allocate_buffers(480, 640);
          }

          /** \brief Class destructor. */
          ~PeopleDetector () {}

          //// PUBLIC METHODS /////

          /** \brief The actuall processing callback */
          void
          process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

          /** \brief Set the number of body parts used in the RDF, defaults to 25 */
          inline void
          setNumberOfParts (unsigned int number_of_parts)
          {
            number_of_parts_ = number_of_parts;
          }
          /** \brief Get the number of body parts used in the RDF, defaults to 25 */
          inline unsigned int
          getNumberOfParts () const
          {
            return (number_of_parts_);
          }

          /** \brief Set the tolerance for the delta on the Hue in Seeded Hue Segmentation step */
          inline void
          setDeltaHueTolerance (unsigned int delta_hue_tolerance)
          {
            delta_hue_tolerance_ = delta_hue_tolerance;
          }

          /** \brief Get the tolerance for the delta on the Hue in Seeded Hue Segmentation step, defaults to 5 */
          inline unsigned int
          getDeltaHueTolerance () const
          {
            return (delta_hue_tolerance_);
          }

          /** \brief if set the proces step will do a second iteration with SHS */
          inline void
          setDoSHS (bool do_shs)
          {
            do_shs_ = do_shs;
          }
          /** \brief returns if the process step does or doesn't do a reiteration with SHS */
          inline bool
          getDoSHS () const
          {
            return (do_shs_);
          }
          /** \brief Sets how much the results from SHS are dilated before annotating them */
          inline void
          setDilationSize (unsigned int dilation_size)
          {
            dilation_size_ = dilation_size;
          }
          /** \brief Returns the dilation size in second iteration */
          inline unsigned int 
          getDilationSize () const
          {
            return (dilation_size_);
          }

          /** \brief Class getName method. */
          virtual std::string getClassName () const { return "PeopleDetector"; }

        public:
          unsigned int  number_of_parts_;
          unsigned int  delta_hue_tolerance_;
          bool          do_shs_;
          unsigned int  dilation_size_;

          DeviceArray<unsigned char> kernelRect5x5_;

          void allocate_buffers(int rows, int cols);
      };
    }
  }
}
#endif // PCL_GPU_PEOPLE_PERSON_H_

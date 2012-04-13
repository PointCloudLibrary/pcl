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

      class PCL_EXPORTS Person
      {
        public:

          typedef boost::shared_ptr<Person> Ptr;
          typedef pcl::PointXYZRGB InputPointT;

          DeviceArray<InputPointT> cloud_device_;

          typedef DeviceArray2D<unsigned short> Depth;
          typedef DeviceArray2D<unsigned char> Labels;
          typedef DeviceArray2D<pcl::RGB> Image;

          Depth depth_device_;
          Depth depth_device2_;

          Image cmap_device_;

          RDFBodyPartsDetector::Ptr rdf_detector_;
          FaceDetector::Ptr face_detector_;
          OtherDetector::Ptr other_detector_;

          /** \brief Class constructor. */
          Person () : number_of_parts_(25), delta_hue_tolerance_(5),                  
                  do_shs_(true), dilation_size_(2),  name_("Generic"), counter_(0) 
          {}

          /** \brief Class destructor. */
          ~Person () {}

          //// PUBLIC METHODS /////               

          /** \brief The actuall processing callback */
          void
          process (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

          /** \brief Read XML configuration file for a specific person */
          void 
          readPersonXMLConfig (std::istream& is);

          /** \brief Write XML configuration file for a specific person */
          void 
          writePersonXMLConfig (std::ostream& os);

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
          virtual std::string getClassName () const { return ("Person"); }

        public:          
          unsigned int                                number_of_parts_;                    
          unsigned int                                delta_hue_tolerance_;          
          bool                                        do_shs_;
          unsigned int                                dilation_size_;
          unsigned int                                counter_;
          
          std::string                                 name_;                  // Name of the person
          std::vector<float>                          max_part_size_;         // Max primary eigenvalue for each body part
          std::vector<std::vector<float> >            part_ideal_length_;     // Ideal length between two body parts
          std::vector<std::vector<float> >            max_length_offset_;     // Max allowed length offset between two body parts
      };
    }
  }
}
#endif // PCL_GPU_PEOPLE_PERSON_H_

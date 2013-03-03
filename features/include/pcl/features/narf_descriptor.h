/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#ifndef PCL_NARF_DESCRIPTOR_H_
#define PCL_NARF_DESCRIPTOR_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic ignored "-Weffc++"
#endif
namespace pcl
{
  // Forward declarations
  class RangeImage;

  /** @b Computes NARF feature descriptors for points in a range image
    * See B. Steder, R. B. Rusu, K. Konolige, and W. Burgard
    *     Point Feature Extraction on 3D Range Scans Taking into Account Object Boundaries
    *     In Proc. of the IEEE Int. Conf. on Robotics &Automation (ICRA). 2011. 
    * \author Bastian Steder
    * \ingroup features
    */
  class PCL_EXPORTS NarfDescriptor : public Feature<PointWithRange,Narf36>
  {
    public:
      typedef boost::shared_ptr<NarfDescriptor> Ptr;
      typedef boost::shared_ptr<const NarfDescriptor> ConstPtr;
      // =====TYPEDEFS=====
      typedef Feature<PointWithRange,Narf36> BaseClass;
      
      // =====STRUCTS/CLASSES=====
      struct Parameters
      {
        Parameters() : support_size(-1.0f), rotation_invariant(true) {}
        float support_size;
        bool rotation_invariant;
      };
      
      // =====CONSTRUCTOR & DESTRUCTOR=====
      /** Constructor */
      NarfDescriptor (const RangeImage* range_image=NULL, const std::vector<int>* indices=NULL);
      /** Destructor */
      virtual ~NarfDescriptor();
      
      // =====METHODS=====
      //! Set input data
      void 
      setRangeImage (const RangeImage* range_image, const std::vector<int>* indices=NULL);
      
      //! Overwrite the compute function of the base class
      void 
      compute (PointCloudOut& output);
      
      // =====GETTER=====
      //! Get a reference to the parameters struct
      Parameters& 
      getParameters () { return parameters_;}
      
    protected:
      // =====PROTECTED MEMBER VARIABLES=====
      const RangeImage* range_image_;
      Parameters parameters_;
      
      // =====PROTECTED METHODS=====
      /** Implementation of abstract derived function */
      virtual void 
      computeFeature (PointCloudOut& output);
  };

}  // namespace end
#if defined BUILD_Maintainer && defined __GNUC__ && __GNUC__ == 4 && __GNUC_MINOR__ > 3
#pragma GCC diagnostic warning "-Weffc++"
#endif

#endif  //#ifndef PCL_NARF_DESCRIPTOR_H_

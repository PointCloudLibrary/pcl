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

#pragma once

#include <Eigen/Core> // for Vector3f, Matrix
#include <Eigen/Geometry> // for Affine3f

namespace pcl 
{
  /**
    * \brief Calculates a transformation based on corresponding 3D points
    * \author Bastian Steder
    * \ingroup common
    */
  class TransformationFromCorrespondences 
  {
     public:
        //-----CONSTRUCTOR&DESTRUCTOR-----
        /** Constructor - dimension gives the size of the vectors to work with. */
        TransformationFromCorrespondences () 
        { reset (); }

        //-----METHODS-----
        /** Reset the object to work with a new data set */
        inline void 
        reset ();
        
        /** Get the summed up weight of all added vectors */
        inline float 
        getAccumulatedWeight () const { return accumulated_weight_;}
        
        /** Get the number of added vectors */
        inline unsigned int 
        getNoOfSamples () const { return no_of_samples_;}
        
        /** Add a new sample */
        inline void 
        add (const Eigen::Vector3f& point, const Eigen::Vector3f& corresponding_point, float weight=1.0);
        
        /** Calculate the transformation that will best transform the points into their correspondences */
        inline Eigen::Affine3f 
        getTransformation ();
        
        //-----VARIABLES-----
        
     protected:
        //-----METHODS-----
        //-----VARIABLES-----
        unsigned int no_of_samples_ = 0;
        float accumulated_weight_ = 0;
        Eigen::Vector3f mean1_ = Eigen::Vector3f::Identity ();
        Eigen::Vector3f mean2_ = Eigen::Vector3f::Identity ();
        Eigen::Matrix<float, 3, 3> covariance_ = Eigen::Matrix<float, 3, 3>::Identity ();
  };

}  // END namespace

#include <pcl/common/impl/transformation_from_correspondences.hpp>

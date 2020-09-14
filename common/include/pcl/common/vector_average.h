/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#pragma once

#include <Eigen/Core> // for Matrix

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

namespace pcl 
{
  /** \brief Calculates the weighted average and the covariance matrix
    *
    * A class to calculate the weighted average and the covariance matrix of a set of vectors with given weights.
    * The original data is not saved. Mean and covariance are calculated iteratively.
    * \author Bastian Steder
    * \ingroup common
    */
  template <typename real, int dimension>
  class VectorAverage
  {
     public:
         using VectorType = Eigen::Matrix<real, dimension, 1>;
         using MatrixType = Eigen::Matrix<real, dimension, dimension>;
        //-----CONSTRUCTOR&DESTRUCTOR-----
        /** Constructor - dimension gives the size of the vectors to work with. */
        VectorAverage ();

        //-----METHODS-----
        /** Reset the object to work with a new data set */
        inline void 
        reset ();
        
        /** Get the mean of the added vectors */
        inline const
        VectorType& getMean () const { return mean_;}
        
        /** Get the covariance matrix of the added vectors */
        inline const
        MatrixType& getCovariance () const { return covariance_;}
        
        /** Get the summed up weight of all added vectors */
        inline real
        getAccumulatedWeight () const { return accumulatedWeight_;}
        
        /** Get the number of added vectors */
        inline unsigned int
        getNoOfSamples () { return noOfSamples_;}
        
        /** Add a new sample */
        inline void
        add (const VectorType& sample, real weight=1.0);

        /** Do Principal component analysis */
        inline void
        doPCA (VectorType& eigen_values, VectorType& eigen_vector1,
               VectorType& eigen_vector2, VectorType& eigen_vector3) const;
        
        /** Do Principal component analysis */
        inline void
        doPCA (VectorType& eigen_values) const;
        
        /** Get the eigenvector corresponding to the smallest eigenvalue */
        inline void
        getEigenVector1 (VectorType& eigen_vector1) const;

        PCL_MAKE_ALIGNED_OPERATOR_NEW
        
        //-----VARIABLES-----

        
     protected:
        //-----METHODS-----
        //-----VARIABLES-----
        unsigned int noOfSamples_ = 0;
        real accumulatedWeight_ = 0;
        VectorType mean_ = VectorType::Identity ();
        MatrixType covariance_ = MatrixType::Identity ();
  };

  using VectorAverage2f = VectorAverage<float, 2>;
  using VectorAverage3f = VectorAverage<float, 3>;
  using VectorAverage4f = VectorAverage<float, 4>;
}  // END namespace

#include <pcl/common/impl/vector_average.hpp>

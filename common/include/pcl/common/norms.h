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
 *
 */

#pragma once

/**
  * \file norms.h
  * Define standard C methods to calculate different norms
  * \ingroup common
  */

/*@{*/
namespace pcl
{
  /** \brief Enum that defines all the types of norms available.
   * \note Any new norm type should have its own enum value and its own case in the selectNorm () method
   * \ingroup common
   */
  enum NormType {L1, L2_SQR, L2, LINF, JM, B, SUBLINEAR, CS, DIV, PF, K, KL, HIK};

  /** \brief Method that calculates any norm type available, based on the norm_type variable
   * \note FloatVectorT is any type of vector with its values accessible via [ ]
   * \ingroup common
   * */
  template <typename FloatVectorT> inline float
  selectNorm (FloatVectorT A, FloatVectorT B, int dim, NormType norm_type);

  /** \brief Compute the L1 norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  L1_Norm (FloatVectorT A, FloatVectorT B, int dim);
  
  /** \brief Compute the squared L2 norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  L2_Norm_SQR (FloatVectorT A, FloatVectorT B, int dim);
  
  /** \brief Compute the L2 norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  L2_Norm (FloatVectorT A, FloatVectorT B, int dim);

  /** \brief Compute the L-infinity norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */  
  template <typename FloatVectorT> inline float
  Linf_Norm (FloatVectorT A, FloatVectorT B, int dim);

  /** \brief Compute the JM norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  JM_Norm (FloatVectorT A, FloatVectorT B, int dim);

  /** \brief Compute the B norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  B_Norm (FloatVectorT A, FloatVectorT B, int dim);

  /** \brief Compute the sublinear norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  Sublinear_Norm (FloatVectorT A, FloatVectorT B, int dim);

  /** \brief Compute the CS norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  CS_Norm (FloatVectorT A, FloatVectorT B, int dim);

  /** \brief Compute the div norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  Div_Norm (FloatVectorT A, FloatVectorT B, int dim);

  /** \brief Compute the PF norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \param P1 the first parameter
    * \param P2 the second parameter
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  PF_Norm (FloatVectorT A, FloatVectorT B, int dim, float P1, float P2);

  /** \brief Compute the K norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \param P1 the first parameter
    * \param P2 the second parameter
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  K_Norm (FloatVectorT A, FloatVectorT B, int dim, float P1, float P2);

  /** \brief Compute the KL between two discrete probability density functions
    * \param A the first discrete PDF
    * \param B the second discrete PDF
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  KL_Norm (FloatVectorT A, FloatVectorT B, int dim);

  /** \brief Compute the HIK norm of the vector between two points
    * \param A the first point
    * \param B the second point
    * \param dim the number of dimensions in \a A and \a B (dimensions must match)
    * \note FloatVectorT is any type of vector with its values accessible via [ ]
    * \ingroup common
    */
  template <typename FloatVectorT> inline float
  HIK_Norm (FloatVectorT A, FloatVectorT B, int dim);
}
/*@}*/
#include <pcl/common/impl/norms.hpp>

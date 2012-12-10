/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
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
 *  $Id$
 */

#ifndef PCL_NORMAL_BASED_SIGNATURE_H_
#define PCL_NORMAL_BASED_SIGNATURE_H_

#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief Normal-based feature signature estimation class. Obtains the feature vector by applying Discrete Cosine and
    * Fourier Transforms on an NxM array of real numbers representing the projection distances of the points in the input
    * cloud to a disc around the point of interest.
    * Please consult the following publication for more details:
    *    Xinju Li and Igor Guskov
    *    Multi-scale features for approximate alignment of point-based surfaces
    *    Proceedings of the third Eurographics symposium on Geometry processing
    *    July 2005, Vienna, Austria
    *
    * \note These features were meant to be used at keypoints detected by a detector using different smoothing radii
    * (e.g., SmoothedSurfacesKeypoint)
    * \author Alexandru-Eugen Ichim
    */
  template <typename PointT, typename PointNT, typename PointFeature>
  class NormalBasedSignatureEstimation : public FeatureFromNormals<PointT, PointNT, PointFeature>
  {
    public:
      using Feature<PointT, PointFeature>::input_;
      using Feature<PointT, PointFeature>::tree_;
      using Feature<PointT, PointFeature>::search_radius_;
      using PCLBase<PointT>::indices_;
      using FeatureFromNormals<PointT, PointNT, PointFeature>::normals_;

      typedef pcl::PointCloud<PointFeature> FeatureCloud;
      typedef typename boost::shared_ptr<NormalBasedSignatureEstimation<PointT, PointNT, PointFeature> > Ptr;
      typedef typename boost::shared_ptr<const NormalBasedSignatureEstimation<PointT, PointNT, PointFeature> > ConstPtr;



      /** \brief Empty constructor, initializes the internal parameters to the default values
        */
      NormalBasedSignatureEstimation ()
        : FeatureFromNormals<PointT, PointNT, PointFeature> (),
          scale_h_ (),
          N_ (36),
          M_ (8),
          N_prime_ (4),
          M_prime_ (3)
      {
      }

      /** \brief Setter method for the N parameter - the length of the columns used for the Discrete Fourier Transform. 
        * \param[in] n the length of the columns used for the Discrete Fourier Transform. 
        */
      inline void
      setN (size_t n) { N_ = n; }

      /** \brief Returns the N parameter - the length of the columns used for the Discrete Fourier Transform. */
      inline size_t
      getN () { return N_; }

      /** \brief Setter method for the M parameter - the length of the rows used for the Discrete Cosine Transform.
        * \param[in] m the length of the rows used for the Discrete Cosine Transform.
        */
      inline void
      setM (size_t m) { M_ = m; }

      /** \brief Returns the M parameter - the length of the rows used for the Discrete Cosine Transform */
      inline size_t
      getM () { return M_; }

      /** \brief Setter method for the N' parameter - the number of columns to be taken from the matrix of DFT and DCT
        * values that will be contained in the output feature vector
        * \note This value directly influences the dimensions of the type of output points (PointFeature)
        * \param[in] n_prime the number of columns from the matrix of DFT and DCT that will be contained in the output
        */
      inline void
      setNPrime (size_t n_prime) { N_prime_ = n_prime; }

      /** \brief Returns the N' parameter - the number of rows to be taken from the matrix of DFT and DCT
        * values that will be contained in the output feature vector
        * \note This value directly influences the dimensions of the type of output points (PointFeature)
        */
      inline size_t
      getNPrime () { return N_prime_; }

      /** \brief Setter method for the M' parameter - the number of rows to be taken from the matrix of DFT and DCT
        * values that will be contained in the output feature vector
        * \note This value directly influences the dimensions of the type of output points (PointFeature)
        * \param[in] m_prime the number of rows from the matrix of DFT and DCT that will be contained in the output
        */
      inline void
      setMPrime (size_t m_prime) { M_prime_ = m_prime; }

      /** \brief Returns the M' parameter - the number of rows to be taken from the matrix of DFT and DCT
        * values that will be contained in the output feature vector
        * \note This value directly influences the dimensions of the type of output points (PointFeature)
        */
      inline size_t
      getMPrime () { return M_prime_; }

      /** \brief Setter method for the scale parameter - used to determine the radius of the sampling disc around the
        * point of interest - linked to the smoothing scale of the input cloud
        */
      inline void
      setScale (float scale) { scale_h_ = scale; }

      /** \brief Returns the scale parameter - used to determine the radius of the sampling disc around the
        * point of interest - linked to the smoothing scale of the input cloud
        */
      inline float
      getScale () { return scale_h_; }


    protected:
      void
      computeFeature (FeatureCloud &output);

    private:
      float scale_h_;
      size_t N_, M_, N_prime_, M_prime_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/normal_based_signature.hpp>
#endif

#endif /* PCL_NORMAL_BASED_SIGNATURE_H_ */

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#ifndef PCL_DENSE_CRF_H_
#define PCL_DENSE_CRF_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ml/pairwise_potential.h>

namespace pcl
{
  /** \brief
   * 
   */
  class PCL_EXPORTS DenseCrf
  {
    public:

      /** \brief Constructor for DenseCrf class */
      DenseCrf (int N, int m);

      /** \brief Deconstructor for DenseCrf class */
      ~DenseCrf ();
      
      /** \brief set the input data vector.
       * The input data vector holds the measurements
       * coordinates as ijk of the voxel grid
       */
      void
      setDataVector (const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > data);

      /** \brief The associated color of the data
       */
      void
      setColorVector (const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > color);

      void
      setUnaryEnergy (const std::vector<float> unary);
 
      /** \brief      */
      void
      addPairwiseEnergy (const std::vector<float> &feature, const int feature_dimension, const float w);
      
      
      /** \brief Add a pairwise gaussian kernel
       * 
       */
      void
      addPairwiseGaussian (float sx, float sy, float sz, float w);
      
      /** \brief Add a bilateral gaussian kernel
       * 
       */
      void
      addPairwiseBilateral (float sx, float sy, float sz, 
                            float sr, float sg, float sb,
                            float w);


      void
      addPairwiseNormals (std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > &coord,
                          std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &normals,
                          float sx, float sy, float sz, 
                          float snx, float sny, float snz,
                          float w);
      

      void
      inference (int n_iterations, std::vector<float> &result, float relax = 1.0f);
 
      void
      mapInference (int n_iterations, std::vector<int> &result, float relax = 1.0f);
      
      void
      expAndNormalize (std::vector<float> &out, const std::vector<float> &in,
                       float scale, float relax = 1.0f);
 
      void
      expAndNormalizeORI ( float* out, const float* in, float scale=1.0f, float relax=1.0f );
      void map ( int n_iterations, std::vector<int> result, float relax=1.0f );
      std::vector<float> runInference( int n_iterations, float relax );
      void startInference();
      void stepInference( float relax );
      

      void
      runInference (float relax);


      void
      getBarycentric (int idx, std::vector<float> &bary);

      void
      getFeatures (int idx, std::vector<float> &features);
      


    protected:

      /** \brief Number of variables and labels */
      int N_, M_;

      /** \brief Data vector */
      std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > data_;

      /** \brief Color vector */
      std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > color_;

      /** TODO: double might use to much memory */
      /** \brief CRF unary potentials */
      std::vector<float> unary_;

      std::vector<float> current_;
      std::vector<float> next_;
      std::vector<float> tmp_;

      /** \brief pairwise potentials */
      std::vector<PairwisePotential*> pairwise_potential_;
          
      /** \brief input types */
      bool xyz_, rgb_, normal_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}





#endif

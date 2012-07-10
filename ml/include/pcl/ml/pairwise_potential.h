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

#ifndef PCL_PAIRWISE_POTENTIAL_H_
#define PCL_PAIRWISE_POTENTIAL_H_

#include <vector>

#include <pcl/ml/permutohedral.h>

namespace pcl
{
  /** \brief
   * 
   */
  class PairwisePotential
  {
    public:

      /** \brief Constructor for DenseCrf class */
      PairwisePotential (const std::vector<float> &feature, const int D, const int N, const float w);

      /** \brief Deconstructor for DenseCrf class */
      ~PairwisePotential () {};

      /** \brief  */
      void
      compute (std::vector<float> &out, const std::vector<float> &in,
               std::vector<float> &tmp, int value_size) const;

    protected:
      /** \brief Permutohedral lattice */
      Permutohedral lattice_;

      /** \brief Number of variables */
      int N_;

      /** \brief weight */
      float w_;

      /** \brief norm */
      std::vector<float> norm_;

      //DBUG
    public:
      std::vector<float> bary_;
      std::vector<float> features_;

  };
}

#endif

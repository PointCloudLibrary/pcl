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

#include <pcl/ml/pairwise_potential.h>

pcl::PairwisePotential::PairwisePotential(const std::vector<float>& feature,
                                          const int feature_dimension,
                                          const int N,
                                          const float w)
: N_(N), w_(w)
{
  // lattice_.init (feature, feature_dimension, N);
  // std::cout << "0---------" << std::endl;
  lattice_.init(feature, feature_dimension, N);

  // std::cout << "1---------" << std::endl;

  // lattice_.debug ();

  norm_.resize(N);
  for (int i = 0; i < N; i++)
    norm_[i] = 1;

  // Compute the normalization factor

  // lattice_.compute (norm_, norm_, 1);

  /*
  std::vector<float> normOLD;
  normOLD.resize (N);
  for (int i = 0; i < N; i++)
    normOLD[i] = 1;
  */

  // std::cout << "2---------" << std::endl;

  lattice_.compute(norm_, norm_, 1);

  // std::cout << "3---------" << std::endl;

  /*
    ///////////
    // DEBUG //
    bool same = true;
    for (std::size_t i = 0; i < normOLD.size (); i++)
    {
      if (norm_[i] != normOLD[i])
        same = false;
    }
    if (same)
      std::cout << "DEBUG norm -  OK" << std::endl;
    else
      std::cout << "DEBUG norm - ERROR" << std::endl;
  */

  // per pixel normalization
  for (int i = 0; i < N; i++)
    norm_[i] = 1.0f / (norm_[i] + 1e-20f);

  // std::cout << "4---------" << std::endl;

  bary_ = lattice_.barycentric_;

  // std::cout << "5---------" << std::endl;

  features_ = feature;

  // std::cout << "6---------" << std::endl;

  /*
    std::cout << "bary size: " << bary_.size () << std::endl;
    for (int g = 0; g < 25; g++)
      std::cout << bary_[g] << std::endl;
  */
}

void
pcl::PairwisePotential::compute(std::vector<float>& out,
                                const std::vector<float>& in,
                                std::vector<float>& tmp,
                                int value_size) const
{
  lattice_.compute(tmp, in, value_size);
  for (int i = 0, k = 0; i < N_; i++)
    for (int j = 0; j < value_size; j++, k++)
      out[k] += w_ * norm_[i] * tmp[k];
}

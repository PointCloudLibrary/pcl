/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */

#pragma once

#include <random>

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /**
   * \brief ShapeContext3DEstimation implements the 3D shape context descriptor
   *        based on Frome et al. (ECCV 2004).
   *
   * Notes:
   * - The suggested PointOutT is pcl::ShapeContext1980 (fixed-length descriptor).
   * - PCL requires one descriptor per input point and descriptors to be a fixed length.
   * - The original paper suggests multiple azimuth rotations (L rotations). That approach
   *   would expand descriptor length to descriptor_length_ * azimuth_bins_, which breaks
   *   PCL assumptions (fixed size and one-to-one mapping). Therefore the implementation
   *   below uses a deterministic azimuth normalization (shift) that keeps the descriptor
   *   length unchanged while removing the randomness introduced by the random local X axis.
   */
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::ShapeContext1980>
  class ShapeContext3DEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<ShapeContext3DEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const ShapeContext3DEstimation<PointInT, PointNT, PointOutT> >;

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::searchForNeighbors;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;
      using PointCloudIn = typename Feature<PointInT, PointOutT>::PointCloudIn;

      /** \brief Constructor.
        * \param[in] random If true the random seed is set to current time, else it is
        * set to 12345 prior to computing the descriptor (used to select X axis).
        */
      ShapeContext3DEstimation (bool random = false) :
        radii_interval_(0),
        theta_divisions_(0),
        phi_divisions_(0),
        volume_lut_(0),
        rng_dist_ (0.0f, 1.0f)
      {
        feature_name_ = "ShapeContext3DEstimation";
        search_radius_ = 2.5;

        // Initialize RNG: deterministic by default (helps reproducible testing)
        if (random)
        {
          std::random_device rd;
          rng_.seed (rd());
        }
        else
          rng_.seed (12345u);
      }

      ~ShapeContext3DEstimation() override = default;

      /** \return the number of bins along the azimuth */
      inline std::size_t getAzimuthBins () { return (azimuth_bins_); }

      /** \return The number of bins along the elevation */
      inline std::size_t getElevationBins () { return (elevation_bins_); }

      /** \return The number of bins along the radii direction */
      inline std::size_t getRadiusBins () { return (radius_bins_); }

      inline void setMinimalRadius (double radius) { min_radius_ = radius; }
      inline double getMinimalRadius () { return (min_radius_); }

      inline void setPointDensityRadius (double radius) { point_density_radius_ = radius; }
      inline double getPointDensityRadius () { return (point_density_radius_); }

    protected:
      /** Initialize internal tables (radii intervals, angular divisions, volume LUT). */
      bool initCompute () override;

      /**
       * Estimate a descriptor for a given point.
       * - index: point index to estimate descriptor for
       * - normals: normals used (precomputed)
       * - rf: output reference frame (3x3 stored in rf[9]); 3DSC does not define repeatable RF so we set rf=0
       * - desc: output descriptor (must be descriptor_length_)
       */
      bool computePoint (std::size_t index, const pcl::PointCloud<PointNT> &normals, float rf[9], std::vector<float> &desc);

      /** Compute feature for all indices (fills output cloud). */
      void computeFeature (PointCloudOut &output) override;

      /* Lookup / intermediate data */
      std::vector<float> radii_interval_;
      std::vector<float> theta_divisions_;
      std::vector<float> phi_divisions_;
      std::vector<float> volume_lut_;

      /* Histogram bin configuration (defaults chosen to match ShapeContext1980) */
      std::size_t azimuth_bins_{12};
      std::size_t elevation_bins_{11};
      std::size_t radius_bins_{15};

      /* Parameters */
      double min_radius_{0.1};
      double point_density_radius_{0.2};
      std::size_t descriptor_length_{};

      /* RNG for random local x-axis selection */
      std::mt19937 rng_;
      std::uniform_real_distribution<float> rng_dist_;

      /* Old (commented) API for L-rotation approach left in file for history; we DO NOT use it.
       * //void shiftAlongAzimuth (std::size_t block_size, std::vector<float>& desc);
       */

      /**
       * Our deterministic azimuth normalization:
       * Rotate azimuth bins so that the azimuth block with the largest accumulated
       * energy becomes the first block (index 0). This removes randomness introduced
       * by the local random X axis selection while preserving a fixed descriptor length.
       *
       * - Input: desc (size == descriptor_length_)
       * - Output: desc is circularly shifted in-place so that the dominant azimuth block
       *   is aligned to the start of the descriptor array.
       */
      void shiftAlongAzimuth (std::vector<float>& desc) const;

      /** Return a random float in [0,1) using the internal RNG. */
      inline float rnd () { return (rng_dist_ (rng_)); }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/3dsc.hpp>
#endif
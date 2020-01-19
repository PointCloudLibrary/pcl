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

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief UniqueShapeContext implements the Unique Shape Context Descriptor
    * described here:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano,
    *     "Unique Shape Context for 3D data description",
    *     International Workshop on 3D Object Retrieval (3DOR 10) -
    *     in conjunction with ACM Multimedia 2010
    *
    * The suggested PointOutT is pcl::UniqueShapeContext1960
    *
    * \author Alessandro Franchi, Federico Tombari, Samuele Salti (original code)
    * \author Nizar Sallem (port to PCL)
    * \ingroup features
    */
  template <typename PointInT, typename PointOutT = pcl::UniqueShapeContext1960, typename PointRFT = pcl::ReferenceFrame>
  class UniqueShapeContext : public Feature<PointInT, PointOutT>,
                             public FeatureWithLocalReferenceFrames<PointInT, PointRFT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::fake_surface_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::searchForNeighbors;
      using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::frames_;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;
      using PointCloudIn = typename Feature<PointInT, PointOutT>::PointCloudIn;
      using Ptr = shared_ptr<UniqueShapeContext<PointInT, PointOutT, PointRFT> >;
      using ConstPtr = shared_ptr<const UniqueShapeContext<PointInT, PointOutT, PointRFT> >;


      /** \brief Constructor. */
      UniqueShapeContext () :
        radii_interval_(0), theta_divisions_(0), phi_divisions_(0), volume_lut_(0),
        azimuth_bins_(14), elevation_bins_(14), radius_bins_(10),
        min_radius_(0.1), point_density_radius_(0.1), descriptor_length_ (), local_radius_ (2.0)
      {
        feature_name_ = "UniqueShapeContext";
        search_radius_ = 2.0;
      }

      ~UniqueShapeContext() { }

      /** \return The number of bins along the azimuth. */
      inline std::size_t
      getAzimuthBins () const { return (azimuth_bins_); }

      /** \return The number of bins along the elevation */
      inline std::size_t
      getElevationBins () const { return (elevation_bins_); }

      /** \return The number of bins along the radii direction. */
      inline std::size_t
      getRadiusBins () const { return (radius_bins_); }

      /** The minimal radius value for the search sphere (rmin) in the original paper
        * \param[in] radius the desired minimal radius
        */
      inline void
      setMinimalRadius (double radius) { min_radius_ = radius; }

      /** \return The minimal sphere radius. */
      inline double
      getMinimalRadius () const { return (min_radius_); }

      /** This radius is used to compute local point density
        * density = number of points within this radius
        * \param[in] radius Value of the point density search radius
        */
      inline void
      setPointDensityRadius (double radius) { point_density_radius_ = radius; }

      /** \return The point density search radius. */
      inline double
      getPointDensityRadius () const { return (point_density_radius_); }

      /** Set the local RF radius value
        * \param[in] radius the desired local RF radius
        */
      inline void
      setLocalRadius (double radius) { local_radius_ = radius; }

      /** \return The local RF radius. */
      inline double
      getLocalRadius () const { return (local_radius_); }

    protected:
      /** Compute 3D shape context feature descriptor
        * \param[in] index point index in input_
        * \param[out] desc descriptor to compute
        */
      void
      computePointDescriptor (std::size_t index, std::vector<float> &desc);

      /** \brief Initialize computation by allocating all the intervals and the volume lookup table. */
      bool
      initCompute () override;

      /** \brief The actual feature computation.
        * \param[out] output the resultant features
        */
      void
      computeFeature (PointCloudOut &output) override;

      /** \brief values of the radii interval. */
      std::vector<float> radii_interval_;

      /** \brief Theta divisions interval. */
      std::vector<float> theta_divisions_;

      /** \brief Phi divisions interval. */
      std::vector<float> phi_divisions_;

      /** \brief Volumes look up table. */
      std::vector<float> volume_lut_;

      /** \brief Bins along the azimuth dimension. */
      std::size_t azimuth_bins_;

      /** \brief Bins along the elevation dimension. */
      std::size_t elevation_bins_;

      /** \brief Bins along the radius dimension. */
      std::size_t radius_bins_;

      /** \brief Minimal radius value. */
      double min_radius_;

      /** \brief Point density radius. */
      double point_density_radius_;

      /** \brief Descriptor length. */
      std::size_t descriptor_length_;

      /** \brief Radius to compute local RF. */
      double local_radius_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/usc.hpp>
#endif

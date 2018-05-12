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

#ifndef PCL_FEATURES_3DSC_H_
#define PCL_FEATURES_3DSC_H_

#include <pcl/point_types.h>
#include <pcl/features/boost.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief ShapeContext3DEstimation implements the 3D shape context descriptor as
    * described in:
    *   - Andrea Frome, Daniel Huber, Ravi Kolluri and Thomas Bülow, Jitendra Malik
    *     Recognizing Objects in Range Data Using Regional Point Descriptors,
    *     In proceedings of the 8th European Conference on Computer Vision (ECCV),
    *     Prague, May 11-14, 2004
    *
    * The suggested PointOutT is pcl::ShapeContext1980
    *
    * \attention
    * The convention for a 3D shape context descriptor is:
    *   - if a query point's nearest neighbors cannot be estimated, the feature descriptor will be set to NaN (not a number), and the RF to 0
    *   - it is impossible to estimate a 3D shape context descriptor for a
    *     point that doesn't have finite 3D coordinates. Therefore, any point
    *     that contains NaN data on x, y, or z, will have its boundary feature
    *     property set to NaN.
    *
    * \author Alessandro Franchi, Samuele Salti, Federico Tombari (original code)
    * \author Nizar Sallem (port to PCL)
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::ShapeContext1980>
  class ShapeContext3DEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<ShapeContext3DEstimation<PointInT, PointNT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const ShapeContext3DEstimation<PointInT, PointNT, PointOutT> > ConstPtr;

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::searchForNeighbors;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

      /** \brief Constructor.
        * \param[in] random If true the random seed is set to current time, else it is
        * set to 12345 prior to computing the descriptor (used to select X axis)
        */
      ShapeContext3DEstimation (bool random = false) :
        radii_interval_(0),
        theta_divisions_(0),
        phi_divisions_(0),
        volume_lut_(0),
        azimuth_bins_(12),
        elevation_bins_(11),
        radius_bins_(15),
        min_radius_(0.1),
        point_density_radius_(0.2),
        descriptor_length_ (),
        rng_alg_ (),
        rng_ (new boost::uniform_01<boost::mt19937> (rng_alg_))
      {
        feature_name_ = "ShapeContext3DEstimation";
        search_radius_ = 2.5;

        // Create a random number generator object
        if (random)
          rng_->base ().seed (static_cast<unsigned> (std::time(0)));
        else
          rng_->base ().seed (12345u);
      }

      virtual ~ShapeContext3DEstimation() {}

      //inline void
      //setAzimuthBins (size_t bins) { azimuth_bins_ = bins; }

      /** \return the number of bins along the azimuth */
      inline size_t
      getAzimuthBins () { return (azimuth_bins_); }

      //inline void
      //setElevationBins (size_t bins) { elevation_bins_ = bins; }

      /** \return The number of bins along the elevation */
      inline size_t
      getElevationBins () { return (elevation_bins_); }

      //inline void
      //setRadiusBins (size_t bins) { radius_bins_ = bins; }

      /** \return The number of bins along the radii direction */
      inline size_t
      getRadiusBins () { return (radius_bins_); }

      /** \brief The minimal radius value for the search sphere (rmin) in the original paper
        * \param[in] radius the desired minimal radius
        */
      inline void
      setMinimalRadius (double radius) { min_radius_ = radius; }

      /** \return The minimal sphere radius */
      inline double
      getMinimalRadius () { return (min_radius_); }

      /** \brief This radius is used to compute local point density
        * density = number of points within this radius
        * \param[in] radius value of the point density search radius
        */
      inline void
      setPointDensityRadius (double radius) { point_density_radius_ = radius; }

      /** \return The point density search radius */
      inline double
      getPointDensityRadius () { return (point_density_radius_); }

    protected:
      /** \brief Initialize computation by allocating all the intervals and the volume lookup table. */
      bool
      initCompute ();

      /** \brief Estimate a descriptor for a given point.
        * \param[in] index the index of the point to estimate a descriptor for
        * \param[in] normals a pointer to the set of normals
        * \param[in] rf the reference frame
        * \param[out] desc the resultant estimated descriptor
        * \return true if the descriptor was computed successfully, false if there was an error
        * (e.g. the nearest neighbor didn't return any neighbors)
        */
      bool
      computePoint (size_t index, const pcl::PointCloud<PointNT> &normals, float rf[9], std::vector<float> &desc);

      /** \brief Estimate the actual feature.
        * \param[out] output the resultant feature
        */
      void
      computeFeature (PointCloudOut &output);

      /** \brief Values of the radii interval */
      std::vector<float> radii_interval_;

      /** \brief Theta divisions interval */
      std::vector<float> theta_divisions_;

      /** \brief Phi divisions interval */
      std::vector<float> phi_divisions_;

      /** \brief Volumes look up table */
      std::vector<float> volume_lut_;

      /** \brief Bins along the azimuth dimension */
      size_t azimuth_bins_;

      /** \brief Bins along the elevation dimension */
      size_t elevation_bins_;

      /** \brief Bins along the radius dimension */
      size_t radius_bins_;

      /** \brief Minimal radius value */
      double min_radius_;

      /** \brief Point density radius */
      double point_density_radius_;

      /** \brief Descriptor length */
      size_t descriptor_length_;

      /** \brief Boost-based random number generator algorithm. */
      boost::mt19937 rng_alg_;

      /** \brief Boost-based random number generator distribution. */
      boost::shared_ptr<boost::uniform_01<boost::mt19937> > rng_;

     /*  \brief Shift computed descriptor "L" times along the azimuthal direction
       * \param[in] block_size the size of each azimuthal block
       * \param[in] desc at input desc == original descriptor and on output it contains
       *  shifted descriptor resized descriptor_length_ * azimuth_bins_
       */
      //void
      //shiftAlongAzimuth (size_t block_size, std::vector<float>& desc);

      /** \brief Boost-based random number generator. */
      inline double
      rnd ()
      {
        return ((*rng_) ());
      }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/3dsc.hpp>
#endif

#endif  //#ifndef PCL_3DSC_H_

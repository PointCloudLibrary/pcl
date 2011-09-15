/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, www.pointclouds.org
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
 *  $Id$
 *
 */

#ifndef PCL_3DSC_H_
#define PCL_3DSC_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /* namespace features */
  /* { */
    /** Class ShapeContext3DEstimation implements the 3D shape context descriptor as
      * described here
      * <ul>
      * <li> Andrea Frome, Daniel Huber, Ravi Kolluri and Thomas Bülow, Jitendra Malik
      *      Recognizing Objects in Range Data Using Regional Point Descriptors,
      *      In proceedings of the 8th European Conference on Computer Vision (ECCV),
      *      Prague, May 11-14, 2004
      * </li>
      * </ul>
      * The 3DSC computed feature has the following structure
      * <ul>
      * <li> rf float[9] = x_axis | y_axis | normal and represents the local frame </li>
      * <li> desc std::vector<float> which size is determined by the number of bins
      * radius_bins_ + elevation_bins_ + azimuth_bins_ </li>
      * </ul>
      * \author Federico (original code)
      * \author Nizar Sallem (port to PCL)
      * \ingroup features
      */
    template <typename PointInT, typename PointNT, typename PointOutT> 
    class ShapeContext3DEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
    {
      public:
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
         
         /** Constructor
           * \param random If true the random seed is set to cuurent time else it is set
           * to 12345. The randomness is used to select X axis 
           */
         ShapeContext3DEstimation(bool random = false) :
           radii_interval_(0), theta_divisions_(0), phi_divisions_(0), volume_lut_(0),
           azimuth_bins_(12), elevation_bins_(11), radius_bins_(15), 
           min_radius_(0.1), point_density_radius_(0.2)
         {
           feature_name_ = "ShapeContext3DEstimation";
           search_radius_ = 2.5;
           if(random)
             srand(time(NULL));
           else
             srand(12345); 
         }

        ~ShapeContext3DEstimation() {}

        /** set number of bins along the azimth to \param bins */
        inline void setAzimuthBins(size_t bins) { azimuth_bins_ = bins; }
        /** \return the number of bins along the azimuth */
        inline size_t getAzimuthBins(size_t bins) { return (azimuth_bins_); } 
        /** set number of bins along the elevation to \param bins */
        inline void setElevationBins(size_t bins) { elevation_bins_ = bins; }
        /** \return the number of bins along the elevation */
        inline size_t getElevationBins(size_t bins) { return (elevation_bins_); } 
        /** set number of bins along the radii to \param bins */
        inline void setRadiusBins(size_t bins) { radius_bins_ = bins; }
        /** \return the number of bins along the radii direction */
        inline size_t getRadiusBins(size_t bins) { return (radius_bins_); } 
        /** The minimal radius value for the search sphere (rmin) in the original paper 
          * \param radius the desired minimal radius
          */
        inline void setMinimalRadius(float radius) { min_radius_ = radius; }
        /** \return the minimal sphere radius */
        inline float getMinimalRadius() { return (min_radius_); }
        /** This radius is used to compute local point density 
          * density = number of points within this radius
          * \param radius Value of the point density search radius
          */
        inline void setPointDensityRadius(double radius) { point_density_radius_ = radius; }
        /** \return point density search radius */
        inline double getPointDensityRadius() { return (point_density_radius_); }
        
      protected:
        /** initilize computation by allocating all the intervals and the volume look up 
          * table
          */
        bool initCompute() ;

        void
        computePoint(size_t index, const pcl::PointCloud<PointInT> &input, const pcl::PointCloud<PointNT> &normals, float rf[9], std::vector<float> &desc);

        void
        computeFeature(PointCloudOut &output);

      protected:
        /** values of the radii interval */
        std::vector<float> radii_interval_;
        /** theta divisions interval */
        std::vector<float> theta_divisions_;
        /** phi divisions interval */
        std::vector<float> phi_divisions_;
        /** volumes look up table */
        std::vector<float> volume_lut_;

      private:
        /** bins along the azimuth dimension */
        size_t azimuth_bins_;
        /** bins along the elevation dimension */
        size_t elevation_bins_;
        /** bins along the radius dimension */
        size_t radius_bins_;
        /** minimal radius value */
        double min_radius_;
        /** point density radius */
        double point_density_radius_;
        /** descriptor length */
        size_t descriptor_length_;
    };
  /* }; */
}

#endif  //#ifndef PCL_3DSC_H_

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
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  // FORWARD DECLARATIONS:
  class RangeImage;
  template <typename PointType>
  class PointCloud;

  /** \brief @b Extract obstacle borders from range images, meaning positions where there is a transition from foreground
    * to background.
    * \author Bastian Steder
    * \ingroup features
    */
  class PCL_EXPORTS RangeImageBorderExtractor : public Feature<PointWithRange,BorderDescription>
  {
    public:
      using Ptr = shared_ptr<RangeImageBorderExtractor>;
      using ConstPtr = shared_ptr<const RangeImageBorderExtractor>;
      // =====TYPEDEFS=====
      using BaseClass = Feature<PointWithRange,BorderDescription>;
      
      // =====PUBLIC STRUCTS=====
      //! Stores some information extracted from the neighborhood of a point
      struct LocalSurface
      {
        LocalSurface () : 
           max_neighbor_distance_squared () {}

        Eigen::Vector3f normal;
        Eigen::Vector3f neighborhood_mean;
        Eigen::Vector3f eigen_values;
        Eigen::Vector3f normal_no_jumps;
        Eigen::Vector3f neighborhood_mean_no_jumps;
        Eigen::Vector3f eigen_values_no_jumps;
        float max_neighbor_distance_squared;
      };
      
      //! Stores the indices of the shadow border corresponding to obstacle borders
      struct ShadowBorderIndices 
      {
        ShadowBorderIndices () : left (-1), right (-1), top (-1), bottom (-1) {}
        int left, right, top, bottom;
      };

      //! Parameters used in this class
      struct Parameters
      {
        Parameters () : max_no_of_threads(1), pixel_radius_borders (3), pixel_radius_plane_extraction (2), pixel_radius_border_direction (2), 
                       minimum_border_probability (0.8f), pixel_radius_principal_curvature (2) {}
        int max_no_of_threads;
        int pixel_radius_borders;
        int pixel_radius_plane_extraction;
        int pixel_radius_border_direction;
        float minimum_border_probability;
        int pixel_radius_principal_curvature;
      };
      
      // =====STATIC METHODS=====
      /** \brief Take the information from BorderTraits to calculate the local direction of the border
       * \param border_traits contains the information needed to calculate the border angle
       */
      static inline float
      getObstacleBorderAngle (const BorderTraits& border_traits);
      
      // =====CONSTRUCTOR & DESTRUCTOR=====
      /** Constructor */
      RangeImageBorderExtractor (const RangeImage* range_image=nullptr);
      /** Destructor */
      ~RangeImageBorderExtractor ();
      
      // =====METHODS=====
      /** \brief Provide a pointer to the range image
        * \param range_image a pointer to the range_image
        */
      void
      setRangeImage (const RangeImage* range_image);
      
      /** \brief Erase all data calculated for the current range image */
      void
      clearData ();
       
      /** \brief Get the 2D directions in the range image from the border directions - probably mainly useful for 
        * visualization 
        */
      float*
      getAnglesImageForBorderDirections ();

      /** \brief Get the 2D directions in the range image from the surface change directions - probably mainly useful for 
        * visualization 
        */
      float*
      getAnglesImageForSurfaceChangeDirections ();
      
      /** Overwrite the compute function of the base class */
      void
      compute (PointCloudOut& output);
      
      // =====GETTER=====
      Parameters&
      getParameters () { return (parameters_); }

      bool
      hasRangeImage () const { return range_image_ != nullptr; }

      const RangeImage&
      getRangeImage () const { return *range_image_; }

      float*
      getBorderScoresLeft ()   { extractBorderScoreImages (); return border_scores_left_.data (); }

      float*
      getBorderScoresRight ()  { extractBorderScoreImages (); return border_scores_right_.data (); }

      float*
      getBorderScoresTop ()    { extractBorderScoreImages (); return border_scores_top_.data (); }

      float*
      getBorderScoresBottom () { extractBorderScoreImages (); return border_scores_bottom_.data (); }

      LocalSurface**
      getSurfaceStructure () { extractLocalSurfaceStructure (); return surface_structure_; }

      PointCloudOut&
      getBorderDescriptions () { classifyBorders (); return *border_descriptions_; }

      ShadowBorderIndices**
      getShadowBorderInformations () { findAndEvaluateShadowBorders (); return shadow_border_informations_; }

      Eigen::Vector3f**
      getBorderDirections () { calculateBorderDirections (); return border_directions_; }

      float*
      getSurfaceChangeScores () { calculateSurfaceChanges (); return surface_change_scores_; }

      Eigen::Vector3f*
      getSurfaceChangeDirections () { calculateSurfaceChanges (); return surface_change_directions_; }
      
      
    protected:
      // =====PROTECTED MEMBER VARIABLES=====
      Parameters parameters_;
      const RangeImage* range_image_;
      int range_image_size_during_extraction_;
      std::vector<float> border_scores_left_, border_scores_right_;
      std::vector<float> border_scores_top_, border_scores_bottom_;
      LocalSurface** surface_structure_;
      PointCloudOut* border_descriptions_;
      ShadowBorderIndices** shadow_border_informations_;
      Eigen::Vector3f** border_directions_;
      
      float* surface_change_scores_;
      Eigen::Vector3f* surface_change_directions_;
      
      
      // =====PROTECTED METHODS=====
      /** \brief Calculate a border score based on how distant the neighbor is, compared to the closest neighbors
       * /param local_surface
       * /param x
       * /param y
       * /param offset_x
       * /param offset_y
       * /param pixel_radius (defaults to 1)
       * /return the resulting border score
       */
      inline float
      getNeighborDistanceChangeScore (const LocalSurface& local_surface, int x, int y, 
                                      int offset_x, int offset_y, int pixel_radius=1) const;
      
      /** \brief Calculate a border score based on how much the neighbor is away from the local surface plane
        * \param local_surface
        * \param x
        * \param y
        * \param offset_x
        * \param offset_y
        * \return the resulting border score
        */
      inline float
      getNormalBasedBorderScore (const LocalSurface& local_surface, int x, int y, 
                                 int offset_x, int offset_y) const;
      
      /** \brief Find the best corresponding shadow border and lower score according to the shadow borders value
        * \param x
        * \param y
        * \param offset_x
        * \param offset_y
        * \param border_scores
        * \param border_scores_other_direction
        * \param shadow_border_idx
        * \return
        */
      inline bool
      changeScoreAccordingToShadowBorderValue (int x, int y, int offset_x, int offset_y, float* border_scores,
                                               float* border_scores_other_direction, int& shadow_border_idx) const;
      
      /** \brief Returns a new score for the given pixel that is >= the original value, based on the neighbors values
        * \param x the x-coordinate of the input pixel
        * \param y the y-coordinate of the input pixel
        * \param border_scores the input border scores
        * \return the resulting updated border score
        */
      inline float
      updatedScoreAccordingToNeighborValues (int x, int y, const float* border_scores) const;

      /** \brief For all pixels, returns a new score that is >= the original value, based on the neighbors values
        * \param border_scores the input border scores
        * \return a pointer to the resulting array of updated scores
        */
      float*
      updatedScoresAccordingToNeighborValues (const float* border_scores) const;

      /** \brief Replace all border score values with updates according to \a updatedScoreAccordingToNeighborValues */
      void
      updateScoresAccordingToNeighborValues ();
      
      /** \brief Check if a potential border point has a corresponding shadow border
        * \param x the x-coordinate of the input point
        * \param y the y-coordinate of the input point
        * \param offset_x
        * \param offset_y
        * \param border_scores_left
        * \param border_scores_right
        * \param shadow_border_idx
        * \return a boolean value indicating whether or not the point has a corresponding shadow border
       */
      inline bool
      checkPotentialBorder (int x, int y, int offset_x, int offset_y, float* border_scores_left,
                            float* border_scores_right, int& shadow_border_idx) const;

      /** \brief Check if a potential border point is a maximum regarding the border score
        * \param x the x-coordinate of the input point
        * \param y the y-coordinate of the input point
        * \param offset_x
        * \param offset_y
        * \param border_scores
        * \param shadow_border_idx
        * \result a boolean value indicating whether or not the point is a maximum
        */
      inline bool
      checkIfMaximum (int x, int y, int offset_x, int offset_y, float* border_scores, int shadow_border_idx) const;
      
      /** \brief Find the best corresponding shadow border and lower score according to the shadow borders value */
      void
      findAndEvaluateShadowBorders ();
      
      /** \brief Extract local plane information in every point (see getSurfaceStructure ()) */
      void
      extractLocalSurfaceStructure ();
      
      /** \brief Get images representing the probability that the corresponding pixels are borders in that direction 
        * (see getBorderScores... ())
        */
      void
      extractBorderScoreImages ();
      
      /** \brief Classify the pixels in the range image according to the different classes defined below in 
        * enum BorderClass. minImpactAngle (in radians) defines how flat the angle at which a surface was seen can be. 
        */
      void
      classifyBorders ();
      
      /** \brief Calculate the 3D direction of the border just using the border traits at this position (facing away from 
        * the obstacle)
        * \param x the x-coordinate of the input position
        * \param y the y-coordinate of the input position
        */
      inline void
      calculateBorderDirection (int x, int y);
      
      /** \brief Call \a calculateBorderDirection for every point and average the result over 
        * parameters_.pixel_radius_border_direction
        */
      void
      calculateBorderDirections ();
      
      /** \brief Calculate a 3d direction from a border point by projecting the direction in the range image - returns 
        * false if direction could not be calculated
        * \param border_description
        * \param direction
        * \param local_surface
        * \return a boolean value indicating whether or not a direction could be calculated
        */
      inline bool
      get3dDirection (const BorderDescription& border_description, Eigen::Vector3f& direction,
                      const LocalSurface* local_surface=nullptr);
      
      /** \brief Calculate the main principal curvature (the largest eigenvalue and corresponding eigenvector for the 
        * normals in the area) in the given point
        * \param x the x-coordinate of the input point
        * \param y the y-coordinate of the input point
        * \param radius the pixel radius that is used to find neighboring points
        * \param magnitude the resulting magnitude
        * \param main_direction the resulting direction
        */
      inline bool
      calculateMainPrincipalCurvature (int x, int y, int radius, float& magnitude,
                                       Eigen::Vector3f& main_direction) const;
      
      /** \brief Uses either the border or principal curvature to define a score how much the surface changes in a point 
          (1 for a border) and what the main direction of that change is */
      void
      calculateSurfaceChanges ();

      /** \brief Apply a blur to the surface change images */
      void
      blurSurfaceChanges ();
      
      /** \brief Implementation of abstract derived function */
      void
      computeFeature (PointCloudOut &output) override;

    private:
      std::vector<float>
      updatedScoresAccordingToNeighborValues (const std::vector<float>& border_scores) const;
  };
}  // namespace end

#include <pcl/features/impl/range_image_border_extractor.hpp>  // Definitions of templated and inline functions

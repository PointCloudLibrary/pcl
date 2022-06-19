/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief BOARDLocalReferenceFrameEstimation implements the BOrder Aware Repeatable Directions algorithm
    * for local reference frame estimation as described here:
    *
    *  - A. Petrelli, L. Di Stefano,
    *    "On the repeatability of the local reference frame for partial shape matching",
    *    13th International Conference on Computer Vision (ICCV), 2011
    *
    * \author Alioscia Petrelli (original), Tommaso Cavallari (PCL port)
    * \ingroup features
    */
  template<typename PointInT, typename PointNT, typename PointOutT = ReferenceFrame>
  class BOARDLocalReferenceFrameEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const BOARDLocalReferenceFrameEstimation<PointInT, PointNT, PointOutT> >;

      /** \brief Constructor. */
      BOARDLocalReferenceFrameEstimation () :
        tangent_radius_ (0.0f),
        find_holes_ (false),
        margin_thresh_ (0.85f),
        check_margin_array_size_ (24),
        hole_size_prob_thresh_ (0.2f),
        steep_thresh_ (0.1f)
      {
        feature_name_ = "BOARDLocalReferenceFrameEstimation";
        setCheckMarginArraySize (check_margin_array_size_);
      }
      
      /** \brief Empty destructor */
      ~BOARDLocalReferenceFrameEstimation () override = default;

      //Getters/Setters

      /** \brief Set the maximum distance of the points used to estimate the x_axis and y_axis of the BOARD Reference Frame for a given point.
        *
        * \param[in] radius The search radius for x and y axes. If not set or set to 0 the parameter given with setRadiusSearch is used.
        */
      inline void
      setTangentRadius (float radius)
      {
        tangent_radius_ = radius;
      }

      /** \brief Get the maximum distance of the points used to estimate the x_axis and y_axis of the BOARD Reference Frame for a given point.
        *
        * \return The search radius for x and y axes. If set to 0 the parameter given with setRadiusSearch is used.
        */
      inline float
      getTangentRadius () const
      {
        return (tangent_radius_);
      }

      /** \brief Sets whether holes in the margin of the support, for each point, are searched and accounted for in the estimation of the 
        *          Reference Frame or not.
        *
        * \param[in] find_holes Enable/Disable the search for holes in the support.
        */
      inline void
      setFindHoles (bool find_holes)
      {
        find_holes_ = find_holes;
      }

      /** \brief Gets whether holes in the margin of the support, for each point, are searched and accounted for in the estimation of the 
        *          Reference Frame or not.
        *
        * \return The search for holes in the support is enabled/disabled.
        */
      inline bool
      getFindHoles () const
      {
        return (find_holes_);
      }

      /** \brief Sets the percentage of the search radius (or tangent radius if set) after which a point is considered part of the support margin.
        *
        * \param[in] margin_thresh the percentage of the search radius after which a point is considered a margin point.
        */
      inline void
      setMarginThresh (float margin_thresh)
      {
        margin_thresh_ = margin_thresh;
      }

      /** \brief Gets the percentage of the search radius (or tangent radius if set) after which a point is considered part of the support margin.
        *
        * \return The percentage of the search radius after which a point is considered a margin point.
        */
      inline float
      getMarginThresh () const
      {
        return (margin_thresh_);
      }

      /** \brief Sets the number of slices in which is divided the margin for the search of missing regions.
        *
        * \param[in] size the number of margin slices.
        */
      void
      setCheckMarginArraySize (int size)
      {
        check_margin_array_size_ = size;

        check_margin_array_.clear ();
        check_margin_array_.resize (check_margin_array_size_);

        margin_array_min_angle_.clear ();
        margin_array_min_angle_.resize (check_margin_array_size_);

        margin_array_max_angle_.clear ();
        margin_array_max_angle_.resize (check_margin_array_size_);

        margin_array_min_angle_normal_.clear ();
        margin_array_min_angle_normal_.resize (check_margin_array_size_);

        margin_array_max_angle_normal_.clear ();
        margin_array_max_angle_normal_.resize (check_margin_array_size_);
      }

      /** \brief Gets the number of slices in which is divided the margin for the search of missing regions.
        *
        * \return the number of margin slices.
        */
      inline int
      getCheckMarginArraySize () const
      {
        return (check_margin_array_size_);
      }

      /** \brief Given the angle width of a hole in the support margin, sets the minimum percentage of a circumference this angle 
        *         must cover to be considered a missing region in the support and hence used for the estimation of the Reference Frame.
        *
        * \param[in] prob_thresh the minimum percentage of a circumference after which a hole is considered in the calculation
        */
      inline void
      setHoleSizeProbThresh (float prob_thresh)
      {
        hole_size_prob_thresh_ = prob_thresh;
      }

      /** \brief Given the angle width of a hole in the support margin, gets the minimum percentage of a circumference this angle 
        *         must cover to be considered a missing region in the support and hence used for the estimation of the Reference Frame.
        *
        * \return the minimum percentage of a circumference after which a hole is considered in the calculation
        */
      inline float
      getHoleSizeProbThresh () const
      {
        return (hole_size_prob_thresh_);
      }

      /** \brief Sets the minimum steepness that the normals of the points situated on the borders of the hole, with reference
        *         to the normal of the best point found by the algorithm, must have in order to be considered in the calculation of the Reference Frame.
        *
        * \param[in] steep_thresh threshold that defines if a missing region contains a point with the most different normal.
        */
      inline void
      setSteepThresh (float steep_thresh)
      {
        steep_thresh_ = steep_thresh;
      }

      /** \brief Gets the minimum steepness that the normals of the points situated on the borders of the hole, with reference
        *         to the normal of the best point found by the algorithm, must have in order to be considered in the calculation of the Reference Frame.
        *
        * \return threshold that defines if a missing region contains a point with the most different normal.
        */
      inline float
      getSteepThresh () const
      {
        return (steep_thresh_);
      }

    protected:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::tree_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      using PointCloudIn = typename Feature<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;

      void
      resetData ()
      {
        setCheckMarginArraySize (check_margin_array_size_);
      }

      /** \brief Estimate the LRF descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] index the index of the point in input_
        * \param[out] lrf the resultant local reference frame
        */
      float
      computePointLRF (const int &index, Eigen::Matrix3f &lrf);

      /** \brief Abstract feature estimation method.
        * \param[out] output the resultant features
        */
      void
      computeFeature (PointCloudOut &output) override;

      /** \brief Given an axis (with origin axis_origin), return the orthogonal axis directed to point.
        *
        * \note axis must be normalized.
        *
        * \param[in] axis the axis
        * \param[in] axis_origin the axis_origin
        * \param[in] point the point towards which the resulting axis is directed
        * \param[out] directed_ortho_axis the directed orthogonal axis calculated
        */
      void
      directedOrthogonalAxis (Eigen::Vector3f const &axis, Eigen::Vector3f const &axis_origin,
                              Eigen::Vector3f const &point, Eigen::Vector3f &directed_ortho_axis);

      /** \brief return the angle (in radians) that rotate v1 to v2 with respect to axis .
        *
        * \param[in] v1 the first vector
        * \param[in] v2 the second vector
        * \param[in] axis the rotation axis. Axis must be normalized and orthogonal to plane defined by v1 and v2.
        * \return angle
        */
      float
      getAngleBetweenUnitVectors (Eigen::Vector3f const &v1, Eigen::Vector3f const &v2, Eigen::Vector3f const &axis);

      /** \brief Disambiguates a normal direction using adjacent normals
        * 
        * \param[in] normals_cloud a cloud of normals used for the calculation
        * \param[in] normal_indices the indices of the normals in the cloud that should to be used for the calculation
        * \param[in,out] normal the normal to disambiguate, the calculation is performed in place
        */
      void
      normalDisambiguation (pcl::PointCloud<PointNT> const &normals_cloud, pcl::Indices const &normal_indices,
                            Eigen::Vector3f &normal);

      /** \brief Compute Least Square Plane Fitting in a set of 3D points
        *
        * \param[in] points Matrix(nPoints,3) of 3D points coordinates
        * \param[out] center centroid of the distribution of points that belongs to the fitted plane
        * \param[out] norm normal to the fitted plane
        */
      void
      planeFitting (Eigen::Matrix<float, Eigen::Dynamic, 3> const &points, Eigen::Vector3f &center,
                    Eigen::Vector3f &norm);

      /** \brief Given a plane (origin and normal) and a point, return the projection of x on plane
        *
        * Equivalent to vtkPlane::ProjectPoint()
        *
        * \param[in] point the point to project
        * \param[in] origin_point a point belonging to the plane
        * \param[in] plane_normal normal of the plane
        * \param[out] projected_point the projection of the point on the plane
        */
      void
      projectPointOnPlane (Eigen::Vector3f const &point, Eigen::Vector3f const &origin_point,
                           Eigen::Vector3f const &plane_normal, Eigen::Vector3f &projected_point);

      /** \brief Given an axis, return a random orthogonal axis
        *
        * \param[in] axis input axis
        * \param[out] rand_ortho_axis an axis orthogonal to the input axis and whose direction is random
        */
      void
      randomOrthogonalAxis (Eigen::Vector3f const &axis, Eigen::Vector3f &rand_ortho_axis);

      /** \brief Check if val1 and val2 are equals.
        *
        * \param[in] val1 first number to check.
        * \param[in] val2 second number to check.
        * \param[in] zero_float_eps epsilon
        * \return true if val1 is equal to val2, false otherwise.
        */
      inline bool
      areEquals (float val1, float val2, float zero_float_eps = 1E-8f) const
      {
        return (std::abs (val1 - val2) < zero_float_eps);
      }

    private:
      /** \brief Radius used to find tangent axis. */
      float tangent_radius_;

      /** \brief If true, check if support is complete or has missing regions because it is too near to mesh borders. */
      bool find_holes_;

      /** \brief Threshold that define if a support point is near the margins. */
      float margin_thresh_; 

      /** \brief Number of slices that divide the support in order to determine if a missing region is present. */
      int check_margin_array_size_; 

      /** \brief Threshold used to determine a missing region */
      float hole_size_prob_thresh_; 

      /** \brief Threshold that defines if a missing region contains a point with the most different normal. */
      float steep_thresh_; 

      std::vector<bool> check_margin_array_;
      std::vector<float> margin_array_min_angle_;
      std::vector<float> margin_array_max_angle_;
      std::vector<float> margin_array_min_angle_normal_;
      std::vector<float> margin_array_max_angle_normal_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/board.hpp>
#endif

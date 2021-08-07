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
 */

#pragma once

#ifdef __GNUC__
#pragma GCC system_header
#endif

#include <pcl/pcl_base.h>
#include <pcl/memory.h>
#include <pcl/types.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/pcl_exports.h>

namespace pcl
{
  /** \brief Correspondence represents a match between two entities (e.g., points, descriptors, etc). This is
    * represented via the indices of a \a source point and a \a target point, and the distance between them.
    *
    * \author Dirk Holz, Radu B. Rusu, Bastian Steder
    * \ingroup common
    */
  struct Correspondence
  {
    /** \brief Index of the query (source) point. */
    index_t index_query = 0;
    /** \brief Index of the matching (target) point. Set to -1 if no correspondence found. */
    index_t index_match = UNAVAILABLE;
    /** \brief Distance between the corresponding points, or the weight denoting the confidence in correspondence estimation */
    union
    {
      float distance = std::numeric_limits<float>::max();
      float weight;
    };

    /** \brief Standard constructor.
      * Sets \ref index_query to 0, \ref index_match to -1, and \ref distance to FLT_MAX.
      */
    inline Correspondence () = default;

    /** \brief Constructor. */
    inline Correspondence (index_t _index_query, index_t _index_match, float _distance) :
      index_query (_index_query), index_match (_index_match), distance (_distance)
    {}

    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** \brief overloaded << operator */
  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Correspondence& c);

  using Correspondences = std::vector< pcl::Correspondence, Eigen::aligned_allocator<pcl::Correspondence> >;
  using CorrespondencesPtr = shared_ptr<Correspondences>;
  using CorrespondencesConstPtr = shared_ptr<const Correspondences >;

  /**
    * \brief Get the query points of correspondences that are present in
    * one correspondence vector but not in the other, e.g., to compare
    * correspondences before and after rejection.
    * \param[in] correspondences_before Vector of correspondences before rejection
    * \param[in] correspondences_after Vector of correspondences after rejection
    * \param[out] indices Query point indices of correspondences that have been rejected
    * \param[in] presorting_required Enable/disable internal sorting of vectors.
    * By default (true), vectors are internally sorted before determining their difference.
    * If the order of correspondences in \a correspondences_after is not different (has not been changed)
    * from the order in \b correspondences_before this pre-processing step can be disabled
    * in order to gain efficiency. In order to disable pre-sorting set \a presorting_required to false.
    */
  void
  getRejectedQueryIndices (const pcl::Correspondences &correspondences_before,
                           const pcl::Correspondences &correspondences_after,
                           Indices& indices,
                           bool presorting_required = true);

  /**
    * \brief Representation of a (possible) correspondence between two 3D points in two different coordinate frames
    *        (e.g. from feature matching)
    * \ingroup common
    */
  struct PointCorrespondence3D : public Correspondence
  {
    Eigen::Vector3f point1;  //!< The 3D position of the point in the first coordinate frame
    Eigen::Vector3f point2;  //!< The 3D position of the point in the second coordinate frame

    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
  using PointCorrespondences3DVector = std::vector<PointCorrespondence3D, Eigen::aligned_allocator<PointCorrespondence3D> >;

  /**
    * \brief Representation of a (possible) correspondence between two points (e.g. from feature matching),
    *        that encode complete 6DOF transformations.
    * \ingroup common
    */
  struct PointCorrespondence6D : public PointCorrespondence3D
  {
    Eigen::Affine3f transformation;  //!< The transformation to go from the coordinate system
                                        //!< of point2 to the coordinate system of point1

    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
  using PointCorrespondences6DVector = std::vector<PointCorrespondence6D, Eigen::aligned_allocator<PointCorrespondence6D> >;

  /**
    * \brief Comparator to enable us to sort a vector of PointCorrespondences according to their scores using
    *        std::sort (begin(), end(), isBetterCorrespondence);
    * \ingroup common
    */
  inline bool
  isBetterCorrespondence (const Correspondence &pc1, const Correspondence &pc2)
  {
    return (pc1.distance > pc2.distance);
  }
}

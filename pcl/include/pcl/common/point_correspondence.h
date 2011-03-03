/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

/* \author Bastian Steder */

#ifndef PCL_POINT_CORRESPONDENCE_H_
#define PCL_POINT_CORRESPONDENCE_H_

#include <vector>
#include <Eigen/Geometry>

namespace pcl {

/**
 * \brief Representation of a (possible) correspondence between two points in two different coordinate frames
 *        (e.g. from feature matching)
 */
struct PointCorrespondence
{
  int index1;   //!< The index of point1
  int index2;   //!< The index of point2 
  float score;  /**< The score of this correspondence
                  *  (e.g. from the descriptor distance of the feature matching process) */
  //inline bool operator<(const PointCorrespondence& other) const { return score<other.score;}
};


/**
 * \brief Representation of a (possible) correspondence between two 3D points in two different coordinate frames
 *        (e.g. from feature matching)
 */
struct PointCorrespondence3D : public PointCorrespondence
{
  Eigen::Vector3f point1;  //!< The 3D position of the point in the first coordinate frame
  Eigen::Vector3f point2;  //!< The 3D position of the point in the second coordinate frame

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef std::vector<PointCorrespondence3D, Eigen::aligned_allocator<PointCorrespondence3D> >
        PointCorrespondences3DVector;


/**
 * \brief Representation of a (possible) correspondence between two points (e.g. from feature matching),
 *        that encode complete 6DOF transoformations.
 */
struct PointCorrespondence6D : public PointCorrespondence3D
{
  Eigen::Affine3f transformation;  //!< The transformation to go from the coordinate system
                                      //!< of point2 to the coordinate system of point1
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef std::vector<PointCorrespondence6D, Eigen::aligned_allocator<PointCorrespondence6D> >
        PointCorrespondences6DVector;

/**
 * \brief Comparator to enable us to sort a vector of PointCorrespondences according to their scores using
 *        std::sort(begin(), end(), isBetterCorrespondence);
 */
inline bool
isBetterCorrespondence (const PointCorrespondence& pc1, const PointCorrespondence& pc2)
{
  return pc1.score > pc2.score;
}

}  // end namespace pcl

#endif  //#ifndef PCL_POINT_CORRESPONDENCE_H_

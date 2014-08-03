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
 * Author : Sergey Ushakov
 * Email  : sergey.s.ushakov@mail.ru
 *
 */

#ifndef PCL_ROPS_ESIMATION_H_
#define PCL_ROPS_ESIMATION_H_

#include <pcl/PolygonMesh.h>
#include <pcl/features/feature.h>
#include <set>

namespace pcl
{
  /** \brief
    * This class implements the method for extracting RoPS features presented in the article
    * "Rotational Projection Statistics for 3D Local Surface Description and Object Recognition" by
    * Yulan Guo, Ferdous Sohel, Mohammed Bennamoun, Min Lu and Jianwei Wan.
    */
  template <typename PointInT, typename PointOutT>
  class PCL_EXPORTS ROPSEstimation : public pcl::Feature <PointInT, PointOutT>
  {
    public:

      using Feature <PointInT, PointOutT>::input_;
      using Feature <PointInT, PointOutT>::indices_;
      using Feature <PointInT, PointOutT>::surface_;
      using Feature <PointInT, PointOutT>::tree_;

      typedef typename pcl::Feature <PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename pcl::Feature <PointInT, PointOutT>::PointCloudIn PointCloudIn;

    public:

      /** \brief Simple constructor. */
      ROPSEstimation ();

      /** \brief Virtual destructor. */
      virtual
      ~ROPSEstimation ();

      /** \brief Allows to set the number of partition bins that is used for distribution matrix calculation.
        * \param[in] number_of_bins number of partition bins
        */
      void
      setNumberOfPartitionBins (unsigned int number_of_bins);

      /** \brief Returns the nmber of partition bins. */
      unsigned int
      getNumberOfPartitionBins () const;

      /** \brief This method sets the number of rotations.
        * \param[in] number_of_rotations number of rotations
        */
      void
      setNumberOfRotations (unsigned int number_of_rotations);

      /** \brief returns the number of rotations. */
      unsigned int
      getNumberOfRotations () const;

      /** \brief Allows to set the support radius that is used to crop the local surface of the point.
        * \param[in] support_radius support radius
        */
      void
      setSupportRadius (float support_radius);

      /** \brief Returns the support radius. */
      float
      getSupportRadius () const;

      /** \brief This method sets the triangles of the mesh.
        * \param[in] triangles list of triangles of the mesh
        */
      void
      setTriangles (const std::vector <pcl::Vertices>& triangles);

      /** \brief Returns the triangles of the mesh.
        * \param[out] triangles triangles of tthe mesh
        */
      void
      getTriangles (std::vector <pcl::Vertices>& triangles) const;

    private:

      /** \brief Abstract feature estimation method.
        * \param[out] output the resultant features
        */
      virtual void
      computeFeature (PointCloudOut& output);

      /** \brief This method simply builds the list of triangles for every point.
        * The list of triangles for each point consists of indices of triangles it belongs to.
        * The only purpose of this method is to improve perfomance of the algorithm.
        */
      void
      buildListOfPointsTriangles ();

      /** \brief This method crops all the triangles within the given radius of the given point.
        * \param[in] point point for which the local surface is computed
        * \param[out] local_triangles strores the indices of the triangles that belong to the local surface
        * \param[out] local_points stores the indices of the points that belong to the local surface
        */
      void
      getLocalSurface (const PointInT& point, std::set <unsigned int>& local_triangles, std::vector <int>& local_points) const;

      /** \brief This method computes LRF (Local Reference Frame) matrix for the given point.
        * \param[in] point point for which the LRF is computed
        * \param[in] local_triangles list of triangles that represents the local surface of the point
        * \paran[out] lrf_matrix strores computed LRF matrix for the given point
        */
      void
      computeLRF (const PointInT& point, const std::set <unsigned int>& local_triangles, Eigen::Matrix3f& lrf_matrix) const;

      /** \brief This method calculates the eigen values and eigen vectors
        * for the given covariance matrix. Note that it returns normalized eigen
        * vectors that always form the right-handed coordinate system.
        * \param[in] matrix covariance matrix of the cloud
        * \param[out] major_axis eigen vector which corresponds to a major eigen value
        * \param[out] middle_axis eigen vector which corresponds to a middle eigen value
        * \param[out] minor_axis eigen vector which corresponds to a minor eigen value
        */
      void
      computeEigenVectors (const Eigen::Matrix3f& matrix, Eigen::Vector3f& major_axis, Eigen::Vector3f& middle_axis,
                           Eigen::Vector3f& minor_axis) const;

      /** \brief This method translates the cloud so that the given point becomes the origin.
        * After that the cloud is rotated with the help of the given matrix.
        * \param[in] point point which stores the translation information
        * \param[in] matrix rotation matrix
        * \param[in] local_points point to transform
        * \param[out] transformed_cloud stores the transformed cloud
        */
      void
      transformCloud (const PointInT& point, const Eigen::Matrix3f& matrix, const std::vector <int>& local_points, PointCloudIn& transformed_cloud) const;

      /** \brief This method rotates the cloud around the given axis and computes AABB of the rotated cloud.
        * \param[in] axis axis around which cloud must be rotated
        * \param[in] angle angle in degrees
        * \param[in] cloud cloud to rotate
        * \param[out] rotated_cloud stores the rotated cloud
        * \param[out] min stores the min point of the AABB
        * \param[out] max stores the max point of the AABB
        */
      void
      rotateCloud (const PointInT& axis, const float angle, const PointCloudIn& cloud, PointCloudIn& rotated_cloud,
                   Eigen::Vector3f& min, Eigen::Vector3f& max) const;

      /** \brief This method projects the local surface onto the XY, XZ or YZ plane
        * and computes the distribution matrix.
        * \param[in] projection represents the case of projection. 1 - XY, 2 - XZ, 3 - YZ
        * \param[in] min min point of the AABB
        * \param[in] max max point of the AABB
        * \param[in] cloud cloud containing the points of the local surface
        * \param[out] matrix stores computed distribution matrix
        */
      void
      getDistributionMatrix (const unsigned int projection, const Eigen::Vector3f& min, const Eigen::Vector3f& max, const PointCloudIn& cloud, Eigen::MatrixXf& matrix) const;

      /** \brief This method computes the set ofcentral moments for the given matrix.
        * \param[in] matrix input matrix
        * \param[out] moments set of computed moments
        */
      void
      computeCentralMoments (const Eigen::MatrixXf& matrix, std::vector <float>& moments) const;

    private:

      /** \brief Stores the number of partition bins that is used for distribution matrix calculation. */
      unsigned int number_of_bins_;

      /** \brief Stores number of rotations. Central moments are calculated for every rotation. */
      unsigned int number_of_rotations_;

      /** \brief Support radius that is used to crop the local surface of the point. */
      float support_radius_;

      /** \brief Stores the squared support radius. Used to improve performance. */
      float sqr_support_radius_;

      /** \brief Stores the angle step. Step is calculated with respect to number of rotations. */
      float step_;

      /** \brief Stores the set of triangles reprsenting the mesh. */
      std::vector <pcl::Vertices> triangles_;

      /** \brief Stores the set of triangles for each point. Its purpose is to improve perfomance. */
      std::vector <std::vector <unsigned int> > triangles_of_the_point_;

    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#define PCL_INSTANTIATE_ROPSEstimation(InT, OutT) template class pcl::ROPSEstimation<InT, OutT>;

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/rops_estimation.hpp>
#endif

#endif

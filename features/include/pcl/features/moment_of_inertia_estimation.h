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

#pragma once

#include <vector>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/pcl_base.h>

namespace pcl
{
  /** \brief
    * Implements the method for extracting features based on moment of inertia. It also
    * calculates AABB, OBB and eccentricity of the projected cloud.
    */
  template <typename PointT>
  class PCL_EXPORTS MomentOfInertiaEstimation : public pcl::PCLBase <PointT>
  {
    public:

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::fake_indices_;
      using PCLBase <PointT>::use_indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

      using PointCloudConstPtr = typename pcl::PCLBase<PointT>::PointCloudConstPtr;
      using PointIndicesConstPtr = typename pcl::PCLBase<PointT>::PointIndicesConstPtr;

    public:

      /** \brief Provide a pointer to the input dataset
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      void
      setInputCloud (const PointCloudConstPtr& cloud) override;

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param[in] indices a pointer to the vector of indices that represents the input data.
        */
      void
      setIndices (const IndicesPtr& indices) override;

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param[in] indices a pointer to the vector of indices that represents the input data.
        */
      void
      setIndices (const IndicesConstPtr& indices) override;

      /** \brief Provide a pointer to the vector of indices that represents the input data.
        * \param[in] indices a pointer to the vector of indices that represents the input data.
        */
      void
      setIndices (const PointIndicesConstPtr& indices) override;

      /** \brief Set the indices for the points laying within an interest region of 
        * the point cloud.
        * \note you shouldn't call this method on unorganized point clouds!
        * \param[in] row_start the offset on rows
        * \param[in] col_start the offset on columns
        * \param[in] nb_rows the number of rows to be considered row_start included
        * \param[in] nb_cols the number of columns to be considered col_start included
        */
      void
      setIndices (std::size_t row_start, std::size_t col_start, std::size_t nb_rows, std::size_t nb_cols) override;

      /** \brief Constructor that sets default values for member variables. */
      MomentOfInertiaEstimation ();

      /** \brief Virtual destructor which frees the memory. */
      
      ~MomentOfInertiaEstimation () override;

      /** \brief This method allows to set the angle step. It is used for the rotation
        * of the axis which is used for moment of inertia/eccentricity calculation.
        * \param[in] step angle step
        */
      void
      setAngleStep (const float step);

      /** \brief Returns the angle step. */
      float
      getAngleStep () const;

      /** \brief This method allows to set the normalize_ flag. If set to false, then
        * point_mass_ will be used to scale the moment of inertia values. Otherwise,
        * point_mass_ will be set to 1 / number_of_points. Default value is true.
        * \param[in] need_to_normalize desired value
        */
      void
      setNormalizePointMassFlag (bool need_to_normalize);

      /** \brief Returns the normalize_ flag. */
      bool
      getNormalizePointMassFlag () const;

      /** \brief This method allows to set point mass that will be used for
        * moment of inertia calculation. It is needed to scale moment of inertia values.
        * default value is 0.0001.
        * \param[in] point_mass point mass
        */
      void
      setPointMass (const float point_mass);

      /** \brief Returns the mass of point. */
      float
      getPointMass () const;

      /** \brief This method launches the computation of all features. After execution
        * it sets is_valid_ flag to true and each feature can be accessed with the
        * corresponding get method.
        */
      void
      compute ();

      /** \brief This method gives access to the computed axis aligned bounding box. It returns true
        * if the current values (eccentricity, moment of inertia etc) are valid and false otherwise.
        * \param[out] min_point min point of the AABB
        * \param[out] max_point max point of the AABB
        */
      bool
      getAABB (PointT& min_point, PointT& max_point) const;

      /** \brief This method gives access to the computed oriented bounding box. It returns true
        * if the current values (eccentricity, moment of inertia etc) are valid and false otherwise.
        * Note that in order to get the OBB, each vertex of the given AABB (specified with min_point and max_point)
        * must be rotated with the given rotational matrix (rotation transform) and then positioned.
        * Also pay attention to the fact that this is not the minimal possible bounding box. This is the bounding box
        * which is oriented in accordance with the eigen vectors.
        * \param[out] min_point min point of the OBB
        * \param[out] max_point max point of the OBB
        * \param[out] position position of the OBB
        * \param[out] rotational_matrix this matrix represents the rotation transform
        */
      bool
      getOBB (PointT& min_point, PointT& max_point, PointT& position, Eigen::Matrix3f& rotational_matrix) const;

      /** \brief This method gives access to the computed eigen values. It returns true
        * if the current values (eccentricity, moment of inertia etc) are valid and false otherwise.
        * \param[out] major major eigen value
        * \param[out] middle middle eigen value
        * \param[out] minor minor eigen value
        */
      bool
      getEigenValues (float& major, float& middle, float& minor) const;

      /** \brief This method gives access to the computed eigen vectors. It returns true
        * if the current values (eccentricity, moment of inertia etc) are valid and false otherwise.
        * \param[out] major axis which corresponds to the eigen vector with the major eigen value
        * \param[out] middle axis which corresponds to the eigen vector with the middle eigen value
        * \param[out] minor axis which corresponds to the eigen vector with the minor eigen value
        */
      bool
      getEigenVectors (Eigen::Vector3f& major, Eigen::Vector3f& middle, Eigen::Vector3f& minor) const;

      /** \brief This method gives access to the computed moments of inertia. It returns true
        * if the current values (eccentricity, moment of inertia etc) are valid and false otherwise.
        * \param[out] moment_of_inertia computed moments of inertia
        */
      bool
      getMomentOfInertia (std::vector <float>& moment_of_inertia) const;

      /** \brief This method gives access to the computed ecentricities. It returns true
        * if the current values (eccentricity, moment of inertia etc) are valid and false otherwise.
        * \param[out] eccentricity computed eccentricities
        */
      bool
      getEccentricity (std::vector <float>& eccentricity) const;

      /** \brief This method gives access to the computed mass center. It returns true
        * if the current values (eccentricity, moment of inertia etc) are valid and false otherwise.
        * Note that when mass center of a cloud is computed, mass point is always considered equal 1.
        * \param[out] mass_center computed mass center
        */
      bool
      getMassCenter (Eigen::Vector3f& mass_center) const;

    private:

      /** \brief This method rotates the given vector around the given axis.
        * \param[in] vector vector that must be rotated
        * \param[in] axis axis around which vector must be rotated
        * \param[in] angle angle in degrees
        * \param[out] rotated_vector resultant vector
        */
      void
      rotateVector (const Eigen::Vector3f& vector, const Eigen::Vector3f& axis, const float angle, Eigen::Vector3f& rotated_vector) const;

      /** \brief This method computes center of mass and axis aligned bounding box. */
      void
      computeMeanValue ();

      /** \brief This method computes the oriented bounding box. */
      void
      computeOBB ();

      /** \brief This method computes the covariance matrix for the input_ cloud.
        * \param[out] covariance_matrix stores the computed covariance matrix
        */
      void
      computeCovarianceMatrix (Eigen::Matrix <float, 3, 3>& covariance_matrix) const;

      /** \brief This method computes the covariance matrix for the given cloud.
        * It uses all points in the cloud, unlike the previous method that uses indices.
        * \param[in] cloud cloud for which covariance matrix will be computed
        * \param[out] covariance_matrix stores the computed covariance matrix
        */
      void
      computeCovarianceMatrix (PointCloudConstPtr cloud, Eigen::Matrix <float, 3, 3>& covariance_matrix) const;

      /** \brief This method calculates the eigen values and eigen vectors
        * for the given covariance matrix. Note that it returns normalized eigen
        * vectors that always form the right-handed coordinate system.
        * \param[in] covariance_matrix covariance matrix
        * \param[out] major_axis eigen vector which corresponds to a major eigen value
        * \param[out] middle_axis eigen vector which corresponds to a middle eigen value
        * \param[out] minor_axis eigen vector which corresponds to a minor eigen value
        * \param[out] major_value major eigen value
        * \param[out] middle_value middle eigen value
        * \param[out] minor_value minor eigen value
        */
      void
      computeEigenVectors (const Eigen::Matrix <float, 3, 3>& covariance_matrix, Eigen::Vector3f& major_axis,
                           Eigen::Vector3f& middle_axis, Eigen::Vector3f& minor_axis, float& major_value, float& middle_value,
                           float& minor_value);

      /** \brief This method returns the moment of inertia of a given input_ cloud.
        * Note that when moment of inertia is computed it is multiplied by the point mass.
        * Point mass can be accessed with the corresponding get/set methods.
        * \param[in] current_axis axis that will be used in moment of inertia computation
        * \param[in] mean_value mean value(center of mass) of the cloud
        */
      float
      calculateMomentOfInertia (const Eigen::Vector3f& current_axis, const Eigen::Vector3f& mean_value) const;

      /** \brief This method simply projects the given input_ cloud on the plane specified with
        * the normal vector.
        * \param[in] normal_vector nrmal vector of the plane
        * \param[in] point point belonging to the plane
        * \param[out] projected_cloud projected cloud
        */
      void
      getProjectedCloud (const Eigen::Vector3f& normal_vector, const Eigen::Vector3f& point, typename pcl::PointCloud <PointT>::Ptr projected_cloud) const;

      /** \brief This method returns the eccentricity of the projected cloud.
        * \param[in] covariance_matrix covariance matrix of the projected cloud
        * \param[in] normal_vector normal vector of the plane, it is used to discard the
        *            third eigen vector and eigen value*/
      float
      computeEccentricity (const Eigen::Matrix <float, 3, 3>& covariance_matrix, const Eigen::Vector3f& normal_vector);

    private:

      /** \brief Indicates if the stored values (eccentricity, moment of inertia, AABB etc.)
        * are valid when accessed with the get methods. */
      bool is_valid_;

      /** \brief Stores the angle step */
      float step_;

      /** \brief Stores the mass of point in the cloud */
      float point_mass_;

      /** \brief Stores the flag for mass normalization */
      bool normalize_;

      /** \brief Stores the mean value (center of mass) of the cloud */
      Eigen::Vector3f mean_value_;

      /** \brief Major eigen vector */
      Eigen::Vector3f major_axis_;

      /** \brief Middle eigen vector */
      Eigen::Vector3f middle_axis_;

      /** \brief Minor eigen vector */
      Eigen::Vector3f minor_axis_;

      /** \brief Major eigen value */
      float major_value_;

      /** \brief Middle eigen value */
      float middle_value_;

      /** \brief Minor eigen value */
      float minor_value_;

      /** \brief Stores calculated moments of inertia */
      std::vector <float> moment_of_inertia_;

      /** \brief Stores calculated eccentricities */
      std::vector <float> eccentricity_;

      /** \brief Min point of the axis aligned bounding box */
      PointT aabb_min_point_;

      /** \brief Max point of the axis aligned bounding box */
      PointT aabb_max_point_;

      /** \brief Min point of the oriented bounding box */
      PointT obb_min_point_;

      /** \brief Max point of the oriented bounding box */
      PointT obb_max_point_;

      /** \brief Stores position of the oriented bounding box */
      Eigen::Vector3f obb_position_;

      /** \brief Stores the rotational matrix of the oriented bounding box */
      Eigen::Matrix3f obb_rotational_matrix_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#define PCL_INSTANTIATE_MomentOfInertiaEstimation(T) template class pcl::MomentOfInertiaEstimation<T>;

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/moment_of_inertia_estimation.hpp>
#endif

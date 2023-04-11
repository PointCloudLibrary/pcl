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
 * Author : Alessio Stella
 * Email  : alessio.stella.g@gmail.com
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
    * Implements the method for extracting features based on statistical regression to a best fit plane
    * z=a*x+b*y+c calculating the center of mass (with uniform and unitary mass density), three main
    * regression vectors applied on it, AABB and OBB.
    */
  template <typename PointT>
  class PCL_EXPORTS RegressionEstimation : public pcl::PCLBase <PointT>
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
      RegressionEstimation ();

      /** \brief Virtual destructor which frees the memory. */
      ~RegressionEstimation () override;

      /** \brief This method allows to set the angle step. Not used yet.
        * \param[in] step angle step
        */
      //void
      //setAngleStep (const float step);

      /** \brief Returns the angle step. Not used yet.*/
      //float
      //getAngleStep () const;


       /** \brief This method allows to set the value of the essential flag which is false by default.
       *  set it true if you wish to speed up the calculation and gain about 30% of processing time
       *  If true only the most important subset of features is calculated (regression
       * vectors and mass center). If false also AABB and OBB boxes are calculated
       */
      void
      setEssential(const bool essential);

      /** \brief Returns the value of the essential flag which is false by default.
        *  If true only the most important subset of features is calculated (regression vectors and mass center)
        * if false also AABB and OBB boxes are calculated
        */
      bool
      getEssential() const;


      /** \brief This method launches the computation of all features. After execution
        * it sets is_valid_ flag to true and each feature can be accessed with the
        * corresponding get method.
        */
      void
      compute();

      /** \brief Alternative to compute() This method launches the computation of all features. After execution
       * it sets is_valid_ flag to true and each feature can be accessed with the
       * corresponding get method.
       */
      void
      computeByPCA();

      /** \brief This method gives access to the computed axis aligned bounding box. It returns true
        * if the current values are valid and false otherwise.
        * \param[out] min_point min point of the AABB
        * \param[out] max_point max point of the AABB
        */
      bool
      getAABB (PointT& min_point, PointT& max_point) const;

      /** \brief This method gives access to the computed oriented bounding box. It returns true
        * if the current values are valid and false otherwise.
        * Pay attention to the fact that this is not the minimal possible bounding box. This is the bounding box
        * which is oriented in accordance with the regression vectors.
        * \param[out] min_point min point of the OBB
        * \param[out] max_point max point of the OBB
        * \param[out] position position of the OBB
        * \param[out] rotational_matrix this matrix represents the rotation transform
        */
      bool
      getOBB (PointT& min_point, PointT& max_point, PointT& position, Eigen::Matrix3f& rotational_matrix) const;

      /** \brief This method gives access to the computed pseudoeigen values. Not used yet.
        * It returns true
        * if the current values are valid and false otherwise.
        * \param[out] major major eigen value
        * \param[out] middle middle eigen value
        * \param[out] minor minor eigen value
        */
      //bool
      //getRegressionValues (float& major, float& middle, float& minor) const;

      /** \brief This method gives access to the computed main regression axes
        * It returns true if the current values are valid and false otherwise.
        * It can be considered a (non equivalent) substitute for pcl::MomentOfInertiaEstimation<PointT>::getEigenVectors
        * \param[out] major axis 
        * \param[out] middle axis 
        * \param[out] minor axis 
        */
      bool
      getRegressionVectors (Eigen::Vector3f& major, Eigen::Vector3f& middle, Eigen::Vector3f& minor) const;


      /** \brief This method gives access to the computed mass center. It returns true
        * if the current values are valid and false otherwise.
        * Note that when mass center of a cloud is computed, mass point is always considered equal 1.
        * \param[out] mass_center computed mass center
        */
      bool
      getMassCenter (Eigen::Vector3f& mass_center) const;

    private:
      /** \brief This method calculates the mass center and calculates the best fit plane according to a
       * statistical regression algorithm
       * 
       * \param[out] a , b , c the coefficients of the calculated best fit plane of equation z=a*x+b*y+c 
       */
      void
      PlaneFittingCloud(double& a, double& b, double& c);


      /** \brief This method computes the oriented bounding box. */
      void
      computeOBB ();


    private:

      /** \brief Indicates if only the most essential set of features are calculated
        * default is false. */
      bool essential_;

      /** \brief Indicates if the stored values 
        * are valid when accessed with the get methods. */
      bool is_valid_;

      /** \brief Stores the angle step: not used yet */
      float step_;

      /** \brief Stores the mean value (center of mass) of the cloud */
      Eigen::Vector3f mean_value_;

      /** \brief Major regression vector */
      Eigen::Vector3f major_axis_;

      /** \brief Middle regression vector */
      Eigen::Vector3f middle_axis_;

      /** \brief Minor regression vector */
      Eigen::Vector3f minor_axis_;

      /** \brief Major regression value */
      float major_value_;

      /** \brief Middle regression value */
      float middle_value_;

      /** \brief Minor regression value */
      float minor_value_;

      /** \brief Min point of the axis aligned bounding box */
      PointT aabb_min_point_;

      /** \brief Max point of the axis aligned bounding box */
      PointT aabb_max_point_;

      /** \brief points coords relative to center of mass */
      std::vector<double> x_m;
      std::vector<double> y_m;
      std::vector<double> z_m;

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

#define PCL_INSTANTIATE_RegressionEstimation(T) template class pcl::RegressionEstimation<T>;

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/regression_estimation.hpp>
#endif

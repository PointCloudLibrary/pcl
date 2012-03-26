/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */
#ifndef PCL_INTENSITY_GRADIENT_H_
#define PCL_INTENSITY_GRADIENT_H_

#include <pcl/features/feature.h>
#include <pcl/common/intensity.h>

namespace pcl
{
  /** \brief IntensityGradientEstimation estimates the intensity gradient for a point cloud that contains position
    * and intensity values.  The intensity gradient at a given point will be a vector orthogonal to the surface
    * normal and pointing in the direction of the greatest increase in local intensity; the vector's magnitude
    * indicates the rate of intensity change.
    * \author Michael Dixon
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT, typename IntensitySelectorT = pcl::common::IntensityFieldAccessor<PointInT> >
  class IntensityGradientEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      /** \brief Empty constructor. */
      IntensityGradientEstimation ()
      {
        feature_name_ = "IntensityGradientEstimation";
        threads_ = 1;
      };

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      inline void
      setNumberOfThreads (int nr_threads)
      {
        threads_ = nr_threads == 0 ? 1 : nr_threads;
      }

    protected:
      /** \brief Estimate the intensity gradients for a set of points given in <setInputCloud (), setIndices ()> using
        *  the surface in setSearchSurface () and the spatial locator in setSearchMethod ().
        *  \param output the resultant point cloud that contains the intensity gradient vectors
        */
      void
      computeFeature (PointCloudOut &output);

      /** \brief Estimate the intensity gradient around a given point based on its spatial neighborhood of points
        * \param cloud a point cloud dataset containing XYZI coordinates (Cartesian coordinates + intensity)
        * \param indices the indices of the neighoring points in the dataset
        * \param point the 3D Cartesian coordinates of the point at which to estimate the gradient
        * \param normal the 3D surface normal of the given point
        * \param gradient the resultant 3D gradient vector
        */
      void
      computePointIntensityGradient (const pcl::PointCloud<PointInT> &cloud,
                                     const std::vector<int> &indices,
                                     const Eigen::Vector3f &point, 
                                     float mean_intensity, 
                                     const Eigen::Vector3f &normal,
                                     Eigen::Vector3f &gradient);

    private:
      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &) {}

    protected:
      ///intensity field accessor structure
      IntensitySelectorT intensity_;
      ///number of threads to be used, default 1
      int threads_;
  };

  /** \brief IntensityGradientEstimation estimates the intensity gradient for a point cloud that contains position
    * and intensity values.  The intensity gradient at a given point will be a vector orthogonal to the surface
    * normal and pointing in the direction of the greatest increase in local intensity; the vector's magnitude
    * indicates the rate of intensity change.
    * \author Michael Dixon
    * \ingroup features
    */
  template <typename PointInT, typename PointNT>
  class IntensityGradientEstimation<PointInT, PointNT, Eigen::MatrixXf>: public IntensityGradientEstimation<PointInT, PointNT, pcl::IntensityGradient>
  {
    public:
      using IntensityGradientEstimation<PointInT, PointNT, pcl::IntensityGradient>::indices_;
      using IntensityGradientEstimation<PointInT, PointNT, pcl::IntensityGradient>::normals_;
      using IntensityGradientEstimation<PointInT, PointNT, pcl::IntensityGradient>::input_;
      using IntensityGradientEstimation<PointInT, PointNT, pcl::IntensityGradient>::surface_;
      using IntensityGradientEstimation<PointInT, PointNT, pcl::IntensityGradient>::k_;
      using IntensityGradientEstimation<PointInT, PointNT, pcl::IntensityGradient>::search_parameter_;
      using IntensityGradientEstimation<PointInT, PointNT, pcl::IntensityGradient>::compute;

    protected:
      /** \brief Estimate the intensity gradients for a set of points given in <setInputCloud (), setIndices ()> using
        *  the surface in setSearchSurface () and the spatial locator in setSearchMethod ().
        *  \param output the resultant point cloud that contains the intensity gradient vectors
        */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output);

      /** \brief Make the compute (&PointCloudOut); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      compute (pcl::PointCloud<pcl::Normal> &) {}
  };
}

#endif // #ifndef PCL_INTENSITY_GRADIENT_H_

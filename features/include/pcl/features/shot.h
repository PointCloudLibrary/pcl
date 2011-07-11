/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *
 */

#ifndef PCL_SHOT_H_
#define PCL_SHOT_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief @b SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for 
    * a given point cloud dataset containing points and normals.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> F. Tombari, S. Salti, L. Di Stefano
    *      Unique Signatures of Histograms for Local Surface Description.
    *      In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *      Heraklion, Greece, September 5-11 2010.
    * </li>
    * <li> F. Tombari, S. Salti, L. Di Stefano
    *      A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *      In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *      Brussels, Belgium, September 11-14 2011.
    * </li>
    * </ul>
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT> 
  class SHOTEstimationBase : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

    protected:
      /** \brief Empty constructor. */
      SHOTEstimationBase (int nr_shape_bins = 10) :
        nr_shape_bins_ (nr_shape_bins),
        rf_ (3),                    // Initialize the placeholder for the point's RF
        nr_grid_sector_ (32),
        maxAngularSectors_ (28),
        descLength_ (0)
      {
        feature_name_ = "SHOTEstimation";
      };

    public:
      /** \brief @b SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for 
        * a given point cloud dataset containing points and normals.
        * \param cloud the dataset containing the XYZ Cartesian coordinates of the points
        * \param normals the dataset containing the surface normals at each point in \a cloud
        * \param index the index of the point
        * \param indices the k-neighborhood point indices in the dataset
        * \param nr_bins the number of bins in each histogram
        * \param shot the resultant SHOT descriptor representing the feature at the query point
        */
      virtual void 
      computePointSHOT (const pcl::PointCloud<PointInT> &cloud, 
                        const pcl::PointCloud<PointNT> &normals,
                        const int index, 
                        const std::vector<int> &indices, 
                        const std::vector<float> &dists, 
                        Eigen::VectorXf &shot,
                        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf) = 0;

      /** \brief
        */
      float 
      getSHOTLocalRF (const pcl::PointCloud<PointInT> &cloud, 
                      const pcl::PointCloud<PointNT> &normals, 
                      const int index, 
                      const std::vector<int> &indices, 
                      const std::vector<float> &dists, 
                      std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf);

    protected:

      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void 
      computeFeature (PointCloudOut &output);

      void 
      interpolateSingleChannel (const pcl::PointCloud<PointInT> &cloud, 
                                const std::vector<int> &indices,
                                const std::vector<float> &dists, 
                                const Eigen::Vector4f &centralPoint, 
                                const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf
,
                                std::vector<double> &binDistance, 
                                const int nr_bins,
                                Eigen::VectorXf &shot);

      /** \brief The number of bins in each shape histogram. */
      const int nr_shape_bins_;

      /** \brief Placeholder for a point's SHOT. */
      Eigen::VectorXf shot_;

      /** \brief Placeholder for a point's RF. */
      std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > rf_;

      /** \brief The squared search radius.*/
      double sqradius_;

      /** \brief 1/4 of the squared search radius. */
      double sqradius4_;

      /** \brief 3/4 of the search radius. */
      double radius3_4_;

      /** \brief 1/4 of the search radius. */
      double radius1_4_;

      /** \brief 1/2 of the search radius. */
      double radius1_2_;

      /** \brief Number of azimuthal sectors. */
      const int nr_grid_sector_;

      /** \brief . */
      const int maxAngularSectors_;

      /** \brief One SHOT length. */
      int descLength_;
  };

  /** \brief @b SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for 
    * a given point cloud dataset containing points and normals.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> F. Tombari, S. Salti, L. Di Stefano
    *      Unique Signatures of Histograms for Local Surface Description.
    *      In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *      Heraklion, Greece, September 5-11 2010.
    * </li>
    * <li> F. Tombari, S. Salti, L. Di Stefano
    *      A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *      In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *      Brussels, Belgium, September 11-14 2011.
    * </li>
    * </ul>
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT> 
  class SHOTEstimation : public SHOTEstimationBase<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::descLength_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::nr_grid_sector_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::nr_shape_bins_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::sqradius_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::sqradius4_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::radius3_4_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::radius1_4_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::radius1_2_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::rf_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::maxAngularSectors_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::interpolateSingleChannel;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT>::shot_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

      /** \brief Empty constructor. */
      SHOTEstimation (int nr_shape_bins = 10) : SHOTEstimationBase<PointInT, PointNT, PointOutT> (nr_shape_bins)
      {
        feature_name_ = "SHOTEstimation";
      };

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param cloud the dataset containing the XYZ Cartesian coordinates of the points
        * \param normals the dataset containing the surface normals at each point in \a cloud
        * \param index the index of the point
        * \param indices the k-neighborhood point indices in the dataset
        * \param nr_bins the number of bins in each histogram
        * \param shot the resultant SHOT descriptor representing the feature at the query point
        */
      void 
      computePointSHOT (const pcl::PointCloud<PointInT> &cloud, 
                        const pcl::PointCloud<PointNT> &normals,
                        const int index, 
                        const std::vector<int> &indices, 
                        const std::vector<float> &dists, 
                        Eigen::VectorXf &shot,
                        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf);
  };

  /** \brief @b SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for a given point cloud dataset
    * containing points and normals.
    *
    * @note If you use this code in any academic work, please cite:
    *
    * <ul>
    * <li> F. Tombari, S. Salti, L. Di Stefano
    *      Unique Signatures of Histograms for Local Surface Description.
    *      In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *      Heraklion, Greece, September 5-11 2010.
    * </li>
    * <li> F. Tombari, S. Salti, L. Di Stefano
    *      A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *      In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *      Brussels, Belgium, September 11-14 2011.
    * </li>
    * </ul>
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointNT, typename PointOutT> 
  class SHOTEstimation<pcl::PointXYZRGBA, PointNT, PointOutT> : public SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>
  {
    public:
      using Feature<pcl::PointXYZRGBA, PointOutT>::feature_name_;
      using Feature<pcl::PointXYZRGBA, PointOutT>::getClassName;
      using Feature<pcl::PointXYZRGBA, PointOutT>::indices_;
      using Feature<pcl::PointXYZRGBA, PointOutT>::k_;
      using Feature<pcl::PointXYZRGBA, PointOutT>::search_parameter_;
      using Feature<pcl::PointXYZRGBA, PointOutT>::search_radius_;
      using Feature<pcl::PointXYZRGBA, PointOutT>::surface_;
      using FeatureFromNormals<pcl::PointXYZRGBA, PointNT, PointOutT>::normals_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::descLength_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::nr_grid_sector_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::nr_shape_bins_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::sqradius_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::sqradius4_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::radius3_4_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::radius1_4_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::radius1_2_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::rf_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::maxAngularSectors_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::interpolateSingleChannel;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT>::shot_;

      typedef typename Feature<pcl::PointXYZRGBA, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Feature<pcl::PointXYZRGBA, PointOutT>::PointCloudIn PointCloudIn;

      /** \brief Empty constructor. */
      SHOTEstimation (bool describeShape = true, 
                      bool describeColor = false, 
                      const int nr_shape_bins = 10, 
                      const int nr_color_bins = 30) 
        : SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, PointOutT> (nr_shape_bins),
          b_describe_shape_ (describeShape),
          b_describe_color_ (describeColor),
          nr_color_bins_ (nr_color_bins)
      {
        feature_name_ = "SHOTEstimation";
      };

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param cloud the dataset containing the XYZ Cartesian coordinates of the points
        * \param normals the dataset containing the surface normals at each point in \a cloud
        * \param index the index of the point
        * \param indices the k-neighborhood point indices in the dataset
        * \param nr_bins the number of bins in each histogram
        * \param shot the resultant SHOT descriptor representing the feature at the query point
        */
      void 
      computePointSHOT (const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, 
                        const pcl::PointCloud<PointNT> &normals, 
                        const int index, 
                        const std::vector<int> &indices, 
                        const std::vector<float> &dists, 
                        Eigen::VectorXf &shot,
                        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf);

    protected:

      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void 
      computeFeature (PointCloudOut &output);

      /** \brief
       */
      void 
      interpolateDoubleChannel (const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, 
                                const std::vector<int> &indices,
                                const std::vector<float> &dists, 
                                const Eigen::Vector4f &centralPoint, 
                                const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &rf,
                                std::vector<double> &binDistanceShape, 
                                std::vector<double> &binDistanceColor, 
                                const int nr_bins_shape,
                                const int nr_bins_color,
                                Eigen::VectorXf &shot);

      /** \brief
       */
      static void 
      RGB2CIELAB (unsigned char R, unsigned char G, unsigned char B, float &L, float &A, float &B2);

      /** \brief Compute shape descriptor. */
      bool b_describe_shape_;

      /** \brief Compute color descriptor. */
      bool b_describe_color_;

      /** \brief The number of bins in each color histogram. */
      int nr_color_bins_;

    public:
      static float sRGB_LUT[256];
      static float sXYZ_LUT[4000];
  };
}

#endif  //#ifndef PCL_SHOT_H_



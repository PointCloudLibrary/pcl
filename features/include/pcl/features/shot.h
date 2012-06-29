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
 *
 */

#ifndef PCL_SHOT_H_
#define PCL_SHOT_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>

namespace pcl
{
  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
    * a given point cloud dataset containing points and normals.
    *
    * The suggested PointOutT is pcl::SHOT.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT, typename PointRFT = pcl::ReferenceFrame>
  class SHOTEstimationBase : public FeatureFromNormals<PointInT, PointNT, PointOutT>,
                             public FeatureWithLocalReferenceFrames<PointInT, PointRFT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::fake_surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;
      using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::frames_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

    protected:
      /** \brief Empty constructor.
        * \param[in] nr_shape_bins the number of bins in the shape histogram
        */
      SHOTEstimationBase (int nr_shape_bins = 10) :
        nr_shape_bins_ (nr_shape_bins),
        shot_ (),
        sqradius_ (0), radius3_4_ (0), radius1_4_ (0), radius1_2_ (0),
        nr_grid_sector_ (32),
        maxAngularSectors_ (28),
        descLength_ (0)
      {
        feature_name_ = "SHOTEstimation";
      };

    public:
       /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
         * \param[in] index the index of the point in indices_
         * \param[in] indices the k-neighborhood point indices in surface_
         * \param[in] sqr_dists the k-neighborhood point distances in surface_
         * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
         */
      virtual void
      computePointSHOT (const int index,
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot) = 0;

    protected:

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

      /** \brief Quadrilinear interpolation used when color and shape descriptions are NOT activated simultaneously
        *
        * \param[in] indices the neighborhood point indices
        * \param[in] sqr_dists the neighborhood point distances
        * \param[in] index the index of the point in indices_
        * \param[out] binDistance the resultant distance shape histogram
        * \param[in] nr_bins the number of bins in the shape histogram
        * \param[out] shot the resultant SHOT histogram
        */
      void
      interpolateSingleChannel (const std::vector<int> &indices,
                                const std::vector<float> &sqr_dists,
                                const int index,
                                std::vector<double> &binDistance,
                                const int nr_bins,
                                Eigen::VectorXf &shot);

      /** \brief Normalize the SHOT histogram.
        * \param[in,out] shot the SHOT histogram
        * \param[in] desc_length the length of the histogram
        */
      void
      normalizeHistogram (Eigen::VectorXf &shot, int desc_length);


      /** \brief Create a binned distance shape histogram
        * \param[in] index the index of the point in indices_
        * \param[in] indices the k-neighborhood point indices in surface_
        * \param[in] sqr_dists the k-neighborhood point distances in surface_
        * \param[out] bin_distance_shape the resultant histogram
        */
      void
      createBinDistanceShape (int index, const std::vector<int> &indices,
                              std::vector<double> &bin_distance_shape);

      /** \brief The number of bins in each shape histogram. */
      int nr_shape_bins_;

      /** \brief Placeholder for a point's SHOT. */
      Eigen::VectorXf shot_;

      /** \brief The squared search radius. */
      double sqradius_;

      /** \brief 3/4 of the search radius. */
      double radius3_4_;

      /** \brief 1/4 of the search radius. */
      double radius1_4_;

      /** \brief 1/2 of the search radius. */
      double radius1_2_;

      /** \brief Number of azimuthal sectors. */
      const int nr_grid_sector_;

      /** \brief ... */
      const int maxAngularSectors_;

      /** \brief One SHOT length. */
      int descLength_;

      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &) {}
  };

  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
    * a given point cloud dataset containing points and normals.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  //template <typename PointInT, typename PointNT, typename PointRFT>
  //PCL_DEPRECATED (class, "SHOTEstimationBase<..., Eigen::MatrixXf, ...> IS DEPRECATED")
    //SHOTEstimationBase<PointInT, PointNT, Eigen::MatrixXf, PointRFT> : public SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>
  //{
    //public:
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::getClassName;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::input_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::indices_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::k_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::search_parameter_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::search_radius_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::surface_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::descLength_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::nr_grid_sector_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::nr_shape_bins_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::sqradius_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::radius3_4_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::radius1_4_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::radius1_2_;
      //using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::shot_;
      //using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::frames_;
//
      ///** \brief Empty constructor.
        //* \param[in] nr_shape_bins the number of bins in the shape histogram
        //*/
      //SHOTEstimationBase (int nr_shape_bins = 10) : SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT> (nr_shape_bins) {};
//
      ///** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        //* <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        //* setSearchMethod ()
        //* \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        //*/
      //void
      //computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output);
//
      ///** \brief Base method for feature estimation for all points given in
        //* <setInputCloud (), setIndices ()> using the surface in setSearchSurface ()
        //* and the spatial locator in setSearchMethod ()
        //* \param[out] output the resultant point cloud model dataset containing the estimated features
        //*/
      //void
      //computeEigen (pcl::PointCloud<Eigen::MatrixXf> &output)
      //{
        //pcl::SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::computeEigen (output);
      //}
//
      ///** \brief Make the compute (&PointCloudOut); inaccessible from outside the class
        //* \param[out] output the output point cloud
        //*/
      //void
      //compute (pcl::PointCloud<pcl::SHOT> &) {}
  //};

  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
    * a given point cloud dataset containing points and normals.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT352, typename PointRFT = pcl::ReferenceFrame>
  class SHOTEstimation : public SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>
  {
    public:
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::feature_name_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::getClassName;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::indices_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::k_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_parameter_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_radius_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::surface_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::input_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::normals_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::descLength_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::nr_grid_sector_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::nr_shape_bins_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::sqradius_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius3_4_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius1_4_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius1_2_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::maxAngularSectors_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::interpolateSingleChannel;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::shot_;
      using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::frames_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

      /** \brief Empty constructor. */
      SHOTEstimation () : SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> (10)
      {
        feature_name_ = "SHOTEstimation";
      };

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] index the index of the point in indices_
        * \param[in] indices the k-neighborhood point indices in surface_
        * \param[in] sqr_dists the k-neighborhood point distances in surface_
        * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
        */
      virtual void
      computePointSHOT (const int index,
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot);
    protected:
      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void
      computeFeature (pcl::PointCloud<PointOutT> &output);
  };

  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
    * a given point cloud dataset containing points and normals.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointRFT>
  class PCL_DEPRECATED_CLASS (SHOTEstimation, "SHOTEstimation<..., pcl::SHOT, ...> IS DEPRECATED, USE SHOTEstimation<..., pcl::SHOT352, ...> INSTEAD")
    <PointInT, PointNT, pcl::SHOT, PointRFT>
    : public SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>
  {
    public:
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::feature_name_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::getClassName;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::indices_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::k_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::search_parameter_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::search_radius_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::surface_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::input_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::normals_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::descLength_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::nr_grid_sector_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::nr_shape_bins_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::sqradius_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::radius3_4_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::radius1_4_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::radius1_2_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::maxAngularSectors_;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::interpolateSingleChannel;
      using SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT>::shot_;
      using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::frames_;

      typedef typename Feature<PointInT, pcl::SHOT>::PointCloudIn PointCloudIn;

      /** \brief Empty constructor.
        * \param[in] nr_shape_bins the number of bins in the shape histogram
        */
      SHOTEstimation (int nr_shape_bins = 10) : SHOTEstimationBase<PointInT, PointNT, pcl::SHOT, PointRFT> (nr_shape_bins)
      {
        feature_name_ = "SHOTEstimation";
      };

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] index the index of the point in indices_
        * \param[in] indices the k-neighborhood point indices in surface_
        * \param[in] sqr_dists the k-neighborhood point distances in surface_
        * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
        */
      virtual void
      computePointSHOT (const int index,
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot);

    protected:
      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void
      computeFeature (pcl::PointCloud<pcl::SHOT> &output);
  };

  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for
    * a given point cloud dataset containing points and normals.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointRFT>
  class SHOTEstimation<PointInT, PointNT, Eigen::MatrixXf, PointRFT> : public SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>
  {
    public:
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::feature_name_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::getClassName;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::indices_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::k_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::search_parameter_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::search_radius_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::surface_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::input_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::normals_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::descLength_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::nr_grid_sector_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::nr_shape_bins_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::sqradius_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::radius3_4_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::radius1_4_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::radius1_2_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::maxAngularSectors_;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::interpolateSingleChannel;
      using SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::shot_;
      using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::frames_;

      /** \brief Empty constructor. */
      SHOTEstimation (int nr_shape_bins = 10) : SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT> ()
      {
        feature_name_ = "SHOTEstimation";
        nr_shape_bins_ = nr_shape_bins;
      };

      /** \brief Base method for feature estimation for all points given in
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface ()
        * and the spatial locator in setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset containing the estimated features
        */
      void
      computeEigen (pcl::PointCloud<Eigen::MatrixXf> &output)
      {
        pcl::SHOTEstimation<PointInT, PointNT, pcl::SHOT352, PointRFT>::computeEigen (output);
      }

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] index the index of the point in indices_
        * \param[in] indices the k-neighborhood point indices in surface_
        * \param[in] sqr_dists the k-neighborhood point distances in surface_
        * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
        */
      //virtual void
      //computePointSHOT (const int index,
                        //const std::vector<int> &indices,
                        //const std::vector<float> &sqr_dists,
                        //Eigen::VectorXf &shot);

    protected:
      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output);

	  
      /** \brief Make the compute (&PointCloudOut); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      compute (pcl::PointCloud<pcl::SHOT352> &) { assert(0); }
  };

  /** \brief SHOTColorEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for a given point cloud dataset
    * containing points, normals and colors.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::SHOT1344, typename PointRFT = pcl::ReferenceFrame>
  class SHOTColorEstimation : public SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>
  {
    public:
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::feature_name_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::getClassName;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::indices_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::k_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_parameter_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::search_radius_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::surface_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::input_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::normals_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::descLength_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::nr_grid_sector_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::nr_shape_bins_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::sqradius_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius3_4_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius1_4_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::radius1_2_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::maxAngularSectors_;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::interpolateSingleChannel;
      using SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT>::shot_;
      using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::frames_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;

      /** \brief Empty constructor.
        * \param[in] describe_shape
        * \param[in] describe_color
        */
      SHOTColorEstimation (bool describe_shape = true,
                           bool describe_color = true)
        : SHOTEstimationBase<PointInT, PointNT, PointOutT, PointRFT> (10),
          b_describe_shape_ (describe_shape),
          b_describe_color_ (describe_color),
          nr_color_bins_ (30)
      {
        feature_name_ = "SHOTColorEstimation";
      };

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] index the index of the point in indices_
        * \param[in] indices the k-neighborhood point indices in surface_
        * \param[in] sqr_dists the k-neighborhood point distances in surface_
        * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
        */
      virtual void
      computePointSHOT (const int index,
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot);
    protected:
      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void
      computeFeature (pcl::PointCloud<PointOutT> &output);

      /** \brief Quadrilinear interpolation; used when color and shape descriptions are both activated
        * \param[in] indices the neighborhood point indices
        * \param[in] sqr_dists the neighborhood point distances
        * \param[in] index the index of the point in indices_
        * \param[out] binDistanceShape the resultant distance shape histogram
        * \param[out] binDistanceColor the resultant color shape histogram
        * \param[in] nr_bins_shape the number of bins in the shape histogram
        * \param[in] nr_bins_color the number of bins in the color histogram
        * \param[out] shot the resultant SHOT histogram
        */
      void
      interpolateDoubleChannel (const std::vector<int> &indices,
                                const std::vector<float> &sqr_dists,
                                const int index,
                                std::vector<double> &binDistanceShape,
                                std::vector<double> &binDistanceColor,
                                const int nr_bins_shape,
                                const int nr_bins_color,
                                Eigen::VectorXf &shot);

      /** \brief Compute shape descriptor. */
      bool b_describe_shape_;

      /** \brief Compute color descriptor. */
      bool b_describe_color_;

      /** \brief The number of bins in each color histogram. */
      int nr_color_bins_;

    public:
      /** \brief Converts RGB triplets to CIELab space.
        * \param[in] R the red channel
        * \param[in] G the green channel
        * \param[in] B the blue channel
        * \param[out] L the lightness
        * \param[out] A the first color-opponent dimension
        * \param[out] B2 the second color-opponent dimension
        */
      static void
      RGB2CIELAB (unsigned char R, unsigned char G, unsigned char B, float &L, float &A, float &B2);

      static float sRGB_LUT[256];
      static float sXYZ_LUT[4000];
  };

  template <typename PointInT, typename PointNT, typename PointRFT>
  class SHOTColorEstimation<PointInT, PointNT, Eigen::MatrixXf, PointRFT> : public SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>
  {
    public:
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::feature_name_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::getClassName;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::indices_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::k_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::search_parameter_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::search_radius_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::surface_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::input_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::normals_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::descLength_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::nr_grid_sector_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::nr_shape_bins_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::sqradius_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::radius3_4_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::radius1_4_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::radius1_2_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::maxAngularSectors_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::interpolateSingleChannel;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::shot_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::b_describe_shape_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::b_describe_color_;
      using SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::nr_color_bins_;
      using FeatureWithLocalReferenceFrames<PointInT, PointRFT>::frames_;

      /** \brief Empty constructor.
        * \param[in] describe_shape
        * \param[in] describe_color
        */
      SHOTColorEstimation (bool describe_shape = true,
                           bool describe_color = true,
                           int nr_shape_bins = 10,
                           int nr_color_bins = 30)
        : SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT> (describe_shape, describe_color)
      {
        feature_name_ = "SHOTColorEstimation";
        nr_shape_bins_ = nr_shape_bins;
        nr_color_bins_ = nr_color_bins;
      };

      /** \brief Base method for feature estimation for all points given in
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface ()
        * and the spatial locator in setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset containing the estimated features
        */
      void
      computeEigen (pcl::PointCloud<Eigen::MatrixXf> &output)
      {
        pcl::SHOTColorEstimation<PointInT, PointNT, pcl::SHOT1344, PointRFT>::computeEigen (output);
      }

    protected:
      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output);

	  
	  /** \brief Make the compute (&PointCloudOut); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      compute (pcl::PointCloud<pcl::SHOT1344> &) { assert(0); }
  };

  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for a given point cloud dataset
    * containing points and normals.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointNT, typename PointRFT>
  class PCL_DEPRECATED_CLASS (SHOTEstimation, "SHOTEstimation<pcl::PointXYZRGBA,...,pcl::SHOT,...> IS DEPRECATED, USE SHOTEstimation<pcl::PointXYZRGBA,...,pcl::SHOT352,...> FOR SHAPE AND SHOTColorEstimation<pcl::PointXYZRGBA,...,pcl::SHOT1344,...> FOR SHAPE+COLOR INSTEAD")
    <pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>
    : public SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT> 
  {
    public:
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::feature_name_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::indices_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::k_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::search_parameter_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::search_radius_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::surface_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::input_;
      using FeatureFromNormals<pcl::PointXYZRGBA, PointNT, pcl::SHOT>::normals_;
      using FeatureWithLocalReferenceFrames<pcl::PointXYZRGBA, PointRFT>::frames_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::getClassName;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::descLength_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::nr_grid_sector_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::nr_shape_bins_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::sqradius_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::radius3_4_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::radius1_4_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::radius1_2_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::maxAngularSectors_;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::interpolateSingleChannel;
      using SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::shot_;

      typedef typename Feature<pcl::PointXYZRGBA, pcl::SHOT>::PointCloudOut PointCloudOut;
      typedef typename Feature<pcl::PointXYZRGBA, pcl::SHOT>::PointCloudIn PointCloudIn;

      /** \brief Empty constructor.
        * \param[in] describe_shape
        * \param[in] describe_color
        * \param[in] nr_shape_bins
        * \param[in] nr_color_bins
        */
      SHOTEstimation (bool describe_shape = true,
                      bool describe_color = false,
                      const int nr_shape_bins = 10,
                      const int nr_color_bins = 30)
        : SHOTEstimationBase<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT> (nr_shape_bins),
          b_describe_shape_ (describe_shape),
          b_describe_color_ (describe_color),
          nr_color_bins_ (nr_color_bins)
      {
        feature_name_ = "SHOTEstimation";
      };

      /** \brief Estimate the SHOT descriptor for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] index the index of the point in indices_
        * \param[in] indices the k-neighborhood point indices in surface_
        * \param[in] sqr_dists the k-neighborhood point distances in surface_
        * \param[out] shot the resultant SHOT descriptor representing the feature at the query point
        */
      virtual void
      computePointSHOT (const int index,
                        const std::vector<int> &indices,
                        const std::vector<float> &sqr_dists,
                        Eigen::VectorXf &shot);

    protected:

      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void
      computeFeature (PointCloudOut &output);

      /** \brief Quadrilinear interpolation; used when color and shape descriptions are both activated
        * \param[in] indices the neighborhood point indices
        * \param[in] sqr_dists the neighborhood point distances
        * \param[in] index the index of the point in indices_
        * \param[out] binDistanceShape the resultant distance shape histogram
        * \param[out] binDistanceColor the resultant color shape histogram
        * \param[in] nr_bins_shape the number of bins in the shape histogram
        * \param[in] nr_bins_color the number of bins in the color histogram
        * \param[out] shot the resultant SHOT histogram
        */
      void
      interpolateDoubleChannel (const std::vector<int> &indices,
                                const std::vector<float> &sqr_dists,
                                const int index,
                                std::vector<double> &binDistanceShape,
                                std::vector<double> &binDistanceColor,
                                const int nr_bins_shape,
                                const int nr_bins_color,
                                Eigen::VectorXf &shot);

      /** \brief Converts RGB triplets to CIELab space.
        * \param[in] R the red channel
        * \param[in] G the green channel
        * \param[in] B the blue channel
        * \param[out] L the lightness
        * \param[out] A the first color-opponent dimension
        * \param[out] B2 the second color-opponent dimension
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

  /** \brief SHOTEstimation estimates the Signature of Histograms of OrienTations (SHOT) descriptor for a given point cloud dataset
    * containing points and normals.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     Unique Signatures of Histograms for Local Surface Description.
    *     In Proceedings of the 11th European Conference on Computer Vision (ECCV),
    *     Heraklion, Greece, September 5-11 2010.
    *   - F. Tombari, S. Salti, L. Di Stefano
    *     A Combined Texture-Shape Descriptor For Enhanced 3D Feature Matching.
    *     In Proceedings of the 18th International Conference on Image Processing (ICIP),
    *     Brussels, Belgium, September 11-14 2011.
    *
    * \author Samuele Salti, Federico Tombari
    * \ingroup features
    */
  template <typename PointNT, typename PointRFT>
  class PCL_DEPRECATED_CLASS (SHOTEstimation, "SHOTEstimation<pcl::PointXYZRGBA,...,Eigen::MatrixXf,...> IS DEPRECATED, USE SHOTColorEstimation<pcl::PointXYZRGBA,...,Eigen::MatrixXf,...> FOR SHAPE AND SHAPE+COLOR INSTEAD")
    <pcl::PointXYZRGBA, PointNT, Eigen::MatrixXf, PointRFT>
    : public SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>
  {
    public:
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::feature_name_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::getClassName;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::indices_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::k_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::search_parameter_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::search_radius_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::surface_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::input_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::descLength_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::nr_grid_sector_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::nr_shape_bins_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::sqradius_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::radius3_4_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::radius1_4_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::radius1_2_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::maxAngularSectors_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::interpolateSingleChannel;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::shot_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::b_describe_shape_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::b_describe_color_;
      using SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT>::nr_color_bins_;
      using FeatureWithLocalReferenceFrames<pcl::PointXYZRGBA, PointRFT>::frames_;

      /** \brief Empty constructor.
        * \param[in] describe_shape
        * \param[in] describe_color
        * \param[in] nr_shape_bins
        * \param[in] nr_color_bins
        */
      SHOTEstimation (bool describe_shape = true,
                      bool describe_color = false,
                      const int nr_shape_bins = 10,
                      const int nr_color_bins = 30)
        : SHOTEstimation<pcl::PointXYZRGBA, PointNT, pcl::SHOT, PointRFT> (describe_shape, describe_color, nr_shape_bins, nr_color_bins) {};

   protected:
      /** \brief Estimate the Signatures of Histograms of OrienTations (SHOT) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains the SHOT feature estimates
        */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output);

	  
	  /** \brief Make the compute (&PointCloudOut); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      compute (pcl::PointCloud<pcl::SHOT> &) { assert(0); }
  };
}

#endif  //#ifndef PCL_SHOT_H_



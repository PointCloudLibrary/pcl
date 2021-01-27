/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2016-, Open Perception, Inc.
 *  Copyright (c) 2016, Voxar Labs, CIn-UFPE / DEINFO-UFRPE
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

#include <pcl/features/feature.h>

namespace pcl
{
  /// Different histogram interpolation methods
  enum HistogramInterpolationMethod
  {
    INTERP_NONE,         ///< no interpolation
    INTERP_TRILINEAR,    ///< trilinear interpolation
    INTERP_QUADRILINEAR  ///< quadrilinear interpolation
  };

  /** \brief GASDEstimation estimates the Globally Aligned Spatial Distribution (GASD) descriptor for a given
   * point cloud dataset given XYZ data.
   *
   * The suggested PointOutT is pcl::GASDSignature512.
   *
   * \note If you use this code in any academic work, please cite:
   *
   *   - J. Lima, V. Teichrieb.
   *     An Efficient Global Point Cloud Descriptor for Object Recognition and Pose Estimation.
   *     In Proceedings of the 29th SIBGRAPI - Conference on Graphics, Patterns and Images,
   *     Sao Jose dos Campos, Brazil, October 4-7 2016.
   *
   * \author Joao Paulo Lima
   *
   * Voxar Labs, Centro de Informatica, Universidade Federal de Pernambuco, Brazil
   *
   * Departamento de Estatistica e Informatica, Universidade Federal Rural de Pernambuco, Brazil
   *
   * \ingroup features
   */
  template <typename PointInT, typename PointOutT = GASDSignature512>
  class GASDEstimation : public Feature<PointInT, PointOutT>
  {
    public:
      using typename Feature<PointInT, PointOutT>::PointCloudIn;
      using typename Feature<PointInT, PointOutT>::PointCloudOut;
      using Ptr = shared_ptr<GASDEstimation<PointInT, PointOutT> >;
      using ConstPtr = shared_ptr<const GASDEstimation<PointInT, PointOutT> >;

      /** \brief Constructor.
       * \param[in] view_direction view direction
       * \param[in] shape_half_grid_size shape half grid size
       * \param[in] shape_hists_size shape histograms size
       * \param[in] shape_interp shape histograms interpolation method
       */
      GASDEstimation (const Eigen::Vector3f &view_direction = Eigen::Vector3f (0.0f, 0.0f, 1.0f),
                      const std::size_t shape_half_grid_size = 4,
                      const std::size_t shape_hists_size = 1,
                      const HistogramInterpolationMethod shape_interp = INTERP_TRILINEAR) :
          view_direction_ (view_direction),
          shape_half_grid_size_ (shape_half_grid_size),
          shape_hists_size_ (shape_hists_size),
          shape_interp_ (shape_interp)
      {
        search_radius_ = 0;
        k_ = 1;
        feature_name_ = "GASDEstimation";
      }

      /** \brief Set the view direction.
       * \param[in] dir view direction
       */
      inline void
      setViewDirection (const Eigen::Vector3f &dir)
      {
        view_direction_ = dir;
      }

      /** \brief Set the shape half grid size.
       * \param[in] shgs shape half grid size
       */
      inline void
      setShapeHalfGridSize (const std::size_t shgs)
      {
        shape_half_grid_size_ = shgs;
      }

      /** \brief Set the shape histograms size. If size is 1, then each histogram bin will store the number
       * of points that belong to its correspondent cell in the 3D regular grid. If size > 1, then for each cell
       * it will be computed a histogram of normalized distances between each sample and the cloud centroid
       * \param[in] shs shape histograms size
       */
      inline void
      setShapeHistsSize (const std::size_t shs)
      {
        shape_hists_size_ = shs;
      }

      /** \brief Set the shape histograms interpolation method.
       * \param[in] interp shape histograms interpolation method
       */
      inline void
      setShapeHistsInterpMethod (const HistogramInterpolationMethod interp)
      {
        shape_interp_ = interp;
      }

      /**
       * \brief Returns the transformation aligning the point cloud to the canonical coordinate system
       */
      const Eigen::Matrix4f&
      getTransform () const
      {
        return transform_;
      }

      /** \brief Overloaded computed method from pcl::Feature.
       * \param[out] output the resultant point cloud model dataset containing the estimated feature
       */
      void
      compute (PointCloudOut &output);

    protected:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;

      /** \brief Point cloud aligned to the canonical coordinate system. */
      PointCloudIn shape_samples_;

      /** \brief Normalization factor with respect to axis-aligned bounding cube centered on the origin. */
      float max_coord_;

      /** \brief Normalized sample contribution with respect to the total number of points in the cloud. */
      float hist_incr_;

      /** \brief Current position of output descriptor point cloud. */
      std::size_t pos_;

      /** \brief add a sample to its respective histogram, optionally performing interpolation.
       * \param[in] p histogram sample
       * \param[in] max_coord normalization factor with respect to axis-aligned bounding cube centered on the origin
       * \param[in] half_grid_size half size of the regular grid used to compute the descriptor
       * \param[in] interp interpolation method to be used while computing the descriptor
       * \param[in] hbin histogram bin
       * \param[in] hist_incr normalization factor of sample contribution
       * \param[in,out] hists updated histograms
       */
      void
      addSampleToHistograms (const Eigen::Vector4f &p,
                             const float max_coord,
                             const std::size_t half_grid_size,
                             const HistogramInterpolationMethod interp,
                             const float hbin,
                             const float hist_incr,
                             std::vector<Eigen::VectorXf> &hists);

      /** \brief Estimate GASD descriptor
       *
       * \param[out] output the resultant point cloud model dataset containing the GASD feature
       */
      void
      computeFeature (PointCloudOut &output) override;

    private:
      /** \brief Transform that aligns the point cloud to the canonical coordinate system. */
      Eigen::Matrix4f transform_;

      /** \brief Viewing direction, default value is (0, 0, 1). */
      Eigen::Vector3f view_direction_;

      /** \brief Half size of the regular grid used to compute the shape descriptor. */
      std::size_t shape_half_grid_size_;

      /** \brief Size of the histograms of normalized distances between each sample and the cloud centroid. */
      std::size_t shape_hists_size_;

      /** \brief Interpolation method to be used while computing the shape descriptor. */
      HistogramInterpolationMethod shape_interp_;

      /** \brief Estimates a reference frame for the point cloud and uses it to compute a transform that aligns the point cloud to the canonical coordinate system. */
      void
      computeAlignmentTransform ();

      /** \brief copy computed shape histograms to output descriptor point cloud
       * \param[in] grid_size size of the regular grid used to compute the descriptor
       * \param[in] hists_size size of the shape histograms
       * \param[in] hists shape histograms
       * \param[out] output output descriptor point cloud
       * \param[in,out] pos current position of output descriptor point cloud
       */
      void
      copyShapeHistogramsToOutput (const std::size_t grid_size,
                                   const std::size_t hists_size,
                                   const std::vector<Eigen::VectorXf> &hists,
                                   PointCloudOut &output,
                                   std::size_t &pos);
  };

  /** \brief GASDColorEstimation estimates the Globally Aligned Spatial Distribution (GASD) descriptor for a given
   * point cloud dataset given XYZ and RGB data.
   *
   * The suggested PointOutT is pcl::GASDSignature984.
   *
   * \note If you use this code in any academic work, please cite:
   *
   *   - J. Lima, V. Teichrieb.
   *     An Efficient Global Point Cloud Descriptor for Object Recognition and Pose Estimation.
   *     In Proceedings of the 29th SIBGRAPI - Conference on Graphics, Patterns and Images,
   *     Sao Jose dos Campos, Brazil, October 4-7 2016.
   *
   * \author Joao Paulo Lima
   *
   * Voxar Labs, Centro de Informatica, Universidade Federal de Pernambuco, Brazil
   *
   * Departamento de Estatistica e Informatica, Universidade Federal Rural de Pernambuco, Brazil
   *
   * \ingroup features
   */
  template <typename PointInT, typename PointOutT = GASDSignature984>
  class GASDColorEstimation : public GASDEstimation<PointInT, PointOutT>
  {
    public:
      using typename Feature<PointInT, PointOutT>::PointCloudOut;
      using Ptr = shared_ptr<GASDColorEstimation<PointInT, PointOutT> >;
      using ConstPtr = shared_ptr<const GASDColorEstimation<PointInT, PointOutT> >;

      /** \brief Constructor.
       * \param[in] view_direction view direction
       * \param[in] shape_half_grid_size shape half grid size
       * \param[in] shape_hists_size shape histograms size
       * \param[in] color_half_grid_size color half grid size
       * \param[in] color_hists_size color histograms size
       * \param[in] shape_interp shape histograms interpolation method
       * \param[in] color_interp color histograms interpolation method
       */
      GASDColorEstimation (const Eigen::Vector3f &view_direction = Eigen::Vector3f (0.0f, 0.0f, 1.0f),
                           const std::size_t shape_half_grid_size = 3,
                           const std::size_t shape_hists_size = 1,
                           const std::size_t color_half_grid_size = 2,
                           const std::size_t color_hists_size = 12,
                           const HistogramInterpolationMethod shape_interp = INTERP_NONE,
                           const HistogramInterpolationMethod color_interp = INTERP_NONE) :
          GASDEstimation<PointInT, PointOutT> (view_direction, shape_half_grid_size, shape_hists_size, shape_interp),
          color_half_grid_size_ (color_half_grid_size),
          color_hists_size_ (color_hists_size),
          color_interp_ (color_interp)
      {
        feature_name_ = "GASDColorEstimation";
      }

      /** \brief Set the color half grid size.
       * \param[in] chgs color half grid size
       */
      inline void
      setColorHalfGridSize (const std::size_t chgs)
      {
        color_half_grid_size_ = chgs;
      }

      /** \brief Set the color histograms size (number of bins in the hue histogram for each cell of the 3D regular grid).
       * \param[in] chs color histograms size
       */
      inline void
      setColorHistsSize (const std::size_t chs)
      {
        color_hists_size_ = chs;
      }

      /** \brief Set the color histograms interpolation method.
       * \param[in] interp color histograms interpolation method
       */
      inline void
      setColorHistsInterpMethod (const HistogramInterpolationMethod interp)
      {
        color_interp_ = interp;
      }

    protected:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using GASDEstimation<PointInT, PointOutT>::shape_samples_;
      using GASDEstimation<PointInT, PointOutT>::max_coord_;
      using GASDEstimation<PointInT, PointOutT>::hist_incr_;
      using GASDEstimation<PointInT, PointOutT>::pos_;

    private:
      /** \brief Half size of the regular grid used to compute the color descriptor. */
      std::size_t color_half_grid_size_;

      /** \brief Size of the hue histograms. */
      std::size_t color_hists_size_;

      /** \brief Interpolation method to be used while computing the color descriptor. */
      HistogramInterpolationMethod color_interp_;

      /** \brief copy computed color histograms to output descriptor point cloud
       * \param[in] grid_size size of the regular grid used to compute the descriptor
       * \param[in] hists_size size of the color histograms
       * \param[in,out] hists color histograms, which are finalized, since they are circular
       * \param[out] output output descriptor point cloud
       * \param[in,out] pos current position of output descriptor point cloud
       */
      void
      copyColorHistogramsToOutput (const std::size_t grid_size,
                                   const std::size_t hists_size,
                                   std::vector<Eigen::VectorXf> &hists,
                                   PointCloudOut &output,
                                   std::size_t &pos);

      /** \brief Estimate GASD color descriptor
       *
       * \param[out] output the resultant point cloud model dataset containing the GASD color feature
       */
      void
      computeFeature (PointCloudOut &output) override;
  };
}  // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/gasd.hpp>
#endif

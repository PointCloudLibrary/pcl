/*
 * crh_recognition.h
 *
 *  Created on: Mar 30, 2012
 *      Author: aitor
 */

#ifndef CRH_ALIGNMENT_H_
#define CRH_ALIGNMENT_H_

#include <pcl/common/common.h>
#include <pcl/features/crh.h>
#include <pcl/common/fft/kiss_fftr.h>
#include <pcl/common/transforms.h>

namespace pcl
{

  /** \brief CRHAlignment uses two Camera Roll Histograms (CRH) to find the
   * roll rotation that aligns both views. See:
   *   - CAD-Model Recognition and 6 DOF Pose Estimation
   *     A. Aldoma, N. Blodow, D. Gossow, S. Gedikli, R.B. Rusu, M. Vincze and G. Bradski
   *     ICCV 2011, 3D Representation and Recognition (3dRR11) workshop
   *     Barcelona, Spain, (2011)
   *
   * \author Aitor Aldoma
   * \ingroup recognition
   */

  template<typename PointT, int nbins_>
    class PCL_EXPORTS CRHAlignment
    {
    private:

      /** \brief Sorts peaks */
      typedef struct
      {
        bool
        operator() (std::pair<float, int> const& a, std::pair<float, int> const& b)
        {
          return a.first > b.first;
        }
      } peaks_ordering;

      typedef typename pcl::PointCloud<PointT>::Ptr PointTPtr;

      /** \brief View of the model to be aligned to input_view_ */
      PointTPtr target_view_;
      /** \brief View of the input */
      PointTPtr input_view_;
      /** \brief Centroid of the model_view_ */
      Eigen::Vector3f centroid_target_;
      /** \brief Centroid of the input_view_ */
      Eigen::Vector3f centroid_input_;
      /** \brief transforms from model view to input view */
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_;
      /** \brief Allowed maximum number of peaks  */
      int max_peaks_;
      /** \brief Quantile of peaks after sorting to be checked  */
      float quantile_;
      /** \brief Threshold for a peak to be accepted.
       * If peak_i >= (max_peak * accept_threhsold_) => peak is accepted
       */
      float accept_threshold_;

      /** \brief computes the transformation to the z-axis
        * \param[in] centroid
        * \param[out] trasnformation to z-axis
        */
      void
      computeTransformToZAxes (Eigen::Vector3f & centroid, Eigen::Affine3f & transform)
      {
        Eigen::Vector3f plane_normal;
        plane_normal[0] = -centroid[0];
        plane_normal[1] = -centroid[1];
        plane_normal[2] = -centroid[2];
        Eigen::Vector3f z_vector = Eigen::Vector3f::UnitZ ();
        plane_normal.normalize ();
        Eigen::Vector3f axis = plane_normal.cross (z_vector);
        double rotation = -asin (axis.norm ());
        axis.normalize ();
        transform = Eigen::Affine3f (Eigen::AngleAxisf (static_cast<float>(rotation), axis));
      }

      /** \brief computes the roll transformation
        * \param[in] centroid input
        * \param[in] centroid view
        * \param[in] roll_angle
        * \param[out] roll transformation
        */
      void
      computeRollTransform (Eigen::Vector3f & centroidInput, Eigen::Vector3f & centroidResult, double roll_angle, Eigen::Affine3f & final_trans)
      {
        Eigen::Affine3f transformInputToZ;
        computeTransformToZAxes (centroidInput, transformInputToZ);

        transformInputToZ = transformInputToZ.inverse ();
        Eigen::Affine3f transformRoll (Eigen::AngleAxisf (-static_cast<float>(roll_angle * M_PI / 180), Eigen::Vector3f::UnitZ ()));
        Eigen::Affine3f transformDBResultToZ;
        computeTransformToZAxes (centroidResult, transformDBResultToZ);

        final_trans = transformInputToZ * transformRoll * transformDBResultToZ;
      }
    public:

      /** \brief Constructor. */
      CRHAlignment() {
        max_peaks_ = 5;
        quantile_ = 0.2f;
        accept_threshold_ = 0.8f;
      }

      /** \brief returns the computed transformations
       * \param[out] transforms transformations
       */
      void getTransforms(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & transforms) {
        transforms = transforms_;
      }

      /** \brief sets model and input views
       * \param[in] input_view
       * \param[in] target_view
       */
      void
      setInputAndTargetView (PointTPtr & input_view, PointTPtr & target_view)
      {
        target_view_ = target_view;
        input_view_ = input_view;
      }

      /** \brief sets model and input centroids
        * \param[in] c1 model view centroid
        * \param[in] c2 input view centroid
        */
      void
      setInputAndTargetCentroids (Eigen::Vector3f & c1, Eigen::Vector3f & c2)
      {
        centroid_target_ = c2;
        centroid_input_ = c1;
      }

      /** \brief Computes the transformation aligning model to input
       * \param[in] input_ftt CRH histogram of the input cloud
       * \param[in] target_ftt CRH histogram of the target cloud
       */
      void
      align (pcl::PointCloud<pcl::Histogram<nbins_> > & input_ftt, pcl::PointCloud<pcl::Histogram<nbins_> > & target_ftt)
      {

        transforms_.clear(); //clear from last round...

        std::vector<float> peaks;
        computeRollAngle (input_ftt, target_ftt, peaks);

        //if the number of peaks is too big, we should try to reduce using siluette matching

        for (size_t i = 0; i < peaks.size(); i++)
        {
          Eigen::Affine3f rollToRot;
          computeRollTransform (centroid_input_, centroid_target_, peaks[i], rollToRot);

          Eigen::Matrix4f rollHomMatrix = Eigen::Matrix4f ();
          rollHomMatrix.setIdentity (4, 4);
          rollHomMatrix = rollToRot.matrix ();

          Eigen::Matrix4f translation2;
          translation2.setIdentity (4, 4);
          Eigen::Vector3f centr = rollToRot * centroid_target_;
          translation2 (0, 3) = centroid_input_[0] - centr[0];
          translation2 (1, 3) = centroid_input_[1] - centr[1];
          translation2 (2, 3) = centroid_input_[2] - centr[2];

          Eigen::Matrix4f resultHom (translation2 * rollHomMatrix);
          transforms_.push_back(resultHom.inverse());
        }

      }

      /** \brief Computes the roll angle that aligns input to modle.
       * \param[in] input_ftt CRH histogram of the input cloud
       * \param[in] target_ftt CRH histogram of the target cloud
       * \param[out] peaks Vector containing angles where the histograms correlate
       */
      void
      computeRollAngle (pcl::PointCloud<pcl::Histogram<nbins_> > & input_ftt, pcl::PointCloud<pcl::Histogram<nbins_> > & target_ftt,
                        std::vector<float> & peaks)
      {

        pcl::PointCloud<pcl::Histogram<nbins_> > input_ftt_negate (input_ftt);

        for (int i = 2; i < (nbins_); i += 2)
          input_ftt_negate.points[0].histogram[i] = -input_ftt_negate.points[0].histogram[i];

        int nr_bins_after_padding = 180;
        int peak_distance = 5;
        int cutoff = nbins_ - 1;

        kiss_fft_cpx * multAB = new kiss_fft_cpx[nr_bins_after_padding];
        for (int i = 0; i < nr_bins_after_padding; i++)
          multAB[i].r = multAB[i].i = 0.f;

        int k = 0;
        multAB[k].r = input_ftt_negate.points[0].histogram[0] * target_ftt.points[0].histogram[0];
        k++;

        float a, b, c, d;
        for (int i = 1; i < cutoff; i += 2, k++)
        {
          a = input_ftt_negate.points[0].histogram[i];
          b = input_ftt_negate.points[0].histogram[i + 1];
          c = target_ftt.points[0].histogram[i];
          d = target_ftt.points[0].histogram[i + 1];
          multAB[k].r = a * c - b * d;
          multAB[k].i = b * c + a * d;

          float tmp = std::sqrt (multAB[k].r * multAB[k].r + multAB[k].i * multAB[k].i);

          multAB[k].r /= tmp;
          multAB[k].i /= tmp;
        }

        multAB[nbins_ - 1].r = input_ftt_negate.points[0].histogram[nbins_ - 1] * target_ftt.points[0].histogram[nbins_ - 1];

        kiss_fft_cfg mycfg = kiss_fft_alloc (nr_bins_after_padding, 1, NULL, NULL);
        kiss_fft_cpx * invAB = new kiss_fft_cpx[nr_bins_after_padding];
        kiss_fft (mycfg, multAB, invAB);

        std::vector < std::pair<float, int> > scored_peaks (nr_bins_after_padding);
        for (int i = 0; i < nr_bins_after_padding; i++)
          scored_peaks[i] = std::make_pair (invAB[i].r, i);

        std::sort (scored_peaks.begin (), scored_peaks.end (), peaks_ordering ());

        std::vector<int> peaks_indices;
        std::vector<float> peaks_values;

        // we look at the upper quantile_
        float quantile = quantile_;
        int max_inserted= max_peaks_;

        int inserted=0;
        bool stop=false;
        for (int i = 0; (i < static_cast<int> (quantile * static_cast<float> (nr_bins_after_padding))) && !stop; i++)
        {
          if (scored_peaks[i].first >= scored_peaks[0].first * accept_threshold_)
          {
            bool insert = true;

            for (size_t j = 0; j < peaks_indices.size (); j++)
            { //check inserted peaks, first pick always inserted
              if (std::abs (peaks_indices[j] - scored_peaks[i].second) <= peak_distance || std::abs (
                                                                                             peaks_indices[j] - (scored_peaks[i].second
                                                                                                 - nr_bins_after_padding)) <= peak_distance)
              {
                insert = false;
                break;
              }
            }

            if (insert)
            {
              peaks_indices.push_back (scored_peaks[i].second);
              peaks_values.push_back (scored_peaks[i].first);
              peaks.push_back (static_cast<float> (scored_peaks[i].second * (360 / nr_bins_after_padding)));
              inserted++;
              if(inserted >= max_inserted)
                stop = true;
            }
          }
        }
      }
    };
}

#endif /* CRH_ALIGNMENT_H_ */
